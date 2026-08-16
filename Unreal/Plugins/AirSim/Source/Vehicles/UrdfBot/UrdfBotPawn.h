// The Unreal side of a URDF robot: one movable component per URDF link.
//
// ⚠ Components, not actors. PHYSICS_ENGINE_ANALYSIS.md §6.0 speaks of "per-link actors", and one
// actor per link would work, but AirSim's vehicle model is one APawn per vehicle throughout —
// PawnSimApi::Params carries a single APawn*, getExistingVehiclePawns enumerates pawns, and the
// camera director indexes by pawn. Separate actors would need their own spawning, lifetime and
// discovery to sit alongside that. Scene components attached to the pawn are moved by the same
// SetWorldLocationAndRotation call, carry their own collision (which is what LiDAR traces), and
// accept attached cameras and sensors — so they buy the whole capability with none of the
// machinery. The distinction matters nowhere else in the design.
//
// Nothing here simulates. Unreal physics is OFF on every link component: Box3D owns the poses and
// these components are written from it. Leaving Chaos enabled would give the robot two solvers
// fighting over it, which is the failure this architecture exists to avoid.
#pragma once

#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "GameFramework/Pawn.h"
#include "UObject/ConstructorHelpers.h"

#include "AirBlueprintLib.h"
#include "PIPCamera.h"
#include "PawnEvents.h"
#include "common/AirSimSettings.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "urdf/UrdfModel.hpp"

#include <atomic>
#include <map>
#include <string>
#include <vector>

#include "UrdfBotPawn.generated.h"

UCLASS()
class AIRSIM_API AUrdfBotPawn : public APawn
{
    GENERATED_BODY()

public:
    AUrdfBotPawn();

    virtual void BeginPlay() override;
    virtual void Tick(float Delta) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other,
                           class UPrimitiveComponent* OtherComp, bool bSelfMoved,
                           FVector HitLocation, FVector HitNormal, FVector NormalImpulse,
                           const FHitResult& Hit) override;

    /// Create one component per URDF link. Called by the sim api once the model is parsed, because
    /// the pawn is spawned before settings are read and cannot parse the file itself.
    void buildFromModel(const urdf::Robot& model, const std::string& urdf_dir,
                        const std::vector<std::string>& mesh_search_paths);

    void initializeForBeginPlay();

    /// Link name -> component. This is what per-link sensor and camera mounting resolves against.
    const std::map<std::string, USceneComponent*>& getLinkComponents() const
    {
        return link_components_;
    }
    USceneComponent* getLinkComponent(const std::string& link_name) const;

    /// Link components in URDF link-index order, so a pose array can be written without a lookup
    /// per link per frame.
    const TArray<USceneComponent*>& getLinkComponentsByIndex() const { return links_by_index_; }

    /// An actor rigidly attached to `link_name`, created on first use. This is what a per-link
    /// sensor is mounted on — see UrdfLinkMount.h for why it is an actor and not a component.
    /// Returns null if the link does not exist.
    AActor* getOrCreateLinkMount(const std::string& link_name);

    const common_utils::UniqueValueMap<std::string, APIPCamera*> getCameras() const;
    PawnEvents* getPawnEvents() { return &pawn_events_; }

    /// Keyboard/gamepad axes, in [-1, 1].
    ///
    /// ⚠ Written on the GAME thread by the input handlers, read on the PHYSICS thread by
    /// UrdfBotSimApi::update(). Atomics rather than plain floats: the Car and Skid pawns share this
    /// state unsynchronised and get away with it, but this crosses the same physics/game boundary
    /// everything else in this vehicle is careful about, and relaxed atomics cost nothing.
    struct DriveInput
    {
        std::atomic<float> throttle{ 0.0f };  // +1 forward
        std::atomic<float> steering{ 0.0f };  // +1 right
    };
    const DriveInput& getDriveInput() const { return drive_input_; }

private:
    /// Resolve a URDF <visual> mesh to a UStaticMesh, or return null.
    ///
    /// ⚠ Returns null for every <mesh>, which is more often than one might expect. URDF meshes are
    /// STL/DAE files on disk; Unreal renders UStaticMesh assets from its content tree, and there is
    /// no runtime importer. createLinkComponent therefore falls back to the link's <collision>
    /// primitive, which is what Box3D actually simulates. A link with neither a drawable visual nor
    /// a primitive collision is drawn as nothing — that is the R2 gap the audit reports, made
    /// visible.
    UStaticMesh* resolveVisualMesh(const urdf::Geometry& geometry) const;

    USceneComponent* createLinkComponent(const urdf::Link& link, int index);

    /// Attach one drawable primitive to a link, applying its <origin> and dimensions.
    void attachGeometry(USceneComponent* link_component, const urdf::Geometry& geometry,
                        const urdf::Origin& origin, const FName& name, bool collidable);

    /// Key bindings must live on the pawn: Unreal will not bind keys to a non-UObject.
    void setupInputBindings();
    void onMoveForward(float value);
    void onMoveRight(float value);
    void onStop();

    DriveInput drive_input_;

    PawnEvents pawn_events_;

    UPROPERTY()
    UClass* pip_camera_class_;

    UPROPERTY()
    USceneComponent* root_component_;

    /// Held against GC. The std::map below aliases these and is the lookup path.
    UPROPERTY()
    TArray<USceneComponent*> links_by_index_;

    UPROPERTY()
    UStaticMesh* box_mesh_;
    UPROPERTY()
    UStaticMesh* cylinder_mesh_;
    UPROPERTY()
    UStaticMesh* sphere_mesh_;

    UPROPERTY()
    TArray<AActor*> link_mounts_;

    std::map<std::string, USceneComponent*> link_components_;
    std::map<std::string, AActor*> link_mount_by_name_;
};
