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
#include "urdf/UrdfMesh.hpp"
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
    void setVisualCollision(bool enabled) { visual_collision_ = enabled; }

    /// Segmentation stencil ID for every mesh this robot draws.
    ///
    /// ⚠ Without this a URDF robot is INVISIBLE TO SEGMENTATION while appearing normally in Scene
    /// and Depth. AirSim assigns stencil IDs in a startup pass (UAirBlueprintLib::
    /// InitializeMeshStencilIDs); link components are created later, from the URDF, so they miss it
    /// and render as background. One ID per robot rather than per link, because what another robot
    /// needs to answer is "which vehicle is that", not "which bracket is that".
    void setSegmentationId(int id) { segmentation_id_ = id; }

    /// Shading switches, settable from settings.json so a shading problem can be bisected without a
    /// rebuild between attempts. See AirSimSettings for what each one tests.
    void setMeshShading(bool cast_shadow, bool smooth_normals, bool flip_winding,
                        bool base_material, bool inset_shadow, bool two_sided_shadow,
                        bool contact_shadow, double decimate_grid,
                        const std::string& asset_dir, double asset_scale,
                        bool runtime_static, bool tint)
    {
        mesh_runtime_static_ = runtime_static;
        mesh_tint_ = tint;
        mesh_asset_dir_ = asset_dir;
        mesh_asset_scale_ = asset_scale;
        mesh_decimate_grid_ = decimate_grid;
        mesh_contact_shadow_ = contact_shadow;
        mesh_inset_shadow_ = inset_shadow;
        mesh_two_sided_shadow_ = two_sided_shadow;
        mesh_base_material_ = base_material;
        mesh_cast_shadow_ = cast_shadow;
        mesh_smooth_normals_ = smooth_normals;
        mesh_flip_winding_ = flip_winding;
    }

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
    /// Build a UProceduralMeshComponent from a URDF <mesh> file, or return false if it cannot be
    /// loaded. This is what makes a real robot visible: on ExoMy **all 23 visuals are meshes**, so
    /// without it the robot is drawn as whatever <collision> primitives happen to exist — six wheel
    /// cylinders — and is equally invisible to its own LiDAR.
    /// Draw a <mesh> visual from a pre-imported UStaticMesh asset, if one exists for it.
    /// Returns false when no asset is found, leaving the caller to use the runtime STL path.
    bool attachStaticMeshAsset(USceneComponent* link_component, const urdf::Geometry& g,
                               const urdf::Origin& origin, const FName& name, bool collidable);

    bool attachMeshGeometry(USceneComponent* link_component, const urdf::Geometry& g,
                            const urdf::Origin& origin, const urdf::Material& material,
                            const FName& name, bool collidable);

    /// Base material for procedural link meshes. A UProceduralMeshComponent has none of its own —
    /// unlike the engine BasicShapes the primitive path uses — so without this a mesh robot renders
    /// black.
    UPROPERTY()
    UMaterialInterface* mesh_material_ = nullptr;

    /// Loaded STL files, keyed by resolved absolute path.
    ///
    /// ⚠ Not an optimisation: ExoMy names six identical wheel STLs and several repeated brackets,
    /// so an uncached build re-parses the same files many times during a single spawn.
    std::map<std::string, urdf::MeshData> mesh_cache_;

    /// Where the spawn's time actually went. Reported once per build, because "loading is slow" is
    /// not actionable and "collision cooking took 4.1 s of 4.4 s" is.
    /// Cook collision for procedural visual meshes.
    ///
    /// ⚠ True by default and it should usually stay true: this collision is what LiDAR, echo and
    /// distance sensors trace, so false makes the robot invisible to its own sensors and to every
    /// other robot's. It exists because runtime tri-mesh cooking dominates spawn time on a
    /// mesh-heavy robot, and while iterating on something else that trade is sometimes worth it.
    bool visual_collision_ = true;
    int segmentation_id_ = -1;
    bool mesh_cast_shadow_ = true;
    bool mesh_smooth_normals_ = false;
    bool mesh_flip_winding_ = false;
    bool mesh_base_material_ = false;
    bool mesh_inset_shadow_ = true;
    bool mesh_contact_shadow_ = true;

    /// Vertex-cluster grid in metres; 0 disables. See attachMeshGeometry for why this exists.
    double mesh_decimate_grid_ = 0.0;
    std::string mesh_asset_dir_;
    double mesh_asset_scale_ = 1.0;
    int32 mesh_from_asset_ = 0;
    int32 mesh_from_stl_ = 0;
    int64 mesh_triangles_before_ = 0;
    int64 mesh_triangles_after_ = 0;
    bool mesh_two_sided_shadow_ = false;

    /// Force VSM to invalidate this robot's cached shadow pages every frame.
    /// Default TRUE: the links are runtime-built meshes moved every frame, which is
    /// what EShadowCacheInvalidationBehavior::Always exists for.
    bool mesh_shadow_invalidate_always_ = true;

    /// Runtime-built UStaticMesh per (file, scale). A 23-link robot reuses meshes.
    TMap<FString, UStaticMesh*> static_mesh_cache_;
    int32 mesh_built_static_ = 0;

    /// Resolve the shared base material. Must run before ANY attach path uses it.
    void ensureMeshMaterial();

    bool attachBuiltStaticMesh(USceneComponent* link_component,
                               const urdf::Geometry& g, const urdf::Origin& origin,
                               const urdf::Material& material, const FName& name,
                               bool collidable);

    /// Build real UStaticMeshes at runtime instead of using procedural components.
    /// Default TRUE: every VSM knob applied to UProceduralMeshComponent was measured
    /// to do nothing, while a UStaticMeshComponent gets engine-built bounds,
    /// tangents and shadow-cache behaviour.
    bool mesh_runtime_static_ = true;

    /// Apply inline URDF <material><color> values through one MID per generated mesh visual.
    /// FALSE assigns the shared base material directly and is retained as a diagnostic escape hatch.
    bool mesh_tint_ = true;

    class UStaticMesh* buildStaticMeshFromData(const urdf::MeshData& mesh,
                                               const FString& key, double sx,
                                               double sy, double sz);

    bool logged_material_once_ = false;
    bool logged_material_params_ = false;
    bool logged_material_readback_ = false;
    bool logged_param_once_ = false;

    double mesh_load_seconds_ = 0.0;
    double mesh_section_seconds_ = 0.0;
    int64 mesh_triangle_total_ = 0;

    /// Where <mesh filename=...> is resolved from. Captured in buildFromModel, which previously
    /// discarded both because nothing could load a mesh.
    std::string urdf_dir_;
    std::vector<std::string> mesh_search_paths_;

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
