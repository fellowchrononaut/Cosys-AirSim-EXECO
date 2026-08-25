// The world-scoped shared solver scenes, and the narrow seam a vehicle uses to join one.
//
// ⚠ A vehicle sim api must not reach the sim mode, the coordinator, or another vehicle. It asks
// this interface for the scene its backend belongs to, registers itself, and is done. Everything
// world-level — how many scenes exist, which one owns the level mirror, when topology is frozen,
// when the manifest is committed — lives on the far side of this seam, because those are exactly
// the decisions that must not depend on which robot happened to initialise first.
#pragma once

#include "CoreMinimal.h"

#include "common/AirSimSettings.hpp"
#include "physics/PhysicsSceneCoordinator.hpp"
#include "urdf/UrdfCollisionDebug.hpp"
#include "urdf/UrdfRobotBackend.hpp"
#include "urdf/UrdfStaticWorld.hpp"

#if WITH_BOX3D_BINDING
#include "urdf/backends/Box3DSceneParticipant.hpp"
#endif

#if WITH_MUJOCO_BINDING
#include "urdf/backends/mujoco/MuJoCoSceneParticipant.hpp"
#endif

#include <memory>
#include <string>
#include <vector>

/// ⚠ A REAL log category, not `UAirBlueprintLib::LogMessageString`. That helper draws on screen
/// only — every UE_LOG inside it is commented out — and it keys messages by prefix, so two
/// messages sharing one prefix overwrite each other. A coordinated run's authority map, fixed dt,
/// and reset epochs must be readable in Blocks.log after the fact, which is the whole point of
/// recording them. Same lesson as LogUrdfBot.
DECLARE_LOG_CATEGORY_EXTERN(LogPhysicsCoordinator, Log, All);

class ICoordinatedPhysicsScene
{
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

    virtual ~ICoordinatedPhysicsScene() = default;

    /// The one authoritative simulation timestep. A backend's internal fixed step must equal it,
    /// so one coordinated tick is exactly one solver step.
    virtual double fixedStepSeconds() const = 0;
    virtual const AirSimSettings::PhysicsCoordinatorSetting& coordinatorSettings() const = 0;

    /// The stable coordinator id a vehicle publishes state under.
    static std::string vehicleBodyId(const std::string& vehicle_name)
    {
        return "vehicle/" + (vehicle_name.empty() ? std::string("<default>") : vehicle_name);
    }

    /// The participant id of the vehicle's own pre/post-solve hooks. Deliberately distinct from the
    /// body id above: the body is OWNED by the scene that solves it, and the vehicle api is a
    /// separate participant that only latches commands and reads results.
    static std::string vehicleParticipantId(const std::string& vehicle_name)
    {
        return "vehicle-api/" + (vehicle_name.empty() ? std::string("<default>") : vehicle_name);
    }

    /// Exactly one participant may mirror the world's movable obstacles.
    ///
    /// ⚠ In a private world each robot registered its own kinematic copy of every moving prop,
    /// which was right precisely because the worlds were private. In one shared world that would
    /// create N overlapping copies of one lift, and a robot would be pushed by the other robots'
    /// copies of it. The first claimant owns them for everybody.
    virtual bool claimKinematicMirror() = 0;

#if WITH_BOX3D_BINDING
    /// The shared Box3D world, created on first use.
    virtual b3urdf::Box3DPhysicsScene& box3dScene() = 0;
    /// Declare that a registered Box3D robot publishes coordinator state as `stable_id`.
    virtual void publishBox3DBody(const std::string& stable_id, size_t scene_handle) = 0;
#endif

#if WITH_MUJOCO_BINDING
    /// The shared MuJoCo scene, created on first use.
    virtual urdf::MuJoCoPhysicsScene& mujocoScene() = 0;
    /// Declare that a registered MuJoCo articulation publishes coordinator state as `stable_id`.
    virtual void publishMuJoCoBody(const std::string& stable_id,
                                   urdf::MuJoCoPhysicsScene::ArticulationHandle articulation) = 0;
#endif

    /// The collision geometry of whichever shared scene this world has, at this instant.
    ///
    /// ⚠ FOR LOOKING AT, NEVER FOR DECIDING. Nothing in the simulation may branch on this: it is a
    /// read-only view whose whole purpose is to be compared against the level by eye. Returns false
    /// when no scene exists yet or none has compiled, which is a legitimate state and not an error.
    ///
    /// The caller must not be stepping the solver when it calls this.
    virtual bool collisionDebugGeometry(const urdf::CollisionDebugFilter& filter,
                                        urdf::CollisionDebugSnapshot& out) const = 0;

    /// Join the coordinated step/reset phases. Registration must happen before the manifest is
    /// committed, which is why a vehicle does it while it is being constructed.
    virtual void registerParticipant(
        const std::string& stable_id, int order,
        std::shared_ptr<msr::airlib::PhysicsSceneParticipant> participant) = 0;
};

/// The concrete world-scoped scene set. Owned by the sim mode for exactly one UWorld.
class FCoordinatedPhysicsScene : public ICoordinatedPhysicsScene
{
public:
    FCoordinatedPhysicsScene(msr::airlib::PhysicsSceneCoordinator& coordinator,
                             const AirSimSettings::PhysicsCoordinatorSetting& settings,
                             double fixed_step_seconds);

    double fixedStepSeconds() const override { return fixed_step_seconds_; }
    const AirSimSettings::PhysicsCoordinatorSetting& coordinatorSettings() const override
    {
        return settings_;
    }
    bool claimKinematicMirror() override;

#if WITH_BOX3D_BINDING
    b3urdf::Box3DPhysicsScene& box3dScene() override;
    void publishBox3DBody(const std::string& stable_id, size_t scene_handle) override;
#endif

#if WITH_MUJOCO_BINDING
    urdf::MuJoCoPhysicsScene& mujocoScene() override;
    void publishMuJoCoBody(const std::string& stable_id,
                           urdf::MuJoCoPhysicsScene::ArticulationHandle articulation) override;
#endif

    void registerParticipant(
        const std::string& stable_id, int order,
        std::shared_ptr<msr::airlib::PhysicsSceneParticipant> participant) override;

    bool collisionDebugGeometry(const urdf::CollisionDebugFilter& filter,
                                urdf::CollisionDebugSnapshot& out) const override;

    /// Supply the world's one sampled ground height field. Must be set before any scene that needs
    /// a ground is created, i.e. before the first vehicle joins.
    void setGroundHeightField(const urdf::BackendOptions::HeightField& field);

    /// Freeze the scenario: stage every published body, commit the manifest, and let each scene
    /// seal its topology. Called once, after every vehicle exists and before the executor starts.
    void commitManifest();

    bool isEmpty() const { return published_bodies_.empty(); }
    /// One line per published body, for the run's experiment metadata.
    std::string describePopulation() const;

private:
    struct PublishedBody {
        std::string stable_id;
        std::string participant_id;
        std::string backend;
    };

    msr::airlib::PhysicsSceneCoordinator& coordinator_;
    const AirSimSettings::PhysicsCoordinatorSetting& settings_;
    double fixed_step_seconds_ = 0.0;
    bool kinematic_mirror_claimed_ = false;
    urdf::BackendOptions::HeightField ground_height_field_;
    std::vector<PublishedBody> published_bodies_;

#if WITH_BOX3D_BINDING
    std::shared_ptr<urdf::Box3DSceneParticipant> box3d_;
#endif

#if WITH_MUJOCO_BINDING
    std::shared_ptr<urdf::MuJoCoSceneParticipant> mujoco_;
#endif
};
