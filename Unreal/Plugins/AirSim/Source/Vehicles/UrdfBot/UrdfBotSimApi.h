// The URDF vehicle's sim api — and the thread boundary that makes it safe.
//
// ⚠ Read this before changing update(), updateRenderedState() or updateRendering(). The three run
// on different threads under different locks, and the split is not incidental:
//
//   update(dt)              PHYSICS thread, inside World::update(). Steps Box3D and publishes an
//                           immutable snapshot of link poses. Touches no Unreal object at all.
//   updateRenderedState(dt) GAME thread, holding physics_world_->lock() (SimModeWorldBase.cpp:177).
//                           Copies the snapshot out. Kept to a copy because the lock is held.
//   updateRendering(dt)     GAME thread, OUTSIDE the lock. Writes the component transforms.
//
// This is forced, not chosen. The I-R investigation established that mutating UE scene components
// from the physics thread is illegal in Unreal and fails as a *race* rather than deterministically
// — it is what makes the GPU LiDAR path crash — and that marshalling synchronously to the game
// thread deadlocks, because ASimModeWorldBase::Tick already holds physics_world_->lock() while the
// physics thread would be waiting on it. The required shape is therefore strictly
// producer/consumer with no synchronous dependency in either direction, which is exactly what
// AirSim's existing three-method split already provides. No new threading machinery is needed, and
// none should be added.
//
// ⚠ getPhysicsBody() must keep returning nullptr (the UpdatableObject default). That is what stops
// World::insert routing this vehicle into the single shared PhysicsEngineBase, and it is what lets
// a FastPhysics drone and a Box3D rover coexist in one MultiAgent world (§6.0b). update() still
// runs on the physics thread regardless, because UpdatableContainer::update() calls every member.
#pragma once

#include "CoreMinimal.h"

#include "PawnSimApi.h"
#include "UrdfBotPawn.h"
#include "common/AirSimSettings.hpp"
#include "urdf/UrdfCollisionAudit.hpp"
#include "urdf/UrdfModel.hpp"
#include "urdf/UrdfRobotBackend.hpp"
#include "vehicles/urdfbot/api/UrdfBotApiBase.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

class UrdfBotSimApi : public PawnSimApi
{
public:
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::AirSimSettings AirSimSettings;

    UrdfBotSimApi(const Params& params, const AirSimSettings::VehicleSetting* vehicle_setting);
    virtual ~UrdfBotSimApi() = default;

    virtual void initialize() override;

    // --- physics thread ---------------------------------------------------------------------
    virtual void update(float delta = 0) override;

    // --- game thread ------------------------------------------------------------------------
    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;

    virtual void reportState(StateReporter& reporter) override;
    virtual std::string getRecordFileLine(bool is_header_line) const override;

    msr::airlib::UrdfBotApiBase* getVehicleApi() const { return vehicle_api_.get(); }
    virtual msr::airlib::VehicleApiBase* getVehicleApiBase() const override
    {
        return vehicle_api_.get();
    }

    const urdf::Robot& getModel() const { return model_; }
    urdf::UrdfRobotBackend* getBackend() const { return backend_.get(); }

protected:
    virtual void resetImplementation() override;

private:
    void loadModelAndBackend();
    void setupDriveJoints();
    void applyDriveInput();

    /// Joint index + multiplier, resolved once at load. Names are resolved here rather than each
    /// step so a typo is reported at startup instead of silently doing nothing forever.
    struct DriveMapping
    {
        size_t joint = 0;
        double multiplier = 1.0;
    };
    std::vector<DriveMapping> drive_joints_;
    std::vector<DriveMapping> steer_joints_;

    /// One mirrored obstacle: the Unreal component that drives it, and its handle in the backend.
    ///
    /// Weak, because a mirrored actor may be destroyed mid-run — a stale entry simply stops being
    /// pushed. The body stays in the solver at its last pose, which is the honest outcome: the
    /// mirror cannot know whether the actor was deleted or merely unloaded.
    struct KinematicMirror {
        TWeakObjectPtr<UPrimitiveComponent> component;
        int handle = -1;
    };
    std::vector<KinematicMirror> kinematic_mirrors_;

    /// Last drive axes written to the log, so the diagnostic fires on change rather than at 333 Hz.
    float last_logged_throttle_ = 0.0f;
    float last_logged_steering_ = 0.0f;

    const AirSimSettings::VehicleSetting* vehicle_setting_;

    urdf::Robot model_;
    std::unique_ptr<urdf::UrdfRobotBackend> backend_;
    std::unique_ptr<msr::airlib::UrdfBotApiBase> vehicle_api_;
    urdf::CollisionAudit audit_;

    /// Published by the physics thread, consumed by the game thread. Guarded rather than atomic
    /// because it is a whole array: a torn read would put half the robot in the previous frame,
    /// which reads as a robot tearing itself apart and is not obviously a threading bug.
    mutable std::mutex snapshot_mutex_;
    std::vector<urdf::LinkPose> snapshot_;       // physics thread writes, under the mutex
    std::vector<urdf::LinkPose> render_poses_;   // game thread only, no lock needed to read

    /// The clock time this vehicle was last stepped. World::update() calls update() with delta
    /// defaulted to **0** (worldUpdatorAsync passes nothing), so dt has to come from the clock —
    /// the same way FastPhysicsEngine derives it. Using the argument would freeze the robot.
    msr::airlib::TTimePoint last_update_time_ = 0;

    /// Where the robot's URDF origin sits in the Unreal world — the pawn's spawn transform,
    /// captured once. Game thread only.
    /// The mirrored level, shared with every other urdfbot in this world.
    ///
    /// Held for the robot's lifetime because the backend's cooked geometry is keyed on this
    /// pointer's identity (Box3DStaticGeometry) and must not be re-cooked on reset. Dropping it
    /// here would let the cook die and the level silently re-cook.
    ///
    /// Replaces robot_origin_, which composed link poses onto the pawn's spawn transform. That is
    /// incompatible with a level mirrored once at fixed world coordinates — see analysis doc
    /// §6.0c; the robot is now placed into the shared frame via BackendOptions::root_position.
    std::shared_ptr<const urdf::StaticWorld> static_world_;

    bool built_ = false;
    int64_t steps_taken_ = 0;
};
