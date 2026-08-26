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
#include "SimMode/CoordinatedPhysicsScene.h"
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

    /// `coordinated_scene` is null in Legacy mode, and non-null when this robot must join the
    /// world's shared solver scene instead of creating a private one.
    UrdfBotSimApi(const Params& params, const AirSimSettings::VehicleSetting* vehicle_setting,
                  ICoordinatedPhysicsScene* coordinated_scene = nullptr);
    virtual ~UrdfBotSimApi();

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
    /// The overlay's view of this robot. Delegates to whichever backend it has — shared or
    /// private — under the same mutex every other backend read takes.
    bool collisionDebugGeometry(const urdf::CollisionDebugFilter& filter,
                                urdf::CollisionDebugSnapshot& out) const override;

    /// This robot's links as MPM colliders, qualified by vehicle name.
    bool describeColliders(urdf::PhysicsColliderSet& out) const override;
    bool applyLinkWrench(size_t link_index, const urdf::Wrench& wrench) override;
    bool setLinkWorldCollision(size_t link_index, bool enabled) override;


    /// The robot's kinematics and the simulator time they were sampled at, as ONE value.
    ///
    /// ⚠ Called from the RPC thread. It returns a copy of a snapshot published by the game thread
    /// rather than reading kinematics_ live, and that is the entire point: the pair has to be
    /// taken at one instant. Reading getGroundTruthKinematics() here and clock()->nowNanos()
    /// separately would stamp the kinematics with a time up to a frame later than the frame they
    /// describe, and under a paused or scaled clock the gap is unbounded.
    msr::airlib::UrdfBotApiBase::UrdfBotState getUrdfBotState() const;

    // --- coordinated physics, physics thread --------------------------------------------------
    //
    // ⚠ These replace update()'s solve, they do not supplement it. The shared world is advanced
    // once per coordinated tick by its own scene participant; this robot only latches its commands
    // before that solve and reads the result after it, which is the same order the private-world
    // update() had inside one call.
    void coordinatedPreSolve();
    void coordinatedPostSolve(double dt);

    /// Everything that needs solver state but could not run at load.
    ///
    /// ⚠ MuJoCo CANNOT be built one robot at a time. `mjs_attach` + `mj_compile` is a batch
    /// operation, so a shared MuJoCo scene has no bodies until every articulation has joined and
    /// the manifest commits — while Box3D's shared scene builds each robot immediately and is
    /// queryable at once. The mass cross-check and the first rendered poses therefore run here,
    /// driven by the participant's manifest-finalize hook, for backends that are not live at load.
    /// Called on the game thread, once, and never throws: a diagnostic must not abort a scenario
    /// that has already been committed.
    void onSharedSceneReady() noexcept;

    bool isCoordinated() const { return coordinated_scene_ != nullptr; }

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
        /// Steering contribution SUMMED into this joint's velocity target — skid steer. Zero for a
        /// joint the operator did not list in SkidSteerJoints, which is why drive and skid must be
        /// accumulated per joint rather than written by two separate loops: the second would
        /// overwrite the first and steering would silently replace throttle.
        double skid = 0.0;
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

    /// Frames of deferred kinematic re-mirroring still to attempt. See refreshKinematicMirror.
    ///
    /// ⚠ Not zero, because the mirror taken during initialisation CANNOT see other URDF robots —
    /// they are still being spawned, and their link components do not exist yet. Re-collecting the
    /// kinematic half on the game thread once everything exists is what makes Box3D robots
    /// interact with each other at all.
    int kinematic_refresh_frames_ = 8;

    /// Re-collect kinematic bodies and register any that are not already tracked.
    ///
    /// ⚠ GAME THREAD, WITH PHYSICS PAUSED. Called only from updateRenderedState, which is the one
    /// place both hold: it reads Unreal actor transforms (game thread) and mutates the backend
    /// (physics stopped under physics_world_->lock()).
    void refreshKinematicMirror();

    /// Whether THIS robot maintains the world's kinematic obstacle mirror. Always true in Legacy
    /// mode, where each robot owns a private world; in a shared world exactly one robot claims it.
    bool owns_kinematic_mirror_ = true;

    /// Last drive axes written to the log, so the diagnostic fires on change rather than at 333 Hz.
    float last_logged_throttle_ = 0.0f;
    float last_logged_steering_ = 0.0f;

    const AirSimSettings::VehicleSetting* vehicle_setting_;

    /// Borrowed from the sim mode, which owns it for the whole UWorld. Null in Legacy mode.
    ICoordinatedPhysicsScene* coordinated_scene_ = nullptr;
    /// The coordinator holds this by shared_ptr; it holds this api by raw pointer and is detached
    /// in the destructor, so a callback can never reach a destroyed vehicle.
    std::shared_ptr<class UrdfBotStepParticipant> step_participant_;

    urdf::Robot model_;
    std::unique_ptr<urdf::UrdfRobotBackend> backend_;
    /// Guards every RPC-thread read/write of backend_ against resetImplementation()'s
    /// backend_->reset(), which destroys and rebuilds every Box3D body and joint (§6.4) rather than
    /// rewriting poses in place. Without this, an RPC accessor mid-read of a body Box3D has just
    /// freed is a use-after-free, not a stale value - confirmed from a crash dump: getJointStates()
    /// reading through a destroyed body while the ROS wrapper's continuous /joint_states poll raced
    /// a reset, 2026-08-20. UrdfBotApi is handed a pointer to this at construction and takes it for
    /// the duration of every method that touches backend_.
    ///
    /// ⚠ Does NOT cover update() on the physics thread, which also calls backend_ directly every
    /// step (drive-joint writes, step()). Reset only races update() if something can invoke it from
    /// a thread other than the one update() runs on; deliberately left unaudited and unlocked here -
    /// scope this in if that turns out to happen.
    /// `mutable` so the const collision-overlay read can take it. Taking the lock is not a
    /// logical mutation of the vehicle; skipping it while the physics thread rebuilds every body
    /// on reset would be a crash.
    mutable std::mutex backend_mutex_;
    /// The URDF exactly as read from disk, served to clients through getUrdfXml(). Held because
    /// a ROS client in a container generally cannot open UrdfFile itself.
    std::string urdf_xml_raw_;

    std::unique_ptr<msr::airlib::UrdfBotApiBase> vehicle_api_;
    urdf::CollisionAudit audit_;

    /// Published by the physics thread, consumed by the game thread. Guarded rather than atomic
    /// because it is a whole array: a torn read would put half the robot in the previous frame,
    /// which reads as a robot tearing itself apart and is not obviously a threading bug.
    mutable std::mutex snapshot_mutex_;
    std::vector<urdf::LinkPose> snapshot_;       // physics thread writes, under the mutex
    std::vector<urdf::LinkPose> render_poses_;   // game thread only, no lock needed to read

    /// Kinematics + simulator stamp, published together by the game thread at the moment the
    /// kinematics are recomputed, and read by the RPC thread.
    ///
    /// ⚠ Guarded rather than atomic for the same reason snapshot_ is: it is wider than a word, and
    /// a torn read would pair one frame's pose with another frame's timestamp — which is precisely
    /// the error this member exists to remove, and would be invisible in the data.
    mutable std::mutex state_mutex_;
    msr::airlib::UrdfBotApiBase::UrdfBotState published_state_;

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

    /// Ground sampling, for backends that cannot mirror the level's ground (MuJoCo).
    bool sampleGroundHeightField(const urdf::Vec3& centre, double half_extent,
                                 urdf::BackendOptions::HeightField& out);
    void refreshGroundHeightField();

    /// Step the backend before play so the robot starts at rest, then publish those
    /// poses to the renderer. Called at load AND after every reset — reset rebuilds
    /// the world at the spawn pose, so a load-only settle is silently discarded.
    void settleAndPublish();

    /// Where the current height patch is centred, so the robot driving out of it can be detected.
    urdf::Vec3 height_field_centre_;
    double height_field_half_extent_ = 0;
    bool has_height_field_ = false;

    /// False while a shared scene has not yet produced solver state for this robot.
    bool backend_state_available_ = true;
    bool built_ = false;
    int64_t steps_taken_ = 0;
};
