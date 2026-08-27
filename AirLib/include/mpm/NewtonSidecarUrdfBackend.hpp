// A URDF backend that does not simulate: the VEHICLE lives in the Newton MPM sidecar.
//
// ⚠ WHY THIS EXISTS — plan D15, step 3.
//
// Every other backend behind `UrdfRobotBackend` integrates the robot. This one does not integrate
// anything. It writes joint targets into shared memory, reads link poses back, and interpolates
// between them for however many render frames fall between two published samples. The dynamics
// happen in a separate Python process where Newton solves the robot and the sand as ONE model.
//
// The reason that is worth a whole backend rather than a mode of the MPM link we already had:
// harvested impulses carry only the reaction to MOTION. A kinematic collider has no mass, so
// gravity never acts on it inside the sand solve, and a wheel RESTING on settled sand transfers no
// momentum and reports no force. Measured on a 52 kg Scout, same bed, everything else identical:
//
//     vehicle solved alongside the sand (SolverCoupledProxy) ... 515 N   (1.01x its weight)
//     colliders kinematic, impulses harvested over the wire ....   8 N   (0.015x)
//
// Support and sinkage — the whole of the crater application — live on the far side of that gap,
// and no impulse scale on the wire can manufacture the missing 500 N.
//
// ⚠ NEWTON CANNOT BE LINKED IN. It is Python + Warp with runtime CUDA JIT; embedding it means
// CPython inside UnrealEditor, against the recorded toolchain constraint (host glibc 2.39 vs UE's
// 2.28 sysroot) and with a JIT on the game thread. So the vehicle moves OUT rather than Newton
// moving in, and this class is the seam that makes that invisible to everything above it.
//
// ⚠ LAG THE POSES, NEVER THE FORCES. This is why the direction is right rather than merely
// convenient. A one-step-old POSE is a small position error that sensors barely notice and that
// interpolation hides outright. A one-step-old FORCE applied to a 57 g wheel is the instability
// this workstream spent 2026-08-26 chasing. Nothing in this class ever blocks the game thread
// waiting for the sidecar: if no new pose has arrived, the last one is extrapolated and the sim
// keeps its own tick rate. That is the answer to plan D15's "who owns time" — the simulator does.
//
// ⚠ WHAT IT CANNOT DO, stated here rather than discovered later:
//   - `applyExternalWrench` is not supported. The body being pushed is in another process and the
//     wire has no force channel INTO it. It logs an error ONCE and ignores the call — it must not
//     throw, because UrdfBotSimApi::applyLinkWrench reaches it on the GAME THREAD and an escaping
//     exception there takes the editor down instead of reporting the problem.
//   - `getJointState().effort` is a placeholder unless the sidecar sets `effort_reported`.
//   - `describeColliders` returns false: the sand already has the geometry, and publishing our own
//     copy would invite the old kinematic-collider path to run alongside this one.
#pragma once

#include "urdf/UrdfRobotBackend.hpp"
#include "urdf/UrdfCollisionDebug.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace msr
{
namespace airlib
{
namespace mpm
{

class NewtonSidecarUrdfBackend : public urdf::UrdfRobotBackend
{
public:
    struct Options {
        /// Directory holding the shared-memory segments. /dev/shm is tmpfs, so this is RAM.
        std::string directory = "/dev/shm";
        /// Must equal the sidecar's `--own-vehicle` spec name; that is how one block carries
        /// several vehicles.
        std::string vehicle_name;
        /// ⚠ HOW FAR BEHIND THE NEWEST SAMPLE TO RENDER, in multiples of the sidecar's published
        /// interval. Interpolation needs to sit BETWEEN two samples, so rendering at the newest
        /// one means extrapolating every frame. One interval of delay buys interpolation for the
        /// price of ~40 ms of lag on a 25 Hz publisher, which is below what an operator driving a
        /// rover can perceive and far below what the force path could have tolerated.
        double delay_intervals = 1.0;
        /// Give up on the sidecar after this long with no fresh pose, and say so once.
        double stale_warn_seconds = 2.0;
    };

    explicit NewtonSidecarUrdfBackend(Options options);
    ~NewtonSidecarUrdfBackend() override;

    const char* backendName() const override { return "NewtonSidecar"; }

    // ---- world -------------------------------------------------------------------------

    /// ⚠ MIRRORED OVER THE WIRE, not dropped. The plugin walks the level once and hands every
    /// backend the same geometry; this one serialises it into shared memory so the sidecar can
    /// build the real floor and the real obstacles instead of a flat plane at a height somebody
    /// typed in. That number was got wrong twice in one afternoon — the two ends differ in z by the
    /// AirSim NED origin's offset from the Unreal world origin — and a level whose ground is not
    /// flat cannot be expressed as a number at all.
    void setStaticWorld(std::shared_ptr<const urdf::StaticWorld> world) override;
    /// True: we DO consume it. Returning false made the plugin log "ignores the static world
    /// mirror" and skip work this backend now depends on.
    bool mirrorsStaticWorld() const override { return true; }
    /// True: mirrored dynamic actors are forwarded to the sidecar, so the robot and the sand can
    /// both be pushed by things that move in the level. ⚠ One-directional — see WireKinematicBody.
    bool mirrorsKinematicBodies() const override { return true; }
    /// The sidecar has its own ground. Asking the plugin for scaffolding would put the robot on two
    /// floors at slightly different heights, which reads as jitter.
    bool needsScaffoldingGroundPlane() const override { return false; }

    int addKinematicBody(const urdf::KinematicBody& body) override;
    /// Narrow what a named mirrored actor is solid to, from an authored NewtonPhysicsComponent.
    /// ⚠ Call BEFORE addKinematicBody: both default to true, so an actor with no component keeps
    /// exactly the behaviour it had before the component existed.
    void setKinematicFlags(const std::string& name, bool interact_with_mpm,
                           bool collide_with_robots);
    void setKinematicPose(int handle, const urdf::Vec3& position,
                          const urdf::Quat& orientation) override;

    // ---- lifecycle ---------------------------------------------------------------------

    void buildFromUrdf(const urdf::Robot& model, const urdf::BackendOptions& opts) override;
    void reset() override;
    int step(double dt) override;

    // ---- topology ----------------------------------------------------------------------

    size_t linkCount() const override { return link_names_.size(); }
    size_t jointCount() const override { return joint_names_.size(); }
    const std::string& linkName(size_t link) const override;
    const std::string& jointName(size_t joint) const override;
    int findLink(const std::string& name) const override;
    int findJoint(const std::string& name) const override;

    // ---- state -------------------------------------------------------------------------

    urdf::LinkPose getLinkPose(size_t link) const override;
    urdf::Twist getLinkTwist(size_t link) const override;
    urdf::JointState getJointState(size_t joint) const override;
    double totalMass() const override { return total_mass_; }

    // ---- control -----------------------------------------------------------------------

    void setJointTarget(size_t joint, urdf::ControlMode mode, double value) override;
    /// ⚠ NO-OP WITH A REASON. Gains live in the sidecar's vehicle spec (`wheel_kd`), because that
    /// is where the actuator model is. Silently accepting a gain the solver will never see would
    /// let an operator tune a number that does nothing — and `wheel_kd` is the parameter that
    /// decides whether this vehicle climbs at all, so a fake knob for it is worse than none.
    void setPositionGains(size_t joint, double hertz, double damping_ratio) override;
    void applyExternalWrench(size_t link, const urdf::Wrench& wrench) override;

    // ---- diagnostics -------------------------------------------------------------------

    /// ⚠ THE OVERLAY EXISTS BECAUSE THIS BACKEND'S GEOMETRY IS IN ANOTHER PROCESS. With Box3D and
    /// MuJoCo an operator can at least reason about shapes the same process built; here the solver
    /// is a separate program and the only evidence of where anything is has been the sand's
    /// reaction to it. That cost two wrong diagnoses on 2026-08-27 — "the mirrored ball is frozen
    /// at its build pose" was inferred from sand displacement and was wrong.
    ///
    /// ⚠ PROVENANCE IS NOT DECORATION HERE. Vehicle links are `Realised`: their poses come back
    /// from Newton itself over the wire, so they are what the solver actually has. Mirrored actors
    /// are `Submitted`: they are drawn where we TOLD the sidecar to put them, which is exactly the
    /// claim that needs checking when a robot reacts to something the sand ignores.
    bool collisionDebugGeometry(const urdf::CollisionDebugFilter& filter,
                                urdf::CollisionDebugSnapshot& out) const override;

    /// True once a pose block with our vehicle in it has been read at least once.
    bool isConnected() const;
    /// Simulated seconds between the two most recent published poses; 0 before the second one.
    double observedPublishInterval() const;
    /// How many render steps have been served by extrapolation past the newest sample.
    uint64_t extrapolatedSteps() const;
    /// The translation being added to every published pose so the root lands where the settings
    /// asked. The sidecar's SAND must be drawn with the same shift or the robot and the bed it is
    /// driving on end up 100+ m apart, each internally consistent. False until the first sample.
    bool frameOffset(urdf::Vec3& out) const;
    /// True once the level's collision geometry has been written to the wire.
    bool staticWorldPublished() const;
    /// How fast the sidecar's SIMULATED time advances per second of ours. 1.0 means real time;
    /// 0.18 was measured live on a 1.37 M-particle bed. The playback clock runs at this rate.
    double observedRate() const;

private:
    /// Serialise the level's collision geometry into shared memory, once, after the frame anchor
    /// is known. Cheap no-op on every later call.
    void publishStaticWorld();

    /// Open and map the command segment. Called at build, and again whenever the segment is
    /// replaced underneath us — see `revalidateSegments`.
    void attachCommandSegment();

    /// ⚠ NOTICE WHEN A SEGMENT HAS BEEN REPLACED UNDERNEATH US. Restarting the sidecar deletes and
    /// recreates every block; `mmap` keeps the old, now-unlinked inode alive, so without this the
    /// plugin goes on writing commands nobody reads and reading poses that never change, with
    /// every counter reporting connected. That is why a sidecar restart needed a PIE restart to go
    /// with it.
    void revalidateSegments();

    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::vector<std::string> link_names_;
    std::vector<std::string> joint_names_;
    double total_mass_ = 0.0;
};

} // namespace mpm
} // namespace airlib
} // namespace msr
