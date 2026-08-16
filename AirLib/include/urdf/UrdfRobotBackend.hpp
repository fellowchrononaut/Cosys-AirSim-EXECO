// The dynamics backend behind a URDF vehicle.
//
// This is the seam described in PHYSICS_ENGINE_ANALYSIS.md §6.3, and it is the highest-value
// structural decision in the proposal: it is what makes a backend swap possible and what makes a
// Box3D failure survivable.
//
//     UrdfRobotBackend        (abstract; AirLib — this file)
//     +-- Box3DUrdfBackend    (AirLib solver + plugin geometry; the research bet)
//     +-- ChaosUrdfBackend    (plugin; UPhysicsConstraintComponent — optional, Gate 3)
//
// ⚠ Scope, per §6.0: a backend supplies **link poses, link twists and joint state, and nothing
// else**. Cameras, LiDAR, IMU, recording and the whole 3DGEER render path stay exactly where they
// are and are unaffected by which backend moved the robot. Nothing about sensors appears here, and
// nothing should be added.
//
// ⚠ This header is deliberately free of Box3D, Unreal and AirLib types. Anything that leaks an
// engine type through this interface defeats its purpose.
#pragma once

#include "urdf/UrdfMimic.hpp"
#include "urdf/UrdfModel.hpp"

#include <string>

namespace urdf {

struct Quat {
    double x = 0, y = 0, z = 0, w = 1;
};

struct LinkPose {
    Vec3 position;
    Quat orientation;
};

struct Twist {
    Vec3 linear;
    Vec3 angular;
};

/// A force/torque pair to apply to one link, in **world** axes.
///
/// Nothing calls this at Gate 2, and it is here on purpose. A URDF *drone* is URDF links and joints
/// in the solver plus AirLib's existing rotor vertices applying a wrench at a point — AirSim rotors
/// are already `PhysicsBodyVertex` objects producing exactly this (MultiRotorPhysicsBody.hpp:105).
/// Keeping the hook is the entire cost of leaving that option open (§6.0b).
///
/// ⚠ Applied wrenches are consumed by the next step and do not persist; a continuous force must be
/// re-applied every step, before step(). That is the convention of every engine behind this
/// interface, and making it explicit here stops it becoming a per-backend surprise.
struct Wrench {
    Vec3 force;             ///< newtons, world axes
    Vec3 torque;            ///< newton-metres, world axes
    Vec3 point;             ///< world-frame point at which `force` acts
    bool at_center = true;  ///< ignore `point` and apply `force` at the centre of mass
};

enum class ControlMode { None, Position, Velocity, Effort };

struct JointState {
    double position = 0;  ///< rad or m
    double velocity = 0;  ///< rad/s or m/s
    double effort = 0;    ///< N.m or N — the motor's applied effort, not the constraint reaction
};

/// Options every backend must honour. Engine-specific tuning (solver substeps, hull budgets,
/// worker counts) belongs to the concrete backend, not here.
struct BackendOptions {
    /// Internal step. A backend must consume whole steps of exactly this size and carry the
    /// remainder, so the outer loop's dt cannot perturb the result. MultiAgent advances
    /// SteppableClock by getPhysicsLoopPeriod() * 1E-9 = 3 ms per executor iteration.
    double fixed_timestep = 0.003;

    double gravity_z = -9.81;

    /// Anchor the root link to the world. URDF has no notion of a fixed base — it is a loader
    /// convention — so it is explicit. True for arms and test rigs; **false** for anything that
    /// drives or flies, whose root must fall under gravity and rest on terrain.
    bool fixed_base = true;

    /// Collision between links joined by a joint. URDF's convention is that adjacent links do not
    /// self-collide.
    bool collide_connected = false;

    /// A flat static floor at this height in the robot's frame.
    ///
    /// ⚠ SCAFFOLDING. Real static geometry means mirroring the Unreal collision world into the
    /// backend; until that exists the backend's world holds the robot and nothing else, so a
    /// driving robot falls forever. A plane makes flat-ground robots testable and is not a
    /// substitute — terrain, ramps and obstacles are simply not there.
    bool add_ground_plane = false;
    double ground_plane_z = 0.0;

    /// How <mimic> joints are to be honoured. See UrdfMimic.hpp: exact handling is automatic,
    /// approximate handling is opt-in.
    MimicPolicy mimic;
};

class UrdfRobotBackend {
public:
    virtual ~UrdfRobotBackend() = default;

    /// Stable identifier for logs and settings ("Box3D", "Chaos").
    virtual const char* backendName() const = 0;

    /// Instantiate `model`. Throws std::runtime_error, with a message naming the offending link or
    /// joint, for anything the backend cannot represent faithfully. Refusing is deliberate: a
    /// backend that loads a robot quietly differing from its URDF is the defect this whole
    /// interface exists to prevent.
    virtual void buildFromUrdf(const Robot& model, const BackendOptions& opts) = 0;

    /// Return to the build-time state reproducibly.
    ///
    /// ⚠ Not "rewrite the poses". Box3D has no rollback determinism — contact caches, warm-start
    /// impulses and island state survive a pose write, so a pose-restoring reset diverges silently
    /// from a fresh build. That is the worst possible failure for a dataset generator, so the
    /// contract here is *reproducibility*, and a backend that cannot offer it any other way must
    /// rebuild (§6.4).
    virtual void reset() = 0;

    /// Advance by `dt` seconds. Returns the number of fixed internal steps actually taken.
    virtual int step(double dt) = 0;

    // --- structure -------------------------------------------------------------------------
    virtual size_t linkCount() const = 0;
    virtual size_t jointCount() const = 0;
    virtual const std::string& linkName(size_t link) const = 0;
    virtual const std::string& jointName(size_t joint) const = 0;
    virtual int findLink(const std::string& name) const = 0;
    virtual int findJoint(const std::string& name) const = 0;

    // --- state -----------------------------------------------------------------------------
    // In the URDF/ROS frame: right-handed, Z-up, metres. Conversion to Unreal or to AirSim NED
    // happens at the plugin boundary, never here.
    virtual LinkPose getLinkPose(size_t link) const = 0;
    virtual Twist getLinkTwist(size_t link) const = 0;
    virtual JointState getJointState(size_t joint) const = 0;

    /// Mass actually realised in the solver, for cross-checking against the URDF. A mismatch means
    /// links were merged, dropped or given shape-derived mass.
    virtual double totalMass() const = 0;

    // --- control ---------------------------------------------------------------------------
    virtual void setJointTarget(size_t joint, ControlMode mode, double value) = 0;
    virtual void setPositionGains(size_t joint, double hertz, double damping_ratio) = 0;
    virtual void applyExternalWrench(size_t link, const Wrench& wrench) = 0;
};

} // namespace urdf
