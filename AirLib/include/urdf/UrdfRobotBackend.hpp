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

#include "urdf/UrdfConvexDecomposition.hpp"
#include "urdf/UrdfMimic.hpp"
#include "urdf/UrdfModel.hpp"
#include "urdf/UrdfStaticWorld.hpp"

#include <memory>
#include <string>

namespace urdf {

// urdf::Quat now lives in UrdfModel.hpp — see the note at its definition. It is still spelled
// `urdf::Quat` here and everywhere else, so nothing that used it needs to change.

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
    /// The URDF **as text**, when the caller has it.
    ///
    /// ⚠ Optional, and only some backends need it. Box3D builds from the parsed `Robot` struct and
    /// ignores this. MuJoCo has its own URDF reader (`src/xml/xml_urdf.cc`) and takes XML, so
    /// handing it the original bytes avoids re-serialising a parsed model back into XML — a round
    /// trip that would quietly become a second, divergent definition of the robot.
    ///
    /// Empty means "not supplied"; a backend that requires it must say so rather than guess.
    std::string urdf_xml;

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

    /// Where to look for `<mesh>` files named by `<collision>`. `mesh_base_dir` is the directory
    /// holding the URDF; `mesh_search_paths` are the roots a `package://` reference resolves
    /// against. A backend that cannot find a named mesh must refuse, not skip the shape.
    std::string mesh_base_dir;
    std::vector<std::string> mesh_search_paths;

    /// World pose of the **root link** at build time, in the solver frame.
    ///
    /// Was implicitly identity before static geometry existed, with the plugin composing the whole
    /// robot onto its spawn transform afterwards. That cannot survive a shared world: if each
    /// robot's frame is anchored at its own spawn point, the same level triangle needs a different
    /// position per robot, so one cook cannot serve two robots and §6.0c stage 3 is unreachable.
    /// The robot is now placed **in** a world-frame shared by everything.
    Vec3 root_position;
    Quat root_orientation;

    /// A flat static floor at this height in the solver frame.
    ///
    /// ⚠ SCAFFOLDING, now superseded by `setStaticWorld`. Kept as an explicit fallback for the
    /// headless harness (which has no Unreal level to mirror) and for a level whose mirror comes
    /// back empty. A robot given neither a static world nor a ground plane falls forever, which is
    /// why `UrdfBotSimApi` warns loudly about exactly that case.
    bool add_ground_plane = false;
    double ground_plane_z = 0.0;

    /// Convex decomposition of <mesh> collision, shared by both backends. Defaults are in
    /// UrdfConvexDecomposition.hpp; `enabled` with no CoACD in the build is harmless and simply
    /// yields one part, i.e. the behaviour that existed before.
    DecompositionOptions decomposition;

    /// How <mimic> joints are to be honoured. See UrdfMimic.hpp: exact handling is automatic,
    /// approximate handling is opt-in.
    MimicPolicy mimic;
};

class UrdfRobotBackend {
public:
    virtual ~UrdfRobotBackend() = default;

    /// Stable identifier for logs and settings ("Box3D", "Chaos").
    virtual const char* backendName() const = 0;

    /// Supply the static world the robot stands on. Call **before** `buildFromUrdf`.
    ///
    /// Passed as a shared_ptr rather than by value because its identity is the cache key: hand the
    /// same pointer to every robot in the scene and the level is cooked once, then attached to each
    /// robot's world for ~0.006 ms (§6.0c, verified in tests/test_static_geometry.cpp). Hand out
    /// copies instead and each robot pays the full cook.
    ///
    /// The reference is held, so the cooked form survives `reset()` — which destroys and rebuilds
    /// the world (§6.4) and would otherwise re-cook the level on every reset. Re-attaching cooked
    /// geometry is what makes reset stay cheap.
    ///
    /// A null pointer means "no static world", which is a legitimate configuration (a fixed-base
    /// arm needs none) and is not the same as an empty one.
    virtual void setStaticWorld(std::shared_ptr<const StaticWorld> world) = 0;

    /// Whether setStaticWorld actually puts the level into this engine's world.
    ///
    /// ⚠ PURE VIRTUAL, AND DELIBERATELY NOT DEFAULTED TO TRUE. A backend that accepts a
    /// StaticWorld and quietly does nothing with it is indistinguishable, from the caller's side,
    /// from one that mirrored it perfectly — until the robot falls through the floor. That is
    /// exactly what happened when MuJoCo was added: UrdfBotSimApi suppresses its scaffolding
    /// ground plane whenever the level mirrored, on the reasoning that a flat plane and a real
    /// level are not additive, and the MuJoCo robot was left with a physics world containing
    /// itself and nothing else.
    ///
    /// A new backend must therefore answer this question rather than inherit an answer, because
    /// the wrong answer is silent and the failure is a robot falling out of the map.
    virtual bool mirrorsStaticWorld() const = 0;

    /// Register a body whose pose is driven from outside the solver, and return a handle for
    /// `setKinematicPose`. May be called before or after `buildFromUrdf`.
    ///
    /// ⚠ Must survive `reset()`. Reset rebuilds the solver, so a backend that forgets its kinematic
    /// bodies there would silently lose every mirrored obstacle — the same class of defect as
    /// losing position gains on reset, which cost three sessions of debugging. The backend owns the
    /// rebuild, so it owns restoring this.
    virtual int addKinematicBody(const KinematicBody& body) = 0;

    /// Push the current pose of a registered kinematic body. Applied at the next step.
    ///
    /// Cheap and idempotent: call it every frame with whatever the source says now. Poses that do
    /// not change cost nothing beyond the store.
    virtual void setKinematicPose(int handle, const Vec3& position, const Quat& orientation) = 0;

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
