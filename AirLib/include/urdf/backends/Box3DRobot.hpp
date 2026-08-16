// A URDF robot instantiated in a Box3D world.
//
// Scope, matching PHYSICS_ENGINE_ANALYSIS.md §6.0: this produces **link poses and joint state**.
// It is a dynamics backend, not a simulator. No sensors, no cameras, no rendering — those stay on
// the AirSim side and are unaffected by which backend moves the robot.
#pragma once

#include "urdf/UrdfModel.hpp"

#include <box3d/box3d.h>

#include <string>
#include <vector>

namespace b3urdf {

struct BuildOptions {
    /// Box3D rejects fewer than 4 substeps for stiff chains.
    int substeps = 4;

    /// Sim clock step in the target simmode. MultiAgent advances SteppableClock by exactly this
    /// much per executor iteration — see SimModeWorldMultiAgent::setupClockSpeed, which overrides
    /// the 20 ms ASimModeBase default with getPhysicsLoopPeriod() * 1E-9.
    double fixed_timestep = 0.003;

    /// Keep at 1 only if you need to match a specific recorded hash. Box3D is deterministic across
    /// worker counts (verified: test_determinism.c asserts one hash for workers 1..5), so this is
    /// a throughput knob, not a correctness one.
    int worker_count = 1;

    double gravity_z = -9.81;

    /// Anchor the root link to the world as a static body. URDF has no notion of a fixed base —
    /// it is a loader convention — so it is explicit here. True for arms and test rigs mounted to
    /// ground; **false** for a free-flying or driving robot such as a rover, whose root must fall
    /// under gravity and rest on terrain. robox3d defaults to fixed; we default to fixed too, so
    /// that forgetting to set it produces an obviously-pinned robot rather than one that silently
    /// sinks through the floor.
    bool fixed_base = true;

    /// Collision between links joined by a joint. URDF's convention is that adjacent links do not
    /// self-collide; Box3D's b3JointDef::collideConnected defaults to false, which agrees.
    bool collide_connected = false;

    /// Convex-hull vertex budget per collision shape.
    int max_hull_vertices = 64;

    /// Sides used when tessellating a URDF <cylinder> into a hull.
    int cylinder_sides = 16;
};

enum class ControlMode { None, Position, Velocity, Effort };

struct JointState {
    double position = 0;   // rad or m
    double velocity = 0;   // rad/s or m/s
    double effort = 0;     // N.m or N — the motor's applied effort, not the constraint reaction
};

struct LinkState {
    double position[3]{ 0, 0, 0 };
    double orientation[4]{ 0, 0, 0, 1 };  // x, y, z, w
    double linear_velocity[3]{ 0, 0, 0 };
    double angular_velocity[3]{ 0, 0, 0 };
};

class Box3DRobot {
public:
    Box3DRobot() = default;
    ~Box3DRobot();

    Box3DRobot(const Box3DRobot&) = delete;
    Box3DRobot& operator=(const Box3DRobot&) = delete;

    /// Build the robot into a freshly created private world.
    ///
    /// One world per robot, deliberately (analysis doc §6.0b): cross-engine robot-to-robot contact
    /// is impossible in the target simulator anyway, and a private world keeps the reset rebuild
    /// (R10) contained to this robot instead of disturbing every vehicle.
    void build(const urdf::Robot& model, const BuildOptions& opts = {});

    /// Destroy and rebuild the world. Box3D has no rollback determinism — the FAQ is explicit that
    /// a world cannot be restored to a prior state and resumed with identical results, because
    /// contact caches, warm-start impulses and island state survive a pose write. So a reproducible
    /// reset must rebuild, not rewrite poses. Cost is measured at Gate 1 (M6).
    void reset();

    /// Advance by `dt` seconds using a fixed internal timestep, consuming whole steps only and
    /// carrying the remainder. Keeps b3World_Step's dt identical regardless of what the caller
    /// passes, which is what makes the engine's determinism usable.
    /// Returns the number of Box3D steps actually taken.
    int step(double dt);

    /// Advance by exactly one fixed timestep, ignoring the accumulator. For tests that want an
    /// exact step count.
    void stepOnce();

    // --- state ---------------------------------------------------------------------------
    size_t linkCount() const { return links_.size(); }
    size_t jointCount() const { return joints_.size(); }
    const std::string& linkName(size_t i) const { return links_[i].name; }
    const std::string& jointName(size_t i) const { return joints_[i].name; }
    int findJoint(const std::string& name) const;

    LinkState linkState(size_t i) const;
    JointState jointState(size_t i) const;

    /// Total mass actually realised in the world, for cross-checking against the URDF.
    double totalMass() const;

    // --- control -------------------------------------------------------------------------
    void setControlMode(size_t joint, ControlMode mode);
    /// Position control uses Box3D's joint spring (hertz / damping ratio) plus the motor.
    void setPositionGains(size_t joint, double hertz, double damping_ratio);
    void setTarget(size_t joint, double value);

    // --- introspection for tests ---------------------------------------------------------
    b3WorldId worldId() const { return world_; }
    b3BodyId bodyId(size_t link) const { return links_[link].body; }
    /// Sum over dynamic bodies of position and velocity, folded in a stable order. Cheap proxy for
    /// "did this run reproduce", complementing Box3D's own recording/replay validation.
    unsigned int stateHash() const;

    int stepsTaken() const { return steps_taken_; }
    const BuildOptions& options() const { return opts_; }

private:
    struct LinkRec {
        std::string name;
        b3BodyId body = b3_nullBodyId;
        b3Transform initial{};                 // world pose at build time
        std::vector<b3HullData*> owned_hulls;  // freed on teardown
    };
    struct JointRec {
        std::string name;
        urdf::JointType type = urdf::JointType::Fixed;
        b3JointId joint = b3_nullJointId;
        int parent_link = -1;
        int child_link = -1;
        b3Vec3 axis_child{ 0, 0, 1 };          // normalised, in the child frame
        ControlMode mode = ControlMode::None;
        double target = 0;
        double effort_limit = 0;
        double velocity_limit = 0;
    };

    void destroyWorld();
    void createWorld();
    void instantiate();
    void addCollisionShapes(const urdf::Link& link, LinkRec& rec, b3BodyId body);
    void applyInertial(const urdf::Link& link, b3BodyId body);

    urdf::Robot model_;
    BuildOptions opts_;
    b3WorldId world_ = b3_nullWorldId;
    std::vector<LinkRec> links_;
    std::vector<JointRec> joints_;
    double accumulator_ = 0;
    int steps_taken_ = 0;
    bool built_ = false;
};

} // namespace b3urdf
