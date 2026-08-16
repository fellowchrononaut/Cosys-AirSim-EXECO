// A URDF robot instantiated in a Box3D world.
//
// Scope, matching PHYSICS_ENGINE_ANALYSIS.md §6.0: this produces **link poses and joint state**.
// It is a dynamics backend, not a simulator. No sensors, no cameras, no rendering — those stay on
// the AirSim side and are unaffected by which backend moves the robot.
#pragma once

#include "urdf/UrdfMimic.hpp"
#include "urdf/UrdfModel.hpp"
#include "urdf/backends/Box3DMath.hpp"  // WorldPose

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

    /// A flat static floor in the robot's own world, at this height in the URDF frame.
    ///
    /// ⚠ SCAFFOLDING, not the static-geometry feature. Real static geometry means mirroring the
    /// Unreal collision world in (analysis doc §6.1: take Box3DUnreal's cooking/baking approach),
    /// and until that exists a Box3D world contains the robot and nothing else — so a driving robot
    /// simply falls forever. A plane makes a flat-ground robot testable; it is not a substitute,
    /// and anything with terrain, ramps or obstacles will drive straight through them.
    bool add_ground_plane = false;
    double ground_plane_z = 0.0;
    double ground_friction = 0.9;

    /// How <mimic> joints are honoured. See UrdfMimic.hpp — cosmetic couplings are resolved
    /// exactly and automatically; load-bearing ones need an explicit opt-in to the servo-follower
    /// approximation.
    urdf::MimicPolicy mimic;
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

    /// Total mass actually realised as dynamic bodies, for cross-checking against the URDF.
    ///
    /// ⚠ This excludes links resolved kinematically under a cosmetic <mimic>, which Box3D creates
    /// as massless kinematic bodies. So `totalMass() + kinematicMass()` is what should match the
    /// URDF, and the difference is the mass error the cosmetic path introduces. That error is
    /// exactly what MimicPolicy's two thresholds bound — which is their physical meaning.
    double totalMass() const;

    /// URDF mass of the links excluded from the dynamics by the cosmetic-mimic path. Zero for a
    /// model with no cosmetic mimic, which is the common case.
    double kinematicMass() const;

    // --- control -------------------------------------------------------------------------
    void setControlMode(size_t joint, ControlMode mode);
    /// Position control uses Box3D's joint spring (hertz / damping ratio); the motor stays off.
    void setPositionGains(size_t joint, double hertz, double damping_ratio);
    void setTarget(size_t joint, double value);

    /// Apply a world-frame force and torque to one link for the next step.
    ///
    /// ⚠ Box3D clears applied forces on every step, so this must be re-issued each step for a
    /// continuous load. Nothing calls it at Gate 2; it is the hook a URDF drone's rotors need.
    /// No-op on a link resolved kinematically (a cosmetic-mimic link has no dynamics to disturb).
    void applyWrench(size_t link, const double force_world[3], const double torque_world[3],
                     const double point_world[3], bool at_center);

    // --- mimic ---------------------------------------------------------------------------
    /// How each <mimic> joint in the model was classified at build time, in Robot::joints order.
    /// Enumerable on purpose: the operator can audit what the loader decided.
    const std::vector<urdf::MimicClassification>& mimicClassifications() const { return mimic_; }

    /// True if this link's pose is computed by forward kinematics rather than integrated by Box3D
    /// — i.e. it sits at or below a cosmetic <mimic> joint.
    bool isLinkKinematic(size_t link) const { return links_[link].kinematic; }

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

        /// Pose comes from forward kinematics, not from the solver: this link sits at or below a
        /// cosmetic <mimic> joint, for which no Box3D joint is created at all. Reading the body's
        /// transform for such a link would return a stale build-time pose, so linkState() resolves
        /// it instead — in the backend rather than in the renderer, so that sensors, recording and
        /// the RPC API all get the same right answer.
        bool kinematic = false;
        int kinematic_parent = -1;             // link index to compose from
        int kinematic_joint = -1;              // joint index connecting the two
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

        /// <mimic> bookkeeping. `mimic_source >= 0` means q is slaved to another joint.
        int mimic_source = -1;
        double mimic_multiplier = 1;
        double mimic_offset = 0;
        urdf::MimicRole mimic_role = urdf::MimicRole::None;
    };

    void destroyWorld();
    void createWorld();
    void instantiate();
    void addCollisionShapes(const urdf::Link& link, LinkRec& rec, b3BodyId body);
    void applyInertial(const urdf::Link& link, b3BodyId body);

    /// Mark the subtree under every cosmetic <mimic> joint as kinematically resolved.
    void classifyMimic();
    /// Drive every load-bearing <mimic> joint's servo-follower toward its source. Called once per
    /// fixed step, before b3World_Step.
    void updateMimicFollowers();
    /// Joint coordinate of a mimic joint, from its source. Chains resolve by recursion.
    double mimicPosition(size_t joint) const;
    double mimicVelocity(size_t joint) const;
    /// World pose of a kinematically-resolved link. WorldPose rather than b3Transform because
    /// the latter's position is float in both precision modes — see Box3DMath.hpp.
    WorldPose kinematicTransform(size_t link) const;

    urdf::Robot model_;
    BuildOptions opts_;
    std::vector<urdf::MimicClassification> mimic_;
    b3WorldId world_ = b3_nullWorldId;
    std::vector<LinkRec> links_;
    std::vector<JointRec> joints_;
    double accumulator_ = 0;
    int steps_taken_ = 0;
    bool built_ = false;
};

} // namespace b3urdf
