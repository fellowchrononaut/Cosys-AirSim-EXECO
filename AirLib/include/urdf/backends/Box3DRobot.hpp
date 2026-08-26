// A URDF robot instantiated in a Box3D world.
//
// Scope, matching PHYSICS_ENGINE_ANALYSIS.md §6.0: this produces **link poses and joint state**.
// It is a dynamics backend, not a simulator. No sensors, no cameras, no rendering — those stay on
// the AirSim side and are unaffected by which backend moves the robot.
#pragma once

#include "urdf/UrdfConvexDecomposition.hpp"
#include "urdf/UrdfMimic.hpp"
#include "urdf/UrdfModel.hpp"
#include "urdf/UrdfStaticWorld.hpp"
#include "urdf/backends/Box3DMath.hpp"  // WorldPose
#include "urdf/backends/Box3DStaticGeometry.hpp"

#include <box3d/box3d.h>

#include <memory>
#include <string>
#include <vector>

namespace b3urdf {

class Box3DPhysicsScene;

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

    /// Convex decomposition of <mesh> collision. Without it a concave link is one fat hull; see
    /// UrdfConvexDecomposition.hpp. Passed straight through from BackendOptions.
    urdf::DecompositionOptions decomposition;

    /// Where to look for `<mesh>` files named by `<collision>`. `mesh_base_dir` is the directory
    /// holding the URDF, which is what a plain relative filename resolves against; the search paths
    /// are the ROS-package roots that `package://` needs.
    std::string mesh_base_dir;
    std::vector<std::string> mesh_search_paths;

    /// Sides used when tessellating a URDF <cylinder> into a hull.
    int cylinder_sides = 16;

    /// World pose of the **root link** at build time, in the solver frame.
    ///
    /// Identity was implicit before static geometry existed: the robot was built at the origin and
    /// the plugin composed the result onto its spawn transform afterwards. That is incompatible
    /// with a level shared between robots — a triangle at a fixed place in the map would need a
    /// different position in each robot's frame — so the robot is now placed *in* a world frame
    /// whose origin is the Unreal world origin. See analysis doc §6.0c.
    urdf::Vec3 root_position;
    urdf::Quat root_orientation;

    /// A flat static floor in the robot's own world, at this height in the URDF frame.
    ///
    /// ⚠ SCAFFOLDING, superseded by `Box3DRobot::setStaticWorld`. Retained for the headless
    /// harness, which has no Unreal level to mirror, and as the explicit fallback when a level's
    /// mirror comes back empty. A robot given neither falls forever.
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

    /// Register a body whose pose is pushed in from outside each step, returning its handle.
    ///
    /// ⚠ Survives `reset()` by construction: the descriptions live outside the b3World and are
    /// recreated by `instantiate()`. That is deliberate and not incidental — the position-gains
    /// defect (2026-08-17) was exactly this mistake made once already, and reset silently dropping
    /// every mirrored obstacle would look identical to a level that never mirrored.
    int addKinematicBody(const urdf::KinematicBody& body);

    /// Push a registered kinematic body's current pose. Applied on the next step.
    void setKinematicPose(int handle, const urdf::Vec3& position, const urdf::Quat& orientation);

    size_t kinematicBodyCount() const { return kinematic_.size(); }

    /// Supply the static world to stand on. Call **before** `build()`.
    ///
    /// Cooks now (or reuses an existing cook of the same pointer — see Box3DStaticGeometry) and
    /// keeps the cooked form alive, so the level is not re-cooked when `reset()` rebuilds the
    /// world. Handing the *same* StaticWorld pointer to every robot in the scene is what makes the
    /// level cost one cook for the whole scene rather than one per robot.
    void setStaticWorld(std::shared_ptr<const urdf::StaticWorld> world);

    /// The cook backing this robot's static geometry, or nullptr if it has none. For logging the
    /// triangle count and cook cost, which is how an operator sees whether the level actually
    /// mirrored.
    const Box3DStaticGeometry* staticGeometry() const { return static_geometry_.get(); }

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

    /// Re-apply every Effort-mode joint's torque for the step that is about to run.
    ///
    /// ⚠ Called from stepOnce, not from setTarget. Box3D clears applied forces after each step, so
    /// a torque set once would be felt for a single step and then disappear.
    void applyJointTorques();

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

    /// A link whose <mesh> hull did not fit Box3D's 255-half-edge ceiling at the requested vertex
    /// budget and was rebuilt coarser.
    struct HullBudgetReduction {
        std::string link;
        int requested = 0;
        int used = 0;
    };

    /// Links that had neither <inertial> nor <collision> and are therefore resolved by forward
    /// kinematics rather than simulated. Usually frame markers (base_footprint, imu_link) and
    /// entirely normal — but worth naming, because such a link contributes no mass to the robot.
    const std::vector<std::string>& masslessMarkers() const { return massless_markers_; }

    /// Links whose collision hull is coarser than asked for. Empty is the normal case.
    ///
    /// ⚠ Reported rather than swallowed: a hull rebuilt at 16 vertices instead of 64 is a
    /// materially different shape from the one the operator believes is being simulated, and the
    /// difference is invisible in the render.
    const std::vector<HullBudgetReduction>& hullBudgetReductions() const
    {
        return hull_budget_reductions_;
    }

    /// A link <mesh> that was broken into several convex parts instead of being collapsed to one
    /// hull. This is the fidelity WIN, and it is reported for the same reason the losses are:
    /// a link that suddenly has fourteen shapes instead of one costs more per step, and an
    /// operator should be able to see where that went.
    struct MeshDecomposition {
        std::string link;
        std::string mesh;
        size_t parts = 0;
        double seconds = 0;
        bool from_cache = false;
    };

    /// A link <mesh> that did NOT decompose and is therefore still one convex hull — i.e. still
    /// "fatter than it looks".
    ///
    /// ⚠ This is the one that matters. Every entry here is a concave link being simulated as a
    /// solid block, which is invisible in the render and looks exactly like correct behaviour
    /// until something fails to fit through a gap. `note` carries the reason: no CoACD in the
    /// build, disabled by settings, or a CoACD failure on that specific mesh.
    struct MeshDecompositionFallback {
        std::string link;
        std::string mesh;
        std::string note;
    };

    const std::vector<MeshDecomposition>& decompositions() const { return decompositions_; }

    /// Convex parts discarded because they had no volume to hull.
    ///
    /// ⚠ Non-zero is NORMAL and not a fault: CoACD emits thin slivers among its parts. It is
    /// reported because it is also the number that would have crashed the editor — b3CreateHull
    /// segfaults on a degenerate cloud rather than refusing it (see hasHullableVolume in
    /// Box3DRobot.cpp). A sudden jump here means the decomposition threshold is producing junk.
    size_t degenerateParts() const { return degenerate_parts_dropped_; }
    const std::vector<MeshDecompositionFallback>& decompositionFallbacks() const
    {
        return decomposition_fallbacks_;
    }

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
    /// True for the legacy one-robot/one-world path. A robot returned by Box3DPhysicsScene is a
    /// participant in that scene and the scene alone owns stepping and reset.
    bool ownsWorld() const { return owns_world_; }
    /// True once solver objects exist for this robot.
    bool isBuilt() const { return built_; }

    /// Suspend or restore this link's collision against the MIRRORED STATIC WORLD only (plan D10).
    /// Every other contact — self-collision, other robots, other links — is untouched.
    ///
    /// ⚠ TRANSITIONS ONLY. Box3D documents a filter change as "almost as expensive as recreating
    /// the shape", so the caller must own hysteresis and call this when a link actually crosses a
    /// patch boundary, not once per tick. A repeat call with the current value is free.
    ///
    /// ⚠ THIS REMOVES GROUND SUPPORT. A link with world collision off inside a patch whose sand
    /// cannot carry it will fall to whatever is left below. The caller is responsible for the bed
    /// being able to hold the vehicle, or for a floor of last resort.
    bool setLinkWorldCollision(size_t link, bool enabled);

private:
    friend class Box3DPhysicsScene;

    struct LinkRec {
        std::string name;
        b3BodyId body = b3_nullBodyId;
        /// World pose at build time. WorldPose, not b3Transform, because b3Transform::p is float in
        /// both precision modes — tolerable when every robot was built at the origin, not tolerable
        /// now that a robot sits at its true position in a level that may be kilometres across.
        WorldPose initial{};
        std::vector<b3HullData*> owned_hulls;  // freed on teardown

        /// Pose comes from forward kinematics, not from the solver: this link sits at or below a
        /// cosmetic <mimic> joint, for which no Box3D joint is created at all. Reading the body's
        /// transform for such a link would return a stale build-time pose, so linkState() resolves
        /// it instead — in the backend rather than in the renderer, so that sensors, recording and
        /// the RPC API all get the same right answer.
        bool kinematic = false;

        /// Does this link still collide with the mirrored static world? Cached so that
        /// setLinkWorldCollision() can skip the expensive filter write when nothing changed.
        bool world_collision = true;
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

    /// Per-joint control configuration, kept **outside** the Box3D world so it survives the
    /// destroy-and-rebuild that reset() performs.
    ///
    /// ⚠ This exists because reset() rebuilds the solver (§6.4), which silently discarded every
    /// b3Joint's spring gains and enable flags. The steering springs were configured once at load
    /// and were gone after AirSim's startup reset: the spring was re-enabled every step by the
    /// control loop, but at default stiffness, so it exerted no restoring force and the wheels
    /// free-swung. Driving kept working throughout, because velocity control is a motor rather
    /// than a spring — which is exactly why it went unnoticed.
    struct JointControl {
        ControlMode mode = ControlMode::None;
        double target = 0;
        double hertz = 0;
        double damping_ratio = 0;
        bool gains_set = false;
    };

    /// Re-apply `control_` to freshly created joints. Called by reset(), never by build().
    void reapplyControl();

    /// A body driven from outside the solver. The description is kept so the body can be recreated
    /// after reset()'s world rebuild; `target` is the pose most recently pushed in.
    struct KinematicRec {
        urdf::KinematicBody desc;
        b3BodyId body = b3_nullBodyId;
        WorldPose target;
        std::vector<b3HullData*> owned_hulls;
    };

    /// Create the kinematic bodies in the current world. Called by instantiate(), so reset()
    /// restores them along with everything else.
    void instantiateKinematic();
    void addKinematicShapes(KinematicRec& r);

    /// Tear down this robot's solver objects. The legacy path also destroys `world_`; a robot in a
    /// Box3DPhysicsScene destroys only its own bodies because the scene owns the shared world.
    void destroyWorld();
    void createWorld();
    void instantiate();
    void addCollisionShapes(const urdf::Link& link, LinkRec& rec, b3BodyId body);
    void applyInertial(const urdf::Link& link, b3BodyId body);

    /// Mark the subtree under every cosmetic <mimic> joint as kinematically resolved.
    void classifyMimic();
    /// Resolve massless, shapeless frame-marker links kinematically instead of creating zero-mass
    /// dynamic bodies for them. See the implementation for why this is not optional.
    void classifyMasslessMarkers();
    /// Drive every load-bearing <mimic> joint's servo-follower toward its source. Called once per
    /// fixed step, before b3World_Step.
    void updateMimicFollowers();
    /// Joint coordinate of a mimic joint, from its source. Chains resolve by recursion.
    double mimicPosition(size_t joint) const;
    double mimicVelocity(size_t joint) const;
    /// World pose of a kinematically-resolved link. WorldPose rather than b3Transform because
    /// the latter's position is float in both precision modes — see Box3DMath.hpp.
    WorldPose kinematicTransform(size_t link) const;

    /// Shared-scene seam. These are private so no caller can accidentally step or rebuild only one
    /// participant in a shared world; Box3DPhysicsScene is the sole owner of that lifecycle.
    void buildInWorld(const urdf::Robot& model, const BuildOptions& opts, b3WorldId world);
    std::unique_ptr<Box3DRobot> cloneIntoWorld(b3WorldId world) const;
    void prepareSharedStep();
    void finishSharedStep();
    void swapSharedRuntime(Box3DRobot& other) noexcept;

    urdf::Robot model_;
    BuildOptions opts_;
    std::vector<urdf::MimicClassification> mimic_;
    std::vector<HullBudgetReduction> hull_budget_reductions_;
    std::vector<MeshDecomposition> decompositions_;
    size_t degenerate_parts_dropped_ = 0;
    std::vector<MeshDecompositionFallback> decomposition_fallbacks_;
    std::vector<std::string> massless_markers_;
    std::vector<JointControl> control_;
    std::vector<KinematicRec> kinematic_;
    b3WorldId world_ = b3_nullWorldId;
    bool owns_world_ = true;
    /// Held, not borrowed: the cook must outlive every world whose shapes reference its meshes,
    /// and it must survive reset()'s world rebuild.
    std::shared_ptr<const Box3DStaticGeometry> static_geometry_;
    std::vector<LinkRec> links_;
    std::vector<JointRec> joints_;
    double accumulator_ = 0;
    int steps_taken_ = 0;
    bool built_ = false;
};

} // namespace b3urdf
