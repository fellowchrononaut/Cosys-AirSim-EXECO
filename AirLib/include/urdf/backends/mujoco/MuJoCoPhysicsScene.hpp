// Experimental world-scoped MuJoCo scene.
//
// This is deliberately separate from MuJoCoUrdfBackend. The existing backend remains one private
// mjModel/mjData per robot and is the Legacy path. This class is the Phase-4 proof that multiple
// URDF articulations can instead be attached, namespaced, compiled, stepped and reset as one
// solver scene. No caller is switched to it merely because this seam exists.
#pragma once

#include "urdf/UrdfRobotBackend.hpp"
#include "urdf/UrdfStaticWorld.hpp"
#include "urdf/UrdfCollisionDebug.hpp"
#include "urdf/backends/mujoco/MuJoCoStaticWorld.hpp"

#include <memory>

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

struct mjModel_;
struct mjData_;
struct mjSpec_;

namespace urdf {

class MuJoCoPhysicsScene
{
public:
    struct Options {
        double fixed_timestep = 0.003;
        double gravity_z = -9.81;

        /// Fallback floor, used only when no global height field was supplied.
        bool add_ground_plane = false;
        double ground_plane_z = 0.0;

        /// ⚠ THE ONE GLOBAL GROUND. A per-robot height field is rejected by `addArticulation`,
        /// because a robot-centred field moved for one robot moves it for every robot, and two
        /// overlapping fields double-contact. This is the world's, sampled once.
        BackendOptions::HeightField ground_height_field;

        /// World-level level-geometry policy. `static_world_radius` of 0 takes the level whole:
        /// a shared world has no single robot to clip around, so clipping must be an explicit
        /// choice with an explicit centre rather than a silent per-robot default.
        BackendOptions::StaticMeshMode static_mesh_mode = BackendOptions::StaticMeshMode::Split;
        double static_world_radius = 0.0;
        Vec3 static_world_clip_center;
        int static_world_max_triangles = 20000;
    };

    struct ArticulationHandle {
        size_t index = static_cast<size_t>(-1);
        bool valid() const { return index != static_cast<size_t>(-1); }
    };

    struct LinkHandle {
        size_t articulation = static_cast<size_t>(-1);
        size_t link = static_cast<size_t>(-1);
        bool valid() const
        {
            return articulation != static_cast<size_t>(-1) && link != static_cast<size_t>(-1);
        }
    };

    struct JointHandle {
        size_t articulation = static_cast<size_t>(-1);
        size_t joint = static_cast<size_t>(-1);
        bool valid() const
        {
            return articulation != static_cast<size_t>(-1) && joint != static_cast<size_t>(-1);
        }
    };

    MuJoCoPhysicsScene();
    explicit MuJoCoPhysicsScene(const Options& options);
    ~MuJoCoPhysicsScene();

    MuJoCoPhysicsScene(const MuJoCoPhysicsScene&) = delete;
    MuJoCoPhysicsScene& operator=(const MuJoCoPhysicsScene&) = delete;

    /// Attach one URDF into the still-editable scene. The stable id is encoded into a collision-
    /// free MuJoCo prefix, so two copies may use identical URDF link/joint/asset names.
    ///
    /// Global values in BackendOptions must agree with Options. Ground is emitted once by the
    /// scene, never once per robot. Height fields are intentionally rejected by this isolated
    /// spike until a world-level descriptor owns them.
    ArticulationHandle addArticulation(const std::string& stable_id, const Robot& model,
                                       const BackendOptions& options);

    /// Supply the one mirrored level for the whole scene. Call before `compile`.
    ///
    /// ⚠ Emitted at compile time, not here, so the ground and the level are described to MuJoCo in
    /// one place with one policy. A null pointer is a legitimate configuration (no level at all);
    /// it is not the same as an empty one, and a robot given neither must fall loudly.
    void setStaticWorld(std::shared_ptr<const StaticWorld> world);
    const StaticWorldEmitStats& staticWorldStats() const { return static_world_stats_; }

    /// Freeze topology, emit the ground and level, and create the one shared mjModel/mjData pair.
    void compile();

    /// Advance the shared mjData with exactly one mj_step. dt must equal fixed_timestep; this seam
    /// never hides per-robot accumulators or solver substeps.
    void step(double dt);

    /// Reset every articulation and all contact/solver state together without rebuilding topology.
    ///
    /// Matches the per-robot backend's semantics: control CONFIGURATION (mode and gains) survives,
    /// control STATE (the commanded target) does not.
    void reset();

    // --- control ------------------------------------------------------------------------------
    //
    // ⚠ A URDF declares no actuators and MuJoCo requires them, so one torque motor per movable
    // joint is added to each articulation's spec BEFORE it is attached - the attach then namespaces
    // the actuator along with everything else. Position and velocity control are computed as a PD
    // over that torque here, exactly as the per-robot backend does, so a controller written against
    // one sees the same gains in the other.
    /// Both are pure CONFIGURATION and may be called before the scene compiles: a host sets up its
    /// drive joints while it is still assembling the vehicle, long before every articulation has
    /// joined. Gains are resolved against the compiled inertia when the model is built.
    void setJointTarget(JointHandle joint, ControlMode mode, double value);
    void setPositionGains(JointHandle joint, double hertz, double damping_ratio);
    void applyExternalWrench(LinkHandle link, const Wrench& wrench);

    Twist getLinkTwist(LinkHandle link) const;
    /// Mass realised in the solver for one articulation, for cross-checking against its URDF.
    double articulationMass(ArticulationHandle articulation) const;
    /// Mass of every body in the shared scene except the world body.
    double totalMass() const;

    /// Geoms this articulation actually has after compilation.
    ///
    /// ⚠ Zero means the robot collides with NOTHING and will fall through the world, however
    /// healthy every other number looks. MuJoCo drops meshes it cannot read without complaining.
    size_t collisionGeomsRealised(ArticulationHandle articulation) const;

    /// Mesh references the last `addArticulation` could not resolve to a readable file.
    const std::vector<std::string>& unresolvedMeshes() const { return unresolved_meshes_; }

    bool compiled() const { return model_ != nullptr && data_ != nullptr; }
    bool faulted() const { return faulted_; }
    size_t articulationCount() const { return articulations_.size(); }
    ArticulationHandle findArticulation(const std::string& stable_id) const;
    size_t linkCount(ArticulationHandle articulation) const;
    size_t jointCount(ArticulationHandle articulation) const;
    const std::string& stableId(ArticulationHandle articulation) const;

    LinkHandle findLink(ArticulationHandle articulation, const std::string& local_name) const;
    JointHandle findJoint(ArticulationHandle articulation, const std::string& local_name) const;
    /// Index-based access, in the articulation's own ordering.
    ///
    /// ⚠ Joint indices skip fixed, floating and planar joints, exactly as the per-robot backend's
    /// do. A client enumerating joints must see the same list whichever backend moved the robot.
    LinkHandle linkAt(ArticulationHandle articulation, size_t index) const;
    JointHandle jointAt(ArticulationHandle articulation, size_t index) const;
    const std::string& linkName(LinkHandle link) const;
    const std::string& jointName(JointHandle joint) const;
    const std::string& qualifiedLinkName(LinkHandle link) const;
    const std::string& qualifiedJointName(JointHandle joint) const;

    LinkPose getLinkPose(LinkHandle link) const;
    JointState getJointState(JointHandle joint) const;

    /// Every collision geom the COMPILED MODEL holds, at this instant, in solver frame.
    ///
    /// ⚠ READ BACK, not remembered. Mesh vertices come from `mesh_vert`/`mesh_face`, which for a
    /// vertex-only mesh is the convex hull qhull built — so this reports what MuJoCo will actually
    /// collide against, including the places where that is fatter than the asset we submitted.
    /// Height-field elevations are denormalised back to metres, because MuJoCo rescales them to
    /// [0,1] at compile and a caller must not have to know that.
    ///
    /// Cheap enough to call per frame at the default budget, and it takes no locks: the caller
    /// owns the step, so it must call this while the solver is not stepping.
    void collisionDebugGeometry(const CollisionDebugFilter& filter,
                                CollisionDebugSnapshot& out) const;

    /// Describe ONE articulation's links as registerable colliders (plan §11.1).
    ///
    /// ⚠ Per articulation, not per scene. A sidecar registers colliders against a robot, and a
    /// whole-scene call would hand it every other robot's links as though they were this one's.
    void describeColliders(ArticulationHandle articulation, PhysicsColliderSet& out) const;

    /// Diagnostics used by the headless proof. IDs for handles remain unchanged across reset.
    int compiledBodyId(LinkHandle link) const;
    int compiledJointId(JointHandle joint) const;
    size_t compiledBodyCount() const;
    size_t compiledJointCount() const;
    size_t contactCountBetween(ArticulationHandle first, ArticulationHandle second) const;
    double simulationTime() const;
    uint64_t stepsTaken() const { return steps_taken_; }

private:
    struct LinkRec {
        std::string name;
        std::string qualified_name;
        int body = -1;
    };

    struct JointRec {
        std::string name;
        std::string qualified_name;
        int joint = -1;
        int qposadr = -1;
        int dofadr = -1;

        /// The torque motor added for this joint, resolved after compilation.
        int actuator = -1;
        double effort_limit = 0;

        // Control configuration survives reset; the target does not.
        ControlMode mode = ControlMode::None;
        double target = 0;
        double kp = 0;
        double kd = 0;

        /// Gains as REQUESTED, kept separately from the kp/kd they resolve to.
        ///
        /// ⚠ hertz -> kp needs the compiled joint inertia, which does not exist until the scene
        /// compiles - but a host configures its drive joints while it is still assembling the
        /// vehicle. Storing the request and resolving it at compile is what lets configuration
        /// happen in any order; requiring a compiled scene here instead just moved the ordering
        /// problem into every caller.
        bool gains_requested = false;
        double gain_hertz = 0;
        double gain_damping_ratio = 0;
    };

    struct ArticulationRec {
        std::string stable_id;
        std::string prefix;
        std::vector<LinkRec> links;
        std::vector<JointRec> joints;
        std::map<std::string, size_t> link_index;
        std::map<std::string, size_t> joint_index;
        size_t collision_geoms = 0;
    };

    static std::string makePrefix(const std::string& stable_id);
    static bool isMappedJoint(JointType type);
    void requireEditable(const char* operation) const;
    void requireCompiled(const char* operation) const;
    const ArticulationRec& articulation(ArticulationHandle handle) const;
    const LinkRec& link(LinkHandle handle) const;
    const JointRec& joint(JointHandle handle) const;
    void resolveCompiledHandles();
    void applyControls();
    JointRec& mutableJoint(JointHandle handle);
    void destroy();

    Options options_;
    mjSpec_* spec_ = nullptr;
    mjModel_* model_ = nullptr;
    mjData_* data_ = nullptr;
    std::vector<ArticulationRec> articulations_;
    std::map<std::string, size_t> articulation_index_;
    std::vector<int> body_owner_;
    std::shared_ptr<const StaticWorld> static_world_;
    std::vector<std::string> unresolved_meshes_;
    StaticWorldEmitStats static_world_stats_;
    uint64_t steps_taken_ = 0;
    bool faulted_ = false;
};

} // namespace urdf
