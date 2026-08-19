// The MuJoCo-backed URDF robot — a SECOND backend alongside Box3D, not a replacement.
//
// Selected per vehicle by settings: "PhysicsEngine": "MuJoCo". Absent or "Box3D" keeps the
// existing engine, and a build without the MuJoCo dependency compiles this out entirely
// (WITH_MUJOCO_BINDING=0). The two engines never run the same robot.
//
// ⚠ WHY MUJOCO IS WORTH A SECOND BACKEND. The hand-written Go2 trot walks on Box3D only at
// kp = 160, against the kp = 20 the same robot is trained at in MuJoCo — 8x stiffer, measured
// 2026-08-18. Running one robot, one controller and one command on both engines behind this
// interface is what turns that into an attributable difference (contact stiffness? friction?
// solver compliance?) rather than an anecdote. See SIMVAL/urdf_physics/NEWTON-ASSESSMENT.md.
//
// ⚠ MODEL CONSTRUCTION TAKES THE URDF **TEXT**, not the parsed Robot. MuJoCo has its own URDF
// reader, so BackendOptions::urdf_xml is handed straight to mj_parseXMLString. Re-serialising our
// parsed model back into XML would create a second, quietly divergent definition of the robot.
// The parsed `Robot` is still used — for joint enumeration and for the actuators URDF cannot
// declare.
//
// ⚠ A URDF DECLARES NO ACTUATORS AND MUJOCO REQUIRES THEM. `xml_urdf.cc` creates none, so nothing
// could be commanded. One torque actuator is added per movable joint through the mjSpec API before
// compilation. Position and velocity control are then computed here as a PD over that torque,
// which keeps the gains explicit rather than hidden inside a solver's actuator model — the same
// reasoning that made client-side PD the right choice for the Go2.
#pragma once

#include "urdf/UrdfRobotBackend.hpp"
#include "urdf/UrdfStaticWorld.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

struct mjModel_;
struct mjData_;
struct mjSpec_;

namespace urdf {

class MuJoCoUrdfBackend : public UrdfRobotBackend
{
public:
    MuJoCoUrdfBackend() = default;
    ~MuJoCoUrdfBackend() override;

    MuJoCoUrdfBackend(const MuJoCoUrdfBackend&) = delete;
    MuJoCoUrdfBackend& operator=(const MuJoCoUrdfBackend&) = delete;

    const char* backendName() const override { return "MuJoCo"; }

    void setStaticWorld(std::shared_ptr<const StaticWorld> world) override;

    /// TRUE since 2026-08-19: setStaticWorld now emits the mirrored level into the spec as convex
    /// geoms, so UrdfBotSimApi may suppress its scaffolding floor for this engine exactly as it
    /// does for Box3D.
    bool mirrorsStaticWorld() const override { return true; }

    /// ⚠ STILL FALSE. addKinematicBody remains a no-op, so a MuJoCo robot collides with the LEVEL
    /// but not with other vehicles — it will drive through a Box3D rover as if it were not there.
    /// Flip this in the same change that makes addKinematicBody real, never before: the flag is
    /// what stops UrdfBotSimApi registering handles that silently do nothing.
    /// TRUE since 2026-08-19: addKinematicBody now creates real MuJoCo mocap bodies, so a
    /// MuJoCo robot is pushed by other vehicles and by movable level objects. Still
    /// ONE-DIRECTIONAL, as KinematicBody documents — they push it, it never pushes them.
    bool mirrorsKinematicBodies() const override { return true; }

    /// True: MuJoCo cannot represent a vast concave ground mesh, so it skips it and
    /// stands on the exactly-traced plane instead. See the base declaration.
    bool needsScaffoldingGroundPlane() const override { return true; }
    int addKinematicBody(const KinematicBody& body) override;
    void setKinematicPose(int handle, const Vec3& position, const Quat& orientation) override;
    void buildFromUrdf(const Robot& model, const BackendOptions& opts) override;
    void reset() override;
    int step(double dt) override;

    size_t linkCount() const override { return links_.size(); }
    size_t jointCount() const override { return joints_.size(); }
    const std::string& linkName(size_t link) const override { return links_.at(link).name; }
    const std::string& jointName(size_t joint) const override { return joints_.at(joint).name; }
    int findLink(const std::string& name) const override;
    int findJoint(const std::string& name) const override;

    LinkPose getLinkPose(size_t link) const override;
    Twist getLinkTwist(size_t link) const override;
    JointState getJointState(size_t joint) const override;
    double totalMass() const override;

    void setJointTarget(size_t joint, ControlMode mode, double value) override;
    void setPositionGains(size_t joint, double hertz, double damping_ratio) override;
    void applyExternalWrench(size_t link, const Wrench& wrench) override;

    /// How many <collision> elements the URDF declared, and how many geoms MuJoCo actually
    /// compiled (excluding our scaffolding ground plane).
    ///
    /// ⚠ Read these. MuJoCo drops collision geometry it cannot load and reports nothing: no
    /// exception, no message, a model that compiles. The two causes are `package://` URIs, which it
    /// has no notion of, and COLLADA meshes, which it does not read — between them that is most
    /// robot URDFs in existence. Measured on the Go2: every <mesh> collision gone, including the
    /// body shell, leaving 27 primitive geoms and a torso that touches nothing.
    size_t collisionGeomsDeclared() const { return collision_geoms_declared_; }
    size_t collisionGeomsRealised() const { return collision_geoms_realised_; }

    /// The mesh collisions that most likely account for a shortfall between the two counts above,
    /// as "link <- filename". Empty when the counts agree, so an intact model reports nothing.
    const std::vector<std::string>& droppedCollisions() const { return dropped_collisions_; }

    /// How much of the mirrored level made it into this robot's world.
    ///
    /// ⚠ Worth logging even when nothing is dropped. Box3D and MuJoCo now both collide with the
    /// level, but they do NOT get identical geometry from it: Box3D cooks concave tri-meshes
    /// directly, whereas MuJoCo can only take convex shapes and therefore gets the CoACD
    /// decomposition of each. Two engines, one level, two collision approximations.
    size_t staticGeomsEmitted() const { return static_geoms_emitted_; }
    size_t staticShapesDropped() const { return static_shapes_dropped_; }

    /// World-body geoms in the COMPILED model, and the bounding box of their positions.
    ///
    /// ⚠ This is the number that matters; staticGeomsEmitted() only says what was handed to the
    /// spec. The bounds are here because "the level compiled" and "the level is where the robot
    /// is" are different claims, and only the second one keeps a robot off the floor.
    size_t staticGeomsCompiled() const { return static_geoms_compiled_; }

    /// Contact bitmasks of the first world geom and the first robot geom. MuJoCo collides a pair
    /// only if (contype1 & conaffinity2) || (contype2 & conaffinity1) — a zero anywhere makes a
    /// geom present but intangible, which is indistinguishable from missing until you look here.
    int staticContype() const { return static_contype_; }
    int staticConaffinity() const { return static_conaffinity_; }
    int robotContype() const { return robot_contype_; }
    int robotConaffinity() const { return robot_conaffinity_; }

    /// Distance straight down from the spawn point to the nearest static geom, and which geom.
    /// Negative means NOTHING is underneath the robot — the single most useful fact when a robot
    /// falls through a level that every other diagnostic says is present.
    double groundProbeDistance() const { return ground_probe_distance_; }
    int groundProbeGeom() const { return ground_probe_geom_; }
    double ceilingProbeDistance() const { return ceiling_probe_distance_; }

    /// Largest absolute vertex coordinate seen in a REJECTED static shape. Non-zero
    /// means the mirror produced corrupt geometry, not merely awkward geometry.
    double staticWorstVertex() const { return static_worst_vertex_; }

    /// Largest span of any single mirrored shape, and which level actor produced it.
    /// Reported ALWAYS, not only on rejection: it is the one number that identifies
    /// which piece of level geometry is wrong.
    double staticWorstSpan() const { return static_worst_span_; }

    /// Mirrored shapes that were vast and flat enough to become mjGEOM_PLANE rather
    /// than a convex mesh — typically the level's ground.
    size_t staticPlanesEmitted() const { return static_planes_emitted_; }
    /// True if the ground came from a sampled height grid rather than the flat plane.
    bool usedHeightField() const { return used_height_field_; }

    /// Write every COMPILED collision geom to a Wavefront OBJ, so what MuJoCo actually simulates
    /// can be looked at directly.
    ///
    /// ⚠ Reads mjModel, not our own bookkeeping. The whole point is to show what the SOLVER has —
    /// after convexification, after decomposition, after any approximation — rather than what we
    /// believe we handed it. Those diverged badly enough today to be worth never assuming again.
    bool writeCollisionObj(const std::string& path) const;

    /// Re-point and re-fill the ground height grid as the robot drives, without
    /// recompiling. Same rows/cols as at build time, or it refuses. False if this
    /// robot has no height field.
    bool updateGroundHeightField(const BackendOptions::HeightField& hf);
    size_t staticShapesClipped() const { return static_shapes_clipped_; }
    /// Shapes skipped for being kilometres across - the level's ground, which the
    /// exactly-traced plane represents instead.
    size_t staticShapesOversize() const { return static_shapes_oversize_; }
    /// Level triangles emitted as thin convex prisms - the exact surface
    /// representation that replaced convex decomposition for static geometry.
    size_t staticTrianglesEmitted() const { return static_triangles_emitted_; }
    /// Triangles dropped because the cap was reached - i.e. level geometry that is
    /// simply NOT THERE for this robot. Non-zero is a tuning signal, not a detail.
    size_t staticTrianglesSkipped() const { return static_triangles_skipped_; }
    /// Level objects that were already convex and went in whole, as one exact geom
    /// each, instead of being split per triangle.
    size_t staticConvexObjects() const { return static_convex_objects_; }
    /// Inward-facing meshes - rooms, tunnels, anything the robot is INSIDE. Always
    /// split, never taken whole.
    size_t staticEnclosures() const { return static_enclosures_; }

    /// Collision geoms attached to a kinematic body in the COMPILED model. Zero means the
    /// body exists and can be posed while contributing nothing to contact — which looks
    /// exactly like the mirror not working.
    int kinematicGeomCount(int handle) const;

    /// mjData mocap index for a kinematic handle, or -1 if the body never made it into
    /// the compiled model. -1 is why a registered body silently fails to push.
    int kinematicMocapId(int handle) const
    {
        return (handle >= 0 && handle < static_cast<int>(kinematic_.size()))
                   ? kinematic_[static_cast<size_t>(handle)].mocapid : -1;
    }
    size_t staticShapesClippedAway() const { return static_shapes_clipped_away_; }
    double planeZ() const { return plane_z_; }
    double planeSpanLoZ() const { return plane_span_lo_z_; }
    const std::string& planeBody() const { return plane_body_; }
    const std::string& staticWorstSpanBody() const { return static_worst_span_body_; }
    void staticGeomBounds(double out_min[3], double out_max[3]) const
    {
        for (int k = 0; k < 3; ++k) { out_min[k] = static_geom_bounds_min_[k];
                                      out_max[k] = static_geom_bounds_max_[k]; }
    }

private:
    /// One mirrored body whose pose is pushed in from outside each step.
    struct KinematicRec {
        Vec3 position;
        Quat orientation;
        int mocapid = -1;   ///< index into mjData::mocap_pos / mocap_quat; -1 until compiled
    };

    void emitMocapBody(const KinematicBody& body, size_t index);
    void resolveMocapIds();
    void recompileIfNeeded();

    std::vector<KinematicRec> kinematic_;
    std::vector<KinematicBody> pending_kinematic_;   ///< registered before the spec existed
    bool needs_recompile_ = false;

    /// One URDF link, and the MuJoCo body it became.
    struct LinkRec {
        std::string name;
        int body = -1;              ///< mjOBJ_BODY id, or -1 if MuJoCo dropped the link
    };

    /// One movable URDF joint: its MuJoCo joint, the actuator we added for it, and its command.
    struct JointRec {
        std::string name;
        int joint = -1;             ///< mjOBJ_JOINT id
        int actuator = -1;          ///< mjOBJ_ACTUATOR id of the torque motor added at build time
        int qposadr = -1;
        int dofadr = -1;
        ControlMode mode = ControlMode::None;
        double target = 0;
        double effort_limit = 0;
        // Position-control gains, held here rather than in the model so setPositionGains can be
        // called at any time without recompiling.
        double kp = 0;
        double kd = 0;
    };

    void destroy();
    /// Evaluate every joint's control law and write d_->ctrl. Called once per internal step,
    /// because a PD term is only meaningful against the state it was computed from.
    void applyControl();

    mjModel_* m_ = nullptr;
    mjData_* d_ = nullptr;
    mjSpec_* spec_ = nullptr;

    std::vector<LinkRec> links_;
    std::vector<JointRec> joints_;
    std::map<std::string, size_t> link_index_;
    std::map<std::string, size_t> joint_index_;

    std::shared_ptr<const StaticWorld> static_world_;

    /// Static geoms actually emitted, and mesh shapes that could not be. Reported for the same
    /// reason as the robot's own dropped collisions: level geometry that quietly did not load is a
    /// robot driving through walls, which reads as a broken backend.
    size_t static_geoms_emitted_ = 0;
    size_t static_shapes_dropped_ = 0;
    size_t static_geoms_compiled_ = 0;
    double static_worst_vertex_ = 0;
    bool used_height_field_ = false;
    size_t static_planes_emitted_ = 0;
    size_t static_shapes_clipped_ = 0;
    size_t static_shapes_oversize_ = 0;
    size_t static_triangles_emitted_ = 0;
    size_t static_triangles_skipped_ = 0;
    size_t static_convex_objects_ = 0;
    size_t static_enclosures_ = 0;
    size_t static_shapes_clipped_away_ = 0;
    double plane_z_ = 0;
    double plane_span_lo_z_ = 0;
    std::string plane_body_;
    double static_worst_span_ = 0;
    std::string static_worst_span_body_;
    double ground_probe_distance_ = -1;
    double ceiling_probe_distance_ = -1;
    int ground_probe_geom_ = -1;
    int static_contype_ = -1, static_conaffinity_ = -1;
    int robot_contype_ = -1, robot_conaffinity_ = -1;
    double static_geom_bounds_min_[3] = { 0, 0, 0 };
    double static_geom_bounds_max_[3] = { 0, 0, 0 };

    size_t collision_geoms_declared_ = 0;
    size_t collision_geoms_realised_ = 0;
    std::vector<std::string> dropped_collisions_;

    BackendOptions opts_;
    double accumulator_ = 0;
    long long steps_taken_ = 0;
};

} // namespace urdf
