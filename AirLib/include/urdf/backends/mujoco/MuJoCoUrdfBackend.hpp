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

    /// ⚠ FALSE, and this is what keeps a MuJoCo robot on the ground. setStaticWorld below is a
    /// no-op, so UrdfBotSimApi must NOT suppress its scaffolding floor for this engine the way it
    /// does for Box3D. Flip this to true in the same commit that makes setStaticWorld real, never
    /// before.
    bool mirrorsStaticWorld() const override { return false; }
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

private:
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

    size_t collision_geoms_declared_ = 0;
    size_t collision_geoms_realised_ = 0;
    std::vector<std::string> dropped_collisions_;

    BackendOptions opts_;
    double accumulator_ = 0;
    long long steps_taken_ = 0;
};

} // namespace urdf
