// One articulation of a shared MuJoCo scene, behind the portable backend interface.
//
// ⚠ A SEPARATE CLASS, NOT A MODE OF MuJoCoUrdfBackend. Box3D's backend could carry both ownership
// models because every accessor went through one `robot()`; MuJoCo's per-robot backend owns an
// mjModel, an mjData, an mjSpec and a dozen legacy-only cook diagnostics, so branching each
// accessor would be surgery on the working private-world path for no gain. This facade holds no
// solver state at all: it is a scene pointer and a handle.
//
// ⚠ It REFUSES step(), reset() and setStaticWorld(). In a shared scene each of those means "once
// per robot" applied to something the world has exactly one of, and a silent no-op would be worse
// than a throw: the run would look coordinated and integrate N times per tick.
#pragma once

#include "urdf/UrdfRobotBackend.hpp"
#include "urdf/backends/mujoco/MuJoCoPhysicsScene.hpp"

#include <string>

namespace urdf {

class MuJoCoSharedUrdfBackend : public UrdfRobotBackend
{
public:
    /// `scene` must outlive this facade; the world owns both.
    MuJoCoSharedUrdfBackend(MuJoCoPhysicsScene& scene, std::string stable_id);

    const char* backendName() const override { return "MuJoCo"; }

    void setStaticWorld(std::shared_ptr<const StaticWorld> world) override;
    bool mirrorsStaticWorld() const override { return true; }

    /// ⚠ FALSE, like the per-robot backend. The shared scene has no mocap bodies yet, so a moving
    /// Unreal obstacle is not in this world. Answering true here would make the host register
    /// kinematic bodies that go nowhere and then log that the robot is solid to them.
    bool mirrorsKinematicBodies() const override { return false; }

    /// ⚠ FALSE, unlike the per-robot backend. Its `true` exists so each robot gets a traced floor
    /// under its own spawn; this world's ground is world-level and already emitted by the scene, so
    /// asking for a per-robot one would put a second floor through the map.
    bool needsScaffoldingGroundPlane() const override { return false; }

    int addKinematicBody(const KinematicBody& body) override;
    void setKinematicPose(int handle, const Vec3& position, const Quat& orientation) override;

    /// Registers this articulation with the scene. Nothing is solvable until the scene compiles.
    void buildFromUrdf(const Robot& model, const BackendOptions& opts) override;
    void reset() override;
    int step(double dt) override;

    size_t linkCount() const override;
    size_t jointCount() const override;
    const std::string& linkName(size_t link) const override;
    const std::string& jointName(size_t joint) const override;
    int findLink(const std::string& name) const override;
    int findJoint(const std::string& name) const override;

    LinkPose getLinkPose(size_t link) const override;
    Twist getLinkTwist(size_t link) const override;
    JointState getJointState(size_t joint) const override;
    double totalMass() const override;

    void setJointTarget(size_t joint, ControlMode mode, double value) override;
    void setPositionGains(size_t joint, double hertz, double damping_ratio) override;
    void applyExternalWrench(size_t link, const Wrench& wrench) override;

    const std::string& stableId() const { return stable_id_; }
    bool describeColliders(PhysicsColliderSet& out) const override;

    MuJoCoPhysicsScene::ArticulationHandle articulation() const { return articulation_; }

private:
    MuJoCoPhysicsScene::ArticulationHandle requireArticulation() const;

    MuJoCoPhysicsScene& scene_;
    std::string stable_id_;
    MuJoCoPhysicsScene::ArticulationHandle articulation_;
};

} // namespace urdf
