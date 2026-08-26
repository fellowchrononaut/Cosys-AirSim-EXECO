// Box3D behind the UrdfRobotBackend interface.
//
// A thin adapter, on purpose. Box3DRobot is where the engine knowledge lives and is what the
// headless Gate 0/1 harness measures directly; this class exists so the simulator can hold a
// backend without knowing which one it has, and so a second backend can be added without the
// vehicle code changing (analysis doc §6.3).
#pragma once

#include "urdf/UrdfRobotBackend.hpp"
#include "urdf/backends/Box3DPhysicsScene.hpp"
#include "urdf/backends/Box3DRobot.hpp"

namespace urdf {

/// Box3D behind the portable backend interface, in one of two ownership modes.
///
/// PRIVATE (default, unchanged): this object owns a `Box3DRobot` which owns a whole `b3World`. It
/// steps and resets itself, and two robots built this way cannot touch each other.
///
/// SHARED (`attachToScene`): the robot is a participant in a `Box3DPhysicsScene` that owns the one
/// `b3World`. `buildFromUrdf` then only REGISTERS the robot — nothing exists in the solver until
/// the scene builds every participant together — and `step`/`reset` are refused here because
/// calling them once per robot would advance or rebuild the shared world N times.
class Box3DUrdfBackend : public UrdfRobotBackend {
public:
    const char* backendName() const override { return "Box3D"; }

    /// Join a shared scene. Call before `buildFromUrdf`; the scene must outlive this backend.
    void attachToScene(b3urdf::Box3DPhysicsScene* scene);
    bool isShared() const { return scene_ != nullptr; }
    /// This robot's index in the shared scene, valid once `buildFromUrdf` has registered it.
    size_t sceneHandle() const;
    /// True once solver objects exist: after `buildFromUrdf` when private, after the scene's build
    /// when shared. Every state accessor below requires it.
    bool isLive() const;

    void setStaticWorld(std::shared_ptr<const StaticWorld> world) override;
    bool mirrorsStaticWorld() const override { return true; }
    bool mirrorsKinematicBodies() const override { return true; }
    /// False: b3CreateMesh cooks the level's real triangles, so a plane would be a
    /// redundant second floor sitting through the map.
    bool needsScaffoldingGroundPlane() const override { return false; }
    int addKinematicBody(const KinematicBody& body) override;
    void setKinematicPose(int handle, const Vec3& position, const Quat& orientation) override;
    void buildFromUrdf(const Robot& model, const BackendOptions& opts) override;
    void reset() override;
    int step(double dt) override;

    size_t linkCount() const override { return robot().linkCount(); }
    size_t jointCount() const override { return robot().jointCount(); }
    const std::string& linkName(size_t link) const override { return robot().linkName(link); }
    const std::string& jointName(size_t joint) const override { return robot().jointName(joint); }
    int findLink(const std::string& name) const override;
    int findJoint(const std::string& name) const override { return robot().findJoint(name); }

    LinkPose getLinkPose(size_t link) const override;
    Twist getLinkTwist(size_t link) const override;
    JointState getJointState(size_t joint) const override;
    double totalMass() const override;

    void setJointTarget(size_t joint, ControlMode mode, double value) override;
    void setPositionGains(size_t joint, double hertz, double damping_ratio) override;
    void applyExternalWrench(size_t link, const Wrench& wrench) override;
    bool setLinkWorldCollision(size_t link, bool enabled) override;

    // --- Box3D-specific, outside the portable interface by design ---------------------------
    /// Engine tuning that has no meaning for another backend: solver substeps, worker count, hull
    /// budgets. Set before buildFromUrdf(); the portable BackendOptions are merged over the top.
    b3urdf::BuildOptions& tuning() { return tuning_; }

    bool collisionDebugGeometry(const CollisionDebugFilter& filter,
                                CollisionDebugSnapshot& out) const override;

    bool describeColliders(PhysicsColliderSet& out) const override;

    const b3urdf::Box3DRobot& robot() const;
    b3urdf::Box3DRobot& robot();

private:
    b3urdf::Box3DRobot owned_robot_;
    /// Null when this backend owns its world. Otherwise the scene that does.
    b3urdf::Box3DPhysicsScene* scene_ = nullptr;
    size_t scene_handle_ = 0;
    bool registered_with_scene_ = false;
    /// Kinematic bodies registered BEFORE this robot joined the shared scene.
    ///
    /// ⚠ The host mirrors the level's moving obstacles while it is still assembling the vehicle,
    /// which is before `buildFromUrdf` has registered anything with the scene. In the private-world
    /// path `Box3DRobot` accepted those and instantiated them at build; a shared-scene robot has no
    /// solver object yet, so they are held here and replayed IN ORDER at registration. Order is
    /// what makes the handle returned early still name the same body afterwards.
    std::vector<KinematicBody> pending_kinematic_;
    b3urdf::BuildOptions tuning_;
    Robot model_;
};

} // namespace urdf
