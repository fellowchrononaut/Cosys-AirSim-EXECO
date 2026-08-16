// Box3D behind the UrdfRobotBackend interface.
//
// A thin adapter, on purpose. Box3DRobot is where the engine knowledge lives and is what the
// headless Gate 0/1 harness measures directly; this class exists so the simulator can hold a
// backend without knowing which one it has, and so a second backend can be added without the
// vehicle code changing (analysis doc §6.3).
#pragma once

#include "urdf/UrdfRobotBackend.hpp"
#include "urdf/backends/Box3DRobot.hpp"

namespace urdf {

class Box3DUrdfBackend : public UrdfRobotBackend {
public:
    const char* backendName() const override { return "Box3D"; }

    void buildFromUrdf(const Robot& model, const BackendOptions& opts) override;
    void reset() override;
    int step(double dt) override;

    size_t linkCount() const override { return robot_.linkCount(); }
    size_t jointCount() const override { return robot_.jointCount(); }
    const std::string& linkName(size_t link) const override { return robot_.linkName(link); }
    const std::string& jointName(size_t joint) const override { return robot_.jointName(joint); }
    int findLink(const std::string& name) const override;
    int findJoint(const std::string& name) const override { return robot_.findJoint(name); }

    LinkPose getLinkPose(size_t link) const override;
    Twist getLinkTwist(size_t link) const override;
    JointState getJointState(size_t joint) const override;
    double totalMass() const override;

    void setJointTarget(size_t joint, ControlMode mode, double value) override;
    void setPositionGains(size_t joint, double hertz, double damping_ratio) override;
    void applyExternalWrench(size_t link, const Wrench& wrench) override;

    // --- Box3D-specific, outside the portable interface by design ---------------------------
    /// Engine tuning that has no meaning for another backend: solver substeps, worker count, hull
    /// budgets. Set before buildFromUrdf(); the portable BackendOptions are merged over the top.
    b3urdf::BuildOptions& tuning() { return tuning_; }

    const b3urdf::Box3DRobot& robot() const { return robot_; }
    b3urdf::Box3DRobot& robot() { return robot_; }

private:
    b3urdf::Box3DRobot robot_;
    b3urdf::BuildOptions tuning_;
    Robot model_;
};

} // namespace urdf
