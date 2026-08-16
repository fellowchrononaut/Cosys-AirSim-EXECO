#include "urdf/backends/Box3DUrdfBackend.hpp"

namespace urdf {

void Box3DUrdfBackend::buildFromUrdf(const Robot& model, const BackendOptions& opts)
{
    model_ = model;

    b3urdf::BuildOptions b = tuning_;
    b.fixed_timestep = opts.fixed_timestep;
    b.gravity_z = opts.gravity_z;
    b.fixed_base = opts.fixed_base;
    b.collide_connected = opts.collide_connected;
    b.mimic = opts.mimic;
    b.add_ground_plane = opts.add_ground_plane;
    b.ground_plane_z = opts.ground_plane_z;

    robot_.build(model_, b);
}

void Box3DUrdfBackend::reset()
{
    robot_.reset();
}

int Box3DUrdfBackend::step(double dt)
{
    return robot_.step(dt);
}

int Box3DUrdfBackend::findLink(const std::string& name) const
{
    for (size_t i = 0; i < robot_.linkCount(); ++i)
        if (robot_.linkName(i) == name) return static_cast<int>(i);
    return -1;
}

LinkPose Box3DUrdfBackend::getLinkPose(size_t link) const
{
    const b3urdf::LinkState s = robot_.linkState(link);
    LinkPose p;
    p.position = Vec3{ s.position[0], s.position[1], s.position[2] };
    p.orientation = Quat{ s.orientation[0], s.orientation[1], s.orientation[2], s.orientation[3] };
    return p;
}

Twist Box3DUrdfBackend::getLinkTwist(size_t link) const
{
    const b3urdf::LinkState s = robot_.linkState(link);
    Twist t;
    t.linear = Vec3{ s.linear_velocity[0], s.linear_velocity[1], s.linear_velocity[2] };
    t.angular = Vec3{ s.angular_velocity[0], s.angular_velocity[1], s.angular_velocity[2] };
    return t;
}

JointState Box3DUrdfBackend::getJointState(size_t joint) const
{
    const b3urdf::JointState s = robot_.jointState(joint);
    JointState out;
    out.position = s.position;
    out.velocity = s.velocity;
    out.effort = s.effort;
    return out;
}

double Box3DUrdfBackend::totalMass() const
{
    // The interface's contract is "mass realised by the backend", which must be comparable against
    // the URDF. Box3DRobot::totalMass() reports dynamic bodies only, so the kinematic
    // (cosmetic-mimic) links have to be added back or a caller cross-checking the figure would see
    // a discrepancy with no way to explain it.
    return robot_.totalMass() + robot_.kinematicMass();
}

void Box3DUrdfBackend::setJointTarget(size_t joint, ControlMode mode, double value)
{
    b3urdf::ControlMode m = b3urdf::ControlMode::None;
    switch (mode) {
    case ControlMode::None:     m = b3urdf::ControlMode::None;     break;
    case ControlMode::Position: m = b3urdf::ControlMode::Position; break;
    case ControlMode::Velocity: m = b3urdf::ControlMode::Velocity; break;
    case ControlMode::Effort:   m = b3urdf::ControlMode::Effort;   break;
    }
    robot_.setControlMode(joint, m);
    robot_.setTarget(joint, value);
}

void Box3DUrdfBackend::setPositionGains(size_t joint, double hertz, double damping_ratio)
{
    robot_.setPositionGains(joint, hertz, damping_ratio);
}

void Box3DUrdfBackend::applyExternalWrench(size_t link, const Wrench& w)
{
    const double f[3] = { w.force.x, w.force.y, w.force.z };
    const double t[3] = { w.torque.x, w.torque.y, w.torque.z };
    const double p[3] = { w.point.x, w.point.y, w.point.z };
    robot_.applyWrench(link, f, t, p, w.at_center);
}

} // namespace urdf
