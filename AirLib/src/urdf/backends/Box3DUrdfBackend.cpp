#include "urdf/backends/Box3DUrdfBackend.hpp"
#include "urdf/backends/Box3DCollisionReadback.hpp"

#include <stdexcept>

namespace urdf {

void Box3DUrdfBackend::attachToScene(b3urdf::Box3DPhysicsScene* scene)
{
    if (scene == nullptr)
        throw std::invalid_argument("Box3DUrdfBackend::attachToScene received a null scene");
    if (registered_with_scene_ || owned_robot_.isBuilt())
        throw std::logic_error("Box3DUrdfBackend::attachToScene must be called before buildFromUrdf");
    scene_ = scene;
}

size_t Box3DUrdfBackend::sceneHandle() const
{
    if (!registered_with_scene_)
        throw std::logic_error(
            "Box3DUrdfBackend: no shared-scene handle until buildFromUrdf has registered the robot");
    return scene_handle_;
}

bool Box3DUrdfBackend::isLive() const
{
    if (scene_ != nullptr)
        return registered_with_scene_;
    return owned_robot_.isBuilt();
}

b3urdf::Box3DRobot& Box3DUrdfBackend::robot()
{
    return const_cast<b3urdf::Box3DRobot&>(
        static_cast<const Box3DUrdfBackend*>(this)->robot());
}

const b3urdf::Box3DRobot& Box3DUrdfBackend::robot() const
{
    if (scene_ == nullptr)
        return owned_robot_;
    if (!registered_with_scene_)
        throw std::logic_error(
            "Box3DUrdfBackend: this robot has not been registered with its shared scene yet");
    return scene_->robot(scene_handle_);
}

void Box3DUrdfBackend::setStaticWorld(std::shared_ptr<const StaticWorld> world)
{
    if (scene_ != nullptr)
        throw std::logic_error(
            "Box3DUrdfBackend: a shared scene has exactly one static world; set it with "
            "Box3DPhysicsScene::setStaticWorld rather than once per robot");
    owned_robot_.setStaticWorld(std::move(world));
}

int Box3DUrdfBackend::addKinematicBody(const KinematicBody& body)
{
    if (scene_ != nullptr && !registered_with_scene_) {
        pending_kinematic_.push_back(body);
        return static_cast<int>(pending_kinematic_.size()) - 1;
    }
    return robot().addKinematicBody(body);
}

void Box3DUrdfBackend::setKinematicPose(int handle, const Vec3& position, const Quat& orientation)
{
    if (scene_ != nullptr && !registered_with_scene_) {
        // Update the queued description rather than dropping the pose: a body pushed before the
        // scene existed must still be instantiated where the host last said it was.
        if (handle < 0 || static_cast<size_t>(handle) >= pending_kinematic_.size())
            throw std::out_of_range("Box3DUrdfBackend: unknown pending kinematic body handle");
        pending_kinematic_[static_cast<size_t>(handle)].position = position;
        pending_kinematic_[static_cast<size_t>(handle)].orientation = orientation;
        return;
    }
    robot().setKinematicPose(handle, position, orientation);
}

void Box3DUrdfBackend::buildFromUrdf(const Robot& model, const BackendOptions& opts)
{
    model_ = model;

    b3urdf::BuildOptions b = tuning_;
    b.mesh_base_dir = opts.mesh_base_dir;
    b.mesh_search_paths = opts.mesh_search_paths;
    b.root_position = opts.root_position;
    b.root_orientation = opts.root_orientation;
    b.fixed_timestep = opts.fixed_timestep;
    b.gravity_z = opts.gravity_z;
    b.fixed_base = opts.fixed_base;
    b.collide_connected = opts.collide_connected;
    b.mimic = opts.mimic;
    b.decomposition = opts.decomposition;
    b.add_ground_plane = opts.add_ground_plane;
    b.ground_plane_z = opts.ground_plane_z;

    if (scene_ != nullptr) {
        if (registered_with_scene_)
            throw std::logic_error(
                "Box3DUrdfBackend: a shared-scene robot is registered once; rebuild the whole "
                "Box3DPhysicsScene instead of rebuilding one participant");
        // Scene-wide values are the scene's, and a per-robot scaffolding slab would become one
        // floor per robot in a world that has exactly one.
        b.add_ground_plane = false;
        scene_handle_ = scene_->addRobot(model_, b);
        registered_with_scene_ = true;

        // Replay in registration order, so a handle handed out before the build still names the
        // same body. Box3DRobot returns sequential indices, so these come back identical.
        for (size_t index = 0; index < pending_kinematic_.size(); ++index) {
            const int replayed = robot().addKinematicBody(pending_kinematic_[index]);
            if (replayed != static_cast<int>(index))
                throw std::logic_error(
                    "Box3DUrdfBackend: replaying a queued kinematic body changed its handle");
        }
        pending_kinematic_.clear();
        return;
    }

    owned_robot_.build(model_, b);
}

void Box3DUrdfBackend::reset()
{
    if (scene_ != nullptr)
        throw std::logic_error(
            "Box3DUrdfBackend: a shared-scene robot cannot reset alone; the global reset "
            "transaction rebuilds every participant of the shared b3World together");
    owned_robot_.reset();
}

int Box3DUrdfBackend::step(double dt)
{
    if (scene_ != nullptr)
        throw std::logic_error(
            "Box3DUrdfBackend: a shared-scene robot cannot step alone; stepping once per robot "
            "would advance the shared b3World once per robot");
    return owned_robot_.step(dt);
}

int Box3DUrdfBackend::findLink(const std::string& name) const
{
    for (size_t i = 0; i < robot().linkCount(); ++i)
        if (robot().linkName(i) == name) return static_cast<int>(i);
    return -1;
}

LinkPose Box3DUrdfBackend::getLinkPose(size_t link) const
{
    const b3urdf::LinkState s = robot().linkState(link);
    LinkPose p;
    p.position = Vec3{ s.position[0], s.position[1], s.position[2] };
    p.orientation = Quat{ s.orientation[0], s.orientation[1], s.orientation[2], s.orientation[3] };
    return p;
}

Twist Box3DUrdfBackend::getLinkTwist(size_t link) const
{
    const b3urdf::LinkState s = robot().linkState(link);
    Twist t;
    t.linear = Vec3{ s.linear_velocity[0], s.linear_velocity[1], s.linear_velocity[2] };
    t.angular = Vec3{ s.angular_velocity[0], s.angular_velocity[1], s.angular_velocity[2] };
    return t;
}

JointState Box3DUrdfBackend::getJointState(size_t joint) const
{
    const b3urdf::JointState s = robot().jointState(joint);
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
    return robot().totalMass() + robot().kinematicMass();
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
    robot().setControlMode(joint, m);
    robot().setTarget(joint, value);
}

void Box3DUrdfBackend::setPositionGains(size_t joint, double hertz, double damping_ratio)
{
    robot().setPositionGains(joint, hertz, damping_ratio);
}

void Box3DUrdfBackend::applyExternalWrench(size_t link, const Wrench& w)
{
    const double f[3] = { w.force.x, w.force.y, w.force.z };
    const double t[3] = { w.torque.x, w.torque.y, w.torque.z };
    const double p[3] = { w.point.x, w.point.y, w.point.z };
    robot().applyWrench(link, f, t, p, w.at_center);
}

bool Box3DUrdfBackend::collisionDebugGeometry(const CollisionDebugFilter& filter,
                                              CollisionDebugSnapshot& out) const
{
    out = CollisionDebugSnapshot();
    out.backend = "box3d";

    // ⚠ A SHARED ROBOT DEFERS TO ITS SCENE, and reports only ITSELF. Reading the whole shared world
    // through one robot's facade would draw every robot's geometry once per robot - N copies of the
    // same wireframe, which looks like nothing at all until the frame rate collapses. The scene's
    // own accessor is what a caller that wants the whole world should use.
    if (scene_ != nullptr) {
        if (!registered_with_scene_)
            return false;
        readBox3DRobotCollision(scene_->robot(scene_handle_), std::string(), filter, out);
        return true;
    }

    if (!robot().isBuilt())
        return false;
    readBox3DRobotCollision(robot(), std::string(), filter, out);
    if (filter.include_world && robot().staticGeometry() != nullptr)
        readBox3DStaticWorld(robot().staticGeometry()->source(), filter, out);
    return true;
}

bool Box3DUrdfBackend::describeColliders(PhysicsColliderSet& out) const
{
    out = PhysicsColliderSet();
    if (scene_ != nullptr) {
        if (!registered_with_scene_)
            return false;
        // ⚠ Unqualified ON PURPOSE. This facade describes one robot and does not know the
        // coordinator's stable id for it; the sim api that does qualifies these ids with the
        // vehicle name, exactly as it does for the debug overlay. Inventing a prefix here would
        // produce a second, competing naming scheme for the same links.
        describeBox3DColliders(scene_->robot(scene_handle_), std::string(), out);
        return true;
    }
    if (!robot().isBuilt())
        return false;
    describeBox3DColliders(robot(), std::string(), out);
    return true;
}

} // namespace urdf
