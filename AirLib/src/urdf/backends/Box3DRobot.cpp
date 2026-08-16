#include "urdf/backends/Box3DRobot.hpp"

#include "urdf/backends/Box3DMath.hpp"

#include <cmath>
#include <cstring>
#include <functional>
#include <stdexcept>

namespace b3urdf {
namespace {

/// Box3D caps revolute limits at +/-0.99*pi. Wider URDF limits cannot be expressed as a joint
/// limit; the caller must clamp commands instead. Reported rather than silently truncated.
constexpr double kRevoluteLimitCap = 0.99 * 3.14159265358979323846;

double dot(const b3Vec3& a, const b3Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

} // namespace

Box3DRobot::~Box3DRobot()
{
    destroyWorld();
}

void Box3DRobot::destroyWorld()
{
    for (auto& l : links_) {
        for (b3HullData* h : l.owned_hulls) b3DestroyHull(h);
        l.owned_hulls.clear();
    }
    if (b3World_IsValid(world_)) b3DestroyWorld(world_);
    world_ = b3_nullWorldId;
    links_.clear();
    joints_.clear();
    accumulator_ = 0;
    steps_taken_ = 0;
}

void Box3DRobot::createWorld()
{
    b3WorldDef def = b3DefaultWorldDef();
    def.gravity = b3Vec3{ 0.0f, 0.0f, static_cast<float>(opts_.gravity_z) };
    def.workerCount = static_cast<uint32_t>(opts_.worker_count);
    world_ = b3CreateWorld(&def);
    if (!b3World_IsValid(world_)) throw std::runtime_error("b3CreateWorld failed");
}

void Box3DRobot::build(const urdf::Robot& model, const BuildOptions& opts)
{
    destroyWorld();
    model_ = model;
    opts_ = opts;
    if (opts_.substeps < 4)
        throw std::runtime_error("Box3D rejects substeps < 4 for stiff chains; got " +
                                 std::to_string(opts_.substeps));
    createWorld();
    instantiate();
    built_ = true;
}

void Box3DRobot::reset()
{
    if (!built_) throw std::runtime_error("reset() before build()");
    urdf::Robot m = model_;
    BuildOptions o = opts_;
    build(m, o);
}

void Box3DRobot::applyInertial(const urdf::Link& link, b3BodyId body)
{
    if (!link.has_inertial) {
        // No <inertial>: fall back to shape-derived mass. This is what UrdfSim did for *every*
        // link, including ones that did specify an inertia tensor.
        b3Body_ApplyMassFromShapes(body);
        return;
    }

    const urdf::Inertial& in = link.inertial;

    // URDF expresses <inertia> about the centre of mass, in the frame given by <inertial><origin>.
    // Box3D's b3MassData::inertia is "about the shape center of mass" in body-frame axes, so a
    // non-identity origin rotation must be applied to the tensor.
    double rotated[9];
    const b3Quat q = quatFromRpy(in.origin.rpy);
    rotateInertia(in.inertia.data(), q, rotated);

    b3MassData md;
    md.mass = static_cast<float>(in.mass);
    md.center = toB3(in.origin.xyz);
    md.inertia.cx = b3Vec3{ static_cast<float>(rotated[0]), static_cast<float>(rotated[3]),
                            static_cast<float>(rotated[6]) };
    md.inertia.cy = b3Vec3{ static_cast<float>(rotated[1]), static_cast<float>(rotated[4]),
                            static_cast<float>(rotated[7]) };
    md.inertia.cz = b3Vec3{ static_cast<float>(rotated[2]), static_cast<float>(rotated[5]),
                            static_cast<float>(rotated[8]) };
    b3Body_SetMassData(body, md);
}

void Box3DRobot::addCollisionShapes(const urdf::Link& link, LinkRec& rec, b3BodyId body)
{
    for (const urdf::Collision& col : link.collisions) {
        b3ShapeDef sd = b3DefaultShapeDef();
        // Mass comes from <inertial> where present, so shapes must not overwrite it.
        sd.updateBodyMass = false;
        sd.enableContactEvents = true;

        const b3Transform xf = transformFromOrigin(col.origin);

        switch (col.geometry.type) {
        case urdf::GeometryType::Sphere: {
            // Box3D spheres carry a centre, so the collision origin's translation folds in.
            // A rotation on a sphere is meaningless and is ignored.
            b3Sphere s;
            s.center = xf.p;
            s.radius = static_cast<float>(col.geometry.radius);
            b3CreateSphereShape(body, &sd, &s);
            break;
        }
        case urdf::GeometryType::Box: {
            const b3BoxHull box = b3MakeBoxHull(static_cast<float>(col.geometry.box_size.x * 0.5),
                                                static_cast<float>(col.geometry.box_size.y * 0.5),
                                                static_cast<float>(col.geometry.box_size.z * 0.5));
            b3CreateTransformedHullShape(body, &sd, &box.base, xf, b3Vec3_one);
            break;
        }
        case urdf::GeometryType::Cylinder: {
            // ⚠ b3CreateCylinder builds a **Y-aligned** hull spanning [yOffset, yOffset+height]
            // (hull.c:1789). URDF cylinders are **Z-aligned and centred on the origin**. So:
            // offset by -length/2 to centre it, then rotate +Y onto +Z.
            const double h = col.geometry.length;
            b3HullData* cyl = b3CreateCylinder(static_cast<float>(h),
                                               static_cast<float>(col.geometry.radius),
                                               static_cast<float>(-h * 0.5),
                                               opts_.cylinder_sides);
            if (!cyl) throw std::runtime_error("b3CreateCylinder failed for link '" + link.name + "'");
            rec.owned_hulls.push_back(cyl);

            b3Transform y_to_z;
            y_to_z.p = b3Vec3_zero;
            y_to_z.q = quatBetween(b3Vec3_axisY, b3Vec3_axisZ);
            b3CreateTransformedHullShape(body, &sd, cyl, mulT(xf, y_to_z), b3Vec3_one);
            break;
        }
        case urdf::GeometryType::Mesh:
            // Deferred past Gate 1 on purpose. Box3D can build a single convex hull from points
            // (b3CreateHull), and mesh shapes are static-only (docs/loose_ends.md #7) so a moving
            // link could never use the triangles directly anyway. Loading STL/OBJ and, later,
            // convex *decomposition* are Gate 2 work.
            throw std::runtime_error("link '" + link.name + "': <mesh> collision is not supported "
                                     "yet (Gate 1 covers box/cylinder/sphere). Mesh links need a "
                                     "convex hull or decomposition — see analysis doc section 4.5.");
        }
    }
}

void Box3DRobot::instantiate()
{
    const size_t n_links = model_.links.size();
    links_.resize(n_links);

    // Walk the tree from the root accumulating world transforms. A URDF joint <origin> is the
    // transform from the parent link frame to the joint frame, and the child link frame coincides
    // with the joint frame at zero joint position.
    std::vector<b3Transform> world_xf(n_links);
    std::function<void(int, const b3Transform&)> place = [&](int li, const b3Transform& parent_xf) {
        world_xf[li] = parent_xf;
        for (int ji : model_.links[li].child_joints) {
            const urdf::Joint& j = model_.joints[ji];
            place(j.child_index, mulT(parent_xf, transformFromOrigin(j.origin)));
        }
    };
    place(model_.root_link, b3Transform_identity);

    for (size_t i = 0; i < n_links; ++i) {
        const urdf::Link& link = model_.links[i];
        LinkRec& rec = links_[i];
        rec.name = link.name;
        rec.initial = world_xf[i];

        const bool is_static_root = opts_.fixed_base && static_cast<int>(i) == model_.root_link;

        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = is_static_root ? b3_staticBody : b3_dynamicBody;
        bd.position = b3Pos{ world_xf[i].p.x, world_xf[i].p.y, world_xf[i].p.z };
        bd.rotation = world_xf[i].q;
        bd.name = rec.name.c_str();
        // A robot link that falls asleep stops reporting joint state, which reads as a frozen
        // robot. Sleep is a game optimisation we do not want here.
        bd.enableSleep = false;

        rec.body = b3CreateBody(world_, &bd);
        addCollisionShapes(link, rec, rec.body);
        // Mass properties are meaningless on a static body and Box3D ignores them there.
        if (!is_static_root) applyInertial(link, rec.body);
    }

    joints_.reserve(model_.joints.size());
    for (const urdf::Joint& j : model_.joints) {
        // ⚠ Refuse rather than silently ignore. <dynamics damping="..."/> is viscous joint damping
        // in N.m.s/rad and <dynamics friction="..."/> is static friction in N.m. Box3D has no
        // direct equivalent for either: friction can be emulated with a zero-speed motor capped at
        // the friction torque, and damping needs a per-step torque this backend does not yet
        // apply. Accepting them and doing nothing is precisely the UrdfSim failure this workstream
        // exists to avoid (its <inertia>, <mimic> and <safety_controller> were all parsed and
        // dropped). Implement in Gate 2; refuse until then.
        if (j.dynamics.damping != 0.0 || j.dynamics.friction != 0.0)
            throw std::runtime_error(
                "joint '" + j.name + "': <dynamics damping/friction> is parsed but not yet applied "
                "by this backend, so accepting it would produce a robot that does not match its "
                "URDF. Remove it, or implement joint damping (Gate 2).");

        JointRec rec;
        rec.name = j.name;
        rec.type = j.type;
        rec.parent_link = j.parent_index;
        rec.child_link = j.child_index;
        rec.axis_child = normalized(j.axis);
        rec.effort_limit = j.limit.effort;
        rec.velocity_limit = j.limit.velocity;

        // See b3_math.h::axisToZ for the derivation of these two frames, and for why the target
        // axis differs by joint type: revolute rotates about local +Z, prismatic slides along
        // local +X.
        const bool slides = (j.type == urdf::JointType::Prismatic);
        b3Transform axis_only;
        axis_only.p = b3Vec3_zero;
        axis_only.q = slides ? axisToX(j.axis) : axisToZ(j.axis);

        const b3Transform frame_a = mulT(transformFromOrigin(j.origin), axis_only);
        const b3Transform frame_b = axis_only;

        b3JointDef base = {};

        switch (j.type) {
        case urdf::JointType::Revolute:
        case urdf::JointType::Continuous: {
            b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
            def.base.bodyIdA = links_[j.parent_index].body;
            def.base.bodyIdB = links_[j.child_index].body;
            def.base.localFrameA = frame_a;
            def.base.localFrameB = frame_b;
            def.base.collideConnected = opts_.collide_connected;

            if (j.type == urdf::JointType::Revolute && j.limit.present) {
                if (j.limit.lower < -kRevoluteLimitCap || j.limit.upper > kRevoluteLimitCap) {
                    // Recorded rather than silently clamped: the caller must clamp commands.
                    def.enableLimit = false;
                }
                else {
                    def.enableLimit = true;
                    def.lowerAngle = static_cast<float>(j.limit.lower);
                    def.upperAngle = static_cast<float>(j.limit.upper);
                }
            }
            // ⚠ URDF <limit effort=...> is a CEILING, not a command. Enabling the motor here with
            // motorSpeed = 0 would turn every limited joint into a brake holding position with
            // `effort` N.m — a robot that silently refuses to move under gravity. The motor is
            // therefore left off; setControlMode() turns it on and setTarget() drives it, and the
            // effort figure is retained as the cap applied at that point.
            def.maxMotorTorque = static_cast<float>(j.limit.effort);
            def.enableMotor = false;
            rec.joint = b3CreateRevoluteJoint(world_, &def);
            break;
        }
        case urdf::JointType::Prismatic: {
            b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
            def.base.bodyIdA = links_[j.parent_index].body;
            def.base.bodyIdB = links_[j.child_index].body;
            def.base.localFrameA = frame_a;
            def.base.localFrameB = frame_b;
            def.base.collideConnected = opts_.collide_connected;
            if (j.limit.present) {
                def.enableLimit = true;
                def.lowerTranslation = static_cast<float>(j.limit.lower);
                def.upperTranslation = static_cast<float>(j.limit.upper);
            }
            // Same as revolute: effort is a ceiling, not a command. See above.
            def.maxMotorForce = static_cast<float>(j.limit.effort);
            def.enableMotor = false;
            rec.joint = b3CreatePrismaticJoint(world_, &def);
            break;
        }
        case urdf::JointType::Fixed: {
            // Welding is correct but not optimal: merging fixed-joined links into one body would
            // remove a constraint and sidestep the mass-ratio stretching of loose_ends.md #1.
            // Merging is a Gate 2 optimisation; welding keeps the link-to-body mapping 1:1, which
            // keeps Gate 1's measurements easy to attribute.
            b3WeldJointDef def = b3DefaultWeldJointDef();
            def.base.bodyIdA = links_[j.parent_index].body;
            def.base.bodyIdB = links_[j.child_index].body;
            def.base.localFrameA = transformFromOrigin(j.origin);
            def.base.localFrameB = b3Transform_identity;
            def.base.collideConnected = opts_.collide_connected;
            rec.joint = b3CreateWeldJoint(world_, &def);
            break;
        }
        case urdf::JointType::Floating:
            // No constraint at all: the child is a free body. Nothing to create.
            break;
        case urdf::JointType::Planar: {
            // Motion confined to the plane whose normal is the axis. Box3D expresses this with
            // per-axis motion locks on the body rather than a joint.
            b3MotionLocks locks{};
            const b3Vec3 a = rec.axis_child;
            locks.linearX = std::fabs(a.x) > 0.9f;
            locks.linearY = std::fabs(a.y) > 0.9f;
            locks.linearZ = std::fabs(a.z) > 0.9f;
            if (!(locks.linearX || locks.linearY || locks.linearZ))
                throw std::runtime_error("joint '" + j.name +
                                         "': planar joints need an axis aligned with X, Y or Z");
            b3Body_SetMotionLocks(links_[j.child_index].body, locks);
            break;
        }
        }

        (void)base;
        joints_.push_back(std::move(rec));
    }
}

int Box3DRobot::findJoint(const std::string& name) const
{
    for (size_t i = 0; i < joints_.size(); ++i)
        if (joints_[i].name == name) return static_cast<int>(i);
    return -1;
}

void Box3DRobot::stepOnce()
{
    b3World_Step(world_, static_cast<float>(opts_.fixed_timestep), opts_.substeps);
    ++steps_taken_;
}

int Box3DRobot::step(double dt)
{
    accumulator_ += dt;
    int taken = 0;
    while (accumulator_ >= opts_.fixed_timestep) {
        stepOnce();
        accumulator_ -= opts_.fixed_timestep;
        ++taken;
    }
    return taken;
}

LinkState Box3DRobot::linkState(size_t i) const
{
    LinkState s;
    const b3BodyId b = links_[i].body;
    const b3Pos p = b3Body_GetPosition(b);
    const b3Quat q = b3Body_GetRotation(b);
    const b3Vec3 v = b3Body_GetLinearVelocity(b);
    const b3Vec3 w = b3Body_GetAngularVelocity(b);
    s.position[0] = p.x; s.position[1] = p.y; s.position[2] = p.z;
    s.orientation[0] = q.v.x; s.orientation[1] = q.v.y; s.orientation[2] = q.v.z; s.orientation[3] = q.s;
    s.linear_velocity[0] = v.x; s.linear_velocity[1] = v.y; s.linear_velocity[2] = v.z;
    s.angular_velocity[0] = w.x; s.angular_velocity[1] = w.y; s.angular_velocity[2] = w.z;
    return s;
}

JointState Box3DRobot::jointState(size_t i) const
{
    const JointRec& r = joints_[i];
    JointState s;
    if (!b3Joint_IsValid(r.joint)) return s;

    switch (r.type) {
    case urdf::JointType::Revolute:
    case urdf::JointType::Continuous: {
        s.position = b3RevoluteJoint_GetAngle(r.joint);
        s.effort = b3RevoluteJoint_GetMotorTorque(r.joint);
        // ⚠ Box3D has b3PrismaticJoint_GetSpeed but **no revolute equivalent**. Joint velocity is
        // therefore derived: project the child's angular velocity relative to the parent onto the
        // joint axis, expressed in world coordinates.
        const b3Vec3 wc = b3Body_GetAngularVelocity(links_[r.child_link].body);
        const b3Vec3 wp = b3Body_GetAngularVelocity(links_[r.parent_link].body);
        const b3Vec3 rel{ wc.x - wp.x, wc.y - wp.y, wc.z - wp.z };
        const b3Quat qc = b3Body_GetRotation(links_[r.child_link].body);
        const b3Vec3 axis_world = rotate(qc, r.axis_child);
        s.velocity = dot(rel, axis_world);
        break;
    }
    case urdf::JointType::Prismatic:
        s.position = b3PrismaticJoint_GetTranslation(r.joint);
        s.velocity = b3PrismaticJoint_GetSpeed(r.joint);
        s.effort = b3PrismaticJoint_GetMotorForce(r.joint);
        break;
    default:
        break;  // fixed / floating / planar carry no scalar joint coordinate
    }
    return s;
}

double Box3DRobot::totalMass() const
{
    double m = 0;
    for (const auto& l : links_) m += b3Body_GetMass(l.body);
    return m;
}

void Box3DRobot::setControlMode(size_t joint, ControlMode mode)
{
    JointRec& r = joints_[joint];
    r.mode = mode;
    if (!b3Joint_IsValid(r.joint)) return;
    if (r.type != urdf::JointType::Revolute && r.type != urdf::JointType::Continuous) return;

    switch (mode) {
    case ControlMode::Position:
        // ⚠ Spring ON, motor **OFF**. The spring drives the joint to its target angle; leaving the
        // motor enabled at its default motorSpeed = 0 turns it into a brake holding zero velocity
        // with `effort` N.m, which the spring cannot overcome. The first version of this code did
        // exactly that and the arm did not move at all — the measured "tracking error" came out as
        // precisely the commanded target, which is what gave it away.
        b3RevoluteJoint_EnableSpring(r.joint, true);
        b3RevoluteJoint_EnableMotor(r.joint, false);
        break;
    case ControlMode::Velocity:
        b3RevoluteJoint_EnableSpring(r.joint, false);
        b3RevoluteJoint_EnableMotor(r.joint, true);
        break;
    case ControlMode::Effort:
    case ControlMode::None:
        b3RevoluteJoint_EnableSpring(r.joint, false);
        b3RevoluteJoint_EnableMotor(r.joint, false);
        break;
    }
}

void Box3DRobot::setPositionGains(size_t joint, double hertz, double damping_ratio)
{
    JointRec& r = joints_[joint];
    if (!b3Joint_IsValid(r.joint)) return;
    if (r.type != urdf::JointType::Revolute && r.type != urdf::JointType::Continuous) return;
    // Box3D's guidance: spring hertz should stay below half the step frequency (Nyquist). At the
    // 3 ms sim step that ceiling is ~166 Hz.
    b3RevoluteJoint_SetSpringHertz(r.joint, static_cast<float>(hertz));
    b3RevoluteJoint_SetSpringDampingRatio(r.joint, static_cast<float>(damping_ratio));
}

void Box3DRobot::setTarget(size_t joint, double value)
{
    JointRec& r = joints_[joint];
    r.target = value;
    if (!b3Joint_IsValid(r.joint)) return;

    if (r.type == urdf::JointType::Revolute || r.type == urdf::JointType::Continuous) {
        switch (r.mode) {
        case ControlMode::Position:
            b3RevoluteJoint_SetTargetAngle(r.joint, static_cast<float>(value));
            break;
        case ControlMode::Velocity:
            b3RevoluteJoint_SetMotorSpeed(r.joint, static_cast<float>(value));
            break;
        case ControlMode::Effort:
            // Box3D has no direct torque input on a revolute joint. Effort control is expressed as
            // an unreachable speed capped by the requested torque, which is the standard idiom and
            // the one robox3d uses.
            b3RevoluteJoint_EnableMotor(r.joint, true);
            b3RevoluteJoint_SetMaxMotorTorque(r.joint, static_cast<float>(std::fabs(value)));
            b3RevoluteJoint_SetMotorSpeed(r.joint, value >= 0 ? 1e4f : -1e4f);
            break;
        case ControlMode::None:
            break;
        }
    }
    else if (r.type == urdf::JointType::Prismatic) {
        if (r.mode == ControlMode::Position)
            b3PrismaticJoint_SetTargetTranslation(r.joint, static_cast<float>(value));
        else if (r.mode == ControlMode::Velocity)
            b3PrismaticJoint_SetMotorSpeed(r.joint, static_cast<float>(value));
    }
}

unsigned int Box3DRobot::stateHash() const
{
    unsigned int h = 5381;
    auto fold = [&h](double v) {
        // Fold the float bit pattern, not the double: the underlying state is float in both
        // precision modes except for positions in large-world builds.
        float f = static_cast<float>(v);
        unsigned char buf[sizeof(float)];
        std::memcpy(buf, &f, sizeof(float));
        for (unsigned char c : buf) h = ((h << 5) + h) ^ c;
    };
    for (size_t i = 0; i < links_.size(); ++i) {
        const LinkState s = linkState(i);
        for (int k = 0; k < 3; ++k) fold(s.position[k]);
        for (int k = 0; k < 4; ++k) fold(s.orientation[k]);
        for (int k = 0; k < 3; ++k) fold(s.linear_velocity[k]);
        for (int k = 0; k < 3; ++k) fold(s.angular_velocity[k]);
    }
    return h;
}

} // namespace b3urdf
