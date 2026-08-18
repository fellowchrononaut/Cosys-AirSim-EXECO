#include "urdf/backends/Box3DRobot.hpp"

#include "urdf/UrdfMesh.hpp"

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

/// Stand-in for "no effort ceiling stated in the URDF". Large enough never to bind in practice,
/// finite so it cannot poison the solver the way an infinity would.
constexpr float kUnlimitedEffort = 1.0e6f;

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
    // ⚠ Only the b3 handles are invalidated. The kinematic DESCRIPTIONS and last-pushed poses are
    // kept, because instantiate() recreates the bodies from them after reset()'s rebuild. Clearing
    // this vector here would make a reset silently delete every mirrored obstacle, which looks
    // exactly like a level that never mirrored.
    for (KinematicRec& r : kinematic_) {
        for (b3HullData* h : r.owned_hulls) b3DestroyHull(h);
        r.owned_hulls.clear();
        r.body = b3_nullBodyId;
    }

    if (b3World_IsValid(world_)) b3DestroyWorld(world_);
    world_ = b3_nullWorldId;
    links_.clear();
    joints_.clear();
    mimic_.clear();
    hull_budget_reductions_.clear();
    massless_markers_.clear();
    accumulator_ = 0;
    steps_taken_ = 0;
}

int Box3DRobot::addKinematicBody(const urdf::KinematicBody& body)
{
    const int handle = static_cast<int>(kinematic_.size());

    KinematicRec rec;
    rec.desc = body;
    rec.target.p = toB3Pos(body.position);
    rec.target.q = normalize(b3Quat{ b3Vec3{ static_cast<float>(body.orientation.x),
                                             static_cast<float>(body.orientation.y),
                                             static_cast<float>(body.orientation.z) },
                                     static_cast<float>(body.orientation.w) });
    kinematic_.push_back(std::move(rec));

    // If a world already exists this is a late registration, so create it now. If not, instantiate()
    // will create it — either way the description is what survives, not the b3BodyId.
    if (b3World_IsValid(world_)) {
        KinematicRec& r = kinematic_.back();
        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = b3_kinematicBody;
        bd.position = r.target.p;
        bd.rotation = r.target.q;
        bd.name = r.desc.name.c_str();
        r.body = b3CreateBody(world_, &bd);
        addKinematicShapes(r);
    }

    return handle;
}

void Box3DRobot::setKinematicPose(int handle, const urdf::Vec3& position,
                                  const urdf::Quat& orientation)
{
    if (handle < 0 || static_cast<size_t>(handle) >= kinematic_.size()) return;

    KinematicRec& r = kinematic_[static_cast<size_t>(handle)];
    r.target.p = toB3Pos(position);
    r.target.q = normalize(b3Quat{ b3Vec3{ static_cast<float>(orientation.x),
                                           static_cast<float>(orientation.y),
                                           static_cast<float>(orientation.z) },
                                   static_cast<float>(orientation.w) });
    // Applied in stepOnce, not here: b3Body_SetTargetTransform converts a pose into the VELOCITY
    // needed to reach it in one timestep, so it is meaningful only against the step that follows.
    // Calling it here would set a velocity sized for a step that may not come next, and the body
    // would overshoot or crawl depending on how often the source updates.
}

void Box3DRobot::instantiateKinematic()
{
    for (KinematicRec& r : kinematic_) {
        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = b3_kinematicBody;
        // Recreated at the LAST pushed pose, not the registration pose: a reset in the middle of a
        // run should put mirrored obstacles where they currently are, not where they were at load.
        bd.position = r.target.p;
        bd.rotation = r.target.q;
        bd.name = r.desc.name.c_str();
        r.body = b3CreateBody(world_, &bd);
        addKinematicShapes(r);
    }
}

void Box3DRobot::addKinematicShapes(KinematicRec& r)
{
    b3ShapeDef sd = b3DefaultShapeDef();
    sd.baseMaterial.friction = static_cast<float>(r.desc.friction);
    sd.baseMaterial.restitution = static_cast<float>(r.desc.restitution);
    // A kinematic body is infinitely massive by definition; letting shapes write mass data would
    // turn it into something the solver tries to move.
    sd.updateBodyMass = false;

    for (const urdf::StaticShape& shape : r.desc.shapes) {
        switch (shape.kind) {
        case urdf::StaticShapeKind::Mesh:
            // ⚠ Refused, not silently skipped. Box3D mesh shapes are static-only
            // (loose_ends.md #7), so a mesh here would produce a body that exists and collides with
            // nothing — an obstacle you can see, that the mirror reports, and that the robot drives
            // straight through. The plugin hulls tri-meshes before they reach this point.
            throw std::runtime_error("kinematic body '" + r.desc.name +
                                     "': mesh shapes are static-only in Box3D and cannot be used "
                                     "on a moving body. Supply a convex hull instead.");
        case urdf::StaticShapeKind::Hull: {
            if (shape.points.size() < 4) break;
            std::vector<b3Vec3> pts;
            pts.reserve(shape.points.size());
            for (const urdf::Vec3& v : shape.points) pts.push_back(toB3(v));
            b3HullData* hull = nullptr;
            for (int budget = opts_.max_hull_vertices; budget >= 8 && !hull; budget /= 2)
                hull = b3CreateHull(pts.data(), static_cast<int>(pts.size()), budget);
            if (!hull) break;
            b3CreateHullShape(r.body, &sd, hull);
            b3DestroyHull(hull);
            break;
        }
        case urdf::StaticShapeKind::Sphere: {
            b3Sphere sp;
            sp.center = toB3(shape.center_a);
            sp.radius = static_cast<float>(shape.radius);
            b3CreateSphereShape(r.body, &sd, &sp);
            break;
        }
        case urdf::StaticShapeKind::Capsule: {
            b3Capsule c;
            c.center1 = toB3(shape.center_a);
            c.center2 = toB3(shape.center_b);
            c.radius = static_cast<float>(shape.radius);
            b3CreateCapsuleShape(r.body, &sd, &c);
            break;
        }
        }
    }
}

void Box3DRobot::setStaticWorld(std::shared_ptr<const urdf::StaticWorld> world)
{
    // Cooks here, at set time, rather than lazily inside instantiate(). That is deliberate: set is
    // called from the game thread during vehicle setup, while instantiate() runs on the physics
    // thread — including on every reset. Cooking a level-sized mesh on the physics thread would
    // put tens of milliseconds into the executor loop, which is the budget issue I-R and R14 are
    // about. Attaching, which is what instantiate() does, costs microseconds.
    static_geometry_ = Box3DStaticGeometry::acquire(std::move(world));
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

    // Classify <mimic> before anything is created: a cosmetic coupling means *not* creating a
    // joint, so the decision has to precede instantiation rather than be patched in afterwards.
    mimic_ = urdf::classifyMimicJoints(model_, opts_.mimic);
    urdf::requireMimicSupported(model_, mimic_, opts_.mimic);

    createWorld();
    instantiate();
    // Sized here, after joints exist. build() always starts from defaults; reset() is what carries
    // configuration across, and it does so explicitly rather than by this vector quietly surviving.
    control_.assign(joints_.size(), JointControl{});
    built_ = true;
}

void Box3DRobot::reset()
{
    if (!built_) throw std::runtime_error("reset() before build()");

    // ⚠ Control CONFIGURATION survives the rebuild; control STATE does not.
    //
    // The rebuild is not optional — Box3D has no rollback determinism, so a reproducible reset must
    // destroy and recreate rather than rewrite poses (§6.4). But that also destroys every joint's
    // spring gains and enable flags, and a caller that configured them once before the reset has no
    // way to know they are gone. Making every caller re-configure after every reset is an interface
    // that will be got wrong — and was: the steering springs were lost to AirSim's startup reset and
    // the wheels free-swung for three sessions while driving kept working.
    //
    // So: mode and gains are restored, because they describe how the joint is controlled. Targets
    // are NOT — they are commands, and a robot must not drive off again just because it was reset.
    std::vector<JointControl> saved = control_;

    urdf::Robot m = model_;
    BuildOptions o = opts_;
    build(m, o);

    if (saved.size() == joints_.size()) {
        for (size_t i = 0; i < saved.size(); ++i) {
            saved[i].target = 0;
            control_[i] = saved[i];
        }
        reapplyControl();
    }
}

void Box3DRobot::reapplyControl()
{
    for (size_t i = 0; i < joints_.size(); ++i) {
        const JointControl c = control_[i];
        if (c.gains_set) setPositionGains(i, c.hertz, c.damping_ratio);
        if (c.mode != ControlMode::None) {
            setControlMode(i, c.mode);
            setTarget(i, c.target);
        }
    }
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
        case urdf::GeometryType::Mesh: {
            // ⚠ ONE CONVEX HULL per <mesh>, not the triangles.
            //
            // Not a shortcut that could be improved by trying harder: Box3D's mesh shapes are
            // **static-only** (docs/loose_ends.md #7) and cannot take part in dynamic-vs-dynamic
            // contact, so a moving link can never use its triangles directly whatever we do. The
            // real alternative is convex *decomposition* (CoACD/VHACD) into several hulls, which is
            // a separate dependency and a separate decision.
            //
            // ⚠ So a concave link is simulated as its convex hull and is therefore **fatter than it
            // looks**: a C-shaped bracket collides as a solid block, and a robot with a hollow
            // chassis cannot have anything pass through the hollow. That is a real modelling error,
            // it is invisible in the render, and it is why it is stated here and reported by the
            // audit rather than left for someone to discover.
            const std::string resolved =
                urdf::resolveMeshPath(col.geometry.mesh_filename, opts_.mesh_base_dir,
                                      opts_.mesh_search_paths);
            if (resolved.empty())
                throw std::runtime_error("link '" + link.name + "': <collision> mesh '" +
                                         col.geometry.mesh_filename +
                                         "' could not be found. Set UrdfMeshSearchPaths. Refusing "
                                         "rather than simulating a link with no shape.");

            const urdf::MeshData mesh = urdf::loadMesh(resolved);
            if (mesh.vertices.size() < 4)
                throw std::runtime_error("link '" + link.name + "': mesh '" + resolved +
                                         "' has too few vertices to hull (" +
                                         std::to_string(mesh.vertices.size()) + ").");

            std::vector<b3Vec3> pts;
            pts.reserve(mesh.vertices.size());
            for (const urdf::Vec3& v : mesh.vertices)
                pts.push_back(b3Vec3{ static_cast<float>(v.x * col.geometry.mesh_scale.x),
                                      static_cast<float>(v.y * col.geometry.mesh_scale.y),
                                      static_cast<float>(v.z * col.geometry.mesh_scale.z) });

            // ⚠ Box3D's hull builder has a hard ceiling of **255 half-edges**, which is not the
            // same constraint as the vertex budget and is not checkable in advance: a 64-vertex
            // budget produced 264 half-edges on ExoMy's base_link and was refused outright. So the
            // budget is walked down until one is accepted rather than failing on the first try.
            //
            // Halving is the right ladder because the failure is a *topology* limit: a hull that
            // overflows at 64 vertices is not going to fit by shaving two off.
            b3HullData* hull = nullptr;
            int budget = opts_.max_hull_vertices;
            for (; budget >= 8; budget /= 2) {
                hull = b3CreateHull(pts.data(), static_cast<int>(pts.size()), budget);
                if (hull != nullptr) break;
            }
            if (hull == nullptr)
                throw std::runtime_error("link '" + link.name + "': b3CreateHull rejected mesh '" +
                                         resolved + "' at every vertex budget from " +
                                         std::to_string(opts_.max_hull_vertices) +
                                         " down to 8 (degenerate or coplanar point cloud).");

            // Recorded, not swallowed. A coarser hull is a *different shape* from the one the
            // operator thinks is being simulated, and the difference grows as the budget falls.
            if (budget < opts_.max_hull_vertices)
                hull_budget_reductions_.push_back({ link.name, opts_.max_hull_vertices, budget });

            // b3CreateHullShape and b3CreateTransformedHullShape both CLONE the hull, so ours is
            // freed immediately. (b3CreateMeshShape does not clone — the two differ, and the
            // static-geometry cache depends on that difference.)
            b3CreateTransformedHullShape(body, &sd, hull, xf, b3Vec3_one);
            b3DestroyHull(hull);
            break;
        }
        }
    }
}

void Box3DRobot::classifyMimic()
{
    // Everything at or below a cosmetic <mimic> joint is resolved by forward kinematics: no Box3D
    // joint is created for it and no body is integrated. Doing it this way rather than creating a
    // free body and overwriting its pose is the point — an overwritten body wastes solver work and
    // leaves Box3D's state disagreeing with what is drawn (analysis doc §6.5).
    //
    // The whole subtree, not just the mimicking link: a decoration hanging off a decoration is
    // still a decoration, and leaving its children jointed to a body that never moves would strand
    // them at the origin.
    for (const urdf::MimicClassification& c : mimic_) {
        if (c.role != urdf::MimicRole::Cosmetic) continue;
        const urdf::Joint& j = model_.joints[c.joint];
        for (int li : model_.subtreeLinks(j.child_index)) {
            LinkRec& rec = links_[li];
            rec.kinematic = true;
            const int pj = model_.links[li].parent_joint;
            rec.kinematic_joint = pj;
            rec.kinematic_parent = pj >= 0 ? model_.joints[pj].parent_index : -1;
        }
    }

    classifyMasslessMarkers();
}

void Box3DRobot::classifyMasslessMarkers()
{
    // A link with **no <inertial> and no collision geometry** has no way to acquire mass: the
    // <inertial> fallback is b3Body_ApplyMassFromShapes, and there are no shapes. It becomes a
    // dynamic body of mass zero.
    //
    // ⚠ That does not error. It **silently freezes the whole robot** — a zero-mass dynamic body
    // welded into the chain by a fixed joint stops the island moving, and the rover simply never
    // falls. No exception, no warning, and a physics backend that looks broken. This is the exact
    // plausible-wrong-answer class this workstream exists to catch, and it is not exotic: frame
    // markers (base_footprint, imu_link, camera_link) are in a large fraction of published URDFs.
    //
    // The right treatment is the one already built for cosmetic <mimic>: resolve the pose by
    // forward kinematics and create no dynamics for it. That is also what such a link physically
    // *is* — a named frame rigidly attached to its parent.
    for (size_t li = 0; li < links_.size(); ++li) {
        const urdf::Link& link = model_.links[li];
        if (links_[li].kinematic) continue;                 // already resolved kinematically
        if (link.has_inertial || !link.collisions.empty()) continue;

        const int pj = link.parent_joint;
        if (pj < 0) {
            // The root. Nothing to hang it off, and a massless free root is genuinely ill-posed.
            throw std::runtime_error(
                "root link '" + link.name + "' has neither <inertial> nor <collision>, so it has "
                "no mass and no shape. A massless free root cannot be simulated. Give it an "
                "<inertial>, or enable collision synthesis from <visual>.");
        }

        // ⚠ Only a FIXED joint may be resolved this way. On a revolute or prismatic joint a
        // massless link's motion is genuinely undefined — there is no inertia for the joint to act
        // against — so refusing is the honest answer rather than pinning it to its parent and
        // pretending the joint works.
        if (model_.joints[pj].type != urdf::JointType::Fixed) {
            throw std::runtime_error(
                "link '" + link.name + "' has neither <inertial> nor <collision> but hangs off "
                "non-fixed joint '" + model_.joints[pj].name + "'. A massless link on a movable "
                "joint has undefined dynamics. Give it an <inertial>, or make the joint fixed.");
        }

        LinkRec& rec = links_[li];
        rec.kinematic = true;
        rec.kinematic_joint = pj;
        rec.kinematic_parent = model_.joints[pj].parent_index;
        massless_markers_.push_back(link.name);
    }
}

void Box3DRobot::instantiate()
{
    // Static world geometry. Re-attached here rather than in build() precisely because reset()
    // routes through instantiate() — a level that vanished on the first reset would be a very
    // memorable bug. Re-attaching is cheap (~0.006 ms per cooked mesh) because the cook itself is
    // held by static_geometry_ and survives the world teardown; see Box3DStaticGeometry.
    if (static_geometry_) static_geometry_->attachTo(world_);
    instantiateKinematic();

    // Built from opts_, so reset() — which rebuilds the world — recreates it automatically. A
    // ground plane that vanished on reset would be a memorable bug.
    if (opts_.add_ground_plane) {
        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = b3_staticBody;
        bd.position = toB3Pos(0.0, 0.0, opts_.ground_plane_z - 0.5);
        bd.name = "ground_plane";
        b3BodyId ground = b3CreateBody(world_, &bd);

        b3ShapeDef sd = b3DefaultShapeDef();
        sd.baseMaterial.friction = static_cast<float>(opts_.ground_friction);
        // A thick, wide slab rather than a true half-space: Box3D has no infinite plane, and depth
        // keeps a fast-moving wheel from tunnelling through it.
        const b3BoxHull box = b3MakeBoxHull(50.0f, 50.0f, 0.5f);
        b3CreateHullShape(ground, &sd, &box.base);
    }

    const size_t n_links = model_.links.size();
    links_.resize(n_links);
    classifyMimic();

    // Walk the tree from the root accumulating world transforms. A URDF joint <origin> is the
    // transform from the parent link frame to the joint frame, and the child link frame coincides
    // with the joint frame at zero joint position.
    // ⚠ WorldPose, not b3Transform. b3Transform::p is float in BOTH precision modes, so composing
    // world poses through it silently discards BOX3D_DOUBLE_PRECISION (which AirLib pins ON). That
    // was survivable while every robot was built at the origin and coordinates stayed around a
    // metre. It is not survivable now: the root is placed at its true position in the level, so a
    // robot 2 km from the map origin would have its links composed at ~0.1 mm resolution and its
    // joint frames would visibly disagree with each other.
    std::vector<WorldPose> world_xf(n_links);
    std::function<void(int, const WorldPose&)> place = [&](int li, const WorldPose& parent_xf) {
        world_xf[li] = parent_xf;
        for (int ji : model_.links[li].child_joints) {
            const urdf::Joint& j = model_.joints[ji];
            place(j.child_index, mulW(parent_xf, transformFromOrigin(j.origin)));
        }
    };

    WorldPose root;
    root.p = toB3Pos(opts_.root_position);
    root.q = normalize(b3Quat{ b3Vec3{ static_cast<float>(opts_.root_orientation.x),
                                       static_cast<float>(opts_.root_orientation.y),
                                       static_cast<float>(opts_.root_orientation.z) },
                               static_cast<float>(opts_.root_orientation.w) });
    place(model_.root_link, root);

    for (size_t i = 0; i < n_links; ++i) {
        const urdf::Link& link = model_.links[i];
        LinkRec& rec = links_[i];
        rec.name = link.name;
        rec.initial = world_xf[i];

        const bool is_static_root = opts_.fixed_base && static_cast<int>(i) == model_.root_link;

        b3BodyDef bd = b3DefaultBodyDef();
        bd.type = is_static_root  ? b3_staticBody
                  : rec.kinematic ? b3_kinematicBody
                                  : b3_dynamicBody;
        bd.position = world_xf[i].p;
        bd.rotation = world_xf[i].q;
        bd.name = rec.name.c_str();
        // A robot link that falls asleep stops reporting joint state, which reads as a frozen
        // robot. Sleep is a game optimisation we do not want here.
        bd.enableSleep = false;

        rec.body = b3CreateBody(world_, &bd);
        addCollisionShapes(link, rec, rec.body);
        // Mass properties are meaningless on static and kinematic bodies; Box3D ignores them there.
        if (!is_static_root && !rec.kinematic) applyInertial(link, rec.body);
    }

    joints_.reserve(model_.joints.size());
    for (size_t ji = 0; ji < model_.joints.size(); ++ji) {
        const urdf::Joint& j = model_.joints[ji];
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

        if (j.hasMimic()) {
            rec.mimic_source = model_.findJoint(j.mimic_source_joint);
            rec.mimic_multiplier = j.mimic_multiplier;
            rec.mimic_offset = j.mimic_offset;
            for (const urdf::MimicClassification& c : mimic_)
                if (c.joint == static_cast<int>(ji)) rec.mimic_role = c.role;
        }

        // A cosmetic mimic, and everything below it, gets no Box3D joint at all — that is the
        // whole point of the classification. The link's pose is composed in linkState() instead.
        if (links_[j.child_index].kinematic) {
            joints_.push_back(std::move(rec));
            continue;
        }

        // See Box3DMath.hpp::revoluteAxisFrame for the derivation of these two frames, and for why the target
        // axis differs by joint type: revolute rotates about local +Z, prismatic slides along
        // local +X.
        const bool slides = (j.type == urdf::JointType::Prismatic);
        b3Transform axis_only;
        axis_only.p = b3Vec3_zero;
        axis_only.q = slides ? prismaticAxisFrame(j.axis) : revoluteAxisFrame(j.axis);

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
            //
            // ⚠ An absent or zero effort means UNLIMITED, not zero. URDF requires <limit> only for
            // revolute and prismatic joints, so a `continuous` joint — every wheel on every rover —
            // legitimately carries no effort figure. Taking that as maxMotorTorque = 0 gives a
            // motor that can apply no torque, and velocity control silently does nothing. That is
            // exactly how the demo rover failed to drive.
            def.maxMotorTorque = (j.limit.effort > 0.0) ? static_cast<float>(j.limit.effort)
                                                        : kUnlimitedEffort;
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
            // Same as revolute: effort is a ceiling, not a command, and absent means unlimited.
            def.maxMotorForce = (j.limit.effort > 0.0) ? static_cast<float>(j.limit.effort)
                                                       : kUnlimitedEffort;
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
        const bool load_bearing_mimic = rec.mimic_role == urdf::MimicRole::LoadBearing;
        joints_.push_back(std::move(rec));

        // A load-bearing mimic keeps its joint — it has to, because it carries force — and is
        // driven by a servo-follower: position control retargeted from the source joint every
        // step. Approximate by construction, which is why reaching this line at all required an
        // explicit opt-in back in build().
        if (load_bearing_mimic) {
            const size_t idx = joints_.size() - 1;
            setControlMode(idx, ControlMode::Position);
            setPositionGains(idx, opts_.mimic.follower_hertz, opts_.mimic.follower_damping_ratio);
        }
    }
}

int Box3DRobot::findJoint(const std::string& name) const
{
    for (size_t i = 0; i < joints_.size(); ++i)
        if (joints_[i].name == name) return static_cast<int>(i);
    return -1;
}

double Box3DRobot::mimicPosition(size_t joint) const
{
    const JointRec& r = joints_[joint];
    if (r.mimic_source < 0) return jointState(joint).position;
    // Recursion terminates because the parser rejects mimic cycles (resolveMimic).
    const double q_src = mimicPosition(static_cast<size_t>(r.mimic_source));
    return r.mimic_multiplier * q_src + r.mimic_offset;
}

double Box3DRobot::mimicVelocity(size_t joint) const
{
    const JointRec& r = joints_[joint];
    if (r.mimic_source < 0) return jointState(joint).velocity;
    // d/dt (m*q + c) = m*q̇ — the offset differentiates away.
    return r.mimic_multiplier * mimicVelocity(static_cast<size_t>(r.mimic_source));
}

WorldPose Box3DRobot::kinematicTransform(size_t link) const
{
    const LinkRec& rec = links_[link];
    if (!rec.kinematic || rec.kinematic_parent < 0 || rec.kinematic_joint < 0) {
        // ⚠ WorldPose, not b3Transform: b3Transform::p is float in both precision modes, while
        // b3Body_GetPosition returns a double b3Pos under BOX3D_DOUBLE_PRECISION (which AirLib
        // pins on). Going through b3Transform here would truncate world coordinates to float.
        WorldPose xf;
        xf.p = b3Body_GetPosition(rec.body);
        xf.q = b3Body_GetRotation(rec.body);
        return xf;
    }

    // URDF composition: child frame = parent frame ∘ <origin> ∘ joint displacement about <axis>.
    // The parent may itself be kinematic, so this recurses up to the first solver-owned link.
    const WorldPose parent = kinematicTransform(static_cast<size_t>(rec.kinematic_parent));
    const JointRec& jr = joints_[rec.kinematic_joint];
    const urdf::Joint& j = model_.joints[rec.kinematic_joint];

    // A joint inside a cosmetic subtree that is not itself a mimic has no solver state to read, so
    // it sits at its commanded target — which is 0 unless something set one.
    const double q = jr.mimic_source >= 0 ? mimicPosition(static_cast<size_t>(rec.kinematic_joint))
                                          : jr.target;

    b3Transform motion = b3Transform_identity;
    switch (jr.type) {
    case urdf::JointType::Revolute:
    case urdf::JointType::Continuous: {
        const double half = 0.5 * q;
        const double s = std::sin(half);
        const b3Vec3 a = jr.axis_child;
        motion.q = b3Quat{ b3Vec3{ static_cast<float>(a.x * s), static_cast<float>(a.y * s),
                                   static_cast<float>(a.z * s) },
                           static_cast<float>(std::cos(half)) };
        break;
    }
    case urdf::JointType::Prismatic: {
        const b3Vec3 a = jr.axis_child;
        motion.p = b3Vec3{ static_cast<float>(a.x * q), static_cast<float>(a.y * q),
                           static_cast<float>(a.z * q) };
        break;
    }
    default:
        break;  // fixed / floating / planar contribute no scalar displacement
    }

    // The joint origin and the joint's own displacement are both local, link-sized transforms, so
    // they compose in float; only the world accumulation is done in double.
    return mulW(parent, mulT(transformFromOrigin(j.origin), motion));
}

void Box3DRobot::updateMimicFollowers()
{
    for (size_t i = 0; i < joints_.size(); ++i) {
        JointRec& r = joints_[i];
        if (r.mimic_role != urdf::MimicRole::LoadBearing) continue;
        if (!b3Joint_IsValid(r.joint)) continue;
        setTarget(i, mimicPosition(i));
    }
}

void Box3DRobot::stepOnce()
{
    // Before the solve, so the follower's target is consistent with the state it was read from.
    updateMimicFollowers();
    // Kinematic targets, immediately before the step they are sized for. b3Body_SetTargetTransform
    // converts a pose into the velocity needed to reach it in exactly `timeStep`, so it must be
    // issued against the step that follows it and re-issued every step — a velocity is consumed by
    // the integrator, not held.
    for (KinematicRec& r : kinematic_) {
        if (!b3Body_IsValid(r.body)) continue;
        b3WorldTransform target;
        target.p = r.target.p;
        target.q = r.target.q;
        b3Body_SetTargetTransform(r.body, target, static_cast<float>(opts_.fixed_timestep),
                                  /*wake=*/true);
    }

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
    // ⚠ A kinematically-resolved link (at or below a cosmetic <mimic>) has no Box3D joint, so its
    // body still sits at its build-time pose. Reading it would hand back a decoration frozen at
    // the origin while the robot drove away — a plausible wrong answer of exactly the kind this
    // workstream keeps finding. Resolve it here, in the backend, so every consumer agrees.
    if (links_[i].kinematic) {
        const WorldPose xf = kinematicTransform(i);
        LinkState ks;
        ks.position[0] = xf.p.x; ks.position[1] = xf.p.y; ks.position[2] = xf.p.z;
        ks.orientation[0] = xf.q.v.x; ks.orientation[1] = xf.q.v.y;
        ks.orientation[2] = xf.q.v.z; ks.orientation[3] = xf.q.s;
        // Velocity of a kinematic decoration is not reported: deriving it would mean
        // differentiating a pose the solver never integrated. Zero is the honest answer, and no
        // sensor reads these links.
        return ks;
    }

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

    // A cosmetic mimic has no Box3D joint. Its coordinate is exactly the mimic relation, which is
    // a better answer than the zeros the invalid-joint path below would return.
    if (r.mimic_role == urdf::MimicRole::Cosmetic) {
        s.position = mimicPosition(i);
        s.velocity = mimicVelocity(i);
        return s;  // effort is genuinely zero: nothing drives it
    }

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

double Box3DRobot::kinematicMass() const
{
    double m = 0;
    for (size_t i = 0; i < links_.size(); ++i)
        if (links_[i].kinematic && model_.links[i].has_inertial) m += model_.links[i].inertial.mass;
    return m;
}

void Box3DRobot::applyWrench(size_t link, const double force_world[3],
                             const double torque_world[3], const double point_world[3],
                             bool at_center)
{
    const LinkRec& rec = links_[link];
    // A kinematic decoration has no dynamics to push on, and Box3D would ignore it anyway. Saying
    // so here keeps the no-op deliberate rather than accidental.
    if (rec.kinematic) return;

    const b3Vec3 f{ static_cast<float>(force_world[0]), static_cast<float>(force_world[1]),
                    static_cast<float>(force_world[2]) };
    const b3Vec3 t{ static_cast<float>(torque_world[0]), static_cast<float>(torque_world[1]),
                    static_cast<float>(torque_world[2]) };

    if (at_center) {
        b3Body_ApplyForceToCenter(rec.body, f, true);
    }
    else {
        // toB3Pos, not a braced init: this is a world position, whose precision differs between
        // build modes. See Box3DMath.hpp.
        const b3Pos p = toB3Pos(point_world[0], point_world[1], point_world[2]);
        b3Body_ApplyForce(rec.body, f, p, true);
    }
    b3Body_ApplyTorque(rec.body, t, true);
}

void Box3DRobot::setControlMode(size_t joint, ControlMode mode)
{
    JointRec& r = joints_[joint];
    r.mode = mode;
    if (joint < control_.size()) control_[joint].mode = mode;
    if (!b3Joint_IsValid(r.joint)) return;

    const bool revolute = r.type == urdf::JointType::Revolute ||
                          r.type == urdf::JointType::Continuous;
    const bool prismatic = r.type == urdf::JointType::Prismatic;
    if (!revolute && !prismatic) return;

    // ⚠ Position control is spring ON, motor **OFF**. The spring drives the joint to its target;
    // leaving the motor enabled at its default motorSpeed = 0 turns it into a brake holding zero
    // velocity with `effort` N.m, which the spring cannot overcome. The first version of this code
    // did exactly that and the arm did not move at all — the reported "tracking error" came out as
    // precisely the commanded target, which is what gave it away.
    bool spring = false, motor = false;
    switch (mode) {
    case ControlMode::Position: spring = true;  motor = false; break;
    case ControlMode::Velocity: spring = false; motor = true;  break;
    case ControlMode::Effort:
    case ControlMode::None:     spring = false; motor = false; break;
    }

    if (revolute) {
        b3RevoluteJoint_EnableSpring(r.joint, spring);
        b3RevoluteJoint_EnableMotor(r.joint, motor);
    }
    else {
        // Prismatic used to fall through this function entirely, so position control on a sliding
        // joint silently did nothing. That matters now: a parallel gripper — the canonical
        // load-bearing <mimic> — is prismatic, and its servo-follower runs on this path.
        b3PrismaticJoint_EnableSpring(r.joint, spring);
        b3PrismaticJoint_EnableMotor(r.joint, motor);
    }
}

void Box3DRobot::setPositionGains(size_t joint, double hertz, double damping_ratio)
{
    JointRec& r = joints_[joint];
    if (joint < control_.size()) {
        control_[joint].hertz = hertz;
        control_[joint].damping_ratio = damping_ratio;
        control_[joint].gains_set = true;
    }
    if (!b3Joint_IsValid(r.joint)) return;
    // Box3D's guidance: spring hertz should stay below half the step frequency (Nyquist). At the
    // 3 ms sim step that ceiling is ~166 Hz.
    if (r.type == urdf::JointType::Revolute || r.type == urdf::JointType::Continuous) {
        b3RevoluteJoint_SetSpringHertz(r.joint, static_cast<float>(hertz));
        b3RevoluteJoint_SetSpringDampingRatio(r.joint, static_cast<float>(damping_ratio));
    }
    else if (r.type == urdf::JointType::Prismatic) {
        b3PrismaticJoint_SetSpringHertz(r.joint, static_cast<float>(hertz));
        b3PrismaticJoint_SetSpringDampingRatio(r.joint, static_cast<float>(damping_ratio));
    }
}

void Box3DRobot::setTarget(size_t joint, double value)
{
    JointRec& r = joints_[joint];
    r.target = value;
    if (joint < control_.size()) control_[joint].target = value;
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
