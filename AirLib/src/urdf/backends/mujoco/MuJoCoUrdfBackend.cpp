#include "urdf/backends/mujoco/MuJoCoUrdfBackend.hpp"

#include "urdf/backends/mujoco/MuJoCoCollisionReadback.hpp"
#include "urdf/backends/mujoco/MuJoCoStaticWorld.hpp"
#include "urdf/backends/mujoco/MuJoCoUrdfXml.hpp"

#include "urdf/UrdfConvexDecomposition.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <fstream>
#include <cstring>
#include <stdexcept>
#include <string>

namespace urdf {
namespace {

/// Time constant of the inertia-scaled velocity servo, in seconds. Must stay comfortably larger
/// than the solver step; at the nominal 3 ms tick this is ~17 steps.
constexpr double kVelocityTimeConstant = 0.05;

/// MuJoCo reports quaternions as (w, x, y, z); ours are (x, y, z, w).
Quat toQuat(const mjtNum* q)
{
    return Quat{ q[1], q[2], q[3], q[0] };
}

} // namespace

MuJoCoUrdfBackend::~MuJoCoUrdfBackend()
{
    destroy();
}

void MuJoCoUrdfBackend::destroy()
{
    if (d_) { mj_deleteData(d_); d_ = nullptr; }
    if (m_) { mj_deleteModel(m_); m_ = nullptr; }
    if (spec_) { mj_deleteSpec(spec_); spec_ = nullptr; }
}

void MuJoCoUrdfBackend::buildFromUrdf(const Robot& model, const BackendOptions& opts)
{
    destroy();
    opts_ = opts;
    links_.clear(); joints_.clear();
    link_index_.clear(); joint_index_.clear();
    accumulator_ = 0; steps_taken_ = 0;

    // ⚠ The TEXT, not the parsed model. Said plainly rather than guessed at, because the failure
    // otherwise is a robot that loads with no geometry and no obvious reason.
    if (opts.urdf_xml.empty())
        throw std::runtime_error(
            "MuJoCo backend: BackendOptions::urdf_xml is empty. This backend parses URDF text with "
            "MuJoCo's own reader rather than rebuilding the parsed model, so the caller must supply "
            "the original bytes.");

    char err[1024] = { 0 };
    // ⚠ Resolve package:// and relative mesh references BEFORE MuJoCo's reader sees them. It has
    // no notion of ROS packages, and it drops what it cannot open without complaining - a robot
    // that loads, renders, and has no collision geometry at all. See MuJoCoUrdfXml.hpp.
    unresolved_meshes_.clear();
    const std::string resolved_xml = resolveMeshPathsForMuJoCo(
        opts.urdf_xml, opts.mesh_base_dir, opts.mesh_search_paths, &unresolved_meshes_);
    spec_ = mj_parseXMLString(resolved_xml.c_str(), nullptr, err, sizeof(err));
    if (!spec_)
        throw std::runtime_error(std::string("MuJoCo could not parse the URDF: ") + err);

    // ⚠ Where MuJoCo looks for meshes. It resolves <mesh filename> against meshdir, and unlike our
    // own loader it has no package:// notion and no search-path list — so a URDF using package://
    // will not resolve here even though it does for Box3D.
    if (!opts.mesh_base_dir.empty())
        mjs_setString(spec_->compiler.meshdir, opts.mesh_base_dir.c_str());

    // ⚠ FUSESTATIC OFF, and this is not a tuning choice — it is a correctness one that cost two
    // failing checks to find. MuJoCo's URDF importer turns it ON (measured: compiler.fusestatic=1
    // straight out of mj_parseXMLString), and it then DELETES every body that has no degrees of
    // freedom, welding its geoms and mass into the parent. For a fixed-base robot that starts with
    // the root link itself: a two-link URDF compiled to nbody=2 (world + arm), with "base" simply
    // gone. mj_name2id could not find it, and the pawn rendered that link at the world origin
    // while the rest of the robot stood correctly somewhere else.
    //
    // It is not only the root. Any <joint type="fixed"> chain fuses the same way, so IMU frames,
    // sensor mounts and foot frames all vanish — precisely the links a caller is most likely to
    // ask for a pose of.
    //
    // This backend's contract is that link index i is model.links[i] on every engine (see the
    // indexing loop below), and a link that does not exist cannot satisfy it. Fusing is a rendering
    // and performance optimisation for a simulator that owns its own visuals; Unreal owns ours.
    spec_->compiler.fusestatic = 0;

    // --- where the robot is, and whether it can move ------------------------------------------
    //
    // ⚠ MUJOCO'S URDF IMPORTER GIVES A FIXED BASE AND A POSE OF ZERO, and neither default is what
    // a simulator wants. URDF has no way to say "this robot is at (12, -3, 0.4) and free to move";
    // Gazebo supplies that from its world file, and here it comes from the Unreal pawn's spawn
    // transform. Without the two edits below, every MuJoCo robot loads bolted to the world origin —
    // which looks exactly like a broken backend rather than a missing default.
    //
    // Both are mjSpec edits, so both must happen BEFORE mj_compile.
    mjsBody* world = mjs_findBody(spec_, "world");
    mjsBody* root = nullptr;
    if (model.root_link >= 0 && model.root_link < static_cast<int>(model.links.size()))
        root = mjs_findBody(spec_, model.links[model.root_link].name.c_str());

    if (root) {
        root->pos[0] = opts.root_position.x;
        root->pos[1] = opts.root_position.y;
        root->pos[2] = opts.root_position.z;
        // MuJoCo quaternions are (w, x, y, z); ours are (x, y, z, w). Same convention flip as
        // toQuat() above, in the opposite direction.
        root->quat[0] = opts.root_orientation.w;
        root->quat[1] = opts.root_orientation.x;
        root->quat[2] = opts.root_orientation.y;
        root->quat[3] = opts.root_orientation.z;

        // ⚠ A free joint is how MuJoCo spells "floating base" — there is no fixed_base flag. The
        // URDF importer attaches the root rigidly to the world body, and adding this six-DOF joint
        // is what detaches it. opts.fixed_base then needs no code at all: not adding the joint IS
        // the fixed base, which is why the true branch below is empty of anything but a comment.
        if (!opts.fixed_base)
            mjs_addFreeJoint(root);
    }
    else {
        // Named rather than silent: a robot whose root cannot be found gets neither its spawn pose
        // nor its free joint, and will sit at the origin looking like a physics failure.
        throw std::runtime_error(
            "MuJoCo backend: could not find the root link '" +
            (model.root_link >= 0 ? model.links[model.root_link].name : std::string("(none)")) +
            "' in the compiled spec. The robot would load fixed at the world origin.");
    }
    // --- the ground and the mirrored level ----------------------------------------------------
    // ⚠ ONE implementation, shared with the world-scoped MuJoCoPhysicsScene - see
    // MuJoCoStaticWorld.hpp for why this must not become a second copy.
    {
        StaticWorldEmitOptions emit;
        emit.ground_height_field = opts.ground_height_field;
        emit.add_ground_plane = opts.add_ground_plane;
        emit.ground_plane_z = opts.ground_plane_z;
        emit.clip_center = opts.root_position;
        emit.static_world_radius = opts.static_world_radius;
        emit.static_world_max_triangles = opts.static_world_max_triangles;
        emit.static_mesh_mode = opts.static_mesh_mode;
        emit.build_progress = opts.build_progress;

        StaticWorldEmitStats emitted;
        emitStaticWorld(spec_, world, static_world_.get(), emit, emitted);

        static_geoms_emitted_ = emitted.geoms_emitted;
        static_shapes_dropped_ = emitted.shapes_dropped;
        static_triangles_emitted_ = emitted.triangles_emitted;
        static_triangles_skipped_ = emitted.triangles_skipped;
        static_triangles_clipped_away_ = emitted.triangles_clipped_away;
        static_convex_objects_ = emitted.convex_objects;
        static_enclosures_ = emitted.enclosures;
        static_worst_span_ = emitted.worst_span;
        static_worst_span_body_ = emitted.worst_span_body;
        static_worst_vertex_ = emitted.worst_vertex;
        used_height_field_ = emitted.used_height_field;
    }

    // Bodies registered before the spec existed (UrdfBotSimApi registers the level's movable
    // objects between setStaticWorld and buildFromUrdf) are emitted now.
    for (size_t i = 0; i < pending_kinematic_.size(); ++i)
        emitMocapBody(pending_kinematic_[i], i);
    pending_kinematic_.clear();

    // --- actuators ---------------------------------------------------------------------------
    // ⚠ A URDF declares none and MuJoCo requires them: without this nothing is commandable, and
    // the robot would load, render and then ignore every control call. One TORQUE motor per movable
    // joint; position and velocity control are computed as a PD over it in applyControl(), so the
    // gains stay visible instead of being buried in an actuator's gain/bias parameters.
    for (const Joint& j : model.joints) {
        if (j.type == JointType::Fixed || j.type == JointType::Floating ||
            j.type == JointType::Planar)
            continue;
        mjsActuator* a = mjs_addActuator(spec_, nullptr);
        mjs_setName(a->element, ("act_" + j.name).c_str());
        a->trntype = mjTRN_JOINT;
        mjs_setString(a->target, j.name.c_str());
        a->gaintype = mjGAIN_FIXED;
        a->biastype = mjBIAS_NONE;
        a->gainprm[0] = 1.0;                 // ctrl is the torque, in N.m
        // URDF <limit effort> is a real actuator ceiling; honour it so a client cannot command a
        // torque no motor could produce. Absent or zero means unlimited.
        if (j.limit.present && j.limit.effort > 0.0) {
            a->forcerange[0] = -j.limit.effort;
            a->forcerange[1] = j.limit.effort;
        }
    }

    m_ = mj_compile(spec_, nullptr);
    if (!m_) {
        const char* e = mjs_getError(spec_);
        throw std::runtime_error(std::string("MuJoCo could not compile the model: ") +
                                 (e ? e : "(no detail)"));
    }
    d_ = mj_makeData(m_);
    if (!d_) throw std::runtime_error("MuJoCo: mj_makeData failed");

    m_->opt.timestep = opts.fixed_timestep;

    // --- index the links and joints we were given, in the ORDER THE URDF DECLARED THEM ----------
    // ⚠ Order matters: every caller addresses links and joints by the index this backend reports,
    // and the Box3D backend indexes by the parsed model's order. A MuJoCo-internal ordering here
    // would make the same index mean two different links depending on the engine.
    for (const Link& l : model.links) {
        LinkRec rec;
        rec.name = l.name;
        rec.body = mj_name2id(m_, mjOBJ_BODY, l.name.c_str());
        link_index_[rec.name] = links_.size();
        links_.push_back(rec);
    }
    for (const Joint& j : model.joints) {
        if (j.type == JointType::Fixed || j.type == JointType::Floating ||
            j.type == JointType::Planar)
            continue;
        JointRec rec;
        rec.name = j.name;
        rec.joint = mj_name2id(m_, mjOBJ_JOINT, j.name.c_str());
        rec.actuator = mj_name2id(m_, mjOBJ_ACTUATOR, ("act_" + j.name).c_str());
        if (rec.joint >= 0) {
            rec.qposadr = m_->jnt_qposadr[rec.joint];
            rec.dofadr = m_->jnt_dofadr[rec.joint];
        }
        rec.effort_limit = j.limit.present ? j.limit.effort : 0.0;
        joint_index_[rec.name] = joints_.size();
        joints_.push_back(rec);
    }

    // ⚠ COUNT WHAT SURVIVED mj_compile, not what was added to the spec. The two are different
    // numbers and only this one describes the world the robot is actually in: geoms are added to
    // the mjSpec, but it is mj_compile that turns them into mjModel geoms, and a spec-time count
    // reporting "172 emitted" while the compiled model holds none is exactly how a robot ends up
    // falling through a level the log says is there.
    {
        size_t world_geoms = 0;
        double lo[3] = { 1e30, 1e30, 1e30 }, hi[3] = { -1e30, -1e30, -1e30 };
        for (int g = 0; g < m_->ngeom; ++g) {
            if (m_->geom_bodyid[g] != 0) continue;   // body 0 is the world
            ++world_geoms;
            // ⚠ pos +/- rbound, not pos. A centroid says where a shape is CENTRED, not what it
            // covers. Reporting centroid bounds is exactly how "the spawn is inside the level"
            // read as true while nothing at all was underneath the robot.
            const double r = m_->geom_rbound[g];
            for (int k = 0; k < 3; ++k) {
                const double v = m_->geom_pos[3 * g + k];
                if (v - r < lo[k]) lo[k] = v - r;
                if (v + r > hi[k]) hi[k] = v + r;
            }
        }
        static_geoms_compiled_ = world_geoms;

        // ⚠ CONTACT FILTERING, captured because geometry that exists in the right place and still
        // does not collide leaves nothing else to look at. MuJoCo pairs two geoms only if
        // (contype1 & conaffinity2) || (contype2 & conaffinity1); a zero on either side is an
        // object that is present, visible to a raycast, and completely intangible.
        static_contype_ = -1; static_conaffinity_ = -1;
        robot_contype_ = -1; robot_conaffinity_ = -1;
        for (int g = 0; g < m_->ngeom; ++g) {
            if (m_->geom_bodyid[g] == 0 && static_contype_ < 0) {
                static_contype_ = m_->geom_contype[g];
                static_conaffinity_ = m_->geom_conaffinity[g];
            }
            if (m_->geom_bodyid[g] != 0 && robot_contype_ < 0) {
                robot_contype_ = m_->geom_contype[g];
                robot_conaffinity_ = m_->geom_conaffinity[g];
            }
        }
        for (int k = 0; k < 3; ++k) {
            static_geom_bounds_min_[k] = world_geoms ? lo[k] : 0.0;
            static_geom_bounds_max_[k] = world_geoms ? hi[k] : 0.0;
        }
    }

    resolveMocapIds();
    for (size_t i = 0; i < kinematic_.size(); ++i)
        setKinematicPose(static_cast<int>(i), kinematic_[i].position, kinematic_[i].orientation);

    // ⚠ ASK THE SOLVER WHETHER THE ROBOT HAS A FLOOR, rather than inferring it from counts and
    // bounding boxes. "172 geoms compiled" and "the spawn is inside their bounds" were both true
    // while the robot fell through the level — neither answers the only question that matters,
    // which is whether anything is directly underneath it. mj_ray does, and it uses MuJoCo's own
    // geometry rather than a reimplementation that could disagree.
    //
    // flg_static must be true or the mirrored level — all of it on the world body — is skipped.
    if (m_ && d_) {
        mj_forward(m_, d_);
        const mjtNum from[3] = { opts.root_position.x, opts.root_position.y,
                                 opts.root_position.z };
        const mjtNum down[3] = { 0, 0, -1 };
        int hit_geom = -1;
        const int exclude = links_.empty() ? -1 : links_[0].body;
        ground_probe_distance_ = mj_ray(m_, d_, from, down, nullptr, /*flg_static=*/1, exclude,
                                        &hit_geom, nullptr);
        ground_probe_geom_ = hit_geom;

        // ⚠ Upward too. "Nothing below" has two very different causes — no floor at this (x, y),
        // or a floor the robot spawned UNDERNEATH — and only a second ray separates them.
        const mjtNum up[3] = { 0, 0, 1 };
        int up_geom = -1;
        ceiling_probe_distance_ = mj_ray(m_, d_, from, up, nullptr, /*flg_static=*/1, exclude,
                                         &up_geom, nullptr);
    }

    // ⚠ Refuse to run with a link that did not survive compilation. Every consumer addresses links
    // by index and would otherwise read the world body's pose — a robot part frozen at the origin,
    // with no error anywhere. fusestatic above is the known cause; this catches the next one.
    {
        std::string missing;
        for (const LinkRec& l : links_)
            if (l.body < 0) missing += (missing.empty() ? "" : ", ") + l.name;
        if (!missing.empty())
            throw std::runtime_error(
                "MuJoCo backend: these URDF links have no body in the compiled model: " + missing +
                ". Their poses would read as the world origin.");
    }

    // ⚠ RECONCILE THE COLLISION GEOMETRY, because MuJoCo drops what it cannot load WITHOUT AN
    // ERROR. Measured on the Go2: 27 collision geoms compiled and nmesh = 0 — every <mesh>
    // collision silently gone, including the one covering the entire body shell. The robot loads,
    // renders correctly in Unreal (which does its own mesh loading) and then walks around with a
    // hole where its torso should be.
    //
    // Two causes, both invisible:
    //   * package:// - MuJoCo has no notion of ROS packages and no mesh_search_paths, so any URDF
    //     written for ROS refers to files it cannot find. Our own loader resolves these.
    //   * .dae - MuJoCo reads STL, OBJ and MSH. COLLADA is not among them, and ours is the format
    //     most robot URDFs actually ship.
    //
    // This is the same failure the R2 audit exists to catch for Box3D (a robot that looks complete
    // and collides with almost nothing), so it gets the same treatment: counted, attributed, and
    // reported to the caller rather than discovered from behaviour.
    dropped_collisions_.clear();
    {
        size_t declared = 0;
        std::vector<std::string> mesh_collisions;
        for (const Link& l : model.links) {
            for (const Collision& c : l.collisions) {
                ++declared;
                if (c.geometry.type == GeometryType::Mesh)
                    mesh_collisions.push_back(l.name + " <- " + c.geometry.mesh_filename);
            }
        }
        // The ground plane is ours, not the URDF's; do not count it against the model.
        collision_geoms_declared_ = declared;
        // ⚠ Count only geoms belonging to the ROBOT's bodies. m_->ngeom now includes the mirrored
        // level, so the old whole-model count reported "178 of 6" for a 6-collision Scout — a
        // reconciliation that cannot spot a dropped mesh because it is dominated by scenery.
        size_t robot_geoms = 0;
        for (int g = 0; g < m_->ngeom; ++g)
            if (m_->geom_bodyid[g] != 0) ++robot_geoms;
        collision_geoms_realised_ = robot_geoms;

        // ⚠ Attributed to meshes only when geoms are ACTUALLY missing. A relative-path .stl loads
        // here perfectly well, and naming every mesh collision as dropped would cry wolf on the
        // models that work. The shortfall is the evidence; the mesh list is the explanation for it.
        if (collision_geoms_realised_ < collision_geoms_declared_)
            dropped_collisions_ = std::move(mesh_collisions);
    }

    // Put the model in a valid state before anyone reads a pose.
    mj_forward(m_, d_);
}

void MuJoCoUrdfBackend::reset()
{
    if (!m_ || !d_) return;
    // ⚠ mj_resetData, not a rebuild. MuJoCo separates the compiled model from the state, so a reset
    // is cheap and cannot perturb the model — unlike Box3D, where reset destroys and recreates the
    // world and the cooked geometry has to be kept alive across it.
    mj_resetData(m_, d_);
    for (JointRec& j : joints_) { j.mode = ControlMode::None; j.target = 0; }
    accumulator_ = 0;
    steps_taken_ = 0;
    mj_forward(m_, d_);
}

void MuJoCoUrdfBackend::applyControl()
{
    if (!m_ || !d_) return;
    for (const JointRec& j : joints_) {
        if (j.actuator < 0 || j.qposadr < 0) continue;
        double tau = 0.0;
        switch (j.mode) {
        case ControlMode::Effort:
            tau = j.target;
            break;
        case ControlMode::Position:
            // PD about the target angle. Computed here so kp and kd are exactly what the caller
            // asked for; MuJoCo's own position actuator would hide them in gainprm/biasprm.
            tau = j.kp * (j.target - d_->qpos[j.qposadr]) - j.kd * d_->qvel[j.dofadr];
            break;
        case ControlMode::Velocity: {
            // ⚠ INERTIA-SCALED - see MuJoCoPhysicsScene::applyControls for the measurement. A
            // hard-coded gain is a torque-per-rad/s and diverges on a low-inertia joint with a
            // large URDF effort limit; the ExoMy's wheels reach 1.2e6 rad/s within a few steps.
            // Fixed in BOTH backends together so the private-world and shared-scene paths remain
            // comparable, which is the entire point of having two.
            const double inertia = m_->dof_M0[j.dofadr] > 0 ? m_->dof_M0[j.dofadr] : 1.0;
            const double gain = j.kd > 0 ? j.kd : inertia / kVelocityTimeConstant;
            tau = gain * (j.target - d_->qvel[j.dofadr]);
            break;
        }
        case ControlMode::None:
            tau = 0.0;
            break;
        }
        // forcerange clamps this too, but clamping here as well keeps getJointState's reported
        // effort honest about what was actually asked for.
        if (j.effort_limit > 0.0) {
            if (tau > j.effort_limit) tau = j.effort_limit;
            else if (tau < -j.effort_limit) tau = -j.effort_limit;
        }
        d_->ctrl[j.actuator] = tau;
    }
}

int MuJoCoUrdfBackend::step(double dt)
{
    // ⚠ At the START of a step, never part-way through one: a recompile reallocates mjModel and
    // mjData, and doing that between substeps would pull the ground out from under the solver.
    recompileIfNeeded();

    if (!m_ || !d_) return 0;
    accumulator_ += dt;
    int taken = 0;
    while (accumulator_ >= opts_.fixed_timestep) {
        // ⚠ Control is recomputed EVERY internal step, not once per outer call. A PD term is only
        // meaningful against the state it was measured from; holding one torque across several
        // steps is exactly what made the Go2's 50 Hz loop pump energy and flip the robot.
        applyControl();
        mj_step(m_, d_);
        accumulator_ -= opts_.fixed_timestep;
        ++taken;
        ++steps_taken_;
    }
    return taken;
}

int MuJoCoUrdfBackend::findLink(const std::string& name) const
{
    auto it = link_index_.find(name);
    return it == link_index_.end() ? -1 : static_cast<int>(it->second);
}

int MuJoCoUrdfBackend::findJoint(const std::string& name) const
{
    auto it = joint_index_.find(name);
    return it == joint_index_.end() ? -1 : static_cast<int>(it->second);
}

LinkPose MuJoCoUrdfBackend::getLinkPose(size_t link) const
{
    LinkPose p;
    const LinkRec& rec = links_.at(link);
    if (!d_ || rec.body < 0) return p;
    p.position = Vec3{ d_->xpos[3 * rec.body], d_->xpos[3 * rec.body + 1],
                       d_->xpos[3 * rec.body + 2] };
    p.orientation = toQuat(&d_->xquat[4 * rec.body]);
    return p;
}

Twist MuJoCoUrdfBackend::getLinkTwist(size_t link) const
{
    Twist t;
    const LinkRec& rec = links_.at(link);
    if (!m_ || !d_ || rec.body < 0) return t;
    // ⚠ mj_objectVelocity with flg_local = 0 gives WORLD-frame velocity, ordered [angular; linear].
    // Reading d->cvel directly would give a spatial velocity about the body frame origin, which is
    // a different quantity and would look almost right.
    mjtNum vel[6] = { 0 };
    mj_objectVelocity(m_, d_, mjOBJ_BODY, rec.body, vel, 0);
    t.angular = Vec3{ vel[0], vel[1], vel[2] };
    t.linear = Vec3{ vel[3], vel[4], vel[5] };
    return t;
}

JointState MuJoCoUrdfBackend::getJointState(size_t joint) const
{
    JointState s;
    const JointRec& rec = joints_.at(joint);
    if (!d_ || rec.qposadr < 0) return s;
    s.position = d_->qpos[rec.qposadr];
    s.velocity = d_->qvel[rec.dofadr];
    // The torque actually applied by this joint's actuator — not the constraint reaction, matching
    // what the Box3D backend reports.
    if (rec.actuator >= 0) s.effort = d_->actuator_force[rec.actuator];
    return s;
}

double MuJoCoUrdfBackend::totalMass() const
{
    if (!m_) return 0.0;
    double sum = 0.0;
    for (int b = 1; b < m_->nbody; ++b) sum += m_->body_mass[b];   // body 0 is the world
    return sum;
}

void MuJoCoUrdfBackend::setJointTarget(size_t joint, ControlMode mode, double value)
{
    JointRec& rec = joints_.at(joint);
    rec.mode = mode;
    rec.target = value;
}

void MuJoCoUrdfBackend::setPositionGains(size_t joint, double hertz, double damping_ratio)
{
    JointRec& rec = joints_.at(joint);
    if (!m_ || rec.dofadr < 0) return;
    // ⚠ hertz -> kp needs an inertia, and this is where the two backends genuinely differ. Box3D's
    // soft constraint is mass-normalised, so `hertz` fixes a natural frequency and the stiffness it
    // yields is m_eff(q)*(2*pi*f)^2 — a Kp that changes as the robot moves (see
    // SIMVAL/urdf_physics/NEXT-SESSION.md §13.5). Here the same conversion is done ONCE against the
    // joint's diagonal inertia, so Kp is constant.
    //
    // That is a deliberate difference, not an oversight: a constant Kp is what a controller is
    // written against. It also means the same `hertz` will not produce identical behaviour on both
    // engines, which is precisely the kind of thing the two-backend comparison exists to expose.
    const double inertia = m_->dof_M0[rec.dofadr] > 0 ? m_->dof_M0[rec.dofadr] : 1.0;
    const double omega = 2.0 * 3.14159265358979323846 * hertz;
    rec.kp = inertia * omega * omega;
    rec.kd = 2.0 * damping_ratio * std::sqrt(rec.kp * inertia);
}

void MuJoCoUrdfBackend::applyExternalWrench(size_t link, const Wrench& wrench)
{
    const LinkRec& rec = links_.at(link);
    if (!d_ || rec.body < 0) return;
    // ⚠ xfrc_applied is in WORLD coordinates and about the body's centre of mass, and MuJoCo clears
    // it on reset but NOT between steps — it persists until overwritten, which is the opposite of
    // Box3D's convention. Callers re-issuing every step therefore behave identically on both.
    d_->xfrc_applied[6 * rec.body + 0] = wrench.force.x;
    d_->xfrc_applied[6 * rec.body + 1] = wrench.force.y;
    d_->xfrc_applied[6 * rec.body + 2] = wrench.force.z;
    d_->xfrc_applied[6 * rec.body + 3] = wrench.torque.x;
    d_->xfrc_applied[6 * rec.body + 4] = wrench.torque.y;
    d_->xfrc_applied[6 * rec.body + 5] = wrench.torque.z;
}

void MuJoCoUrdfBackend::setStaticWorld(std::shared_ptr<const StaticWorld> world)
{
    // ⚠ STORED, NOT APPLIED — the geoms are emitted in buildFromUrdf. MuJoCo has no way to add
    // collision geometry to a compiled mjModel: everything must exist in the mjSpec before
    // mj_compile. UrdfBotSimApi already calls this before buildFromUrdf, which is what makes the
    // ordering work; keeping the dependency explicit here rather than assuming it.
    static_world_ = std::move(world);
}

void MuJoCoUrdfBackend::emitMocapBody(const KinematicBody& body, size_t index)
{
    mjsBody* world = mjs_findBody(spec_, "world");
    if (!world) return;

    // ⚠ A MOCAP BODY is MuJoCo's primitive for "driven from outside the solver": no joints, no
    // integration, pose written straight into mjData each step. That is exactly the contract
    // KinematicBody describes — one-directional, pushes the robot, never pushed back.
    mjsBody* b = mjs_addBody(world, nullptr);
    const std::string name = "kin_" + std::to_string(index);
    mjs_setName(b->element, name.c_str());
    b->mocap = 1;
    b->pos[0] = body.position.x; b->pos[1] = body.position.y; b->pos[2] = body.position.z;
    b->quat[0] = body.orientation.w; b->quat[1] = body.orientation.x;
    b->quat[2] = body.orientation.y; b->quat[3] = body.orientation.z;

    // ⚠ Shapes stay in BODY-LOCAL coordinates here, unlike the static world. A mocap body moves,
    // so baking world coordinates into its vertices — which is right for the level, because the
    // level never moves — would freeze it at its registration pose while mocap_pos moved an empty
    // frame around.
    int shape_index = 0;
    for (const StaticShape& shape : body.shapes) {
        switch (shape.kind) {
        case StaticShapeKind::Sphere: {
            mjsGeom* g = mjs_addGeom(b, nullptr);
            g->type = mjGEOM_SPHERE;
            g->pos[0] = shape.center_a.x; g->pos[1] = shape.center_a.y; g->pos[2] = shape.center_a.z;
            g->size[0] = shape.radius;
            g->friction[0] = body.friction;
            break;
        }
        case StaticShapeKind::Capsule: {
            mjsGeom* g = mjs_addGeom(b, nullptr);
            g->type = mjGEOM_CAPSULE;
            g->fromto[0] = shape.center_a.x; g->fromto[1] = shape.center_a.y;
            g->fromto[2] = shape.center_a.z; g->fromto[3] = shape.center_b.x;
            g->fromto[4] = shape.center_b.y; g->fromto[5] = shape.center_b.z;
            g->size[0] = shape.radius;
            g->friction[0] = body.friction;
            break;
        }
        default: {
            // Hull (and, defensively, Mesh): KinematicBody documents that these are always convex
            // hulls, never concave meshes, because a moving body cannot use Box3D's static mesh
            // path either. So no decomposition is needed — the points go in as they are.
            if (!hasHullableVolume(shape.points)) break;
            std::vector<float> verts;
            verts.reserve(shape.points.size() * 3);
            for (const Vec3& v : shape.points) {
                verts.push_back(static_cast<float>(v.x));
                verts.push_back(static_cast<float>(v.y));
                verts.push_back(static_cast<float>(v.z));
            }
            const std::string mesh_name = name + "_m" + std::to_string(shape_index);
            mjsMesh* mesh = mjs_addMesh(spec_, nullptr);
            mjs_setName(mesh->element, mesh_name.c_str());
            mjs_setFloat(mesh->uservert, verts.data(), static_cast<int>(verts.size()));

            mjsGeom* g = mjs_addGeom(b, nullptr);
            g->type = mjGEOM_MESH;
            mjs_setString(g->meshname, mesh_name.c_str());
            g->friction[0] = body.friction;
            break;
        }
        }
        ++shape_index;
    }
}

int MuJoCoUrdfBackend::addKinematicBody(const KinematicBody& body)
{
    const int handle = static_cast<int>(kinematic_.size());
    KinematicRec rec;
    rec.position = body.position;
    rec.orientation = body.orientation;
    kinematic_.push_back(rec);

    // ⚠ TWO ARRIVAL TIMES, and both must work. Bodies registered before buildFromUrdf are emitted
    // by it. But UrdfBotSimApi ALSO refreshes the mirror after every pawn exists — that is the
    // whole reason other vehicles are visible at all — and by then the model is compiled. MuJoCo
    // cannot add a body to a compiled mjModel, so the spec is edited and a recompile is flagged;
    // mj_recompile preserves state, so the robot does not jump.
    if (spec_) {
        emitMocapBody(body, static_cast<size_t>(handle));
        if (m_) needs_recompile_ = true;
    }
    else {
        pending_kinematic_.push_back(body);
    }
    return handle;
}

void MuJoCoUrdfBackend::setKinematicPose(int handle, const Vec3& position, const Quat& orientation)
{
    if (handle < 0 || handle >= static_cast<int>(kinematic_.size())) return;
    KinematicRec& rec = kinematic_[static_cast<size_t>(handle)];
    rec.position = position;
    rec.orientation = orientation;

    if (!m_ || !d_ || rec.mocapid < 0) return;
    d_->mocap_pos[3 * rec.mocapid + 0] = position.x;
    d_->mocap_pos[3 * rec.mocapid + 1] = position.y;
    d_->mocap_pos[3 * rec.mocapid + 2] = position.z;
    // MuJoCo quaternions are (w, x, y, z); ours are (x, y, z, w).
    d_->mocap_quat[4 * rec.mocapid + 0] = orientation.w;
    d_->mocap_quat[4 * rec.mocapid + 1] = orientation.x;
    d_->mocap_quat[4 * rec.mocapid + 2] = orientation.y;
    d_->mocap_quat[4 * rec.mocapid + 3] = orientation.z;
}

bool MuJoCoUrdfBackend::updateGroundHeightField(const BackendOptions::HeightField& hf)
{
    // ⚠ IN PLACE, NO RECOMPILE. mjModel::hfield_data is a plain float buffer and geom_pos is
    // writable, so re-sampling the ground as the robot drives costs a memcpy rather than a model
    // rebuild. That is what makes a moving region of interest affordable at all — rebuilding the
    // model every time the robot crossed a patch boundary would stall the physics thread.
    //
    // ⚠ The GRID SHAPE may not change, only its contents and where it sits. Rows and columns are
    // baked into the compiled model, so a caller must re-sample at the same resolution. Refusing
    // here rather than writing past the buffer.
    if (!m_ || !d_ || !used_height_field_ || !hf.valid()) return false;

    const int id = mj_name2id(m_, mjOBJ_HFIELD, "ground_hfield");
    if (id < 0) return false;
    if (m_->hfield_nrow[id] != hf.rows || m_->hfield_ncol[id] != hf.cols) return false;

    std::memcpy(m_->hfield_data + m_->hfield_adr[id], hf.heights.data(),
                hf.heights.size() * sizeof(float));

    const int geom_id = mj_name2id(m_, mjOBJ_GEOM, "ground_hfield_geom");
    if (geom_id >= 0) {
        m_->geom_pos[3 * geom_id + 0] = hf.center_x;
        m_->geom_pos[3 * geom_id + 1] = hf.center_y;
        m_->geom_pos[3 * geom_id + 2] = hf.min_z;
    }

    // ⚠ Elevation ceiling too: hfield_size[2] scales the stored heights, so leaving it at the old
    // patch's peak would squash or stretch the new terrain.
    double peak = 0;
    for (float h : hf.heights) peak = std::max(peak, static_cast<double>(h));
    m_->hfield_size[4 * id + 2] = std::max(peak, 0.01);

    mj_forward(m_, d_);
    return true;
}

bool MuJoCoUrdfBackend::writeCollisionObj(const std::string& path) const
{
    if (!m_ || !d_) return false;
    std::ofstream f(path);
    if (!f) return false;

    f << "# MuJoCo COMPILED collision geometry - what the solver actually has.\n";
    f << "# geoms=" << m_->ngeom << "  meshes=" << m_->nmesh << "\n";

    int vbase = 1;   // OBJ indices are 1-based and continue across objects
    for (int g = 0; g < m_->ngeom; ++g) {
        const int type = m_->geom_type[g];
        const char* gname = mj_id2name(m_, mjOBJ_GEOM, g);
        f << "o geom" << g << "_" << (gname ? gname : "unnamed")
          << "_type" << type << "_body" << m_->geom_bodyid[g] << "\n";

        if (type == mjGEOM_MESH) {
            const int mesh = m_->geom_dataid[g];
            if (mesh < 0) continue;
            const int vadr = m_->mesh_vertadr[mesh], vnum = m_->mesh_vertnum[mesh];
            const int fadr = m_->mesh_faceadr[mesh], fnum = m_->mesh_facenum[mesh];

            // ⚠ World placement is geom_xpos/xmat from mjData, not the mesh's own coordinates:
            // MuJoCo recentres meshes at compile time, so the raw vertices are not where the
            // shape sits.
            const mjtNum* p0 = d_->geom_xpos + 3 * g;
            const mjtNum* R = d_->geom_xmat + 9 * g;
            for (int v = 0; v < vnum; ++v) {
                const float* mv = m_->mesh_vert + 3 * (vadr + v);
                const double x = R[0]*mv[0] + R[1]*mv[1] + R[2]*mv[2] + p0[0];
                const double y = R[3]*mv[0] + R[4]*mv[1] + R[5]*mv[2] + p0[1];
                const double z = R[6]*mv[0] + R[7]*mv[1] + R[8]*mv[2] + p0[2];
                f << "v " << x << " " << y << " " << z << "\n";
            }
            for (int t = 0; t < fnum; ++t) {
                const int* fc = m_->mesh_face + 3 * (fadr + t);
                f << "f " << (vbase + fc[0]) << " " << (vbase + fc[1]) << " " << (vbase + fc[2])
                  << "\n";
            }
            vbase += vnum;
        }
        else {
            // Primitives as a marker box at their world pose, so planes, spheres, capsules and
            // height fields still SHOW UP rather than silently missing from the comparison.
            const mjtNum* p0 = d_->geom_xpos + 3 * g;
            const double r = std::max(0.05, static_cast<double>(m_->geom_rbound[g]));
            const double c[8][3] = {{-r,-r,-r},{r,-r,-r},{r,r,-r},{-r,r,-r},
                                    {-r,-r, r},{r,-r, r},{r,r, r},{-r,r, r}};
            for (const auto& q : c)
                f << "v " << (p0[0]+q[0]) << " " << (p0[1]+q[1]) << " " << (p0[2]+q[2]) << "\n";
            const int fc[12][3] = {{0,1,2},{0,2,3},{4,6,5},{4,7,6},{0,4,5},{0,5,1},
                                   {1,5,6},{1,6,2},{2,6,7},{2,7,3},{3,7,4},{3,4,0}};
            for (const auto& t : fc)
                f << "f " << (vbase+t[0]) << " " << (vbase+t[1]) << " " << (vbase+t[2]) << "\n";
            vbase += 8;
        }
    }
    return true;
}

int MuJoCoUrdfBackend::kinematicGeomCount(int handle) const
{
    if (!m_ || handle < 0 || handle >= static_cast<int>(kinematic_.size())) return -1;
    const std::string name = "kin_" + std::to_string(handle);
    const int body_id = mj_name2id(m_, mjOBJ_BODY, name.c_str());
    if (body_id < 0) return -1;
    int n = 0;
    for (int g = 0; g < m_->ngeom; ++g)
        if (m_->geom_bodyid[g] == body_id) ++n;
    return n;
}

void MuJoCoUrdfBackend::resolveMocapIds()
{
    if (!m_) return;
    for (size_t i = 0; i < kinematic_.size(); ++i) {
        const std::string name = "kin_" + std::to_string(i);
        const int body_id = mj_name2id(m_, mjOBJ_BODY, name.c_str());
        kinematic_[i].mocapid = (body_id >= 0) ? m_->body_mocapid[body_id] : -1;
    }
}

void MuJoCoUrdfBackend::recompileIfNeeded()
{
    if (!needs_recompile_ || !spec_ || !m_ || !d_) return;
    needs_recompile_ = false;

    // ⚠ NOT mj_recompile, despite it looking made for this. Its signature is
    //     int mj_recompile(mjSpec*, const mjVFS*, mjModel* m, mjData* d)
    // and internally it calls Compile(vfs, &m) / MakeData(m, &d) on those BY-VALUE parameters
    // (user_api.cc:339-346). It therefore frees the caller's model and data and writes the
    // replacements into its own locals — leaving m_ and d_ dangling. A test here appeared to pass
    // while reading freed memory, which is exactly how that mistake survives review.
    //
    // Compiling into fresh objects and swapping is a few more lines and is actually correct.
    mjModel* nm = mj_compile(spec_, nullptr);
    if (!nm) {
        const char* e = mjs_getError(spec_);
        throw std::runtime_error(std::string("MuJoCo: recompile after adding a kinematic body "
                                             "failed: ") + (e ? e : "(no detail)"));
    }
    mjData* nd = mj_makeData(nm);
    if (!nd) { mj_deleteModel(nm); throw std::runtime_error("MuJoCo: mj_makeData failed on recompile"); }

    // ⚠ Carry the robot's state over by hand. A mocap body adds no degrees of freedom, so nq and
    // nv are unchanged and this is an exact copy; the guard is there so a future change that DOES
    // alter the DOF count resets cleanly instead of copying garbage.
    if (nm->nq == m_->nq && nm->nv == m_->nv) {
        std::memcpy(nd->qpos, d_->qpos, static_cast<size_t>(nm->nq) * sizeof(mjtNum));
        std::memcpy(nd->qvel, d_->qvel, static_cast<size_t>(nm->nv) * sizeof(mjtNum));
        nd->time = d_->time;
    }

    mj_deleteData(d_);
    mj_deleteModel(m_);
    m_ = nm;
    d_ = nd;
    m_->opt.timestep = opts_.fixed_timestep;
    mj_forward(m_, d_);

    resolveMocapIds();

    // Re-apply every stored pose, since the mocap indices may have moved.
    for (size_t i = 0; i < kinematic_.size(); ++i)
        setKinematicPose(static_cast<int>(i), kinematic_[i].position, kinematic_[i].orientation);
}

bool MuJoCoUrdfBackend::collisionDebugGeometry(const CollisionDebugFilter& filter,
                                               CollisionDebugSnapshot& out) const
{
    out = CollisionDebugSnapshot();
    out.backend = "mujoco";
    if (m_ == nullptr || d_ == nullptr)
        return false;
    out.solver_time = d_->time;

    // ⚠ Attribute by the LINK TABLE this backend built, not by MuJoCo's body names. A private-world
    // model carries no namespace prefix, so its body names are the URDF's own and two robots in one
    // scene would be indistinguishable in the overlay; `links_` is what actually knows which URDF
    // link a body id came from.
    readMuJoCoCollisionGeometry(
        m_, d_, filter,
        [this](int bodyid, bool& is_world) -> std::string {
            is_world = (bodyid <= 0);
            if (is_world)
                return "level";
            for (const LinkRec& link : links_)
                if (link.body == bodyid)
                    return link.name;
            const char* body_name = mj_id2name(m_, mjOBJ_BODY, bodyid);
            return body_name ? std::string(body_name) : std::string("<unnamed>");
        },
        out);
    return true;
}

bool MuJoCoUrdfBackend::describeColliders(PhysicsColliderSet& out) const
{
    out = PhysicsColliderSet();
    if (m_ == nullptr || d_ == nullptr)
        return false;

    // A private world holds exactly one robot, so every non-world body is ours. Names come from
    // the link table rather than MuJoCo's, which carries no namespace prefix here.
    describeMuJoCoColliders(
        m_, d_, [](int) { return true; },
        [this](int bodyid, bool& is_world) -> std::string {
            is_world = (bodyid <= 0);
            for (const LinkRec& link : links_)
                if (link.body == bodyid)
                    return link.name;
            const char* name = mj_id2name(m_, mjOBJ_BODY, bodyid);
            return name ? std::string(name) : std::string("<unnamed>");
        },
        out);

    // ⚠ Links MuJoCo dropped are NAMED, not merely absent from the count. A link with body == -1
    // never reached the solver, and a sidecar told nothing about it would register a robot with a
    // silent hole in it.
    for (const LinkRec& link : links_)
        if (link.body < 0)
            out.undescribed.push_back(link.name + " (MuJoCo realised no body for this link)");

    return true;
}

} // namespace urdf
