#include "urdf/backends/mujoco/MuJoCoUrdfBackend.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace urdf {
namespace {

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
    spec_ = mj_parseXMLString(opts.urdf_xml.c_str(), nullptr, err, sizeof(err));
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

    // --- scaffolding floor --------------------------------------------------------------------
    // ⚠ Same fallback role as in the Box3D backend, and the caller applies the same suppression:
    // UrdfBotSimApi sets add_ground_plane only when the level mirror produced nothing, because an
    // infinite plane and a real level are not additive.
    if (opts.add_ground_plane && world) {
        mjsGeom* g = mjs_addGeom(world, nullptr);
        mjs_setName(g->element, "urdf_ground_plane");
        g->type = mjGEOM_PLANE;
        // For a plane, size is (x half-extent, y half-extent, grid spacing); zeros in the first two
        // mean infinite, which is what "the floor" should be. The third is rendering-only here.
        g->size[0] = 0.0;
        g->size[1] = 0.0;
        g->size[2] = 1.0;
        g->pos[2] = opts.ground_plane_z;
    }

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
        collision_geoms_realised_ =
            static_cast<size_t>(m_->ngeom) - (opts.add_ground_plane ? 1u : 0u);

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
        case ControlMode::Velocity:
            tau = j.kd > 0 ? j.kd * (j.target - d_->qvel[j.dofadr])
                           : 10.0 * (j.target - d_->qvel[j.dofadr]);
            break;
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
    // ⚠ NOT IMPLEMENTED YET, and it says so rather than silently ignoring the level. Box3D mirrors
    // Unreal's static geometry into the solver; the MuJoCo equivalent is to add geoms to the spec
    // before compilation, which means this must be called BEFORE buildFromUrdf. Until then a
    // MuJoCo-backed robot only collides with what its own URDF declares plus the ground plane.
    (void)world;
}

int MuJoCoUrdfBackend::addKinematicBody(const KinematicBody& body)
{
    // ⚠ NOT IMPLEMENTED YET. The natural MuJoCo primitive is a MOCAP body — `mjData.mocap_pos` /
    // `mocap_quat` exist precisely for bodies driven from outside the solver — but like static
    // geometry they have to be declared before compilation. Returning -1 means "no handle", and
    // setKinematicPose ignores it, so a caller gets no motion rather than wrong motion.
    (void)body;
    return -1;
}

void MuJoCoUrdfBackend::setKinematicPose(int handle, const Vec3& position, const Quat& orientation)
{
    (void)handle; (void)position; (void)orientation;
}

} // namespace urdf
