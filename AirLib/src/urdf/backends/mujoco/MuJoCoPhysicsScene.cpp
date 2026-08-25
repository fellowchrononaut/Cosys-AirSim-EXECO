#include "urdf/backends/mujoco/MuJoCoPhysicsScene.hpp"

#include "urdf/backends/mujoco/MuJoCoCollisionReadback.hpp"
#include "urdf/backends/mujoco/MuJoCoUrdfXml.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace urdf {
namespace {

/// Time constant of the inertia-scaled velocity servo, in seconds. Must stay comfortably larger
/// than the solver step; at the nominal 3 ms tick this is ~17 steps.
constexpr double kVelocityTimeConstant = 0.05;

struct SpecDeleter {
    void operator()(mjSpec* spec) const
    {
        if (spec)
            mj_deleteSpec(spec);
    }
};

bool sameNumber(double first, double second)
{
    const double scale = std::max(1.0, std::max(std::fabs(first), std::fabs(second)));
    return std::fabs(first - second) <= 1.0e-12 * scale;
}

void requireFinitePose(const BackendOptions& options)
{
    const Vec3& p = options.root_position;
    const Quat& q = options.root_orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
        !std::isfinite(q.w)) {
        throw std::invalid_argument("MuJoCoPhysicsScene: root pose contains a non-finite value");
    }
}

Quat normalized(const Quat& input)
{
    const double norm = std::sqrt(input.x * input.x + input.y * input.y +
                                  input.z * input.z + input.w * input.w);
    if (!std::isfinite(norm) || norm <= 1.0e-12)
        throw std::invalid_argument("MuJoCoPhysicsScene: root orientation is not a valid quaternion");
    return Quat{ input.x / norm, input.y / norm, input.z / norm, input.w / norm };
}

Quat toQuat(const mjtNum* value)
{
    return Quat{ value[1], value[2], value[3], value[0] };
}

std::string specError(mjSpec* spec, const char* fallback)
{
    const char* detail = spec ? mjs_getError(spec) : nullptr;
    return detail && detail[0] ? std::string(detail) : std::string(fallback);
}

} // namespace

MuJoCoPhysicsScene::MuJoCoPhysicsScene()
    : MuJoCoPhysicsScene(Options())
{
}

MuJoCoPhysicsScene::MuJoCoPhysicsScene(const Options& options)
    : options_(options)
{
    if (!std::isfinite(options_.fixed_timestep) || options_.fixed_timestep <= 0.0)
        throw std::invalid_argument("MuJoCoPhysicsScene: fixed_timestep must be finite and positive");
    if (!std::isfinite(options_.gravity_z))
        throw std::invalid_argument("MuJoCoPhysicsScene: gravity_z must be finite");
    if (!std::isfinite(options_.ground_plane_z))
        throw std::invalid_argument("MuJoCoPhysicsScene: ground_plane_z must be finite");

    spec_ = mj_makeSpec();
    if (!spec_)
        throw std::runtime_error("MuJoCoPhysicsScene: mj_makeSpec failed");

    try {
        // Attachment defaults to sharing source elements. Deep-copy makes scene ownership explicit:
        // each temporary parsed URDF spec can be released immediately after mjs_attach returns.
        if (mjs_setDeepCopy(spec_, 1) != 0)
            throw std::runtime_error("MuJoCoPhysicsScene: could not enable deep-copy attachment");

        spec_->compiler.fusestatic = 0;
        spec_->compiler.conflict = mjCONFLICT_ERROR;
        spec_->option.timestep = options_.fixed_timestep;
        spec_->option.gravity[0] = 0.0;
        spec_->option.gravity[1] = 0.0;
        spec_->option.gravity[2] = options_.gravity_z;
        mjs_setString(spec_->modelname, "AirSim shared MuJoCo scene");

        // ⚠ The ground is NOT emitted here. It is emitted at compile time together with the
        // mirrored level, by the one shared emitter, so the world has a single description of what
        // the robots stand on rather than one written here and another written later.
    }
    catch (...) {
        destroy();
        throw;
    }
}

MuJoCoPhysicsScene::~MuJoCoPhysicsScene()
{
    destroy();
}

void MuJoCoPhysicsScene::destroy()
{
    if (data_) {
        mj_deleteData(data_);
        data_ = nullptr;
    }
    if (model_) {
        mj_deleteModel(model_);
        model_ = nullptr;
    }
    if (spec_) {
        mj_deleteSpec(spec_);
        spec_ = nullptr;
    }
}

std::string MuJoCoPhysicsScene::makePrefix(const std::string& stable_id)
{
    static const char hex[] = "0123456789abcdef";
    std::string result = "art_";
    result.reserve(5 + stable_id.size() * 2);
    for (const unsigned char value : stable_id) {
        result.push_back(hex[value >> 4]);
        result.push_back(hex[value & 0x0f]);
    }
    result.push_back('/');
    return result;
}

bool MuJoCoPhysicsScene::isMappedJoint(JointType type)
{
    return type == JointType::Revolute || type == JointType::Continuous ||
           type == JointType::Prismatic;
}

void MuJoCoPhysicsScene::requireEditable(const char* operation) const
{
    if (faulted_)
        throw std::logic_error(std::string("MuJoCoPhysicsScene: ") + operation +
                               " rejected because the scene is faulted");
    if (compiled())
        throw std::logic_error(std::string("MuJoCoPhysicsScene: ") + operation +
                               " rejected after topology was compiled");
}

void MuJoCoPhysicsScene::requireCompiled(const char* operation) const
{
    if (faulted_)
        throw std::logic_error(std::string("MuJoCoPhysicsScene: ") + operation +
                               " rejected because the scene is faulted");
    if (!compiled())
        throw std::logic_error(std::string("MuJoCoPhysicsScene: ") + operation +
                               " requires a compiled scene");
}

MuJoCoPhysicsScene::ArticulationHandle MuJoCoPhysicsScene::addArticulation(
    const std::string& stable_id, const Robot& robot, const BackendOptions& options)
{
    requireEditable("addArticulation");
    if (stable_id.empty())
        throw std::invalid_argument("MuJoCoPhysicsScene: articulation stable_id is empty");
    if (articulation_index_.find(stable_id) != articulation_index_.end())
        throw std::invalid_argument("MuJoCoPhysicsScene: duplicate articulation stable_id '" +
                                    stable_id + "'");
    if (options.urdf_xml.empty())
        throw std::invalid_argument("MuJoCoPhysicsScene: BackendOptions::urdf_xml is empty for '" +
                                    stable_id + "'");
    if (!sameNumber(options.fixed_timestep, options_.fixed_timestep))
        throw std::invalid_argument("MuJoCoPhysicsScene: articulation '" + stable_id +
                                    "' requests a different fixed_timestep than the global scene");
    if (!sameNumber(options.gravity_z, options_.gravity_z))
        throw std::invalid_argument("MuJoCoPhysicsScene: articulation '" + stable_id +
                                    "' requests a different gravity_z than the global scene");
    if (options.add_ground_plane != options_.add_ground_plane ||
        (options.add_ground_plane && !sameNumber(options.ground_plane_z, options_.ground_plane_z))) {
        throw std::invalid_argument("MuJoCoPhysicsScene: articulation '" + stable_id +
                                    "' ground options differ from the global scene");
    }
    if (options.ground_height_field.valid())
        throw std::invalid_argument(
            "MuJoCoPhysicsScene: per-articulation height fields are not sound in a shared scene; "
            "a world-level height-field descriptor is required");
    // ⚠ mesh_search_paths are no longer rejected: the URDF's mesh references are resolved to
    // absolute paths BEFORE MuJoCo's reader sees them (see MuJoCoUrdfXml.hpp). MuJoCo still knows
    // nothing about ROS packages - it simply never meets one.
    if (robot.root_link < 0 || robot.root_link >= static_cast<int>(robot.links.size()))
        throw std::invalid_argument("MuJoCoPhysicsScene: articulation '" + stable_id +
                                    "' has no valid root link");
    for (const Joint& joint : robot.joints) {
        if (joint.type == JointType::Floating || joint.type == JointType::Planar) {
            throw std::invalid_argument("MuJoCoPhysicsScene: articulation '" + stable_id +
                                        "' contains unsupported joint '" + joint.name + "'");
        }
    }
    requireFinitePose(options);
    const Quat orientation = normalized(options.root_orientation);

    // Resolve package:// and relative mesh references before MuJoCo's own reader meets them.
    // Unresolved ones are recorded so a hollow robot is reported rather than discovered by
    // watching it fall through the floor.
    unresolved_meshes_.clear();
    const std::string resolved_xml = resolveMeshPathsForMuJoCo(
        options.urdf_xml, options.mesh_base_dir, options.mesh_search_paths, &unresolved_meshes_);

    char error[1024] = { 0 };
    std::unique_ptr<mjSpec, SpecDeleter> child(
        mj_parseXMLString(resolved_xml.c_str(), nullptr, error, sizeof(error)));
    if (!child)
        throw std::runtime_error("MuJoCoPhysicsScene: could not parse articulation '" + stable_id +
                                 "': " + error);

    // fusestatic is compiler-origin scoped and is not among MuJoCo 3.12's attach-conflict fields.
    // Set it on every imported spec or fixed URDF links disappear even though the parent is correct.
    child->compiler.fusestatic = 0;
    if (!options.mesh_base_dir.empty())
        mjs_setString(child->compiler.meshdir, options.mesh_base_dir.c_str());

    const std::string& root_name = robot.links[static_cast<size_t>(robot.root_link)].name;
    mjsBody* root = mjs_findBody(child.get(), root_name.c_str());
    if (!root)
        throw std::runtime_error("MuJoCoPhysicsScene: could not find root link '" + root_name +
                                 "' in articulation '" + stable_id + "'");
    root->pos[0] = options.root_position.x;
    root->pos[1] = options.root_position.y;
    root->pos[2] = options.root_position.z;
    root->quat[0] = orientation.w;
    root->quat[1] = orientation.x;
    root->quat[2] = orientation.y;
    root->quat[3] = orientation.z;
    if (!options.fixed_base && !mjs_addFreeJoint(root))
        throw std::runtime_error("MuJoCoPhysicsScene: could not add a free joint to articulation '" +
                                 stable_id + "'");

    // ⚠ ACTUATORS GO ON THE CHILD SPEC, BEFORE mjs_attach. A URDF declares none and MuJoCo
    // requires them, so without this the articulation loads, renders, and ignores every control
    // call. Adding them here rather than to the parent lets the attach namespace them along with
    // everything else, so two copies of one robot do not fight over the actuator name.
    for (const Joint& source : robot.joints) {
        if (!isMappedJoint(source.type))
            continue;
        mjsActuator* actuator = mjs_addActuator(child.get(), nullptr);
        if (!actuator)
            throw std::runtime_error("MuJoCoPhysicsScene: could not add an actuator for joint '" +
                                     source.name + "' of '" + stable_id + "'");
        mjs_setName(actuator->element, ("act_" + source.name).c_str());
        actuator->trntype = mjTRN_JOINT;
        mjs_setString(actuator->target, source.name.c_str());
        actuator->gaintype = mjGAIN_FIXED;
        actuator->biastype = mjBIAS_NONE;
        actuator->gainprm[0] = 1.0; // ctrl IS the torque, in N.m
        // URDF <limit effort> is a real motor ceiling; honour it so no client can command a torque
        // the joint could not produce.
        if (source.limit.present && source.limit.effort > 0.0) {
            actuator->forcerange[0] = -source.limit.effort;
            actuator->forcerange[1] = source.limit.effort;
        }
    }

    ArticulationRec record;
    record.stable_id = stable_id;
    record.prefix = makePrefix(stable_id);
    record.links.reserve(robot.links.size());
    for (const Link& source : robot.links) {
        LinkRec link;
        link.name = source.name;
        link.qualified_name = record.prefix + source.name;
        record.link_index.emplace(link.name, record.links.size());
        record.links.push_back(std::move(link));
    }
    for (const Joint& source : robot.joints) {
        if (!isMappedJoint(source.type))
            continue;
        JointRec joint;
        joint.name = source.name;
        joint.qualified_name = record.prefix + source.name;
        joint.effort_limit =
            source.limit.present && source.limit.effort > 0.0 ? source.limit.effort : 0.0;
        record.joint_index.emplace(joint.name, record.joints.size());
        record.joints.push_back(std::move(joint));
    }

    mjsBody* world = mjs_findBody(spec_, "world");
    if (!world)
        throw std::runtime_error("MuJoCoPhysicsScene: parent spec has no world body");
    const int warnings_before = mjs_numWarnings(spec_);
    if (!mjs_attach(world->element, child->element, record.prefix.c_str(), "")) {
        faulted_ = true;
        throw std::runtime_error("MuJoCoPhysicsScene: attaching articulation '" + stable_id +
                                 "' failed: " + specError(spec_, "no detail"));
    }

    // mjCONFLICT_ERROR rejects two authored conflicting values, but a value authored only by the
    // child is retained as an attach warning while the parent wins. Treat that as a hard failure:
    // silently dropping a solver option would make the two robots inhabit different intended worlds.
    const int warnings_after = mjs_numWarnings(spec_);
    if (warnings_after > warnings_before) {
        std::string detail;
        for (int index = warnings_before; index < warnings_after; ++index) {
            const char* warning = mjs_getWarning(spec_, index);
            if (warning && warning[0])
                detail += (detail.empty() ? "" : "; ") + std::string(warning);
        }
        faulted_ = true;
        throw std::runtime_error("MuJoCoPhysicsScene: articulation '" + stable_id +
                                 "' introduced an unresolved global attach conflict: " + detail);
    }

    const size_t index = articulations_.size();
    try {
        articulation_index_.emplace(stable_id, index);
        articulations_.push_back(std::move(record));
    }
    catch (...) {
        faulted_ = true; // attachment cannot be rolled back safely
        throw;
    }
    return ArticulationHandle{ index };
}

void MuJoCoPhysicsScene::setStaticWorld(std::shared_ptr<const StaticWorld> world)
{
    requireEditable("setStaticWorld");
    static_world_ = std::move(world);
}

void MuJoCoPhysicsScene::compile()
{
    requireEditable("compile");
    if (articulations_.empty())
        throw std::logic_error("MuJoCoPhysicsScene: cannot compile an empty scene");

    // The ground and the mirrored level, through the SAME emitter the per-robot backend uses - so
    // a coordinated run and a legacy run stand on identically cooked geometry by construction.
    {
        mjsBody* world = mjs_findBody(spec_, "world");
        if (!world)
            throw std::runtime_error("MuJoCoPhysicsScene: parent spec has no world body");

        StaticWorldEmitOptions emit;
        emit.ground_height_field = options_.ground_height_field;
        emit.add_ground_plane = options_.add_ground_plane;
        emit.ground_plane_z = options_.ground_plane_z;
        emit.clip_center = options_.static_world_clip_center;
        emit.static_world_radius = options_.static_world_radius;
        emit.static_world_max_triangles = options_.static_world_max_triangles;
        emit.static_mesh_mode = options_.static_mesh_mode;

        try {
            emitStaticWorld(spec_, world, static_world_.get(), emit, static_world_stats_);
        }
        catch (...) {
            faulted_ = true;
            throw;
        }
    }

    model_ = mj_compile(spec_, nullptr);
    if (!model_) {
        faulted_ = true;
        throw std::runtime_error("MuJoCoPhysicsScene: shared model compilation failed: " +
                                 specError(spec_, "no detail"));
    }
    data_ = mj_makeData(model_);
    if (!data_) {
        mj_deleteModel(model_);
        model_ = nullptr;
        faulted_ = true;
        throw std::runtime_error("MuJoCoPhysicsScene: mj_makeData failed");
    }

    try {
        resolveCompiledHandles();
        mj_forward(model_, data_);
    }
    catch (...) {
        mj_deleteData(data_);
        data_ = nullptr;
        mj_deleteModel(model_);
        model_ = nullptr;
        faulted_ = true;
        throw;
    }
}

void MuJoCoPhysicsScene::resolveCompiledHandles()
{
    body_owner_.assign(static_cast<size_t>(model_->nbody), -1);
    for (ArticulationRec& record : articulations_)
        record.collision_geoms = 0;
    for (size_t articulation_index = 0; articulation_index < articulations_.size();
         ++articulation_index) {
        ArticulationRec& record = articulations_[articulation_index];
        for (LinkRec& link : record.links) {
            link.body = mj_name2id(model_, mjOBJ_BODY, link.qualified_name.c_str());
            if (link.body < 0) {
                throw std::runtime_error("MuJoCoPhysicsScene: attached link '" +
                                         link.qualified_name + "' did not survive compilation");
            }
            body_owner_.at(static_cast<size_t>(link.body)) =
                static_cast<int>(articulation_index);
            // ⚠ COUNT WHAT SURVIVED COMPILATION, not what the URDF declared. The two differ
            // whenever MuJoCo could not read a mesh, and only this number describes the robot that
            // is actually in the world.
            record.collision_geoms += model_->body_geomnum[link.body];
        }
        for (JointRec& joint : record.joints) {
            joint.joint = mj_name2id(model_, mjOBJ_JOINT, joint.qualified_name.c_str());
            if (joint.joint < 0) {
                throw std::runtime_error("MuJoCoPhysicsScene: attached joint '" +
                                         joint.qualified_name + "' did not survive compilation");
            }
            joint.qposadr = model_->jnt_qposadr[joint.joint];
            joint.dofadr = model_->jnt_dofadr[joint.joint];

            // Apply any gains requested before the model existed, now that the inertia does.
            if (joint.gains_requested) {
                const double inertia =
                    model_->dof_M0[joint.dofadr] > 0 ? model_->dof_M0[joint.dofadr] : 1.0;
                const double omega = 2.0 * 3.14159265358979323846 * joint.gain_hertz;
                joint.kp = inertia * omega * omega;
                joint.kd = 2.0 * joint.gain_damping_ratio * std::sqrt(joint.kp * inertia);
            }

            const std::string actuator_name = record.prefix + "act_" + joint.name;
            joint.actuator = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_name.c_str());
            if (joint.actuator < 0) {
                throw std::runtime_error("MuJoCoPhysicsScene: actuator '" + actuator_name +
                                         "' did not survive compilation, so joint '" +
                                         joint.qualified_name + "' would be uncommandable");
            }
        }
    }
}

void MuJoCoPhysicsScene::step(double dt)
{
    requireCompiled("step");
    if (!std::isfinite(dt) || !sameNumber(dt, options_.fixed_timestep)) {
        throw std::invalid_argument(
            "MuJoCoPhysicsScene: step dt must equal the global fixed_timestep; "
            "this exact-once seam performs no hidden substeps");
    }
    if (steps_taken_ == std::numeric_limits<uint64_t>::max())
        throw std::overflow_error("MuJoCoPhysicsScene: step counter overflow");
    applyControls();
    mj_step(model_, data_);
    ++steps_taken_;
}

void MuJoCoPhysicsScene::reset()
{
    requireCompiled("reset");
    mj_resetData(model_, data_);
    // Control CONFIGURATION survives, control STATE does not - the same split the per-robot
    // backends use, so a reset means "back to the initial condition", not "back to unconfigured".
    for (ArticulationRec& record : articulations_)
        for (JointRec& joint : record.joints)
            joint.target = 0.0;
    steps_taken_ = 0;
    mj_forward(model_, data_);
}

const MuJoCoPhysicsScene::ArticulationRec& MuJoCoPhysicsScene::articulation(
    ArticulationHandle handle) const
{
    if (!handle.valid() || handle.index >= articulations_.size())
        throw std::out_of_range("MuJoCoPhysicsScene: invalid articulation handle");
    return articulations_[handle.index];
}

const MuJoCoPhysicsScene::LinkRec& MuJoCoPhysicsScene::link(LinkHandle handle) const
{
    if (!handle.valid() || handle.articulation >= articulations_.size() ||
        handle.link >= articulations_[handle.articulation].links.size()) {
        throw std::out_of_range("MuJoCoPhysicsScene: invalid link handle");
    }
    return articulations_[handle.articulation].links[handle.link];
}

const MuJoCoPhysicsScene::JointRec& MuJoCoPhysicsScene::joint(JointHandle handle) const
{
    if (!handle.valid() || handle.articulation >= articulations_.size() ||
        handle.joint >= articulations_[handle.articulation].joints.size()) {
        throw std::out_of_range("MuJoCoPhysicsScene: invalid joint handle");
    }
    return articulations_[handle.articulation].joints[handle.joint];
}

MuJoCoPhysicsScene::ArticulationHandle MuJoCoPhysicsScene::findArticulation(
    const std::string& stable_id) const
{
    const auto found = articulation_index_.find(stable_id);
    return found == articulation_index_.end() ? ArticulationHandle{}
                                               : ArticulationHandle{ found->second };
}

size_t MuJoCoPhysicsScene::linkCount(ArticulationHandle handle) const
{
    return articulation(handle).links.size();
}

size_t MuJoCoPhysicsScene::jointCount(ArticulationHandle handle) const
{
    return articulation(handle).joints.size();
}

const std::string& MuJoCoPhysicsScene::stableId(ArticulationHandle handle) const
{
    return articulation(handle).stable_id;
}

MuJoCoPhysicsScene::LinkHandle MuJoCoPhysicsScene::findLink(
    ArticulationHandle handle, const std::string& local_name) const
{
    const ArticulationRec& record = articulation(handle);
    const auto found = record.link_index.find(local_name);
    return found == record.link_index.end() ? LinkHandle{} : LinkHandle{ handle.index, found->second };
}

MuJoCoPhysicsScene::JointHandle MuJoCoPhysicsScene::findJoint(
    ArticulationHandle handle, const std::string& local_name) const
{
    const ArticulationRec& record = articulation(handle);
    const auto found = record.joint_index.find(local_name);
    return found == record.joint_index.end() ? JointHandle{}
                                             : JointHandle{ handle.index, found->second };
}

MuJoCoPhysicsScene::LinkHandle MuJoCoPhysicsScene::linkAt(ArticulationHandle handle,
                                                          size_t index) const
{
    if (index >= articulation(handle).links.size())
        throw std::out_of_range("MuJoCoPhysicsScene: link index out of range");
    return LinkHandle{ handle.index, index };
}

MuJoCoPhysicsScene::JointHandle MuJoCoPhysicsScene::jointAt(ArticulationHandle handle,
                                                            size_t index) const
{
    if (index >= articulation(handle).joints.size())
        throw std::out_of_range("MuJoCoPhysicsScene: joint index out of range");
    return JointHandle{ handle.index, index };
}

const std::string& MuJoCoPhysicsScene::linkName(LinkHandle handle) const
{
    return link(handle).name;
}

const std::string& MuJoCoPhysicsScene::jointName(JointHandle handle) const
{
    return joint(handle).name;
}

const std::string& MuJoCoPhysicsScene::qualifiedLinkName(LinkHandle handle) const
{
    return link(handle).qualified_name;
}

const std::string& MuJoCoPhysicsScene::qualifiedJointName(JointHandle handle) const
{
    return joint(handle).qualified_name;
}

LinkPose MuJoCoPhysicsScene::getLinkPose(LinkHandle handle) const
{
    requireCompiled("getLinkPose");
    const LinkRec& record = link(handle);
    LinkPose result;
    result.position = Vec3{ data_->xpos[3 * record.body], data_->xpos[3 * record.body + 1],
                            data_->xpos[3 * record.body + 2] };
    result.orientation = toQuat(&data_->xquat[4 * record.body]);
    return result;
}

JointState MuJoCoPhysicsScene::getJointState(JointHandle handle) const
{
    requireCompiled("getJointState");
    const JointRec& record = joint(handle);
    JointState result;
    result.position = data_->qpos[record.qposadr];
    result.velocity = data_->qvel[record.dofadr];
    // ⚠ The torque this joint's ACTUATOR applied, not the constraint reaction - the same quantity
    // the per-robot backend reports, so a client comparing engines compares like with like. Left
    // unset this silently read zero, which is indistinguishable from a motor doing nothing.
    if (record.actuator >= 0)
        result.effort = data_->actuator_force[record.actuator];
    return result;
}

int MuJoCoPhysicsScene::compiledBodyId(LinkHandle handle) const
{
    requireCompiled("compiledBodyId");
    return link(handle).body;
}

int MuJoCoPhysicsScene::compiledJointId(JointHandle handle) const
{
    requireCompiled("compiledJointId");
    return joint(handle).joint;
}

size_t MuJoCoPhysicsScene::compiledBodyCount() const
{
    requireCompiled("compiledBodyCount");
    return static_cast<size_t>(model_->nbody);
}

size_t MuJoCoPhysicsScene::compiledJointCount() const
{
    requireCompiled("compiledJointCount");
    return static_cast<size_t>(model_->njnt);
}

size_t MuJoCoPhysicsScene::contactCountBetween(ArticulationHandle first,
                                               ArticulationHandle second) const
{
    requireCompiled("contactCountBetween");
    articulation(first);
    articulation(second);
    size_t count = 0;
    for (mjtSize index = 0; index < data_->ncon; ++index) {
        const mjContact& contact = data_->contact[index];
        const int first_body = model_->geom_bodyid[contact.geom1];
        const int second_body = model_->geom_bodyid[contact.geom2];
        const int first_owner = body_owner_.at(static_cast<size_t>(first_body));
        const int second_owner = body_owner_.at(static_cast<size_t>(second_body));
        if ((first_owner == static_cast<int>(first.index) &&
             second_owner == static_cast<int>(second.index)) ||
            (first_owner == static_cast<int>(second.index) &&
             second_owner == static_cast<int>(first.index))) {
            ++count;
        }
    }
    return count;
}

void MuJoCoPhysicsScene::applyControls()
{
    for (const ArticulationRec& record : articulations_) {
        for (const JointRec& joint : record.joints) {
            if (joint.actuator < 0 || joint.qposadr < 0)
                continue;

            double torque = 0.0;
            switch (joint.mode) {
            case ControlMode::Effort:
                torque = joint.target;
                break;
            case ControlMode::Position:
                // PD about the target, computed here so kp and kd are exactly what the caller
                // asked for; MuJoCo's own position actuator would bury them in gainprm/biasprm.
                torque = joint.kp * (joint.target - data_->qpos[joint.qposadr]) -
                         joint.kd * data_->qvel[joint.dofadr];
                break;
            case ControlMode::Velocity: {
                // ⚠ INERTIA-SCALED, and it must be. A hard-coded gain is a torque-per-rad/s, so
                // what it does depends entirely on the joint it meets. Measured on the ExoMy:
                // wheel axis inertia 8.05e-5 kg.m^2 with URDF <limit effort="1000"> (a placeholder,
                // absurd for a 57 g wheel). A gain of 10 against a 6 rad/s target is 60 N.m, i.e.
                // 745,000 rad/s^2 - one 3 ms step overshoots by 2,236 rad/s, the next saturates the
                // opposite way, and the model explodes to 1.2e6 rad/s. Seen in the editor
                // 2026-08-25.
                //
                // torque = (I/tau) * (target - w) makes it a first-order lag: dw/dt =
                // (target - w)/tau, converging with time constant tau REGARDLESS of inertia, and
                // stable while tau is comfortably larger than dt. An explicitly configured kd (from
                // setPositionGains) is already inertia-derived, so it is honoured as-is.
                const double inertia = model_->dof_M0[joint.dofadr] > 0 ? model_->dof_M0[joint.dofadr] : 1.0;
                const double gain = joint.kd > 0 ? joint.kd : inertia / kVelocityTimeConstant;
                torque = gain * (joint.target - data_->qvel[joint.dofadr]);
                break;
            }
            case ControlMode::None:
                torque = 0.0;
                break;
            }

            // forcerange clamps this too; clamping here as well keeps the reported effort honest
            // about what was actually asked for.
            if (joint.effort_limit > 0.0) {
                if (torque > joint.effort_limit) torque = joint.effort_limit;
                else if (torque < -joint.effort_limit) torque = -joint.effort_limit;
            }
            data_->ctrl[joint.actuator] = torque;
        }
    }
}

MuJoCoPhysicsScene::JointRec& MuJoCoPhysicsScene::mutableJoint(JointHandle handle)
{
    if (!handle.valid() || handle.articulation >= articulations_.size() ||
        handle.joint >= articulations_[handle.articulation].joints.size()) {
        throw std::out_of_range("MuJoCoPhysicsScene: invalid joint handle");
    }
    return articulations_[handle.articulation].joints[handle.joint];
}

void MuJoCoPhysicsScene::setJointTarget(JointHandle handle, ControlMode mode, double value)
{
    // No requireCompiled: a target is configuration, not solver state, and the record it lives in
    // exists as soon as the articulation is added.
    if (faulted_)
        throw std::logic_error("MuJoCoPhysicsScene: setJointTarget rejected on a faulted scene");
    if (!std::isfinite(value))
        throw std::invalid_argument("MuJoCoPhysicsScene: joint target must be finite");
    JointRec& record = mutableJoint(handle);
    record.mode = mode;
    record.target = value;
}

void MuJoCoPhysicsScene::setPositionGains(JointHandle handle, double hertz, double damping_ratio)
{
    if (faulted_)
        throw std::logic_error("MuJoCoPhysicsScene: setPositionGains rejected on a faulted scene");
    if (!std::isfinite(hertz) || hertz < 0.0 || !std::isfinite(damping_ratio) ||
        damping_ratio < 0.0)
        throw std::invalid_argument(
            "MuJoCoPhysicsScene: position gains need a non-negative finite hertz and damping ratio");

    JointRec& record = mutableJoint(handle);
    record.gains_requested = true;
    record.gain_hertz = hertz;
    record.gain_damping_ratio = damping_ratio;

    // Before compilation there is no inertia to convert against; resolveCompiledHandles applies
    // the stored request the moment there is.
    if (!compiled() || record.dofadr < 0)
        return;
    // ⚠ hertz -> kp needs an inertia, and this is where the two rigid backends genuinely differ.
    // Box3D's soft constraint is mass-normalised, so `hertz` fixes a natural frequency and the
    // stiffness it yields changes as the robot moves. Here the conversion is done ONCE against the
    // joint's diagonal inertia, so Kp is constant. That difference is deliberate and is exactly the
    // kind of thing a two-backend comparison exists to expose.
    const double inertia = model_->dof_M0[record.dofadr] > 0 ? model_->dof_M0[record.dofadr] : 1.0;
    const double omega = 2.0 * 3.14159265358979323846 * hertz;
    record.kp = inertia * omega * omega;
    record.kd = 2.0 * damping_ratio * std::sqrt(record.kp * inertia);
}

void MuJoCoPhysicsScene::applyExternalWrench(LinkHandle handle, const Wrench& wrench)
{
    requireCompiled("applyExternalWrench");
    const LinkRec& record = link(handle);
    if (record.body < 0)
        return;
    // ⚠ xfrc_applied is WORLD-frame and about the body's centre of mass, and MuJoCo clears it on
    // reset but NOT between steps - it persists until overwritten, the opposite of Box3D's
    // consume-once convention. A caller re-issuing every step behaves identically on both.
    data_->xfrc_applied[6 * record.body + 0] = wrench.force.x;
    data_->xfrc_applied[6 * record.body + 1] = wrench.force.y;
    data_->xfrc_applied[6 * record.body + 2] = wrench.force.z;
    data_->xfrc_applied[6 * record.body + 3] = wrench.torque.x;
    data_->xfrc_applied[6 * record.body + 4] = wrench.torque.y;
    data_->xfrc_applied[6 * record.body + 5] = wrench.torque.z;
}

Twist MuJoCoPhysicsScene::getLinkTwist(LinkHandle handle) const
{
    requireCompiled("getLinkTwist");
    Twist result;
    const LinkRec& record = link(handle);
    if (record.body < 0)
        return result;
    // ⚠ mj_objectVelocity with flg_local = 0 gives WORLD-frame velocity ordered [angular; linear].
    // Reading data_->cvel directly would give a spatial velocity about the body frame origin - a
    // different quantity that looks almost right.
    mjtNum velocity[6] = { 0 };
    mj_objectVelocity(model_, data_, mjOBJ_BODY, record.body, velocity, 0);
    result.angular = Vec3{ velocity[0], velocity[1], velocity[2] };
    result.linear = Vec3{ velocity[3], velocity[4], velocity[5] };
    return result;
}

double MuJoCoPhysicsScene::articulationMass(ArticulationHandle handle) const
{
    requireCompiled("articulationMass");
    const ArticulationRec& record = articulation(handle);
    double sum = 0.0;
    for (const LinkRec& item : record.links)
        if (item.body > 0) sum += model_->body_mass[item.body];
    return sum;
}

size_t MuJoCoPhysicsScene::collisionGeomsRealised(ArticulationHandle handle) const
{
    requireCompiled("collisionGeomsRealised");
    return articulation(handle).collision_geoms;
}

double MuJoCoPhysicsScene::totalMass() const
{
    requireCompiled("totalMass");
    double sum = 0.0;
    for (int body = 1; body < model_->nbody; ++body) // body 0 is the world
        sum += model_->body_mass[body];
    return sum;
}

double MuJoCoPhysicsScene::simulationTime() const
{
    requireCompiled("simulationTime");
    return data_->time;
}

void MuJoCoPhysicsScene::collisionDebugGeometry(const CollisionDebugFilter& filter,
                                                CollisionDebugSnapshot& out) const
{
    out = CollisionDebugSnapshot();
    out.backend = "mujoco";
    if (!compiled())
        return;
    out.solver_time = data_->time;

    // ⚠ Attribute by `body_owner_`, the same mapping the scene was BUILT with, rather than by
    // unpicking the name prefix. A prefix is a display convention; the owner map is the fact.
    readMuJoCoCollisionGeometry(
        model_, data_, filter,
        [this](int bodyid, bool& is_world) -> std::string {
            is_world = (bodyid <= 0);
            const char* body_name = mj_id2name(model_, mjOBJ_BODY, bodyid);
            if (is_world)
                return "level";
            if (bodyid < static_cast<int>(body_owner_.size())) {
                const int owner = body_owner_[bodyid];
                if (owner >= 0 && owner < static_cast<int>(articulations_.size()))
                    return articulations_[owner].stable_id + "/" +
                           (body_name ? std::string(body_name) : std::string("<unnamed>"));
            }
            return body_name ? std::string(body_name) : std::string("<unnamed>");
        },
        out);
}

void MuJoCoPhysicsScene::describeColliders(ArticulationHandle handle,
                                          PhysicsColliderSet& out) const
{
    out = PhysicsColliderSet();
    if (!compiled())
        return;
    const ArticulationRec& record = articulation(handle);
    const int owner_index = static_cast<int>(handle.index);

    describeMuJoCoColliders(
        model_, data_,
        [this, owner_index](int bodyid) {
            return bodyid < static_cast<int>(body_owner_.size()) &&
                   body_owner_[bodyid] == owner_index;
        },
        [this, &record](int bodyid, bool& is_world) -> std::string {
            is_world = false;
            const char* name = mj_id2name(model_, mjOBJ_BODY, bodyid);
            std::string local = name ? std::string(name) : std::string("<unnamed>");

            // ⚠ STRIP THE NAMESPACE PREFIX. `mjs_attach` namespaces every body as
            // "art_<hex>/base", and leaving that in the stable id would key a sidecar's collider
            // registry on an encoding of our own attach order - stable within one compile and
            // meaningless across a rebuild, which is the opposite of what "stable id" promises.
            if (local.rfind(record.prefix, 0) == 0)
                local.erase(0, record.prefix.size());

            // Qualified by the articulation's stable id: two copies of one URDF share every link
            // name, and a collider id must be unique in the world.
            return record.stable_id + "/" + local;
        },
        out);

    for (const LinkRec& link : record.links)
        if (link.body < 0)
            out.undescribed.push_back(record.stable_id + "/" + link.name +
                                      " (MuJoCo realised no body for this link)");
}

} // namespace urdf
