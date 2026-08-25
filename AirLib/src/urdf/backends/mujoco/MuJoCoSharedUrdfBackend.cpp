#include "urdf/backends/mujoco/MuJoCoSharedUrdfBackend.hpp"

#include <stdexcept>
#include <utility>

namespace urdf {

MuJoCoSharedUrdfBackend::MuJoCoSharedUrdfBackend(MuJoCoPhysicsScene& scene, std::string stable_id)
    : scene_(scene), stable_id_(std::move(stable_id))
{
    if (stable_id_.empty())
        throw std::invalid_argument("MuJoCoSharedUrdfBackend: stable_id is empty");
}

MuJoCoPhysicsScene::ArticulationHandle MuJoCoSharedUrdfBackend::requireArticulation() const
{
    if (!articulation_.valid())
        throw std::logic_error("MuJoCoSharedUrdfBackend: '" + stable_id_ +
                               "' has not been registered with its shared scene yet");
    return articulation_;
}

void MuJoCoSharedUrdfBackend::setStaticWorld(std::shared_ptr<const StaticWorld>)
{
    throw std::logic_error(
        "MuJoCoSharedUrdfBackend: a shared scene has exactly one static world; set it with "
        "MuJoCoPhysicsScene::setStaticWorld rather than once per robot");
}

int MuJoCoSharedUrdfBackend::addKinematicBody(const KinematicBody&)
{
    // Matches the per-robot backend: no mocap bodies yet, and mirrorsKinematicBodies() says so, so
    // the caller is told rather than handed a handle that does nothing.
    return -1;
}

void MuJoCoSharedUrdfBackend::setKinematicPose(int, const Vec3&, const Quat&)
{
}

void MuJoCoSharedUrdfBackend::buildFromUrdf(const Robot& model, const BackendOptions& opts)
{
    if (articulation_.valid())
        throw std::logic_error(
            "MuJoCoSharedUrdfBackend: '" + stable_id_ +
            "' is registered once; rebuild the whole scene rather than one articulation");
    articulation_ = scene_.addArticulation(stable_id_, model, opts);
}

void MuJoCoSharedUrdfBackend::reset()
{
    throw std::logic_error(
        "MuJoCoSharedUrdfBackend: a shared-scene robot cannot reset alone; the global reset "
        "transaction resets every articulation and all contact state together");
}

int MuJoCoSharedUrdfBackend::step(double)
{
    throw std::logic_error(
        "MuJoCoSharedUrdfBackend: a shared-scene robot cannot step alone; stepping once per robot "
        "would advance the shared mjData once per robot");
}

size_t MuJoCoSharedUrdfBackend::linkCount() const
{
    return scene_.linkCount(requireArticulation());
}

size_t MuJoCoSharedUrdfBackend::jointCount() const
{
    return scene_.jointCount(requireArticulation());
}

const std::string& MuJoCoSharedUrdfBackend::linkName(size_t link) const
{
    return scene_.linkName(scene_.linkAt(requireArticulation(), link));
}

const std::string& MuJoCoSharedUrdfBackend::jointName(size_t joint) const
{
    return scene_.jointName(scene_.jointAt(requireArticulation(), joint));
}

int MuJoCoSharedUrdfBackend::findLink(const std::string& name) const
{
    const MuJoCoPhysicsScene::LinkHandle handle = scene_.findLink(requireArticulation(), name);
    return handle.valid() ? static_cast<int>(handle.link) : -1;
}

int MuJoCoSharedUrdfBackend::findJoint(const std::string& name) const
{
    const MuJoCoPhysicsScene::JointHandle handle = scene_.findJoint(requireArticulation(), name);
    return handle.valid() ? static_cast<int>(handle.joint) : -1;
}

LinkPose MuJoCoSharedUrdfBackend::getLinkPose(size_t link) const
{
    return scene_.getLinkPose(scene_.linkAt(requireArticulation(), link));
}

Twist MuJoCoSharedUrdfBackend::getLinkTwist(size_t link) const
{
    return scene_.getLinkTwist(scene_.linkAt(requireArticulation(), link));
}

JointState MuJoCoSharedUrdfBackend::getJointState(size_t joint) const
{
    return scene_.getJointState(scene_.jointAt(requireArticulation(), joint));
}

double MuJoCoSharedUrdfBackend::totalMass() const
{
    // ⚠ THIS articulation's mass, not the scene's. The portable contract is "mass realised by the
    // backend for this robot", cross-checked against its URDF; returning the whole world's mass
    // would make every multi-robot scene look like a mass error.
    return scene_.articulationMass(requireArticulation());
}

void MuJoCoSharedUrdfBackend::setJointTarget(size_t joint, ControlMode mode, double value)
{
    scene_.setJointTarget(scene_.jointAt(requireArticulation(), joint), mode, value);
}

void MuJoCoSharedUrdfBackend::setPositionGains(size_t joint, double hertz, double damping_ratio)
{
    scene_.setPositionGains(scene_.jointAt(requireArticulation(), joint), hertz, damping_ratio);
}

void MuJoCoSharedUrdfBackend::applyExternalWrench(size_t link, const Wrench& wrench)
{
    scene_.applyExternalWrench(scene_.linkAt(requireArticulation(), link), wrench);
}

bool MuJoCoSharedUrdfBackend::describeColliders(PhysicsColliderSet& out) const
{
    out = PhysicsColliderSet();
    if (!scene_.compiled())
        return false;
    scene_.describeColliders(articulation_, out);

    // ⚠ STRIP THE ARTICULATION PREFIX, or the id is qualified TWICE. The scene names colliders
    // "<stable_id>/<link>" and a vehicle's stable id is already "vehicle/Rover1", while
    // UrdfBotSimApi then prepends the vehicle name again — giving
    // "Rover1/vehicle/Rover1/DRV_LF_link", which matches no UrdfLinkPhysics selection. This facade
    // therefore reports links the same way the private-world backend does: bare, letting the one
    // caller that knows the vehicle name do the qualifying.
    //
    // Caught in the first in-sim run: 0 of 6 selected colliders matched, because this override did
    // not exist at all and the base class returned false.
    const std::string& prefix = scene_.stableId(articulation_);
    for (PhysicsColliderDescriptor& collider : out.colliders) {
        if (collider.stable_id.rfind(prefix + "/", 0) == 0)
            collider.stable_id.erase(0, prefix.size() + 1);
    }
    return true;
}

} // namespace urdf
