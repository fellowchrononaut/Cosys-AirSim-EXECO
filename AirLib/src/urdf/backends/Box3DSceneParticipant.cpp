#include "urdf/backends/Box3DSceneParticipant.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace urdf {
namespace {

/// The coordinator's dt and the scene's fixed timestep must be the SAME number, not merely close.
/// A shared world that quietly accumulated a remainder would step a different number of times per
/// tick depending on rounding, which is exactly the irreproducibility fixed stepping exists to
/// remove.
bool sameTimestep(double first, double second)
{
    const double scale = std::max(1.0, std::max(std::fabs(first), std::fabs(second)));
    return std::fabs(first - second) <= 1.0e-12 * scale;
}

msr::airlib::PhysicsCanonicalVector3 toCanonical(const double value[3])
{
    msr::airlib::PhysicsCanonicalVector3 result;
    result.x = value[0];
    result.y = value[1];
    result.z = value[2];
    return result;
}

} // namespace

Box3DSceneParticipant::Box3DSceneParticipant(const b3urdf::Box3DSceneOptions& options)
    : scene_(options)
{
}

void Box3DSceneParticipant::registerRobotBody(const std::string& stable_id, size_t handle)
{
    if (stable_id.empty())
        throw std::invalid_argument("Box3DSceneParticipant: body stable_id is empty");
    for (const BodyRecord& record : bodies_) {
        if (record.stable_id == stable_id)
            throw std::invalid_argument("Box3DSceneParticipant: duplicate body stable_id '" +
                                        stable_id + "'");
        if (record.handle == handle)
            throw std::invalid_argument(
                "Box3DSceneParticipant: robot handle is already published as '" +
                record.stable_id + "'");
    }
    bodies_.push_back(BodyRecord{ stable_id, handle });
}

void Box3DSceneParticipant::onManifestCommit(const msr::airlib::PhysicsManifestContext& context)
{
    // Every participant has registered by now, so this is the one moment the shared world can be
    // built: earlier would freeze topology before the last robot joined, later would let a step
    // run against a scene that does not exist.
    for (const BodyRecord& record : bodies_) {
        const msr::airlib::PhysicsManifestEntry* entry =
            context.candidate_manifest->find(record.stable_id);
        if (entry == nullptr)
            throw std::invalid_argument(
                "Box3DSceneParticipant: no manifest entry for published body '" +
                record.stable_id + "'");
    }

    if (!scene_.isSealed())
        scene_.seal();

    if (scene_.robotCount() != bodies_.size())
        throw std::logic_error(
            "Box3DSceneParticipant: the shared scene holds robots which publish no state; every "
            "registered robot needs a manifest body so no body is invisible to the coordinator");
}

void Box3DSceneParticipant::requireStepMatchesFixedTimestep(double dt) const
{
    if (!sameTimestep(dt, scene_.options().fixed_timestep))
        throw std::invalid_argument(
            "Box3DSceneParticipant: the coordinator's fixed dt differs from the shared Box3D "
            "scene's fixed_timestep. Configure the scene from the same authoritative step so one "
            "coordinated tick is exactly one solver step.");
}

void Box3DSceneParticipant::onStepPrepare(const msr::airlib::PhysicsStepContext& context)
{
    requireStepMatchesFixedTimestep(context.dt);
}

void Box3DSceneParticipant::onStep(const msr::airlib::PhysicsStepContext& context)
{
    requireStepMatchesFixedTimestep(context.dt);
    // Exactly one b3World_Step per coordinated tick, for every robot at once. `stepOnce` rather
    // than `step(dt)` on purpose: the accumulator form could take zero or two steps for one tick.
    scene_.stepOnce();
}

void Box3DSceneParticipant::appendBodyStates(
    uint64_t reset_epoch, uint64_t step_sequence, uint64_t time_nanos,
    std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const
{
    for (const BodyRecord& record : bodies_) {
        const b3urdf::Box3DRobot& robot = scene_.robot(record.handle);
        const b3urdf::LinkState state = robot.linkState(0);

        msr::airlib::PhysicsBodyStateSnapshot snapshot;
        snapshot.stable_id = record.stable_id;
        snapshot.link_id = robot.linkName(0);
        snapshot.authority = msr::airlib::PhysicsBodyAuthority::Box3D;
        snapshot.pose.position = toCanonical(state.position);
        snapshot.pose.orientation.x = state.orientation[0];
        snapshot.pose.orientation.y = state.orientation[1];
        snapshot.pose.orientation.z = state.orientation[2];
        snapshot.pose.orientation.w = state.orientation[3];
        snapshot.twist.linear = toCanonical(state.linear_velocity);
        snapshot.twist.angular = toCanonical(state.angular_velocity);

        // Solved in this very step by this very authority: the state is same-time by construction.
        snapshot.source_reset_epoch = reset_epoch;
        snapshot.source_step_sequence = step_sequence;
        snapshot.source_time_nanos = time_nanos;
        states.push_back(std::move(snapshot));
    }
}

void Box3DSceneParticipant::collectStepBodyStates(
    const msr::airlib::PhysicsStepContext& context,
    std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const
{
    appendBodyStates(context.candidate_stamp.reset_epoch, context.candidate_stamp.step_sequence,
                     context.candidate_stamp.simulation_time_nanos, states);
}

void Box3DSceneParticipant::onResetPrepare(const msr::airlib::PhysicsResetContext&)
{
    // Build the replacement world without touching the live one, so another participant's failure
    // can still leave this scene exactly as it was.
    scene_.prepareReset();
}

void Box3DSceneParticipant::onResetRestore(const msr::airlib::PhysicsResetContext&)
{
    scene_.commitPreparedReset();
}

void Box3DSceneParticipant::collectResetBodyStates(
    const msr::airlib::PhysicsResetContext& context,
    std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const
{
    // The new epoch's t=0 — after any pre-settle, which the coordinator has already run.
    appendBodyStates(context.candidate_stamp.reset_epoch, 0, 0, states);
}

void Box3DSceneParticipant::onResetFinalize(const msr::airlib::PhysicsResetContext&) noexcept
{
    scene_.finalizePreparedReset();
}

void Box3DSceneParticipant::onResetAbort(const msr::airlib::PhysicsResetContext&)
{
    scene_.abortPreparedReset();
}

bool Box3DSceneParticipant::describeColliders(const std::string& stable_id,
                                             urdf::PhysicsColliderSet& out) const
{
    out = urdf::PhysicsColliderSet();
    for (const BodyRecord& body : bodies_)
        if (body.stable_id == stable_id) {
            scene_.describeColliders(body.handle, stable_id, out);
            return true;
        }
    return false;
}

} // namespace urdf
