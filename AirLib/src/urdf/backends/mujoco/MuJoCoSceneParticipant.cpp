#include "urdf/backends/mujoco/MuJoCoSceneParticipant.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace urdf {
namespace {

bool sameTimestep(double first, double second)
{
    const double scale = std::max(1.0, std::max(std::fabs(first), std::fabs(second)));
    return std::fabs(first - second) <= 1.0e-12 * scale;
}

} // namespace

MuJoCoSceneParticipant::MuJoCoSceneParticipant(const MuJoCoPhysicsScene::Options& options)
    : scene_(options), fixed_timestep_(options.fixed_timestep)
{
}

void MuJoCoSceneParticipant::registerArticulationBody(
    const std::string& stable_id, MuJoCoPhysicsScene::ArticulationHandle articulation)
{
    if (stable_id.empty())
        throw std::invalid_argument("MuJoCoSceneParticipant: body stable_id is empty");
    if (!articulation.valid())
        throw std::invalid_argument("MuJoCoSceneParticipant: articulation handle is invalid");
    for (const BodyRecord& record : bodies_) {
        if (record.stable_id == stable_id)
            throw std::invalid_argument("MuJoCoSceneParticipant: duplicate body stable_id '" +
                                        stable_id + "'");
        if (record.articulation.index == articulation.index)
            throw std::invalid_argument(
                "MuJoCoSceneParticipant: articulation is already published as '" +
                record.stable_id + "'");
    }
    bodies_.push_back(BodyRecord{ stable_id, articulation });
}

void MuJoCoSceneParticipant::onManifestCommit(const msr::airlib::PhysicsManifestContext& context)
{
    for (const BodyRecord& record : bodies_) {
        if (context.candidate_manifest->find(record.stable_id) == nullptr)
            throw std::invalid_argument(
                "MuJoCoSceneParticipant: no manifest entry for published body '" +
                record.stable_id + "'");
    }

    // Every articulation has attached by now, so this is the one moment the shared model can be
    // compiled: earlier would freeze topology before the last robot joined, later would let a step
    // run against a model that does not exist.
    if (!scene_.compiled())
        scene_.compile();

    if (scene_.articulationCount() != bodies_.size())
        throw std::logic_error(
            "MuJoCoSceneParticipant: the shared scene holds articulations which publish no state; "
            "every one needs a manifest body so no body is invisible to the coordinator");
}

void MuJoCoSceneParticipant::requireStepMatchesFixedTimestep(double dt) const
{
    if (!sameTimestep(dt, fixed_timestep_))
        throw std::invalid_argument(
            "MuJoCoSceneParticipant: the coordinator's fixed dt differs from the shared MuJoCo "
            "scene's timestep. Configure the scene from the same authoritative step so one "
            "coordinated tick is exactly one mj_step.");
}

void MuJoCoSceneParticipant::onStepPrepare(const msr::airlib::PhysicsStepContext& context)
{
    requireStepMatchesFixedTimestep(context.dt);
}

void MuJoCoSceneParticipant::onStep(const msr::airlib::PhysicsStepContext& context)
{
    requireStepMatchesFixedTimestep(context.dt);
    scene_.step(context.dt);
}

void MuJoCoSceneParticipant::appendBodyStates(
    uint64_t reset_epoch, uint64_t step_sequence, uint64_t time_nanos,
    std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const
{
    for (const BodyRecord& record : bodies_) {
        const MuJoCoPhysicsScene::LinkHandle root = scene_.linkAt(record.articulation, 0);
        const LinkPose pose = scene_.getLinkPose(root);
        const Twist twist = scene_.getLinkTwist(root);

        msr::airlib::PhysicsBodyStateSnapshot snapshot;
        snapshot.stable_id = record.stable_id;
        snapshot.link_id = scene_.linkName(root);
        snapshot.authority = msr::airlib::PhysicsBodyAuthority::MuJoCo;
        snapshot.pose.position = { pose.position.x, pose.position.y, pose.position.z };
        snapshot.pose.orientation.x = pose.orientation.x;
        snapshot.pose.orientation.y = pose.orientation.y;
        snapshot.pose.orientation.z = pose.orientation.z;
        snapshot.pose.orientation.w = pose.orientation.w;
        snapshot.twist.linear = { twist.linear.x, twist.linear.y, twist.linear.z };
        snapshot.twist.angular = { twist.angular.x, twist.angular.y, twist.angular.z };

        // Solved in this very step by this very authority: same-time by construction.
        snapshot.source_reset_epoch = reset_epoch;
        snapshot.source_step_sequence = step_sequence;
        snapshot.source_time_nanos = time_nanos;
        states.push_back(std::move(snapshot));
    }
}

void MuJoCoSceneParticipant::collectStepBodyStates(
    const msr::airlib::PhysicsStepContext& context,
    std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const
{
    appendBodyStates(context.candidate_stamp.reset_epoch, context.candidate_stamp.step_sequence,
                     context.candidate_stamp.simulation_time_nanos, states);
}

void MuJoCoSceneParticipant::onResetRestore(const msr::airlib::PhysicsResetContext&)
{
    // No candidate world and no rollback: mj_resetData restores state exactly without touching the
    // compiled model, so unlike Box3D there is nothing that can fail halfway.
    scene_.reset();
}

void MuJoCoSceneParticipant::collectResetBodyStates(
    const msr::airlib::PhysicsResetContext& context,
    std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const
{
    appendBodyStates(context.candidate_stamp.reset_epoch, 0, 0, states);
}

} // namespace urdf
