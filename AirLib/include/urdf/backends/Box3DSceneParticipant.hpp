// The shared Box3D world as one participant in the world physics coordinator.
//
// This is the seam that makes Phase 3 real rather than a spike: `Box3DPhysicsScene` knows how to
// hold every Box3D robot in one `b3World`, and this class gives that scene the coordinator's
// transaction shape — build at manifest commit, advance exactly once per coordinated step,
// rebuild atomically inside the one global reset, and publish canonical body state.
//
// ⚠ It contributes the state of bodies it OWNS. A vehicle api reads that published snapshot (or,
// today, reads its own backend handle after the step); it must never step or reset the scene, or
// the shared world would advance once per robot instead of once per tick.
#pragma once

#include "physics/PhysicsSceneCoordinator.hpp"
#include "urdf/backends/Box3DPhysicsScene.hpp"

#include <string>
#include <vector>

namespace urdf {

class Box3DSceneParticipant : public msr::airlib::PhysicsSceneParticipant
{
public:
    explicit Box3DSceneParticipant(const b3urdf::Box3DSceneOptions& options);

    b3urdf::Box3DPhysicsScene& scene() { return scene_; }
    const b3urdf::Box3DPhysicsScene& scene() const { return scene_; }

    /// Declare which manifest body a registered robot publishes state for.
    ///
    /// `stable_id` must match a manifest entry whose `participant_id` names this participant, and
    /// `handle` must be the value `Box3DPhysicsScene::addRobot` returned. Registering the same
    /// stable id or the same handle twice is rejected: two entries for one solver body would make
    /// the coordinator publish one body's state under two identities.
    void registerRobotBody(const std::string& stable_id, size_t handle);

    /// Describe one registered robot's colliders, looked up by the stable id the manifest uses.
    ///
    /// ⚠ This participant is the ONLY place that holds the stable id <-> handle mapping, so it is
    /// the only level at which a collider set can be correctly named. Returns false for an id this
    /// participant does not own.
    bool describeColliders(const std::string& stable_id, urdf::PhysicsColliderSet& out) const;

    void onManifestCommit(const msr::airlib::PhysicsManifestContext& context) override;
    void onStepPrepare(const msr::airlib::PhysicsStepContext& context) override;
    void onStep(const msr::airlib::PhysicsStepContext& context) override;
    void collectStepBodyStates(
        const msr::airlib::PhysicsStepContext& context,
        std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const override;

    void onResetPrepare(const msr::airlib::PhysicsResetContext& context) override;
    void onResetRestore(const msr::airlib::PhysicsResetContext& context) override;
    void collectResetBodyStates(
        const msr::airlib::PhysicsResetContext& context,
        std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const override;
    void onResetFinalize(const msr::airlib::PhysicsResetContext& context) noexcept override;
    void onResetAbort(const msr::airlib::PhysicsResetContext& context) override;

private:
    struct BodyRecord {
        std::string stable_id;
        size_t handle = 0;
    };

    void requireStepMatchesFixedTimestep(double dt) const;
    void appendBodyStates(uint64_t reset_epoch, uint64_t step_sequence, uint64_t time_nanos,
                          std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const;

    b3urdf::Box3DPhysicsScene scene_;
    std::vector<BodyRecord> bodies_;
};

} // namespace urdf
