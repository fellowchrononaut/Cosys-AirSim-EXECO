// The shared MuJoCo scene as one participant in the world physics coordinator.
//
// The MuJoCo counterpart of Box3DSceneParticipant, and deliberately the same shape: compile at
// manifest commit, advance exactly once per coordinated step, reset inside the one global
// transaction, and publish canonical body state for the bodies this scene OWNS.
//
// ⚠ Reset differs from Box3D's in kind, not just in code. Box3D has no rollback determinism, so its
// scene rebuilds a whole candidate world and swaps it in. MuJoCo separates the compiled model from
// its state, so `mj_resetData` is exact and cheap and there is nothing to roll back — which is why
// this participant has no prepare/commit candidate and cannot fail halfway.
#pragma once

#include "physics/PhysicsSceneCoordinator.hpp"
#include "urdf/backends/mujoco/MuJoCoPhysicsScene.hpp"

#include <string>
#include <vector>

namespace urdf {

class MuJoCoSceneParticipant : public msr::airlib::PhysicsSceneParticipant
{
public:
    explicit MuJoCoSceneParticipant(const MuJoCoPhysicsScene::Options& options);

    MuJoCoPhysicsScene& scene() { return scene_; }
    const MuJoCoPhysicsScene& scene() const { return scene_; }

    /// Declare which manifest body an articulation publishes state for. `stable_id` must match a
    /// manifest entry naming this participant, and the articulation must already be registered.
    void registerArticulationBody(const std::string& stable_id,
                                  MuJoCoPhysicsScene::ArticulationHandle articulation);

    void onManifestCommit(const msr::airlib::PhysicsManifestContext& context) override;
    void onStepPrepare(const msr::airlib::PhysicsStepContext& context) override;
    void onStep(const msr::airlib::PhysicsStepContext& context) override;
    void collectStepBodyStates(
        const msr::airlib::PhysicsStepContext& context,
        std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const override;

    void onResetRestore(const msr::airlib::PhysicsResetContext& context) override;
    void collectResetBodyStates(
        const msr::airlib::PhysicsResetContext& context,
        std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const override;

private:
    struct BodyRecord {
        std::string stable_id;
        MuJoCoPhysicsScene::ArticulationHandle articulation;
    };

    void requireStepMatchesFixedTimestep(double dt) const;
    void appendBodyStates(uint64_t reset_epoch, uint64_t step_sequence, uint64_t time_nanos,
                          std::vector<msr::airlib::PhysicsBodyStateSnapshot>& states) const;

    MuJoCoPhysicsScene scene_;
    std::vector<BodyRecord> bodies_;
    double fixed_timestep_ = 0.0;
};

} // namespace urdf
