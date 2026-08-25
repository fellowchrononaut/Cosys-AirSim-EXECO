// Focused, dependency-free tests for the Phase-1 coordinator core.

#include "physics/PhysicsSceneCoordinator.hpp"

#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using msr::airlib::PhysicsCoordinatorStamp;
using msr::airlib::PhysicsBodyStateSnapshot;
using msr::airlib::PhysicsManifestContext;
using msr::airlib::PhysicsManifestEntry;
using msr::airlib::PhysicsResetContext;
using msr::airlib::PhysicsSceneCoordinator;
using msr::airlib::PhysicsSceneManifest;
using msr::airlib::PhysicsSceneParticipant;
using msr::airlib::PhysicsStepContext;
using msr::airlib::PhysicsSnapshotTiming;
using msr::airlib::PhysicsWorldIdentity;

namespace
{
    void require(bool condition, const std::string& message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    template <typename TException>
    void requireThrows(const std::function<void()>& fn, const std::string& message)
    {
        try {
            fn();
        }
        catch (const TException&) {
            return;
        }
        throw std::runtime_error(message);
    }

    class RecordingParticipant : public PhysicsSceneParticipant
    {
    public:
        RecordingParticipant(std::string name, std::vector<std::string>& events)
            : name_(std::move(name)), events_(events)
        {
        }

        void onManifestPrepare(const PhysicsManifestContext& context) override
        {
            events_.push_back("manifest.prepare:" + name_);
            last_manifest_ = context.candidate_manifest;
            if (try_reentrant_manifest_edit_) {
                try {
                    coordinator_->discardManifestUpdate();
                }
                catch (const std::logic_error&) {
                    reentrant_manifest_edit_rejected_ = true;
                }
            }
            if (fail_manifest_prepare_)
                throw std::runtime_error("injected manifest prepare failure");
        }
        void onManifestCommit(const PhysicsManifestContext&) override
        {
            events_.push_back("manifest.commit:" + name_);
            if (fail_manifest_commit_)
                throw std::runtime_error("injected manifest commit failure");
        }
        void onManifestFinalize(const PhysicsManifestContext&) noexcept override
        {
            events_.push_back("manifest.finalize:" + name_);
        }
        void onManifestAbort(const PhysicsManifestContext&) override
        {
            events_.push_back("manifest.abort:" + name_);
        }
        void onStepPrepare(const PhysicsStepContext& context) override
        {
            events_.push_back(context.presettling ? "presettle.prepare:" + name_
                                                  : "step.prepare:" + name_);
            last_step_ = context.candidate_stamp;
            if (fail_step_prepare_)
                throw std::runtime_error("injected step prepare failure");
        }
        void onStep(const PhysicsStepContext& context) override
        {
            events_.push_back(context.presettling ? "presettle.step:" + name_ : "step:" + name_);
            if (context.presettling) {
                ++presettle_steps_seen_;
                last_presettle_stamp_ = context.candidate_stamp;
                last_presettle_dt_ = context.dt;
                if (fail_presettle_)
                    throw std::runtime_error("injected presettle failure");
            }
            if (fail_step_)
                throw std::runtime_error("injected step failure");
        }
        void collectStepBodyStates(const PhysicsStepContext& context,
                                   std::vector<PhysicsBodyStateSnapshot>& states) const override
        {
            if (state_id_.empty())
                return;
            PhysicsBodyStateSnapshot state;
            state.stable_id = state_id_;
            state.source_reset_epoch = stale_state_epoch_
                                           ? context.candidate_stamp.reset_epoch + 1
                                           : context.candidate_stamp.reset_epoch;
            state.source_step_sequence =
                lag_state_one_step_ && context.candidate_stamp.step_sequence > 0
                    ? context.candidate_stamp.step_sequence - 1
                    : context.candidate_stamp.step_sequence;
            state.source_time_nanos =
                lag_state_one_step_ && context.candidate_stamp.simulation_time_nanos >=
                                           context.dt_nanos
                    ? context.candidate_stamp.simulation_time_nanos - context.dt_nanos
                    : context.candidate_stamp.simulation_time_nanos;
            states.push_back(std::move(state));
        }
        void onStepCommit(const PhysicsStepContext&) override
        {
            events_.push_back("step.commit:" + name_);
            if (fail_step_commit_)
                throw std::runtime_error("injected step commit failure");
        }
        void onStepFinalize(const PhysicsStepContext&) noexcept override
        {
            events_.push_back("step.finalize:" + name_);
        }
        void onStepAbort(const PhysicsStepContext&) override
        {
            events_.push_back("step.abort:" + name_);
        }
        void onResetPrepare(const PhysicsResetContext& context) override
        {
            events_.push_back("reset.prepare:" + name_);
            last_reset_manifest_ = context.baseline_manifest;
            last_reset_previous_ = context.previous_stamp;
            last_reset_candidate_ = context.candidate_stamp;
            if (fail_reset_prepare_)
                throw std::runtime_error("injected reset prepare failure");
        }
        void onResetRestore(const PhysicsResetContext&) override
        {
            events_.push_back("reset.restore:" + name_);
            if (fail_reset_restore_)
                throw std::runtime_error("injected reset failure");
        }
        void collectResetBodyStates(const PhysicsResetContext& context,
                                    std::vector<PhysicsBodyStateSnapshot>& states) const override
        {
            if (state_id_.empty())
                return;
            PhysicsBodyStateSnapshot state;
            state.stable_id = state_id_;
            state.source_reset_epoch = context.candidate_stamp.reset_epoch;
            state.source_step_sequence = 0;
            state.source_time_nanos = 0;
            states.push_back(std::move(state));
        }
        void onResetCommit(const PhysicsResetContext&) override
        {
            events_.push_back("reset.commit:" + name_);
            if (fail_reset_commit_)
                throw std::runtime_error("injected reset commit failure");
        }
        void onResetFinalize(const PhysicsResetContext&) noexcept override
        {
            events_.push_back("reset.finalize:" + name_);
        }
        void onResetAbort(const PhysicsResetContext&) override
        {
            events_.push_back("reset.abort:" + name_);
        }

        PhysicsSceneCoordinator* coordinator_ = nullptr;
        bool try_reentrant_manifest_edit_ = false;
        bool reentrant_manifest_edit_rejected_ = false;
        bool fail_manifest_prepare_ = false;
        bool fail_manifest_commit_ = false;
        bool fail_step_prepare_ = false;
        bool fail_step_ = false;
        bool fail_step_commit_ = false;
        bool fail_reset_prepare_ = false;
        bool fail_reset_restore_ = false;
        bool fail_reset_commit_ = false;
        std::string state_id_;
        bool lag_state_one_step_ = false;
        bool fail_presettle_ = false;
        int presettle_steps_seen_ = 0;
        double last_presettle_dt_ = 0.0;
        PhysicsCoordinatorStamp last_presettle_stamp_;
        bool stale_state_epoch_ = false;
        std::shared_ptr<const PhysicsSceneManifest> last_manifest_;
        std::shared_ptr<const PhysicsSceneManifest> last_reset_manifest_;
        PhysicsCoordinatorStamp last_step_;
        PhysicsCoordinatorStamp last_reset_previous_;
        PhysicsCoordinatorStamp last_reset_candidate_;

    private:
        std::string name_;
        std::vector<std::string>& events_;
    };

    PhysicsManifestEntry entry(const std::string& id, const std::string& owner,
                               const std::string& initial = std::string())
    {
        PhysicsManifestEntry result;
        result.stable_id = id;
        result.participant_id = owner;
        result.source_id = "test/" + id;
        if (!initial.empty())
            result.baseline_properties["initial"] = initial;
        return result;
    }

    void testLegacyIsNoOp()
    {
        std::vector<std::string> events;
        auto participant = std::make_shared<RecordingParticipant>("legacy", events);
        PhysicsSceneCoordinator coordinator(PhysicsSceneCoordinator::Mode::Legacy, { 7, 3 });
        coordinator.registerParticipant("legacy", 0, participant);
        coordinator.beginManifestUpdate();
        coordinator.stageManifestEntry(entry("body", "legacy", "spawn"));
        const auto manifest = coordinator.commitManifest();

        require(manifest->revision() == 1, "legacy metadata manifest did not commit");
        require(events.empty(), "legacy manifest commit invoked a participant");
        require(!coordinator.step(-1.0), "legacy step was handled");
        require(!coordinator.reset(), "legacy reset was handled");
        require(events.empty(), "legacy step/reset invoked a participant");
        require(coordinator.stamp().step_sequence == 0, "legacy step changed sequence");
        require(coordinator.stamp().reset_epoch == 0, "legacy reset changed epoch");
        require(coordinator.stamp().simulation_time_nanos == 0,
                "legacy step/reset changed simulation time");
    }

    void testDeterministicOrderAndCounters()
    {
        std::vector<std::string> events;
        auto z = std::make_shared<RecordingParticipant>("z", events);
        auto b = std::make_shared<RecordingParticipant>("b", events);
        auto a = std::make_shared<RecordingParticipant>("a", events);
        PhysicsSceneCoordinator coordinator(PhysicsSceneCoordinator::Mode::SingleBackend, { 42, 5 });
        a->coordinator_ = &coordinator;
        a->try_reentrant_manifest_edit_ = true;

        coordinator.registerParticipant("z", 10, z);
        coordinator.registerParticipant("b", 0, b);
        coordinator.registerParticipant("a", 0, a);
        coordinator.beginManifestUpdate();
        coordinator.stageManifestEntry(entry("z_body", "z"));
        coordinator.stageManifestEntry(entry("a_body", "a"));
        coordinator.stageManifestEntry(entry("b_body", "b"));
        coordinator.commitManifest();

        const std::vector<std::string> expected_manifest{
            "manifest.prepare:a", "manifest.prepare:b", "manifest.prepare:z",
            "manifest.commit:a", "manifest.commit:b", "manifest.commit:z",
            "manifest.finalize:a", "manifest.finalize:b", "manifest.finalize:z"
        };
        require(events == expected_manifest, "participants were not ordered by (order, stable_id)");
        require(a->reentrant_manifest_edit_rejected_,
                "manifest callback re-entered the staging transaction");
        events.clear();

        const PhysicsCoordinatorStamp& retained_pre_step_stamp = coordinator.stamp();
        require(coordinator.step(0.003), "enabled step was not handled");
        const std::vector<std::string> expected_step{
            "step.prepare:a", "step.prepare:b", "step.prepare:z",
            "step:a", "step:b", "step:z",
            "step.commit:a", "step.commit:b", "step.commit:z",
            "step.finalize:a", "step.finalize:b", "step.finalize:z"
        };
        require(events == expected_step, "step lifecycle order is not deterministic");
        require(coordinator.stamp().world == PhysicsWorldIdentity{ 42, 5 }, "world identity changed");
        require(coordinator.stamp().manifest_revision == 1, "manifest revision is wrong");
        require(coordinator.stamp().step_sequence == 1, "successful step did not advance sequence");
        require(coordinator.stamp().simulation_time_nanos == 3000000,
                "successful step did not advance logical simulation time by 3 ms");
        require(retained_pre_step_stamp.step_sequence == 0 &&
                    retained_pre_step_stamp.simulation_time_nanos == 0,
                "retained stamp snapshot changed after a later transaction");
        require(a->last_step_.step_sequence == 1, "participant did not receive candidate sequence");
        require(a->last_step_.simulation_time_nanos == 3000000,
                "participant did not receive candidate simulation time");
        require(coordinator.snapshot() != nullptr &&
                    coordinator.snapshot()->stamp().step_sequence == 1 &&
                    coordinator.snapshot()->dtNanos() == 3000000,
                "successful step did not atomically publish its snapshot stamp");
    }

    void testImmutableManifestBaselineAndTopologyFreeze()
    {
        std::vector<std::string> events;
        auto participant = std::make_shared<RecordingParticipant>("scene", events);
        PhysicsSceneCoordinator coordinator(PhysicsSceneCoordinator::Mode::SingleBackend, { 11, 1 });
        coordinator.registerParticipant("scene", 0, participant);

        PhysicsManifestEntry staged = entry("b", "scene", "first");
        coordinator.beginManifestUpdate();
        coordinator.stageManifestEntry(staged);
        coordinator.stageManifestEntry(entry("a", "scene", "alpha"));
        staged.baseline_properties["initial"] = "mutated-after-stage";
        const auto first = coordinator.commitManifest();

        require(first->entries().size() == 2, "manifest lost an entry");
        require(first->entries()[0].stable_id == "a" && first->entries()[1].stable_id == "b",
                "manifest entries are not sorted");
        require(first->find("b")->baseline_properties.at("initial") == "first",
                "staging alias mutated committed baseline");

        requireThrows<std::logic_error>([&]() { coordinator.beginManifestUpdate(); },
                                        "Phase-1 manifest topology was not frozen after commit");
        require(coordinator.manifest() == first && coordinator.stamp().manifest_revision == 1,
                "rejected topology change altered the committed baseline");

        coordinator.step(0.003);
        coordinator.reset("unit-test");
        require(participant->last_reset_manifest_ == first,
                "reset did not receive the exact committed baseline object");
        require(participant->last_reset_previous_.step_sequence == 1,
                "reset previous stamp lost the committed step");
        require(participant->last_reset_candidate_.reset_epoch == 1 &&
                    participant->last_reset_candidate_.step_sequence == 0,
                "reset candidate counters are wrong");
        require(coordinator.stamp().reset_epoch == 1 && coordinator.stamp().step_sequence == 0,
                "successful reset counters are wrong");
        require(coordinator.stamp().simulation_time_nanos == 0,
                "successful reset did not return logical simulation time to zero");
        require(coordinator.snapshot() != nullptr &&
                    coordinator.snapshot()->stamp().reset_epoch == 1 &&
                    coordinator.snapshot()->stamp().simulation_time_nanos == 0,
                "successful reset did not publish a fresh zero-time snapshot");
    }

    void testManifestFailuresAreAtomic()
    {
        {
            std::vector<std::string> events;
            auto a = std::make_shared<RecordingParticipant>("a", events);
            auto b = std::make_shared<RecordingParticipant>("b", events);
            PhysicsSceneCoordinator coordinator(PhysicsSceneCoordinator::Mode::SingleBackend,
                                                { 21, 1 });
            coordinator.registerParticipant("a", 0, a);
            coordinator.registerParticipant("b", 0, b);
            b->fail_manifest_prepare_ = true;
            coordinator.beginManifestUpdate();
            coordinator.stageManifestEntry(entry("body", "a"));

            requireThrows<std::runtime_error>([&]() { coordinator.commitManifest(); },
                                              "manifest prepare failure did not escape");
            const std::vector<std::string> expected{
                "manifest.prepare:a", "manifest.prepare:b",
                "manifest.abort:b", "manifest.abort:a"
            };
            require(events == expected,
                    "manifest prepare failure did not abort entered participants in reverse order");
            require(coordinator.faulted(), "manifest prepare failure did not fault coordinator");
            require(coordinator.manifest() == nullptr &&
                        coordinator.stamp().manifest_revision == 0,
                    "failed manifest prepare published the candidate baseline");
        }

        {
            std::vector<std::string> events;
            auto a = std::make_shared<RecordingParticipant>("a", events);
            auto b = std::make_shared<RecordingParticipant>("b", events);
            PhysicsSceneCoordinator coordinator(PhysicsSceneCoordinator::Mode::SingleBackend,
                                                { 22, 1 });
            coordinator.registerParticipant("a", 0, a);
            coordinator.registerParticipant("b", 0, b);
            b->fail_manifest_commit_ = true;
            coordinator.beginManifestUpdate();
            coordinator.stageManifestEntry(entry("body", "b"));

            requireThrows<std::runtime_error>([&]() { coordinator.commitManifest(); },
                                              "manifest commit failure did not escape");
            const std::vector<std::string> expected{
                "manifest.prepare:a", "manifest.prepare:b",
                "manifest.commit:a", "manifest.commit:b",
                "manifest.abort:b", "manifest.abort:a"
            };
            require(events == expected,
                    "manifest commit failure did not roll back in deterministic reverse order");
            require(coordinator.faulted(), "manifest commit failure did not fault coordinator");
            require(coordinator.manifest() == nullptr &&
                        coordinator.stamp().manifest_revision == 0,
                    "failed manifest commit became authoritative coordinator state");
        }
    }

    void testValidationAndFailureScaffolding()
    {
        std::vector<std::string> events;
        auto a = std::make_shared<RecordingParticipant>("a", events);
        auto b = std::make_shared<RecordingParticipant>("b", events);
        PhysicsSceneCoordinator coordinator(PhysicsSceneCoordinator::Mode::MixedBackendExperimental,
                                            { 99, 4 });
        coordinator.registerParticipant("b", 0, b);
        coordinator.registerParticipant("a", 0, a);
        requireThrows<std::invalid_argument>(
            [&]() { coordinator.registerParticipant("a", 1, a); },
            "duplicate participant was accepted");
        requireThrows<std::invalid_argument>(
            [&]() { coordinator.registerParticipant("alias", 1, a); },
            "one participant instance was accepted under two ids");

        coordinator.beginManifestUpdate();
        requireThrows<std::invalid_argument>(
            [&]() { coordinator.stageManifestEntry(entry("orphan", "missing")); },
            "unknown manifest participant was accepted");
        coordinator.stageManifestEntry(entry("body", "a"));
        coordinator.commitManifest();
        events.clear();

        b->fail_step_ = true;
        requireThrows<std::runtime_error>([&]() { coordinator.step(0.003); },
                                          "injected step failure did not escape");
        require(coordinator.faulted(), "partial step did not fault coordinator");
        require(coordinator.stamp().step_sequence == 0,
                "failed step was exposed as a committed sequence");
        require(coordinator.stamp().simulation_time_nanos == 0,
                "failed step was exposed as committed simulation time");
        const std::vector<std::string> expected_step_tail{
            "step.prepare:a", "step.prepare:b", "step:a", "step:b",
            "step.abort:b", "step.abort:a"
        };
        require(events == expected_step_tail, "step abort did not run in reverse prepared order");

        events.clear();
        b->fail_step_ = false;
        b->fail_reset_prepare_ = true;
        requireThrows<std::runtime_error>([&]() { coordinator.reset("prepare-failure"); },
                                          "injected reset prepare failure did not escape");
        require(coordinator.faulted(), "failed reset prepare did not leave coordinator faulted");
        require(coordinator.stamp().reset_epoch == 1 &&
                    coordinator.stamp().simulation_time_nanos == 0,
                "failed reset prepare did not publish a zero-time fresh epoch");
        const std::vector<std::string> expected_reset_prepare{
            "reset.prepare:a", "reset.prepare:b", "reset.abort:b", "reset.abort:a"
        };
        require(events == expected_reset_prepare,
                "reset prepare failure did not abort entered participants in reverse order");

        events.clear();
        b->fail_reset_prepare_ = false;
        b->fail_reset_restore_ = true;
        requireThrows<std::runtime_error>([&]() { coordinator.reset("recover"); },
                                          "injected reset failure did not escape");
        require(coordinator.faulted(), "failed reset did not leave coordinator faulted");
        require(coordinator.stamp().reset_epoch == 2 && coordinator.stamp().step_sequence == 0,
                "failed reset did not invalidate the old epoch");
        const std::vector<std::string> expected_reset_restore{
            "reset.prepare:a", "reset.prepare:b",
            "reset.restore:a", "reset.restore:b",
            "reset.abort:b", "reset.abort:a"
        };
        require(events == expected_reset_restore,
                "reset restore failure did not abort in reverse prepared order");

        events.clear();
        b->fail_reset_restore_ = false;
        require(coordinator.reset("retry"), "fault recovery reset was not handled");
        require(!coordinator.faulted(), "successful retry did not recover coordinator");
        require(coordinator.stamp().reset_epoch == 3, "retry did not allocate a fresh epoch");

        events.clear();
        b->fail_step_prepare_ = true;
        requireThrows<std::runtime_error>([&]() { coordinator.step(0.003); },
                                          "injected step prepare failure did not escape");
        const std::vector<std::string> expected_step_prepare{
            "step.prepare:a", "step.prepare:b", "step.abort:b", "step.abort:a"
        };
        require(events == expected_step_prepare,
                "step prepare failure did not abort entered participants in reverse order");
        require(coordinator.faulted() && coordinator.stamp().step_sequence == 0 &&
                    coordinator.stamp().simulation_time_nanos == 0,
                "failed step prepare changed committed logical time");
        requireThrows<std::logic_error>([&]() { coordinator.registerParticipant("late", 0, a); },
                                        "participant was registered after manifest commit");
    }

    void testCommitFailuresAbortBeforeFinalize()
    {
        std::vector<std::string> events;
        auto a = std::make_shared<RecordingParticipant>("a", events);
        auto b = std::make_shared<RecordingParticipant>("b", events);
        PhysicsSceneCoordinator coordinator(PhysicsSceneCoordinator::Mode::SingleBackend,
                                            { 120, 2 });
        coordinator.registerParticipant("a", 0, a);
        coordinator.registerParticipant("b", 0, b);
        coordinator.beginManifestUpdate();
        coordinator.stageManifestEntry(entry("body-a", "a"));
        coordinator.stageManifestEntry(entry("body-b", "b"));
        coordinator.commitManifest();
        events.clear();

        b->fail_step_commit_ = true;
        requireThrows<std::runtime_error>([&]() { coordinator.step(0.003); },
                                          "step commit failure did not escape");
        const std::vector<std::string> expected_step{
            "step.prepare:a", "step.prepare:b", "step:a", "step:b",
            "step.commit:a", "step.commit:b", "step.abort:b", "step.abort:a"
        };
        require(events == expected_step,
                "step commit failure finalized or failed to roll back prepared participants");
        require(coordinator.faulted() && coordinator.stamp().step_sequence == 0,
                "failed step commit published its candidate stamp");

        b->fail_step_commit_ = false;
        events.clear();
        require(coordinator.reset("recover-after-step-commit"),
                "reset did not recover a step commit failure");
        events.clear();

        b->fail_reset_commit_ = true;
        requireThrows<std::runtime_error>([&]() { coordinator.reset("commit-failure"); },
                                          "reset commit failure did not escape");
        const std::vector<std::string> expected_reset{
            "reset.prepare:a", "reset.prepare:b",
            "reset.restore:a", "reset.restore:b",
            "reset.commit:a", "reset.commit:b",
            "reset.abort:b", "reset.abort:a"
        };
        require(events == expected_reset,
                "reset commit failure finalized or failed to invoke reverse rollback");
        require(coordinator.faulted() && coordinator.stamp().reset_epoch == 2 &&
                    coordinator.stamp().step_sequence == 0 &&
                    coordinator.stamp().simulation_time_nanos == 0,
                "failed reset commit did not retain its fresh zero-time invalidation epoch");
        require(coordinator.snapshot() == nullptr,
                "failed reset exposed a snapshot from the invalidated old epoch");
    }

    void testSnapshotAuthorityLagAndValidation()
    {
        std::vector<std::string> events;
        auto a = std::make_shared<RecordingParticipant>("a", events);
        auto b = std::make_shared<RecordingParticipant>("b", events);
        a->state_id_ = "body/a";
        b->state_id_ = "body/b";
        b->lag_state_one_step_ = true;

        PhysicsSceneCoordinator coordinator(
            PhysicsSceneCoordinator::Mode::MixedBackendExperimental, { 300, 9 });
        coordinator.registerParticipant("a", 0, a);
        coordinator.registerParticipant("b", 0, b);
        coordinator.beginManifestUpdate();
        coordinator.stageManifestEntry(entry("body/a", "a"));
        coordinator.stageManifestEntry(entry("body/b", "b"));
        coordinator.commitManifest();

        require(coordinator.step(0.003), "snapshot step was not handled");
        const auto snapshot = coordinator.snapshot();
        require(snapshot != nullptr && snapshot->bodies().size() == 2,
                "participants did not contribute one immutable state each");
        require(snapshot->bodies()[0].stable_id == "body/a" &&
                    snapshot->bodies()[1].stable_id == "body/b",
                "snapshot bodies are not sorted by stable id");
        require(snapshot->timing() == PhysicsSnapshotTiming::LaggedMixedTime,
                "lagged source state was labeled synchronized");
        require(snapshot->find("body/a")->state_age_nanos == 0 &&
                    snapshot->find("body/b")->state_age_nanos == 3000000,
                "snapshot state age was not computed from source and commit time");

        require(coordinator.reset("snapshot-reset"), "snapshot reset failed");
        require(coordinator.snapshot()->timing() == PhysicsSnapshotTiming::Synchronized &&
                    coordinator.snapshot()->find("body/b")->source_reset_epoch == 1,
                "reset snapshot retained lag or an old source epoch");

        b->stale_state_epoch_ = true;
        requireThrows<std::invalid_argument>([&]() { coordinator.step(0.003); },
                                             "stale-epoch body state was accepted");
        require(coordinator.faulted() && coordinator.snapshot()->stamp().reset_epoch == 1 &&
                    coordinator.snapshot()->stamp().step_sequence == 0,
                "failed snapshot validation published candidate state");
    }

    void testGlobalResetPresettle()
    {
        using msr::airlib::PhysicsResetPolicy;

        std::vector<std::string> events;
        PhysicsSceneCoordinator coordinator(PhysicsSceneCoordinator::Mode::SingleBackend,
                                            PhysicsWorldIdentity{ 9, 1 });
        auto a = std::make_shared<RecordingParticipant>("a", events);
        a->state_id_ = "body/a";
        coordinator.registerParticipant("scene/a", 0, a);
        coordinator.beginManifestUpdate();
        coordinator.stageManifestEntry(entry("body/a", "scene/a"));
        coordinator.commitManifest();

        require(coordinator.step(0.003), "pre-reset step failed");
        require(coordinator.step(0.003), "pre-reset step failed");

        PhysicsResetPolicy policy;
        policy.presettle = true;
        policy.presettle_dt = 0.003;
        policy.presettle_steps = 4;

        events.clear();
        require(coordinator.reset("presettle-reset", policy), "presettle reset failed");

        require(a->presettle_steps_seen_ == 4, "pre-roll did not advance every participant");
        require(a->last_presettle_dt_ == 0.003, "pre-roll used a dt other than the world's");

        // The pre-roll is deleted from the public timeline: the settled state is t=0.
        const auto stamp = coordinator.stamp();
        require(stamp.reset_epoch == 1 && stamp.step_sequence == 0 &&
                    stamp.simulation_time_nanos == 0,
                "pre-settle leaked into the public step sequence or simulation time");
        require(coordinator.snapshot()->stamp().step_sequence == 0 &&
                    coordinator.snapshot()->stamp().simulation_time_nanos == 0,
                "the published baseline snapshot was not stamped at t=0");

        // Nothing may be published during the pre-roll: no commit, no finalize, no snapshot.
        for (const std::string& event : events) {
            require(event != "step.commit:a" && event != "step.finalize:a",
                    "pre-settle published a step commit");
        }
        const std::string order =
            "reset.prepare:a|reset.restore:a|presettle.prepare:a|presettle.step:a|"
            "presettle.prepare:a|presettle.step:a|presettle.prepare:a|presettle.step:a|"
            "presettle.prepare:a|presettle.step:a|reset.commit:a|reset.finalize:a";
        std::string actual;
        for (const std::string& event : events)
            actual += (actual.empty() ? "" : "|") + event;
        require(actual == order, "unexpected pre-settle phase order: " + actual);

        // The pre-roll's own stamp is monotonic for participants that need one, and discarded.
        require(a->last_presettle_stamp_.step_sequence == 4 &&
                    a->last_presettle_stamp_.reset_epoch == 1,
                "pre-roll stamp was not monotonic inside the new epoch");

        // Ordinary stepping continues from the settled t=0 state.
        require(coordinator.step(0.003), "post-presettle step failed");
        require(coordinator.stamp().step_sequence == 1, "post-presettle step sequence restarted wrong");

        // A failure inside the pre-roll aborts the whole reset rather than publishing a
        // half-settled world.
        a->fail_presettle_ = true;
        requireThrows<std::runtime_error>([&]() { coordinator.reset("failing-presettle", policy); },
                                          "a failing pre-settle was not propagated");
        require(coordinator.faulted(), "a failing pre-settle left the coordinator usable");

        // Policy validation.
        PhysicsSceneCoordinator second(PhysicsSceneCoordinator::Mode::SingleBackend,
                                       PhysicsWorldIdentity{ 10, 1 });
        auto b = std::make_shared<RecordingParticipant>("b", events);
        second.registerParticipant("scene/b", 0, b);
        second.beginManifestUpdate();
        second.commitManifest();
        PhysicsResetPolicy invalid;
        invalid.presettle = true;
        invalid.presettle_dt = 0.0;
        invalid.presettle_steps = 1;
        requireThrows<std::invalid_argument>([&]() { second.reset("bad-dt", invalid); },
                                             "a non-positive pre-settle dt was accepted");
        invalid.presettle_dt = 0.003;
        invalid.presettle_steps = 0;
        requireThrows<std::invalid_argument>([&]() { second.reset("bad-steps", invalid); },
                                             "a zero-step pre-settle was accepted");
    }
}

int main()
{
    try {
        testLegacyIsNoOp();
        testDeterministicOrderAndCounters();
        testImmutableManifestBaselineAndTopologyFreeze();
        testManifestFailuresAreAtomic();
        testValidationAndFailureScaffolding();
        testCommitFailuresAbortBeforeFinalize();
        testSnapshotAuthorityLagAndValidation();
        testGlobalResetPresettle();
        std::cout << "PhysicsSceneCoordinatorTests: PASS\n";
        return 0;
    }
    catch (const std::exception& ex) {
        std::cerr << "PhysicsSceneCoordinatorTests: FAIL: " << ex.what() << "\n";
        return 1;
    }
}
