// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "physics/PhysicsSceneCoordinator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace msr
{
namespace airlib
{
    PhysicsWorldSnapshot::PhysicsWorldSnapshot(
        const PhysicsCoordinatorStamp& stamp, uint64_t dt_nanos,
        std::vector<PhysicsBodyStateSnapshot> bodies)
        : stamp_(stamp), dt_nanos_(dt_nanos), bodies_(std::move(bodies))
    {
        std::sort(bodies_.begin(), bodies_.end(),
                  [](const PhysicsBodyStateSnapshot& lhs,
                     const PhysicsBodyStateSnapshot& rhs) {
                      return lhs.stable_id < rhs.stable_id;
                  });
        if (std::any_of(bodies_.begin(), bodies_.end(),
                        [](const PhysicsBodyStateSnapshot& body) {
                            return body.state_age_nanos != 0;
                        })) {
            timing_ = PhysicsSnapshotTiming::LaggedMixedTime;
        }
    }

    const PhysicsBodyStateSnapshot* PhysicsWorldSnapshot::find(
        const std::string& stable_id) const
    {
        const auto found = std::lower_bound(
            bodies_.begin(), bodies_.end(), stable_id,
            [](const PhysicsBodyStateSnapshot& body, const std::string& id) {
                return body.stable_id < id;
            });
        return found != bodies_.end() && found->stable_id == stable_id ? &*found : nullptr;
    }

    PhysicsSceneManifest::PhysicsSceneManifest(const PhysicsWorldIdentity& world, uint64_t revision,
                                               std::vector<PhysicsManifestEntry> entries)
        : world_(world), revision_(revision), entries_(std::move(entries))
    {
        std::sort(entries_.begin(), entries_.end(), [](const PhysicsManifestEntry& lhs,
                                                       const PhysicsManifestEntry& rhs) {
            return lhs.stable_id < rhs.stable_id;
        });
    }

    const PhysicsManifestEntry* PhysicsSceneManifest::find(const std::string& stable_id) const
    {
        const auto found = std::lower_bound(
            entries_.begin(), entries_.end(), stable_id,
            [](const PhysicsManifestEntry& entry, const std::string& id) {
                return entry.stable_id < id;
            });
        return found != entries_.end() && found->stable_id == stable_id ? &*found : nullptr;
    }

    PhysicsSceneCoordinator::PhysicsSceneCoordinator(Mode mode, PhysicsWorldIdentity world)
        : mode_(mode)
    {
        stamp_.world = world;
    }

    void PhysicsSceneCoordinator::registerParticipant(
        const std::string& stable_id, int order,
        std::shared_ptr<PhysicsSceneParticipant> participant)
    {
        if (manifest_ != nullptr)
            throw std::logic_error("PhysicsSceneCoordinator: participants are fixed after the first manifest commit");
        if (state_ != State::Empty && state_ != State::EditingManifest)
            throw std::logic_error("PhysicsSceneCoordinator: cannot register a participant during a transaction");
        if (stable_id.empty())
            throw std::invalid_argument("PhysicsSceneCoordinator: participant stable_id is empty");
        if (participant == nullptr)
            throw std::invalid_argument("PhysicsSceneCoordinator: participant is null");
        if (participantExists(stable_id))
            throw std::invalid_argument("PhysicsSceneCoordinator: duplicate participant stable_id '" + stable_id + "'");
        const bool instance_exists = std::any_of(
            participants_.begin(), participants_.end(),
            [&participant](const ParticipantRecord& record) {
                return record.participant.get() == participant.get();
            });
        if (instance_exists)
            throw std::invalid_argument("PhysicsSceneCoordinator: participant instance is already registered");

        participants_.push_back(ParticipantRecord{ stable_id, order, std::move(participant) });
        std::sort(participants_.begin(), participants_.end(),
                  [](const ParticipantRecord& lhs, const ParticipantRecord& rhs) {
                      if (lhs.order != rhs.order)
                          return lhs.order < rhs.order;
                      return lhs.stable_id < rhs.stable_id;
                  });
    }

    void PhysicsSceneCoordinator::beginManifestUpdate()
    {
        if (manifest_ != nullptr)
            throw std::logic_error("PhysicsSceneCoordinator: Phase-1 manifest topology is frozen after commit");
        requireState(State::Empty, "beginManifestUpdate");

        staged_entries_.clear();
        state_ = State::EditingManifest;
    }

    void PhysicsSceneCoordinator::stageManifestEntry(PhysicsManifestEntry entry)
    {
        requireState(State::EditingManifest, "stageManifestEntry");
        if (entry.stable_id.empty())
            throw std::invalid_argument("PhysicsSceneCoordinator: manifest stable_id is empty");
        if (!entry.participant_id.empty() && !participantExists(entry.participant_id)) {
            throw std::invalid_argument("PhysicsSceneCoordinator: manifest entry '" + entry.stable_id +
                                        "' names unknown participant '" + entry.participant_id + "'");
        }

        const std::string stable_id = entry.stable_id;
        const auto inserted = staged_entries_.emplace(stable_id, std::move(entry));
        if (!inserted.second)
            throw std::invalid_argument("PhysicsSceneCoordinator: duplicate manifest stable_id '" + stable_id + "'");
    }

    void PhysicsSceneCoordinator::discardManifestUpdate()
    {
        requireState(State::EditingManifest, "discardManifestUpdate");
        staged_entries_.clear();
        state_ = manifest_ == nullptr ? State::Empty : State::Ready;
    }

    std::shared_ptr<const PhysicsSceneManifest> PhysicsSceneCoordinator::commitManifest()
    {
        requireState(State::EditingManifest, "commitManifest");

        if (stamp_.manifest_revision == std::numeric_limits<uint64_t>::max())
            throw std::overflow_error("PhysicsSceneCoordinator: manifest revision overflow");

        std::vector<PhysicsManifestEntry> entries;
        entries.reserve(staged_entries_.size());
        for (const auto& item : staged_entries_)
            entries.push_back(item.second);

        PhysicsCoordinatorStamp candidate_stamp = stamp_;
        ++candidate_stamp.manifest_revision;
        auto candidate = std::shared_ptr<const PhysicsSceneManifest>(
            new PhysicsSceneManifest(stamp_.world, candidate_stamp.manifest_revision,
                                     std::move(entries)));
        const PhysicsManifestContext context{
            stamp_, candidate_stamp, manifest_, candidate
        };

        state_ = State::CommittingManifest;
        std::size_t entered_count = 0;
        if (enabled()) {
            try {
                for (const ParticipantRecord& record : participants_) {
                    // An entered prepare hook may mutate before throwing. Include it in rollback.
                    ++entered_count;
                    record.participant->onManifestPrepare(context);
                }
                for (const ParticipantRecord& record : participants_)
                    record.participant->onManifestCommit(context);
            }
            catch (...) {
                abortManifest(context, entered_count);
                state_ = State::Faulted;
                throw;
            }
        }

        // Publish the immutable baseline and revision only after every participant has committed.
        // Callback failure therefore cannot expose a candidate as authoritative coordinator state.
        manifest_ = candidate;
        stamp_ = candidate_stamp;
        staged_entries_.clear();

        if (enabled()) {
            for (const ParticipantRecord& record : participants_)
                record.participant->onManifestFinalize(context);
        }

        state_ = State::Ready;
        return manifest_;
    }

    bool PhysicsSceneCoordinator::step(double dt)
    {
        if (!enabled())
            return false;
        requireState(State::Ready, "step");
        if (manifest_ == nullptr)
            throw std::logic_error("PhysicsSceneCoordinator: cannot step without a committed manifest");
        if (!std::isfinite(dt) || dt <= 0.0)
            throw std::invalid_argument("PhysicsSceneCoordinator: step dt must be finite and positive");

        constexpr long double nanos_per_second = 1000000000.0L;
        constexpr long double uint64_limit = 18446744073709551616.0L; // 2^64
        const long double rounded_dt_nanos =
            std::round(static_cast<long double>(dt) * nanos_per_second);
        if (!std::isfinite(rounded_dt_nanos) || rounded_dt_nanos >= uint64_limit)
            throw std::overflow_error("PhysicsSceneCoordinator: step dt exceeds timestamp range");
        const uint64_t dt_nanos = static_cast<uint64_t>(rounded_dt_nanos);
        if (dt_nanos == 0)
            throw std::invalid_argument("PhysicsSceneCoordinator: step dt rounds to zero nanoseconds");
        if (stamp_.simulation_time_nanos > std::numeric_limits<uint64_t>::max() - dt_nanos)
            throw std::overflow_error("PhysicsSceneCoordinator: simulation time overflow");

        if (stamp_.step_sequence == std::numeric_limits<uint64_t>::max())
            throw std::overflow_error("PhysicsSceneCoordinator: step sequence overflow");

        PhysicsCoordinatorStamp candidate_stamp = stamp_;
        ++candidate_stamp.step_sequence;
        candidate_stamp.simulation_time_nanos += dt_nanos;
        PhysicsStepContext context{ candidate_stamp, manifest_, dt, dt_nanos,
                                    snapshot_, nullptr };

        state_ = State::Stepping;
        std::size_t entered_count = 0;
        try {
            for (const ParticipantRecord& record : participants_) {
                ++entered_count;
                record.participant->onStepPrepare(context);
            }
            for (const ParticipantRecord& record : participants_)
                record.participant->onStep(context);

            std::vector<PhysicsBodyStateSnapshot> states;
            for (const ParticipantRecord& record : participants_) {
                const std::size_t first_new_state = states.size();
                record.participant->collectStepBodyStates(context, states);
                for (std::size_t index = first_new_state; index < states.size(); ++index) {
                    const PhysicsManifestEntry* entry = manifest_->find(states[index].stable_id);
                    if (entry == nullptr) {
                        throw std::invalid_argument(
                            "PhysicsSceneCoordinator: participant '" + record.stable_id +
                            "' contributed state for unknown manifest body '" +
                            states[index].stable_id + "'");
                    }
                    if (!entry->participant_id.empty() &&
                        entry->participant_id != record.stable_id) {
                        throw std::invalid_argument(
                            "PhysicsSceneCoordinator: participant '" + record.stable_id +
                            "' contributed state owned by participant '" +
                            entry->participant_id + "'");
                    }
                }
            }
            context.candidate_snapshot =
                makeSnapshot(candidate_stamp, dt_nanos, std::move(states));

            for (const ParticipantRecord& record : participants_)
                record.participant->onStepCommit(context);
        }
        catch (...) {
            abortStep(context, entered_count);
            state_ = State::Faulted;
            throw;
        }

        stamp_ = candidate_stamp;
        snapshot_ = context.candidate_snapshot;
        for (const ParticipantRecord& record : participants_)
            record.participant->onStepFinalize(context);
        state_ = State::Ready;
        return true;
    }

    bool PhysicsSceneCoordinator::reset(const std::string& reason,
                                        const PhysicsResetPolicy& policy)
    {
        if (!enabled())
            return false;
        if (state_ != State::Ready && state_ != State::Faulted)
            throw std::logic_error("PhysicsSceneCoordinator: reset requires ready or faulted state");
        if (manifest_ == nullptr)
            throw std::logic_error("PhysicsSceneCoordinator: cannot reset without a committed manifest");

        if (stamp_.reset_epoch == std::numeric_limits<uint64_t>::max())
            throw std::overflow_error("PhysicsSceneCoordinator: reset epoch overflow");
        if (policy.presettle) {
            if (!std::isfinite(policy.presettle_dt) || policy.presettle_dt <= 0.0)
                throw std::invalid_argument(
                    "PhysicsSceneCoordinator: presettle dt must be finite and positive");
            if (policy.presettle_steps == 0)
                throw std::invalid_argument(
                    "PhysicsSceneCoordinator: presettle requires at least one step");
        }

        const PhysicsCoordinatorStamp previous_stamp = stamp_;
        PhysicsCoordinatorStamp candidate_stamp = stamp_;
        ++candidate_stamp.reset_epoch;
        candidate_stamp.step_sequence = 0;
        candidate_stamp.simulation_time_nanos = 0;
        const std::shared_ptr<const PhysicsWorldSnapshot> previous_snapshot = snapshot_;
        PhysicsResetContext context{
            previous_stamp, candidate_stamp, manifest_, reason, previous_snapshot, nullptr
        };

        // Publish the new epoch before touching participant state. If restore fails, stale work from
        // the old epoch must still remain invalid and the coordinator stays faulted.
        stamp_ = candidate_stamp;
        snapshot_.reset();
        state_ = State::Resetting;

        std::size_t entered_count = 0;
        try {
            for (const ParticipantRecord& record : participants_) {
                ++entered_count;
                record.participant->onResetPrepare(context);
            }
            for (const ParticipantRecord& record : participants_)
                record.participant->onResetRestore(context);

            // D9b2. The pre-roll belongs inside this transaction, before any baseline state is
            // collected: the settled configuration IS the new t=0 initial condition, so the
            // interval that produced it must never appear on the public timeline.
            if (policy.presettle)
                runPresettle(candidate_stamp, policy);

            std::vector<PhysicsBodyStateSnapshot> states;
            for (const ParticipantRecord& record : participants_) {
                const std::size_t first_new_state = states.size();
                record.participant->collectResetBodyStates(context, states);
                for (std::size_t index = first_new_state; index < states.size(); ++index) {
                    const PhysicsManifestEntry* entry = manifest_->find(states[index].stable_id);
                    if (entry == nullptr) {
                        throw std::invalid_argument(
                            "PhysicsSceneCoordinator: participant '" + record.stable_id +
                            "' contributed reset state for unknown manifest body '" +
                            states[index].stable_id + "'");
                    }
                    if (!entry->participant_id.empty() &&
                        entry->participant_id != record.stable_id) {
                        throw std::invalid_argument(
                            "PhysicsSceneCoordinator: participant '" + record.stable_id +
                            "' contributed reset state owned by participant '" +
                            entry->participant_id + "'");
                    }
                }
            }
            context.candidate_snapshot =
                makeSnapshot(candidate_stamp, 0, std::move(states));

            for (const ParticipantRecord& record : participants_)
                record.participant->onResetCommit(context);
        }
        catch (...) {
            abortReset(context, entered_count);
            state_ = State::Faulted;
            throw;
        }

        snapshot_ = context.candidate_snapshot;
        for (const ParticipantRecord& record : participants_)
            record.participant->onResetFinalize(context);
        state_ = State::Ready;
        return true;
    }

    bool PhysicsSceneCoordinator::participantExists(const std::string& stable_id) const
    {
        return std::any_of(participants_.begin(), participants_.end(),
                           [&stable_id](const ParticipantRecord& record) {
                               return record.stable_id == stable_id;
                           });
    }

    void PhysicsSceneCoordinator::requireState(State expected, const char* operation) const
    {
        if (state_ != expected)
            throw std::logic_error(std::string("PhysicsSceneCoordinator: ") + operation +
                                   " called in the wrong state");
    }

    void PhysicsSceneCoordinator::abortManifest(const PhysicsManifestContext& context,
                                                std::size_t entered_count) noexcept
    {
        while (entered_count > 0) {
            --entered_count;
            try {
                participants_[entered_count].participant->onManifestAbort(context);
            }
            catch (...) {
            }
        }
    }

    void PhysicsSceneCoordinator::abortStep(const PhysicsStepContext& context,
                                            std::size_t entered_count) noexcept
    {
        while (entered_count > 0) {
            --entered_count;
            try {
                participants_[entered_count].participant->onStepAbort(context);
            }
            catch (...) {
            }
        }
    }

    void PhysicsSceneCoordinator::runPresettle(const PhysicsCoordinatorStamp& baseline_stamp,
                                               const PhysicsResetPolicy& policy)
    {
        // A private, discarded timeline. Participants which need a monotonic stamp during pre-roll
        // get one, but nothing here is published: no snapshot is built and neither the commit nor
        // the finalize phase runs, because there is no committed state for them to describe.
        // A throw propagates to reset()'s handler, which aborts the whole transaction.
        const uint64_t dt_nanos = static_cast<uint64_t>(
            std::llround(policy.presettle_dt * 1000000000.0));
        PhysicsCoordinatorStamp presettle_stamp = baseline_stamp;

        for (uint64_t index = 0; index < policy.presettle_steps; ++index) {
            ++presettle_stamp.step_sequence;
            presettle_stamp.simulation_time_nanos += dt_nanos;

            PhysicsStepContext context{ presettle_stamp, manifest_, policy.presettle_dt,
                                        dt_nanos, nullptr, nullptr };
            context.presettling = true;

            for (const ParticipantRecord& record : participants_)
                record.participant->onStepPrepare(context);
            for (const ParticipantRecord& record : participants_)
                record.participant->onStep(context);
        }
    }

    void PhysicsSceneCoordinator::abortReset(const PhysicsResetContext& context,
                                             std::size_t entered_count) noexcept
    {
        while (entered_count > 0) {
            --entered_count;
            try {
                participants_[entered_count].participant->onResetAbort(context);
            }
            catch (...) {
            }
        }
    }

    std::shared_ptr<const PhysicsWorldSnapshot> PhysicsSceneCoordinator::makeSnapshot(
        const PhysicsCoordinatorStamp& stamp, uint64_t dt_nanos,
        std::vector<PhysicsBodyStateSnapshot> states) const
    {
        const auto finite_vector = [](const PhysicsCanonicalVector3& value) {
            return std::isfinite(value.x) && std::isfinite(value.y) &&
                   std::isfinite(value.z);
        };

        std::sort(states.begin(), states.end(),
                  [](const PhysicsBodyStateSnapshot& lhs,
                     const PhysicsBodyStateSnapshot& rhs) {
                      return lhs.stable_id < rhs.stable_id;
                  });

        for (std::size_t index = 0; index < states.size(); ++index) {
            PhysicsBodyStateSnapshot& body = states[index];
            if (body.stable_id.empty())
                throw std::invalid_argument(
                    "PhysicsSceneCoordinator: snapshot body stable_id is empty");
            if (index > 0 && states[index - 1].stable_id == body.stable_id) {
                throw std::invalid_argument(
                    "PhysicsSceneCoordinator: more than one authority contributed body '" +
                    body.stable_id + "'");
            }
            if (!finite_vector(body.pose.position) ||
                !finite_vector(body.twist.linear) || !finite_vector(body.twist.angular) ||
                !std::isfinite(body.pose.orientation.x) ||
                !std::isfinite(body.pose.orientation.y) ||
                !std::isfinite(body.pose.orientation.z) ||
                !std::isfinite(body.pose.orientation.w)) {
                throw std::invalid_argument(
                    "PhysicsSceneCoordinator: snapshot body '" + body.stable_id +
                    "' contains a non-finite pose or twist");
            }
            const PhysicsCanonicalQuaternion& orientation = body.pose.orientation;
            const double norm_squared = orientation.x * orientation.x +
                                        orientation.y * orientation.y +
                                        orientation.z * orientation.z +
                                        orientation.w * orientation.w;
            if (std::fabs(norm_squared - 1.0) > 1.0e-6) {
                throw std::invalid_argument(
                    "PhysicsSceneCoordinator: snapshot body '" + body.stable_id +
                    "' orientation is not a normalized quaternion");
            }
            if (body.source_reset_epoch != stamp.reset_epoch) {
                throw std::invalid_argument(
                    "PhysicsSceneCoordinator: snapshot body '" + body.stable_id +
                    "' belongs to a stale reset epoch");
            }
            if (body.source_step_sequence > stamp.step_sequence ||
                body.source_time_nanos > stamp.simulation_time_nanos) {
                throw std::invalid_argument(
                    "PhysicsSceneCoordinator: snapshot body '" + body.stable_id +
                    "' reports state from the future");
            }
            body.state_age_nanos = stamp.simulation_time_nanos - body.source_time_nanos;
        }

        return std::shared_ptr<const PhysicsWorldSnapshot>(
            new PhysicsWorldSnapshot(stamp, dt_nanos, std::move(states)));
    }
}
} // namespace msr::airlib
