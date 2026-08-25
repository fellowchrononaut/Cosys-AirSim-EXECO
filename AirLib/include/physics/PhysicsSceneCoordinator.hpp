// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_PhysicsSceneCoordinator_hpp
#define airsim_core_PhysicsSceneCoordinator_hpp

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace msr
{
namespace airlib
{
    /** Stable identity for one simulator-world incarnation.
     *
     * `id` identifies the logical world. `revision` lets an owner distinguish a later PIE/session
     * incarnation which deliberately reuses that logical id. The coordinator never invents either
     * value: the world owner supplies both, so experiment metadata remains reproducible.
     */
    struct PhysicsWorldIdentity
    {
        uint64_t id = 0;
        uint64_t revision = 0;

        bool operator==(const PhysicsWorldIdentity& other) const
        {
            return id == other.id && revision == other.revision;
        }
    };

    /** The generation counters which travel with coordinator-owned state.
     *
     * A successful manifest commit advances `manifest_revision`. A reset advances `reset_epoch`
     * before any participant is restored and returns `step_sequence` and logical simulation time
     * to zero. A successful step advances sequence and time together; a partially failed step does
     * not masquerade as committed state.
     */
    struct PhysicsCoordinatorStamp
    {
        PhysicsWorldIdentity world;
        uint64_t manifest_revision = 0;
        uint64_t reset_epoch = 0;
        uint64_t step_sequence = 0;
        uint64_t simulation_time_nanos = 0;
    };

    /** Canonical coordinator frame: SI units, right-handed, Z-up, quaternion `(x,y,z,w)`. */
    struct PhysicsCanonicalVector3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct PhysicsCanonicalQuaternion
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 1.0;
    };

    struct PhysicsCanonicalPose
    {
        PhysicsCanonicalVector3 position;
        PhysicsCanonicalQuaternion orientation;
    };

    struct PhysicsCanonicalTwist
    {
        PhysicsCanonicalVector3 linear;
        PhysicsCanonicalVector3 angular;
    };

    enum class PhysicsBodyAuthority : uint8_t
    {
        Chaos = 0,
        FastPhysics,
        Box3D,
        MuJoCo,
        Kinematic,
        Static
    };

    /** One authority-owned body/link state contributed after solve or reset restoration. */
    struct PhysicsBodyStateSnapshot
    {
        std::string stable_id;
        std::string link_id;
        PhysicsBodyAuthority authority = PhysicsBodyAuthority::Static;
        PhysicsCanonicalPose pose;
        PhysicsCanonicalTwist twist;

        // The coordinator verifies that this source belongs to the current epoch and is not from
        // the future, then computes state_age_nanos against the candidate commit time.
        uint64_t source_reset_epoch = 0;
        uint64_t source_step_sequence = 0;
        uint64_t source_time_nanos = 0;
        uint64_t state_age_nanos = 0;
    };

    enum class PhysicsSnapshotTiming : uint8_t
    {
        Synchronized = 0,
        LaggedMixedTime
    };

    /** Immutable, sorted, authority-unique state published atomically with a coordinator stamp. */
    class PhysicsWorldSnapshot
    {
    public:
        const PhysicsCoordinatorStamp& stamp() const { return stamp_; }
        uint64_t dtNanos() const { return dt_nanos_; }
        PhysicsSnapshotTiming timing() const { return timing_; }
        const std::vector<PhysicsBodyStateSnapshot>& bodies() const { return bodies_; }
        const PhysicsBodyStateSnapshot* find(const std::string& stable_id) const;

    private:
        friend class PhysicsSceneCoordinator;

        PhysicsWorldSnapshot(const PhysicsCoordinatorStamp& stamp, uint64_t dt_nanos,
                             std::vector<PhysicsBodyStateSnapshot> bodies);

        PhysicsCoordinatorStamp stamp_;
        uint64_t dt_nanos_ = 0;
        PhysicsSnapshotTiming timing_ = PhysicsSnapshotTiming::Synchronized;
        std::vector<PhysicsBodyStateSnapshot> bodies_;
    };

    /** One engine-neutral member of the committed scene/reset baseline.
     *
     * Phase 1 intentionally does not define collision, mass, or solver types. Later layers can add
     * canonical descriptors without changing the transaction model. `baseline_properties` is for
     * small, deterministic metadata only; solver-native reset state remains owned by the participant
     * which receives this immutable manifest on every reset.
     */
    struct PhysicsManifestEntry
    {
        std::string stable_id;
        std::string participant_id;
        std::string source_id;
        std::map<std::string, std::string> baseline_properties;
    };

    /** Immutable committed scene description.
     *
     * Instances can only be constructed by PhysicsSceneCoordinator and are exposed as
     * `shared_ptr<const PhysicsSceneManifest>`. Entries are copied and sorted by stable id at commit,
     * so retaining or modifying a staging value cannot move the reset baseline afterwards.
     */
    class PhysicsSceneManifest
    {
    public:
        const PhysicsWorldIdentity& world() const { return world_; }
        uint64_t revision() const { return revision_; }
        const std::vector<PhysicsManifestEntry>& entries() const { return entries_; }
        const PhysicsManifestEntry* find(const std::string& stable_id) const;

    private:
        friend class PhysicsSceneCoordinator;

        PhysicsSceneManifest(const PhysicsWorldIdentity& world, uint64_t revision,
                             std::vector<PhysicsManifestEntry> entries);

        PhysicsWorldIdentity world_;
        uint64_t revision_ = 0;
        std::vector<PhysicsManifestEntry> entries_;
    };

    struct PhysicsManifestContext
    {
        PhysicsCoordinatorStamp previous_stamp;
        PhysicsCoordinatorStamp candidate_stamp;
        std::shared_ptr<const PhysicsSceneManifest> previous_manifest;
        std::shared_ptr<const PhysicsSceneManifest> candidate_manifest;
    };

    struct PhysicsStepContext
    {
        PhysicsCoordinatorStamp candidate_stamp;
        std::shared_ptr<const PhysicsSceneManifest> manifest;
        double dt = 0.0;
        uint64_t dt_nanos = 0;
        std::shared_ptr<const PhysicsWorldSnapshot> previous_snapshot;
        std::shared_ptr<const PhysicsWorldSnapshot> candidate_snapshot;

        /// True only for the suppressed pre-roll inside a global reset. A participant must advance
        /// its solver exactly as usual and must not publish state, sample sensors, or emit any
        /// externally visible message: this interval is deleted from the public timeline once the
        /// settled configuration becomes the new t=0 initial condition.
        bool presettling = false;
    };

    /** World-level pre-settle policy for one global reset (D9b1/D9b2).
     *
     * The step count is supplied by the owner rather than derived here, so the reset transaction
     * cannot silently disagree with the world's authoritative fixed timestep.
     */
    struct PhysicsResetPolicy
    {
        bool presettle = false;
        double presettle_dt = 0.0;
        uint64_t presettle_steps = 0;
    };

    struct PhysicsResetContext
    {
        PhysicsCoordinatorStamp previous_stamp;
        PhysicsCoordinatorStamp candidate_stamp;
        std::shared_ptr<const PhysicsSceneManifest> baseline_manifest;
        std::string reason;
        std::shared_ptr<const PhysicsWorldSnapshot> previous_snapshot;
        std::shared_ptr<const PhysicsWorldSnapshot> candidate_snapshot;
    };

    /** Engine-neutral lifecycle implemented by a scene, sidecar, adapter, or other participant.
     *
     * The coordinator invokes every phase in deterministic `(order, stable_id)` order. Abort hooks
     * run in reverse order for every participant whose prepare hook was entered (including one which
     * throws) and are best-effort: their exceptions are deliberately suppressed while the original
     * transaction failure is rethrown. A participant retains enough rollback state through commit;
     * only the no-throw finalize hook says every participant committed and that state may be freed.
     * No hook may touch Unreal unless its adapter has explicitly marshalled to the correct Unreal
     * thread; this AirLib core knows no Unreal types.
     */
    class PhysicsSceneParticipant
    {
    public:
        virtual ~PhysicsSceneParticipant() = default;

        virtual void onManifestPrepare(const PhysicsManifestContext& context) { (void)context; }
        virtual void onManifestCommit(const PhysicsManifestContext& context) { (void)context; }
        /// Called only after every participant committed and the coordinator published the new
        /// manifest. This is where an adapter releases rollback state. It is deliberately
        /// noexcept: finalization cannot turn an already-published transaction into a failure.
        virtual void onManifestFinalize(const PhysicsManifestContext& context) noexcept { (void)context; }
        virtual void onManifestAbort(const PhysicsManifestContext& context) { (void)context; }

        virtual void onStepPrepare(const PhysicsStepContext& context) { (void)context; }
        virtual void onStep(const PhysicsStepContext& context) { (void)context; }
        virtual void collectStepBodyStates(const PhysicsStepContext& context,
                                           std::vector<PhysicsBodyStateSnapshot>& states) const
        {
            (void)context;
            (void)states;
        }
        virtual void onStepCommit(const PhysicsStepContext& context) { (void)context; }
        virtual void onStepFinalize(const PhysicsStepContext& context) noexcept { (void)context; }
        virtual void onStepAbort(const PhysicsStepContext& context) { (void)context; }

        virtual void onResetPrepare(const PhysicsResetContext& context) { (void)context; }
        virtual void onResetRestore(const PhysicsResetContext& context) { (void)context; }
        virtual void collectResetBodyStates(const PhysicsResetContext& context,
                                            std::vector<PhysicsBodyStateSnapshot>& states) const
        {
            (void)context;
            (void)states;
        }
        virtual void onResetCommit(const PhysicsResetContext& context) { (void)context; }
        virtual void onResetFinalize(const PhysicsResetContext& context) noexcept { (void)context; }
        virtual void onResetAbort(const PhysicsResetContext& context) { (void)context; }
    };

    /** Phase-1, solver-neutral world physics coordinator.
     *
     * This class is intentionally not an UpdatableObject yet. The future world integration must
     * explicitly place its pre/step/post phases in the existing authoritative executor rather than
     * accidentally creating a second scheduler. Calls are externally serialized by that owner.
     */
    class PhysicsSceneCoordinator
    {
    public:
        /** Kept nested so AirSimSettings may map its own schema enum without this core depending on
         * AirSimSettings.hpp or defining a competing settings type. */
        enum class Mode : uint8_t
        {
            Legacy = 0,
            SingleBackend = 1,
            MixedBackendExperimental = 2
        };

        enum class State : uint8_t
        {
            Empty = 0,
            EditingManifest,
            CommittingManifest,
            Ready,
            Stepping,
            Resetting,
            Faulted
        };

        explicit PhysicsSceneCoordinator(Mode mode, PhysicsWorldIdentity world);

        Mode mode() const { return mode_; }
        bool enabled() const { return mode_ != Mode::Legacy; }
        State state() const { return state_; }
        bool faulted() const { return state_ == State::Faulted; }
        PhysicsCoordinatorStamp stamp() const { return stamp_; }

        /** Register before the first manifest commit. Ownership is shared deliberately: callbacks
         * cannot outlive the object they target. Duplicate ids, duplicate participant instances,
         * and null participants are rejected so one scene cannot accidentally be stepped twice. */
        void registerParticipant(const std::string& stable_id, int order,
                                 std::shared_ptr<PhysicsSceneParticipant> participant);
        std::size_t participantCount() const { return participants_.size(); }

        /** Begin the initial, whole-manifest transaction. The staging area starts empty; the caller
         * must describe the complete baseline rather than accidentally inheriting live state.
         * Phase 1 freezes that baseline after commit. A later scene-reconfiguration API may create
         * another manifest revision, but ordinary reset and this API never change topology. */
        void beginManifestUpdate();
        void stageManifestEntry(PhysicsManifestEntry entry);
        void discardManifestUpdate();
        std::shared_ptr<const PhysicsSceneManifest> commitManifest();

        std::shared_ptr<const PhysicsSceneManifest> manifest() const { return manifest_; }
        std::shared_ptr<const PhysicsWorldSnapshot> snapshot() const { return snapshot_; }

        /** Returns false in Legacy mode without validating inputs, invoking callbacks, or changing
         * counters. This is the compatibility/no-op seam for the initial rollout. */
        bool step(double dt);

        /** Returns false in Legacy mode. In enabled modes, the new epoch is published before reset
         * callbacks so stale sidecar/proxy messages are invalid even when restoration later fails.
         *
         * When `policy.presettle` is set, every participant is advanced `presettle_steps` times
         * inside this one transaction, after restoration and before the baseline snapshot exists.
         * Those steps publish nothing, so the settled configuration is the first state any consumer
         * can observe, and the public step sequence and simulation time still begin at zero. */
        bool reset(const std::string& reason = "reset",
                   const PhysicsResetPolicy& policy = PhysicsResetPolicy());

    private:
        struct ParticipantRecord
        {
            std::string stable_id;
            int order = 0;
            std::shared_ptr<PhysicsSceneParticipant> participant;
        };

        bool participantExists(const std::string& stable_id) const;
        void requireState(State expected, const char* operation) const;
        void abortManifest(const PhysicsManifestContext& context, std::size_t entered_count) noexcept;
        void abortStep(const PhysicsStepContext& context, std::size_t entered_count) noexcept;
        void abortReset(const PhysicsResetContext& context, std::size_t entered_count) noexcept;
        void runPresettle(const PhysicsCoordinatorStamp& baseline_stamp,
                          const PhysicsResetPolicy& policy);
        std::shared_ptr<const PhysicsWorldSnapshot> makeSnapshot(
            const PhysicsCoordinatorStamp& stamp, uint64_t dt_nanos,
            std::vector<PhysicsBodyStateSnapshot> states) const;

        Mode mode_;
        State state_ = State::Empty;
        PhysicsCoordinatorStamp stamp_;
        std::vector<ParticipantRecord> participants_;
        std::map<std::string, PhysicsManifestEntry> staged_entries_;
        std::shared_ptr<const PhysicsSceneManifest> manifest_;
        std::shared_ptr<const PhysicsWorldSnapshot> snapshot_;
    };
}
} // namespace msr::airlib

#endif
