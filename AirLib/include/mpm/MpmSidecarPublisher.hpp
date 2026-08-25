// The simulator's end of the Newton MPM sidecar link.
//
// ⚠ THE SIM MUST NEVER BLOCK ON THE SIDECAR. Newton runs out of process on a GPU whose cadence does
// not match ours — the coordinated tick is 3 ms, one measured MPM frame is 16.67 ms — so waiting
// for it would make the rigid simulation run at the sand's speed. The sim writes and moves on.
//
// ⚠ BUT IT MUST NOTICE. Plan M2's exit criterion is that a timeout or epoch mismatch **pauses with
// a diagnostic** and that no stale collider state is used. Not blocking and not noticing are
// different things, and the acknowledgement in MpmStatusBlock is what separates them: the sidecar
// echoes the last step it consumed, and `health()` turns the difference into a number the caller
// can act on.
//
// ⚠ WHAT THIS DOES NOT DO. It does not read reactions back. M2 is one-way by decision, because
// neither backend can supply a link's articulated effective inertia (plan §11.1) and a two-way
// impulse computed from the isolated link inertia would be wrong in a way no log line shows. The
// return path arrives in M3/M4 behind that spike, and there is deliberately no half-built version
// of it here to be mistaken for a working one.
#pragma once

#include "mpm/MpmSidecarProtocol.hpp"
#include "urdf/UrdfPhysicsDescriptor.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace msr
{
namespace airlib
{
namespace mpm
{

class MpmSidecarPublisher
{
public:
    struct Options {
        /// Directory for the segments. /dev/shm is only the default — a container gets a private
        /// 64 MiB /dev/shm unless it is explicitly shared, so this must be a parameter. Same
        /// reasoning as stream/SharedMemorySink.hpp.
        std::string directory = "/dev/shm";

        /// How far the sidecar may fall behind, in sim steps, before `health()` calls it stale.
        ///
        /// ⚠ Lag is NORMAL and this is not a deadline. At 3 ms per tick against a 16.67 ms MPM
        /// frame the sidecar is ~6 steps behind at all times even when perfectly healthy. The
        /// default allows ~0.6 s of real lag, which is far beyond the cadence ratio and therefore
        /// only trips on a sidecar that has actually stopped.
        uint64_t max_lag_steps = 200;

        /// Steps between state publications. 1 writes every tick.
        ///
        /// ⚠ Writing more often than the sidecar consumes is not an error and not waste worth
        /// avoiding — the segment is latest-wins for state, 26 KiB, and a fresher pose is strictly
        /// better than an older one. Raise it only to cut bandwidth deliberately.
        uint32_t publish_every_n_steps = 1;
    };

    /// What the sidecar is doing, as far as the sim can tell from outside it.
    struct Health {
        bool link_open = false;
        /// The sidecar has written a status block at all. False before it ever starts.
        bool sidecar_seen = false;
        /// Its stamp matches ours. False means it is simulating a world that no longer exists.
        bool epoch_matches = false;
        /// It has acknowledged a step within `max_lag_steps` of the one we last published.
        bool responsive = false;

        uint64_t published_step = 0;
        uint64_t acknowledged_step = 0;
        uint64_t lag_steps = 0;

        uint64_t particle_count = 0;
        double last_solve_seconds = 0;
        uint32_t fault = 0;
        std::string message;

        /// One line for a log. Says what is wrong, not merely that something is.
        std::string describe() const;
    };

    MpmSidecarPublisher();
    ~MpmSidecarPublisher();

    MpmSidecarPublisher(const MpmSidecarPublisher&) = delete;
    MpmSidecarPublisher& operator=(const MpmSidecarPublisher&) = delete;

    /// Create the three segments. Returns false and leaves the link closed on failure — a sim that
    /// cannot open the link must be able to carry on without sand rather than refuse to start.
    bool open(const Options& options);
    void close();
    bool isOpen() const;

    /// Publish TOPOLOGY. Call once after the manifest commits, and again only on a real topology
    /// change — never on an ordinary reset, which restores state without changing membership.
    ///
    /// `select` decides which links become MPM colliders. Returns false if the population exceeds
    /// the wire's ceilings, which is an error rather than a truncation: a robot whose 30th wheel
    /// quietly failed to register would sink through sand every counter reported as present.
    ///
    /// ⚠ `region` must already be in SOLVER frame. The caller owns that conversion because the
    /// caller is the only side that knows about NED; this class and the sidecar are both
    /// frame-agnostic, and a mirror applied on the wrong side of the boundary produces sand in a
    /// plausible wrong place rather than an obviously wrong one.
    bool publishRegistry(const WireWorldStamp& stamp, double sim_fixed_dt,
                         const WireTerrainRegion& region,
                         const std::vector<urdf::PhysicsColliderSet>& robots,
                         const std::vector<std::string>& selected_ids);

    /// Publish per-step STATE. Cheap; call every tick.
    ///
    /// ⚠ Order must match the registry exactly — the state array is indexed by registry position,
    /// not by name, because a 64-byte string comparison per collider per tick is not a thing to do
    /// at 333 Hz. `publishRegistry` records the order and this asserts against it.
    bool publishState(const WireWorldStamp& stamp, uint64_t step, double simulation_time,
                      const std::vector<urdf::PhysicsColliderSet>& robots);

    /// Read the sidecar's acknowledgement. Never blocks.
    Health health(const WireWorldStamp& expected_stamp) const;

    /// Ids in registry order, for a caller that wants to log or check the mapping.
    const std::vector<std::string>& publishedIds() const { return published_ids_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::vector<std::string> published_ids_;
    Options options_;
    uint64_t last_published_step_ = 0;
};

} // namespace mpm
} // namespace airlib
} // namespace msr
