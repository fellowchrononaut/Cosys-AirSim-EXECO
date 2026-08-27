// One snapshot of what the Newton MPM sidecar's wire says about itself.
//
// ⚠ THE WIRE, NOT THE PROCESS, AND THE TWO MUST NEVER BE CONFLATED. On 2026-08-27 the question
// "is the sidecar running" was answered for over an hour by `pgrep -f own-vehicle-publish-hz`,
// which matches any process whose command line contains that string — the launching shell, a stale
// wrapper, the checking command itself. It reported ALIVE while no sidecar existed. A live process
// is not a working one either: a sidecar blocked in an 11 s model rebuild is alive at 0 % CPU with
// a frozen particle block, and its sand goes stale on screen while `ps` says everything is fine.
//
// So this reader deliberately answers ONE half — what the blocks contain — and the caller decides
// liveness by whether the counters ADVANCE between two snapshots. That is a stronger question than
// "does a process exist", and it is the one that determines whether the sand on screen is real.
//
// ⚠ HEADER-LIGHT ON PURPOSE. It depends on MpmSidecarProtocol.hpp and POSIX mmap and nothing else,
// so it can be compiled and run against a live session outside Unreal — which is how it was
// checked against `tools/sidecar_monitor.py`, the Python reader of the same blocks.
#pragma once

#include <cstdint>
#include <string>

namespace msr
{
namespace airlib
{
namespace mpm
{

/// Everything the four shared-memory blocks say, in one instant. Absent blocks are reported as
/// absent rather than as zeroes: "no particle block" and "a particle block holding zero particles"
/// are different failures and only one of them means the sidecar has not started.
struct SidecarStatus
{
    /// ⚠ The version THIS BUILD speaks. A block whose version differs is reported below rather
    /// than parsed: a consumer that silently misreads a pose produces sand that deforms in the
    /// wrong place, which reads as a physics bug and is not one.
    uint32_t expected_version = 0;

    struct Particles {
        bool present = false;
        bool version_mismatch = false;
        uint32_t version = 0;
        uint64_t step = 0;
        uint32_t published = 0;      ///< decimated count actually on the wire
        uint64_t total = 0;          ///< how many the solver holds
        float radius = 0.0f;
    } particles;

    struct Poses {
        bool present = false;
        bool version_mismatch = false;
        uint32_t version = 0;
        uint64_t step = 0;
        double time = 0.0;           ///< SIMULATED seconds, not wall time
        double publish_interval = 0.0;
        uint64_t acknowledged_command_step = 0;
        uint32_t vehicle_count = 0;
        std::string vehicle_name;
        uint32_t link_count = 0;
        uint32_t joint_count = 0;
        double root[3] = {0.0, 0.0, 0.0};
    } poses;

    struct Commands {
        bool present = false;
        bool version_mismatch = false;
        uint32_t version = 0;
        uint64_t step = 0;
        uint64_t reset_epoch = 0;
        /// ⚠ WHICH PIE SESSION. Random per backend, because every counter on it restarts from the
        /// same base each Play — see MpmSidecarProtocol.hpp.
        uint64_t session_id = 0;
        uint32_t kinematic_revision = 0;
        uint32_t kinematic_count = 0;
    } commands;

    struct Level {
        bool present = false;
        bool version_mismatch = false;
        uint32_t version = 0;
        uint32_t revision = 0;
        uint32_t body_count = 0;
        uint32_t kinematic_count = 0;
        uint32_t triangle_count = 0;
        bool truncated = false;
    } level;

    /// ⚠ THE REGISTRATION THE TWO ENDS DISAGREE ABOUT, surfaced as its own question. The sidecar
    /// REFUSES mirrored-actor poses whose revision does not match the model it built, and in that
    /// state every mirrored actor is frozen at its build pose while both ends report healthy.
    bool kinematic_revision_matches() const
    {
        return !commands.present || !level.present ||
               commands.kinematic_revision == level.revision;
    }

    /// True when nothing at all could be read — no sidecar has ever run against this directory.
    bool anyBlockPresent() const
    {
        return particles.present || poses.present || commands.present || level.present;
    }
};

/// Read all four blocks in one pass. Never throws and never blocks; a block that is missing, the
/// wrong size, or torn is reported rather than waited for.
///
/// ⚠ TORN READS ARE REJECTED, NOT RETRIED FOREVER. Each block is a seqlock whose sequence is ODD
/// while the writer is inside an update; this makes a bounded number of attempts and gives up,
/// because a status panel that stalls the game thread to get a consistent read of a diagnostic is
/// worse than one that says "busy" for a frame.
SidecarStatus readSidecarStatus(const std::string& directory);

} // namespace mpm
} // namespace airlib
} // namespace msr
