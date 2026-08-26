// The simulator's read end of the sand's reaction — plan M3/M4.
//
// ⚠ THIS ONE IS PHYSICS, unlike MpmParticleReader. What it returns is applied to rigid bodies, so
// every property that made the particle stream safe to be sloppy about matters here: a stale frame
// applied as a live force keeps pushing a rover the sand stopped touching, and a misattributed
// collider puts one wheel's reaction on another.
//
// ⚠ IMPULSES (N.s), reduced per collider about its centre of mass by the sidecar. The consumer must
// convert to whatever its rigid seam accepts, and both current backends take NEWTONS — that
// conversion is a recorded decision (plan M3), not an implementation detail.
//
// ⚠ LAGGED. The sidecar solves at its own cadence, so a frame describes an interval that has
// already passed. A run using it records LaggedImpulseTwoWay, never TwoWay, and the effective
// inertia question in plan 11.1 remains open regardless of how good the numbers look.
#pragma once

#include "mpm/MpmSidecarProtocol.hpp"

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

class MpmImpulseReader
{
public:
    struct Frame {
        bool valid = false;
        /// One entry per registered collider, in REGISTRY ORDER — the same order the state block
        /// is written in, so index i is the same body at both ends.
        std::vector<WireColliderImpulse> colliders;
        uint32_t collider_count = 0;
        /// Simulated seconds these impulses were accumulated over. ⚠ NOT the consumer's tick.
        double mpm_dt = 0;
        uint64_t sidecar_step = 0;
        double sidecar_time = 0;
        WireWorldStamp stamp;
    };

    MpmImpulseReader();
    ~MpmImpulseReader();

    MpmImpulseReader(const MpmImpulseReader&) = delete;
    MpmImpulseReader& operator=(const MpmImpulseReader&) = delete;

    /// Attach to a segment the SIDECAR created. False when it does not exist, which is the ordinary
    /// state whenever two-way is off or the sidecar has not started.
    bool open(const std::string& directory);
    void close();
    bool isOpen() const;

    /// Read the newest frame. False when nothing new since `last_step` — which the caller must NOT
    /// treat as "apply the previous one again".
    bool read(Frame& out, uint64_t last_step) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace mpm
} // namespace airlib
} // namespace msr
