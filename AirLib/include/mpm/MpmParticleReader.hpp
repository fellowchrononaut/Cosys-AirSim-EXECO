// The simulator's read end of the MPM particle stream.
//
// ⚠ RENDERING ONLY. Nothing in the simulation may branch on what this returns. It is a decimated,
// lagged copy of particle state that lives on another process's GPU, and treating it as physics
// would be reading the sand's shadow. It exists so an operator can SEE the sand in PIE instead of
// waiting for an offline video whose time axis is the sidecar's rather than theirs.
//
// ⚠ LATEST-WINS, and no acknowledgement. Opposite of MpmSidecarPublisher on purpose: a dropped
// render frame is invisible, a stale one is a visible lie, and a reader that made the sidecar wait
// would stall the solver for the sake of pixels.
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

class MpmParticleReader
{
public:
    struct Frame {
        bool valid = false;
        /// Positions in SOLVER frame. The caller converts; this class knows nothing about Unreal.
        std::vector<float> positions;  ///< x,y,z triples
        uint32_t count = 0;
        uint64_t total_particles = 0;  ///< before decimation, so a partial view can say so
        float radius = 0.01f;
        uint64_t sidecar_step = 0;
        double sidecar_time = 0;
        WireWorldStamp stamp;
    };

    MpmParticleReader();
    ~MpmParticleReader();

    MpmParticleReader(const MpmParticleReader&) = delete;
    MpmParticleReader& operator=(const MpmParticleReader&) = delete;

    /// Attach to a segment the SIDECAR created. Returns false when it does not exist yet, which is
    /// the ordinary state before the sidecar starts and is not an error.
    bool open(const std::string& directory);
    void close();
    bool isOpen() const;

    /// Read the newest frame. Returns false if nothing new since `last_step`, so a caller can skip
    /// the conversion and the renderer update entirely on an unchanged frame.
    bool read(Frame& out, uint64_t last_step) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace mpm
} // namespace airlib
} // namespace msr
