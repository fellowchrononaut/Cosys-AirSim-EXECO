#include "mpm/MpmParticleReader.hpp"

#include <atomic>
#include <cstring>

#if defined(__linux__) || defined(__unix__) || defined(__APPLE__)
#define MPM_SHM_SUPPORTED 1
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#else
#define MPM_SHM_SUPPORTED 0
#endif

namespace msr
{
namespace airlib
{
namespace mpm
{

struct MpmParticleReader::Impl
{
#if MPM_SHM_SUPPORTED
    MpmParticleBlock* block = nullptr;
    int fd = -1;
#endif
};

MpmParticleReader::MpmParticleReader() : impl_(new Impl()) {}
MpmParticleReader::~MpmParticleReader() { close(); }

bool MpmParticleReader::isOpen() const
{
#if MPM_SHM_SUPPORTED
    return impl_ && impl_->block != nullptr;
#else
    return false;
#endif
}

bool MpmParticleReader::open(const std::string& directory)
{
    close();
#if MPM_SHM_SUPPORTED
    const std::string path = directory + "/" + kParticleSegment;

    // ⚠ Opened READ-ONLY and never created. The sidecar owns this segment; creating it here would
    // let a simulator started first present an empty block that the sidecar then overwrites, and
    // the window between is a renderer drawing nothing while reporting itself connected.
    impl_->fd = ::open(path.c_str(), O_RDONLY);
    if (impl_->fd < 0)
        return false;

    struct stat info;
    if (::fstat(impl_->fd, &info) != 0 ||
        static_cast<size_t>(info.st_size) != sizeof(MpmParticleBlock)) {
        // A size mismatch is a PROTOCOL mismatch, not a transient: the two halves were built from
        // different versions of MpmSidecarProtocol.hpp.
        ::close(impl_->fd);
        impl_->fd = -1;
        return false;
    }

    void* address = ::mmap(nullptr, sizeof(MpmParticleBlock), PROT_READ, MAP_SHARED, impl_->fd, 0);
    if (address == MAP_FAILED) {
        ::close(impl_->fd);
        impl_->fd = -1;
        return false;
    }
    impl_->block = static_cast<MpmParticleBlock*>(address);
    return true;
#else
    (void)directory;
    return false;
#endif
}

void MpmParticleReader::close()
{
#if MPM_SHM_SUPPORTED
    if (impl_ && impl_->block != nullptr) {
        ::munmap(impl_->block, sizeof(MpmParticleBlock));
        impl_->block = nullptr;
    }
    if (impl_ && impl_->fd >= 0) {
        ::close(impl_->fd);
        impl_->fd = -1;
    }
#endif
}

bool MpmParticleReader::read(Frame& out, uint64_t last_step) const
{
    out.valid = false;
#if MPM_SHM_SUPPORTED
    if (!isOpen())
        return false;

    const MpmParticleBlock& block = *impl_->block;
    if (block.magic != kParticleMagic || block.version != kProtocolVersion)
        return false;
    // Nothing new: skip the copy AND the renderer update rather than redrawing identical points.
    if (block.sidecar_step == last_step)
        return false;

    // Seqlock: bounded retries, then give up until next frame. Spinning here would put the render
    // thread's stall back into the game thread, which is what latest-wins exists to avoid.
    for (int attempt = 0; attempt < 8; ++attempt) {
        const uint32_t before = block.sequence;
        if (before % 2 != 0)
            continue;
        std::atomic_thread_fence(std::memory_order_acquire);

        const uint32_t count = block.particle_count > kMaxRenderParticles ? kMaxRenderParticles
                                                                         : block.particle_count;
        out.positions.resize(static_cast<size_t>(count) * 3);
        if (count > 0)
            std::memcpy(out.positions.data(), block.positions,
                        static_cast<size_t>(count) * 3 * sizeof(float));
        out.count = count;
        out.total_particles = block.total_particles;
        out.radius = block.radius;
        out.sidecar_step = block.sidecar_step;
        out.sidecar_time = block.sidecar_time;
        out.stamp = block.stamp;

        std::atomic_thread_fence(std::memory_order_acquire);
        if (block.sequence == before) {
            out.valid = true;
            return true;
        }
    }
    return false;
#else
    (void)out; (void)last_step;
    return false;
#endif
}

} // namespace mpm
} // namespace airlib
} // namespace msr
