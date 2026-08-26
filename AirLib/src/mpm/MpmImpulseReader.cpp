#include "mpm/MpmImpulseReader.hpp"

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

struct MpmImpulseReader::Impl
{
#if MPM_SHM_SUPPORTED
    MpmImpulseBlock* block = nullptr;
    int fd = -1;
#endif
};

MpmImpulseReader::MpmImpulseReader() : impl_(new Impl()) {}
MpmImpulseReader::~MpmImpulseReader() { close(); }

bool MpmImpulseReader::isOpen() const
{
#if MPM_SHM_SUPPORTED
    return impl_ && impl_->block != nullptr;
#else
    return false;
#endif
}

bool MpmImpulseReader::open(const std::string& directory)
{
    close();
#if MPM_SHM_SUPPORTED
    const std::string path = directory + "/" + kImpulseSegment;

    // ⚠ Opened READ-ONLY and never created. The sidecar owns this segment; creating it here would
    // let a simulator started first present an empty block that the sidecar then overwrites, and
    // the window between is a renderer drawing nothing while reporting itself connected.
    impl_->fd = ::open(path.c_str(), O_RDONLY);
    if (impl_->fd < 0)
        return false;

    struct stat info;
    if (::fstat(impl_->fd, &info) != 0 ||
        static_cast<size_t>(info.st_size) != sizeof(MpmImpulseBlock)) {
        // A size mismatch is a PROTOCOL mismatch, not a transient: the two halves were built from
        // different versions of MpmSidecarProtocol.hpp.
        ::close(impl_->fd);
        impl_->fd = -1;
        return false;
    }

    void* address = ::mmap(nullptr, sizeof(MpmImpulseBlock), PROT_READ, MAP_SHARED, impl_->fd, 0);
    if (address == MAP_FAILED) {
        ::close(impl_->fd);
        impl_->fd = -1;
        return false;
    }
    impl_->block = static_cast<MpmImpulseBlock*>(address);
    return true;
#else
    (void)directory;
    return false;
#endif
}

void MpmImpulseReader::close()
{
#if MPM_SHM_SUPPORTED
    if (impl_ && impl_->block != nullptr) {
        ::munmap(impl_->block, sizeof(MpmImpulseBlock));
        impl_->block = nullptr;
    }
    if (impl_ && impl_->fd >= 0) {
        ::close(impl_->fd);
        impl_->fd = -1;
    }
#endif
}

bool MpmImpulseReader::read(Frame& out, uint64_t last_step) const
{
    out.valid = false;
#if MPM_SHM_SUPPORTED
    if (!isOpen())
        return false;

    const MpmImpulseBlock& block = *impl_->block;
    if (block.magic != kImpulseMagic || block.version != kProtocolVersion)
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

        const uint32_t count =
            block.collider_count > kMaxColliders ? kMaxColliders : block.collider_count;
        out.colliders.resize(count);
        if (count > 0)
            std::memcpy(out.colliders.data(), block.colliders,
                        static_cast<size_t>(count) * sizeof(WireColliderImpulse));
        out.collider_count = count;
        out.mpm_dt = block.mpm_dt;
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
