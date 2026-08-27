#include "mpm/MpmSidecarStatus.hpp"

#include "mpm/MpmSidecarProtocol.hpp"

#include <atomic>
#include <cstring>

#if defined(__linux__) || defined(__APPLE__)
#define MPM_STATUS_SHM_SUPPORTED 1
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#else
#define MPM_STATUS_SHM_SUPPORTED 0
#endif

namespace msr
{
namespace airlib
{
namespace mpm
{
namespace
{

#if MPM_STATUS_SHM_SUPPORTED

/// Map one block read-only, extract just the fields wanted through its seqlock, unmap.
///
/// ⚠ EXTRACTS, NEVER COPIES THE WHOLE BLOCK. The first version of this took a `Block` by reference
/// and memcpy'd into it; `MpmStaticWorldBlock` is 24 MB, so the caller's local blew the 8 MB stack
/// and the process died with SIGSEGV on the first call. In the editor that is a crash on opening a
/// status panel — a diagnostic tool taking down the thing it is diagnosing. `extract` is handed a
/// const pointer to the LIVE mapping and copies out the handful of scalars that are wanted; the
/// seqlock is re-checked afterwards, so a torn read is rejected exactly as a full copy would be.
///
/// ⚠ MAPPED AND UNMAPPED EACH CALL, unlike the backend which keeps its mappings. A status reader
/// holding a mapping would hold an unlinked inode alive across a sidecar restart and go on
/// reporting the dead run's numbers as current — the very failure `revalidateSegments` exists to
/// escape, reintroduced in the one place whose whole job is to tell the truth about what is
/// running.
template <typename Block, typename Extract>
bool readFields(const std::string& directory, const char* name, bool& present, Extract extract)
{
    present = false;
    const std::string path = directory + "/" + name;
    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0)
        return false;

    struct stat info;
    if (::fstat(fd, &info) != 0 || static_cast<size_t>(info.st_size) != sizeof(Block)) {
        // ⚠ A WRONG-SIZED BLOCK IS A PROTOCOL MISMATCH, not an absent one, and the difference
        // matters to whoever reads the panel: "no sidecar" and "a sidecar built from a different
        // version of the protocol" have completely different fixes.
        ::close(fd);
        present = true;
        return false;
    }

    void* address = ::mmap(nullptr, sizeof(Block), PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (address == MAP_FAILED)
        return false;
    present = true;

    const Block* src = static_cast<const Block*>(address);
    const auto* seq = reinterpret_cast<const volatile std::atomic<uint32_t>*>(&src->sequence);
    bool ok = false;
    for (int attempt = 0; attempt < 8; ++attempt) {
        const uint32_t before = seq->load(std::memory_order_acquire);
        if (before % 2 != 0)
            continue;                       // the writer is inside an update
        extract(*src);
        const uint32_t after = seq->load(std::memory_order_acquire);
        if (before == after) {
            ok = true;
            break;
        }
    }
    ::munmap(address, sizeof(Block));
    return ok;
}

#endif // MPM_STATUS_SHM_SUPPORTED

} // namespace

SidecarStatus readSidecarStatus(const std::string& directory)
{
    SidecarStatus status;
    status.expected_version = kProtocolVersion;

#if !MPM_STATUS_SHM_SUPPORTED
    (void)directory;
    return status;
#else
    {
        bool present = false;
        auto& out = status.particles;
        if (readFields<MpmParticleBlock>(directory, kParticleSegment, present,
                                         [&out](const MpmParticleBlock& b) {
                                             out.version = b.version;
                                             out.step = b.sidecar_step;
                                             out.published = b.particle_count;
                                             out.total = b.total_particles;
                                             out.radius = b.radius;
                                         })) {
            out.present = true;
            out.version_mismatch = (out.version != kProtocolVersion);
        }
        else {
            out.present = present;
            out.version_mismatch = present;   // present but unreadable => mismatch
        }
    }

    {
        bool present = false;
        auto& out = status.poses;
        if (readFields<MpmVehiclePoseBlock>(directory, kVehiclePoseSegment, present,
                                            [&out](const MpmVehiclePoseBlock& b) {
                                                out.version = b.version;
                                                out.step = b.sidecar_step;
                                                out.time = b.sidecar_time;
                                                out.publish_interval = b.publish_interval_seconds;
                                                out.acknowledged_command_step =
                                                    b.acknowledged_command_step;
                                                out.vehicle_count = b.vehicle_count;
                                                if (b.vehicle_count == 0)
                                                    return;
                                                const WireVehiclePose& v = b.vehicles[0];
                                                out.vehicle_name.assign(
                                                    v.vehicle_name,
                                                    ::strnlen(v.vehicle_name,
                                                              kMaxVehicleNameChars));
                                                out.link_count = v.link_count;
                                                out.joint_count = v.joint_count;
                                                if (v.link_count == 0)
                                                    return;
                                                out.root[0] = v.links[0].position.x;
                                                out.root[1] = v.links[0].position.y;
                                                out.root[2] = v.links[0].position.z;
                                            })) {
            out.present = true;
            out.version_mismatch = (out.version != kProtocolVersion);
        }
        else {
            out.present = present;
            out.version_mismatch = present;
        }
    }

    {
        bool present = false;
        auto& out = status.commands;
        if (readFields<MpmVehicleCommandBlock>(directory, kVehicleCommandSegment, present,
                                               [&out](const MpmVehicleCommandBlock& b) {
                                                   out.version = b.version;
                                                   out.step = b.step;
                                                   out.reset_epoch = b.reset_epoch;
                                                   out.session_id = b.session_id;
                                                   out.kinematic_revision = b.kinematic_revision;
                                                   out.kinematic_count = b.kinematic_count;
                                               })) {
            out.present = true;
            out.version_mismatch = (out.version != kProtocolVersion);
        }
        else {
            out.present = present;
            out.version_mismatch = present;
        }
    }

    {
        bool present = false;
        auto& out = status.level;
        if (readFields<MpmStaticWorldBlock>(directory, kStaticWorldSegment, present,
                                            [&out](const MpmStaticWorldBlock& b) {
                                                out.version = b.version;
                                                out.revision = b.revision;
                                                out.body_count = b.body_count;
                                                out.kinematic_count = b.kinematic_count;
                                                out.triangle_count = b.index_count / 3;
                                                out.truncated = b.truncated != 0;
                                            })) {
            out.present = true;
            out.version_mismatch = (out.version != kProtocolVersion);
        }
        else {
            out.present = present;
            out.version_mismatch = present;
        }
    }

    return status;
#endif
}

} // namespace mpm
} // namespace airlib
} // namespace msr
