#include "mpm/MpmSidecarPublisher.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <sstream>

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
namespace
{

/// Copy a std::string into a fixed wire field, always NUL-terminated.
///
/// ⚠ TRUNCATION IS REPORTED, not absorbed. A stable id is the sidecar's registry key; two ids that
/// differ only past the 63rd character would silently become one collider, and the sand would be
/// pushed by a wheel that is not there.
bool copyId(const std::string& source, char (&destination)[kMaxColliderIdChars])
{
    std::memset(destination, 0, kMaxColliderIdChars);
    if (source.size() >= kMaxColliderIdChars)
        return false;
    std::memcpy(destination, source.data(), source.size());
    return true;
}

WireVec3 toWire(const urdf::Vec3& v) { return WireVec3{ v.x, v.y, v.z }; }
WireQuat toWire(const urdf::Quat& q) { return WireQuat{ q.x, q.y, q.z, q.w }; }

uint32_t toWireKind(urdf::CollisionShape::Kind kind)
{
    using K = urdf::CollisionShape::Kind;
    switch (kind) {
    case K::Sphere: return static_cast<uint32_t>(WireShapeKind::Sphere);
    case K::Capsule: return static_cast<uint32_t>(WireShapeKind::Capsule);
    case K::Cylinder: return static_cast<uint32_t>(WireShapeKind::Cylinder);
    case K::Box: return static_cast<uint32_t>(WireShapeKind::Box);
    case K::Mesh: return static_cast<uint32_t>(WireShapeKind::ConvexHull);
    case K::HeightField:
    case K::Plane: return static_cast<uint32_t>(WireShapeKind::Plane);
    }
    return static_cast<uint32_t>(WireShapeKind::ConvexHull);
}

uint32_t toWireRole(urdf::CouplingRole role)
{
    switch (role) {
    case urdf::CouplingRole::Static: return static_cast<uint32_t>(WireCouplingRole::Static);
    case urdf::CouplingRole::KinematicOneWay:
        return static_cast<uint32_t>(WireCouplingRole::KinematicOneWay);
    case urdf::CouplingRole::DynamicTwoWay:
        return static_cast<uint32_t>(WireCouplingRole::DynamicTwoWay);
    }
    return static_cast<uint32_t>(WireCouplingRole::KinematicOneWay);
}

} // namespace

struct MpmSidecarPublisher::Impl
{
#if MPM_SHM_SUPPORTED
    MpmRegistryBlock* registry = nullptr;
    MpmStateBlock* state = nullptr;
    MpmStatusBlock* status = nullptr;
    int registry_fd = -1;
    int state_fd = -1;
    int status_fd = -1;

    template <typename Block>
    Block* map(const std::string& path, int& fd, bool zero)
    {
        fd = ::open(path.c_str(), O_RDWR | O_CREAT, 0666);
        if (fd < 0) return nullptr;
        if (::ftruncate(fd, static_cast<off_t>(sizeof(Block))) != 0) {
            ::close(fd);
            fd = -1;
            return nullptr;
        }
        void* address = ::mmap(nullptr, sizeof(Block), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (address == MAP_FAILED) {
            ::close(fd);
            fd = -1;
            return nullptr;
        }
        // ⚠ Zero only the segments WE own. The status block belongs to the sidecar; wiping it on
        // open would erase an acknowledgement from a sidecar that is already running and make a
        // healthy link look dead.
        if (zero) std::memset(address, 0, sizeof(Block));
        return static_cast<Block*>(address);
    }

    template <typename Block>
    void unmap(Block*& block, int& fd)
    {
        if (block != nullptr) {
            ::munmap(block, sizeof(Block));
            block = nullptr;
        }
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }
#endif
};

std::string MpmSidecarPublisher::Health::describe() const
{
    std::ostringstream out;
    if (!link_open) {
        out << "MPM link CLOSED";
        return out.str();
    }
    if (!sidecar_seen) {
        out << "MPM sidecar has never written a status block — is it running? "
               "(sim is publishing at step " << published_step << ")";
        return out.str();
    }
    if (!epoch_matches) {
        out << "MPM sidecar is on a DIFFERENT WORLD/EPOCH (" << message
            << "). Its sand belongs to a run that no longer exists.";
        return out.str();
    }
    if (fault != 0) {
        out << "MPM sidecar reports FAULT " << fault << ": " << message;
        return out.str();
    }
    if (!responsive) {
        out << "MPM sidecar is STALE: acknowledged step " << acknowledged_step << " against "
            << published_step << " published (" << lag_steps << " behind)";
        return out.str();
    }
    out << "MPM sidecar healthy: " << lag_steps << " steps behind, " << particle_count
        << " particles, last solve " << (last_solve_seconds * 1000.0) << " ms";
    return out.str();
}

MpmSidecarPublisher::MpmSidecarPublisher() : impl_(new Impl()) {}

MpmSidecarPublisher::~MpmSidecarPublisher() { close(); }

bool MpmSidecarPublisher::isOpen() const
{
#if MPM_SHM_SUPPORTED
    return impl_ && impl_->registry != nullptr && impl_->state != nullptr &&
           impl_->status != nullptr;
#else
    return false;
#endif
}

bool MpmSidecarPublisher::open(const Options& options)
{
    close();
    options_ = options;
#if MPM_SHM_SUPPORTED
    const std::string dir = options.directory;
    impl_->registry = impl_->map<MpmRegistryBlock>(dir + "/" + kRegistrySegment,
                                                   impl_->registry_fd, /*zero=*/true);
    impl_->state = impl_->map<MpmStateBlock>(dir + "/" + kStateSegment, impl_->state_fd,
                                             /*zero=*/true);
    impl_->status = impl_->map<MpmStatusBlock>(dir + "/" + kStatusSegment, impl_->status_fd,
                                               /*zero=*/false);
    if (!isOpen()) {
        close();
        return false;
    }
    impl_->registry->magic = kRegistryMagic;
    impl_->registry->version = kProtocolVersion;
    impl_->state->magic = kStateMagic;
    impl_->state->version = kProtocolVersion;
    return true;
#else
    return false;
#endif
}

void MpmSidecarPublisher::close()
{
#if MPM_SHM_SUPPORTED
    if (impl_) {
        impl_->unmap(impl_->registry, impl_->registry_fd);
        impl_->unmap(impl_->state, impl_->state_fd);
        impl_->unmap(impl_->status, impl_->status_fd);
    }
#endif
    published_ids_.clear();
    last_published_step_ = 0;
}

bool MpmSidecarPublisher::publishRegistry(const WireWorldStamp& stamp, double sim_fixed_dt,
                                          const WireTerrainRegion& region,
                                          const std::vector<urdf::PhysicsColliderSet>& robots,
                                          const std::vector<std::string>& selected_ids)
{
#if MPM_SHM_SUPPORTED
    if (!isOpen()) return false;

    published_ids_.clear();
    MpmRegistryBlock& block = *impl_->registry;

    // Seqlock: odd while writing. A reader that catches an odd sequence retries rather than
    // consuming a registry that is half old and half new.
    std::atomic_thread_fence(std::memory_order_release);
    block.sequence |= 1u;
    std::atomic_thread_fence(std::memory_order_release);

    block.stamp = stamp;
    block.sim_fixed_dt = sim_fixed_dt;
    block.region = region;
    uint32_t written = 0;
    bool ok = true;

    for (const urdf::PhysicsColliderSet& robot : robots) {
        for (const urdf::PhysicsColliderDescriptor& collider : robot.colliders) {
            // ⚠ SELECTION IS THE M2 EXIT CRITERION — "selected colliders affect MPM; unselected
            // links do not". An empty selection means none, never all: defaulting to all would put
            // a whole rover's geometry into the sand the first time somebody forgot the setting.
            if (std::find(selected_ids.begin(), selected_ids.end(), collider.stable_id) ==
                selected_ids.end())
                continue;

            if (written >= kMaxColliders) {
                ok = false;
                break;
            }

            WireColliderRegistration& out = block.colliders[written];
            out = WireColliderRegistration();
            if (!copyId(collider.stable_id, out.stable_id)) {
                ok = false;
                continue;
            }
            out.role = toWireRole(collider.role);
            out.mass = collider.inertial.mass;
            out.com_local = toWire(collider.inertial.com_local);
            for (int i = 0; i < 9; ++i)
                out.inertia_local[i] = collider.inertial.inertia_local[i];
            out.inertia_is_articulated_effective =
                collider.inertial.is_articulated_effective_inertia ? 1u : 0u;
            out.friction = collider.material.friction;
            out.restitution = collider.material.restitution;
            out.material_reported = collider.material.reported ? 1u : 0u;

            uint32_t shape_index = 0;
            for (const urdf::CollisionShape& shape : collider.shapes) {
                if (shape_index >= kMaxShapesPerCollider) {
                    ok = false;
                    break;
                }
                WireShape& wire = out.shapes[shape_index];
                wire.kind = toWireKind(shape.kind);
                wire.position = toWire(shape.position);
                wire.orientation = toWire(shape.orientation);
                wire.radius = shape.radius;
                wire.half_length = shape.half_length;
                wire.half_extents = toWire(shape.half_extents);

                const uint32_t vertices =
                    static_cast<uint32_t>(std::min<size_t>(shape.vertices.size(),
                                                           kMaxShapeVertices));
                // ⚠ A hull silently reduced to its first 64 vertices is a DIFFERENT SHAPE, and a
                // smaller one — the wheel would sink. Reported, not clamped quietly.
                if (shape.vertices.size() > kMaxShapeVertices) ok = false;
                wire.vertex_count = vertices;
                for (uint32_t v = 0; v < vertices; ++v)
                    wire.vertices[v] = toWire(shape.vertices[v]);
                ++shape_index;
            }
            out.shape_count = shape_index;

            published_ids_.push_back(collider.stable_id);
            ++written;
        }
    }

    block.collider_count = written;

    std::atomic_thread_fence(std::memory_order_release);
    block.sequence = (block.sequence + 1u) & ~1u;
    std::atomic_thread_fence(std::memory_order_release);
    return ok;
#else
    (void)stamp; (void)sim_fixed_dt; (void)robots; (void)selected_ids;
    return false;
#endif
}

bool MpmSidecarPublisher::publishState(const WireWorldStamp& stamp, uint64_t step,
                                       double simulation_time,
                                       const std::vector<urdf::PhysicsColliderSet>& robots)
{
#if MPM_SHM_SUPPORTED
    if (!isOpen() || published_ids_.empty()) return false;
    if (options_.publish_every_n_steps > 1 && (step % options_.publish_every_n_steps) != 0)
        return true;

    MpmStateBlock& block = *impl_->state;

    std::atomic_thread_fence(std::memory_order_release);
    block.sequence |= 1u;
    std::atomic_thread_fence(std::memory_order_release);

    block.stamp = stamp;
    block.step = step;
    block.simulation_time = simulation_time;

    // ⚠ Indexed by REGISTRY ORDER, and that order is rebuilt here by walking the same sets in the
    // same sequence. Matching by name instead would be a 64-byte comparison per collider per tick
    // at 333 Hz; matching by position is free and is why publishRegistry recorded the order.
    size_t index = 0;
    bool ok = true;
    for (const urdf::PhysicsColliderSet& robot : robots) {
        for (const urdf::PhysicsColliderDescriptor& collider : robot.colliders) {
            if (index >= published_ids_.size()) break;
            if (collider.stable_id != published_ids_[index]) continue;

            WireColliderState& out = block.colliders[index];
            out.position = toWire(collider.position);
            out.orientation = toWire(collider.orientation);
            out.linear_velocity = toWire(collider.linear_velocity);
            out.angular_velocity = toWire(collider.angular_velocity);
            ++index;
        }
    }
    // ⚠ A short write means the population changed without the registry being republished — the
    // sidecar would then push sand with poses belonging to different links. Loud, not silent.
    if (index != published_ids_.size()) ok = false;
    block.collider_count = static_cast<uint32_t>(index);

    std::atomic_thread_fence(std::memory_order_release);
    block.sequence = (block.sequence + 1u) & ~1u;
    std::atomic_thread_fence(std::memory_order_release);

    last_published_step_ = step;
    return ok;
#else
    (void)stamp; (void)step; (void)simulation_time; (void)robots;
    return false;
#endif
}

MpmSidecarPublisher::Health MpmSidecarPublisher::health(const WireWorldStamp& expected) const
{
    Health result;
    result.published_step = last_published_step_;
#if MPM_SHM_SUPPORTED
    if (!isOpen()) return result;
    result.link_open = true;

    // Seqlock read: retry a bounded number of times, then report what we have rather than spin.
    MpmStatusBlock snapshot;
    bool consistent = false;
    for (int attempt = 0; attempt < 16; ++attempt) {
        const uint32_t before = impl_->status->sequence;
        if (before % 2 != 0) continue;
        std::atomic_thread_fence(std::memory_order_acquire);
        snapshot = *impl_->status;
        std::atomic_thread_fence(std::memory_order_acquire);
        if (impl_->status->sequence == before) {
            consistent = true;
            break;
        }
    }
    if (!consistent) return result;
    if (snapshot.magic != kStatusMagic) return result;

    result.sidecar_seen = true;
    result.fault = snapshot.fault;
    result.message = snapshot.message;
    result.particle_count = snapshot.particle_count;
    result.last_solve_seconds = snapshot.last_solve_seconds;
    result.acknowledged_step = snapshot.acknowledged_step;

    result.epoch_matches = snapshot.stamp.world_id == expected.world_id &&
                           snapshot.stamp.world_revision == expected.world_revision &&
                           snapshot.stamp.manifest_revision == expected.manifest_revision &&
                           snapshot.stamp.reset_epoch == expected.reset_epoch;
    if (!result.epoch_matches) {
        std::ostringstream out;
        out << "sidecar on world " << snapshot.stamp.world_id << "/"
            << snapshot.stamp.world_revision << " manifest " << snapshot.stamp.manifest_revision
            << " epoch " << snapshot.stamp.reset_epoch << "; sim on world " << expected.world_id
            << "/" << expected.world_revision << " manifest " << expected.manifest_revision
            << " epoch " << expected.reset_epoch;
        result.message = out.str();
        return result;
    }

    result.lag_steps = last_published_step_ > snapshot.acknowledged_step
                           ? last_published_step_ - snapshot.acknowledged_step
                           : 0;
    result.responsive = snapshot.fault == 0 && result.lag_steps <= options_.max_lag_steps;
#else
    (void)expected;
#endif
    return result;
}

} // namespace mpm
} // namespace airlib
} // namespace msr
