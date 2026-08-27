#include "mpm/NewtonSidecarUrdfBackend.hpp"

#include "mpm/MpmSidecarProtocol.hpp"
#include "common/common_utils/Utils.hpp"

#include <algorithm>
#include <vector>
#include <map>
#include <utility>
#include <atomic>
#include <chrono>
#include <cmath>
#include <random>
#include <cstring>
#include <stdexcept>

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

/// One published sample, kept host-side so interpolation never touches shared memory.
struct PoseSample {
    double time = 0;                       ///< the sidecar's SIMULATED time
    uint64_t step = 0;
    std::vector<urdf::LinkPose> poses;
    std::vector<urdf::Twist> twists;
    std::vector<urdf::JointState> joints;
    bool effort_reported = false;
};

urdf::Vec3 lerp(const urdf::Vec3& a, const urdf::Vec3& b, double t)
{
    return urdf::Vec3{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
}

/// Cubic Hermite with the published twist as the tangent.
///
/// ⚠ THE TANGENTS MUST BE SCALED BY dt. The Hermite basis is defined on the unit interval, so a
/// velocity in m/s becomes the derivative with respect to the parameter only after multiplying by
/// the interval length. Omitting it overshoots in proportion to the sample rate — an error that
/// looks like a tuning problem and is a units problem.
urdf::Vec3 hermite(const urdf::Vec3& p0, const urdf::Vec3& v0,
                   const urdf::Vec3& p1, const urdf::Vec3& v1, double t, double dt)
{
    const double t2 = t * t, t3 = t2 * t;
    const double h00 = 2 * t3 - 3 * t2 + 1;
    const double h10 = t3 - 2 * t2 + t;
    const double h01 = -2 * t3 + 3 * t2;
    const double h11 = t3 - t2;
    return urdf::Vec3{
        h00 * p0.x + h10 * dt * v0.x + h01 * p1.x + h11 * dt * v1.x,
        h00 * p0.y + h10 * dt * v0.y + h01 * p1.y + h11 * dt * v1.y,
        h00 * p0.z + h10 * dt * v0.z + h01 * p1.z + h11 * dt * v1.z};
}

/// ⚠ SHORTEST ARC. Without the dot-sign flip, two quaternions describing nearly the same rotation
/// can interpolate the long way round and a wheel spins backwards for one frame — which reads as a
/// physics glitch and is a sign convention.
urdf::Quat slerp(const urdf::Quat& a, urdf::Quat b, double t)
{
    double d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0) {
        b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w;
        d = -d;
    }
    urdf::Quat out;
    if (d > 0.9995) {
        out.x = a.x + t * (b.x - a.x);
        out.y = a.y + t * (b.y - a.y);
        out.z = a.z + t * (b.z - a.z);
        out.w = a.w + t * (b.w - a.w);
    }
    else {
        const double theta = std::acos(std::max(-1.0, std::min(1.0, d)));
        const double s = std::sin(theta);
        const double wa = std::sin((1.0 - t) * theta) / s;
        const double wb = std::sin(t * theta) / s;
        out.x = wa * a.x + wb * b.x;
        out.y = wa * a.y + wb * b.y;
        out.z = wa * a.z + wb * b.z;
        out.w = wa * a.w + wb * b.w;
    }
    const double n = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z + out.w * out.w);
    if (n > 0) { out.x /= n; out.y /= n; out.z /= n; out.w /= n; }
    return out;
}

} // namespace

struct NewtonSidecarUrdfBackend::Impl
{
    Options options;

#if MPM_SHM_SUPPORTED
    MpmVehicleCommandBlock* command = nullptr;
    int command_fd = -1;
    const MpmVehiclePoseBlock* pose = nullptr;
    int pose_fd = -1;

    /// ⚠ WHICH FILE EACH MAPPING IS OF, so a segment replaced underneath us can be NOTICED.
    /// `tools/sidecar_ctl.sh` deletes and recreates every segment on restart — it has to, because
    /// a stale particle block read once and then never updated is exactly the "sand appeared and
    /// disappeared" the operator reported. But `mmap` keeps the OLD, now-unlinked inode alive, so
    /// this side goes on writing commands into memory nobody will ever read and goes on reading
    /// poses that will never change, with every counter reporting connected. That is why every
    /// sidecar restart needed a PIE restart to go with it.
    dev_t command_dev = 0; ino_t command_ino = 0;
    dev_t pose_dev = 0;    ino_t pose_ino = 0;
    dev_t static_dev = 0;  ino_t static_ino = 0;
    /// Countdown to the next revalidation; stat-ing three paths every physics step would be three
    /// syscalls at 333 Hz to answer a question that changes at human speed.
    int revalidate_countdown = 0;
#endif

    /// The two samples the current render time falls between. `previous` may be empty at startup.
    PoseSample previous;
    PoseSample latest;
    bool have_previous = false;
    bool have_latest = false;

    double playback_time = 0.0;      ///< where we are rendering, in the sidecar's time base
    double seconds_since_new = 0.0;
    /// ⚠ HOW FAST THE SIDECAR'S SIMULATED TIME ADVANCES PER SECOND OF OURS. Not 1.0, and assuming
    /// it was is the bug this exists to fix: at 1.37 M sand particles the sidecar runs at roughly
    /// 0.18x real time, so a playback clock advanced by our own dt outruns it more than five times
    /// over, pins itself against the extrapolation clamp and snaps back on every publication.
    /// Measured live rather than configured, because it depends on the bed, the GPU and what else
    /// is running.
    double observed_rate = 1.0;
    double wall_since_sample = 0.0;
    bool rate_seeded = false;
    bool stale_warned = false;
    bool connected_logged = false;
    uint64_t extrapolated = 0;
    uint64_t command_step = 0;

    /// Targets accumulated by setJointTarget between steps, indexed like joint_names_.
    std::vector<urdf::ControlMode> modes;
    std::vector<double> targets;

    /// Where each of our joints sits in the published joint array, resolved by name once.
    std::vector<int> wire_joint_index;
    /// Where each of our links sits in the published link array.
    std::vector<int> wire_link_index;
    bool mapping_resolved = false;
    bool mapping_warned = false;
    bool wrench_warned = false;

    /// ⚠ THE SIDECAR'S FRAME IS ITS OWN, AND ANCHORING IT IS NOT OPTIONAL.
    ///
    /// `BackendOptions::root_position` is where the plugin says the root link belongs, in the
    /// SOLVER frame. Every other backend builds the robot there. The sidecar cannot: it is a
    /// separate process that was started before the editor and placed its vehicle wherever its
    /// own --own-vehicle-x said. Publishing those coordinates as if they were solver-frame put a
    /// Scout at Unreal world (-1.2, 0) while this level's play area sits ~(121.5, -24.6) m away,
    /// so the robot rendered perfectly and was 125 m off screen. `simGetObjectPose` said
    /// (-122.707, +24.642) against a PlayerStart at the origin.
    ///
    /// The offset is measured ONCE, from the first sample: whatever the sidecar calls the root's
    /// position then is defined to be `root_position`. Everything after is shifted by the same
    /// vector, so the robot appears exactly where the settings asked and the sidecar stays free to
    /// use whatever coordinates it likes.
    urdf::Vec3 requested_root;
    urdf::Vec3 frame_offset;
    bool offset_resolved = false;
    /// Incremented by reset(); published on every command so the sidecar cannot miss the edge.
    uint64_t reset_epoch = 0;
    /// ⚠ RANDOM, ONCE, PER BACKEND — and that is the entire point. This backend is constructed
    /// fresh for every PIE session, so every COUNTER on it restarts from the same base the last
    /// session started from: stop PIE without pressing BackSpace, press Play, and the sidecar is
    /// sent reset_epoch 0 (already applied) and static revision 1 (already built). No edge on
    /// either, so it carries on with the previous session's rutted bed and the robot wherever it
    /// was left. Reported on 2026-08-27 as "the reset does not put the robot back", and
    /// misdiagnosed at the time as anchor drift. A random value cannot collide across sessions.
    uint64_t session_id = 0;

    std::shared_ptr<const urdf::StaticWorld> static_world;
#if MPM_SHM_SUPPORTED
    MpmStaticWorldBlock* static_block = nullptr;
    int static_fd = -1;
#endif
    bool static_published = false;
    uint32_t static_revision = 0;

    /// Mirrored dynamic actors: shapes registered once, poses refreshed every tick.
    std::vector<std::vector<urdf::Collision>> link_collisions;
    std::vector<urdf::KinematicBody> kinematic;
    /// name -> (interact_with_mpm, collide_with_robots), from an authored NewtonPhysicsComponent.
    std::map<std::string, std::pair<bool, bool>> kinematic_flags;
    struct KinPose { urdf::Vec3 position; urdf::Quat orientation; };
    std::vector<KinPose> kinematic_poses;
    bool kinematic_warned = false;
};

NewtonSidecarUrdfBackend::NewtonSidecarUrdfBackend(Options options)
    : impl_(new Impl())
{
    // ⚠ SEEDED FROM THE CLOCK AND THE ADDRESS, not from a counter. Two editors launched in the
    // same second must not produce the same id, and neither must two Play presses.
    {
        std::random_device rd;
        std::mt19937_64 gen(static_cast<uint64_t>(rd()) ^
                            static_cast<uint64_t>(std::chrono::steady_clock::now()
                                                      .time_since_epoch().count()) ^
                            reinterpret_cast<uintptr_t>(this));
        impl_->session_id = gen();
        if (impl_->session_id == 0) impl_->session_id = 1;   // 0 means "not set" on the wire
    }

    impl_->options = std::move(options);
}

NewtonSidecarUrdfBackend::~NewtonSidecarUrdfBackend()
{
#if MPM_SHM_SUPPORTED
    if (impl_->command) ::munmap(impl_->command, sizeof(MpmVehicleCommandBlock));
    if (impl_->command_fd >= 0) ::close(impl_->command_fd);
    if (impl_->pose)
        ::munmap(const_cast<MpmVehiclePoseBlock*>(impl_->pose), sizeof(MpmVehiclePoseBlock));
    if (impl_->pose_fd >= 0) ::close(impl_->pose_fd);
    if (impl_->static_block) ::munmap(impl_->static_block, sizeof(MpmStaticWorldBlock));
    if (impl_->static_fd >= 0) ::close(impl_->static_fd);
#endif
}

void NewtonSidecarUrdfBackend::setStaticWorld(std::shared_ptr<const urdf::StaticWorld> world)
{
    // ⚠ STORED, NOT PUBLISHED YET. Publishing needs `frame_offset`, and that is only known once the
    // sidecar has told us where it thinks its robot is — see step(). Writing 24 MB here with a zero
    // offset would put the level a hundred metres from the robot standing on it.
    impl_->static_world = std::move(world);
    impl_->static_published = false;
}

namespace
{
/// Copy the END of a long name into a fixed buffer, prefixed with "~" when it was shortened.
///
/// ⚠ Unreal path names share a long, identical prefix and differ only at the tail, so a head
/// truncation throws away the identifying half and keeps the useless one. Everything that reads
/// these — the contact reporter, the sidecar's logs — is trying to answer "WHICH object is this".
void copyTailName(char* dst, size_t cap, const std::string& name)
{
    if (cap == 0) return;
    if (name.size() < cap) {
        std::snprintf(dst, cap, "%s", name.c_str());
        return;
    }
    const std::string tail = name.substr(name.size() - (cap - 2));
    std::snprintf(dst, cap, "~%s", tail.c_str());
}
} // namespace

namespace
{
#if MPM_SHM_SUPPORTED
/// True when `path` is no longer the file this side mapped — deleted, or replaced by a new one.
bool segmentReplaced(const std::string& path, dev_t dev, ino_t ino)
{
    struct stat info;
    if (::stat(path.c_str(), &info) != 0)
        return true;                                  // gone entirely
    return info.st_dev != dev || info.st_ino != ino;  // a DIFFERENT file now lives at that name
}
#endif
} // namespace

void NewtonSidecarUrdfBackend::revalidateSegments()
{
#if MPM_SHM_SUPPORTED
    // ⚠ ONCE A SECOND-ISH, not every step. Three stat() calls at 333 Hz to answer a question that
    // changes when a human restarts a process is a thousand syscalls a second for nothing.
    if (--impl_->revalidate_countdown > 0)
        return;
    impl_->revalidate_countdown = 200;

    const std::string dir = impl_->options.directory + "/";

    // ---- commands: WE create this one, so remap it and carry on ----------------------------
    if (impl_->command != nullptr &&
        segmentReplaced(dir + kVehicleCommandSegment, impl_->command_dev, impl_->command_ino)) {
        common_utils::Utils::log(
            "NewtonSidecar: the command segment was replaced (the sidecar restarted). Re-attaching "
            "and republishing the level — until now this needed a PIE restart as well.",
            common_utils::Utils::kLogLevelInfo);
        ::munmap(impl_->command, sizeof(MpmVehicleCommandBlock));
        if (impl_->command_fd >= 0) ::close(impl_->command_fd);
        impl_->command = nullptr;
        impl_->command_fd = -1;
        attachCommandSegment();

        // ⚠ AND EVERYTHING WE HAD ALREADY TOLD IT IS NOW UNSAID. A restarted sidecar has no level,
        // no registration and no anchor; leaving `static_published` true would leave it running on
        // its flat fallback ground with the robot colliding with nothing, which is precisely what
        // the operator saw all afternoon.
        impl_->static_published = false;
        impl_->offset_resolved = false;
    }

    // ---- poses: the sidecar owns this one; drop ours and let step() re-attach --------------
    if (impl_->pose != nullptr &&
        segmentReplaced(dir + kVehiclePoseSegment, impl_->pose_dev, impl_->pose_ino)) {
        ::munmap(const_cast<MpmVehiclePoseBlock*>(impl_->pose), sizeof(MpmVehiclePoseBlock));
        if (impl_->pose_fd >= 0) ::close(impl_->pose_fd);
        impl_->pose = nullptr;
        impl_->pose_fd = -1;
        // ⚠ AND THE SAMPLES WE ARE INTERPOLATING BETWEEN BELONG TO A DEAD RUN. Keeping them would
        // interpolate the new sidecar's first pose against the old one's last, which on a robot
        // that has been driven is a link snapping across the map.
        impl_->have_latest = false;
        impl_->have_previous = false;
    }

    // ---- the level: ours, and republished by clearing the flag -----------------------------
    if (impl_->static_block != nullptr &&
        segmentReplaced(dir + kStaticWorldSegment, impl_->static_dev, impl_->static_ino)) {
        ::munmap(impl_->static_block, sizeof(MpmStaticWorldBlock));
        if (impl_->static_fd >= 0) ::close(impl_->static_fd);
        impl_->static_block = nullptr;
        impl_->static_fd = -1;
        impl_->static_published = false;
    }
#endif
}

void NewtonSidecarUrdfBackend::publishStaticWorld()
{
#if MPM_SHM_SUPPORTED
    if (impl_->static_published || impl_->static_world == nullptr || !impl_->offset_resolved)
        return;

    if (impl_->static_block == nullptr) {
        const std::string path = impl_->options.directory + "/" + kStaticWorldSegment;
        // We are the writer, so this one is created here — same reasoning as the command segment.
        impl_->static_fd = ::open(path.c_str(), O_RDWR | O_CREAT, 0666);
        if (impl_->static_fd < 0) {
            common_utils::Utils::log("NewtonSidecar: cannot open " + path +
                                     " — the sidecar will fall back to its flat ground plane.",
                                     common_utils::Utils::kLogLevelError);
            impl_->static_published = true;   // do not retry every tick
            return;
        }
        if (::ftruncate(impl_->static_fd, sizeof(MpmStaticWorldBlock)) != 0) {
            // Not fatal on its own; the size check below is the authority.
        }
        struct stat info;
        if (::fstat(impl_->static_fd, &info) != 0 ||
            static_cast<size_t>(info.st_size) != sizeof(MpmStaticWorldBlock)) {
            common_utils::Utils::log(
                "NewtonSidecar: " + path + " is the wrong size — PROTOCOL MISMATCH between the "
                "plugin and the sidecar. Delete the segment and restart both.",
                common_utils::Utils::kLogLevelError);
            ::close(impl_->static_fd);
            impl_->static_fd = -1;
            impl_->static_published = true;
            return;
        }
        void* address = ::mmap(nullptr, sizeof(MpmStaticWorldBlock), PROT_READ | PROT_WRITE,
                               MAP_SHARED, impl_->static_fd, 0);
        if (address == MAP_FAILED) {
            ::close(impl_->static_fd);
            impl_->static_fd = -1;
            impl_->static_published = true;
            return;
        }
        impl_->static_block = static_cast<MpmStaticWorldBlock*>(address);
        impl_->static_dev = info.st_dev;
        impl_->static_ino = info.st_ino;
    }

    MpmStaticWorldBlock* b = impl_->static_block;
    auto* seq = reinterpret_cast<volatile std::atomic<uint32_t>*>(&b->sequence);
    seq->fetch_add(1, std::memory_order_release);      // odd: writing

    b->magic = kStaticWorldMagic;
    b->version = kProtocolVersion;
    b->frame_offset.x = impl_->frame_offset.x;
    b->frame_offset.y = impl_->frame_offset.y;
    b->frame_offset.z = impl_->frame_offset.z;

    uint32_t bodies = 0, shapes = 0, verts = 0, indices = 0;
    bool truncated = false;

    const urdf::StaticWorld& world = *impl_->static_world;
    for (const urdf::StaticBody& sb : world.bodies) {
        if (bodies >= kMaxStaticBodies) { truncated = true; break; }

        // ⚠ MEASURE BEFORE COMMITTING. A body written half-way — some shapes in, the pools full
        // before the rest — is a solid that is missing a face, and a robot drives through a missing
        // face exactly as it would through geometry that was never sent. Bodies are all-or-nothing.
        uint32_t need_shapes = 0, need_verts = 0, need_indices = 0;
        for (const urdf::StaticShape& ss : sb.shapes) {
            ++need_shapes;
            need_verts += static_cast<uint32_t>(ss.points.size());
            need_indices += static_cast<uint32_t>(ss.indices.size());
        }
        if (shapes + need_shapes > kMaxStaticShapes ||
            verts + need_verts > kMaxStaticVertices ||
            indices + need_indices > kMaxStaticIndices) {
            truncated = true;
            break;
        }

        WireStaticBody& wb = b->bodies[bodies];
        copyTailName(wb.name, kMaxStaticNameChars, sb.name);
        wb.position.x = sb.position.x;
        wb.position.y = sb.position.y;
        wb.position.z = sb.position.z;
        wb.orientation.x = sb.orientation.x;
        wb.orientation.y = sb.orientation.y;
        wb.orientation.z = sb.orientation.z;
        wb.orientation.w = sb.orientation.w;
        wb.friction = sb.friction;
        wb.restitution = sb.restitution;
        wb.shape_start = shapes;
        wb.shape_count = need_shapes;

        for (const urdf::StaticShape& ss : sb.shapes) {
            WireStaticShape& ws = b->shapes[shapes++];
            ws.kind = static_cast<uint32_t>(ss.kind);
            ws.radius = ss.radius;
            ws.center_a.x = ss.center_a.x;
            ws.center_a.y = ss.center_a.y;
            ws.center_a.z = ss.center_a.z;
            ws.center_b.x = ss.center_b.x;
            ws.center_b.y = ss.center_b.y;
            ws.center_b.z = ss.center_b.z;
            ws.vertex_start = verts;
            ws.vertex_count = static_cast<uint32_t>(ss.points.size());
            for (const urdf::Vec3& p : ss.points) {
                b->vertices[verts * 3 + 0] = static_cast<float>(p.x);
                b->vertices[verts * 3 + 1] = static_cast<float>(p.y);
                b->vertices[verts * 3 + 2] = static_cast<float>(p.z);
                ++verts;
            }
            ws.index_start = indices;
            ws.index_count = static_cast<uint32_t>(ss.indices.size());
            for (int idx : ss.indices)
                b->indices[indices++] = idx;
        }
        ++bodies;
    }

    // ⚠ THE MOVING ACTORS SHARE THE POOLS AND THE REVISION. Publishing them in a second message
    // would let a consumer hold shapes from one registration and poses meant for another; keeping
    // them here makes that impossible by construction.
    uint32_t kin = 0;
    for (size_t k = 0; k < impl_->kinematic.size(); ++k) {
        const urdf::KinematicBody& kb = impl_->kinematic[k];
        if (kin >= kMaxKinematicBodies) { truncated = true; break; }
        uint32_t need_shapes = 0, need_verts = 0, need_indices = 0;
        for (const urdf::StaticShape& ss : kb.shapes) {
            ++need_shapes;
            need_verts += static_cast<uint32_t>(ss.points.size());
            need_indices += static_cast<uint32_t>(ss.indices.size());
        }
        if (shapes + need_shapes > kMaxStaticShapes ||
            verts + need_verts > kMaxStaticVertices ||
            indices + need_indices > kMaxStaticIndices) {
            truncated = true;
            break;
        }
        WireKinematicBody& wk = b->kinematic[kin];
        // ⚠ KEEP THE TAIL, NOT THE HEAD. These names are Unreal path names —
        // "/Game/FlyingCPP/Maps/UEDPIE_0_execo_test.execo_test:PersistentLevel.StaticMeshActor_0
        // .StaticMeshComponent0" — and a 64-character head truncation keeps the map path, which is
        // identical for every actor in the level, and discards the actor name, which is the only
        // part anyone needs. Reported 2026-08-27 when a contact list named the object the robot was
        // stuck against as "...execo_test:PersistentL" and could not distinguish it from anything
        // else in the map.
        copyTailName(wk.name, kMaxStaticNameChars, kb.name);
        wk.friction = kb.friction;
        wk.restitution = kb.restitution;
        // ⚠ DEFAULTS ARE BOTH ON. An actor with no NewtonPhysicsComponent must behave exactly as
        // it did before the component existed; the flags are narrowed only where one was authored.
        wk.interact_with_mpm = 1;
        wk.collide_with_robots = 1;
        auto it = impl_->kinematic_flags.find(kb.name);
        if (it != impl_->kinematic_flags.end()) {
            wk.interact_with_mpm = it->second.first ? 1u : 0u;
            wk.collide_with_robots = it->second.second ? 1u : 0u;
        }
        wk.shape_start = shapes;
        wk.shape_count = need_shapes;
        // ⚠ SHIFTED INTO THE SIDECAR'S FRAME, like every other pose on this wire. The registration
        // pose is in the solver frame; the sidecar builds in its own.
        const auto& kpose = (k < impl_->kinematic_poses.size())
                                ? impl_->kinematic_poses[k]
                                : Impl::KinPose{kb.position, kb.orientation};
        wk.position.x = kpose.position.x - impl_->frame_offset.x;
        wk.position.y = kpose.position.y - impl_->frame_offset.y;
        wk.position.z = kpose.position.z - impl_->frame_offset.z;
        wk.orientation.x = kpose.orientation.x;
        wk.orientation.y = kpose.orientation.y;
        wk.orientation.z = kpose.orientation.z;
        wk.orientation.w = kpose.orientation.w;
        for (const urdf::StaticShape& ss : kb.shapes) {
            WireStaticShape& ws = b->shapes[shapes++];
            ws.kind = static_cast<uint32_t>(ss.kind);
            ws.radius = ss.radius;
            ws.center_a.x = ss.center_a.x; ws.center_a.y = ss.center_a.y;
            ws.center_a.z = ss.center_a.z;
            ws.center_b.x = ss.center_b.x; ws.center_b.y = ss.center_b.y;
            ws.center_b.z = ss.center_b.z;
            ws.vertex_start = verts;
            ws.vertex_count = static_cast<uint32_t>(ss.points.size());
            for (const urdf::Vec3& p : ss.points) {
                b->vertices[verts * 3 + 0] = static_cast<float>(p.x);
                b->vertices[verts * 3 + 1] = static_cast<float>(p.y);
                b->vertices[verts * 3 + 2] = static_cast<float>(p.z);
                ++verts;
            }
            ws.index_start = indices;
            ws.index_count = static_cast<uint32_t>(ss.indices.size());
            for (int idx : ss.indices)
                b->indices[indices++] = idx;
        }
        ++kin;
    }
    b->kinematic_count = kin;

    b->body_count = bodies;
    b->shape_count = shapes;
    b->vertex_count = verts;
    b->index_count = indices;
    b->truncated = truncated ? 1u : 0u;
    b->revision = ++impl_->static_revision;
    seq->fetch_add(1, std::memory_order_release);      // even: readable

    impl_->static_published = true;

    common_utils::Utils::log(
        common_utils::Utils::stringf(
            "NewtonSidecar '%s': mirrored the level to the sidecar — %u static bodies, %u moving "
            "actors, %u shapes, %u vertices, %u triangles, offset (%+.3f %+.3f %+.3f), "
            "revision %u.",
            impl_->options.vehicle_name.c_str(), bodies, kin, shapes, verts, indices / 3,
            impl_->frame_offset.x, impl_->frame_offset.y, impl_->frame_offset.z,
            b->revision),
        common_utils::Utils::kLogLevelInfo);

    // ⚠ TRUNCATION IS AN ERROR, NOT A FOOTNOTE. What the sidecar builds is then a level with holes
    // in it, and a robot falls through a floor that was dropped exactly as it would through one
    // that never existed. Say which limit bound so it can be raised deliberately.
    if (truncated) {
        common_utils::Utils::log(
            common_utils::Utils::stringf(
                "NewtonSidecar '%s': the level did NOT fit and the mirror is PARTIAL — stopped at "
                "%u/%u bodies, %u/%u shapes, %u/%u vertices, %u/%u indices. The sidecar's robot "
                "will pass through whatever was dropped. Raise the kMaxStatic* caps in "
                "MpmSidecarProtocol.hpp and protocol.py together.",
                impl_->options.vehicle_name.c_str(),
                bodies, kMaxStaticBodies, shapes, kMaxStaticShapes,
                verts, kMaxStaticVertices, indices, kMaxStaticIndices),
            common_utils::Utils::kLogLevelError);
    }
#endif
}

bool NewtonSidecarUrdfBackend::staticWorldPublished() const
{
    return impl_->static_published;
}

void NewtonSidecarUrdfBackend::setKinematicFlags(const std::string& name, bool interact_with_mpm,
                                                 bool collide_with_robots)
{
    // ⚠ KEYED BY NAME, and set BEFORE addKinematicBody, because the caller resolves the authored
    // component from the Unreal actor and the sidecar only ever sees the name.
    impl_->kinematic_flags[name] = {interact_with_mpm, collide_with_robots};
    impl_->static_published = false;
}

int NewtonSidecarUrdfBackend::addKinematicBody(const urdf::KinematicBody& body)
{
    // ⚠ REGISTERED HERE, PUBLISHED WITH THE STATIC WORLD. Both halves of the level mirror are known
    // at load and share one revision, so a consumer can never be holding shapes from one
    // registration and poses meant for another.
    if (impl_->kinematic.size() >= kMaxKinematicBodies) {
        if (!impl_->kinematic_warned) {
            impl_->kinematic_warned = true;
            common_utils::Utils::log(
                common_utils::Utils::stringf(
                    "NewtonSidecar '%s': more than %u moving actors in this level; the rest are "
                    "NOT mirrored and the robot will pass through them.",
                    impl_->options.vehicle_name.c_str(), kMaxKinematicBodies),
                common_utils::Utils::kLogLevelError);
        }
        return -1;
    }
    impl_->kinematic.push_back(body);
    impl_->kinematic_poses.push_back({body.position, body.orientation});
    // ⚠ The registration changed, so anything already published is stale.
    impl_->static_published = false;
    return static_cast<int>(impl_->kinematic.size()) - 1;
}

void NewtonSidecarUrdfBackend::setKinematicPose(int handle, const urdf::Vec3& position,
                                                const urdf::Quat& orientation)
{
    // ⚠ STORED, NOT SENT HERE. This is called once per mirrored body per tick, from the game
    // thread; writing the shared block per call would take the seqlock N times a frame and let a
    // reader see half the actors moved. step() publishes them all inside one update.
    if (handle < 0 || handle >= static_cast<int>(impl_->kinematic_poses.size()))
        return;
    impl_->kinematic_poses[handle] = {position, orientation};
}

void NewtonSidecarUrdfBackend::attachCommandSegment()
{
#if MPM_SHM_SUPPORTED
    const std::string command_path = impl_->options.directory + "/" + kVehicleCommandSegment;
    const std::string pose_path = impl_->options.directory + "/" + kVehiclePoseSegment;

    // ⚠ THE COMMAND SEGMENT IS CREATED IF ABSENT; THE POSE SEGMENT IS NOT.
    //
    // We are the WRITER of commands, so a sidecar started later must find our block waiting. We
    // are only a READER of poses, and creating that one would let a simulator started first
    // present an all-zero block that the sidecar then overwrites — the window between is a robot
    // rendered at the origin while every counter reports connected. Missing is the honest state,
    // and step() reports it.
    impl_->command_fd = ::open(command_path.c_str(), O_RDWR | O_CREAT, 0666);
    if (impl_->command_fd < 0)
        throw std::runtime_error("NewtonSidecar backend: cannot open " + command_path);
    if (::ftruncate(impl_->command_fd, sizeof(MpmVehicleCommandBlock)) != 0) {
        // ⚠ Not fatal on its own: another process may have created it at the right size already,
        // and ftruncate on some filesystems refuses to shrink-or-grow a mapped file. The size
        // check below is the authority.
    }
    struct stat info;
    if (::fstat(impl_->command_fd, &info) != 0 ||
        static_cast<size_t>(info.st_size) != sizeof(MpmVehicleCommandBlock)) {
        ::close(impl_->command_fd);
        impl_->command_fd = -1;
        throw std::runtime_error(
            "NewtonSidecar backend: " + command_path + " is the wrong size. This is a PROTOCOL "
            "MISMATCH — the plugin and the sidecar were built from different versions of "
            "MpmSidecarProtocol.hpp / protocol.py. Delete the segment and restart both.");
    }
    void* address = ::mmap(nullptr, sizeof(MpmVehicleCommandBlock), PROT_READ | PROT_WRITE,
                           MAP_SHARED, impl_->command_fd, 0);
    if (address == MAP_FAILED) {
        ::close(impl_->command_fd);
        impl_->command_fd = -1;
        throw std::runtime_error("NewtonSidecar backend: cannot map " + command_path);
    }
    impl_->command = static_cast<MpmVehicleCommandBlock*>(address);
    impl_->command_dev = info.st_dev;
    impl_->command_ino = info.st_ino;
    impl_->command->magic = kVehicleCommandMagic;
    impl_->command->version = kProtocolVersion;

#endif
}

void NewtonSidecarUrdfBackend::buildFromUrdf(const urdf::Robot& model,
                                             const urdf::BackendOptions& opts)
{
    impl_->requested_root = opts.root_position;
    impl_->offset_resolved = false;
    link_names_.clear();
    joint_names_.clear();
    total_mass_ = 0.0;

    // ⚠ THE URDF IS PARSED HERE FOR ITS NAMES AND MASSES ONLY. The sidecar builds the robot from
    // its own copy of the same file; this side never integrates anything. Keeping the name list in
    // URDF order matters because every consumer above (sensors, ROS, the render component)
    // addresses links by the index this vector defines.
    impl_->link_collisions.clear();
    for (const auto& link : model.links) {
        link_names_.push_back(link.name);
        total_mass_ += link.inertial.mass;
        // ⚠ KEPT FOR THE OVERLAY ONLY. Nothing here is simulated on this side; these are the URDF's
        // own collision primitives, drawn at the poses Newton reports so an operator can see where
        // the solver believes the robot is rather than infer it from what the sand does.
        impl_->link_collisions.push_back(link.collisions);
    }
    for (const auto& joint : model.joints)
        joint_names_.push_back(joint.name);

    impl_->modes.assign(joint_names_.size(), urdf::ControlMode::None);
    impl_->targets.assign(joint_names_.size(), 0.0);
    impl_->wire_joint_index.assign(joint_names_.size(), -1);
    impl_->wire_link_index.assign(link_names_.size(), -1);

    if (impl_->options.vehicle_name.empty())
        throw std::invalid_argument(
            "NewtonSidecar backend: no vehicle name was set. It must match the sidecar's "
            "--own-vehicle spec name, which is how one wire block carries several vehicles.");

#if !MPM_SHM_SUPPORTED
    throw std::runtime_error("NewtonSidecar backend needs POSIX shared memory; this platform has "
                             "none compiled in.");
#else
    attachCommandSegment();
    // The pose segment may legitimately not exist yet — the sidecar creates it. step() retries.
#endif
}

void NewtonSidecarUrdfBackend::reset()
{
    impl_->have_previous = false;
    impl_->have_latest = false;
    impl_->playback_time = 0.0;
    impl_->seconds_since_new = 0.0;
    impl_->extrapolated = 0;
    impl_->stale_warned = false;
    std::fill(impl_->modes.begin(), impl_->modes.end(), urdf::ControlMode::None);
    std::fill(impl_->targets.begin(), impl_->targets.end(), 0.0);
    // ⚠ The MAPPING is deliberately NOT cleared. Link and joint names do not change across a
    // reset, and re-resolving would re-emit the "names the sidecar does not have" warning on every
    // global reset.
    //
    // ⚠ THE ANCHOR IS DELIBERATELY *NOT* RE-RESOLVED, and an earlier version clearing it here was
    // actively misleading. Re-anchoring shifts the whole picture so the robot's CURRENT position
    // maps onto the spawn — which drags the sand along with it, because the bed is drawn through
    // the same offset. The operator sees the robot and its ruts teleport to the player start with
    // every rut intact. That is a camera move dressed as a reset.
    //
    // The anchor stays valid across a reset precisely because the sidecar puts its vehicle back at
    // its own spawn, which is what the offset was computed against in the first place.
    //
    // ⚠ THIS IS WHAT ACTUALLY RESETS ANYTHING. The vehicle is solved in another process, so the
    // simulator resetting its own world reaches none of it. Bumping the epoch is the request; the
    // sidecar rebuilds its model — fresh bed, vehicle at spawn — and the next poses reflect it.
    // ⚠ Safe to re-resolve NOW that the anchor is a declared constant rather than a sampled pose:
    // the sidecar rebuilds to the same spawn, so this recomputes the same offset. It exists so a
    // sidecar restarted with a different --own-vehicle-x is picked up instead of silently using a
    // stale offset.
    impl_->offset_resolved = false;
    ++impl_->reset_epoch;
    common_utils::Utils::log(
        common_utils::Utils::stringf(
            "NewtonSidecar '%s': reset requested (epoch %llu) — the sidecar will rebuild its sand "
            "and return the vehicle to its spawn. Expect a pause while it does.",
            impl_->options.vehicle_name.c_str(),
            static_cast<unsigned long long>(impl_->reset_epoch)),
        common_utils::Utils::kLogLevelInfo);
}

#if MPM_SHM_SUPPORTED
namespace
{

/// Copy the pose block through its seqlock. Returns false if the writer was mid-update.
///
/// ⚠ A TORN READ ON AN ARTICULATED BODY IS INDISTINGUISHABLE FROM A JOINT SNAPPING — some links
/// from step N and some from N+1 is a pose no configuration of the robot can produce. The sequence
/// is ODD while the writer is inside an update; both loads are acquire so the compiler cannot hoist
/// the body copy outside them.
bool readPoseBlock(const MpmVehiclePoseBlock* src, MpmVehiclePoseBlock& out)
{
    const auto* seq = reinterpret_cast<const volatile std::atomic<uint32_t>*>(&src->sequence);
    for (int attempt = 0; attempt < 8; ++attempt) {
        const uint32_t before = seq->load(std::memory_order_acquire);
        if (before % 2 != 0)
            continue;
        std::memcpy(&out, src, sizeof(MpmVehiclePoseBlock));
        const uint32_t after = seq->load(std::memory_order_acquire);
        if (before == after)
            return true;
    }
    return false;
}

} // namespace
#endif

int NewtonSidecarUrdfBackend::step(double dt)
{
#if !MPM_SHM_SUPPORTED
    (void)dt;
    return 0;
#else
    // ---- publish the joint targets -------------------------------------------------------
    if (impl_->command) {
        MpmVehicleCommandBlock* b = impl_->command;
        // Seqlock: odd while we are inside the update.
        auto* seq = reinterpret_cast<volatile std::atomic<uint32_t>*>(&b->sequence);
        seq->fetch_add(1, std::memory_order_release);

        b->magic = kVehicleCommandMagic;
        b->version = kProtocolVersion;
        b->vehicle_count = 1;
        b->step = ++impl_->command_step;
        b->simulation_time = impl_->playback_time;
        // ⚠ Published EVERY tick, not only on the tick it changes. The sidecar samples this block
        // latest-wins and may miss any individual write; carrying the current value means it
        // notices the edge whenever it next looks, rather than depending on catching one frame.
        b->reset_epoch = impl_->reset_epoch;
        // ⚠ Also every tick, for the same reason: the sidecar must be able to notice a new session
        // whenever it next looks, not only on the one frame the session began.
        b->session_id = impl_->session_id;

        // ⚠ ALL OF THEM INSIDE ONE UPDATE. setKinematicPose is called once per body per tick from
        // the game thread; taking the seqlock there would let a reader see half the actors moved,
        // which on a level with a moving platform is a solid that has torn in two.
        //
        // ⚠ The revision is stamped so the sidecar can refuse poses belonging to a registration it
        // has not built. Applying pose[i] to the wrong body is a silent, plausible-looking error.
        b->kinematic_revision = impl_->static_revision;
        const uint32_t kin_n = static_cast<uint32_t>(
            std::min<size_t>(impl_->kinematic_poses.size(), kMaxKinematicBodies));
        b->kinematic_count = kin_n;
        for (uint32_t i = 0; i < kin_n; ++i) {
            const auto& kp = impl_->kinematic_poses[i];
            // Shifted into the sidecar's frame by the same offset its poses come back through.
            b->kinematic_poses[i].position.x = kp.position.x - impl_->frame_offset.x;
            b->kinematic_poses[i].position.y = kp.position.y - impl_->frame_offset.y;
            b->kinematic_poses[i].position.z = kp.position.z - impl_->frame_offset.z;
            b->kinematic_poses[i].orientation.x = kp.orientation.x;
            b->kinematic_poses[i].orientation.y = kp.orientation.y;
            b->kinematic_poses[i].orientation.z = kp.orientation.z;
            b->kinematic_poses[i].orientation.w = kp.orientation.w;
        }

        WireVehicleCommand& v = b->vehicles[0];
        std::snprintf(v.vehicle_name, kMaxVehicleNameChars, "%s",
                      impl_->options.vehicle_name.c_str());
        uint32_t n = 0;
        for (size_t j = 0; j < joint_names_.size() && n < kMaxJointsPerVehicle; ++j) {
            // ⚠ MODE None IS OMITTED, NOT SENT AS ZERO. A passive rocker-bogie joint must stay
            // free to swing; commanding it to velocity 0 is a brake, and a braked rocker lifts
            // wheels off the ground on uneven terrain.
            if (impl_->modes[j] == urdf::ControlMode::None)
                continue;
            WireJointCommand& jc = v.joints[n++];
            std::snprintf(jc.joint_name, kMaxLinkNameChars, "%s", joint_names_[j].c_str());
            // Only velocity crosses the wire today; the sidecar says so if it is sent anything
            // else, rather than applying a velocity while the caller believed it sent a position.
            jc.target_mode = static_cast<uint32_t>(
                impl_->modes[j] == urdf::ControlMode::Velocity ? WireJointTargetMode::Velocity
                : impl_->modes[j] == urdf::ControlMode::Position ? WireJointTargetMode::Position
                                                                 : WireJointTargetMode::Torque);
            jc.target = impl_->targets[j];
        }
        v.joint_count = n;
        seq->fetch_add(1, std::memory_order_release);
    }

    // ---- attach to the pose segment if the sidecar has since created it -------------------
    if (!impl_->pose) {
        const std::string pose_path = impl_->options.directory + "/" + kVehiclePoseSegment;
        impl_->pose_fd = ::open(pose_path.c_str(), O_RDONLY);
        if (impl_->pose_fd >= 0) {
            struct stat info;
            if (::fstat(impl_->pose_fd, &info) == 0 &&
                static_cast<size_t>(info.st_size) == sizeof(MpmVehiclePoseBlock)) {
                void* address = ::mmap(nullptr, sizeof(MpmVehiclePoseBlock), PROT_READ,
                                       MAP_SHARED, impl_->pose_fd, 0);
                if (address != MAP_FAILED) {
                    impl_->pose = static_cast<const MpmVehiclePoseBlock*>(address);
                    impl_->pose_dev = info.st_dev;
                    impl_->pose_ino = info.st_ino;
                }
            }
            if (!impl_->pose) {
                ::close(impl_->pose_fd);
                impl_->pose_fd = -1;
            }
        }
    }

    // ---- take the newest published sample -------------------------------------------------
    bool got_new = false;
    if (impl_->pose) {
        MpmVehiclePoseBlock snapshot;
        if (readPoseBlock(impl_->pose, snapshot) && snapshot.magic == kVehiclePoseMagic &&
            snapshot.version == kProtocolVersion) {
            for (uint32_t i = 0; i < snapshot.vehicle_count && i < kMaxVehicles; ++i) {
                const WireVehiclePose& v = snapshot.vehicles[i];
                if (impl_->options.vehicle_name != v.vehicle_name)
                    continue;
                if (impl_->have_latest && snapshot.sidecar_step == impl_->latest.step)
                    break;                       // nothing new since last tick

                PoseSample sample;
                sample.time = snapshot.sidecar_time;
                sample.step = snapshot.sidecar_step;
                sample.effort_reported = v.effort_reported != 0;

                if (!impl_->mapping_resolved) {
                    for (size_t l = 0; l < link_names_.size(); ++l) {
                        impl_->wire_link_index[l] = -1;
                        for (uint32_t k = 0; k < v.link_count && k < kMaxLinksPerVehicle; ++k) {
                            // ⚠ SUFFIX MATCH. Newton labels links "robot/link_name" while our URDF
                            // model knows them as "link_name". Comparing whole strings resolved
                            // nothing and left every link at the origin.
                            const std::string wire = v.links[k].link_name;
                            const size_t slash = wire.rfind('/');
                            const std::string bare =
                                slash == std::string::npos ? wire : wire.substr(slash + 1);
                            if (bare == link_names_[l]) {
                                impl_->wire_link_index[l] = static_cast<int>(k);
                                break;
                            }
                        }
                    }
                    for (size_t j = 0; j < joint_names_.size(); ++j) {
                        impl_->wire_joint_index[j] = -1;
                        for (uint32_t k = 0; k < v.joint_count && k < kMaxJointsPerVehicle; ++k) {
                            const std::string wire = v.joints[k].joint_name;
                            const size_t slash = wire.rfind('/');
                            const std::string bare =
                                slash == std::string::npos ? wire : wire.substr(slash + 1);
                            if (bare == joint_names_[j]) {
                                impl_->wire_joint_index[j] = static_cast<int>(k);
                                break;
                            }
                        }
                    }
                    impl_->mapping_resolved = true;
                }

                // ⚠ ANCHOR ON THE FIRST SAMPLE. The root link is link 0 by construction (the URDF
                // order this backend publishes), so whatever the sidecar says it is at now becomes
                // the solver-frame root_position the plugin asked for.
                if (!impl_->offset_resolved && impl_->wire_link_index[0] >= 0) {
                    // ⚠ ANCHOR ON THE DECLARED SPAWN, NEVER ON THIS SAMPLE. The sidecar free-runs,
                    // so by the time we read our first pose its robot has settled or rolled a
                    // little from where it was built — and an offset derived from that sample
                    // absorbs the difference. Every Play and every reset then anchors to a
                    // different instant, and the vehicle reappears a few centimetres from where it
                    // was last time with nothing to explain it. `spawn_position` is a constant the
                    // sidecar states once per model, so the offset is the same every time.
                    const WireVec3& r = v.spawn_position;
                    // ⚠ X AND Y ONLY. Z IS NOT ARBITRARY AND MUST NOT BE ANCHORED.
                    //
                    // Where in the level a robot stands is a free choice, so x/y are shifted to
                    // wherever the settings asked. HEIGHT is not: both worlds have a ground, the
                    // sidecar's at its own `ground_z` and the level's at its floor, and they are
                    // meant to be the same surface. Shifting z by
                    // (settings spawn - sidecar rest height) lifted the whole sidecar world by
                    // 0.565 m, so the Scout and its sand floated visibly clear of the level's
                    // floor — each internally consistent, both wrong against the level.
                    //
                    // The consequence, said plainly because it surprises: for a sidecar-owned
                    // vehicle the settings' Z does NOT place the robot. The sidecar's own spawn
                    // height and ground plane do. Set them with --own-vehicle-z and --ground-z.
                    impl_->frame_offset = urdf::Vec3{
                        impl_->requested_root.x - r.x,
                        impl_->requested_root.y - r.y,
                        0.0};
                    impl_->offset_resolved = true;
                    common_utils::Utils::log(
                        common_utils::Utils::stringf(
                            "NewtonSidecar '%s': anchoring the sidecar frame in X/Y only, on the "
                            "sidecar's DECLARED SPAWN (%.3f %.3f %.3f) rather than on a sampled "
                            "pose; settings ask for (%.3f %.3f %.3f); applying offset "
                            "(%+.3f %+.3f %+.3f). Z is NOT anchored - the sidecar's own ground "
                            "plane and spawn height decide height, so this robot rests where "
                            "--own-vehicle-z and --ground-z put it, not where the settings' Z says.",
                            impl_->options.vehicle_name.c_str(),
                            r.x, r.y, r.z,
                            impl_->requested_root.x, impl_->requested_root.y,
                            impl_->requested_root.z,
                            impl_->frame_offset.x, impl_->frame_offset.y, impl_->frame_offset.z),
                        common_utils::Utils::kLogLevelInfo);
                }
                const urdf::Vec3 off = impl_->frame_offset;

                sample.poses.resize(link_names_.size());
                sample.twists.resize(link_names_.size());
                for (size_t l = 0; l < link_names_.size(); ++l) {
                    const int k = impl_->wire_link_index[l];
                    if (k < 0)
                        continue;               // stays identity; reported once below
                    const WireLinkPose& w = v.links[k];
                    // ⚠ POSITION ONLY. The offset is a translation, so twists and orientations are
                    // unchanged by it; rotating the sidecar's frame is not supported and would need
                    // the orientation half of root_orientation applied to every vector here.
                    sample.poses[l].position = urdf::Vec3{w.position.x + off.x,
                                                          w.position.y + off.y,
                                                          w.position.z + off.z};
                    sample.poses[l].orientation = urdf::Quat{w.orientation.x, w.orientation.y,
                                                             w.orientation.z, w.orientation.w};
                    sample.twists[l].linear = urdf::Vec3{w.linear_velocity.x, w.linear_velocity.y,
                                                         w.linear_velocity.z};
                    sample.twists[l].angular = urdf::Vec3{w.angular_velocity.x,
                                                          w.angular_velocity.y,
                                                          w.angular_velocity.z};
                }
                sample.joints.resize(joint_names_.size());
                for (size_t j = 0; j < joint_names_.size(); ++j) {
                    const int k = impl_->wire_joint_index[j];
                    if (k < 0)
                        continue;
                    sample.joints[j].position = v.joints[k].position;
                    sample.joints[j].velocity = v.joints[k].velocity;
                    sample.joints[j].effort = sample.effort_reported ? v.joints[k].effort : 0.0;
                }

                if (impl_->have_latest) {
                    impl_->previous = impl_->latest;
                    impl_->have_previous = true;
                }
                impl_->latest = std::move(sample);
                impl_->have_latest = true;
                got_new = true;
                break;
            }
        }
    }

    // ⚠ BEFORE the republish, because it is what decides whether a republish is owed: if the
    // sidecar restarted, every segment we hold is an orphaned inode and everything we told it is
    // unsaid.
    revalidateSegments();

    // ⚠ AFTER the anchor, not before: the geometry is shifted into the sidecar's frame by the same
    // offset the poses use, and that is only known once the sidecar has told us where its robot is.
    // Returns immediately once published, so this costs a branch per step.
    publishStaticWorld();

    // ---- advance the playback clock --------------------------------------------------------
    //
    // ⚠ THE PLAYBACK CLOCK IS OURS AND RUNS ON OUR dt. It is deliberately not the sidecar's step
    // counter and deliberately not wall time. Following the sidecar's counter would make our frame
    // rate its frame rate — which is the coupling this whole design exists to avoid — and wall time
    // would drift the instant the solve ran slower than real time, which at 1.37 M particles it
    // routinely does.
    impl_->wall_since_sample += dt;

    if (got_new) {
        const double interval = impl_->have_previous
                                    ? impl_->latest.time - impl_->previous.time
                                    : 0.0;
        // ⚠ MEASURE THE RATE FROM SIM TIME AGAINST OUR OWN ELAPSED TIME. `interval` is how much
        // SIMULATED time passed between the last two publications; `wall_since_sample` is how much
        // of ours passed while we waited for this one. Their ratio is how fast the sidecar is
        // actually running relative to us, and it is what the playback clock must advance at.
        if (interval > 0.0 && impl_->wall_since_sample > 1e-6) {
            const double instant = interval / impl_->wall_since_sample;
            // Heavily smoothed: the ratio of two small intervals is noisy, and a playback rate
            // that jitters is visible as speed wobble even when its average is right.
            impl_->observed_rate = impl_->rate_seeded
                                       ? 0.9 * impl_->observed_rate + 0.1 * instant
                                       : instant;
            impl_->rate_seeded = true;
        }
        impl_->wall_since_sample = 0.0;
        impl_->seconds_since_new = 0.0;
        impl_->stale_warned = false;

        const double fallback = interval > 0 ? interval : 1.0 / 30.0;
        const double delay = impl_->options.delay_intervals * fallback;
        const double target = impl_->latest.time - delay;
        // ⚠ RESYNC ONLY WHEN WE HAVE FALLEN OUTSIDE THE BRACKET. Snapping every time a sample
        // arrives would make the playback clock jump by the jitter between publications, and a
        // pose that jumps backwards by a few milliseconds every frame is visible as a stutter even
        // when the average rate is perfect.
        if (impl_->playback_time > impl_->latest.time ||
            impl_->playback_time < target - 4.0 * fallback)
            impl_->playback_time = target;
    }
    else {
        impl_->seconds_since_new += dt;
    }

    // ⚠ ADVANCED AT THE SIDECAR'S RATE, NOT OURS, and clamped so it cannot run past the newest
    // sample it has actually been given. A vehicle rendered in slow motion is TRUE — the physics
    // really is progressing at that rate — whereas one advanced at our clock is a smooth lie that
    // snaps back four times a second.
    impl_->playback_time += dt * (impl_->rate_seeded ? impl_->observed_rate : 1.0);
    if (impl_->have_latest && impl_->playback_time > impl_->latest.time)
        impl_->playback_time = impl_->latest.time;
    return 0;
#endif
}

urdf::LinkPose NewtonSidecarUrdfBackend::getLinkPose(size_t link) const
{
    urdf::LinkPose out;
    if (link >= link_names_.size() || !impl_->have_latest)
        return out;
    if (!impl_->have_previous)
        return impl_->latest.poses[link];

    const PoseSample& a = impl_->previous;
    const PoseSample& b = impl_->latest;
    const double span = b.time - a.time;
    if (span <= 1e-9)
        return b.poses[link];

    double t = (impl_->playback_time - a.time) / span;
    if (t > 1.0) {
        // ⚠ EXTRAPOLATION IS ALLOWED AND COUNTED. When the sidecar is slower than the renderer,
        // holding the last pose would stutter at exactly the sidecar's period; continuing along
        // the published twist is smooth and wrong by a bounded amount. It is capped so a sidecar
        // that has DIED does not send the vehicle off to infinity.
        ++impl_->extrapolated;
        t = std::min(t, 2.0);
    }
    t = std::max(t, 0.0);

    out.position = hermite(a.poses[link].position, a.twists[link].linear,
                           b.poses[link].position, b.twists[link].linear, t, span);
    out.orientation = slerp(a.poses[link].orientation, b.poses[link].orientation,
                            std::min(t, 1.0));
    return out;
}

urdf::Twist NewtonSidecarUrdfBackend::getLinkTwist(size_t link) const
{
    urdf::Twist out;
    if (link >= link_names_.size() || !impl_->have_latest)
        return out;
    if (!impl_->have_previous)
        return impl_->latest.twists[link];

    const PoseSample& a = impl_->previous;
    const PoseSample& b = impl_->latest;
    const double span = b.time - a.time;
    const double t = span > 1e-9
                         ? std::max(0.0, std::min(1.0, (impl_->playback_time - a.time) / span))
                         : 1.0;
    // ⚠ TWIST IS LERPED, NOT HERMITED. A Hermite on velocity would need acceleration as its
    // tangent, which the wire does not carry; using the velocities themselves as tangents would be
    // a units error dressed as smoothing.
    out.linear = lerp(a.twists[link].linear, b.twists[link].linear, t);
    out.angular = lerp(a.twists[link].angular, b.twists[link].angular, t);
    return out;
}

urdf::JointState NewtonSidecarUrdfBackend::getJointState(size_t joint) const
{
    urdf::JointState out;
    if (joint >= joint_names_.size() || !impl_->have_latest)
        return out;
    if (!impl_->have_previous)
        return impl_->latest.joints[joint];

    const PoseSample& a = impl_->previous;
    const PoseSample& b = impl_->latest;
    const double span = b.time - a.time;
    const double t = span > 1e-9
                         ? std::max(0.0, std::min(1.0, (impl_->playback_time - a.time) / span))
                         : 1.0;
    // ⚠ NO ANGLE WRAPPING HERE ON PURPOSE. Newton reports a continuous joint's position as an
    // accumulating angle, not wrapped to [-pi, pi], so a plain lerp is correct. Wrapping it would
    // make a spinning wheel's reported angle jump every revolution, and anything differentiating
    // it would see an impulse.
    out.position = a.joints[joint].position +
                   t * (b.joints[joint].position - a.joints[joint].position);
    out.velocity = a.joints[joint].velocity +
                   t * (b.joints[joint].velocity - a.joints[joint].velocity);
    out.effort = b.joints[joint].effort;
    return out;
}

const std::string& NewtonSidecarUrdfBackend::linkName(size_t link) const
{
    static const std::string empty;
    return link < link_names_.size() ? link_names_[link] : empty;
}

const std::string& NewtonSidecarUrdfBackend::jointName(size_t joint) const
{
    static const std::string empty;
    return joint < joint_names_.size() ? joint_names_[joint] : empty;
}

int NewtonSidecarUrdfBackend::findLink(const std::string& name) const
{
    for (size_t i = 0; i < link_names_.size(); ++i)
        if (link_names_[i] == name) return static_cast<int>(i);
    return -1;
}

int NewtonSidecarUrdfBackend::findJoint(const std::string& name) const
{
    for (size_t i = 0; i < joint_names_.size(); ++i)
        if (joint_names_[i] == name) return static_cast<int>(i);
    return -1;
}

void NewtonSidecarUrdfBackend::setJointTarget(size_t joint, urdf::ControlMode mode, double value)
{
    if (joint >= joint_names_.size())
        return;
    impl_->modes[joint] = mode;
    impl_->targets[joint] = value;
}

void NewtonSidecarUrdfBackend::setPositionGains(size_t, double, double)
{
    // Deliberately nothing. See the note on the declaration: the actuator model lives in the
    // sidecar's vehicle spec, and accepting a gain here that no solver will read would be a knob
    // that does nothing on the one parameter that decides whether the vehicle climbs.
}

void NewtonSidecarUrdfBackend::applyExternalWrench(size_t, const urdf::Wrench&)
{
    // ⚠ LOUD ONCE, THEN NO-OP — AND IT USED TO THROW, WHICH WAS WRONG.
    //
    // The body is in another process and protocol v5 carries no force channel into it, so this
    // genuinely cannot be honoured. The first version threw, on the principle that a silent no-op
    // lets a rotor, a winch or a tow force be applied every tick with no effect and no error.
    // That principle is right and the mechanism was not: `UrdfBotSimApi::applyLinkWrench` calls
    // this on the GAME THREAD, where an escaping exception takes the editor down rather than
    // reporting a problem. Crashing the simulator is a worse answer to "this is unsupported" than
    // any log.
    //
    // Logged once rather than per tick: a message every frame is scrolled past and ignored, which
    // is the same outcome as not warning at all.
    if (!impl_->wrench_warned) {
        impl_->wrench_warned = true;
        common_utils::Utils::log(
            "NewtonSidecar backend: applyExternalWrench is NOT supported and is being ignored. "
            "The vehicle is solved in the sidecar process and protocol v5 carries no force channel "
            "into it. Drive it through a URDF joint the sidecar can actuate, or run this vehicle "
            "on Box3D/MuJoCo. This is reported once per run.",
            common_utils::Utils::kLogLevelError);
    }
}

namespace
{

/// Turn one URDF collision primitive into an overlay geom, placed by `body` in the solver frame.
///
/// ⚠ The URDF's own `origin` is body-LOCAL, so it composes with the body pose rather than replacing
/// it. Getting that wrong draws every shape at its link's origin, which looks plausible on a wheel
/// whose collision is centred and wrong on everything else.
bool overlayGeomFromCollision(const urdf::Collision& c, const urdf::LinkPose& body,
                              urdf::CollisionShape::Provenance provenance,
                              urdf::CollisionShape& out, std::string& unhandled_kind)
{
    const urdf::Vec3 local = c.origin.xyz;
    const urdf::Quat body_q = body.orientation;

    // world = body_pos + R(body_q) * local
    const double x = body_q.x, y = body_q.y, z = body_q.z, w = body_q.w;
    const double xx = x * x, yy = y * y, zz = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;
    out.position.x = body.position.x +
        (1 - 2 * (yy + zz)) * local.x + 2 * (xy - wz) * local.y + 2 * (xz + wy) * local.z;
    out.position.y = body.position.y +
        2 * (xy + wz) * local.x + (1 - 2 * (xx + zz)) * local.y + 2 * (yz - wx) * local.z;
    out.position.z = body.position.z +
        2 * (xz - wy) * local.x + 2 * (yz + wx) * local.y + (1 - 2 * (xx + yy)) * local.z;

    // ⚠ THE URDF ORIGIN'S rpy IS NOT OPTIONAL, and omitting it made this overlay lie.
    //
    // An earlier version dropped it "for primitives whose orientation the overlay does not need to
    // be exact about". That is false for the most important primitive in the scene: a wheel's
    // collision cylinder carries rpy = (pi/2, 0, 0) to lay its axis along the axle, so dropping it
    // draws every wheel rotated 90 degrees — as a flipper rather than a wheel. Reported on
    // 2026-08-27 as "Newton must be seeing flippers", which is exactly what a diagnostic that
    // misdraws orientation is for: producing confident wrong conclusions about the solver.
    //
    // world_q = body_q * rpy_q, fixed-axis roll/pitch/yaw as URDF defines it.
    const urdf::Vec3 rpy = c.origin.rpy;
    const double cr = std::cos(rpy.x * 0.5), sr = std::sin(rpy.x * 0.5);
    const double cp = std::cos(rpy.y * 0.5), sp = std::sin(rpy.y * 0.5);
    const double cy = std::cos(rpy.z * 0.5), sy = std::sin(rpy.z * 0.5);
    const urdf::Quat lq{sr * cp * cy - cr * sp * sy,
                        cr * sp * cy + sr * cp * sy,
                        cr * cp * sy - sr * sp * cy,
                        cr * cp * cy + sr * sp * sy};
    out.orientation = urdf::Quat{
        body_q.w * lq.x + body_q.x * lq.w + body_q.y * lq.z - body_q.z * lq.y,
        body_q.w * lq.y - body_q.x * lq.z + body_q.y * lq.w + body_q.z * lq.x,
        body_q.w * lq.z + body_q.x * lq.y - body_q.y * lq.x + body_q.z * lq.w,
        body_q.w * lq.w - body_q.x * lq.x - body_q.y * lq.y - body_q.z * lq.z};
    out.provenance = provenance;

    switch (c.geometry.type) {
    case urdf::GeometryType::Sphere:
        out.kind = urdf::CollisionShape::Kind::Sphere;
        out.radius = c.geometry.radius;
        return true;
    case urdf::GeometryType::Cylinder:
        out.kind = urdf::CollisionShape::Kind::Cylinder;
        out.radius = c.geometry.radius;
        out.half_length = 0.5 * c.geometry.length;
        return true;
    case urdf::GeometryType::Box:
        out.kind = urdf::CollisionShape::Kind::Box;
        out.half_extents = urdf::Vec3{0.5 * c.geometry.box_size.x, 0.5 * c.geometry.box_size.y,
                                      0.5 * c.geometry.box_size.z};
        return true;
    case urdf::GeometryType::Mesh:
    default:
        // ⚠ COUNTED, NOT DRAWN. Re-loading every collision mesh to draw it would put a file read on
        // the game thread; an overlay that quietly skipped them would be an overlay that is missing
        // exactly the geometry a wheel actually collides with, with nothing saying so.
        unhandled_kind = "mesh";
        return false;
    }
}

} // namespace

bool NewtonSidecarUrdfBackend::collisionDebugGeometry(const urdf::CollisionDebugFilter& filter,
                                                      urdf::CollisionDebugSnapshot& out) const
{
    out.backend = "newton-sidecar";
    out.solver_time = impl_->have_latest ? impl_->latest.time : 0.0;

    const bool have_radius = filter.radius > 0.0;
    auto within = [&](const urdf::Vec3& p) {
        if (!have_radius) return true;
        const double dx = p.x - filter.center.x, dy = p.y - filter.center.y;
        return (dx * dx + dy * dy) <= filter.radius * filter.radius;
    };
    std::string unhandled;
    bool hull_boxes = false;
    bool big_meshes = false;

    // ---- the robot, as NEWTON reports it ---------------------------------------------------
    //
    // ⚠ REALISED, and it is the only Realised thing this backend can offer: these poses came back
    // from the solver itself over the wire. If the overlay and the rendered robot disagree, the
    // render is wrong; if the overlay and the sand disagree, the sand is telling the truth.
    if (filter.include_robots && impl_->have_latest) {
        for (size_t l = 0; l < link_names_.size() && l < impl_->link_collisions.size(); ++l) {
            const urdf::LinkPose pose = getLinkPose(l);
            if (!within(pose.position)) continue;
            for (const urdf::Collision& c : impl_->link_collisions[l]) {
                if (out.geoms.size() >= filter.max_geoms) { ++out.omitted; continue; }
                urdf::CollisionShape g;
                if (overlayGeomFromCollision(c, pose, urdf::CollisionShape::Provenance::Realised,
                                             g, unhandled))
                    out.geoms.push_back(std::move(g));
                else
                    ++out.omitted;
            }
        }
    }

    // ---- mirrored level actors, as WE SENT THEM --------------------------------------------
    //
    // ⚠ SUBMITTED, deliberately marked as such. These are drawn where this side TOLD the sidecar to
    // put them, not where the sidecar has them — and that distinction is the whole diagnostic when
    // a robot reacts to an object the sand ignores. If the overlay sits on the Unreal actor, the
    // pose is leaving here correctly and the problem is downstream.
    if (filter.include_world) {
        for (size_t k = 0; k < impl_->kinematic.size() && k < impl_->kinematic_poses.size(); ++k) {
            const auto& kp = impl_->kinematic_poses[k];
            urdf::LinkPose pose;
            pose.position = kp.position;
            pose.orientation = kp.orientation;
            if (!within(pose.position)) continue;
            for (const urdf::StaticShape& ss : impl_->kinematic[k].shapes) {
                if (out.geoms.size() >= filter.max_geoms) { ++out.omitted; continue; }
                urdf::CollisionShape g;
                g.provenance = urdf::CollisionShape::Provenance::Submitted;
                g.position = pose.position;
                g.orientation = pose.orientation;
                switch (ss.kind) {
                case urdf::StaticShapeKind::Sphere:
                    g.kind = urdf::CollisionShape::Kind::Sphere;
                    g.radius = ss.radius;
                    break;
                case urdf::StaticShapeKind::Capsule:
                    g.kind = urdf::CollisionShape::Kind::Capsule;
                    g.radius = ss.radius;
                    g.half_length = 0.5 * std::sqrt(
                        (ss.center_b.x - ss.center_a.x) * (ss.center_b.x - ss.center_a.x) +
                        (ss.center_b.y - ss.center_a.y) * (ss.center_b.y - ss.center_a.y) +
                        (ss.center_b.z - ss.center_a.z) * (ss.center_b.z - ss.center_a.z));
                    break;
                default: {
                    // ⚠ REAL TRIANGLES WHEN WE HAVE THEM. A cooked mesh carries indices, so draw
                    // the actual surface — a cone drawn as its bounding cube reads as "Newton has
                    // a cube there", which is a claim about the solver that the overlay is not
                    // entitled to make. Reported as exactly that on 2026-08-27.
                    if (ss.indices.size() >= 3 && !ss.points.empty()) {
                        g.kind = urdf::CollisionShape::Kind::Mesh;
                        g.vertices = ss.points;
                        g.indices = ss.indices;
                        break;
                    }
                    // A hull arrives as a point cloud with no triangles, and triangulating it here
                    // would put a convex hull on the game thread every frame. The bounding box is
                    // the honest fallback — it answers "is the object where I think it is" — and it
                    // is COUNTED as an approximation rather than passed off as the shape.
                    double mx = 0, my = 0, mz = 0;
                    for (const urdf::Vec3& p : ss.points) {
                        mx = std::max(mx, std::abs(p.x));
                        my = std::max(my, std::abs(p.y));
                        mz = std::max(mz, std::abs(p.z));
                    }
                    if (mx <= 0 && my <= 0 && mz <= 0) { ++out.omitted; continue; }
                    g.kind = urdf::CollisionShape::Kind::Box;
                    g.half_extents = urdf::Vec3{mx, my, mz};
                    hull_boxes = true;
                    break;
                }
                }
                out.geoms.push_back(std::move(g));
            }
        }
    }

    // ---- the mirrored STATIC level, radius-filtered ----------------------------------------
    //
    // ⚠ THIS WAS EXCLUDED AND THAT WAS THE WRONG CALL. It was left out because it is 172 bodies and
    // 39 696 triangles on Blocks and "does not move" — but the level mirror includes every BLOCKING
    // component, and a blocking volume is invisible in the level by design. So the one class of
    // geometry an operator cannot see in Unreal was the one class this overlay refused to draw, and
    // the result was a robot colliding with thin air and no way to look at what it hit.
    //
    // The filter's radius is what makes it affordable: at the default 15 m only the handful of
    // bodies near the camera are emitted, and max_geoms still caps it.
    if (filter.include_world && impl_->static_world != nullptr) {
        for (const urdf::StaticBody& sb : impl_->static_world->bodies) {
            for (const urdf::StaticShape& ss : sb.shapes) {
                // ⚠ TEST THE SHAPE'S EXTENT, NOT THE BODY'S ORIGIN. An earlier version rejected a
                // body whose ORIGIN was outside the radius, which excluded everything on Blocks:
                // its ground is a single 40 km mesh whose origin is kilometres from the camera
                // while its triangles are under the wheels. The overlay drew nothing at all and
                // looked like the static world had not been mirrored.
                double bound = ss.radius;
                for (const urdf::Vec3& p : ss.points) {
                    const double d = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                    bound = std::max(bound, d);
                }
                if (have_radius) {
                    const double dx = sb.position.x - filter.center.x;
                    const double dy = sb.position.y - filter.center.y;
                    if (std::sqrt(dx * dx + dy * dy) - bound > filter.radius) continue;
                }
                // ⚠ AND SKIP THE ENORMOUS ONES. A 40 km ground mesh now PASSES the test above, and
                // drawing 39 696 triangles of it would bury the small blocking volumes this
                // overlay exists to reveal — and those are the only geometry that is invisible in
                // the level to begin with. Counted, so the omission is never silent.
                if (ss.indices.size() / 3 > 2000) {
                    ++out.omitted;
                    big_meshes = true;
                    continue;
                }
                if (out.geoms.size() >= filter.max_geoms) { ++out.omitted; continue; }
                urdf::CollisionShape g;
                g.provenance = urdf::CollisionShape::Provenance::Submitted;
                g.position = sb.position;
                g.orientation = sb.orientation;
                if (ss.kind == urdf::StaticShapeKind::Sphere) {
                    g.kind = urdf::CollisionShape::Kind::Sphere;
                    g.radius = ss.radius;
                }
                else if (ss.indices.size() >= 3 && !ss.points.empty()) {
                    g.kind = urdf::CollisionShape::Kind::Mesh;
                    g.vertices = ss.points;
                    g.indices = ss.indices;
                }
                else {
                    double mx = 0, my = 0, mz = 0;
                    for (const urdf::Vec3& p : ss.points) {
                        mx = std::max(mx, std::abs(p.x));
                        my = std::max(my, std::abs(p.y));
                        mz = std::max(mz, std::abs(p.z));
                    }
                    if (mx <= 0 && my <= 0 && mz <= 0) { ++out.omitted; continue; }
                    g.kind = urdf::CollisionShape::Kind::Box;
                    g.half_extents = urdf::Vec3{mx, my, mz};
                    hull_boxes = true;
                }
                out.geoms.push_back(std::move(g));
            }
        }
    }

    if (!unhandled.empty())
        out.omitted_kinds.push_back(unhandled);
    // ⚠ SAY WHERE THE PICTURE IS AN APPROXIMATION. A bounding box drawn for a hull is in the right
    // place and the wrong shape, and an operator reading it as the solver's actual geometry will
    // conclude the wrong thing about a cone.
    if (hull_boxes)
        out.omitted_kinds.push_back("hull (drawn as its bounding box, not its true shape)");
    if (big_meshes)
        out.omitted_kinds.push_back("static mesh >2000 tris (the level's ground; you can see it)");
    // ⚠ SAY WHAT IS NOT HERE. The mirrored STATIC level is deliberately absent: it is 172 bodies and
    // 39 696 triangles on Blocks, it does not move, and drawing it would bury the two things that
    // do. It is reported as an omitted kind so the overlay never implies the level was not sent.
    return true;
}

bool NewtonSidecarUrdfBackend::isConnected() const
{
    return impl_->have_latest;
}

double NewtonSidecarUrdfBackend::observedPublishInterval() const
{
    if (!impl_->have_previous || !impl_->have_latest)
        return 0.0;
    return impl_->latest.time - impl_->previous.time;
}

uint64_t NewtonSidecarUrdfBackend::extrapolatedSteps() const
{
    return impl_->extrapolated;
}

bool NewtonSidecarUrdfBackend::frameOffset(urdf::Vec3& out) const
{
    if (!impl_->offset_resolved)
        return false;
    out = impl_->frame_offset;
    return true;
}

double NewtonSidecarUrdfBackend::observedRate() const
{
    return impl_->rate_seeded ? impl_->observed_rate : 1.0;
}

} // namespace mpm
} // namespace airlib
} // namespace msr
