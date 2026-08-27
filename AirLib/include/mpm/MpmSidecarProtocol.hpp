// The wire contract between EXECOsim and the Newton MPM sidecar.
//
// ⚠ THIS IS A WIRE FORMAT. A separately-compiled Python process reads these bytes
// (tools/mpm_sidecar/protocol.py mirrors this file). Every struct here must stay trivially
// copyable and standard-layout, and ANY change to a field is a breaking change that must bump
// `kProtocolVersion` — a consumer that silently misreads a pose produces sand that deforms in the
// wrong place, which looks like a physics bug and is not one.
//
// ⚠ WHY OUT OF PROCESS AT ALL. Newton is 769 Python files and zero C or C++, with no headers, no
// shared library and no C entry points (urdf_physics/NEWTON-ASSESSMENT.md). It cannot be linked
// into the Unreal process. The process boundary is not a design preference, it is the only option.
//
// ⚠ WHY ACKNOWLEDGED, not latest-wins. `stream/SharedMemorySink.hpp` is deliberately latest-wins:
// for camera frames a stale frame is worse than a dropped one, and the writer must never block. The
// opposite is true here. Plan M2's exit criterion is that a timeout or epoch mismatch **pauses with
// a diagnostic** and that no stale collider state is used — so the sidecar must be able to say what
// it has consumed, and the sim must be able to notice it falling behind. Shared memory alone is not
// intrinsically lossless (plan §M2); the acknowledgement in MpmStatusBlock is what makes it so.
//
// ⚠ CADENCE MISMATCH IS THE NORMAL CASE, not an error. The coordinated tick is 3 ms; one MPM frame
// measured 16.67 ms of simulated time — about 5.6 ticks per MPM step (Newton granular example,
// 2026-08-25). The sidecar is EXPECTED to lag by a bounded number of ticks. What must never happen
// is lagging without anybody knowing, or the solver consuming collider state from an epoch that no
// longer exists.
//
// LAYOUT — two segments, split by lifetime, mirroring plan §M2's own division:
//
//   <dir>/execosim_mpm_registry   topology. Written at scenario build and at manifest change ONLY.
//                                 Collider ids, geometry, mass properties, coupling roles.
//   <dir>/execosim_mpm_state      per-step. Fixed-size pose/twist array indexed by registry order.
//   <dir>/execosim_mpm_status     the sidecar's acknowledgement and health, written by the sidecar.
//
// Splitting them is what lets "ordinary reset restores their state without changing topology"
// (plan §M2) be a property of the format rather than a convention: a reset rewrites `state` and
// leaves `registry` untouched, and a registry change is therefore always a real topology change.
#pragma once

#include <cstddef>
#include <cstdint>

namespace msr
{
namespace airlib
{
namespace mpm
{

/// Bump on ANY layout change. The sidecar refuses a version it was not built for rather than
/// reading a struct it does not understand.
/// ⚠ 3 since 2026-08-25: added MpmParticleBlock so Unreal can RENDER the sand. Until then the
/// particles existed only in the sidecar's GPU memory and the only way to see them was an offline
/// video, whose time axis is the sidecar's rather than the operator's.
///
/// ⚠ 2 since 2026-08-25: MpmRegistryBlock gained the TERRAIN REGION. Before that the simulator
/// declared a region in settings and the sidecar spawned sand wherever its own CLI defaults said,
/// with nothing carrying one to the other — the rover would drive through empty space while every
/// diagnostic reported healthy. The region is not optional metadata; it is where the sand IS.
constexpr uint32_t kProtocolVersion = 5;

constexpr uint32_t kRegistryMagic = 0x4D504D52u;  // 'MPMR'
constexpr uint32_t kStateMagic = 0x4D504D53u;     // 'MPMS'
constexpr uint32_t kStatusMagic = 0x4D504D48u;    // 'MPMH'
constexpr uint32_t kImpulseMagic = 0x4D504D49; // 'MPMI'
constexpr uint32_t kParticleMagic = 0x4D504D50u;  // 'MPMP'
constexpr uint32_t kVehicleCommandMagic = 0x4D504D43u;  // 'MPMC' - Unreal -> sidecar
constexpr uint32_t kVehiclePoseMagic = 0x4D504D56u;     // 'MPMV' - sidecar -> Unreal
constexpr uint32_t kStaticWorldMagic = 0x4D504D57u;     // 'MPMW' - level collision geometry

/// Hard ceilings, so both segments are fixed-size and can be mapped once.
///
/// ⚠ Exceeded is an ERROR, never a silent truncation. A robot whose 30th wheel quietly failed to
/// register would sink through sand that every counter reported as present.
constexpr uint32_t kMaxColliders = 256;
constexpr uint32_t kMaxShapesPerCollider = 16;
constexpr uint32_t kMaxColliderIdChars = 64;
constexpr uint32_t kMaxTerrainIdChars = 64;
/// Convex hull vertices carried per shape. A wheel or foot is far below this; anything above it is
/// a modelling choice that should be made deliberately rather than absorbed here.
constexpr uint32_t kMaxShapeVertices = 64;

/// ⚠ D15 LIMITS. Deliberately small: a sidecar-owned vehicle is one Newton model per process
/// and the SOLVE cost, not the wire, is what bounds how many can exist. Four vehicles at 64
/// links is ~42 KB of pose and ~20 KB of command, noise next to the 1.2 MB particle block.
constexpr uint32_t kMaxVehicles = 4;
constexpr uint32_t kMaxLinksPerVehicle = 64;
constexpr uint32_t kMaxJointsPerVehicle = 64;
constexpr uint32_t kMaxVehicleNameChars = 64;
constexpr uint32_t kMaxLinkNameChars = 64;

/// ⚠ THE LEVEL'S OWN COLLISION GEOMETRY, sized for a real map rather than for Blocks. Blocks is 172
/// bodies / 39 696 triangles; these caps are ~25x that and cost ~24 MB of tmpfs - RAM, but cheap RAM
/// next to the 1.2 MB particle block written every solve. Vertices are float32, unlike everything
/// else on this wire: a level triangle is a picture of a surface, not a pose, and doubling its
/// precision would double a 24 MB transfer to buy nothing a millimetre-scale sand grid can use.
constexpr uint32_t kMaxStaticBodies = 512;
constexpr uint32_t kMaxStaticShapes = 2048;
constexpr uint32_t kMaxStaticVertices = 1000000;
constexpr uint32_t kMaxStaticIndices = 3000000;
constexpr uint32_t kMaxStaticNameChars = 64;

/// ⚠ MIRRORED DYNAMIC ACTORS - level objects that MOVE. Few by nature (a level has hundreds of
/// static bodies and a handful of things that move), so 64 is generous. Their SHAPES ride the
/// static world block's pools; their POSES ride the command block, already written every tick.
constexpr uint32_t kMaxKinematicBodies = 64;

/// Mirrors urdf::CollisionShape::Kind. Duplicated deliberately: this is a wire enum whose numeric
/// values are frozen by `kProtocolVersion`, and inheriting them from a header that is free to be
/// reordered would make an internal refactor silently reinterpret the wire.
enum class WireShapeKind : uint32_t {
    Sphere = 0,
    Capsule = 1,
    Cylinder = 2,
    Box = 3,
    ConvexHull = 4,
    Plane = 5
};

/// Mirrors urdf::CouplingRole, frozen for the same reason.
enum class WireCouplingRole : uint32_t {
    Static = 0,
    KinematicOneWay = 1,
    DynamicTwoWay = 2
};

/// ⚠ Frame, everywhere in this file: SOLVER frame — URDF/ROS, right-handed, Z-up, metres, origin at
/// the Unreal world origin. Invariant 0 (plan §5): COM and inertia are BODY-LOCAL; pose, twist and
/// any returned impulse are WORLD. Newton shares this convention, so no conversion happens at this
/// boundary and none should be added — a mirror introduced here would be invisible until the sand
/// deformed on the wrong side of the wheel.
struct WireVec3 {
    double x = 0, y = 0, z = 0;
};

/// x, y, z, w — the same order as urdf::Quat. Newton/warp use w-first; the sidecar converts, and
/// that conversion lives in exactly one place (protocol.py) rather than at each use.
struct WireQuat {
    double x = 0, y = 0, z = 0, w = 1;
};

struct WireShape {
    uint32_t kind = 0;          ///< WireShapeKind
    uint32_t vertex_count = 0;  ///< ConvexHull only

    /// Shape frame within its collider's body frame.
    WireVec3 position;
    WireQuat orientation;

    double radius = 0;       ///< Sphere, Capsule, Cylinder
    double half_length = 0;  ///< Capsule, Cylinder
    WireVec3 half_extents;   ///< Box

    WireVec3 vertices[kMaxShapeVertices];
};

/// One collider's TOPOLOGY. Written at registration; never per step.
struct WireColliderRegistration {
    char stable_id[kMaxColliderIdChars] = {};

    uint32_t shape_count = 0;
    uint32_t role = 0;  ///< WireCouplingRole

    double mass = 0;
    WireVec3 com_local;
    double inertia_local[9] = {};

    /// ⚠ MUST be honoured by the sidecar, not merely logged. False means the inertia above is the
    /// isolated rigid-body tensor, NOT the articulated operational-space effective inertia (plan
    /// §11.1). A sidecar that computed a two-way reaction from it would be producing sinkage and
    /// traction numbers that are wrong in a way no log line shows.
    uint32_t inertia_is_articulated_effective = 0;

    double friction = 0;
    double restitution = 0;
    uint32_t material_reported = 0;

    WireShape shapes[kMaxShapesPerCollider];
};

/// One collider's per-step STATE. Indexed by position in the registry.
struct WireColliderState {
    WireVec3 position;
    WireQuat orientation;
    WireVec3 linear_velocity;
    WireVec3 angular_velocity;
};

/// Identity of the world this data belongs to.
///
/// ⚠ CARRIED IN EVERY BLOCK, and compared on every read. A sidecar that kept simulating across a
/// global reset would be pushing sand with colliders from a run that no longer exists — plan §M2
/// requires an epoch mismatch to pause with a diagnostic, and this is the field that detects it.
struct WireWorldStamp {
    uint64_t world_id = 0;
    uint64_t world_revision = 0;
    uint64_t manifest_revision = 0;
    uint64_t reset_epoch = 0;
};

/// Where the deformable terrain IS, and how much of it there is.
///
/// ⚠ SOLVER FRAME, already converted. The operator declares the region in global NED in settings —
/// consistent with every vehicle position — and the simulator converts it here through the one
/// UrdfTransform conversion the rest of the URDF path uses. Putting NED on the wire would make the
/// sidecar, which is otherwise entirely frame-agnostic, need to know about AirSim's conventions;
/// and a Y-mirror applied on the wrong side of this boundary has already cost this workstream one
/// full debugging cycle (the height field sampled 120 m from where it was reported).
struct WireTerrainRegion {
    char terrain_id[kMaxTerrainIdChars] = {};

    /// Centre of the sand patch, solver frame.
    WireVec3 center;
    /// Half-extents in x, y and z. `center.z - half_extent.z` is where the sand rests, and is
    /// where the sidecar puts its ground plane unless told otherwise.
    WireVec3 half_extent;

    /// The operator's declared cadence ratio, for the sidecar to report against rather than to
    /// obey — the sidecar's own solve rate is whatever the GPU gives it.
    uint32_t rigid_ticks_per_mpm_step = 0;
    /// 0 when the simulator declared no terrain; the sidecar must then fall back to its CLI and
    /// SAY SO, because silently inventing a patch is how the two ends stop agreeing.
    uint32_t valid = 0;
};

/// What the sand did back to one collider over a single MPM frame.
///
/// ⚠ IMPULSE, NOT FORCE. Newton reports per-grid-node impulses (N.s) accumulated across its own
/// step; the sidecar reduces them per collider and sends them as impulses because that is what they
/// are. The simulator converts to whatever its rigid seam accepts — both backends take newtons —
/// and the conversion is a decision recorded in the plan (M3), not an implementation detail:
/// dividing by the wrong dt silently scales every force the sand applies.
///
/// ⚠ `angular` is about the collider's CURRENT centre of mass, matching Newton's own
/// `compute_body_forces` kernel (`r = impulse_pos - transform_point(X_wb, X_com)`). A consumer that
/// applies `linear` at a different point and adds `angular` would double-count the moment arm.
struct WireColliderImpulse {
    WireVec3 linear;   ///< N.s, world axes
    WireVec3 angular;  ///< N.m.s about the collider's centre of mass, world axes
    /// Grid nodes that contributed. 0 means the sand never touched this collider this frame, which
    /// is different from an impulse that happened to sum to zero.
    uint32_t contact_nodes = 0;
    uint32_t reserved = 0;
};

struct MpmImpulseBlock {
    uint32_t magic = kImpulseMagic;
    uint32_t version = kProtocolVersion;
    uint32_t sequence = 0;
    uint32_t collider_count = 0;

    WireWorldStamp stamp;

    uint64_t sidecar_step = 0;
    double sidecar_time = 0;
    /// Simulated seconds these impulses were accumulated over. The consumer needs it to convert to
    /// a force, and must NOT assume it equals its own tick.
    double mpm_dt = 0;

    WireColliderImpulse colliders[kMaxColliders];
};

struct MpmRegistryBlock {
    uint32_t magic = kRegistryMagic;
    uint32_t version = kProtocolVersion;
    /// Seqlock: odd while the writer is mid-update. Same discipline as ShmHeader.
    uint32_t sequence = 0;
    uint32_t collider_count = 0;

    WireWorldStamp stamp;

    /// Solver step the sim will advance by, in seconds. The sidecar does NOT have to match it —
    /// the cadences legitimately differ — but it must know it to interpret the twists.
    double sim_fixed_dt = 0;

    WireTerrainRegion region;

    WireColliderRegistration colliders[kMaxColliders];
};

struct MpmStateBlock {
    uint32_t magic = kStateMagic;
    uint32_t version = kProtocolVersion;
    uint32_t sequence = 0;
    uint32_t collider_count = 0;

    WireWorldStamp stamp;

    /// The sim's own step counter and clock at the moment this was written. The sidecar echoes
    /// `step` back in MpmStatusBlock::acknowledged_step, and the difference is the lag.
    uint64_t step = 0;
    double simulation_time = 0;

    WireColliderState colliders[kMaxColliders];
};

/// Written by the SIDECAR, read by the sim. The other half of the acknowledgement.
struct MpmStatusBlock {
    uint32_t magic = kStatusMagic;
    uint32_t version = kProtocolVersion;
    uint32_t sequence = 0;

    /// 0 healthy; non-zero is a sidecar-defined fault, with `message` explaining it.
    uint32_t fault = 0;

    WireWorldStamp stamp;

    /// The highest `MpmStateBlock::step` the sidecar has actually consumed. THE acknowledgement.
    uint64_t acknowledged_step = 0;
    /// The sidecar's own solver step count and simulated time — deliberately separate, because
    /// they advance at a different cadence and conflating them would hide exactly that.
    uint64_t sidecar_step = 0;
    double sidecar_time = 0;

    /// Wall-clock seconds the sidecar spent in its last solve. The GPU budget, observed rather
    /// than assumed.
    double last_solve_seconds = 0;

    uint64_t particle_count = 0;

    char message[256] = {};
};

/// Particles the sidecar offers Unreal for RENDERING ONLY.
///
/// ⚠ RENDERING ONLY, and nothing may branch on this. It is a decimated, lagged copy of state that
/// lives on another process's GPU; treating it as physics would be reading the sand's shadow.
///
/// ⚠ LATEST-WINS, which is the OPPOSITE of the collider path and deliberately so. Colliders are
/// acknowledged because a dropped pose means the sand was pushed by a robot that was somewhere
/// else. For rendering the reverse holds: a dropped frame is invisible, a stale frame is a visible
/// lie, and a writer that waited for the renderer would stall the solver. Same reasoning as
/// stream/SharedMemorySink.hpp, reached from the other direction.
///
/// ⚠ float32, not double. This is pixels, not physics: halving the payload matters more than the
/// last seven digits, and 100 k particles is already 1.2 MB per frame.
constexpr uint32_t kMaxRenderParticles = 100000;

struct MpmParticleBlock {
    uint32_t magic = kParticleMagic;
    uint32_t version = kProtocolVersion;
    uint32_t sequence = 0;
    /// Entries actually written into `positions`.
    uint32_t particle_count = 0;

    WireWorldStamp stamp;

    uint64_t sidecar_step = 0;
    double sidecar_time = 0;

    /// How many particles the solver ACTUALLY has, before decimation.
    ///
    /// ⚠ Carried so a viewer can say "showing 50,000 of 306,456" rather than quietly implying the
    /// sand is sparser than it is. A decimated render that looks like the whole field is the same
    /// class of lie as a counter reporting success about the wrong place.
    uint64_t total_particles = 0;

    /// Representative particle radius in metres, for sizing whatever the renderer draws.
    float radius = 0.01f;

    /// Solver frame, x,y,z triples. Only the first `particle_count * 3` are meaningful.
    float positions[kMaxRenderParticles * 3];
};

/// Default segment names. A directory is a parameter, exactly as in SharedMemorySink — /dev/shm is
/// only the default, and a container without the host's shm needs a bind-mounted path instead.

/// WireJointTargetMode - numeric values frozen by kProtocolVersion.
/// ⚠ NONE IS NOT "ZERO". A joint with mode None is left alone by the sidecar, which is what a
/// passive rocker-bogie needs; a joint commanded to velocity 0 is actively HELD at zero, which is
/// a brake. Conflating the two locks the suspension of any vehicle whose settings omit a joint.
enum class WireJointTargetMode : uint32_t {
    None = 0,
    Position = 1,
    Velocity = 2,
    Torque = 3,
};

/// A level object whose pose is pushed in every tick rather than frozen at load.
///
/// ⚠ ONE-DIRECTIONAL BY CONSTRUCTION, and this is the honest limit rather than an oversight. The
/// simulator dictates the pose, so the robot and the sand are pushed BY these bodies and never push
/// back. A crate dropped into the bed will plough it and displace it, and will fall through it as
/// though the sand were not there, because the thing deciding where the crate goes is Unreal's own
/// physics and it has never heard of the sand. Making the crate float needs the crate solved in
/// Newton - the same argument as D15 itself, one object further out.
///
/// ⚠ NEVER a mesh: urdf::KinematicBody's shapes are primitives only.
struct WireKinematicBody {
    char name[kMaxStaticNameChars] = {};
    uint32_t shape_start = 0;
    uint32_t shape_count = 0;
    double friction = 0.7;
    double restitution = 0;
};

/// ⚠ POSITIONALLY MATCHED to the kinematic array in the static world block, not keyed by name. A
/// name lookup per body per tick is wasted work on a hot path, and the registration is revisioned -
/// so the contract is that pose[i] belongs to kinematic[i] of the SAME revision, and a consumer
/// must ignore poses whose revision it has not built.
struct WireKinematicPose {
    WireVec3 position;
    WireQuat orientation;
};

struct WireJointCommand {
    char joint_name[kMaxLinkNameChars] = {};
    uint32_t target_mode = 0;   ///< WireJointTargetMode
    uint32_t _pad = 0;
    double target = 0;
};

struct WireVehicleCommand {
    char vehicle_name[kMaxVehicleNameChars] = {};
    uint32_t joint_count = 0;
    uint32_t _pad = 0;
    WireJointCommand joints[kMaxJointsPerVehicle];
};

/// Unreal -> sidecar. Latest-wins.
///
/// ⚠ LATEST-WINS, AND THAT IS THE RATE-DECOUPLING DECISION MADE CONCRETE. The sidecar consumes
/// whatever command is current when it starts a solve and never waits for a newer one. A dropped
/// command is a target that was superseded before it could be applied, which is exactly what
/// should happen to it - the alternative is the simulator blocking on the sand solver, which is
/// what plan D15 calls "who owns time" and what PX4 lockstep forbids outright.
struct MpmVehicleCommandBlock {
    uint32_t magic = kVehicleCommandMagic;
    uint32_t version = kProtocolVersion;
    uint32_t sequence = 0;      ///< seqlock; ODD while the writer is inside an update
    uint32_t vehicle_count = 0;
    WireWorldStamp stamp;
    uint64_t step = 0;
    double simulation_time = 0;
    /// ⚠ THE ONLY THING THAT CAN ACTUALLY RESET A SIDECAR-OWNED VEHICLE. The simulator resetting
    /// its own world does nothing to a robot solved in another process: its Newton model keeps the
    /// vehicle where it drove to and the sand keeps every rut. Bumping this is what tells the
    /// sidecar to rebuild - fresh bed, vehicle back at its spawn.
    uint64_t reset_epoch = 0;
    /// ⚠ THE MOVING HALF OF THE LEVEL MIRROR, on the block already written every tick.
    /// `kinematic_revision` says which registration these poses belong to; a consumer that has
    /// built a different revision must ignore them rather than apply pose[i] to the wrong body.
    uint32_t kinematic_revision = 0;
    uint32_t kinematic_count = 0;
    WireKinematicPose kinematic_poses[kMaxKinematicBodies];
    WireVehicleCommand vehicles[kMaxVehicles];
};

/// ⚠ VELOCITIES ARE CARRIED ON PURPOSE, and they are what make rate decoupling work.
///
/// The sidecar publishes at 20-30 Hz and Unreal renders at 60. Interpolating position alone
/// between two 33 ms-apart samples gives visible corner-cutting on a turning wheel; position plus
/// twist gives a Hermite the consumer can evaluate at any instant, and extrapolation for frames
/// past the newest sample. This is also why the pose path can afford to lag at all: a one-step-old
/// POSE is a small position error, whereas a one-step-old FORCE applied to a light wheel is the
/// instability this workstream spent 2026-08-26 chasing.
struct WireLinkPose {
    char link_name[kMaxLinkNameChars] = {};
    WireVec3 position;
    WireQuat orientation;
    WireVec3 linear_velocity;
    WireVec3 angular_velocity;
};

/// ⚠ `effort` IS NOT REPORTED YET, and `effort_reported` on the enclosing WireVehiclePose says so
/// rather than leaving a consumer to trust a zero. Position and velocity come straight out of
/// Newton's joint_q / joint_qd and are exact; applied joint torque needs the solver's own per-joint
/// force array, which the coupled proxy does not expose today. A silent 0.0 would be
/// indistinguishable from a motor doing nothing - exactly the shape of bug that cost this
/// workstream a day, when a published velocity turned out to be the wrong half of a twist and read
/// as a stationary vehicle.
struct WireJointState {
    char joint_name[kMaxLinkNameChars] = {};
    double position = 0;   ///< rad or m
    double velocity = 0;   ///< rad/s or m/s
    double effort = 0;     ///< N.m or N - ONLY meaningful when effort_reported is non-zero
};

struct WireVehiclePose {
    char vehicle_name[kMaxVehicleNameChars] = {};
    uint32_t link_count = 0;
    uint32_t joint_count = 0;
    uint32_t effort_reported = 0;   ///< 0 = every WireJointState::effort below is a placeholder
    uint32_t _pad = 0;
    /// ⚠ WHERE THE ROOT LINK WAS BUILT, constant for the life of a model. Anchor on THIS, never on
    /// a sampled pose. The sidecar free-runs, so between this side building and reading its first
    /// pose the robot has settled or rolled a little, and an offset derived from that sample
    /// silently absorbs it — the symptom being a vehicle that reappears a few centimetres away
    /// every reset, for no reason the operator can see.
    WireVec3 spawn_position;
    WireLinkPose links[kMaxLinksPerVehicle];
    WireJointState joints[kMaxJointsPerVehicle];
};

/// Sidecar -> Unreal. Latest-wins, never blocking.
///
/// ⚠ `sidecar_time` IS THE TIMESTAMP THE INTERPOLATION IS ANCHORED TO, and it is the sidecar's
/// SIMULATED time, not wall time. A consumer that interpolated against its own clock would drift
/// whenever the solve ran slower than real time - which it does, routinely, at 1.37 M particles.
///
/// ⚠ `publish_interval_seconds` is the sidecar's NOMINAL cadence, published so the consumer can
/// size its interpolation window without measuring it. It is advisory: the honest instantaneous
/// interval is the difference of two consecutive `sidecar_time` values, and a consumer that needs
/// to notice a stall must use that rather than trust this.
struct MpmVehiclePoseBlock {
    uint32_t magic = kVehiclePoseMagic;
    uint32_t version = kProtocolVersion;
    uint32_t sequence = 0;
    uint32_t vehicle_count = 0;
    WireWorldStamp stamp;
    uint64_t sidecar_step = 0;
    double sidecar_time = 0;
    double publish_interval_seconds = 0;
    uint64_t acknowledged_command_step = 0;
    WireVehiclePose vehicles[kMaxVehicles];
};


/// Mirrors urdf::StaticShapeKind. Values frozen by kProtocolVersion.
enum class WireStaticShapeKind : uint32_t { Hull = 0, Mesh = 1, Sphere = 2, Capsule = 3 };

/// ⚠ VERTICES AND INDICES LIVE IN SHARED POOLS on the block, not in this struct. A per-shape array
/// would have to be sized for the worst shape and multiplied by kMaxStaticShapes, which for a level
/// mesh is absurd; the start/count pairs index the flat arrays instead.
struct WireStaticShape {
    uint32_t kind = 0;
    uint32_t vertex_start = 0;
    uint32_t vertex_count = 0;
    uint32_t index_start = 0;
    uint32_t index_count = 0;
    uint32_t _pad = 0;
    WireVec3 center_a;      ///< Sphere: centre. Capsule: first hemisphere centre.
    WireVec3 center_b;      ///< Capsule: second hemisphere centre.
    double radius = 0;
};

/// ⚠ BODY-LOCAL SHAPES PLUS A WORLD TRANSFORM, matching urdf::StaticBody rather than flattening to
/// world space. That is what lets one cooked mesh be reused by every instance of the same asset
/// placed in a level - the difference between cooking a map once and once per placed rock.
struct WireStaticBody {
    char name[kMaxStaticNameChars] = {};
    WireVec3 position;
    WireQuat orientation;
    uint32_t shape_start = 0;
    uint32_t shape_count = 0;
    double friction = 0.7;
    double restitution = 0;
};

/// Unreal -> sidecar. The level's collision geometry, published ONCE per revision.
///
/// ⚠ WHY THIS EXISTS. Without it the sidecar's robot stands on a flat plane at a height somebody
/// typed in, and collides with nothing else in the level at all. Getting that height right by hand
/// failed twice in one afternoon: the two ends disagree about z by the AirSim NED origin's offset
/// from the Unreal world origin (0.640 m on Blocks), so a plausible number can be wrong by exactly
/// that and look like a physics bug. A level whose ground is not flat cannot be expressed as a
/// number at all, and a level with obstacles never could.
///
/// ⚠ PUBLISHED ON A REVISION, NOT PER TICK. It is 24 MB and static by construction; the sidecar
/// rebuilds its collision only when `revision` changes.
///
/// ⚠ `frame_offset` TRAVELS WITH THE GEOMETRY. The vertices are in the SOLVER frame (origin: the
/// Unreal world origin) and the sidecar works in its own frame, offset from it. Sending the vector
/// alongside the data means the sidecar subtracts it itself, rather than the two ends separately
/// agreeing on a convention that has already been got wrong once.
struct MpmStaticWorldBlock {
    uint32_t magic = kStaticWorldMagic;
    uint32_t version = kProtocolVersion;
    uint32_t sequence = 0;
    uint32_t revision = 0;
    uint32_t body_count = 0;
    uint32_t shape_count = 0;
    uint32_t vertex_count = 0;
    uint32_t index_count = 0;
    /// ⚠ Non-zero means the level did NOT fit and what follows is a PARTIAL world. A robot that
    /// falls through a floor which was silently dropped is the worst outcome here, so both ends
    /// report this rather than letting it look like geometry that was never there.
    uint32_t truncated = 0;
    uint32_t _pad = 0;
    WireVec3 frame_offset;
    uint32_t kinematic_count = 0;
    uint32_t _pad2 = 0;
    WireKinematicBody kinematic[kMaxKinematicBodies];
    WireStaticBody bodies[kMaxStaticBodies];
    WireStaticShape shapes[kMaxStaticShapes];
    float vertices[kMaxStaticVertices * 3];
    int32_t indices[kMaxStaticIndices];
};

constexpr const char* kRegistrySegment = "execosim_mpm_registry";
constexpr const char* kStateSegment = "execosim_mpm_state";
constexpr const char* kStatusSegment = "execosim_mpm_status";
constexpr const char* kImpulseSegment = "execosim_mpm_impulse";
constexpr const char* kParticleSegment = "execosim_mpm_particles";
constexpr const char* kVehicleCommandSegment = "execosim_mpm_vehicle_command";
constexpr const char* kVehiclePoseSegment = "execosim_mpm_vehicle_pose";
constexpr const char* kStaticWorldSegment = "execosim_mpm_static_world";

static_assert(sizeof(WireVec3) == 24, "WireVec3 must be 3 packed doubles");
static_assert(sizeof(WireQuat) == 32, "WireQuat must be 4 packed doubles");
static_assert(sizeof(WireStaticShape) == 80, "WireStaticShape layout changed");
static_assert(sizeof(WireStaticBody) == 144, "WireStaticBody layout changed");
static_assert(sizeof(MpmStaticWorldBlock) == 24243272, "MpmStaticWorldBlock layout changed");
static_assert(sizeof(WireJointCommand) == 80, "WireJointCommand layout changed");
static_assert(sizeof(WireVehicleCommand) == 5192, "WireVehicleCommand layout changed");
static_assert(sizeof(MpmVehicleCommandBlock) == 24432, "MpmVehicleCommandBlock layout changed");
static_assert(sizeof(WireLinkPose) == 168, "WireLinkPose layout changed");
static_assert(sizeof(WireJointState) == 88, "WireJointState layout changed");
static_assert(sizeof(WireVehiclePose) == 16488, "WireVehiclePose layout changed");
static_assert(sizeof(MpmVehiclePoseBlock) == 66032, "MpmVehiclePoseBlock layout changed");


} // namespace mpm
} // namespace airlib
} // namespace msr
