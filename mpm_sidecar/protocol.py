"""Python mirror of AirLib/include/mpm/MpmSidecarProtocol.hpp.

⚠ THIS FILE IS HALF OF A WIRE FORMAT. The other half is the C++ header, and the two are only
correct together. `ctypes.Structure` is used rather than `struct.unpack` format strings on purpose:
ctypes computes offsets from the field declarations the same way the C++ compiler does, so a field
inserted in the middle of one side and not the other is caught by the size assertions at import
rather than by sand deforming in the wrong place.

⚠ IF YOU CHANGE ANYTHING HERE, change the C++ header and bump `PROTOCOL_VERSION` in both. A
consumer that silently misreads a pose produces a physics bug that is not a physics bug.

Run this module directly to print the struct sizes; they must match the C++ ones exactly:
    python mpm_sidecar/protocol.py
"""

from __future__ import annotations

import ctypes
import gc
import mmap
import os
import time

# ⚠ 5 since 2026-08-26: added MpmVehicleCommandBlock and MpmVehiclePoseBlock — plan D15, the
# VEHICLE moving into the sidecar. Unreal sends joint targets, the sidecar sends back link poses.
# Existing block LAYOUTS are untouched by this bump, so v3/v4 recordings still parse.
#
# ⚠ 4 since 2026-08-26: added MpmImpulseBlock — the sand's reaction back onto the colliders. The
# registry and state LAYOUTS are unchanged by this bump, which is what makes recordings written at
# version 3 still readable; see `validate(..., accept_older=True)`.
# ⚠ 3 since 2026-08-25: added MpmParticleBlock so Unreal can render the sand.
# ⚠ 2 since 2026-08-25: the registry gained the terrain REGION. Before that the simulator declared
# a region in settings and this sidecar spawned sand wherever its CLI defaults said, with nothing
# carrying one to the other.
PROTOCOL_VERSION = 6

REGISTRY_MAGIC = 0x4D504D52  # 'MPMR'
STATE_MAGIC = 0x4D504D53     # 'MPMS'
STATUS_MAGIC = 0x4D504D48    # 'MPMH'
IMPULSE_MAGIC = 0x4D504D49   # 'MPMI'
PARTICLE_MAGIC = 0x4D504D50  # 'MPMP'
VEHICLE_COMMAND_MAGIC = 0x4D504D43  # 'MPMC' — Unreal -> sidecar joint targets
VEHICLE_POSE_MAGIC = 0x4D504D56     # 'MPMV' — sidecar -> Unreal link poses
STATIC_WORLD_MAGIC = 0x4D504D57     # 'MPMW' — Unreal -> sidecar level collision geometry

MAX_COLLIDERS = 256
MAX_SHAPES_PER_COLLIDER = 16
MAX_COLLIDER_ID_CHARS = 64
MAX_TERRAIN_ID_CHARS = 64
MAX_RENDER_PARTICLES = 100000
MAX_SHAPE_VERTICES = 64

# ⚠ D15 LIMITS. Deliberately small: a sidecar-owned vehicle is one Newton model per process and the
# solve cost, not the wire, is what bounds how many can exist. Four vehicles at 64 links is ~30 KB
# of pose and ~20 KB of command, which is noise next to the 1.2 MB particle block.
MAX_VEHICLES = 4
MAX_LINKS_PER_VEHICLE = 64
MAX_JOINTS_PER_VEHICLE = 64
MAX_VEHICLE_NAME_CHARS = 64
MAX_LINK_NAME_CHARS = 64

# ⚠ THE LEVEL'S OWN COLLISION GEOMETRY, sized for a real map rather than for Blocks. Blocks is 172
# bodies / 39 696 triangles; the caps below are ~25x that and cost ~24 MB of tmpfs, which is RAM but
# cheap RAM next to the 1.2 MB particle block being written every solve. Vertices are float32 here,
# unlike everything else on this wire: a level triangle is a picture of a surface, not a pose, and
# doubling its precision would double a 24 MB transfer to buy nothing a millimetre-scale sand grid
# can use.
MAX_STATIC_BODIES = 512
MAX_STATIC_SHAPES = 2048
MAX_STATIC_VERTICES = 1000000
MAX_STATIC_INDICES = 3000000
MAX_STATIC_NAME_CHARS = 64

# ⚠ MIRRORED DYNAMIC ACTORS — level objects that MOVE. Few by nature (a level has hundreds of
# static bodies and a handful of things that move), so 64 is generous. Their SHAPES ride the static
# world block's pools, because both are known at registration; their POSES ride the command block,
# which is already written every tick.
MAX_KINEMATIC_BODIES = 64

# WireStaticShapeKind — mirrors urdf::StaticShapeKind, values frozen by PROTOCOL_VERSION.
STATIC_SHAPE_HULL = 0
STATIC_SHAPE_MESH = 1
STATIC_SHAPE_SPHERE = 2
STATIC_SHAPE_CAPSULE = 3

REGISTRY_SEGMENT = "execosim_mpm_registry"
STATE_SEGMENT = "execosim_mpm_state"
STATUS_SEGMENT = "execosim_mpm_status"
IMPULSE_SEGMENT = "execosim_mpm_impulse"
PARTICLE_SEGMENT = "execosim_mpm_particles"
VEHICLE_COMMAND_SEGMENT = "execosim_mpm_vehicle_command"
VEHICLE_POSE_SEGMENT = "execosim_mpm_vehicle_pose"
STATIC_WORLD_SEGMENT = "execosim_mpm_static_world"

# WireShapeKind — numeric values frozen by PROTOCOL_VERSION.
SHAPE_SPHERE = 0
SHAPE_CAPSULE = 1
SHAPE_CYLINDER = 2
SHAPE_BOX = 3
SHAPE_CONVEX_HULL = 4
SHAPE_PLANE = 5

# WireJointTargetMode — numeric values frozen by PROTOCOL_VERSION.
# ⚠ NONE IS NOT "ZERO". A joint with mode NONE is left alone by the sidecar, which is what a
# passive rocker-bogie needs; a joint commanded to velocity 0 is actively held at zero, which is a
# brake. Conflating the two locks the suspension of any vehicle whose settings omit a joint.
JOINT_TARGET_NONE = 0
JOINT_TARGET_POSITION = 1
JOINT_TARGET_VELOCITY = 2
JOINT_TARGET_TORQUE = 3

# WireCouplingRole
ROLE_STATIC = 0
ROLE_KINEMATIC_ONE_WAY = 1
ROLE_DYNAMIC_TWO_WAY = 2


class WireVec3(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double), ("z", ctypes.c_double)]

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


class WireQuat(ctypes.Structure):
    """x, y, z, w — matching urdf::Quat.

    ⚠ Newton and warp use w-FIRST (`wp.quat` is x,y,z,w but USD/`wp.quat_from_matrix` conventions
    vary, and newton's body_q is [pos, quat] with quat as x,y,z,w). The conversion for whatever the
    consumer needs belongs in one place — `as_wxyz()` here — not scattered at each use.
    """

    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double),
                ("z", ctypes.c_double), ("w", ctypes.c_double)]

    def as_xyzw(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.w)

    def as_wxyz(self) -> tuple[float, float, float, float]:
        return (self.w, self.x, self.y, self.z)


class WireShape(ctypes.Structure):
    _fields_ = [
        ("kind", ctypes.c_uint32),
        ("vertex_count", ctypes.c_uint32),
        ("position", WireVec3),
        ("orientation", WireQuat),
        ("radius", ctypes.c_double),
        ("half_length", ctypes.c_double),
        ("half_extents", WireVec3),
        ("vertices", WireVec3 * MAX_SHAPE_VERTICES),
    ]

    def hull_vertices(self) -> list[tuple[float, float, float]]:
        return [self.vertices[i].as_tuple() for i in range(self.vertex_count)]


class WireColliderRegistration(ctypes.Structure):
    _fields_ = [
        ("stable_id", ctypes.c_char * MAX_COLLIDER_ID_CHARS),
        ("shape_count", ctypes.c_uint32),
        ("role", ctypes.c_uint32),
        ("mass", ctypes.c_double),
        ("com_local", WireVec3),
        ("inertia_local", ctypes.c_double * 9),
        ("inertia_is_articulated_effective", ctypes.c_uint32),
        ("friction", ctypes.c_double),
        ("restitution", ctypes.c_double),
        ("material_reported", ctypes.c_uint32),
        ("shapes", WireShape * MAX_SHAPES_PER_COLLIDER),
    ]

    def name(self) -> str:
        return self.stable_id.decode("utf-8", "replace")


class WireColliderState(ctypes.Structure):
    _fields_ = [
        ("position", WireVec3),
        ("orientation", WireQuat),
        ("linear_velocity", WireVec3),
        ("angular_velocity", WireVec3),
    ]


class WireWorldStamp(ctypes.Structure):
    _fields_ = [
        ("world_id", ctypes.c_uint64),
        ("world_revision", ctypes.c_uint64),
        ("manifest_revision", ctypes.c_uint64),
        ("reset_epoch", ctypes.c_uint64),
    ]

    def key(self) -> tuple[int, int, int, int]:
        return (self.world_id, self.world_revision, self.manifest_revision, self.reset_epoch)

    def __str__(self) -> str:
        return (f"world {self.world_id}/{self.world_revision} "
                f"manifest {self.manifest_revision} epoch {self.reset_epoch}")


class WireTerrainRegion(ctypes.Structure):
    """Where the sand is, in SOLVER frame — already converted from the operator's NED declaration.

    ⚠ `valid == 0` means the simulator declared no terrain. The sidecar must then fall back to its
    own CLI and say so out loud; silently inventing a patch is how the two ends stop agreeing about
    where the sand is while both report healthy.
    """

    _fields_ = [
        ("terrain_id", ctypes.c_char * MAX_TERRAIN_ID_CHARS),
        ("center", WireVec3),
        ("half_extent", WireVec3),
        ("rigid_ticks_per_mpm_step", ctypes.c_uint32),
        ("valid", ctypes.c_uint32),
    ]

    def name(self) -> str:
        return self.terrain_id.decode("utf-8", "replace")

    def bounds(self):
        """(lo, hi) corners of the patch, solver frame."""
        c, h = self.center, self.half_extent
        return ((c.x - h.x, c.y - h.y, c.z - h.z), (c.x + h.x, c.y + h.y, c.z + h.z))


class WireColliderImpulse(ctypes.Structure):
    """What the sand did back to one collider over a single MPM frame.

    ⚠ IMPULSE (N.s), not force. `angular` is about the collider's CURRENT centre of mass, matching
    Newton's own `compute_body_forces` kernel; applying `linear` elsewhere and adding `angular`
    would double-count the moment arm.
    """

    _fields_ = [
        ("linear", WireVec3),
        ("angular", WireVec3),
        ("contact_nodes", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
    ]


class MpmImpulseBlock(ctypes.Structure):
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("collider_count", ctypes.c_uint32),
        ("stamp", WireWorldStamp),
        ("sidecar_step", ctypes.c_uint64),
        ("sidecar_time", ctypes.c_double),
        ("mpm_dt", ctypes.c_double),
        ("colliders", WireColliderImpulse * MAX_COLLIDERS),
    ]


class MpmRegistryBlock(ctypes.Structure):
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("collider_count", ctypes.c_uint32),
        ("stamp", WireWorldStamp),
        ("sim_fixed_dt", ctypes.c_double),
        ("region", WireTerrainRegion),
        ("colliders", WireColliderRegistration * MAX_COLLIDERS),
    ]


class MpmStateBlock(ctypes.Structure):
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("collider_count", ctypes.c_uint32),
        ("stamp", WireWorldStamp),
        ("step", ctypes.c_uint64),
        ("simulation_time", ctypes.c_double),
        ("colliders", WireColliderState * MAX_COLLIDERS),
    ]


class MpmStatusBlock(ctypes.Structure):
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("fault", ctypes.c_uint32),
        ("stamp", WireWorldStamp),
        ("acknowledged_step", ctypes.c_uint64),
        ("sidecar_step", ctypes.c_uint64),
        ("sidecar_time", ctypes.c_double),
        ("last_solve_seconds", ctypes.c_double),
        ("particle_count", ctypes.c_uint64),
        ("message", ctypes.c_char * 256),
    ]


class MpmParticleBlock(ctypes.Structure):
    """Decimated particle positions for RENDERING ONLY — never for physics.

    ⚠ Latest-wins, unlike the collider path: a dropped render frame is invisible, a stale one is a
    visible lie, and a writer that waited for the renderer would stall the solver.
    """

    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("particle_count", ctypes.c_uint32),
        ("stamp", WireWorldStamp),
        ("sidecar_step", ctypes.c_uint64),
        ("sidecar_time", ctypes.c_double),
        ("total_particles", ctypes.c_uint64),
        ("radius", ctypes.c_float),
        ("positions", ctypes.c_float * (MAX_RENDER_PARTICLES * 3)),
    ]


# ---------------------------------------------------------------------------------------------
# D15: the VEHICLE lives in the sidecar. Unreal sends joint targets; the sidecar sends link poses.
# ---------------------------------------------------------------------------------------------


class WireKinematicBody(ctypes.Structure):
    """A level object whose pose is pushed in every tick rather than frozen at load.

    ⚠ ONE-DIRECTIONAL BY CONSTRUCTION, and this is the honest limit rather than an oversight. The
    simulator dictates the pose, so the robot and the sand are pushed BY these bodies and never push
    back. A crate dropped into the bed will plough it and displace it, and will fall through it as
    though the sand were not there, because the thing deciding where the crate goes is Unreal's own
    physics and it has never heard of the sand. Making the crate float needs the crate solved in
    Newton — which is the same argument as D15 itself, one object further out.

    ⚠ NEVER a mesh: urdf::KinematicBody's shapes are primitives only.
    """

    _fields_ = [
        ("name", ctypes.c_char * MAX_STATIC_NAME_CHARS),
        ("shape_start", ctypes.c_uint32),
        ("shape_count", ctypes.c_uint32),
        # ⚠ SOLID TO THE SAND AND SOLID TO THE ROBOT ARE SEPARATE, and they fail separately: on
        # 2026-08-27 a mirrored sphere collided with the robot while the sand ignored it entirely.
        # Carrying both makes that a choice rather than a mystery, and lets a large moving object
        # stay in the scene without paying for it in the MPM contact solve.
        ("interact_with_mpm", ctypes.c_uint32),
        ("collide_with_robots", ctypes.c_uint32),
        ("friction", ctypes.c_double),
        ("restitution", ctypes.c_double),
        # ⚠ WHERE THE ACTOR WAS WHEN IT WAS REGISTERED, and building the body anywhere else is a
        # bug that only a MOVING object recovers from. The sidecar used to build every mirrored
        # body at the identity transform and rely on the per-tick pose to move it there; a cone
        # that never moves therefore sat at the sidecar's ORIGIN — an invisible solid a metre in
        # front of the robot's spawn — while the overlay drew it correctly 9 m away, because the
        # overlay draws where Unreal says it is. Reported 2026-08-27 as "the scout is colliding
        # with the air".
        ("position", WireVec3),
        ("orientation", WireQuat),
    ]

    def label(self) -> str:
        return self.name.decode("utf-8", "replace")


class WireKinematicPose(ctypes.Structure):
    """⚠ POSITIONALLY MATCHED to the kinematic array in the static world block, not keyed by name.
    A name lookup per body per tick would be wasted work on a hot path, and the registration is
    revisioned — so the contract is that pose[i] belongs to kinematic[i] of the SAME revision, and
    the consumer must ignore poses whose revision it has not built."""

    _fields_ = [
        ("position", WireVec3),
        ("orientation", WireQuat),
    ]


class WireJointCommand(ctypes.Structure):
    """One actuated joint's target for one tick."""

    _fields_ = [
        ("joint_name", ctypes.c_char * MAX_LINK_NAME_CHARS),
        ("target_mode", ctypes.c_uint32),   # WireJointTargetMode
        ("_pad", ctypes.c_uint32),
        ("target", ctypes.c_double),
    ]

    def name(self) -> str:
        return self.joint_name.decode("utf-8", "replace")


class WireVehicleCommand(ctypes.Structure):
    _fields_ = [
        ("vehicle_name", ctypes.c_char * MAX_VEHICLE_NAME_CHARS),
        ("joint_count", ctypes.c_uint32),
        ("_pad", ctypes.c_uint32),
        ("joints", WireJointCommand * MAX_JOINTS_PER_VEHICLE),
    ]

    def name(self) -> str:
        return self.vehicle_name.decode("utf-8", "replace")


class MpmVehicleCommandBlock(ctypes.Structure):
    """Unreal -> sidecar. Latest-wins.

    ⚠ LATEST-WINS, AND THAT IS THE RATE-DECOUPLING DECISION MADE CONCRETE. The sidecar consumes
    whatever command is current when it starts a solve and never waits for a newer one. A dropped
    command is a target that was superseded before it could be applied, which is exactly what
    should happen to it — the alternative is the simulator blocking on the sand solver, which is
    what plan D15 calls "who owns time" and what PX4 lockstep forbids outright.
    """

    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("vehicle_count", ctypes.c_uint32),
        ("stamp", WireWorldStamp),
        ("step", ctypes.c_uint64),
        ("simulation_time", ctypes.c_double),
        # ⚠ THE ONLY THING THAT CAN ACTUALLY RESET A SIDECAR-OWNED VEHICLE. The simulator resetting
        # its own world does nothing to a robot solved in another process: its Newton model keeps
        # the vehicle where it drove to and the sand keeps every rut. Bumping this is what tells
        # the sidecar to rebuild — fresh bed, vehicle back at its spawn.
        ("reset_epoch", ctypes.c_uint64),
        # ⚠ WHICH PIE SESSION THIS IS, AND WHY A COUNTER CANNOT ANSWER THAT. `reset_epoch` and
        # `kinematic_revision` both live on the backend, and the backend is constructed fresh for
        # every Play — so a new session restarts them from the same base the old one started from.
        # Stop PIE without pressing BackSpace, press Play again, and the sidecar is sent epoch 0
        # (already applied) and revision 1 (already built): no edge on either, so it carries on
        # with the previous session's rutted bed and the robot wherever it was left. That is the
        # "reset does not put the robot back" report of 2026-08-27, which was misdiagnosed as
        # anchor drift. A value that is RANDOM per backend cannot collide across sessions.
        ("session_id", ctypes.c_uint64),
        # ⚠ THE MOVING HALF OF THE LEVEL MIRROR, on the block that is already written every tick.
        # `kinematic_revision` says which registration these poses belong to; a consumer that has
        # built a different revision must ignore them rather than apply pose[i] to the wrong body.
        ("kinematic_revision", ctypes.c_uint32),
        ("kinematic_count", ctypes.c_uint32),
        ("kinematic_poses", WireKinematicPose * MAX_KINEMATIC_BODIES),
        ("vehicles", WireVehicleCommand * MAX_VEHICLES),
    ]


class WireLinkPose(ctypes.Structure):
    """⚠ VELOCITIES ARE CARRIED ON PURPOSE, and they are what make rate decoupling work.

    The sidecar publishes at 20-30 Hz and Unreal renders at 60. Interpolating position alone
    between two 33 ms-apart samples gives visible corner-cutting on a turning wheel; position plus
    twist gives a Hermite the consumer can evaluate at any instant, and extrapolation for the
    frames past the newest sample. This is also why the pose path can afford to lag at all: a
    one-step-old POSE is a small position error, whereas a one-step-old FORCE applied to a light
    wheel is the instability this workstream spent 2026-08-26 chasing.
    """

    _fields_ = [
        ("link_name", ctypes.c_char * MAX_LINK_NAME_CHARS),
        ("position", WireVec3),
        ("orientation", WireQuat),
        ("linear_velocity", WireVec3),
        ("angular_velocity", WireVec3),
    ]

    def name(self) -> str:
        return self.link_name.decode("utf-8", "replace")


class WireJointState(ctypes.Structure):
    """⚠ `effort` IS NOT REPORTED YET, and `effort_reported` on the enclosing WireVehiclePose says
    so rather than leaving a consumer to trust a zero. Position and velocity come straight out of
    Newton's `joint_q` / `joint_qd` and are exact; applied joint torque needs the solver's own
    force array, which the coupled proxy does not expose per-joint today. A silent 0.0 here would
    be indistinguishable from a motor doing nothing, which is exactly the shape of bug that cost
    this workstream a day (a published velocity that was the wrong half of a twist and read as a
    stationary vehicle)."""

    _fields_ = [
        ("joint_name", ctypes.c_char * MAX_LINK_NAME_CHARS),
        ("position", ctypes.c_double),
        ("velocity", ctypes.c_double),
        ("effort", ctypes.c_double),
    ]

    def name(self) -> str:
        return self.joint_name.decode("utf-8", "replace")


class WireVehiclePose(ctypes.Structure):
    _fields_ = [
        ("vehicle_name", ctypes.c_char * MAX_VEHICLE_NAME_CHARS),
        ("link_count", ctypes.c_uint32),
        ("joint_count", ctypes.c_uint32),
        ("effort_reported", ctypes.c_uint32),
        ("_pad", ctypes.c_uint32),
        # ⚠ WHERE THE ROOT LINK WAS BUILT, constant for the life of a model. The consumer anchors
        # its frame on THIS, never on a sampled pose. Anchoring on the first sample that happens to
        # arrive is a race: this process free-runs, so between the consumer building and reading its
        # first pose the robot has settled or rolled a little, and the offset silently absorbs that.
        # The symptom is a vehicle that reappears a few centimetres from where it was last time,
        # every reset, for no reason the operator can see.
        ("spawn_position", WireVec3),
        ("links", WireLinkPose * MAX_LINKS_PER_VEHICLE),
        ("joints", WireJointState * MAX_JOINTS_PER_VEHICLE),
    ]

    def name(self) -> str:
        return self.vehicle_name.decode("utf-8", "replace")


class MpmVehiclePoseBlock(ctypes.Structure):
    """Sidecar -> Unreal. Latest-wins, never blocking.

    ⚠ `sidecar_time` IS THE TIMESTAMP THE INTERPOLATION IS ANCHORED TO, and it is the sidecar's
    SIMULATED time, not wall time. A consumer that interpolated against its own clock would drift
    whenever the solve ran slower than real time — which it does, routinely, at 1.37 M particles.

    ⚠ `publish_interval_seconds` is the sidecar's NOMINAL cadence, published so the consumer can
    size its interpolation window without measuring it. It is advisory: the honest instantaneous
    interval is the difference of two consecutive `sidecar_time` values, and a consumer that needs
    to notice a stall must use that rather than trust this.
    """

    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("vehicle_count", ctypes.c_uint32),
        ("stamp", WireWorldStamp),
        ("sidecar_step", ctypes.c_uint64),
        ("sidecar_time", ctypes.c_double),
        ("publish_interval_seconds", ctypes.c_double),
        ("acknowledged_command_step", ctypes.c_uint64),
        ("vehicles", WireVehiclePose * MAX_VEHICLES),
    ]


class WireStaticShape(ctypes.Structure):
    """One collision primitive, in the frame of the body that owns it.

    ⚠ VERTICES AND INDICES LIVE IN SHARED POOLS, not in this struct. A per-shape array would have
    to be sized for the worst shape and multiplied by MAX_STATIC_SHAPES, which for a level mesh is
    absurd; the start/count pairs index the flat `vertices` and `indices` arrays on the block.
    """

    _fields_ = [
        ("kind", ctypes.c_uint32),
        ("vertex_start", ctypes.c_uint32),
        ("vertex_count", ctypes.c_uint32),
        ("index_start", ctypes.c_uint32),
        ("index_count", ctypes.c_uint32),
        ("_pad", ctypes.c_uint32),
        ("center_a", WireVec3),
        ("center_b", WireVec3),
        ("radius", ctypes.c_double),
    ]


class WireStaticBody(ctypes.Structure):
    """⚠ BODY-LOCAL SHAPES PLUS A WORLD TRANSFORM, matching urdf::StaticBody rather than flattening
    to world space. That is what lets one cooked mesh be reused by every instance of the same asset
    placed in a level — the difference between cooking a map once and once per placed rock."""

    _fields_ = [
        ("name", ctypes.c_char * MAX_STATIC_NAME_CHARS),
        ("position", WireVec3),
        ("orientation", WireQuat),
        ("shape_start", ctypes.c_uint32),
        ("shape_count", ctypes.c_uint32),
        ("friction", ctypes.c_double),
        ("restitution", ctypes.c_double),
    ]

    def label(self) -> str:
        return self.name.decode("utf-8", "replace")


class MpmStaticWorldBlock(ctypes.Structure):
    """Unreal -> sidecar. The level's collision geometry, published ONCE per revision.

    ⚠ WHY THIS EXISTS. Without it the sidecar's robot stands on a flat plane at a height somebody
    typed in, and collides with nothing else in the level at all. Getting that height right by hand
    failed twice on one afternoon — the two ends disagree about z by the AirSim NED origin's offset
    from the Unreal world origin (0.640 m on Blocks), so a plausible number can be wrong by exactly
    that and look like a physics bug. A level whose ground is not flat cannot be expressed as a
    number at all, and a level with obstacles never could.

    ⚠ PUBLISHED ON A REVISION, NOT PER TICK. It is 24 MB and it is static by construction; the
    sidecar rebuilds its collision only when `revision` changes.

    ⚠ `frame_offset` TRAVELS WITH THE GEOMETRY. The vertices are in the SOLVER frame (origin: the
    Unreal world origin), and the sidecar works in its own frame, offset from it. Sending the vector
    alongside the data means the sidecar subtracts it itself rather than the two ends separately
    agreeing on a convention that has already been got wrong once today.
    """

    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("sequence", ctypes.c_uint32),
        ("revision", ctypes.c_uint32),
        ("body_count", ctypes.c_uint32),
        ("shape_count", ctypes.c_uint32),
        ("vertex_count", ctypes.c_uint32),
        ("index_count", ctypes.c_uint32),
        # ⚠ Non-zero means the level did NOT fit and what follows is a PARTIAL world. A robot that
        # falls through a floor which was silently dropped is the worst outcome here, so both ends
        # report this rather than letting it look like geometry that was never there.
        ("truncated", ctypes.c_uint32),
        ("_pad", ctypes.c_uint32),
        ("frame_offset", WireVec3),
        ("kinematic_count", ctypes.c_uint32),
        ("_pad2", ctypes.c_uint32),
        ("kinematic", WireKinematicBody * MAX_KINEMATIC_BODIES),
        ("bodies", WireStaticBody * MAX_STATIC_BODIES),
        ("shapes", WireStaticShape * MAX_STATIC_SHAPES),
        ("vertices", ctypes.c_float * (MAX_STATIC_VERTICES * 3)),
        ("indices", ctypes.c_int32 * MAX_STATIC_INDICES),
    ]


class SegmentError(RuntimeError):
    pass


class Segment:
    """One mmapped block, opened by name in a directory.

    ⚠ File-backed mmap, NOT `shm_open`, matching `stream/SharedMemorySink.hpp`. /dev/shm IS tmpfs on
    Linux, so a file there is RAM at RAM speed — but the directory has to be a parameter, because a
    container gets a private 64 MiB /dev/shm unless it is explicitly shared.
    """

    def __init__(self, directory: str, name: str, block_type, create: bool = False):
        self.path = os.path.join(directory, name)
        self.block_type = block_type
        size = ctypes.sizeof(block_type)

        if create:
            # ⚠ NEVER TRUNCATE A SEGMENT THAT ALREADY EXISTS AT THE RIGHT SIZE. `open(path, "wb")`
            # truncates, and the simulator has usually already created and mmapped this same file —
            # truncating under a live mapping made the sim's view of the status block stop tracking
            # our writes. Measured 2026-08-26: the sim reported "sidecar has never written a status
            # block" for 56 s while the sidecar was writing one every loop, then saw a half-zero
            # block and paused the world for a wrong epoch that was really an empty read.
            #
            # Adopting an existing correctly-sized file is also the honest behaviour: whoever got
            # there first owns the inode, and both ends want the same bytes.
            if os.path.exists(self.path) and os.path.getsize(self.path) == size:
                self._file = open(self.path, "r+b")
            else:
                with open(self.path, "wb") as handle:
                    handle.write(b"\0" * size)
                self._file = open(self.path, "r+b")
        else:
            if not os.path.exists(self.path):
                raise SegmentError(
                    f"{self.path} does not exist — the simulator has not created it. "
                    f"Start EXECOsim with the MPM sidecar enabled first.")
            actual = os.path.getsize(self.path)
            if actual != size:
                raise SegmentError(
                    f"{self.path} is {actual} bytes, expected {size}. This is a PROTOCOL "
                    f"MISMATCH — the simulator and this sidecar were built from different "
                    f"versions of MpmSidecarProtocol.hpp / protocol.py.")
            self._file = open(self.path, "r+b")

        self._map = mmap.mmap(self._file.fileno(), size)
        self.block = block_type.from_buffer(self._map)

    def close(self) -> None:
        # ⚠ Drop the ctypes view BEFORE the mmap: mmap.close() raises BufferError while an exported
        # buffer is still alive, and the traceback points at the close rather than at the view.
        #
        # ⚠ AND THAT IS NOT ENOUGH ON ITS OWN. Dropping our reference does not collect sub-views
        # ctypes handed out — `block.positions` in the particle writer exports a buffer of its own —
        # so a cycle collection is needed before the mmap will release. Seen for real: a run that
        # had already written its recording still died with "cannot close exported pointers exist",
        # which looks like data loss and is not.
        self.block = None
        gc.collect()
        try:
            self._map.close()
        except BufferError:
            # Teardown only. The segment is a file-backed mapping; the OS reclaims it at exit, and
            # failing here would turn a cosmetic leak into a lost recording.
            pass
        self._file.close()

    def __enter__(self) -> "Segment":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def read_consistent(segment: Segment, retries: int = 64):
    """Read a block through its seqlock, retrying while the writer is mid-update.

    ⚠ The sequence is ODD while the writer is inside an update. Reading without this returns a
    block that is half old and half new — a collider array where some poses are from step N and
    some from N+1, which is indistinguishable from a robot that teleported.
    """
    for _ in range(retries):
        before = segment.block.sequence
        if before % 2 != 0:
            time.sleep(0.0001)
            continue
        snapshot = segment.block_type.from_buffer_copy(segment._map)
        if segment.block.sequence == before:
            return snapshot
        time.sleep(0.0001)
    raise SegmentError(
        f"{segment.path}: could not get a torn-free read after {retries} attempts. The writer is "
        f"updating faster than this reader can copy, or it died mid-update leaving an odd sequence.")


def validate(block, expected_magic: int, what: str, accept_older: bool = False) -> None:
    """Check a block is what it claims to be.

    ⚠ `accept_older` is for RECORDINGS ONLY, never for live shared memory. A recording is a
    historical artefact, and refusing to replay last week's capture because the live protocol moved
    on is the wrong trade — the strictness exists to catch a mismatched live PAIR, which is a
    different situation entirely.

    ⚠ It is only sound while the recorded structs' LAYOUTS are unchanged. Version 4 added a new
    block and touched neither MpmRegistryBlock nor MpmStateBlock, so v3 captures parse exactly. If a
    future bump moves a field inside either, this tolerance becomes a silent misread and must go.
    """
    if block.magic != expected_magic:
        raise SegmentError(
            f"{what}: magic 0x{block.magic:08X}, expected 0x{expected_magic:08X}. This segment is "
            f"not what it claims to be, or was never written.")
    if block.version != PROTOCOL_VERSION:
        if accept_older and 3 <= block.version < PROTOCOL_VERSION:
            print(f"⚠ {what}: recorded at protocol version {block.version}, replaying with "
                  f"{PROTOCOL_VERSION}. The layouts this reads did not change between them.")
        else:
            raise SegmentError(
                f"{what}: protocol version {block.version}, this sidecar speaks "
                f"{PROTOCOL_VERSION}. Rebuild the simulator and the sidecar from the same commit.")


if __name__ == "__main__":
    for name, kind in (("WireStaticShape", WireStaticShape),
                       ("WireStaticBody", WireStaticBody),
                       ("MpmStaticWorldBlock", MpmStaticWorldBlock),
                       ("WireJointCommand", WireJointCommand),
                       ("WireVehicleCommand", WireVehicleCommand),
                       ("MpmVehicleCommandBlock", MpmVehicleCommandBlock),
                       ("WireLinkPose", WireLinkPose),
                       ("WireJointState", WireJointState),
                       ("WireVehiclePose", WireVehiclePose),
                       ("MpmVehiclePoseBlock", MpmVehiclePoseBlock),
                       ("MpmParticleBlock", MpmParticleBlock),
                       ("WireColliderImpulse", WireColliderImpulse),
                       ("MpmImpulseBlock", MpmImpulseBlock),
                       ("WireTerrainRegion", WireTerrainRegion),
                       ("WireShape", WireShape),
                       ("WireColliderRegistration", WireColliderRegistration),
                       ("WireColliderState", WireColliderState),
                       ("MpmRegistryBlock", MpmRegistryBlock),
                       ("MpmStateBlock", MpmStateBlock),
                       ("MpmStatusBlock", MpmStatusBlock)):
        print(f"{name:26s} {ctypes.sizeof(kind):>10d} B")
