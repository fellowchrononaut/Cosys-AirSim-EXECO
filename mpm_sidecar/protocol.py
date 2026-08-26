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

# ⚠ 4 since 2026-08-26: added MpmImpulseBlock — the sand's reaction back onto the colliders. The
# registry and state LAYOUTS are unchanged by this bump, which is what makes recordings written at
# version 3 still readable; see `validate(..., accept_older=True)`.
# ⚠ 3 since 2026-08-25: added MpmParticleBlock so Unreal can render the sand.
# ⚠ 2 since 2026-08-25: the registry gained the terrain REGION. Before that the simulator declared
# a region in settings and this sidecar spawned sand wherever its CLI defaults said, with nothing
# carrying one to the other.
PROTOCOL_VERSION = 4

REGISTRY_MAGIC = 0x4D504D52  # 'MPMR'
STATE_MAGIC = 0x4D504D53     # 'MPMS'
STATUS_MAGIC = 0x4D504D48    # 'MPMH'
IMPULSE_MAGIC = 0x4D504D49   # 'MPMI'
PARTICLE_MAGIC = 0x4D504D50  # 'MPMP'

MAX_COLLIDERS = 256
MAX_SHAPES_PER_COLLIDER = 16
MAX_COLLIDER_ID_CHARS = 64
MAX_TERRAIN_ID_CHARS = 64
MAX_RENDER_PARTICLES = 100000
MAX_SHAPE_VERTICES = 64

REGISTRY_SEGMENT = "execosim_mpm_registry"
STATE_SEGMENT = "execosim_mpm_state"
STATUS_SEGMENT = "execosim_mpm_status"
IMPULSE_SEGMENT = "execosim_mpm_impulse"
PARTICLE_SEGMENT = "execosim_mpm_particles"

# WireShapeKind — numeric values frozen by PROTOCOL_VERSION.
SHAPE_SPHERE = 0
SHAPE_CAPSULE = 1
SHAPE_CYLINDER = 2
SHAPE_BOX = 3
SHAPE_CONVEX_HULL = 4
SHAPE_PLANE = 5

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
    for name, kind in (("MpmParticleBlock", MpmParticleBlock),
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
