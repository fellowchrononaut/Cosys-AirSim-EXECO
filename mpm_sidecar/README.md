# Newton MPM sidecar

The consumer half of EXECOsim's deformable-terrain link. The producer is
`../AirLib/include/mpm/MpmSidecarProtocol.hpp` and `../AirLib/src/mpm/`.

⚠ **`protocol.py` is half of a wire format.** It is a hand-maintained ctypes mirror of
`MpmSidecarProtocol.hpp`, and the two are only correct together. They live in the same repository
so that a change to one and not the other is a single broken commit rather than a silent drift
between two repositories — which is what this directory was moved here to fix (2026-08-26).

Before changing either side, run the round-trip check, which writes the structs from C++ and reads
them back in Python:

    python mpm_sidecar/wire_roundtrip_check.py

## Why a separate process at all

Newton is Python-only with no C API, so the MPM solver cannot be linked into Unreal. EXECOsim
publishes collider poses into `/dev/shm`, the sidecar solves sand on the GPU, and it publishes a
decimated particle set back for rendering.

⚠ **One-way.** Sand is deformed by the robot; the robot feels nothing back. Two-way needs each
link's articulated effective inertia, which no backend can supply — see the plan's §11.1.

## Files

| file | what it is |
|---|---|
| `sidecar.py` | the solver process: reads colliders, steps Newton MPM, publishes particles |
| `protocol.py` | ctypes mirror of the C++ wire header |
| `wire_roundtrip_check.{cpp,py}` | proves the two halves agree on struct layout |
| `fake_sim.cpp` | publishes a synthetic world, for running the sidecar without Unreal |
| `reset_probe.py` | fake simulator that performs a global reset, exercising rebuild-on-epoch |
| `drive_across_patch.py` | drives Rover1 over RPC so a sand test is repeatable |

⚠ `drive_across_patch.py` talks to **port 41454**. MultiAgent mode runs one RPC server per vehicle
family — 41451 multirotor, 41452 car, 41453 computer-vision, 41454 urdfbot — because
`getVehicleApi()` static_casts to the family's api type.

## Environment

See `requirements.txt`. Newton itself is not on PyPI; the pinned upstream commit is recorded there.

## Operating notes

The full runbook, including patch sizing, voxel/depth limits and offline rendering, is
`SIMVAL/Newton-MPM-Discussions/MPM-RUNBOOK.md`.
