"""Read back what wire_roundtrip_check.cpp wrote, and assert every field landed where it belongs.

⚠ THE POINT IS FIELD ORDER, NOT SIZE. protocol.py already asserts sizes at import; two same-typed
fields swapped between the two halves of the wire format keeps every size identical and silently
reinterprets the data. Each field below carries a distinct value so a swap cannot pass.

    g++ -std=c++17 -I AirLib/include \
        mpm_sidecar/wire_roundtrip_check.cpp -o /tmp/wire_write
    /tmp/wire_write /tmp/mpmwire && python mpm_sidecar/wire_roundtrip_check.py /tmp/mpmwire
"""

from __future__ import annotations

import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])

import protocol as P  # noqa: E402

failures = 0


def check(ok: bool, what: str, detail: str = "") -> None:
    global failures
    if ok:
        print(f"  [PASS] {what}" + (f"  {detail}" if detail else ""))
    else:
        failures += 1
        print(f"  [FAIL] {what}" + (f"  {detail}" if detail else ""))


def main() -> int:
    directory = sys.argv[1] if len(sys.argv) > 1 else "/tmp"
    print(f"\n  -- MPM wire round trip ({directory}) --")

    with P.Segment(directory, P.REGISTRY_SEGMENT, P.MpmRegistryBlock) as seg:
        registry = P.read_consistent(seg)
    P.validate(registry, P.REGISTRY_MAGIC, "registry")

    check(registry.collider_count == 2, "registry carries both colliders")
    check(registry.stamp.key() == (11, 22, 33, 44), "world stamp survives",
          str(registry.stamp))
    check(abs(registry.sim_fixed_dt - 0.003) < 1e-12, "sim fixed dt survives",
          f"{registry.sim_fixed_dt}")

    first = registry.colliders[0]
    check(first.name() == "Rover1/wheel_lf", "stable id decodes", first.name())
    check(first.role == P.ROLE_KINEMATIC_ONE_WAY, "coupling role survives")
    check(abs(first.mass - 1.25) < 1e-12, "mass survives")
    check(first.com_local.as_tuple() == (0.1, 0.2, 0.3), "body-local COM survives",
          str(first.com_local.as_tuple()))
    # 100..108 in order: this is what catches a transposed or rotated inertia tensor.
    check(list(first.inertia_local) == [100.0 + i for i in range(9)],
          "inertia tensor survives IN ORDER", str(list(first.inertia_local)[:3]) + "...")
    check(first.inertia_is_articulated_effective == 0,
          "the effective-inertia flag survives, and is false")
    check(abs(first.friction - 0.71) < 1e-12 and abs(first.restitution - 0.13) < 1e-12,
          "friction and restitution are not swapped",
          f"mu={first.friction} e={first.restitution}")

    shape = first.shapes[0]
    check(shape.kind == P.SHAPE_CONVEX_HULL, "shape kind survives")
    check(shape.position.as_tuple() == (1.5, 2.5, 3.5), "shape position survives")
    check(shape.orientation.as_xyzw() == (0.1, 0.2, 0.3, 0.927),
          "shape orientation survives as x,y,z,w", str(shape.orientation.as_xyzw()))
    check(shape.orientation.as_wxyz() == (0.927, 0.1, 0.2, 0.3),
          "and converts to w-first for Newton without reordering the wire")
    check(abs(shape.radius - 0.42) < 1e-12 and abs(shape.half_length - 0.84) < 1e-12,
          "radius and half_length are not swapped")
    check(shape.half_extents.as_tuple() == (4.5, 5.5, 6.5), "box half extents survive")
    check(shape.hull_vertices() == [(10.0, 20.0, 30.0), (11.0, 21.0, 31.0), (12.0, 22.0, 32.0)],
          "hull vertices survive, truncated to vertex_count",
          f"{len(shape.hull_vertices())} of {P.MAX_SHAPE_VERTICES} slots")

    check(registry.colliders[1].name() == "Rover2/wheel_rr", "the second collider is distinct")
    check(registry.colliders[1].role == P.ROLE_STATIC, "and carries its own role")

    with P.Segment(directory, P.STATE_SEGMENT, P.MpmStateBlock) as seg:
        state = P.read_consistent(seg)
    P.validate(state, P.STATE_MAGIC, "state")

    check(state.step == 987654321, "a 64-bit step counter survives without truncation",
          str(state.step))
    check(abs(state.simulation_time - 12.75) < 1e-12, "simulation time survives")
    check(state.stamp.key() == registry.stamp.key(), "state and registry agree on the world")

    body = state.colliders[0]
    # ⚠ THE SWAP TEST. Four same-typed vectors in a row; only distinct values catch a reorder.
    check(body.position.as_tuple() == (1.0, 2.0, 3.0), "position survives")
    check(body.orientation.as_xyzw() == (0.5, 0.5, 0.5, 0.5), "orientation survives")
    check(body.linear_velocity.as_tuple() == (4.0, 5.0, 6.0),
          "linear velocity is NOT read as position", str(body.linear_velocity.as_tuple()))
    check(body.angular_velocity.as_tuple() == (7.0, 8.0, 9.0),
          "angular velocity is NOT read as linear", str(body.angular_velocity.as_tuple()))

    with P.Segment(directory, P.STATUS_SEGMENT, P.MpmStatusBlock) as seg:
        status = P.read_consistent(seg)
    P.validate(status, P.STATUS_MAGIC, "status")

    check(status.acknowledged_step == 987654300, "the acknowledgement survives",
          str(status.acknowledged_step))
    check(status.sidecar_step == 176 and abs(status.sidecar_time - 2.9333) < 1e-12,
          "the sidecar's own step and time are kept SEPARATE from the sim's",
          f"sidecar step {status.sidecar_step} @ {status.sidecar_time}s vs sim step {state.step}")
    check(state.step - status.acknowledged_step == 21,
          "so the lag is computable, which is what makes a stale sidecar detectable",
          f"{state.step - status.acknowledged_step} steps behind")
    check(abs(status.last_solve_seconds - 0.0122) < 1e-12, "solve cost survives")
    check(status.particle_count == 250000, "particle count survives")
    check(status.message.decode().rstrip("\0") == "healthy", "the message decodes",
          status.message.decode().rstrip("\0"))

    print(f"\n{'WIRE ROUND TRIP PASS' if failures == 0 else 'FAILURES'}  "
          f"({failures} failure{'' if failures == 1 else 's'})\n")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
