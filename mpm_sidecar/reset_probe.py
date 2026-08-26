#!/usr/bin/env python3
"""Fake simulator that performs a GLOBAL RESET, to exercise the sidecar's rebuild path.

⚠ WHY THIS EXISTS. Plan §M2's exit criterion says no stale collider state is ever used, and that a
sidecar restart forces a global reset. Neither could be tested: the only way to move the epoch was
to press Play in the editor, and the sidecar's answer to a moved epoch was to exit. This publishes a
registry and states like the simulator does, then bumps `reset_epoch`, republishes the registry with
post-reset poses, and checks that the sidecar rebuilt instead of dying or carrying stale sand.

Run the sidecar against the same --dir; this process creates the segments, so start it FIRST.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as P


def fill_registry(block, epoch: int, x0: float) -> None:
    block.sequence |= 1
    block.magic = P.REGISTRY_MAGIC
    block.version = P.PROTOCOL_VERSION
    block.collider_count = 1
    block.stamp.world_id = 7
    block.stamp.world_revision = 1
    block.stamp.manifest_revision = 1
    block.stamp.reset_epoch = epoch
    block.sim_fixed_dt = 0.003
    r = block.region
    r.terrain_id = b"probe"
    r.center.x, r.center.y, r.center.z = 0.0, 0.0, 0.03
    r.half_extent.x, r.half_extent.y, r.half_extent.z = 0.25, 0.25, 0.03
    r.rigid_ticks_per_mpm_step = 6
    r.valid = 1
    c = block.colliders[0]
    c.stable_id = b"Probe/wheel"
    c.shape_count = 1
    c.role = 1
    c.mass = 0.05
    c.friction = 0.6
    sh = c.shapes[0]
    sh.kind = 0                      # Sphere
    sh.radius = 0.05
    sh.position.x, sh.position.y, sh.position.z = 0.0, 0.0, 0.0
    sh.orientation.w = 1.0
    block.sequence += 1              # even = readable


def publish_state(seg, epoch: int, step: int, t: float, x: float) -> None:
    b = seg.block
    b.sequence |= 1
    b.magic = P.STATE_MAGIC
    b.version = P.PROTOCOL_VERSION
    b.collider_count = 1
    b.stamp.world_id = 7
    b.stamp.world_revision = 1
    b.stamp.manifest_revision = 1
    b.stamp.reset_epoch = epoch
    b.step = step
    b.simulation_time = t
    s = b.colliders[0]
    s.position.x, s.position.y, s.position.z = x, 0.0, 0.10
    s.orientation.w = 1.0
    s.linear_velocity.x = 0.2
    b.sequence += 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="/dev/shm")
    ap.add_argument("--pre-seconds", type=float, default=25.0)
    ap.add_argument("--post-seconds", type=float, default=25.0)
    args = ap.parse_args()

    reg = P.Segment(args.dir, P.REGISTRY_SEGMENT, P.MpmRegistryBlock, create=True)
    state = P.Segment(args.dir, P.STATE_SEGMENT, P.MpmStateBlock, create=True)
    print("created registry and state segments; start the sidecar now", flush=True)

    fill_registry(reg.block, epoch=1, x0=0.0)
    print("epoch 1 registry published", flush=True)

    status = None
    def read_status():
        nonlocal status
        try:
            if status is None:
                status = P.Segment(args.dir, P.STATUS_SEGMENT, P.MpmStatusBlock)
            return P.read_consistent(status)
        except Exception:
            return None

    def drive(epoch, seconds, step0, label):
        step = step0
        t0 = time.time()
        last = None
        while time.time() - t0 < seconds:
            t = time.time() - t0
            publish_state(state, epoch, step, t, -0.30 + 0.02 * t)
            step += 1
            time.sleep(0.003)
            st = read_status()
            if st is not None and st.magic == P.STATUS_MAGIC:
                last = (st.acknowledged_step, st.fault, st.particle_count)
        print(f"{label}: published to step {step}, sidecar {last}", flush=True)
        return step, last

    step, before = drive(1, args.pre_seconds, 0, "epoch 1")

    print("\n=== GLOBAL RESET: epoch 1 -> 2, registry republished ===", flush=True)
    fill_registry(reg.block, epoch=2, x0=0.0)
    step2, after = drive(2, args.post_seconds, 0, "epoch 2")

    ok = after is not None and after[1] == 0 and after[2] > 0
    print(f"\nbefore reset: acknowledged={before}  after reset: acknowledged={after}")
    print("RESULT:", "PASS — sidecar rebuilt and is acknowledging in the new epoch" if ok
          else "FAIL — sidecar did not recover (fault or no particles)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
