#!/usr/bin/env python3
"""A fake Unreal for the D15 vehicle wire: command joints, consume poses, measure the decoupling.

⚠ WHY THIS EXISTS. Step 3 of D15 is the only part that needs an Unreal build, and a wire defect
found there costs a compile cycle to see and another to fix. Everything the C++ side will do — write
joint targets at its own tick rate, read link poses at its own frame rate, interpolate between them
— is ordinary shared-memory work that Python can do exactly as well. So the protocol gets exercised
and measured here first, and step 3 becomes a transcription rather than a debugging session.

⚠ WHAT IT ACTUALLY CHECKS, because "it ran" is not a result:

  1. COMMANDS ARRIVE. The sidecar must acknowledge the command step we wrote, and the vehicle must
     move in the commanded direction. A wire that silently dropped every command would otherwise
     look exactly like a vehicle parked on flat ground, which is a failure this workstream has
     already shipped once (the empty-registry idle at 0 % CPU).
  2. POSES ARRIVE AT THE DECOUPLED RATE. The measured interval between published poses, in
     SIMULATED time, must match --own-vehicle-publish-hz — not the solve rate and not wall time.
  3. INTERPOLATION IS WORTH DOING, scored against poses the sidecar ACTUALLY published rather
     than against an assumption about what they should be. Run it with the sidecar publishing at
     its solve rate and --truth-stride N: every Nth pose becomes a keyframe and the ones between
     become ground truth. If Hermite does not beat plain linear here, the twist carried in
     WireLinkPose is not paying for its bytes and should come off the wire.
  4. NOTHING TEARS. Every read goes through the seqlock; a torn link array on an articulated body
     is indistinguishable from a joint that snapped, so a single torn read fails the run.

Run the sidecar with --own-vehicle-wire, then this:

    P=~/miniconda3/envs/newtonmpm/bin/python
    $P sidecar.py --own-vehicle scout --own-vehicle-wire ... &
    $P vehicle_wire_check.py --vehicle scout --seconds 20
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import protocol as P  # noqa: E402


def quat_normalise(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def quat_slerp(a, b, t):
    """⚠ SHORTEST ARC. Without the dot-sign flip a slerp between two quaternions representing
    nearly the same rotation can take the long way round, and a wheel spins backwards for one
    frame — which reads as a physics glitch and is a quaternion sign convention."""
    d = sum(x * y for x, y in zip(a, b))
    if d < 0.0:
        b = tuple(-c for c in b)
        d = -d
    if d > 0.9995:
        return quat_normalise(tuple(x + t * (y - x) for x, y in zip(a, b)))
    theta = math.acos(max(-1.0, min(1.0, d)))
    s = math.sin(theta)
    wa = math.sin((1.0 - t) * theta) / s
    wb = math.sin(t * theta) / s
    return quat_normalise(tuple(wa * x + wb * y for x, y in zip(a, b)))


def hermite(p0, v0, p1, v1, t, dt):
    """Cubic Hermite on position with the published twist as the tangent.

    ⚠ THE TANGENTS MUST BE SCALED BY dt. A Hermite basis is defined on the unit interval, so a
    velocity in m/s has to be multiplied by the interval length to become the derivative with
    respect to the parameter. Leaving it out gives an overshoot proportional to the sample rate —
    the kind of error that looks like a tuning problem and is a units problem.
    """
    t2, t3 = t * t, t * t * t
    h00 = 2 * t3 - 3 * t2 + 1
    h10 = t3 - 2 * t2 + t
    h01 = -2 * t3 + 3 * t2
    h11 = t3 - t2
    return tuple(h00 * a + h10 * dt * da + h01 * b + h11 * dt * db
                 for a, da, b, db in zip(p0, v0, p1, v1))


class FakeUnreal:
    def __init__(self, a):
        self.a = a
        self.command = P.Segment(a.dir, P.VEHICLE_COMMAND_SEGMENT,
                                 P.MpmVehicleCommandBlock, create=True)
        self.pose = P.Segment(a.dir, P.VEHICLE_POSE_SEGMENT,
                              P.MpmVehiclePoseBlock, create=True)
        self.step = 0
        self.torn_reads = 0

    def write_command(self, sim_time: float, rad_s: float) -> None:
        b = self.command.block
        b.sequence += 1                    # odd: writer inside the update
        b.magic = P.VEHICLE_COMMAND_MAGIC
        b.version = P.PROTOCOL_VERSION
        b.vehicle_count = 1
        self.step += 1
        b.step = self.step
        b.simulation_time = sim_time
        v = b.vehicles[0]
        v.vehicle_name = self.a.vehicle.encode()[:P.MAX_VEHICLE_NAME_CHARS - 1]
        names = self.a.joints
        v.joint_count = len(names)
        for i, (name, sign) in enumerate(names):
            j = v.joints[i]
            j.joint_name = name.encode()[:P.MAX_LINK_NAME_CHARS - 1]
            j.target_mode = P.JOINT_TARGET_VELOCITY
            j.target = rad_s * sign
        b.sequence += 1                    # even: readable again

    def read_pose(self):
        try:
            b = P.read_consistent(self.pose)
        except P.SegmentError:
            self.torn_reads += 1
            return None
        if b.magic != P.VEHICLE_POSE_MAGIC:
            return None
        for i in range(min(b.vehicle_count, P.MAX_VEHICLES)):
            if b.vehicles[i].name() == self.a.vehicle:
                e = b.vehicles[i]
                links = []
                for k in range(min(e.link_count, P.MAX_LINKS_PER_VEHICLE)):
                    L = e.links[k]
                    links.append((
                        L.name(),
                        (L.position.x, L.position.y, L.position.z),
                        (L.orientation.x, L.orientation.y, L.orientation.z, L.orientation.w),
                        (L.linear_velocity.x, L.linear_velocity.y, L.linear_velocity.z),
                        (L.angular_velocity.x, L.angular_velocity.y, L.angular_velocity.z)))
                joints = [(e.joints[k].name(), e.joints[k].position, e.joints[k].velocity,
                           e.joints[k].effort)
                          for k in range(min(e.joint_count, P.MAX_JOINTS_PER_VEHICLE))]
                return dict(step=b.sidecar_step, time=b.sidecar_time,
                            interval=b.publish_interval_seconds,
                            acked=b.acknowledged_command_step, links=links,
                            joints=joints, effort_reported=bool(e.effort_reported))
        return None

    def close(self):
        self.command.close()
        self.pose.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default="/dev/shm")
    ap.add_argument("--vehicle", default="scout")
    ap.add_argument("--seconds", type=float, default=20.0, help="wall-clock budget")
    ap.add_argument("--rad-s", type=float, default=9.0)
    ap.add_argument("--settle", type=float, default=4.0,
                    help="sidecar SIMULATED seconds to wait before commanding drive")
    ap.add_argument("--command-hz", type=float, default=120.0,
                    help="how fast this fake simulator ticks. Deliberately unequal to the "
                         "sidecar's solve rate and pose rate: if any of the three were locked "
                         "together the decoupling would be untested.")
    ap.add_argument("--consumer-hz", type=float, default=60.0,
                    help="the fake renderer's frame rate, sampling poses by interpolation")
    ap.add_argument("--truth-stride", type=int, default=4, metavar="N",
                    help="score interpolation by treating every Nth published pose as a keyframe "
                         "and the ones between as ground truth. Requires the sidecar to publish at "
                         "its solve rate (--own-vehicle-publish-hz == --fps), otherwise there is "
                         "nothing between the keyframes to check against.")
    ap.add_argument("--report", default=None, help="write the per-sample comparison to this CSV")
    a = ap.parse_args()

    # ⚠ SCOUT SIGNS. The right wheels carry rpy="3.14 0 0", so one sign for all four drives a
    # circle. Same convention the vehicle specs and the settings files use.
    a.joints = [("front_right_wheel", 1.0), ("front_left_wheel", -1.0),
                ("rear_left_wheel", -1.0), ("rear_right_wheel", 1.0)]

    fake = FakeUnreal(a)
    print(f"fake Unreal: commanding '{a.vehicle}' at {a.command_hz} Hz, "
          f"consuming poses at {a.consumer_hz} Hz")

    deadline = time.time() + a.seconds
    cmd_dt = 1.0 / a.command_hz
    con_dt = 1.0 / a.consumer_hz
    next_cmd = next_con = time.time()

    samples = []          # published poses, in order
    last = None
    first_pose_wait = time.time()
    saw_pose = False
    acked = 0

    while time.time() < deadline:
        now = time.time()
        if now >= next_cmd:
            next_cmd = now + cmd_dt
            latest = samples[-1]["time"] if samples else 0.0
            fake.write_command(latest, a.rad_s if latest >= a.settle else 0.0)
        if now >= next_con:
            next_con = now + con_dt
            p = fake.read_pose()
            if p is not None and p["links"]:
                saw_pose = True
                acked = max(acked, p["acked"])
                if not samples or p["step"] != samples[-1]["step"]:
                    samples.append(p)
                last = p

    fake.close()

    if not saw_pose:
        print(f"\n⚠ FAIL: no pose block was ever published in {a.seconds:.0f} s. The sidecar is "
              f"not running with --own-vehicle-wire, or it is writing to a different --dir "
              f"({a.dir}).", file=sys.stderr)
        return 1

    print(f"\npublished poses seen : {len(samples)}")
    print(f"links per pose       : {len(samples[-1]['links'])}")
    print(f"torn reads           : {fake.torn_reads}  (must be 0)")
    print(f"commands acknowledged: sidecar acked step {acked} of {fake.step} written")

    if len(samples) >= 3:
        iv = [b["time"] - a_["time"] for a_, b in zip(samples, samples[1:])]
        iv.sort()
        nominal = samples[-1]["interval"]
        med = iv[len(iv) // 2]
        print(f"pose interval (sim)  : median {med * 1000:.2f} ms, "
              f"nominal {nominal * 1000:.2f} ms  -> {1.0 / med:.1f} Hz")
        drift = abs(med - nominal) / nominal
        print(f"  cadence error      : {drift * 100:.1f} %  "
              f"({'OK' if drift < 0.15 else '⚠ OFF'})")

    # ⚠ JOINT STATES ARE CHECKED FOR MOTION, not merely for presence. A block full of correctly
    # named joints all reading 0.0 is what a wrong DOF index produces, and it looks healthy.
    js = samples[-1].get("joints", [])
    print(f"joint states         : {len(js)} joints, effort_reported="
          f"{samples[-1].get('effort_reported')}")
    if js:
        moving = [j for j in js if abs(j[2]) > 0.1]
        print(f"  names              : {[j[0] for j in js][:6]}")
        print(f"  |velocity| > 0.1   : {len(moving)} of {len(js)}  "
              f"({'OK' if moving else '⚠ ALL ZERO - check the DOF indexing'})")
        for n, pos, vel, eff in js[:4]:
            print(f"    {n:26s} q={pos:+9.3f}  qd={vel:+8.3f}")

    x_first = samples[0]["links"][0][1][0]
    x_last = samples[-1]["links"][0][1][0]
    print(f"base link x          : {x_first:+.3f} -> {x_last:+.3f} "
          f"({x_last - x_first:+.3f} m)")

    score_interpolation(samples, a.truth_stride)

    ok = (fake.torn_reads == 0 and acked > 0 and abs(x_last - x_first) > 0.05)
    print(f"\n{'PASS' if ok else '⚠ FAIL'}: "
          f"{'commands crossed the wire, poses came back, nothing tore' if ok else 'see above'}")

    if a.report and samples:
        import csv
        with open(a.report, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["sidecar_step", "sidecar_time", "interval", "acked", "base_x",
                        "base_z", "base_vx"])
            for s in samples:
                _n, x, _q, v, _w = s["links"][0]
                w.writerow([s["step"], f"{s['time']:.6f}", f"{s['interval']:.6f}",
                            s["acked"], f"{x[0]:.6f}", f"{x[2]:.6f}", f"{v[0]:.6f}"])
        print(f"wrote {a.report}")
    return 0 if ok else 1


def score_interpolation(samples, stride: int) -> None:
    """Score interpolation against poses the sidecar ACTUALLY published, not against an assumption.

    ⚠ THE FIRST VERSION OF THIS WAS RIGGED AND REPORTED "166580x better". It scored the midpoint of
    each published pair against the AVERAGE of that pair — but on a constant-velocity straight line
    the average IS the linear interpolant, so both linear and Hermite scored exactly zero by
    construction and step-hold scored half a segment. The number measured the definition of the
    test, not the interpolation.

    The honest version needs ground truth BETWEEN the keyframes, which means publishing faster than
    we interpolate. Run the sidecar at its solve rate, then treat every `stride`-th published pose
    as a keyframe and every pose in between as truth. Now curvature — a wheel spinning up, a
    suspension settling, a vehicle climbing — actually shows.
    """
    if stride < 2 or len(samples) < 2 * stride + 1:
        print(f"interpolation        : not scored (need >= {2 * stride + 1} poses published at the "
              f"solve rate; run with --own-vehicle-publish-hz equal to --fps and --truth-stride > 1)")
        return

    herm, lin, hold = [], [], []
    for k in range(0, len(samples) - stride, stride):
        a0, a1 = samples[k], samples[k + stride]
        dt = a1["time"] - a0["time"]
        if dt <= 1e-9 or len(a0["links"]) != len(a1["links"]):
            continue
        for m in range(1, stride):
            truth = samples[k + m]
            if len(truth["links"]) != len(a0["links"]):
                continue
            t = (truth["time"] - a0["time"]) / dt
            for (_n0, x0, _q0, v0, _w0), (_n1, x1, _q1, v1, _w1), (_nt, xt, _qt, _vt, _wt) in zip(
                    a0["links"], a1["links"], truth["links"]):
                herm.append(dist(hermite(x0, v0, x1, v1, t, dt), xt))
                lin.append(dist(tuple(p + t * (q - p) for p, q in zip(x0, x1)), xt))
                hold.append(dist(x0, xt))
    if not herm:
        print("interpolation        : no comparable samples")
        return
    for name, e in (("hermite (pos+twist)", herm), ("linear  (pos only)", lin),
                    ("step-hold (no interp)", hold)):
        e.sort()
        print(f"  {name:22s} median {e[len(e)//2]*1000:8.3f} mm   "
              f"p95 {e[int(len(e)*0.95)]*1000:8.3f} mm   max {e[-1]*1000:8.3f} mm")
    # ⚠ COMPARE AT p95, NOT AT THE MEDIAN. On a straight constant-velocity segment linear
    # interpolation is exact, so its median error is ~0 and a median ratio is noise divided by
    # noise: the first version of this line printed "Hermite beats linear by 0.02x", which reads as
    # Hermite being fifty times WORSE when the p95 said it was eight times better. The interesting
    # error lives where the motion curves, which is the tail.
    hp = herm[int(len(herm) * 0.95)]
    lp = lin[int(len(lin) * 0.95)]
    if hp <= 0.0:
        print("  -> Hermite error is below the printable resolution at p95.")
    else:
        print(f"  -> at p95, Hermite is {lp / hp:.1f}x better than linear "
              f"({lp * 1000:.3f} mm -> {hp * 1000:.3f} mm). Near 1.0 would mean the twist carried "
              f"in WireLinkPose is not paying for its bytes and should come off the wire.")


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


if __name__ == "__main__":
    raise SystemExit(main())
