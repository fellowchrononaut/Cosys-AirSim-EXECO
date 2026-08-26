#!/usr/bin/env python3
"""Drop two identical rovers from the same height — one onto the sand bed, one onto bare ground —
and measure the difference in where they come to rest.

⚠ WHY A DIFFERENTIAL, NOT AN ABSOLUTE. "The rover settled at z = 0.48" answers nothing on its own:
it needs the URDF's base-to-wheel-bottom offset, the level's ground height and the suspension's
compression to interpret, and getting any of the three wrong silently produces a confident wrong
verdict. Two rovers dropped in the SAME tick of the SAME run differ in exactly one thing — whether
there is sand underneath — so the gap between their resting heights IS the sand's support, with no
geometry to derive.

The bed is 0.25 m deep and its floor is the level's ground, so the gap is bounded:
    gap ~ 0.25 m  the rover rests on top of the sand; it is a surface
    gap ~ 0.00 m  the rover sank straight through to the ground; the sand is not there
    in between     partial support - it sinks in and stops, which is what real sand does

Run it with the editor in PIE and the sidecar up. It takes no control of either vehicle: gravity
does the whole experiment.
"""
import argparse, csv, math, os, sys, time

import msgpackrpc

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol

URDFBOT_PORT = 41454   # see drive_across_patch.py: one rpc server per vehicle family


def pose_z_up(client, vehicle):
    """Vehicle pose height in metres ABOVE the origin.

    ⚠ NED IN, Z-UP OUT. simGetVehiclePose answers in global NED where z points DOWN, so a rover
    that is higher reports a MORE NEGATIVE number. Every threshold below is written as a height,
    which is how a person thinks about a drop, so the flip happens once, here.
    """
    p = client.call("simGetVehiclePose", vehicle)
    pos = p["position"]
    return -pos["z_val"], pos["x_val"], pos["y_val"]


def read_impulses(shm_dir):
    """Newest impulse block, or None when the sidecar is one-way or not up yet."""
    try:
        with protocol.Segment(shm_dir, protocol.IMPULSE_SEGMENT, protocol.MpmImpulseBlock) as seg:
            block = protocol.read_consistent(seg)
            if block is None or block.magic != protocol.IMPULSE_MAGIC:
                return None
            return [(block.colliders[i].linear.as_tuple(),
                     block.colliders[i].contact_nodes)
                    for i in range(min(block.collider_count, protocol.MAX_COLLIDERS))]
    except (protocol.SegmentError, FileNotFoundError, OSError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--sand-vehicle", default="Sand")
    ap.add_argument("--bare-vehicle", default="Bare")
    ap.add_argument("--shm", default="/dev/shm")
    ap.add_argument("--csv", default="/home/deos/s.jois/EXECO/execosim/SIMVAL/logs/drop_test.csv")
    ap.add_argument("--settle-window", type=float, default=2.0,
                    help="seconds at the end of the run averaged to call the resting height")
    args = ap.parse_args()

    c = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", URDFBOT_PORT), timeout=30)
    print("ping:", c.call("ping"))

    rows = []
    period = 1.0 / args.rate
    t0 = time.time()
    peak_jz = 0.0
    peak_j = (0.0, 0.0, 0.0)
    while True:
        t = time.time() - t0
        if t >= args.seconds:
            break
        sz, sx, sy = pose_z_up(c, args.sand_vehicle)
        bz, bx, by = pose_z_up(c, args.bare_vehicle)
        imp = read_impulses(args.shm)
        jz = 0.0
        nodes = 0
        if imp:
            jz = sum(v[2] for v, _ in imp)
            nodes = sum(n for _, n in imp)
            mag = math.sqrt(sum(sum(v[k] for v, _ in imp) ** 2 for k in range(3)))
            if mag > peak_jz:
                peak_jz = mag
                peak_j = tuple(sum(v[k] for v, _ in imp) for k in range(3))
        rows.append((t, sz, sx, sy, bz, bx, by, jz, nodes))
        time.sleep(max(0.0, period - (time.time() - t0 - t)))

    os.makedirs(os.path.dirname(args.csv), exist_ok=True)
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "sand_z", "sand_x", "sand_y", "bare_z", "bare_x", "bare_y",
                    "impulse_z_total", "contact_nodes"])
        w.writerows(rows)

    # ⚠ AVERAGE A WINDOW, NEVER THE LAST SAMPLE. A rover at rest still jitters at the millimetre
    # scale from contact solving, and one unlucky final sample would move the verdict by more than
    # the effect being measured on a marginal case.
    tail = [r for r in rows if r[0] >= args.seconds - args.settle_window]
    if not tail:
        print("no samples in the settle window — the run was too short")
        return 1
    sand_rest = sum(r[1] for r in tail) / len(tail)
    bare_rest = sum(r[4] for r in tail) / len(tail)
    sand_span = max(r[1] for r in tail) - min(r[1] for r in tail)
    bare_span = max(r[4] for r in tail) - min(r[4] for r in tail)
    gap = sand_rest - bare_rest

    print(f"\nsamples {len(rows)} over {rows[-1][0]:.1f} s -> {args.csv}")
    print(f"  start height        {rows[0][1]:.3f} m (sand)   {rows[0][4]:.3f} m (bare)")
    print(f"  resting height      {sand_rest:.3f} m (sand)   {bare_rest:.3f} m (bare)")
    print(f"  still moving by     {sand_span*1000:5.1f} mm    {bare_span*1000:5.1f} mm"
          f"   over the last {args.settle_window:.0f} s")
    print(f"  peak sand impulse   {peak_jz:.4f} N.s  (x {peak_j[0]:+.4f} y {peak_j[1]:+.4f} "
          f"z {peak_j[2]:+.4f})")
    print(f"  drift from spawn X  {rows[-1][2]-rows[0][2]:+.3f} m (sand)  "
          f"{rows[-1][5]-rows[0][5]:+.3f} m (bare)")
    print(f"\n  GAP = {gap*1000:+.0f} mm")

    # ⚠ THE THRESHOLDS ARE A READING AID, NOT THE RESULT. The bed is 250 mm; 20 mm is about one
    # voxel at 0.025 m, which is the smallest displacement this resolution can be said to resolve.
    if bare_span > 0.02 or sand_span > 0.02:
        print("  ⚠ NOT SETTLED — one of the rovers is still moving. Re-run for longer before "
              "reading anything into the gap.")
    if gap > 0.20:
        print("  the sand HELD the rover on its surface.")
    elif gap > 0.02:
        print("  the sand gave PARTIAL support: it sank in and stopped. This is what a real "
              "bed does, and it means D10 has something to gate.")
    else:
        print("  the sand did NOT support the rover: it sank to the same height as bare ground. "
              "At this resolution the bed cannot carry the vehicle, and removing the rigid ground "
              "under it would drop the rover to the floor rather than let it float on sand.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
