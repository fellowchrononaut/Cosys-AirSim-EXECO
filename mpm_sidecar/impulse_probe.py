#!/usr/bin/env python3
"""Measure the sand's reaction on the rover from the live shared-memory impulse block.

⚠ ONE SAMPLE PER SOLVE, NOT PER READ. The reader polls faster than the sidecar solves, so counting
reads weights a slow step several times over and reports several times the force. Dedupe on
`sidecar_step` before reducing.

Impulses are N.s accumulated over `mpm_dt`; force is J/dt. The sign already IS the reaction on the
collider (Newton returns `-cell_volume * impulse_field`), so it is summed, never negated again.
"""
import argparse, os, statistics, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as P


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--weight", type=float, default=16.5, help="rover weight in N, for the ratio")
    ap.add_argument("--dir", default="/dev/shm")
    ap.add_argument("--csv", default=None)
    a = ap.parse_args()

    seg = P.Segment(a.dir, P.IMPULSE_SEGMENT, P.MpmImpulseBlock)
    seen = {}
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        b = P.read_consistent(seg)
        if b and b.magic == P.IMPULSE_MAGIC and b.collider_count:
            n = min(b.collider_count, P.MAX_COLLIDERS)
            seen[b.sidecar_step] = (b.mpm_dt,
                                    [(b.colliders[i].linear.as_tuple(),
                                      b.colliders[i].contact_nodes) for i in range(n)])
        time.sleep(0.02)
    seg.close()

    if not seen:
        print("no impulse data — is the sidecar running with --two-way?")
        return 1

    rows = []
    for step in sorted(seen):
        dt, per = seen[step]
        f = [sum(v[k] for v, _ in per) / dt for k in range(3)]
        rows.append((step, f[0], f[1], f[2], sum(n for _, n in per)))

    # ⚠ THIS IS AN UPPER BOUND, NOT THE APPLIED FORCE. Dividing by mpm_dt assumes the simulator
    # bills the impulse over one MPM step. It does not: it divides by the interval that ACTUALLY
    # elapsed between two acknowledged solves ("force-hold"), which is longer whenever the sidecar
    # runs behind the sim — 0.048 s against an 0.0083 s mpm_dt in a live run, a factor of 5.8. Read
    # the simulator's own "two-way: ... peak |F|" log line for what the rover really feels; use
    # this for the shape of the signal and as a ceiling.
    fz = sorted(r[3] for r in rows)
    mag = sorted((r[1] ** 2 + r[2] ** 2 + r[3] ** 2) ** 0.5 for r in rows)
    W = a.weight
    print(f"{len(rows)} distinct sidecar steps over {a.seconds:.0f} s")
    print(f"  ⚠ forces below are J/mpm_dt = an UPPER BOUND; the sim divides by the real elapsed "
          f"interval (often ~6x longer). Its own log line is the applied force.")
    print(f"  vertical force Fz : median {fz[len(fz)//2]:8.1f} N   min {fz[0]:8.1f}   "
          f"max {fz[-1]:8.1f}")
    print(f"  total |F|         : median {mag[len(mag)//2]:8.1f} N   max {mag[-1]:8.1f}")
    print(f"  vs rover weight   : median {fz[len(fz)//2]/W:+.2f}x   peak {mag[-1]/W:.1f}x  "
          f"(weight {W:.1f} N)")
    print(f"  contact nodes     : {min(r[4] for r in rows)}..{max(r[4] for r in rows)}")
    if a.csv:
        import csv as _csv
        with open(a.csv, "w", newline="") as f:
            w = _csv.writer(f); w.writerow(["step", "Fx", "Fy", "Fz", "nodes"]); w.writerows(rows)
        print(f"  -> {a.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
