#!/usr/bin/env python3
"""Read-only watcher for the D15 vehicle wire. Never writes, so it can run alongside a live sim.

⚠ WHY READ-ONLY MATTERS. vehicle_wire_check.py is a fake Unreal: it WRITES commands. Running it
while the editor is driving would make two writers race for the same block, and the vehicle would
obey whichever wrote last. This one only observes, so what it reports is what the two real ends are
actually saying to each other.
"""
import argparse, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as P

ap = argparse.ArgumentParser()
ap.add_argument("--dir", default="/dev/shm")
ap.add_argument("--vehicle", default="scout")
ap.add_argument("--seconds", type=float, default=10.0)
a = ap.parse_args()

cmd = P.Segment(a.dir, P.VEHICLE_COMMAND_SEGMENT, P.MpmVehicleCommandBlock)
pose = P.Segment(a.dir, P.VEHICLE_POSE_SEGMENT, P.MpmVehiclePoseBlock)
print(f"{'wall':>6} {'cmd.step':>9} {'cmd.joints':>10} {'targets':>28} "
      f"{'pose.step':>9} {'acked':>8} {'base x/y/z':>26}")
t0 = time.time()
last = None
while time.time() - t0 < a.seconds:
    c = P.read_consistent(cmd)
    p = P.read_consistent(pose)
    cj, tgt = 0, ""
    if c.magic == P.VEHICLE_COMMAND_MAGIC:
        for i in range(min(c.vehicle_count, P.MAX_VEHICLES)):
            if c.vehicles[i].name() == a.vehicle:
                e = c.vehicles[i]; cj = e.joint_count
                tgt = " ".join(f"{e.joints[k].target:+.2f}"
                               for k in range(min(cj, 4)))
    px = ""
    if p.magic == P.VEHICLE_POSE_MAGIC:
        for i in range(min(p.vehicle_count, P.MAX_VEHICLES)):
            if p.vehicles[i].name() == a.vehicle and p.vehicles[i].link_count:
                L = p.vehicles[i].links[0]
                px = f"{L.position.x:+8.3f} {L.position.y:+7.3f} {L.position.z:+7.3f}"
    line = (f"{time.time()-t0:6.1f} {c.step:9d} {cj:10d} {tgt:>28} "
            f"{p.sidecar_step:9d} {p.acknowledged_command_step:8d} {px:>26}")
    if line[6:] != (last or ""):
        print(line, flush=True); last = line[6:]
    time.sleep(0.25)
