#!/usr/bin/env python3
"""Hold ONE sphere of a chosen radius fully buried in sand, so penetration can be measured
against collider size and coupling mode without a simulator.

⚠ WHY IT EXISTS. `sidecar_sand_circle.mp4` shows sand cleanly excluded from a 0.15 m ball, while a
0.045 m ExoMy wheel in the live sim has sand inside it at ~ambient density. Two things differ at
once — the collider's size in VOXELS, and whether the collider is kinematic to the MPM or coupled
with a real mass. This fixture changes one at a time.

The sphere is held STILL and fully buried: a stationary solid must simply contain no sand, which
is a cleaner question than what a moving one does to a surface.
"""
import argparse, sys, time
import protocol as P

ap = argparse.ArgumentParser()
ap.add_argument("--radius", type=float, default=0.15)
ap.add_argument("--mass", type=float, default=0.05)
ap.add_argument("--seconds", type=float, default=60.0)
ap.add_argument("--depth", type=float, default=0.30, help="sand bed depth; sphere sits mid-bed")
ap.add_argument("--sweep", type=float, default=0.0,
                help="m/s in +x, starting clear of the bed at x=-0.5. 0 = held still.")
a = ap.parse_args()

reg = P.Segment("/dev/shm", P.REGISTRY_SEGMENT, P.MpmRegistryBlock, create=True)
sta = P.Segment("/dev/shm", P.STATE_SEGMENT, P.MpmStateBlock, create=True)

b = reg.block
b.sequence |= 1
b.magic, b.version, b.collider_count = P.REGISTRY_MAGIC, P.PROTOCOL_VERSION, 1
b.stamp.world_id, b.stamp.world_revision = 7, 1
b.stamp.manifest_revision, b.stamp.reset_epoch = 1, 1
r = b.region
r.terrain_id = b"pen"
r.center.x, r.center.y, r.center.z = 0.0, 0.0, a.depth / 2
r.half_extent.x, r.half_extent.y, r.half_extent.z = 0.30, 0.30, a.depth / 2
r.rigid_ticks_per_mpm_step, r.valid = 6, 1
c = b.colliders[0]
c.stable_id, c.shape_count, c.role, c.mass, c.friction = b"Fixture/ball", 1, 1, a.mass, 0.6
sh = c.shapes[0]
sh.kind, sh.radius = 0, a.radius          # 0 = Sphere
sh.orientation.w = 1.0
b.sequence += 1

print(f"fixture: sphere r={a.radius} m, mass={a.mass} kg, buried at z={a.depth/2} "
      f"in a {a.depth} m bed", flush=True)

t0 = time.time()
step = 0
while time.time() - t0 < a.seconds:
    s = sta.block
    s.sequence |= 1
    s.magic, s.version, s.collider_count = P.STATE_MAGIC, P.PROTOCOL_VERSION, 1
    s.stamp.world_id, s.stamp.world_revision = 7, 1
    s.stamp.manifest_revision, s.stamp.reset_epoch = 1, 1
    s.step, s.simulation_time = step, step / 120.0
    col = s.colliders[0]
    # ⚠ A HELD, BURIED SPHERE MEASURES THE INITIAL CONDITION, NOT THE BOUNDARY. The bed is spawned
    # as a filled box, so particles are born INSIDE the collider and simply stay there — the ratio
    # then reads ~1.4 for a 6-voxel ball, which says nothing about whether the surface excludes
    # sand. Sweeping in from OUTSIDE is what sidecar_sand_circle.mp4 does, and it asks the real
    # question: can sand the collider MOVES INTO get through it?
    x = 0.0 if a.sweep <= 0 else -0.5 + a.sweep * (step / 120.0)
    col.position.x, col.position.y, col.position.z = x, 0.0, a.depth / 2
    col.orientation.w = 1.0
    s.sequence += 1
    step += 1
    time.sleep(1 / 120.0)
reg.close(); sta.close()
