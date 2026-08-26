# Vehicle specs for `--own-vehicle` (D15 step 1)

One JSON file per robot. `sidecar.py --own-vehicle scout` resolves a bare name against this
directory; a path to a `.json` anywhere also works.

⚠ **This is the shape of the future wire message.** In D15 step 2 Unreal supplies the same fields
from its settings — `UrdfPath`, `DriveJoints`, and the per-joint sign convention it already
writes — rather than from a file next to the sidecar. Keeping the file format equal to the eventual
message means step 2 changes the transport and nothing else.

Paths are relative to the SIMVAL repo root unless absolute. An unknown key is a hard error, not a
warning: a spec that says `wheel_mu` when the field is `wheel_friction` would otherwise run at
Newton's 0.5 default and stall at the toe of the mound while the file looked like it had asked
for 1.2.

## The fields that decide whether anything happens

| field | why it matters |
|---|---|
| `drive_sign` | The Scout's right wheels carry `rpy="3.14 0 0"`, so the shared axis `0 -1 0` points the opposite way in world. One sign for all four spun the right pair backwards and drove a 0.35 m circle beside the bed for 24 s without touching sand. |
| `wheel_friction` | The **climb limit**. A heap rests at `atan(mu_sand)`; a vehicle climbs at most `atan(mu_wheel)`. Newton defaults both to 0.5, which puts every vehicle exactly at marginal stability on the slope the sand builds by itself. Measured on this Scout, same bed, all else identical: **0.5** stalls at the toe (+0.082 m), **0.8** creeps 40 % up (+0.251 m), **1.2** crests and drives off the far side (+0.325 m). |
| `wheel_max_torque` | The URDF's `effort="1000.0"` is a placeholder. Taking it literally launched an ExoMy 31 m into the air the instant drive engaged — a velocity target applies `kd * (target - qd)`, and with a 6 rad/s error that is hundreds of N·m on a wheel whose inertia is ~3e-4 kg·m². |
| `sand_links` | `"all"` (default) or `"wheels"`. See below. |

## `sand_links`

`"wheels"` mirrors `UrdfLinkPhysics` / `InteractWithMPM` in the live settings. But that flag exists
because the **bridge** publishes one collider per link over shared memory and pays per link — it is
a transport budget, not a statement about the vehicle. Once the robot is in the sidecar's own
process there is no such budget, and a chassis that cannot touch sand cannot belly out on a mound,
which is a real rover failure mode and exactly the kind of thing the crater question is about. So
`"all"` is the default here.

⚠ It is also cheaper than it looks. `add_urdf` imports the **visual** meshes as well as the
collision primitives — 215 223 vertices on the Scout's `base_link` alone — but those are flagged
`VISIBLE` only and never collide. The chassis's actual collision geometry is two boxes. `"all"`
adds two primitives to the sand solve, not a quarter-million triangles.

⚠ **The 507 N reference in `../../PhysicsEngineDiscussion/newton_probes/` was measured with
`"wheels"`.** A run meant to be compared against that number must pass
`--own-vehicle-sand-links wheels`, or the comparison silently changes two things at once.

## `wheel_max_torque` on `scout.json` is 1.0 N·m and that is a placeholder

It is the value every `scout_*` result in `newton_probes/results/` was produced with, so it is kept
for comparability. It is **not** a Scout 2.0's real wheel torque, which is two orders of magnitude
higher. Four wheels at 1 N·m on 0.165 m wheels is 24 N of tractive force under a 510 N vehicle —
enough to roll on flat ground and to crest this mound at mu 1.2, but it is a number nobody
measured off the real vehicle. Revisit it before any claim about what a Scout *can* climb.
