"""A URDF vehicle built INSIDE the sidecar, so Newton owns both the robot and the sand.

⚠ WHY THIS EXISTS — plan D15, step 1.

Today the sidecar treats every vehicle link as a KINEMATIC collider: `mass=0`, `is_kinematic=True`,
pose pushed over the wire each frame. Newton's documentation is explicit that zero effective mass
means kinematic, and a kinematic collider has no mass for gravity to act on inside the sand solve.
So `collect_collider_impulses` returns only the reaction to MOTION — a wheel resting on settled
sand transfers no momentum and reads ~0 N. Measured 2026-08-26, Scout on the 2 x 1.6 x 0.4 m mound,
everything else identical:

    Newton's own coupled solver ....... 507 N   (0.99x the vehicle's weight)
    our kinematic-collider bridge .....   8 N   (0.015x)

That 60x is not a tuning gap. It is the difference between a body the sand solve knows the weight
of and one it does not, and no impulse scale on the wire can manufacture the missing one. Support
and sinkage — the whole point of the crater application — live on the far side of it.

The fix Newton already ships is `SolverCoupledProxy`: MuJoCo integrates the articulated robot, MPM
integrates the sand, and the proxy hands MPM each body's ARTICULATED EFFECTIVE inertia
(`coupling_eval_effective_mass_block`) rather than a link's own mass, which §11.1 correctly says
EXECOsim cannot supply. To use it, the robot has to be in the same `newton.Model` as the sand — i.e.
in this process. Hence "own vehicle".

⚠ WHAT THIS MODULE IS NOT. It does not talk to shared memory and it does not know Unreal exists.
Step 1 of D15 is exactly this: get a sidecar-owned Scout producing its 507 N through the sidecar's
own sand-spawn and solver config, with no wire in the experiment. Step 2 puts joint commands and
link poses on the wire; step 3 makes Unreal render them.

⚠ EVERY FIX IN HERE WAS PAID FOR ONCE ALREADY in PhysicsEngineDiscussion/newton_probes. The drive
signs, the DOF indexing, the effort relaxation and the torque ceiling each cost a wasted run, and
each is annotated below with what it looked like when it was wrong. This is the second home for
that knowledge, not the first; the probe remains the reference.
"""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
from dataclasses import dataclass, field

# The SIMVAL repo root: this file is <repo>/Cosys-AirSim-EXECO/mpm_sidecar/own_vehicle.py.
# Spec files state their paths relative to this so a spec is portable between checkouts.
REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

VEHICLE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vehicles")


@dataclass
class VehicleSpec:
    """Everything the sidecar needs to build and drive one URDF robot.

    ⚠ THIS IS THE SHAPE OF THE FUTURE WIRE MESSAGE. In D15 step 2 Unreal supplies the same fields
    from its settings (`UrdfPath`, `DriveJoints`, the per-joint sign convention it already writes)
    rather than from a JSON file next to this module. Keeping the file format equal to the eventual
    message means step 2 changes the transport and nothing else.
    """

    name: str
    urdf: str
    packages: dict = field(default_factory=dict)
    drive: list = field(default_factory=list)
    # ⚠ SIGNS ARE PART OF THE ROBOT, NOT AN AFTERTHOUGHT. The Scout's right wheels carry
    # rpy="3.14 0 0" on their joint origins, so the shared axis "0 -1 0" points the opposite way in
    # world. Driving all four with one sign spun the right pair backwards and the vehicle drove a
    # 0.35 m circle beside the bed for 24 s without ever touching sand — a run that looked healthy
    # in every counter. Same +/-1.0 convention the settings files already use in DriveJoints.
    drive_sign: list = field(default_factory=list)
    wheel_marker: str = "wheel"
    # ⚠ WHICH LINKS TOUCH SAND: "all" (the whole robot) or "wheels" (only links whose label
    # contains wheel_marker).
    #
    # "wheels" is what the probes ran, because it mirrors UrdfLinkPhysics in the live settings —
    # the links flagged InteractWithMPM, nothing else. But that flag exists because the BRIDGE has
    # to publish a collider per link over shared memory and the cost is per link; it is a transport
    # budget, not a statement about the vehicle. Once the robot is in this process there is no such
    # budget, and a chassis that cannot touch sand cannot belly out on a mound — which is a real
    # rover failure mode and precisely the kind of thing the crater question is about.
    #
    # ⚠ IT IS ALSO CHEAP, and the reason is worth stating because it looks expensive. `add_urdf`
    # imports the VISUAL meshes too — 215 223 vertices on the Scout's base_link alone — but those
    # are flagged VISIBLE only and never collide. The chassis's actual collision geometry is two
    # boxes. "all" therefore adds two primitives to the sand solve, not a quarter-million triangles.
    #
    # ⚠ THE 507 N REFERENCE WAS MEASURED WITH "wheels". A run meant to be compared against it must
    # say so, or the comparison silently changes two things at once.
    sand_links: str = "all"
    start_z: float = 0.3
    rest_z: float = 0.0
    wheel_radius: float = 0.1
    # ⚠ WHEEL-ON-SAND FRICTION IS THE CLIMB LIMIT and it is a SHAPE property, separate from the
    # sand's internal friction. A heap rests at atan(mu_sand); a vehicle climbs at most
    # atan(mu_wheel). Newton defaults both to 0.5, which places every vehicle exactly at marginal
    # stability on the slope the sand builds by itself. Measured on a 52 kg Scout, 2 x 1.6 x 0.4 m
    # mound, all else identical: mu 0.5 stalls at the toe (+0.082 m), mu 0.8 creeps 40 % up
    # (+0.251 m), mu 1.2 crests and drives off the far side (+0.325 m).
    wheel_friction: float = 1.2
    wheel_kd: float = 0.3
    # ⚠ THE URDF'S effort="1000.0" IS A PLACEHOLDER AND TAKING IT LITERALLY LAUNCHED A ROVER 31 m
    # INTO THE AIR. A velocity target applies kd * (target - qd); with a 6 rad/s error and no
    # meaningful ceiling that is hundreds of N.m on a wheel whose inertia is ~3e-4 kg.m^2. The
    # vehicle left the ground the instant drive engaged, a full 0.85 m before the sand began.
    wheel_max_torque: float = 1.0
    body_friction: float = 0.5
    mass_scale: float = 1.0

    @staticmethod
    def load(ref: str) -> "VehicleSpec":
        """Accept either a bare name ('scout') or a path to a spec JSON."""
        path = ref
        if not os.path.exists(path):
            path = os.path.join(VEHICLE_DIR, ref if ref.endswith(".json") else ref + ".json")
        if not os.path.exists(path):
            available = sorted(f[:-5] for f in os.listdir(VEHICLE_DIR) if f.endswith(".json")) \
                if os.path.isdir(VEHICLE_DIR) else []
            raise SystemExit(f"no vehicle spec '{ref}'. Known: {', '.join(available) or '(none)'}, "
                             f"or give a path to a .json.")
        with open(path) as f:
            data = json.load(f)
        known = {f_.name for f_ in VehicleSpec.__dataclass_fields__.values()}
        # ⚠ REFUSE AN UNKNOWN KEY rather than ignoring it. A spec that says "wheel_mu" when the
        # field is "wheel_friction" would otherwise run at Newton's 0.5 default and stall at the
        # toe of the mound, and the file would look like it had asked for 1.2.
        unknown = set(data) - known
        if unknown:
            raise SystemExit(f"{path}: unknown key(s) {sorted(unknown)}; known keys are "
                             f"{sorted(known)}")
        data.setdefault("name", os.path.splitext(os.path.basename(path))[0])
        data["urdf"] = _resolve(data["urdf"])
        data["packages"] = {k: _resolve(v) for k, v in data.get("packages", {}).items()}
        spec = VehicleSpec(**data)
        if not spec.drive_sign:
            spec.drive_sign = [1.0] * len(spec.drive)
        if len(spec.drive_sign) != len(spec.drive):
            raise SystemExit(f"{path}: {len(spec.drive)} drive joints but "
                             f"{len(spec.drive_sign)} signs")
        if not os.path.exists(spec.urdf):
            raise SystemExit(f"{path}: urdf not found: {spec.urdf}")
        return spec


def _resolve(p: str) -> str:
    """Spec paths are relative to the SIMVAL repo root unless they are already absolute."""
    return p if os.path.isabs(p) else os.path.normpath(os.path.join(REPO, p))


def prepared_urdf(spec: VehicleSpec) -> str:
    """Absolute mesh paths, no Gazebo plugins.

    ⚠ `package://` needs resolve-robotics-uri-py, which the newtonmpm env does not have, and the
    <gazebo> blocks reference .so plugins that mean nothing outside Gazebo. Rewriting beats
    installing a resolver for a couple of substitutions. Same treatment as the probe's.
    """
    xml = open(spec.urdf).read()
    for prefix, root in spec.packages.items():
        xml = xml.replace(prefix, root.rstrip("/") + "/")
    xml = re.sub(r"<gazebo[\s\S]*?</gazebo>", "", xml)
    fd, path = tempfile.mkstemp(suffix=".urdf", prefix=f"sidecar_{spec.name}_")
    with os.fdopen(fd, "w") as f:
        f.write(xml)
    return path


class OwnVehicle:
    """One URDF robot inside the sidecar's own Newton model."""

    def __init__(self, spec: VehicleSpec):
        self.spec = spec
        self.body_start = 0
        self.body_end = 0
        self.drive_dofs: list = []
        self.wheel_bodies: list = []
        self.bodies: list = []
        self.mass = 0.0
        self._urdf_tmp = None

    # ---- build --------------------------------------------------------------------------

    def add_to(self, builder, x: float, y: float, z: float, yaw_deg: float = 0.0) -> None:
        """Add the robot to `builder` at the given pose. Call BEFORE the sand is spawned."""
        import warp as wp

        import newton

        spec = self.spec
        self._urdf_tmp = prepared_urdf(spec)

        self.body_start = builder.body_count
        half = math.radians(yaw_deg) * 0.5
        builder.add_urdf(
            self._urdf_tmp,
            xform=wp.transform(wp.vec3(x, y, z),
                               wp.quat(0.0, 0.0, math.sin(half), math.cos(half))),
            floating=True,
            enable_self_collisions=False,
            collapse_fixed_joints=True,
            ignore_inertial_definitions=False,
        )
        self.body_end = builder.body_count
        self.bodies = list(range(self.body_start, self.body_end))

        if spec.sand_links not in ("all", "wheels"):
            raise SystemExit(f"sand_links must be 'all' or 'wheels', not {spec.sand_links!r}")

        # Wheels always get the wheel friction; whether anything ELSE touches sand is sand_links.
        wheels = 0
        muted = 0
        for b in self.bodies:
            if spec.wheel_marker in builder.body_label[b]:
                wheels += 1
                for sh in builder.body_shapes[b]:
                    builder.shape_material_mu[sh] = spec.wheel_friction
                continue
            if spec.sand_links == "wheels":
                muted += 1
                for sh in builder.body_shapes[b]:
                    builder.shape_flags[sh] = (builder.shape_flags[sh]
                                               & ~newton.ShapeFlags.COLLIDE_PARTICLES)
        if wheels == 0:
            raise SystemExit(
                f"no body label contains the wheel marker '{spec.wheel_marker}'; labels are "
                f"{[builder.body_label[b] for b in self.bodies]}")
        if spec.sand_links == "all":
            print(f"vehicle '{spec.name}': {len(self.bodies)} bodies, ALL of them touch sand "
                  f"({wheels} wheels at mu {spec.wheel_friction}, the rest at "
                  f"mu {spec.body_friction})")
        else:
            print(f"vehicle '{spec.name}': {len(self.bodies)} bodies, {wheels} of them wheels that "
                  f"touch sand (mu {spec.wheel_friction}); {muted} non-wheel body/bodies muted")

        # ⚠ effort="0" IS URDF FOR "PASSIVE", NOT FOR "CLAMPED TO ZERO". A rocker-bogie's rocker
        # joints declare zero effort and zero velocity because nothing drives them; Newton forwards
        # that as MuJoCo actfrcrange = [-0, 0], which MuJoCo rejects outright ("actfrcrange[0]
        # should be smaller than actfrcrange[1]") and the whole build fails. Lifting the LIMIT
        # actuates nothing — target_ke and target_kd stay 0, so these joints remain free to swing,
        # which is exactly what a rocker-bogie must do to keep its wheels on uneven ground.
        relaxed = []
        for d in range(builder.joint_dof_count):
            if builder.joint_effort_limit[d] <= 0.0:
                builder.joint_effort_limit[d] = 1.0e4
                relaxed.append(d)
            if builder.joint_velocity_limit[d] <= 0.0:
                builder.joint_velocity_limit[d] = 1.0e3
        if relaxed:
            print(f"  relaxed zero effort/velocity limits on {len(relaxed)} passive dof(s)")

        self.drive_dofs = self._find_drive_dofs(builder)
        for dof in self.drive_dofs:
            builder.joint_target_mode[dof] = newton.JointTargetMode.VELOCITY
            builder.joint_target_ke[dof] = 0.0
            builder.joint_target_kd[dof] = spec.wheel_kd
            builder.joint_effort_limit[dof] = spec.wheel_max_torque
            builder.joint_velocity_limit[dof] = 30.0

        # ⚠ SCALE MASS AND INERTIA TOGETHER, on the builder, before finalize. Halving mass alone
        # would leave the rocker-bogie's rotational response unchanged, so the "lighter rover"
        # would be a body that cannot exist. finalize() derives the inverse quantities from these.
        if spec.mass_scale != 1.0:
            for b in self.bodies:
                builder.body_mass[b] *= spec.mass_scale
                builder.body_inertia[b] = builder.body_inertia[b] * spec.mass_scale
            print(f"  mass and inertia scaled by {spec.mass_scale}x")

    def _find_drive_dofs(self, builder) -> list:
        # ⚠ JOINT INDEX IS NOT DOF INDEX. `joint_target_*` is addressed by DOF, and a floating base
        # contributes six DOFs before any wheel does, so using the joint index writes the target
        # into the base's linear velocity instead of the wheel's spin. `joint_qd_start` is the map.
        dofs = []
        for name in self.spec.drive:
            idx = next((i for i, l in enumerate(builder.joint_label) if l.endswith(name)), None)
            if idx is None:
                raise SystemExit(f"drive joint '{name}' not found; joint labels are "
                                 f"{builder.joint_label}")
            dofs.append(int(builder.joint_qd_start[idx]))
        print(f"  drive dofs {dofs} of {builder.joint_dof_count}")
        return dofs

    def bind(self, model) -> None:
        """Resolve wheel bodies and total mass on the finalised model."""
        import numpy as np

        # ⚠ body_label, NOT body_key — Model has no body_key, and a hasattr() fallback here once
        # silently selected ALL the rover's bodies including the chassis, which carries no
        # collision geometry at all. Sand fills those freely, so the penetration metric read
        # 1.2-1.4 and looked like sand streaming through solid wheels. No fallback: if the wheels
        # cannot be identified this must fail loudly rather than measure the chassis.
        self.wheel_bodies = [b for b in self.bodies
                             if self.spec.wheel_marker in model.body_label[b]]
        if not self.wheel_bodies:
            raise SystemExit(f"no finalised body label contains '{self.spec.wheel_marker}'")
        self.mass = float(np.sum(model.body_mass.numpy()[self.body_start:self.body_end]))
        print(f"  total vehicle mass {self.mass:.3f} kg (weight {self.mass * 9.81:.1f} N), "
              f"{len(self.wheel_bodies)} wheel bodies")

    # ---- driving ------------------------------------------------------------------------

    def drive(self, control, rad_s: float) -> None:
        """Set every drive wheel's velocity target, with each wheel's own sign applied."""
        tv = control.joint_target_qd.numpy()
        for dof, sign in zip(self.drive_dofs, self.spec.drive_sign):
            tv[dof] = rad_s * sign
        control.joint_target_qd.assign(tv)

    # ---- sand culling -------------------------------------------------------------------

    def collider_volumes(self, builder) -> list:
        """World-space descriptions of the vehicle's shapes, for the sand-spawn cull.

        ⚠ THE BED MUST NOT BE BUILT THROUGH THE ROBOT. `add_particle_grid` fills the declared box
        regardless of what occupies it, and MPM's collision only stops particles CROSSING a
        boundary — it never evicts ones that started inside. Those grains stay put for the life of
        the run, moving with the wheel, and the wheel reads as transparent.

        Returned in the same dict shape `Sidecar._points_in_shape` already consumes, so the cull
        is the existing one rather than a second implementation that can disagree with it.
        """
        import numpy as np
        import warp as wp

        from newton import GeoType, ShapeFlags

        import protocol as P

        # Map Newton's geometry ids onto the wire's, because the cull is written against the wire.
        # Anything not listed falls through to the hull path, which uses a bounding sphere and so
        # over-culls rather than under-culls — a few extra grains next to a wheel settle back in
        # during the presettle, whereas grains left inside one never leave.
        KIND = {
            GeoType.SPHERE: P.SHAPE_SPHERE,
            GeoType.BOX: P.SHAPE_BOX,
            GeoType.CAPSULE: P.SHAPE_CAPSULE,
            GeoType.CYLINDER: P.SHAPE_CAPSULE,
        }

        out = []
        for s in range(len(builder.shape_type)):
            body = builder.shape_body[s]
            if body < self.body_start or body >= self.body_end:
                continue
            # ⚠ CULL AGAINST THE SHAPES THAT ARE ACTUALLY BOUNDARIES, WHICH IS NOT ALL OF THEM.
            #
            # `add_urdf` imports the VISUAL meshes as well as the collision primitives, and for the
            # Scout that is 23 shapes of which exactly 4 — the wheel cylinders — carry
            # COLLIDE_PARTICLES. The visual meshes are flagged VISIBLE only. Culling against them
            # removes sand that nothing will ever push, and the first run of this method did
            # exactly that: it reported "55400 culled from inside 23 collider shape(s), 17.97 % of
            # the bed" for a vehicle parked 1.2 m clear of the sand, i.e. it hollowed out a fifth
            # of the experiment before it began.
            #
            # ⚠ AND THE SIZE WAS FICTION TOO. A MESH's `shape_scale` is a SCALE, (1,1,1) here, not
            # an extent — so the bounding-sphere fallback read radius 1.0 m per visual mesh. Both
            # halves are fixed by asking the same question the solver asks: does this shape collide
            # with particles at all, and if so what is its real geometry.
            if not (int(builder.shape_flags[s]) & int(ShapeFlags.COLLIDE_PARTICLES)):
                continue
            world = wp.transform_multiply(wp.transform(*builder.body_q[body]),
                                          wp.transform(*builder.shape_transform[s]))
            pos = np.array(wp.transform_get_translation(world), dtype=float)
            quat = np.array(wp.transform_get_rotation(world), dtype=float)   # xyzw
            scale = np.array(builder.shape_scale[s], dtype=float)
            gt = GeoType(builder.shape_type[s])
            kind = KIND.get(gt, P.SHAPE_CONVEX_HULL)

            if kind != P.SHAPE_CONVEX_HULL:
                # scale means (radius, _, _) for a sphere, (radius, half_height, _) for a
                # capsule/cylinder, and (hx, hy, hz) for a box — see ModelBuilder.add_shape_*.
                out.append(dict(kind=kind, hull_bound=0.0, pos=pos, quat=quat,
                                radius=float(scale[0]), half_length=float(scale[1]),
                                half_extents=scale.copy()))
                continue

            # ⚠ radius / half_length / half_extents ARE ZEROED ON THIS PATH. They mean nothing for
            # a mesh, and `_points_in_shape`'s hull fallback maxes over all four — so leaving the
            # (1,1,1) scale in place is what produced the 1 m bounding sphere above.
            src = builder.shape_source[s]
            verts = getattr(src, "vertices", None)
            bound = 0.0
            if verts is not None and len(verts):
                bound = float(np.linalg.norm(np.asarray(verts, dtype=float) * scale, axis=1).max())
            else:
                # ⚠ NAMED, NOT SKIPPED IN SILENCE. A particle-colliding shape that contributed no
                # cull volume leaves that part of the robot full of sand for the life of the run,
                # and a transparent-looking wheel reads as a physics result.
                print(f"  ⚠ shape {s} ({gt.name}) on body {builder.body_label[body]} collides with "
                      f"particles but has no vertices to bound; NOT culled")
            out.append(dict(kind=P.SHAPE_CONVEX_HULL, hull_bound=bound, pos=pos, quat=quat,
                            radius=0.0, half_length=0.0, half_extents=np.zeros(3)))

        print(f"  {len(out)} shape(s) collide with sand and will be culled out of the bed")
        return out

    def close(self) -> None:
        if self._urdf_tmp and os.path.exists(self._urdf_tmp):
            os.unlink(self._urdf_tmp)
            self._urdf_tmp = None
