"""The Newton MPM deformable-terrain sidecar for EXECOsim.

⚠ WHAT THIS IS. A separate process that reads collider poses out of shared memory, pushes them
through an implicit-MPM sand patch, and writes back an acknowledgement. It is plan §M2: **one-way**.
The sand is deformed by the robot; the robot feels nothing back.

⚠ WHY OUT OF PROCESS. Newton is 769 Python files with no C API, no headers and no shared library
(urdf_physics/NEWTON-ASSESSMENT.md). It cannot be linked into the Unreal process. The boundary is
forced, not chosen.

⚠ WHY ONE-WAY IS NOT MERELY CAUTIOUS. Two-way needs each collider's articulated
operational-space effective inertia, and plan §11.1 records that EXECOsim cannot supply it — a
URDF link's own inertia is not that quantity. Newton asks for it directly as `body_inv_inertia`
in `setup_collider`, and its own documentation is explicit that a collider is kinematic exactly
when its effective mass is zero. So this sidecar passes `body_mass = zeros`, which is Newton's
documented one-way mode, and there is deliberately no reaction path here to be mistaken for a
working one.

Run it alongside a simulator that has the MPM link enabled:

    ~/miniconda3/envs/newtonmpm/bin/python mpm_sidecar/sidecar.py --dir /dev/shm

Add --headless to run without a window, or --render-to out.mp4 to record (this machine's X session
has no hardware GLX, so an interactive viewer is not useful — see m0/RESULTS.md).
"""

from __future__ import annotations

import argparse
import ctypes
import math
import gc
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ⚠ PREFER THE VENDORED NEWTON, and say which one was loaded. Before Newton was vendored into this
# repository it resolved through `site-packages/newton.pth`, which pointed at a checkout OUTSIDE any
# repo — so which solver ran depended on the machine, not on the checkout, and nothing said so. A
# sand result is only attributable if the solver that produced it is.
_VENDORED_NEWTON = os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "external", "newton"))
if os.path.isdir(os.path.join(_VENDORED_NEWTON, "newton")):
    sys.path.insert(0, _VENDORED_NEWTON)

import protocol as P  # noqa: E402


def report_newton_provenance() -> None:
    """Print where `newton` and `warp` were actually loaded from. Cheap, and it settles arguments."""
    import newton
    import warp
    newton_dir = os.path.dirname(os.path.dirname(os.path.abspath(newton.__file__)))
    vendored = os.path.normpath(newton_dir) == os.path.normpath(_VENDORED_NEWTON)
    print(f"newton: {os.path.dirname(newton.__file__)}"
          f"  [{'VENDORED' if vendored else '⚠ NOT the vendored copy'}]")
    print(f"warp:   {warp.config.version}")


# Faults reported to the simulator through MpmStatusBlock.fault. Numeric values are part of the
# wire contract in spirit: the sim prints them, so they should not be renumbered casually.
FAULT_NONE = 0
FAULT_PROTOCOL = 1
FAULT_SOLVER = 2
FAULT_TOPOLOGY = 3


class Sidecar:
    def __init__(self, args):
        self.args = args
        self.directory = args.dir

        self.registry_seg = None
        self.state_seg = None
        self.status_seg = None
        self.particle_seg = None

        self.model = None
        self._collider_bodies: list[int] = []
        self._collider_masses: list[float] = []
        self._collider_volumes: list = []
        self.impulse_seg = None
        self._impulse_announced = False
        self.solver = None
        self.state_0 = None
        self.state_1 = None
        self.viewer = None

        self.body_index: dict[str, int] = {}
        self.current_stamp: tuple[int, int, int, int] | None = None
        self.sidecar_step = 0
        self.sidecar_time = 0.0
        # ⚠ A SENTINEL IS NEEDED, NOT JUST ZERO. Initialising this to 0 makes
        # `state.step <= acknowledged_step` true for step 0, so the very FIRST state the simulator
        # publishes can never be consumed: the sidecar idles forever at 0 % CPU while reporting
        # itself healthy, which is the worst shape of failure. Caught on the first live run. One
        # integer cannot distinguish "acknowledged step 0" from "acknowledged nothing", so the flag
        # carries that distinction.
        self.acknowledged_step = 0
        self.has_consumed = False
        self.last_solve_seconds = 0.0
        self.particle_count = 0
        self.ground_z = 0.0
        self._region = None
        self._particle_radius = 0.01
        self._particles_announced = False
        self._jump_warned = False
        self._recorded_states = []
        self._recorded_registry = None
        self._initial_state = None
        self._fall_warned = False

    # ---- shared memory -------------------------------------------------------------------

    def attach(self) -> None:
        """Open the three segments. The simulator creates them; this process never does.

        ⚠ Creating them here would let a sidecar started first present an empty registry that the
        simulator then overwrites, and the window between is a sidecar simulating nothing while
        reporting healthy.
        """
        self.registry_seg = P.Segment(self.directory, P.REGISTRY_SEGMENT, P.MpmRegistryBlock)
        self.state_seg = P.Segment(self.directory, P.STATE_SEGMENT, P.MpmStateBlock)
        self.status_seg = P.Segment(self.directory, P.STATUS_SEGMENT, P.MpmStatusBlock,
                                    create=True)
        # ⚠ ADOPTING A SEGMENT MEANS INHERITING ITS CONTENTS. The Segment above deliberately does
        # NOT truncate an existing file — truncating one the simulator had mmapped blinded it for
        # 56 s. But a file left by a DEAD sidecar still holds that run's acknowledgement, particle
        # count and stamp, and the instant we set the magic below the simulator starts believing
        # them. Measured 2026-08-26: it read ack=21493 from a dead process, then saw our real first
        # status arrive with a lower step, concluded a different sidecar had started, and forced
        # three global resets in a row.
        #
        # ⚠ ZERO THE PAYLOAD BEFORE THE MAGIC, never after. The magic is what makes the block
        # readable, so writing it first opens a window where the simulator sees valid-looking stale
        # numbers. This is the same ordering discipline as the seqlock.
        block = self.status_seg.block
        block.fault = 0
        block.acknowledged_step = 0
        block.sidecar_step = 0
        block.sidecar_time = 0.0
        block.last_solve_seconds = 0.0
        block.particle_count = 0
        block.message = b""
        block.stamp = P.WireWorldStamp()
        block.version = P.PROTOCOL_VERSION
        block.magic = P.STATUS_MAGIC

        # ⚠ The sidecar CREATES the particle segment, unlike the collider ones which the simulator
        # owns. Direction of ownership follows direction of data: whoever writes, creates.
        self.particle_seg = P.Segment(self.directory, P.PARTICLE_SEGMENT, P.MpmParticleBlock,
                                      create=True)
        # Same reasoning as the status block: an adopted particle segment still holds the previous
        # run's sand, and a renderer would draw it as though it were ours.
        pblock = self.particle_seg.block
        pblock.particle_count = 0
        pblock.total_particles = 0
        pblock.sidecar_step = 0
        pblock.sidecar_time = 0.0
        pblock.version = P.PROTOCOL_VERSION
        pblock.magic = P.PARTICLE_MAGIC

        # The sand's reaction back onto the colliders. Written by us, so created by us — direction
        # of ownership follows direction of data, like the particle segment.
        if self.args.two_way:
            self.impulse_seg = P.Segment(self.directory, P.IMPULSE_SEGMENT, P.MpmImpulseBlock,
                                         create=True)
            iblock = self.impulse_seg.block
            iblock.collider_count = 0
            iblock.sidecar_step = 0
            iblock.sidecar_time = 0.0
            iblock.mpm_dt = 0.0
            iblock.stamp = P.WireWorldStamp()
            iblock.version = P.PROTOCOL_VERSION
            iblock.magic = P.IMPULSE_MAGIC
        print(f"attached to {self.directory}")

    def publish_status(self, fault: int = FAULT_NONE, message: str = "healthy") -> None:
        """Write the acknowledgement the simulator uses to decide whether we are alive.

        ⚠ THE SEQLOCK MATTERS EVEN HERE. The sim reads this while we write it; an odd sequence is
        what tells it to retry rather than consume a half-written block whose stamp belongs to one
        epoch and whose step belongs to another.
        """
        block = self.status_seg.block
        block.sequence |= 1
        block.fault = fault
        block.acknowledged_step = self.acknowledged_step
        block.sidecar_step = self.sidecar_step
        block.sidecar_time = self.sidecar_time
        block.last_solve_seconds = self.last_solve_seconds
        block.particle_count = self.particle_count
        block.message = message.encode("utf-8")[:255]
        block.sequence = (block.sequence + 1) & ~1

    # ---- model construction --------------------------------------------------------------

    def build(self, registry) -> None:
        """Build a Newton model: one kinematic body per collider, plus the sand patch."""
        report_newton_provenance()
        self._region = registry.region
        import numpy as np
        import warp as wp

        import newton
        from newton.solvers import SolverImplicitMPM

        self._collider_bodies = []
        self._collider_masses = []
        self._collider_volumes = []
        frictions_used = []
        builder = newton.ModelBuilder()
        builder.gravity = float(self.args.gravity)

        # ⚠ Read the CURRENT collider poses first. The simulator has been running since before this
        # process started, so its robots are wherever the operator left them — building at the
        # origin and correcting afterwards is a teleport through the sand, not an initialisation.
        initial_pose = {}
        try:
            # ⚠ In REPLAY there is no shared memory at all — the first recorded sample is the
            # initial pose, and it is supplied by the caller. Reading `self.state_seg` here without
            # this guard raised "'NoneType' object has no attribute 'block'" and failed the build.
            live = self._initial_state if self._initial_state is not None else (
                P.read_consistent(self.state_seg) if self.state_seg is not None else None)
            if live is not None and live.magic == P.STATE_MAGIC:
                for i in range(min(live.collider_count, registry.collider_count)):
                    initial_pose[registry.colliders[i].name()] = (
                        live.colliders[i].position.as_tuple(),
                        live.colliders[i].orientation.as_xyzw())
                if initial_pose:
                    print(f"placing {len(initial_pose)} colliders at their live poses")
        except P.SegmentError:
            # No state yet is legitimate — the simulator may not have ticked. Fall back to identity
            # and let the jump warning say so if it matters.
            pass

        self.body_index.clear()
        count = registry.collider_count
        if count == 0:
            raise RuntimeError(
                "the registry is empty — the simulator published no MPM colliders. Check that "
                "UrdfLinkPhysics selects at least one link with InteractWithMPM.")

        for i in range(count):
            entry = registry.colliders[i]
            name = entry.name()

            # ⚠ BUILD EACH BODY WHERE IT ACTUALLY IS, not at the origin.
            #
            # `transform_identity()` put every collider at (0,0,0) and the first state update then
            # teleported it to its real position — 124 m in a live run, because the solver frame's
            # origin is the UNREAL WORLD origin. Six wheels swept through the sand patch in a
            # single step before the operator touched a key, punching the bed at t=0. Caught by the
            # jump warning, which reported exactly 124.293 m.
            #
            # ⚠ mass=0 and is_kinematic: this body is driven from outside and must never be
            # integrated by Newton. The pose we push each frame IS its motion.
            start = initial_pose.get(name)
            xform = (wp.transform(wp.vec3(*start[0]), wp.quat(*start[1]))
                     if start is not None else wp.transform_identity())
            body = builder.add_body(xform=xform, mass=0.0, is_kinematic=True, label=name)
            self.body_index[name] = body
            # ⚠ REGISTRY ORDER, recorded rather than assumed. The wire's colliders[i] is the i-th
            # registry entry, and Newton's impulses come back keyed by BODY index. These happen to
            # coincide today because bodies are added in this loop and nothing else adds one — but
            # attributing sand force to the wrong wheel is exactly the kind of error that looks
            # plausible in a video, so the mapping is stored instead of inferred.
            self._collider_bodies.append(body)
            # ⚠ KEPT FOR --collider-mass. The wire has carried `mass` since v1 and nothing has ever
            # read it; see the setup_collider call below for why that was the ejection bug.
            self._collider_masses.append(float(entry.mass))
            body_xform = xform

            # ⚠ REPORTED vs ASSUMED, said out loud. A backend that could not state a friction
            # coefficient must not be indistinguishable from one that stated Newton's default.
            shape_friction = float(entry.friction) if entry.material_reported else 0.0
            if self.args.collider_friction > 0.0:
                shape_friction = float(self.args.collider_friction)   # explicit override wins
            frictions_used.append((name, shape_friction if shape_friction > 0.0
                                   else float(self.args.collider_friction_default),
                                   bool(entry.material_reported)))

            for s in range(entry.shape_count):
                shape = entry.shapes[s]
                xform = wp.transform(wp.vec3(*shape.position.as_tuple()),
                                     wp.quat(*shape.orientation.as_xyzw()))
                # ⚠ REMEMBERED IN WORLD SPACE so the sand can be spawned AROUND this shape rather
                # than through it. See _spawn_sand: the bed is a filled box built wherever the
                # operator declared it, with no knowledge of what is standing there, so without
                # this every collider present at t=0 is born full of sand that never leaves.
                world = wp.transform_multiply(body_xform, xform)
                # ⚠ A HULL CARRIES NEITHER radius NOR half_extents — both are 0 on the wire — so
                # the bounding-sphere fallback in _points_in_shape would cull nothing at all and
                # silently leave every Box3D wheel full of sand. Derive the bound from the actual
                # vertices, which is the only place a hull's size is stated.
                bound = float(shape.radius)
                if shape.kind == P.SHAPE_CONVEX_HULL:
                    verts = np.asarray(shape.hull_vertices(), dtype=float)
                    bound = float(np.linalg.norm(verts, axis=1).max()) if len(verts) else 0.0
                self._collider_volumes.append(dict(
                    kind=shape.kind,
                    hull_bound=bound,
                    pos=np.array(wp.transform_get_translation(world), dtype=float),
                    quat=np.array(wp.transform_get_rotation(world), dtype=float),  # xyzw
                    radius=float(shape.radius),
                    half_length=float(shape.half_length),
                    half_extents=np.array(shape.half_extents.as_tuple(), dtype=float),
                ))
                # ⚠ THE WIRE HAS CARRIED THIS SINCE v1 AND IT WAS BEING THROWN AWAY. Every
                # add_shape_* call below used to omit `cfg`, so every collider silently inherited
                # Newton's default mu of 0.5 no matter what the simulator reported. That default
                # happens to equal the sand's own internal friction, i.e. the SMOOTH-WHEEL limit,
                # and it is decisive rather than cosmetic: measured 2026-08-26 on a 52 kg Scout
                # against a 2 x 1.6 x 0.4 m mound, all else identical —
                #     mu 0.5 -> stalls at the toe, climbs 0.082 m
                #     mu 0.8 -> creeps to 40% up the face, climbs 0.251 m
                #     mu 1.2 -> crests and drives off the far side, climbs 0.325 m
                # A lugged wheel shears the SOIL rather than sliding on it, so its effective
                # coefficient is at or above the soil's internal friction; 0.5 understates a
                # treaded tyre. Until this line existed, no settings file could reach it.
                mu = float(shape_friction if shape_friction > 0.0
                           else self.args.collider_friction_default)
                cfg = newton.ModelBuilder.ShapeConfig(mu=mu)

                kind = shape.kind
                if kind == P.SHAPE_SPHERE:
                    builder.add_shape_sphere(body, xform=xform, radius=shape.radius, cfg=cfg)
                elif kind == P.SHAPE_BOX:
                    hx, hy, hz = shape.half_extents.as_tuple()
                    builder.add_shape_box(body, xform=xform, hx=hx, hy=hy, hz=hz, cfg=cfg)
                elif kind in (P.SHAPE_CAPSULE, P.SHAPE_CYLINDER):
                    builder.add_shape_capsule(body, xform=xform, radius=shape.radius,
                                              half_height=shape.half_length, cfg=cfg)
                elif kind == P.SHAPE_CONVEX_HULL:
                    vertices = shape.hull_vertices()
                    if len(vertices) < 4:
                        # ⚠ Named, not skipped in silence. A wheel that contributed no geometry
                        # would let the rover roll through the sand leaving no track, which looks
                        # like a physics result and is a dropped shape.
                        print(f"  ⚠ {name} shape {s}: only {len(vertices)} hull vertices, skipped")
                        continue
                    mesh = self._hull_mesh(np.array(vertices, dtype=np.float32))
                    if mesh is not None:
                        builder.add_shape_mesh(body, xform=xform, mesh=mesh, cfg=cfg)
                else:
                    print(f"  ⚠ {name} shape {s}: wire kind {kind} not handled, skipped")

        # ⚠ THE SAND NEEDS SOMETHING TO REST ON, and forgetting it does not look like an error:
        # the first version of this file had no ground, so 85,264 particles fell freely forever
        # while the link reported "healthy — 6.6 ms solves, lag inside budget" the whole time.
        # Every counter was green and the simulation was meaningless. Caught by looking at it.
        ground_z = self.args.ground_z
        if ground_z is None:
            if self._region is not None and self._region.valid:
                # The floor of the declared region: the sand rests on the bottom of its own patch.
                ground_z = self._region.center.z - self._region.half_extent.z
            else:
                ground_z = self.args.patch_z - self.args.patch_depth * 0.5
        builder.add_ground_plane(
            height=float(ground_z),
            cfg=newton.ModelBuilder.ShapeConfig(mu=float(self.args.ground_friction)))
        self.ground_z = float(ground_z)
        print(f"ground plane at z = {self.ground_z:.3f} m, mu = {self.args.ground_friction}")

        # ⚠ BEFORE the particles, per Newton's own requirement — custom attributes must be
        # registered before anything that carries them exists.
        SolverImplicitMPM.register_custom_attributes(builder)
        self._spawn_sand(builder)

        self.model = builder.finalize()
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.particle_count = len(self.model.particle_q) if self.model.particle_q is not None else 0

        # ⚠ D13 MATERIAL: the sand's own internal friction, which is what sets its ANGLE OF REPOSE.
        # Newton's default is 0.5 -> atan(0.5) = 26.6 deg, and the beds measured on 2026-08-26 stood
        # at 24.2-26.2 deg, confirming it. Filled BEFORE the solver is constructed so no
        # notify_model_changed is needed, exactly as newton/examples/mpm/example_mpm_viscous.py does.
        #
        # ⚠ THE TWO FRICTIONS INTERACT, and getting them equal is a trap rather than a neutral
        # choice: a heap rests at atan(sand mu) and a vehicle climbs at most atan(wheel mu), so
        # equal coefficients put every vehicle exactly at marginal stability on a slope the sand
        # builds by itself. That is the configuration in which nothing ever climbs anything.
        if self.args.sand_friction > 0.0:
            import math as _math
            self.model.mpm.friction.fill_(float(self.args.sand_friction))
            print(f"sand internal friction mu = {self.args.sand_friction} "
                  f"-> angle of repose {_math.degrees(_math.atan(self.args.sand_friction)):.1f} deg")

        config = SolverImplicitMPM.Config()
        config.voxel_size = float(self.args.voxel_size)
        config.tolerance = 1.0e-4
        config.transfer_scheme = "pic"
        config.max_iterations = 50
        config.critical_fraction = 0.0
        config.air_drag = 1.0
        # ⚠ "forward", NOT "backward", AND THE DIFFERENCE BLEW UP THE SAND.
        #
        # "backward" makes Newton derive collider velocity by differencing successive poses and
        # dividing by ITS OWN step (1/fps). But this sidecar samples latest-wins at whatever rate
        # it can manage, so the displacement between two solves covers however much simulated time
        # actually passed — and when a render frame costs ~1 s, the simulator advances ~333 ticks
        # while that displacement is credited to a 16.67 ms window. A ~60x overestimate of wheel
        # speed goes straight into the contact law and the patch detonates.
        #
        # "forward" uses `state.body_qd` instead — the velocity we are ALREADY SENT over the wire,
        # straight out of the rigid solver. It is correct regardless of how sparsely we sample, so
        # the failure disappears rather than being tuned around. It is also Newton's own default;
        # overriding it was the mistake.
        config.collider_velocity_mode = "forward"

        self.solver = SolverImplicitMPM(self.model, config=config)

        # ⚠ THE COLLIDER MASS IS THE TWO-WAY SWITCH, AND ZERO MEANT "INFINITELY HEAVY".
        #
        # Newton's documented mechanism: "Rigid body colliders will be treated as kinematic if
        # their effective mass is zero... An explicit body_mass array is authoritative." Passing
        # zeros therefore does not mean "no mass" — it means the sand is solved against an
        # IMMOVABLE WALL. The impulse that comes back is whatever it takes to stop the sand dead
        # against infinite inertia, and the simulator then applies that number to a 0.28 kg wheel.
        #
        # Measured in PhysicsEngineDiscussion/newton_probes (results/bridge_massscale_*.csv), with
        # everything else held identical, peak sand force on the rover against the mass Newton
        # believes the wheel has:
        #     1x true mass    8.6 N        100x   39.1 N        10000x   78.3 N
        # and the live Unreal bridge, which is past the right-hand end of that table AND has no
        # fixed-point iteration, reached 1443 N on a 16.5 N vehicle. Newton's own coupled solver
        # instead asks the rigid solver for each body's ARTICULATED EFFECTIVE inertia
        # (`coupling_eval_effective_mass_block`) and installs that as the proxy mass.
        #
        # ⚠ THE LINK'S OWN MASS IS NOT THAT QUANTITY — §11.1 is still right about that, and the wire
        # says so honestly via `inertia_is_articulated_effective`, which no backend sets to true
        # yet. But finite-and-approximate beats infinite: the gap this closes is the one between a
        # wall and a wheel, not the one between a wheel and a wheel-on-a-rocker.
        #
        # The bodies stay `is_kinematic=True`, so Newton still never integrates them; their motion
        # is still the pose we push each frame. This array only tells the CONTACT SOLVE how much
        # inertia to expect on the other side.
        mode = self.args.collider_mass
        if mode == "kinematic" or not self._collider_masses:
            body_mass = wp.zeros_like(self.model.body_mass)
            print("colliders are KINEMATIC to the sand (mass 0 = infinite): one-way, M2 behaviour")
        else:
            # ⚠ THE SAND SHOULD FEEL A VEHICLE, NOT SIX LOOSE WHEEL CASTINGS. A URDF wheel link
            # weighs 57 g; the thing actually resisting the sand is that wheel plus its share of the
            # 1.68 kg rover it is bolted to. --collider-mass-total spreads a declared vehicle mass
            # across the colliders IN PROPORTION to their own mass, so the total the contact solve
            # sees is the vehicle and no collider is counted twice.
            #
            # ⚠ NOT "the whole vehicle mass on every collider" — that is the same 1443 N mistake
            # approached from the other side, over-counting by the number of contacts.
            scale = float(self.args.collider_mass_scale)
            total_wire = sum(self._collider_masses)
            if self.args.collider_mass_total > 0.0 and total_wire > 0.0:
                scale = float(self.args.collider_mass_total) / total_wire
                print(f"--collider-mass-total {self.args.collider_mass_total} kg over "
                      f"{total_wire:.3f} kg of link mass -> scale x{scale:.2f}")

            masses = np.zeros(len(self.model.body_mass), dtype=np.float32)
            missing = []
            for slot, body in enumerate(self._collider_bodies):
                m = self._collider_masses[slot] * scale
                # ⚠ A ZERO HERE WOULD SILENTLY RESTORE THE BUG for that one link, and a link whose
                # mass the simulator could not report is exactly the one worth naming.
                if not (m > 0.0):
                    # ⚠ body_label; Model has no body_key. Caught while debugging a penetration
                    # metric that had the same typo and silently measured the wrong bodies.
                    missing.append(self.model.body_label[body])
                    m = float(self.args.collider_mass_floor)
                masses[body] = m
            body_mass = wp.array(masses, dtype=float)
            total = float(masses.sum())
            print(f"colliders have REAL MASS to the sand: {total:.3f} kg over "
                  f"{len(self._collider_bodies)} colliders "
                  f"(x{scale:.2f} scale) — two-way contact")
            if missing:
                print(f"  ⚠ {len(missing)} collider(s) reported no mass on the wire and fell back "
                      f"to {self.args.collider_mass_floor} kg: {', '.join(missing[:6])}")

        self.solver.setup_collider(
            body_mass=body_mass,
            body_q=self.state_0.body_q,
        )

        if frictions_used:
            reported = sum(1 for _, _, r in frictions_used if r)
            mus = sorted({round(m, 3) for _, m, _ in frictions_used})
            print(f"collider friction mu {mus} — {reported} of {len(frictions_used)} reported by "
                  f"the simulator, the rest defaulted to {self.args.collider_friction_default}"
                  + (f" (overridden to {self.args.collider_friction})"
                     if self.args.collider_friction > 0.0 else ""))

        print(f"built: {count} colliders, {self.particle_count} sand particles, "
              f"voxel {config.voxel_size} m")

        if self.args.render_to:
            self._setup_offline_viewer()

    def _hull_mesh(self, vertices):
        """Turn a hull point cloud into a Newton Mesh via its convex hull triangulation."""
        import numpy as np

        import newton

        try:
            from scipy.spatial import ConvexHull
        except ImportError:
            print("  ⚠ scipy is not installed, so convex hulls cannot be triangulated; "
                  "install scipy in this env or the wheels will have no geometry")
            return None
        try:
            hull = ConvexHull(vertices)
        except Exception as error:  # degenerate cloud
            print(f"  ⚠ hull failed ({error}), shape skipped")
            return None
        return newton.Mesh(vertices.astype(np.float32),
                           np.asarray(hull.simplices, dtype=np.int32).flatten())

    def _spawn_sand(self, builder) -> None:
        """A rectangular patch of sand, centred where the operator asked for it."""
        import numpy as np
        import warp as wp

        # ⚠ THE SIMULATOR OWNS WHERE THE SAND IS, when it says so. The CLI is a fallback for
        # standalone and fixture use only. Before the region reached the wire (protocol v1) this
        # method always used the CLI, so the simulator could declare a patch at (2, 0, 0) while the
        # sidecar built one at the origin — the rover would drive through empty space and every
        # diagnostic on both sides would report healthy.
        if self._region is not None and self._region.valid:
            centre = np.array(self._region.center.as_tuple(), dtype=float)
            half = np.array(self._region.half_extent.as_tuple(), dtype=float)
            print(f"sand region from the simulator: '{self._region.name()}' centred "
                  f"({centre[0]:.2f} {centre[1]:.2f} {centre[2]:.2f}), half-extent "
                  f"({half[0]:.2f} {half[1]:.2f} {half[2]:.2f})")
        else:
            centre = np.array([self.args.patch_x, self.args.patch_y, self.args.patch_z],
                              dtype=float)
            half = np.array([self.args.patch_size, self.args.patch_size,
                             self.args.patch_depth * 0.5], dtype=float)
            # Said out loud: a sidecar quietly inventing its own patch is how the two ends stop
            # agreeing about where the sand is while both look healthy.
            print("⚠ the simulator declared NO terrain region — falling back to the CLI patch. "
                  "If a simulator is driving this, its DeformableTerrains region is missing.")
        lo = centre - half
        hi = centre + half

        particles_per_cell = 3
        voxel = float(self.args.voxel_size)
        res = np.array(np.ceil(particles_per_cell * (hi - lo) / voxel), dtype=int)
        res = np.maximum(res, 1)
        cell = (hi - lo) / res
        radius = float(np.max(cell)) * 0.5
        self._particle_radius = radius
        mass = float(np.prod(cell)) * float(self.args.density)

        dims = (int(res[0]) + 1, int(res[1]) + 1, int(res[2]) + 1)
        if not self._collider_volumes or self.args.no_collider_cull:
            builder.add_particle_grid(
                pos=wp.vec3(*lo), rot=wp.quat_identity(), vel=wp.vec3(0.0),
                dim_x=dims[0], dim_y=dims[1], dim_z=dims[2],
                cell_x=float(cell[0]), cell_y=float(cell[1]), cell_z=float(cell[2]),
                mass=mass, jitter=2.0 * radius, radius_mean=radius,
            )
            return

        # ⚠ THE BED MUST NOT BE BUILT THROUGH THE ROBOT.
        #
        # add_particle_grid fills the declared box regardless of what occupies it, so every
        # collider standing in the patch at build time is born full of sand — and MPM's collision
        # only stops particles CROSSING a boundary, never evicts ones that started inside. They
        # stay there for the life of the run, moving with the wheel, and the wheel reads as
        # transparent: sand at ~ambient density inside it, no visible displacement. It also puts a
        # wheel's worth of trapped mass inside every wheel.
        #
        # The grid is reproduced exactly as ModelBuilder.add_particle_grid generates it, jitter
        # and all (same rng seed rule: 42 + len(particle_q)), then filtered, then bulk-added — so
        # culling changes WHICH particles exist and nothing else about them.
        px = np.arange(dims[0]) * float(cell[0])
        py = np.arange(dims[1]) * float(cell[1])
        pz = np.arange(dims[2]) * float(cell[2])
        pts = np.stack(np.meshgrid(px, py, pz)).reshape(3, -1).T + lo
        rng = np.random.default_rng(42 + len(builder.particle_q))
        pts += (rng.random(pts.shape) - 0.5) * (2.0 * radius)

        inside = np.zeros(len(pts), dtype=bool)
        for v in self._collider_volumes:
            inside |= self._points_in_shape(pts, v, skin=radius)

        kept = pts[~inside]
        culled = int(inside.sum())
        builder.add_particles(
            pos=kept.tolist(),
            vel=[(0.0, 0.0, 0.0)] * len(kept),
            mass=[mass] * len(kept),
            radius=[radius] * len(kept),
        )
        print(f"sand: {len(kept)} particles, {culled} culled from inside "
              f"{len(self._collider_volumes)} collider shape(s) "
              f"({100.0 * culled / max(1, len(pts)):.2f}% of the bed)")

    @staticmethod
    def _points_in_shape(pts, v, skin: float = 0.0) -> "np.ndarray":
        """Boolean mask of points inside one collider shape, in world space.

        ⚠ `skin` grows the shape by one particle radius. A particle whose CENTRE is just outside
        the surface still overlaps it, and leaving those behind produces a shell of half-embedded
        grains that the solver immediately has to resolve.
        """
        import numpy as np

        d = pts - v["pos"]
        kind = v["kind"]
        r = v["radius"] + skin

        if kind == P.SHAPE_SPHERE:
            return np.einsum("ij,ij->i", d, d) < r * r

        # Rotate into the shape's local frame: q is xyzw, and the INVERSE rotation is what takes a
        # world offset into local coordinates.
        x, y, z, w = v["quat"]
        n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
        x, y, z, w = x / n, y / n, z / n, w / n
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ])
        local = d @ R          # d @ R == R^T @ d, i.e. world -> local

        if kind == P.SHAPE_BOX:
            h = v["half_extents"] + skin
            return np.all(np.abs(local) < h, axis=1)

        if kind in (P.SHAPE_CAPSULE, P.SHAPE_CYLINDER):
            hl = v["half_length"] + skin
            radial = local[:, 0] ** 2 + local[:, 1] ** 2
            return (radial < r * r) & (np.abs(local[:, 2]) < hl)

        # ⚠ CONVEX HULL FALLS BACK TO A BOUNDING SPHERE, deliberately over-culling rather than
        # under-culling: a few extra grains removed next to a wheel settle back in during
        # presettle, whereas grains left inside it never leave at all. Box3D reports wheels as
        # 32-vertex hulls, so this is the path the ExoMy actually takes.
        bound = max(v.get("hull_bound", 0.0), v["radius"],
                    float(np.max(np.abs(v["half_extents"]))), v["half_length"]) + skin
        return np.einsum("ij,ij->i", d, d) < bound * bound

    def _setup_offline_viewer(self) -> None:
        """Offscreen GL viewer writing PNGs, encoded to video when the run ends.

        ⚠ OFFSCREEN, NOT A WINDOW. `glxinfo` on this host reports `llvmpipe` — Mesa's CPU
        rasteriser — so an on-screen viewer draws sand at about 1 fps and Rerun's web viewer drew
        no particles at all. Rasterising offscreen is equally slow but happens once, and the result
        plays back at true speed. See Newton-MPM-Discussions/m0/RESULTS.md.
        """
        import warp as wp
        from newton._src.viewer.viewer_gl import ViewerGL

        self._frame_dir = os.path.abspath(self.args.render_to) + ".frames"
        os.makedirs(self._frame_dir, exist_ok=True)
        for stale in os.listdir(self._frame_dir):
            os.remove(os.path.join(self._frame_dir, stale))
        self._frames_written = 0

        self.viewer = ViewerGL(headless=True, num_frames=None, vsync=False)
        self.viewer.set_model(self.model)
        self.viewer.show_particles = True

        # ⚠ AIM THE CAMERA AT THE SAND, or the recording is 167 frames of empty space.
        #
        # ViewerGL's default camera sits near the world origin. EXECOsim's solver frame has its
        # origin at the UNREAL WORLD origin, and a patch declared 2 m in front of a rover was at
        # solver (123.57, 24.90) — 123 m away. The first in-sim recording came back with 0.000 %
        # pixel change between every pair of frames: a perfectly valid render of nothing at all.
        # The fixture videos only worked because their sand happened to sit at the origin.
        centre, distance = self._camera_target()
        import math
        pitch = -float(self.args.camera_pitch)
        yaw = float(self.args.camera_yaw)
        rp, ry = math.radians(pitch), math.radians(yaw)
        eye = wp.vec3(
            centre[0] - distance * math.cos(rp) * math.cos(ry),
            centre[1] - distance * math.cos(rp) * math.sin(ry),
            centre[2] - distance * math.sin(rp),
        )
        self.viewer.set_camera(eye, pitch, yaw)
        print(f"camera at ({eye[0]:.2f} {eye[1]:.2f} {eye[2]:.2f}) looking at "
              f"({centre[0]:.2f} {centre[1]:.2f} {centre[2]:.2f}), {distance:.2f} m away")
        print(f"offscreen rendering to {self.args.render_to} "
              f"(every {self.args.render_every} solves)")

    def publish_impulses(self, frame_dt: float) -> None:
        """Reduce Newton's per-grid-node impulses to one spatial impulse per collider, and publish.

        ⚠ NEWTON REPORTS PER NODE, NOT PER BODY. `collect_collider_impulses` returns one impulse and
        one application position for every contacting grid node, keyed by collider id; the reduction
        to a body wrench is the CONSUMER's job, and Newton's own `compute_body_forces` kernel shows
        the intended form: force at the centre of mass plus the moment of `r x f` about it. Doing it
        here rather than in C++ keeps one definition of the reduction next to the solver that
        produced the nodes.

        ⚠ The sign is already the reaction ON the collider — `collect_collider_impulses` returns
        `-cell_volume * impulse_field`. Negating again would have the sand pull the rover in.
        """
        if self.impulse_seg is None or self.solver is None:
            return

        import numpy as np

        imp, pos, cids = self.solver.collect_collider_impulses(self.state_0)
        imp_np = imp.numpy()
        pos_np = pos.numpy()
        cid_np = cids.numpy()
        body_of_collider = self.solver.collider_body_index.numpy()

        n = len(self._collider_bodies)
        linear = np.zeros((n, 3), dtype=np.float64)
        angular = np.zeros((n, 3), dtype=np.float64)
        nodes = np.zeros(n, dtype=np.int64)

        # body index -> our collider slot
        slot_of_body = {int(b): k for k, b in enumerate(self._collider_bodies)}

        body_q = self.state_0.body_q.numpy()
        body_com = self.model.body_com.numpy()

        valid = cid_np >= 0
        if np.any(valid):
            cid_v = cid_np[valid]
            cid_v = cid_v[cid_v < body_of_collider.shape[0]]
            if cid_v.shape[0] > 0:
                bodies = body_of_collider[cid_v]
                imp_v = imp_np[valid][: cid_v.shape[0]]
                pos_v = pos_np[valid][: cid_v.shape[0]]
                for slot_body, slot in slot_of_body.items():
                    mask = bodies == slot_body
                    if not np.any(mask):
                        continue
                    j = imp_v[mask].astype(np.float64)
                    p = pos_v[mask].astype(np.float64)
                    # COM in world: body translation + rotate(quat, local com)
                    q = body_q[slot_body]
                    t = np.asarray(q[0:3], dtype=np.float64)
                    qx, qy, qz, qw = (float(q[3]), float(q[4]), float(q[5]), float(q[6]))
                    c = np.asarray(body_com[slot_body], dtype=np.float64)
                    uv = np.cross(np.array([qx, qy, qz]), c)
                    com_world = t + c + 2.0 * (qw * uv + np.cross(np.array([qx, qy, qz]), uv))
                    r = p - com_world
                    linear[slot] = j.sum(axis=0)
                    angular[slot] = np.cross(r, j).sum(axis=0)
                    nodes[slot] = int(mask.sum())

        block = self.impulse_seg.block
        block.sequence |= 1
        block.stamp = self.status_seg.block.stamp
        block.collider_count = n
        block.sidecar_step = self.sidecar_step
        block.sidecar_time = self.sidecar_time
        block.mpm_dt = float(frame_dt)
        for k in range(n):
            c = block.colliders[k]
            c.linear.x, c.linear.y, c.linear.z = (float(linear[k][0]), float(linear[k][1]),
                                                  float(linear[k][2]))
            c.angular.x, c.angular.y, c.angular.z = (float(angular[k][0]), float(angular[k][1]),
                                                     float(angular[k][2]))
            c.contact_nodes = int(nodes[k])
        block.sequence += 1

        if not self._impulse_announced:
            self._impulse_announced = True
            touched = int((nodes > 0).sum())
            print(f"two-way: publishing impulses for {n} collider(s), {touched} in contact this "
                  f"frame, accumulated over dt={frame_dt:.5f} s", flush=True)

    def publish_particles(self) -> None:
        """Offer Unreal a decimated snapshot of the sand, for rendering only.

        ⚠ THE GPU->HOST COPY IS THE COST HERE, and it is why this is decimated and throttled. The
        particles live in Warp's CUDA memory in this process; Unreal renders from another process
        entirely, so they take a brief hop through host memory. 100 k floats3 is 1.2 MB — nothing
        for /dev/shm, but a full 306 k copy every solve would be pure waste for pixels.
        """
        if self.particle_seg is None:
            return
        if self.sidecar_step % max(1, int(self.args.particle_every)) != 0:
            return

        import numpy as np

        points = self.state_0.particle_q.numpy()
        total = int(points.shape[0])
        cap = min(int(self.args.max_render_particles), P.MAX_RENDER_PARTICLES)

        # ⚠ Strided, not the first N. Taking a prefix of the array renders one CORNER of the patch
        # at full density and leaves the rest empty, which looks exactly like sand that is only
        # there in one place.
        stride = max(1, (total + cap - 1) // cap)
        sample = np.ascontiguousarray(points[::stride][:cap], dtype=np.float32)
        count = int(sample.shape[0])

        block = self.particle_seg.block
        block.sequence |= 1
        block.stamp = self.status_seg.block.stamp
        block.sidecar_step = self.sidecar_step
        block.sidecar_time = self.sidecar_time
        block.total_particles = total
        block.particle_count = count
        block.radius = float(self._particle_radius)
        ctypes.memmove(block.positions, sample.ctypes.data, count * 3 * 4)
        block.sequence = (block.sequence + 1) & ~1

        if not self._particles_announced:
            self._particles_announced = True
            print(f"publishing particles for rendering: {count} of {total} "
                  f"(stride {stride}, radius {self._particle_radius:.4f} m)")

    def _camera_target(self):
        """Where to point the camera, and how far back to stand.

        Prefers the sand region because that is the subject; falls back to the colliders' own
        centroid when no region was declared, so a fixture run still frames its probe.
        """
        if self._region is not None and self._region.valid:
            c = self._region.center.as_tuple()
            h = self._region.half_extent.as_tuple()
            span = max(h[0], h[1]) * 2.0
        else:
            c = (self.args.patch_x, self.args.patch_y, self.args.patch_z)
            span = self.args.patch_size * 2.0
        distance = self.args.camera_distance if self.args.camera_distance > 0 else max(span * 1.8, 2.0)
        return c, distance

    def _capture_frame(self) -> None:
        from PIL import Image

        frame = self.viewer.get_frame().numpy()
        Image.fromarray(frame).save(
            os.path.join(self._frame_dir, f"frame_{self._frames_written:05d}.png"))
        self._frames_written += 1

    def _encode_video(self) -> None:
        """Encode whatever frames were captured. Called however the run ends."""
        import subprocess

        if self.viewer is None or getattr(self, "_frames_written", 0) == 0:
            return
        out_path = os.path.abspath(self.args.render_to)

        # ⚠ PLAYBACK RATE IS DERIVED, NOT 30. Each captured frame represents `render_every` solves,
        # and one solve advances 1/fps of SIMULATED time — so a frame is `render_every / fps`
        # seconds of sim. Hardcoding 30 fps made a recording play back
        # `30 * render_every / fps` times too fast: at the defaults that is 4x, and an operator who
        # spent 30 s driving got a 4 s video. The number below makes one second of video equal one
        # second of MPM simulated time.
        playback_fps = max(float(self.args.fps) / max(1, int(self.args.render_every)), 1.0)
        print(f"encoding {self._frames_written} frames -> {out_path} "
              f"at {playback_fps:.2f} fps (1 s of video = 1 s of simulated time)")
        result = subprocess.run(
            ["ffmpeg", "-y", "-framerate", f"{playback_fps:.4f}",
             "-i", os.path.join(self._frame_dir, "frame_%05d.png"),
             "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", out_path],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode != 0:
            print("ffmpeg failed; the PNG frames are kept in " + self._frame_dir, file=sys.stderr)
            return
        print(f"wrote {out_path} ({os.path.getsize(out_path) / 1048576:.1f} MiB, "
              f"{self._frames_written / playback_fps:.1f} s of simulated time)")
        for name in os.listdir(self._frame_dir):
            os.remove(os.path.join(self._frame_dir, name))
        os.rmdir(self._frame_dir)

    # ---- the loop ------------------------------------------------------------------------

    def apply_collider_state(self, state, registry=None) -> None:
        """Push the simulator's poses into the kinematic bodies."""
        import numpy as np
        import warp as wp

        poses = self.state_0.body_q.numpy()
        twists = self.state_0.body_qd.numpy()
        if registry is None:
            registry = P.read_consistent(self.registry_seg)

        jumped = []
        for i in range(state.collider_count):
            name = registry.colliders[i].name()
            body = self.body_index.get(name)
            if body is None:
                continue
            entry = state.colliders[i]
            new_pos = entry.position.as_tuple()

            # ⚠ A teleport is REPORTED, not absorbed. With "forward" velocities a large pose jump
            # no longer detonates the sand, but it still means we skipped over motion the sand
            # never saw — so it is a fidelity warning, not a crash, and it should be visible.
            old_pos = poses[body][0:3]
            step_jump = float(((new_pos[0] - old_pos[0]) ** 2 + (new_pos[1] - old_pos[1]) ** 2 +
                               (new_pos[2] - old_pos[2]) ** 2) ** 0.5)
            if self.has_consumed and step_jump > self.args.jump_warn:
                jumped.append((name, step_jump))

            poses[body][0:3] = new_pos
            poses[body][3:7] = entry.orientation.as_xyzw()

            # ⚠ Newton's body_qd is (LINEAR, ANGULAR) — first three linear, last three angular
            # (builder.py:5105). MuJoCo's mj_objectVelocity is the opposite order, which is why the
            # wire carries them as two NAMED fields rather than one packed spatial vector: there is
            # no order to get silently wrong here.
            twists[body][0:3] = entry.linear_velocity.as_tuple()
            twists[body][3:6] = entry.angular_velocity.as_tuple()

        self.state_0.body_q = wp.array(poses, dtype=self.state_0.body_q.dtype,
                                       device=self.state_0.body_q.device)
        self.state_0.body_qd = wp.array(twists, dtype=self.state_0.body_qd.dtype,
                                        device=self.state_0.body_qd.device)

        if jumped and not self._jump_warned:
            self._jump_warned = True
            worst = max(jumped, key=lambda j: j[1])
            print(f"⚠ collider jumped {worst[1]:.3f} m between solves ({worst[0]}). The sand did "
                  f"not see the motion in between — velocities are still correct, but the furrow "
                  f"will be sampled coarsely. Reduce --render-every, or drive more slowly.",
                  file=sys.stderr)

    def run_replay(self) -> int:
        """Re-simulate a recorded trajectory, consuming EVERY sample.

        ⚠ This is not a video of the earlier run — it is a fresh solve driven by the same collider
        motion. The live run skipped samples because rendering stalled it; this one skips none, so
        the sand it produces is the sand that motion should have made. The two can legitimately
        differ, and the replay is the more faithful of the pair.
        """
        import numpy as np

        path = self.args.replay_poses
        if not path.endswith(".npz"):
            path += ".npz"
        archive = np.load(path)
        registry = P.MpmRegistryBlock.from_buffer_copy(archive["registry"].tobytes())
        P.validate(registry, P.REGISTRY_MAGIC, "recorded registry", accept_older=True)
        states = archive["states"]
        print(f"replaying {len(states)} samples from {path}")

        self.current_stamp = registry.stamp.key()
        if len(states) > 0:
            self._initial_state = P.MpmStateBlock.from_buffer_copy(states[0].tobytes())
        try:
            self.build(registry)
        except Exception as error:
            print(f"BUILD FAILED: {error}", file=sys.stderr)
            return 1

        frame_dt = 1.0 / float(self.args.fps)
        for index in range(len(states)):
            state = P.MpmStateBlock.from_buffer_copy(states[index].tobytes())
            self.apply_collider_state(state, registry)

            started = time.perf_counter()
            self.solver.step(self.state_0, self.state_1, None, None, frame_dt)
            self.last_solve_seconds = time.perf_counter() - started
            self.publish_impulses(frame_dt)

            self.state_0, self.state_1 = self.state_1, self.state_0
            self.sidecar_step += 1
            self.sidecar_time += frame_dt

            if self.sidecar_step % 60 == 0:
                self._check_sand_is_supported()
            if self.viewer is not None and \
                    self.sidecar_step % max(1, self.args.render_every) == 0:
                self.viewer.begin_frame(self.sidecar_time)
                self.viewer.log_state(self.state_0)
                self.viewer.end_frame()
                self._capture_frame()
            if index % 50 == 0:
                print(f"  {index}/{len(states)} samples, {self.last_solve_seconds*1000:.1f} ms/solve",
                      flush=True)
        print(f"replayed {len(states)} samples")
        return 0

    def rebuild_for_epoch(self, state) -> bool:
        """Rebuild the whole scene for a new world stamp. True when the new registry was found.

        The simulator republishes the registry before it publishes state for a moved stamp, so by
        the time we see a changed epoch the new topology is already there. We still verify it
        rather than assume: a registry that has NOT caught up means the two ends disagree, and
        guessing which is right is exactly the failure the stamp exists to prevent.
        """
        want = state.stamp.key()
        registry = None
        for _ in range(50):                      # bounded: 50 x 20 ms = 1 s
            candidate = P.read_consistent(self.registry_seg)
            if candidate.magic == P.REGISTRY_MAGIC and candidate.stamp.key() == want:
                registry = candidate
                break
            time.sleep(0.02)
        if registry is None:
            return False

        print(f"\nRESET — world stamp moved to {state.stamp}. Rebuilding: fresh sand at the "
              f"declared region, colliders at their post-reset poses.", flush=True)
        if self.args.record_poses and self._recorded_states:
            # ⚠ SAY IT. A capture that spans a reset replays as one continuous trajectory, because
            # the replay has no reset to perform — the rover would appear to teleport back and
            # carry on. Better a warning now than a video nobody can explain later.
            print(f"⚠ this recording already holds {len(self._recorded_states)} samples from the "
                  f"previous epoch; a replay of it will NOT reproduce the reset", flush=True)

        self.current_stamp = want
        self._recorded_registry = bytes(registry)
        self.status_seg.block.stamp = state.stamp

        # ⚠ Drop the old scene BEFORE building the new one. `build()` replaces these at the end, so
        # without this the old and new models are both live across the call and peak GPU use
        # doubles — 2.4 GB at the current patch, and far worse at a fine voxel. The FAULT 2 OOM
        # earlier in this workstream came from exactly that kind of overlap.
        self.solver = None
        self.state_0 = None
        self.model = None
        gc.collect()

        try:
            self.build(registry)
        except Exception as error:
            self.publish_status(FAULT_TOPOLOGY, f"rebuild after reset failed: {error}")
            print(f"REBUILD FAILED: {error}", file=sys.stderr)
            raise

        # A rebuilt scene has consumed nothing. Leaving these at their old values would make the
        # first post-reset state look already-acknowledged and the sidecar would idle forever —
        # the same step-zero deadlock `has_consumed` was added to fix.
        self.sidecar_step = 0
        self.sidecar_time = 0.0
        self.acknowledged_step = 0
        self.has_consumed = False
        self.publish_status()
        return True

    def run(self) -> int:
        import warp as wp

        self.attach()

        registry = P.read_consistent(self.registry_seg)
        P.validate(registry, P.REGISTRY_MAGIC, "registry")
        self._recorded_registry = bytes(registry)
        self.current_stamp = registry.stamp.key()
        self.status_seg.block.stamp = registry.stamp
        print(f"registry: {registry.collider_count} colliders, {registry.stamp}")

        try:
            self.build(registry)
        except Exception as error:
            self.publish_status(FAULT_TOPOLOGY, str(error))
            print(f"BUILD FAILED: {error}", file=sys.stderr)
            return 1

        frame_dt = 1.0 / float(self.args.fps)
        idle_reported = False
        print("running — waiting for collider state")

        parent_pid = int(self.args.parent_pid)
        last_parent_check = 0.0

        while True:
            # ⚠ ORPHAN CONTROL. When the simulator owns this process it passes its own pid; if that
            # process is gone we are an orphan holding GPU memory with nobody to publish for.
            # Measured 2026-08-26: two abandoned sidecars, one for 32 minutes, found only by hand.
            # Checked once a second — os.kill(pid, 0) is cheap but not free, and nothing about this
            # needs to be prompt.
            if parent_pid > 0:
                now_wall = time.time()
                if now_wall - last_parent_check > 1.0:
                    last_parent_check = now_wall
                    try:
                        os.kill(parent_pid, 0)
                    except OSError:
                        print(f"\nparent process {parent_pid} is gone — exiting rather than "
                              f"holding the GPU for a simulator that has stopped.", flush=True)
                        return 0

            state = P.read_consistent(self.state_seg)
            if state.magic != P.STATE_MAGIC:
                time.sleep(0.01)
                continue

            # ⚠ AN EPOCH CHANGE IS A HARD STOP, not something to absorb. Plan §M2: a sidecar
            # restart forces a global reset because MPM environmental memory was lost — the
            # converse holds too. Sand shaped by a previous run's robot is not valid initial
            # state for this one, and quietly carrying on is the failure the stamp exists to
            # prevent.
            # ⚠ ADOPT THE STATE'S EPOCH ON THE FIRST READ, do not inherit the registry's.
            #
            # The registry is published once, when the simulator starts, and is NOT republished on
            # an ordinary global reset — a reset restores state without changing topology, so the
            # collider list stays valid while the epoch moves on. A sidecar started after any reset
            # therefore found registry epoch 1 against state epoch 2 and refused the whole run,
            # even though it was about to build completely fresh sand and had nothing stale to
            # protect. Whatever epoch the world is in when we build IS our baseline; only a change
            # AFTER that means our sand belongs to a run that no longer exists.
            if not self.has_consumed and state.stamp.key() != self.current_stamp:
                print(f"adopting the simulator's current epoch: {state.stamp}")
                self.current_stamp = state.stamp.key()
                self.status_seg.block.stamp = state.stamp

            if state.stamp.key() != self.current_stamp:
                # ⚠ A RESET IS NOT A DEATH — not any more. This used to stop the process, which was
                # right while the simulator published its registry exactly once: the sand was
                # shaped by a run that had ended and there was no new topology to rebuild from.
                # The simulator now republishes whenever the world stamp moves, so the new registry
                # is already in shared memory carrying the post-reset collider poses. Rebuilding is
                # the correct answer, and it is what makes a global reset work WITH sand rather
                # than in spite of it.
                #
                # ⚠ Rebuilding discards the deformed bed on purpose. That IS the environmental
                # memory plan §M2 talks about, and a reset is precisely the moment it should go:
                # D9b2 makes the bed's predeformation part of the recorded t=0 initial condition.
                if self.rebuild_for_epoch(state):
                    idle_reported = False
                    continue
                message = (f"epoch changed to {state.stamp} and the registry did not follow — "
                           f"restart the sidecar")
                print(f"\nEPOCH CHANGE — {state.stamp}, but no matching registry arrived. "
                      f"Stopping rather than deforming sand for a run that has ended.",
                      file=sys.stderr)
                self.publish_status(FAULT_TOPOLOGY, message)
                return 2

            if self.has_consumed and state.step <= self.acknowledged_step:
                if not idle_reported:
                    idle_reported = True
                time.sleep(0.001)
                self.publish_status()
                continue
            idle_reported = False

            # ⚠ Recorded BEFORE the solve and only once consumed, so the file contains exactly the
            # samples the sand actually saw. Logging every published state instead would replay a
            # trajectory this run never simulated.
            if self.args.record_poses:
                self._recorded_states.append(bytes(state))

            self.apply_collider_state(state)

            started = time.perf_counter()
            try:
                self.solver.step(self.state_0, self.state_1, None, None, frame_dt)
            except Exception as error:
                self.publish_status(FAULT_SOLVER, str(error))
                print(f"SOLVER FAILED: {error}", file=sys.stderr)
                return 3
            self.last_solve_seconds = time.perf_counter() - started
            self.publish_impulses(frame_dt)

            self.state_0, self.state_1 = self.state_1, self.state_0
            self.sidecar_step += 1
            self.sidecar_time += frame_dt
            # ⚠ Acknowledged only AFTER the solve. Acknowledging on read would tell the simulator
            # we had consumed state we then dropped on the floor when the solve threw.
            self.acknowledged_step = state.step
            self.has_consumed = True
            self.publish_status()
            self.publish_particles()

            # ⚠ CHECK THAT THE SAND IS ACTUALLY SUPPORTED, periodically. "85,264 particles exist"
            # and "85,264 particles are resting on something" are different claims, and only the
            # first one is visible in a particle count. Newton's own granular example asserts
            # exactly this ("all particles are above the ground").
            if self.sidecar_step % 60 == 0:
                self._check_sand_is_supported()

            if self.viewer is not None and self.sidecar_step % max(1, self.args.render_every) == 0:
                self.viewer.begin_frame(self.sidecar_time)
                self.viewer.log_state(self.state_0)
                self.viewer.end_frame()
                self._capture_frame()

            if self.args.max_steps and self.sidecar_step >= self.args.max_steps:
                print(f"reached --max-steps {self.args.max_steps}")
                return 0

    def _check_sand_is_supported(self) -> None:
        """Warn if the sand has fallen below the ground it is supposed to rest on.

        ⚠ Reported once and then throttled: a sidecar that spams a warning every frame gets
        ignored, which is the same outcome as not warning at all.
        """
        lowest = float(self.state_0.particle_q.numpy()[:, 2].min())
        margin = self.ground_z - lowest
        if margin > 0.5:
            if not self._fall_warned:
                self._fall_warned = True
                print(f"⚠ SAND IS FALLING THROUGH: lowest particle is {margin:.2f} m BELOW the "
                      f"ground plane at z={self.ground_z:.3f}. The patch is not supported — check "
                      f"--ground-z against --patch-z/--patch-depth.", file=sys.stderr)
        elif self._fall_warned and margin <= 0.5:
            self._fall_warned = False

    def save_recording(self) -> None:
        """Write the consumed collider trajectory so it can be replayed with rendering on.

        ⚠ WHY THIS EXISTS. Rasterising is CPU-bound here at ~1 fps, so a live run that records
        video samples the robot coarsely — a wheel moved 0.52 m between two solves in one measured
        run, and the sand never saw the motion between. Recording poses live (cheap) and rendering
        from the recording afterwards (slow, but offline) separates the two: the sand sees every
        sample, and the video sees every frame.

        ⚠ Raw wire blocks, not a parsed schema. The registry carries hulls, materials and the
        region; re-deriving a storage format for all that would be a second wire format to keep in
        step with the first. `MpmStateBlock`/`MpmRegistryBlock` already describe it exactly.
        """
        if not self.args.record_poses or not self._recorded_states:
            return
        import numpy as np

        path = os.path.abspath(self.args.record_poses)
        np.savez_compressed(
            path,
            registry=np.frombuffer(self._recorded_registry, dtype=np.uint8),
            states=np.stack([np.frombuffer(b, dtype=np.uint8) for b in self._recorded_states]),
        )
        size = os.path.getsize(path if path.endswith(".npz") else path + ".npz")
        print(f"recorded {len(self._recorded_states)} collider samples -> {path} "
              f"({size / 1048576:.1f} MiB)")

    def close(self) -> None:
        for segment in (self.registry_seg, self.state_seg, self.status_seg, self.particle_seg):
            if segment is not None:
                segment.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dir", default="/dev/shm", help="directory holding the MPM segments")
    parser.add_argument("--voxel-size", type=float, default=0.05, help="MPM voxel size, metres")
    parser.add_argument("--fps", type=float, default=60.0, help="MPM frames per simulated second")
    parser.add_argument("--gravity", type=float, default=-9.81)
    parser.add_argument("--density", type=float, default=2500.0, help="sand density, kg/m^3")
    parser.add_argument("--patch-x", type=float, default=0.0)
    parser.add_argument("--patch-y", type=float, default=0.0)
    parser.add_argument("--patch-z", type=float, default=0.0)
    parser.add_argument("--patch-size", type=float, default=1.0,
                        help="half-extent of the sand patch in x and y, metres")
    parser.add_argument("--patch-depth", type=float, default=0.2, help="sand depth, metres")
    parser.add_argument("--ground-z", type=float, default=None,
                        help="ground plane height. Defaults to the BOTTOM of the sand patch "
                             "(patch-z - patch-depth/2), which is where the sand needs support.")
    parser.add_argument("--ground-friction", type=float, default=0.5)
    # ⚠ D13 MATERIAL. These decide whether a vehicle climbs at all — see the note at the shape
    # creation in build() for the measured 0.5/0.8/1.2 sweep — and until 2026-08-26 none of them
    # was reachable from a settings file.
    parser.add_argument("--collider-friction", type=float, default=0.0, metavar="MU",
                        help="override wheel-on-sand friction for EVERY collider, ignoring what "
                             "the simulator reported. 0 = use the wire's per-collider value.")
    parser.add_argument("--collider-friction-default", type=float, default=0.5, metavar="MU",
                        help="used for a collider whose backend reported no material. 0.5 is "
                             "Newton's default and the SMOOTH-wheel value; a lugged wheel shears "
                             "soil rather than sliding on it and sits at or above the sand's own "
                             "internal friction.")
    parser.add_argument("--sand-friction", type=float, default=0.0, metavar="MU",
                        help="the sand's INTERNAL friction, which sets its angle of repose "
                             "(atan(mu): 0.5 -> 26.6 deg, measured 26.0-26.2 deg). 0 = Newton's "
                             "default of 0.5. ⚠ A vehicle can only climb a slope up to "
                             "atan(collider friction), so sand friction >= wheel friction means "
                             "the heap builds a face the wheels cannot hold on.")
    parser.add_argument("--camera-distance", type=float, default=0.0,
                        help="metres back from the sand centre. 0 derives it from the patch size.")
    parser.add_argument("--camera-pitch", type=float, default=25.0,
                        help="degrees below horizontal")
    parser.add_argument("--camera-yaw", type=float, default=0.0)
    parser.add_argument("--max-render-particles", type=int, default=40000,
                        help="cap on particles published for Unreal to render. The full field is "
                             "reported alongside so a decimated view never implies it is complete.")
    parser.add_argument("--record-poses", default=None, metavar="PATH",
                        help="capture every consumed collider state to PATH (.npz) so the run can "
                             "be replayed later. Cheap: a few hundred KB per second.")
    parser.add_argument("--replay-poses", default=None, metavar="PATH",
                        help="drive from a recorded file instead of shared memory. No simulator "
                             "needed, and every recorded sample is consumed — so a replay renders "
                             "at FULL fidelity where the live run had to skip.")
    parser.add_argument("--jump-warn", type=float, default=0.05,
                        help="warn once when a collider moves more than this (metres) between two "
                             "solves — the sand never saw the motion in between")
    parser.add_argument("--particle-every", type=int, default=1,
                        help="publish a render snapshot every N solves")
    parser.add_argument("--max-steps", type=int, default=0, help="stop after N solves; 0 = forever")
    parser.add_argument("--two-way", action="store_true",
                        help="publish the sand's reaction impulses back to the simulator (plan "
                             "M3/M4). ⚠ EXPERIMENTAL and off by default: the effective-inertia "
                             "question in plan 11.1 is unresolved, so a run using this must be "
                             "recorded as LaggedImpulseTwoWay, never as TwoWay.")
    # ⚠ THE COLLIDER MASS IS THE TWO-WAY SWITCH. See the setup_collider call in build() for the
    # measured numbers; in short, "kinematic" solves the sand against an immovable wall and returns
    # an impulse sized for infinite inertia, which is what ejected the rover.
    parser.add_argument("--collider-mass", choices=["kinematic", "real", "auto"], default="auto",
                        help="mass the sand's contact solve believes each collider has. "
                             "'kinematic' = 0 = infinite (one-way M2 behaviour); 'real' = the mass "
                             "the simulator reported on the wire; 'auto' (default) = 'real' when "
                             "--two-way is set, 'kinematic' otherwise.")
    parser.add_argument("--collider-mass-scale", type=float, default=1.0,
                        help="multiply the wire masses. A link's own mass is NOT its articulated "
                             "effective mass (plan 11.1); this is the knob for compensating until "
                             "a backend can report the real quantity.")
    parser.add_argument("--no-collider-cull", action="store_true",
                        help="spawn the bed straight through any collider standing in it, the "
                             "pre-2026-08-26 behaviour. Those particles are trapped inside the "
                             "collider for the life of the run and make it look transparent.")
    parser.add_argument("--collider-mass-total", type=float, default=0.0, metavar="KG",
                        help="total mass the sand's contact solve should see across ALL colliders, "
                             "spread in proportion to their link masses. This is the vehicle's "
                             "mass, not the mass per wheel: the effective inertia resisting sand at "
                             "one wheel is its SHARE of the vehicle. Overrides "
                             "--collider-mass-scale. 0 disables.")
    parser.add_argument("--collider-mass-floor", type=float, default=0.05, metavar="KG",
                        help="fallback for a collider whose reported mass is zero, so one missing "
                             "value cannot silently restore infinite inertia on that link")
    parser.add_argument("--parent-pid", type=int, default=0,
                        help="exit when this process disappears. Set by the simulator when it owns "
                             "this sidecar (plan D14), so an editor crash cannot strand a process "
                             "holding GPU memory. 0 disables the check.")
    parser.add_argument("--render-to", default=None, metavar="PATH",
                        help="render offscreen and encode an H.264 video here. ⚠ This machine's X "
                             "session has no hardware GLX (llvmpipe), so an interactive viewer is "
                             "useless and this is the way to actually see the sand.")
    parser.add_argument("--render-every", type=int, default=4,
                        help="capture one frame every N solves. Rasterising is CPU-bound here, so "
                             "capturing every solve makes the sidecar the slow part of the link.")
    args = parser.parse_args()

    # ⚠ RESOLVED ONCE, HERE, so every later read sees a concrete choice rather than "auto".
    # Two-way with kinematic colliders is the combination that ejected the rover: the sand is
    # solved against infinite inertia and the resulting impulse is applied to a light wheel. It is
    # still reachable with an explicit --collider-mass kinematic, because reproducing the old
    # behaviour is how the fix gets measured.
    if args.collider_mass == "auto":
        # ⚠ ALWAYS 'kinematic', INCLUDING WITH --two-way. This briefly resolved to 'real' when
        # two-way was on, on the theory that infinite collider mass was what produced 1443 N
        # reactions. It does cut the force — to ~24 N — but it buys that by making the wheel SOFT
        # to the sand, and a soft contact leaks: sand passes through the wheel instead of being
        # pushed by it. Compare Newton-MPM-Discussions/m0/exomy_box3d_run{1,2}.mp4, recorded on
        # the kinematic path, where the bed is visibly displaced by the wheels.
        #
        # Non-penetration and reaction magnitude are SEPARATE problems. A wheel is a boundary
        # whatever it weighs; mass governs how much it recoils. Trading the boundary away to damp
        # the reaction loses the geometry, which is the part that is visible and the part
        # terramechanics depends on. The force problem belongs on the simulator side, where
        # MpmImpulseMaxForce, MpmImpulseRelaxation and force-hold already live — and in the initial
        # condition, since the rover being spawned buried at the bed floor is what forces a wheel
        # through 0.2 m of sand in the first place.
        args.collider_mass = "kinematic"
        print("--collider-mass auto -> 'kinematic' (hard boundary; see the note in main())")

    sidecar = Sidecar(args)
    try:
        return sidecar.run_replay() if args.replay_poses else sidecar.run()
    except P.SegmentError as error:
        print(f"LINK ERROR: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 0
    finally:
        # ⚠ Encode on EVERY exit path, including Ctrl-C and a fault. A run that produced 400 frames
        # and then threw should still hand back the 400 frames — they are usually what shows why.
        try:
            sidecar.save_recording()
            sidecar._encode_video()
        except Exception as error:
            print(f"could not encode video: {error}", file=sys.stderr)
        sidecar.close()


if __name__ == "__main__":
    sys.exit(main())
