// A stand-in simulator for the MPM sidecar, so the link can be exercised without Unreal.
//
// ⚠ EXISTS BECAUSE THE EDITOR IS THE EXPENSIVE WAY TO TEST A PROTOCOL. Bringing up Unreal to find
// out that a pose landed in the wrong field costs minutes per attempt; this costs seconds and
// drives the same publisher the sim will. It is a test fixture, not a simulator: the motion is
// scripted, and nothing here solves anything.
//
// Publishes one sphere collider descending into the sand patch, then rising back out.
// --motion updown (default) does exactly that on a vertical line, unchanged from the version
// this fixture shipped with. --motion circle does the same descend/hold/rise but sweeps the
// sphere around a horizontal circle throughout, so it ploughs a curved furrow instead of a
// single straight pit — see the phase-design comment above the main loop below.
#include "mpm/MpmSidecarPublisher.hpp"
#include "urdf/UrdfPhysicsDescriptor.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

using namespace msr::airlib::mpm;

int main(int argc, char** argv)
{
    std::string dir = "/dev/shm";
    double dt = 0.003;
    int steps = 4000;
    double radius = 0.15;
    bool lockstep = false;
    std::string motion = "updown";
    // Defaults sized for the recipe's patch (--patch-size 0.6 => a 1.2 m-across patch, half-extent
    // 0.6 m). A 0.30 m circle plus the 0.15 m sphere reaches 0.45 m from centre, comfortably inside
    // the patch with margin, and is far enough off-centre to carve a real ring rather than just
    // stirring the middle voxel column.
    double circleRadius = 0.30;
    double circleRevs = 3.0;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--dir" && i + 1 < argc) dir = argv[++i];
        else if (arg == "--steps" && i + 1 < argc) steps = std::atoi(argv[++i]);
        else if (arg == "--radius" && i + 1 < argc) radius = std::atof(argv[++i]);
        else if (arg == "--lockstep") lockstep = true;
        else if (arg == "--motion" && i + 1 < argc) motion = argv[++i];
        else if (arg == "--circle-radius" && i + 1 < argc) circleRadius = std::atof(argv[++i]);
        else if (arg == "--circle-revs" && i + 1 < argc) circleRevs = std::atof(argv[++i]);
    }

    MpmSidecarPublisher publisher;
    MpmSidecarPublisher::Options options;
    options.directory = dir;
    if (!publisher.open(options)) {
        std::fprintf(stderr, "cannot open MPM segments in %s\n", dir.c_str());
        return 1;
    }

    urdf::PhysicsColliderSet robot;
    robot.backend = "fake";
    urdf::PhysicsColliderDescriptor probe;
    probe.stable_id = "FakeRover/probe";
    probe.link_name = "probe";
    probe.inertial.mass = 5.0;
    probe.inertial.provenance = urdf::InertiaProvenance::SolverRealised;
    probe.inertial.is_articulated_effective_inertia = false;
    probe.role = urdf::CouplingRole::KinematicOneWay;
    probe.material.friction = 0.8;
    probe.material.reported = true;
    urdf::CollisionShape sphere;
    sphere.kind = urdf::CollisionShape::Kind::Sphere;
    sphere.radius = radius;
    probe.shapes.push_back(sphere);
    robot.colliders.push_back(probe);

    std::vector<urdf::PhysicsColliderSet> robots{ robot };
    const std::vector<std::string> selected{ "FakeRover/probe" };

    WireWorldStamp stamp;
    stamp.world_id = 1;
    stamp.world_revision = 1;
    stamp.manifest_revision = 1;
    stamp.reset_epoch = 0;

    if (!publisher.publishRegistry(stamp, dt, robots, selected)) {
        std::fprintf(stderr, "registry publish reported a problem\n");
        return 1;
    }
    std::printf("registered %zu collider(s); driving for %d steps at %.4f s\n",
                publisher.publishedIds().size(), steps, dt);

    // Circle motion is split into three phases that sum to the whole run: descend while turning,
    // hold at depth while turning (this is the phase that actually ploughs the furrow — one pass
    // just draws a groove, staying down for multiple revolutions is what makes it read as a track
    // in the video), then rise while turning. Phases are sized as FRACTIONS of the total run time
    // rather than fixed seconds, so `--steps` can be changed without the motion running off the end
    // mid-phase or sitting idle at the top for the last chunk of the video (an earlier draft used
    // fixed 2.5/5/2.5 s phases against a hardcoded 10 s total; that quietly desynced the moment
    // --steps or --lockstep changed the run's actual duration).
    // ⚠ In lockstep one published step IS one MPM solve, so sim time must advance by the sidecar's
    // frame (1/60 s), not by the rigid tick (3 ms) — otherwise the probe crawls and 250 solves cover
    // 0.75 s of its 6 s descent. runSeconds below uses the same rule to size the phases correctly
    // for whichever mode is active.
    const double runSeconds = steps * (lockstep ? 1.0 / 60.0 : dt);
    const double descendSeconds = 0.25 * runSeconds;
    const double holdSeconds = 0.50 * runSeconds;
    const double riseSeconds = 0.25 * runSeconds;
    const double zTop = 0.35;    // matches updown's start/end height, above the sand surface
    const double zBottom = 0.05; // matches updown's deepest point, proven to plough visibly
    const double omega = circleRevs * 2.0 * 3.14159265 / std::max(runSeconds, 1e-6);
    const double pi = 3.14159265;

    const auto wall_start = std::chrono::steady_clock::now();
    for (int step = 0; step < steps; ++step) {
        const double t = step * (lockstep ? 1.0 / 60.0 : dt);

        double z, vz, theta, omegaNow;
        if (motion == "circle") {
            // z(t): eased (half-cosine) ramps so dz/dt is zero at every phase boundary — matching
            // velocity across boundaries avoids the impulse a kinked z(t) would hand the contact
            // law, same reasoning as updown's use of sin() instead of a step function.
            if (t >= runSeconds) {
                // Past the end of the planned motion (only reachable if this fixture is driven for
                // longer than the phases were sized for, e.g. non-lockstep with the default 4000
                // steps). Freeze at the top with zero velocity rather than reporting a nonzero
                // "commanded" speed for a collider that has actually stopped.
                z = zTop;
                vz = 0.0;
                theta = omega * runSeconds;
                omegaNow = 0.0;
            }
            else if (t < descendSeconds) {
                const double u = t / std::max(descendSeconds, 1e-9);
                const double ease = 0.5 * (1.0 - std::cos(pi * u));
                z = zTop - (zTop - zBottom) * ease;
                vz = -(zTop - zBottom) * 0.5 * pi * std::sin(pi * u) / descendSeconds;
                theta = omega * t;
                omegaNow = omega;
            }
            else if (t < descendSeconds + holdSeconds) {
                z = zBottom;
                vz = 0.0;
                theta = omega * t;
                omegaNow = omega;
            }
            else {
                const double u = (t - descendSeconds - holdSeconds) / std::max(riseSeconds, 1e-9);
                const double ease = 0.5 * (1.0 - std::cos(pi * u));
                z = zBottom + (zTop - zBottom) * ease;
                vz = (zTop - zBottom) * 0.5 * pi * std::sin(pi * u) / riseSeconds;
                theta = omega * t;
                omegaNow = omega;
            }
            const double x = circleRadius * std::cos(theta);
            const double y = circleRadius * std::sin(theta);
            const double vx = -circleRadius * omegaNow * std::sin(theta);
            const double vy = circleRadius * omegaNow * std::cos(theta);
            robots[0].colliders[0].position = urdf::Vec3{ x, y, z };
            robots[0].colliders[0].linear_velocity = urdf::Vec3{ vx, vy, vz };
        }
        else {
            // Down into the sand over 3 s, hold, then back out. Sinusoidal so the velocity is
            // smooth; a step change would be an impulse the contact law has no honest answer for.
            z = 0.35 - 0.30 * std::sin(std::min(t, 6.0) * 3.14159265 / 6.0);
            robots[0].colliders[0].position = urdf::Vec3{ 0.0, 0.0, z };
            robots[0].colliders[0].linear_velocity =
                urdf::Vec3{ 0.0, 0.0,
                            -0.30 * (3.14159265 / 6.0) * std::cos(std::min(t, 6.0) * 3.14159265 / 6.0) };
        }

        publisher.publishState(stamp, static_cast<uint64_t>(step), t, robots);

        if (step % 200 == 0) {
            const auto health = publisher.health(stamp);
            std::printf("[step %5d  z=%.3f]  %s\n", step, z, health.describe().c_str());
            std::fflush(stdout);
        }
        if (lockstep) {
            // ⚠ FIXTURE-ONLY, and NOT how the simulator behaves. Production never blocks on the
            // sidecar — that would make the rigid solver run at the sand's speed, which is the
            // whole reason the link is asynchronous. This mode exists because a render-bound
            // sidecar consuming latest-wins state sees the collider TELEPORT between distant
            // points of its path, so it barely disturbs the sand and the recording shows nothing.
            // Waiting for the acknowledgement makes the probe advance one sim step per solve.
            //
            // It also exercises the acknowledgement as flow control, which is worth knowing works.
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
            while (publisher.health(stamp).acknowledged_step < static_cast<uint64_t>(step)) {
                if (std::chrono::steady_clock::now() > deadline) {
                    std::fprintf(stderr, "lockstep: sidecar did not acknowledge step %d in 30 s\n",
                                 step);
                    return 1;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
        else {
            // Real time, so the sidecar's lag is a real measurement rather than an artefact of
            // this fixture racing ahead.
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long>(dt * 1e6)));
        }
    }

    const auto health = publisher.health(stamp);
    const double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    std::printf("\nfinished %d steps in %.1f s wall\nfinal: %s\n", steps, wall,
                health.describe().c_str());
    return health.responsive ? 0 : 1;
}
