#include "CoordinatedPhysicsScene.h"

DEFINE_LOG_CATEGORY(LogPhysicsCoordinator);

#include <stdexcept>
#include <utility>

namespace
{
/// Scene participants advance the solver; vehicle participants latch controls before it and read
/// state after it. The coordinator runs every phase in (order, id) order, so the vehicles' pre-step
/// hook must sort before the scenes' step, and their post-step hook then reads a world that has
/// already advanced.
constexpr int kVehicleParticipantOrder = 0;
constexpr int kSceneParticipantOrder = 10;
} // namespace

FCoordinatedPhysicsScene::FCoordinatedPhysicsScene(
    msr::airlib::PhysicsSceneCoordinator& coordinator,
    const AirSimSettings::PhysicsCoordinatorSetting& settings, double fixed_step_seconds)
    : coordinator_(coordinator), settings_(settings), fixed_step_seconds_(fixed_step_seconds)
{
    if (!(fixed_step_seconds_ > 0.0))
        throw std::invalid_argument("FCoordinatedPhysicsScene: fixed step must be positive");
}

void FCoordinatedPhysicsScene::setGroundHeightField(const urdf::BackendOptions::HeightField& field)
{
    if (!field.valid())
        throw std::invalid_argument(
            "FCoordinatedPhysicsScene: the sampled ground height field is not well formed");
#if WITH_MUJOCO_BINDING
    if (mujoco_)
        throw std::logic_error(
            "FCoordinatedPhysicsScene: the ground must be set before the first scene that stands "
            "on it is created");
#endif
    ground_height_field_ = field;
}

bool FCoordinatedPhysicsScene::claimKinematicMirror()
{
    if (kinematic_mirror_claimed_)
        return false;
    kinematic_mirror_claimed_ = true;
    return true;
}

void FCoordinatedPhysicsScene::registerParticipant(
    const std::string& stable_id, int order,
    std::shared_ptr<msr::airlib::PhysicsSceneParticipant> participant)
{
    coordinator_.registerParticipant(stable_id, order, std::move(participant));
}

#if WITH_BOX3D_BINDING
b3urdf::Box3DPhysicsScene& FCoordinatedPhysicsScene::box3dScene()
{
    if (!box3d_) {
        b3urdf::Box3DSceneOptions options;
        // The scene's solver step IS the world's authoritative step. Anything else would make one
        // coordinated tick mean a different amount of simulated time in this scene than in the next.
        options.fixed_timestep = fixed_step_seconds_;

        // The world's one authored floor, from world settings rather than from whichever robot
        // happened to ask for the scene first.
        const auto& ground = settings_.ground_plane;
        options.add_ground_plane =
            ground.mode == AirSimSettings::CoordinatedGroundPlaneMode::Explicit;
        options.ground_plane_z = ground.z;
        options.ground_plane_half_extent = ground.half_extent;
        options.ground_friction = ground.friction;

        box3d_ = std::make_shared<urdf::Box3DSceneParticipant>(options);
        coordinator_.registerParticipant("scene/box3d", kSceneParticipantOrder, box3d_);

        UE_LOG(LogPhysicsCoordinator, Log,
               TEXT("shared Box3D world created: fixed step %.6f s, ground plane %s"),
               options.fixed_timestep,
               options.add_ground_plane ? TEXT("AUTHORED") : TEXT("none (static mirror only)"));
    }
    return box3d_->scene();
}

void FCoordinatedPhysicsScene::publishBox3DBody(const std::string& stable_id, size_t scene_handle)
{
    if (!box3d_)
        throw std::logic_error(
            "FCoordinatedPhysicsScene: a Box3D body was published before any Box3D scene existed");
    box3d_->registerRobotBody(stable_id, scene_handle);
    published_bodies_.push_back(PublishedBody{ stable_id, "scene/box3d", "Box3D" });
}
#endif

#if WITH_MUJOCO_BINDING
urdf::MuJoCoPhysicsScene& FCoordinatedPhysicsScene::mujocoScene()
{
    if (!mujoco_) {
        urdf::MuJoCoPhysicsScene::Options options;
        // The scene's solver step IS the world's authoritative step, as for Box3D: one coordinated
        // tick must mean the same amount of simulated time in every scene of the world.
        options.fixed_timestep = fixed_step_seconds_;

        // ⚠ MuJoCo cannot cook the level's concave ground - it hulls every mesh - so unlike Box3D
        // it genuinely needs a ground of its own. The world supplies exactly one, authored: a
        // height field if the operator described one, else the authored plane. There is no probe,
        // because a trace under one robot is not a shared world's floor.
        const auto& ground = settings_.ground_plane;
        options.add_ground_plane =
            ground.mode == AirSimSettings::CoordinatedGroundPlaneMode::Explicit;
        options.ground_plane_z = ground.z;
        // The height field wins where one was sampled: it is exact for slopes and steps, while the
        // plane is a single height and is only a fallback.
        options.ground_height_field = ground_height_field_;

        // ⚠ BOUND THE LEVEL, or MuJoCo compiles tens of thousands of meshes. MuJoCo hulls every
        // mesh geom, so a concave level object is emitted as one thin convex prism PER TRIANGLE -
        // exact, and expensive in geom count. The measured Blocks-style mirror is ~172 bodies and
        // ~39,700 triangles; unclipped that is a model with 20,000 mesh assets after the cap, which
        // takes minutes to compile and steps slowly.
        //
        // The legacy per-robot path bounds it with a 30 m radius around each robot. A shared world
        // has no single robot to centre on, so it is bounded by the region whose ground was
        // actually sampled: geometry beyond that has no floor under it anyway.
        //
        // Split (not Auto) matches the per-vehicle default, so a coordinated run and a legacy run
        // cook the level the same way and remain comparable.
        options.static_mesh_mode = urdf::BackendOptions::StaticMeshMode::Split;
        options.static_world_max_triangles = 20000;
        if (ground_height_field_.valid()) {
            options.static_world_radius = ground_height_field_.half_extent;
            options.static_world_clip_center =
                urdf::Vec3{ ground_height_field_.center_x, ground_height_field_.center_y, 0.0 };
        }
        else {
            options.static_world_radius = 0.0;
            UE_LOG(LogPhysicsCoordinator, Warning,
                   TEXT("no sampled ground region, so the mirrored level is NOT clipped: MuJoCo "
                        "will emit one prism per level triangle up to the %d cap, which is slow to "
                        "compile and to step. Set PhysicsCoordinator.GroundHeightField to bound it."),
                   options.static_world_max_triangles);
        }

        mujoco_ = std::make_shared<urdf::MuJoCoSceneParticipant>(options);
        coordinator_.registerParticipant("scene/mujoco", kSceneParticipantOrder, mujoco_);

        UE_LOG(LogPhysicsCoordinator, Log,
               TEXT("shared MuJoCo scene created: fixed step %.6f s, ground %s"),
               options.fixed_timestep,
               options.ground_height_field.valid() ? TEXT("SAMPLED HEIGHT FIELD")
               : options.add_ground_plane        ? TEXT("AUTHORED PLANE")
                                                 : TEXT("none yet"));
        if (!options.add_ground_plane && !options.ground_height_field.valid()) {
            // Said loudly: MuJoCo with no ground is a robot that falls forever, and the level
            // mirror alone cannot supply one for it.
            UE_LOG(LogPhysicsCoordinator, Warning,
                   TEXT("the shared MuJoCo scene has NO authored ground. MuJoCo cannot represent "
                        "the level's concave ground mesh, so its robots will fall unless "
                        "PhysicsCoordinator.GroundPlane (or a GroundHeightField) is set."));
        }
    }
    return mujoco_->scene();
}

void FCoordinatedPhysicsScene::publishMuJoCoBody(
    const std::string& stable_id, urdf::MuJoCoPhysicsScene::ArticulationHandle articulation)
{
    if (!mujoco_)
        throw std::logic_error(
            "FCoordinatedPhysicsScene: a MuJoCo body was published before any MuJoCo scene existed");
    mujoco_->registerArticulationBody(stable_id, articulation);
    published_bodies_.push_back(PublishedBody{ stable_id, "scene/mujoco", "MuJoCo" });
}
#endif

bool FCoordinatedPhysicsScene::collisionDebugGeometry(const urdf::CollisionDebugFilter& filter,
                                                      urdf::CollisionDebugSnapshot& out) const
{
    out = urdf::CollisionDebugSnapshot();

    // ⚠ Reads the PARTICIPANT'S scene, not `mujocoScene()`/`box3dScene()` - those create a scene on
    // first use, and a debug overlay toggled on before any vehicle joined must not be the thing
    // that brings a solver world into existence.
#if WITH_MUJOCO_BINDING
    if (mujoco_) {
        mujoco_->scene().collisionDebugGeometry(filter, out);
        return !out.geoms.empty() || out.omitted > 0;
    }
#endif
#if WITH_BOX3D_BINDING
    if (box3d_) {
        box3d_->scene().collisionDebugGeometry(filter, out);
        return !out.geoms.empty() || out.omitted > 0;
    }
#endif
    return false;
}

void FCoordinatedPhysicsScene::commitManifest()
{
    if (published_bodies_.empty())
        throw std::runtime_error(
            "PhysicsCoordinator.Mode is coordinated but no vehicle joined a shared physics scene. "
            "Declare at least one urdfbot, or use PhysicsCoordinator.Mode=Legacy.");

    coordinator_.beginManifestUpdate();
    for (const PublishedBody& body : published_bodies_) {
        msr::airlib::PhysicsManifestEntry entry;
        entry.stable_id = body.stable_id;
        entry.participant_id = body.participant_id;
        entry.source_id = body.stable_id;
        // Frozen with the baseline so a run's authority map is recorded rather than reconstructed.
        entry.baseline_properties["authority"] = body.backend;
        entry.baseline_properties["kind"] = "UrdfArticulation";
        coordinator_.stageManifestEntry(std::move(entry));
    }

    // Commit seals every scene's topology through its participant's manifest hook.
    coordinator_.commitManifest();

    UE_LOG(LogPhysicsCoordinator, Log,
           TEXT("manifest committed: revision %llu, %d bodies, world id %llu/%llu"),
           static_cast<unsigned long long>(coordinator_.stamp().manifest_revision),
           static_cast<int32>(published_bodies_.size()),
           static_cast<unsigned long long>(coordinator_.stamp().world.id),
           static_cast<unsigned long long>(coordinator_.stamp().world.revision));
    for (const PublishedBody& body : published_bodies_) {
        UE_LOG(LogPhysicsCoordinator, Log, TEXT("  %s -> %s, solved by %s"),
               UTF8_TO_TCHAR(body.stable_id.c_str()), UTF8_TO_TCHAR(body.backend.c_str()),
               UTF8_TO_TCHAR(body.participant_id.c_str()));
    }

#if WITH_MUJOCO_BINDING
    // ⚠ WHAT THE LEVEL BECAME, said out loud. `emitStaticWorld` fills every one of these counters
    // from the code that does the work, and until now not one of them was printed on the
    // coordinated path: a run could clip away the entire level, or stop at the triangle cap, and
    // the log would show only "shared MuJoCo scene created". The per-robot path has reported this
    // since it existed; the shared path inherited the emitter and not its reporting.
    if (mujoco_) {
        const urdf::StaticWorldEmitStats& stats = mujoco_->scene().staticWorldStats();
        UE_LOG(LogPhysicsCoordinator, Log,
               TEXT("shared MuJoCo level: %d geoms (%d whole convex objects, %d enclosures split), ")
               TEXT("%d triangles emitted, %d clipped away, %d skipped at the cap, %d shapes ")
               TEXT("dropped, ground %s, largest object %.2f m ('%s'), furthest vertex %.1f m"),
               static_cast<int32>(stats.geoms_emitted), static_cast<int32>(stats.convex_objects),
               static_cast<int32>(stats.enclosures), static_cast<int32>(stats.triangles_emitted),
               static_cast<int32>(stats.triangles_clipped_away),
               static_cast<int32>(stats.triangles_skipped),
               static_cast<int32>(stats.shapes_dropped),
               stats.used_height_field ? TEXT("height field") : TEXT("plane or none"),
               stats.worst_span, UTF8_TO_TCHAR(stats.worst_span_body.c_str()), stats.worst_vertex);

        if (stats.triangles_skipped > 0)
            UE_LOG(LogPhysicsCoordinator, Warning,
                   TEXT("%d level triangles were DROPPED at the max-triangle cap - the mirrored ")
                   TEXT("world is incomplete and a robot will drive through something visible. ")
                   TEXT("Narrow PhysicsCoordinator.GroundHeightField, or raise the cap."),
                   static_cast<int32>(stats.triangles_skipped));
        if (stats.shapes_dropped > 0)
            UE_LOG(LogPhysicsCoordinator, Warning,
                   TEXT("%d level shapes were REFUSED as degenerate or non-finite and are not in ")
                   TEXT("the solver at all."),
                   static_cast<int32>(stats.shapes_dropped));
        if (stats.geoms_emitted == 0)
            UE_LOG(LogPhysicsCoordinator, Error,
                   TEXT("the shared MuJoCo scene has NO level geometry whatsoever. Whatever the ")
                   TEXT("mirror reported, nothing reached the solver."));
    }
#endif
}

std::string FCoordinatedPhysicsScene::describePopulation() const
{
    std::string report;
    for (const PublishedBody& body : published_bodies_)
        report += (report.empty() ? "" : "\n") + body.stable_id + " -> " + body.backend +
                  " (" + body.participant_id + ")";
    return report;
}
