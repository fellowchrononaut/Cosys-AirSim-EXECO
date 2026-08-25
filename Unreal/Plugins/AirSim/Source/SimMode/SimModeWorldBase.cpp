#include "SimModeWorldBase.h"
#include "CoordinatedPhysicsScene.h"
#include "PhysicsCollisionDebugDraw.h"
#include "Camera/PlayerCameraManager.h"
#include "GameFramework/PlayerController.h"
#include "Vehicles/UrdfBot/UrdfTransform.h"
#include "physics/FastPhysicsEngine.hpp"
#include "physics/ExternalPhysicsEngine.hpp"
#include "common/ClockFactory.hpp"
#include "common/SteppableClock.hpp"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Engine/World.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <exception>
#include <stdexcept>
#include <utility>
#include "AirBlueprintLib.h"
#include "PhysicsTiming.h"

namespace
{
using PhysicsCoordinatorMode = msr::airlib::AirSimSettings::PhysicsCoordinatorMode;
using PhysicsSceneCoordinator = msr::airlib::PhysicsSceneCoordinator;

PhysicsSceneCoordinator::Mode toCoordinatorMode(PhysicsCoordinatorMode mode)
{
    switch (mode) {
    case PhysicsCoordinatorMode::Legacy:
        return PhysicsSceneCoordinator::Mode::Legacy;
    case PhysicsCoordinatorMode::CoordinatedSingleBackend:
        return PhysicsSceneCoordinator::Mode::SingleBackend;
    case PhysicsCoordinatorMode::MixedBackendExperimental:
        return PhysicsSceneCoordinator::Mode::MixedBackendExperimental;
    }

    throw std::invalid_argument("Unknown PhysicsCoordinator.Mode value");
}

const char* coordinatorModeName(PhysicsCoordinatorMode mode)
{
    switch (mode) {
    case PhysicsCoordinatorMode::Legacy:
        return "Legacy";
    case PhysicsCoordinatorMode::CoordinatedSingleBackend:
        return "CoordinatedSingleBackend";
    case PhysicsCoordinatorMode::MixedBackendExperimental:
        return "MixedBackendExperimental";
    }

    return "Unknown";
}

uint64_t stableWorldId(const std::string& logical_world_name)
{
    // FNV-1a is explicit here rather than std::hash, whose result is not required to be stable.
    uint64_t value = 14695981039346656037ULL;
    for (const unsigned char character : logical_world_name) {
        value ^= character;
        value *= 1099511628211ULL;
    }
    return value;
}

msr::airlib::PhysicsWorldIdentity makeWorldIdentity(const UWorld* world)
{
    FString logical_world_name = world == nullptr ? TEXT("<no-world>") : world->GetMapName();
    if (world != nullptr)
        logical_world_name.RemoveFromStart(world->StreamingLevelsPrefix);

    static std::atomic<uint64_t> next_world_revision(1);
    return msr::airlib::PhysicsWorldIdentity{
        stableWorldId(TCHAR_TO_UTF8(*logical_world_name)),
        next_world_revision.fetch_add(1)
    };
}

/// The one authoritative simulation timestep for a coordinated world.
///
/// It is READ FROM THE INSTALLED CLOCK rather than recomputed from the physics loop period. Those
/// two are deliberately different numbers once ClockSpeed > 1: the sim mode divides the executor's
/// wall period so ticks arrive faster, while each tick must still advance simulation time by the
/// unscaled step. Recomputing here would silently integrate ClockSpeed into dt and change the
/// trajectory of every run that speeds the wall clock up.
msr::airlib::TTimeDelta coordinatedFixedStepSeconds()
{
    // ⚠ Asked of the clock, not decided by a downcast: this module is compiled with RTTI off.
    const msr::airlib::TTimeDelta step = msr::airlib::ClockFactory::get()->fixedStepSeconds();
    if (!(step > 0) || !std::isfinite(step)) {
        throw std::runtime_error(
            "Coordinated physics requires a fixed-step clock, but the installed clock reports no "
            "fixed step (it is wall-derived). This sim mode overrides setupClockSpeed() and must "
            "honour AirSimSettings::clock_type, which coordinated modes force to SteppableClock.");
    }
    return step;
}

/// Convert the world-level pre-settle policy into whole authoritative steps.
///
/// The count is computed once, here, so the reset transaction cannot disagree with the world's dt.
/// A requested interval shorter than one step still costs one step: performing zero steps while
/// reporting `Presettle: true` would misdescribe the recorded initial condition.
msr::airlib::PhysicsResetPolicy makeResetPolicy(
    const msr::airlib::AirSimSettings::PhysicsCoordinatorSetting& setting,
    msr::airlib::TTimeDelta fixed_step_seconds)
{
    msr::airlib::PhysicsResetPolicy policy;
    policy.presettle = setting.presettle;
    if (!policy.presettle)
        return policy;

    policy.presettle_dt = fixed_step_seconds;
    const double steps = std::ceil(setting.presettle_seconds / fixed_step_seconds);
    policy.presettle_steps = steps < 1.0 ? 1u : static_cast<uint64_t>(steps);
    return policy;
}

/// Refuse a coordinated run whose declared population no shared scene can actually own.
///
/// ⚠ Checked BEFORE any vehicle is constructed, and it refuses rather than degrades. A partly
/// coordinated world is the failure this whole design exists to prevent: one robot in a shared
/// scene and another in a private one look identical in the log, produce a run labelled as a clean
/// comparison, and silently answer a different physics question.
/// Sample the world's ONE ground height field by downward traces.
///
/// ⚠ TRACES, NOT MIRRORED GEOMETRY. Unreal answers "how high is the ground here" exactly, for
/// landscape, static meshes and BSP alike, with no mirroring and no convex approximation. Deriving
/// a floor from mirrored geometry failed repeatedly on a map whose ground is one 40 km mesh with a
/// bounding box 442 m tall around a surface at z = 0.36.
///
/// ⚠ WORLD-LEVEL, not robot-level. The legacy path samples a patch centred on each robot and
/// recentres it as that robot drives; in one shared scene that would move the floor for everybody,
/// and two overlapping patches would contact the same foot twice.
bool sampleWorldHeightField(UWorld* unreal_world, float world_to_meters,
                            const msr::airlib::AirSimSettings::GroundHeightFieldSetting& setting,
                            double center_x, double center_y, double half_extent,
                            urdf::BackendOptions::HeightField& out)
{
    const int samples = setting.samplesPerSide(half_extent);
    if (samples < 2 || unreal_world == nullptr)
        return false;

    out = urdf::BackendOptions::HeightField();
    out.rows = samples;
    out.cols = samples;
    out.center_x = center_x;
    out.center_y = center_y;
    out.half_extent = half_extent;
    out.heights.assign(static_cast<size_t>(samples) * samples, 0.0f);

    FCollisionQueryParams params(FName(TEXT("CoordinatedGroundGrid")), /*bTraceComplex=*/true);
    std::vector<double> z(static_cast<size_t>(samples) * samples,
                          std::numeric_limits<double>::quiet_NaN());
    double lowest = std::numeric_limits<double>::max();
    int hits = 0;

    // Deliberately far above and far below anything a level contains: the sampler must not need to
    // know the map's vertical extent to find its floor.
    const float top = static_cast<float>(5000.0 * world_to_meters);
    const float bottom = static_cast<float>(-5000.0 * world_to_meters);

    for (int row = 0; row < samples; ++row) {
        for (int col = 0; col < samples; ++col) {
            // ⚠ Row-major with row along +y and col along +x, matching MuJoCo's hfield indexing.
            // Transposing this yields terrain that is subtly ROTATED - far harder to notice than
            // terrain that is simply missing.
            const double wx = center_x - half_extent + 2.0 * half_extent * col / (samples - 1);
            const double wy = center_y - half_extent + 2.0 * half_extent * row / (samples - 1);
            // ⚠ URDF -> UNREAL IS A Y MIRROR. The solver frame is URDF/FLU; Unreal is left-handed
            // with Y to the right. Tracing at raw (wx, wy) samples a MIRRORED patch of the level -
            // it still hits ground everywhere, so every trace succeeds and the field looks healthy
            // while sitting nowhere near the robots. Use the one conversion the rest of the URDF
            // path uses rather than writing the mirror out again here.
            const FVector column = UrdfTransform::toFVector(urdf::Vec3{ wx, wy, 0.0 },
                                                            world_to_meters);
            const FVector start(column.X, column.Y, top);
            const FVector end(column.X, column.Y, bottom);

            FHitResult hit;
            if (unreal_world->LineTraceSingleByChannel(hit, start, end, ECC_Visibility, params)) {
                const double height = hit.ImpactPoint.Z / world_to_meters;
                z[static_cast<size_t>(row) * samples + col] = height;
                lowest = std::min(lowest, height);
                ++hits;
            }
        }
    }

    if (hits == 0) {
        UE_LOG(LogPhysicsCoordinator, Error,
               TEXT("ground height field: NOT ONE of %d traces hit anything over the region "
                    "centred on (%.1f %.1f). The robots would have no floor."),
               samples * samples, center_x, center_y);
        return false;
    }

    // ⚠ Holes filled with the LOWEST sample rather than left at zero. A gap - a pit, or a trace
    // that escaped through a doorway - would otherwise become a spike at whatever z = 0 means in
    // this map, and a robot would trip over nothing.
    out.min_z = lowest;
    for (size_t i = 0; i < z.size(); ++i)
        out.heights[i] = static_cast<float>((std::isnan(z[i]) ? lowest : z[i]) - lowest);

    UE_LOG(LogPhysicsCoordinator, Log,
           TEXT("ground height field: %d x %d over %.1f m centred on (%.1f %.1f), %d/%d traces hit, "
                "floor at %.3f m"),
           samples, samples, 2.0 * half_extent, center_x, center_y, hits, samples * samples,
           lowest);
    if (hits < samples * samples) {
        UE_LOG(LogPhysicsCoordinator, Warning,
               TEXT("%d of %d ground traces hit NOTHING and were filled with the lowest sample - "
                    "the region may extend past the level"),
               samples * samples - hits, samples * samples);
    }
    return true;
}

void requireSupportedCoordinatedPopulation(
    const msr::airlib::AirSimSettings& settings)
{
    using AirSimSettings = msr::airlib::AirSimSettings;

    std::string rejected;
    std::vector<std::string> box3d_vehicles;
    std::vector<std::string> mujoco_vehicles;
    const auto reject = [&rejected](const std::string& name, const std::string& reason) {
        rejected += (rejected.empty() ? "" : "\n  ") + name + ": " + reason;
    };

    for (const auto& item : settings.vehicles) {
        const AirSimSettings::VehicleSetting& vehicle = *item.second;
        if (vehicle.vehicle_type != AirSimSettings::kVehicleTypeUrdfBot) {
            reject(item.first,
                   "vehicle type \"" + vehicle.vehicle_type +
                       "\" is owned by Chaos or FastPhysics, which have no coordinated shared "
                       "scene yet");
            continue;
        }

        const std::string engine = msr::airlib::Utils::toLower(vehicle.physics_engine);
        if (engine.empty() || engine == "box3d") {
            box3d_vehicles.push_back(item.first);
            continue;
        }
        if (engine == "mujoco") {
            mujoco_vehicles.push_back(item.first);
            continue;
        }
        reject(item.first, "PhysicsEngine \"" + vehicle.physics_engine + "\" is not a URDF backend");
    }

    // ⚠ TWO SHARED SCENES ARE NOT ONE WORLD. Each backend gets its own scene and each solves its
    // own bodies correctly, but nothing carries contact between them: a Box3D rover and a MuJoCo
    // rover would drive through each other while every log line looked healthy. That bridge is
    // Phase 6 (reciprocal kinematic proxies) and does not exist, so a mixed population is refused
    // rather than silently delivered as two worlds sharing a clock.
    if (!box3d_vehicles.empty() && !mujoco_vehicles.empty()) {
        std::string box3d_list, mujoco_list;
        for (const std::string& name : box3d_vehicles)
            box3d_list += (box3d_list.empty() ? "" : ", ") + name;
        for (const std::string& name : mujoco_vehicles)
            mujoco_list += (mujoco_list.empty() ? "" : ", ") + name;
        throw std::runtime_error(
            "PhysicsCoordinator: this population declares two rigid authorities - Box3D (" +
            box3d_list + ") and MuJoCo (" + mujoco_list +
            "). Each gets its own shared scene, and cross-engine contact between them is not "
            "implemented, so those robots would pass through each other. Declare one backend for "
            "the whole population, or use PhysicsCoordinator.Mode=Legacy.");
    }

    if (!rejected.empty()) {
        throw std::runtime_error(
            "PhysicsCoordinator.Mode is coordinated, but these vehicles cannot join a shared "
            "physics scene:\n  " + rejected +
            "\nRefusing to start a partly coordinated world, because the remaining vehicles would "
            "silently keep their private solver worlds. Use PhysicsCoordinator.Mode=Legacy, or "
            "declare a population every shared scene can own.");
    }

    if (settings.vehicles.empty())
        throw std::runtime_error(
            "PhysicsCoordinator.Mode is coordinated but no vehicle is declared; there is no "
            "population to coordinate.");
}
} // namespace

// I-R Phase 0. Non-static: also read by UnrealLidarSensor.cpp.
TAutoConsoleVariable<int32> CVarLogPhysicsTiming(
	TEXT("airsim.LogPhysicsTiming"),
	0,
	TEXT("I-R Phase 0: report physics-loop and game-thread timing every N seconds (0 = off).\n")
	TEXT("Discriminates physics-thread occupancy from Chaos scene-lock contention - see\n")
	TEXT("sim_issues/Lidar_Async_Architecture.md. Diagnostic only; changes no recorded data."),
	ECVF_Default);

void ASimModeWorldBase::BeginPlay()
{
    Super::BeginPlay();
}

void ASimModeWorldBase::preparePhysicsScene()
{
    const auto coordinator_mode = getSettings().physics_coordinator.mode;
    if (coordinator_mode == PhysicsCoordinatorMode::Legacy)
        return;

    // Rejected before setupVehiclesAndCamera constructs any UrdfBotSimApi. Otherwise a failed
    // coordinated startup still performs expensive cooks and, more importantly, temporarily
    // creates the private per-robot worlds whose elimination is the feature's contract.
    requireSupportedCoordinatedPopulation(getSettings());

    physics_scene_coordinator_.reset(new PhysicsSceneCoordinator(
        toCoordinatorMode(coordinator_mode), makeWorldIdentity(GetWorld())));

    try {
        coordinated_physics_scene_.reset(new FCoordinatedPhysicsScene(
            *physics_scene_coordinator_, getSettings().physics_coordinator,
            coordinatedFixedStepSeconds()));
    }
    catch (...) {
        physics_scene_coordinator_.reset();
        throw;
    }

    // ⚠ To a real log category as well as the screen. LogMessageString draws on screen only and
    // keys by prefix, so a coordinated run's mode and dt would scroll away and be unrecoverable.
    // The world's one ground, sampled here - before any vehicle exists, so no robot's arrival can
    // influence what the floor is.
    {
        const auto& field_setting = getSettings().physics_coordinator.ground_height_field;
        if (field_setting.isEnabled()) {
            double center_x = field_setting.center_x;
            double center_y = field_setting.center_y;
            double half_extent = field_setting.half_extent;

            if (field_setting.mode ==
                msr::airlib::AirSimSettings::GroundHeightFieldMode::AutoFromSpawns) {
                // ⚠ DERIVED, AND SAID SO. The authored region is the contract; this fallback exists
                // so a scenario can be run without per-map authoring, and it must never be mistaken
                // for one - a region that changes when a robot is moved is not a fixed world.
                double lo_x = 0, lo_y = 0, hi_x = 0, hi_y = 0;
                bool any = false;
                // ⚠ THREE FRAMES MEET HERE. `position` is AirSim global NED (PlayerStart-relative,
                // Y east, Z down); the solver is URDF/FLU in Unreal world coordinates. Treating one
                // as the other silently mirrors Y and drops the PlayerStart offset, which puts the
                // sampled region somewhere the robots are not. Convert exactly the way the sim mode
                // converts a spawn: NED -> Unreal, then Unreal -> URDF.
                const float to_meters = getGlobalNedTransform().fromNed(1.0f);
                for (const auto& item : getSettings().vehicles) {
                    const auto& position = item.second->position;
                    if (std::isnan(position.x()) || std::isnan(position.y()))
                        continue;
                    const FVector unreal_spawn =
                        getGlobalNedTransform().fromGlobalNed(position);
                    const urdf::Vec3 solver_spawn =
                        UrdfTransform::toUrdfVec(unreal_spawn, to_meters);
                    if (!any) {
                        lo_x = hi_x = solver_spawn.x;
                        lo_y = hi_y = solver_spawn.y;
                        any = true;
                    }
                    lo_x = std::min(lo_x, solver_spawn.x);
                    hi_x = std::max(hi_x, solver_spawn.x);
                    lo_y = std::min(lo_y, solver_spawn.y);
                    hi_y = std::max(hi_y, solver_spawn.y);
                }
                if (!any) {
                    throw std::runtime_error(
                        "PhysicsCoordinator.GroundHeightField.Mode=AutoFromSpawns, but no vehicle "
                        "declares an X/Y spawn to derive the region from. Author the region, or "
                        "give the vehicles positions.");
                }
                center_x = 0.5 * (lo_x + hi_x);
                center_y = 0.5 * (lo_y + hi_y);
                half_extent = 0.5 * std::max(hi_x - lo_x, hi_y - lo_y) + field_setting.margin;

                const long long side = field_setting.samplesPerSide(half_extent);
                if (side * side > field_setting.max_cells) {
                    throw std::runtime_error(
                        "PhysicsCoordinator.GroundHeightField.Mode=AutoFromSpawns derived a "
                        "region needing " + std::to_string(side * side) +
                        " cells, above MaxCells. Reduce Margin, raise CellSize, or author the "
                        "region explicitly.");
                }
                UE_LOG(LogPhysicsCoordinator, Warning,
                       TEXT("ground height field region DERIVED from vehicle spawns: centre "
                            "(%.2f %.2f), half extent %.2f m (margin %.2f). This is not an "
                            "authored world: moving a robot changes the sampled region."),
                       center_x, center_y, half_extent, field_setting.margin);
            }

            urdf::BackendOptions::HeightField sampled;
            if (sampleWorldHeightField(GetWorld(), getGlobalNedTransform().fromNed(1.0f),
                                       field_setting, center_x, center_y, half_extent, sampled)) {
                coordinated_physics_scene_->setGroundHeightField(sampled);
            }
            else {
                throw std::runtime_error(
                    "PhysicsCoordinator.GroundHeightField is enabled but sampling produced no "
                    "ground at all. Check the region against the level rather than starting a run "
                    "whose robots have nothing to stand on.");
            }
        }
    }

    UE_LOG(LogPhysicsCoordinator, Log,
           TEXT("mode=%s, authoritative fixed dt = %.6f s (%.3f ms), presettle=%s %.3f s"),
           UTF8_TO_TCHAR(coordinatorModeName(coordinator_mode)), coordinatedFixedStepSeconds(),
           coordinatedFixedStepSeconds() * 1000.0,
           getSettings().physics_coordinator.presettle ? TEXT("on") : TEXT("off"),
           getSettings().physics_coordinator.presettle_seconds);
    UAirBlueprintLib::LogMessageString(
        "PhysicsCoordinator: ",
        std::string(coordinatorModeName(coordinator_mode)) + ", fixed dt = " +
            std::to_string(coordinatedFixedStepSeconds() * 1000.0) + " ms",
        LogDebugLevel::Informational);
}

ICoordinatedPhysicsScene* ASimModeWorldBase::getCoordinatedPhysicsScene() const
{
    return coordinated_physics_scene_.get();
}

void ASimModeWorldBase::initializeForPlay()
{
    std::vector<msr::airlib::UpdatableObject*> vehicles;
    for (auto& api : getApiProvider()->getVehicleSimApis())
        vehicles.push_back(api);
    //TODO: directly accept getVehicleSimApis() using generic container

    const auto coordinator_mode = getSettings().physics_coordinator.mode;
    if (coordinator_mode == PhysicsCoordinatorMode::Legacy) {
        // Keep the established initialization path and its default start=true behavior intact.
        std::unique_ptr<PhysicsEngineBase> physics_engine = createPhysicsEngine();
        physics_engine_ = physics_engine.get();
        physics_world_.reset(new msr::airlib::PhysicsWorld(std::move(physics_engine),
                                                           vehicles,
                                                           getPhysicsLoopPeriod()));

        // ⚠ Deformable terrain is NOT coordinated-only. Every private world still has real
        // colliders with real poses, and the sidecar consumes poses, not a shared solver. Refusing
        // it here would make sand a reason to abandon Legacy, which is not a trade anyone should
        // be forced into. The world stamp is simply all-zero, and the sidecar compares whatever it
        // is given — a Legacy run is one epoch that never changes.
        openMpmSidecarLink();
        publishMpmRegistry();
        return;
    }

    try {
        if (!physics_scene_coordinator_)
            throw std::logic_error(
                "Coordinated physics was not prepared before vehicle construction");

        if (!coordinated_physics_scene_)
            throw std::logic_error(
                "Coordinated physics scenes were not prepared before vehicle construction");

        // Every vehicle has registered by now, so this is the moment topology is frozen: the
        // manifest is committed and each shared scene seals itself through its participant hook.
        coordinated_physics_scene_->commitManifest();
        UAirBlueprintLib::LogMessageString("PhysicsCoordinator population\n",
                                           coordinated_physics_scene_->describePopulation(),
                                           LogDebugLevel::Informational);
        UE_LOG(LogPhysicsCoordinator, Log, TEXT("declared population:\n%s"),
               UTF8_TO_TCHAR(coordinated_physics_scene_->describePopulation().c_str()));

        // ⚠ Opened here — after the commit, because collider ids are only final once every
        // vehicle has joined — but the REGISTRY is published further down, after the world exists.
        openMpmSidecarLink();

        // Assemble the world with its sole executor stopped. Manifest publication must complete
        // before any backend is allowed to advance, and the coordinator must be attached at
        // construction: the world's first reset is the global transaction that establishes t=0,
        // pre-settle included, and every vehicle's first published state comes out of it.
        const msr::airlib::TTimeDelta fixed_step_seconds = coordinatedFixedStepSeconds();
        std::unique_ptr<PhysicsEngineBase> physics_engine = createPhysicsEngine();
        physics_engine_ = physics_engine.get();
        physics_world_.reset(new msr::airlib::PhysicsWorld(
            std::move(physics_engine), vehicles, getPhysicsLoopPeriod(), false, false,
            fixed_step_seconds, physics_scene_coordinator_.get(),
            makeResetPolicy(getSettings().physics_coordinator, fixed_step_seconds)));

        // ⚠ THE REGISTRY IS PUBLISHED AFTER THE WORLD IS BUILT, and the ordering is not cosmetic.
        // Constructing PhysicsWorld performs the world's FIRST RESET — the global transaction that
        // establishes t=0 — which increments reset_epoch. Publishing before that stamped the
        // registry with epoch 0 while every subsequent state block carried epoch 1, so the sidecar
        // correctly refused the whole run with "the sand belongs to the previous run" the instant
        // it received its first state. The sidecar was right; the ordering was wrong.
        //
        // Publishing here also means collider poses are the SETTLED ones, not the spawn poses, so
        // the sand is built around where the robot actually is.
        publishMpmRegistry();

        physics_world_->startAsyncUpdator();
    }
    catch (...) {
        // stop() is a join fence. This also covers future failures after a worker was started.
        if (physics_world_)
            physics_world_->stopAsyncUpdator();
        physics_world_.reset();
        physics_engine_ = nullptr;
        coordinated_physics_scene_.reset();
        physics_scene_coordinator_.reset();
        throw;
    }
}

void ASimModeWorldBase::registerPhysicsBody(msr::airlib::VehicleSimApiBase* physicsBody)
{
    if (getSettings().physics_coordinator.isCoordinated()) {
        throw std::logic_error(
            "Runtime physics-body registration is disabled when PhysicsCoordinator.Mode is "
            "coordinated and PhysicsCoordinator.TopologyPolicy=Fixed; the body was not reset or registered.");
    }
    if (physicsBody == nullptr)
        throw std::invalid_argument("Cannot register a null physics body");
    if (!physics_world_)
        throw std::logic_error("Cannot register a physics body before the physics world is initialized");

    // Reset the vehicle as well before registering it
    // Similar to what happens in initializeForPlay() above
    physicsBody->reset();
    physics_world_.get()->addBody(physicsBody);
}

void ASimModeWorldBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // Stop and join the sole executor before destroying either its world or coordinator callbacks.
    if (physics_world_)
        physics_world_->stopAsyncUpdator();
    physics_world_.reset();
    physics_engine_ = nullptr;
    // Scenes before the coordinator: the scene object holds the coordinator by reference.
    coordinated_physics_scene_.reset();
    physics_scene_coordinator_.reset();

    Super::EndPlay(EndPlayReason);
}

void ASimModeWorldBase::startAsyncUpdator()
{
    if (physics_world_)
        physics_world_->startAsyncUpdator();
}

void ASimModeWorldBase::stopAsyncUpdator()
{
    if (physics_world_)
        physics_world_->stopAsyncUpdator();
}

long long ASimModeWorldBase::getPhysicsLoopPeriod() const //nanoseconds
{
    return physics_loop_period_;
}
void ASimModeWorldBase::setPhysicsLoopPeriod(long long period)
{
    physics_loop_period_ = period;
}

std::unique_ptr<ASimModeWorldBase::PhysicsEngineBase> ASimModeWorldBase::createPhysicsEngine()
{
    std::unique_ptr<PhysicsEngineBase> physics_engine;
    std::string physics_engine_name = getSettings().physics_engine_name;
    if (physics_engine_name == "")
        physics_engine.reset(); //no physics engine
    else if (physics_engine_name == "FastPhysicsEngine") {
        msr::airlib::Settings fast_phys_settings;
        if (msr::airlib::Settings::singleton().getChild("FastPhysicsEngine", fast_phys_settings)) {
            physics_engine.reset(new msr::airlib::FastPhysicsEngine(fast_phys_settings.getBool("EnableGroundLock", true)));
        }
        else {
            physics_engine.reset(new msr::airlib::FastPhysicsEngine());
        }

        physics_engine->setWind(getSettings().wind);
        physics_engine->setExtForce(getSettings().ext_force);
    }
    else if (physics_engine_name == "ExternalPhysicsEngine") {
        physics_engine.reset(new msr::airlib::ExternalPhysicsEngine());
    }
    else {
        physics_engine.reset();
        UAirBlueprintLib::LogMessageString("Unrecognized physics engine name: ", physics_engine_name, LogDebugLevel::Failure);
    }

    return physics_engine;
}

bool ASimModeWorldBase::isPaused() const
{
    return physics_world_ ? physics_world_->isPaused() : ASimModeBase::isPaused();
}

void ASimModeWorldBase::pause(bool is_paused)
{
    if (physics_world_)
        physics_world_->pause(is_paused);
    ASimModeBase::pause(is_paused);
}

void ASimModeWorldBase::continueForTime(double seconds)
{
    if (!physics_world_)
        return;

    int64 start_frame_number = UKismetSystemLibrary::GetFrameCount();
    if (physics_world_->isPaused()) {
        physics_world_->pause(false);
        UGameplayStatics::SetGamePaused(this->GetWorld(), false);
    }

    physics_world_->continueForTime(seconds);
    while (!physics_world_->isPaused()) {
        continue;
    }
    // wait if no new frame is renderd
    while (start_frame_number == UKismetSystemLibrary::GetFrameCount()) {
        continue;
    }
    UGameplayStatics::SetGamePaused(this->GetWorld(), true);
}

void ASimModeWorldBase::continueForFrames(uint32_t frames)
{
    if (!physics_world_)
        return;

    if (physics_world_->isPaused()) {
        physics_world_->pause(false);
        UGameplayStatics::SetGamePaused(this->GetWorld(), false);
    }

    physics_world_->setFrameNumber((uint32_t)GFrameNumber);
    physics_world_->continueForFrames(frames);
    while (!physics_world_->isPaused()) {
        physics_world_->setFrameNumber((uint32_t)GFrameNumber);
    }
    UGameplayStatics::SetGamePaused(this->GetWorld(), true);
}

void ASimModeWorldBase::setWind(const msr::airlib::Vector3r& wind) const
{
    if (physics_engine_)
        physics_engine_->setWind(wind);
}

void ASimModeWorldBase::setExtForce(const msr::airlib::Vector3r& ext_force) const
{
    if (physics_engine_)
        physics_engine_->setExtForce(ext_force);
}

void ASimModeWorldBase::updateDebugReport(msr::airlib::StateReporterWrapper& debug_reporter)
{
    unused(debug_reporter);
    //we use custom debug reporting for this class
}

void ASimModeWorldBase::Tick(float DeltaSeconds)
{
    if (!physics_world_) {
        Super::Tick(DeltaSeconds);
        return;
    }

    // I-R Phase 0: the game-thread half of the discriminator. If the harm is Chaos scene-lock
    // CONTENTION, this is where it shows - the wait to acquire physics_world_->lock() balloons
    // because the physics thread is holding it while issuing scene queries. If the harm is mere
    // OCCUPANCY, the wait stays small and only the physics loop's own period grows.
    static AirSimPhysicsTiming::Window tick_window;
    static AirSimPhysicsTiming::Window lock_window;
    const int32 timing_period = AirSimPhysicsTiming::ReportPeriodSeconds();
    const bool timing_on = timing_period > 0;

    const auto tick_t0 = AirSimPhysicsTiming::Clock::now();
    if (timing_on) tick_window.noteEntry(tick_t0);

    { //keep this lock as short as possible
        const auto lock_t0 = AirSimPhysicsTiming::Clock::now();
        physics_world_->lock();
        if (timing_on) {
            lock_window.noteEntry(lock_t0);
            lock_window.noteBusy(AirSimPhysicsTiming::ToMs(AirSimPhysicsTiming::Clock::now() - lock_t0));
        }

        physics_world_->enableStateReport(EnableReport);
        physics_world_->updateStateReport();

        for (auto& api : getApiProvider()->getVehicleSimApis())
            api->updateRenderedState(DeltaSeconds);

        physics_world_->unlock();
    }

    if (timing_on) {
        tick_window.noteBusy(AirSimPhysicsTiming::ToMs(AirSimPhysicsTiming::Clock::now() - tick_t0));
        const auto now = AirSimPhysicsTiming::Clock::now();
        if (tick_window.shouldReport(now, timing_period)) {
            UE_LOG(LogTemp, Log,
                   TEXT("[AirSim][timing] GAME thread: tick %.2f ms avg (max %.2f) every %.2f ms avg (max %.2f) | waiting on physics lock %.3f ms avg (max %.3f) = %.1f%% of tick | n=%llu"),
                   tick_window.busy_ms / tick_window.calls, tick_window.busy_max,
                   tick_window.gap_ms / FMath::Max<uint64>(tick_window.calls - 1, 1), tick_window.gap_max,
                   lock_window.busy_ms / FMath::Max<uint64>(lock_window.calls, 1), lock_window.busy_max,
                   100.0 * lock_window.busy_ms / FMath::Max(tick_window.busy_ms, KINDA_SMALL_NUMBER),
                   tick_window.calls);
            tick_window.reset();
            lock_window.reset();
        }
    }

    //perform any expensive rendering update outside of lock region
    for (auto& api : getApiProvider()->getVehicleSimApis())
        api->updateRendering(DeltaSeconds);

    drawCollisionDebugOverlay();

    // ⚠ GAME THREAD, not the physics thread, and deliberately. MPM advances at ~60 Hz while the
    // rigid tick is 333 Hz, so publishing at frame rate is already several times more often than
    // the sidecar can consume — and it keeps this work off the thread whose period is the thing
    // being protected.
    publishMpmState();
    updateMpmParticleRender();

    Super::Tick(DeltaSeconds);
}

// --- Newton MPM deformable terrain link ---------------------------------------------------------
//
// ⚠ ONE-WAY, BY DECISION AND NOT BY OMISSION (plan §M2 / §11.1). Colliders push the sand; the sand
// does not push back. The missing piece is each link's articulated operational-space effective
// inertia, which no backend here can supply — Newton asks for it directly as `body_inv_inertia`,
// and a two-way reaction computed from a link's isolated inertia would give sinkage and traction
// numbers that are wrong in a way no log line shows.
//
// ⚠ THE SIM NEVER BLOCKS ON THE SIDECAR. Newton runs out of process at a cadence that does not
// match ours — the coordinated tick is 3 ms, one measured MPM frame is 16.67 ms. Waiting for it
// would run the rigid simulation at the sand's speed. Not blocking and not noticing are different
// things, though, which is what the acknowledgement is for.

namespace
{
/// ⚠ INSTANCED MESH, NOT NIAGARA — for now, and only because a Niagara system is a `.uasset` that
/// has to be authored in the editor. This path needs no content at all: the engine's own sphere,
/// GPU-instanced. It is uglier than sprites and it is available today, which is the trade that
/// matters while the question is still "is the sand where I think it is".
TAutoConsoleVariable<int32> CVarMpmRender(
    TEXT("airsim.MpmRenderParticles"), 1,
    TEXT("Draw the MPM sidecar's sand particles in the level.\n0: off\n1: on"),
    ECVF_Default);

TAutoConsoleVariable<float> CVarMpmRenderScale(
    TEXT("airsim.MpmRenderScale"), 1.0f,
    TEXT("Multiplier on the drawn particle size. The sidecar reports the real radius; this only "
         "makes the sand easier to see, and does not change anything the solver does."),
    ECVF_Default);

TAutoConsoleVariable<int32> CVarMpmRenderMax(
    TEXT("airsim.MpmRenderMaxInstances"), 40000,
    TEXT("Hard cap on instances drawn. Rebuilding instance transforms is game-thread work, so "
         "this is what keeps a large patch from costing more than the simulation does."),
    ECVF_Default);

/// Linear RGB of the drawn sand. A cvar rather than a constant because the right sand colour
/// depends on the level it sits in, and a rebuild to try a shade costs minutes.
TAutoConsoleVariable<FString> CVarMpmRenderColor(
    TEXT("airsim.MpmRenderColor"), TEXT("0.72,0.56,0.33"),
    TEXT("Linear RGB of the MPM sand, comma separated (e.g. 0.72,0.56,0.33). Appearance only."),
    ECVF_Default);

TAutoConsoleVariable<float> CVarMpmRenderRoughness(
    TEXT("airsim.MpmRenderRoughness"), 0.95f,
    TEXT("Roughness of the sand material. Dry sand is near 1; lower it for a wet, packed look."),
    ECVF_Default);
} // namespace

void ASimModeWorldBase::updateMpmParticleRender()
{
    if (CVarMpmRender.GetValueOnGameThread() == 0) {
        if (mpm_particle_mesh_ != nullptr && mpm_particle_mesh_->GetInstanceCount() > 0)
            mpm_particle_mesh_->ClearInstances();
        return;
    }

    // ⚠ Attach lazily and KEEP RETRYING. The sidecar creates this segment, and it starts minutes
    // after the simulator does (Warp spends that long compiling kernels). Opening once at
    // BeginPlay would mean the renderer never connects on any normal run.
    if (!mpm_particle_reader_.isOpen()) {
        if (!mpm_particle_reader_.open("/dev/shm"))
            return;
        UE_LOG(LogPhysicsCoordinator, Log,
               TEXT("MPM particle stream attached — the sidecar's sand is now drawn in the level"));
    }

    msr::airlib::mpm::MpmParticleReader::Frame frame;
    if (!mpm_particle_reader_.read(frame, mpm_last_render_step_)) {
        // ⚠ A DEAD SIDECAR MUST NOT LEAVE ITS SAND ON SCREEN. Skipping the rebuild when nothing
        // advanced is the right optimisation for an idle frame and the WRONG behaviour when the
        // sidecar has exited: the instances simply stay there, and the operator sees a sand bed
        // that no longer corresponds to anything. Measured: a segment 4.5 minutes stale, still
        // drawn, with nothing anywhere saying so.
        //
        // This is the same failure the collider link's acknowledgement exists to prevent, arriving
        // from the render side, and it deserves the same answer: say it, then stop showing it.
        const double now_idle = FPlatformTime::Seconds();
        if (mpm_last_frame_seconds_ > 0.0 && mpm_particle_mesh_ != nullptr &&
            mpm_particle_mesh_->GetInstanceCount() > 0 &&
            now_idle - mpm_last_frame_seconds_ > 5.0) {
            UE_LOG(LogPhysicsCoordinator, Warning,
                   TEXT("no MPM particle frame for %.1f s — the sidecar has stopped. Clearing the "
                        "sand rather than leaving a stale bed on screen."),
                   now_idle - mpm_last_frame_seconds_);
            mpm_particle_mesh_->ClearInstances();
            // ⚠ REMEMBER WHAT WE JUST THREW AWAY. Re-attaching is right — a new sidecar may create
            // a different file at the same name — but the OLD segment is still on disk with its
            // last frame in it, so a bare re-attach reads step 7792 again, redraws the sand we just
            // cleared, and clears it again five seconds later. Measured: that exact 5 s flap.
            // Latching the step makes the decision stick until something genuinely new arrives.
            mpm_stale_render_step_ = mpm_last_render_step_;
            mpm_particle_reader_.close();
            mpm_last_render_step_ = 0;
            mpm_last_frame_seconds_ = 0.0;
            mpm_render_announced_ = false;
        }
        return;
    }
    // The frame read back after a stale clear is the dead sidecar's last one. Ignore it; anything
    // with a different step is a live sidecar and clears the latch.
    if (mpm_stale_render_step_ != 0 && frame.sidecar_step == mpm_stale_render_step_)
        return;
    mpm_stale_render_step_ = 0;
    mpm_last_render_step_ = frame.sidecar_step;
    mpm_last_frame_seconds_ = FPlatformTime::Seconds();

    if (mpm_particle_mesh_ == nullptr) {
        UStaticMesh* sphere =
            LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
        if (sphere == nullptr) {
            UE_LOG(LogPhysicsCoordinator, Error,
                   TEXT("cannot load /Engine/BasicShapes/Sphere — MPM particles will not be drawn"));
            return;
        }
        mpm_particle_mesh_ = NewObject<UInstancedStaticMeshComponent>(this);
        mpm_particle_mesh_->SetStaticMesh(sphere);
        // ⚠ NO COLLISION, and this is not an optimisation. These spheres are a picture of another
        // solver's particles; giving them collision would let the rover drive on its own
        // visualisation, which is a feedback loop rather than a simulation.
        mpm_particle_mesh_->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        mpm_particle_mesh_->SetCastShadow(false);
        mpm_particle_mesh_->SetMobility(EComponentMobility::Movable);
        mpm_particle_mesh_->RegisterComponent();
        mpm_particle_mesh_->AttachToComponent(GetRootComponent(),
                                              FAttachmentTransformRules::KeepWorldTransform);
    }

    // SAND SHOULD LOOK LIKE SAND. The engine sphere ships with a flat grey material, which reads as
    // debug geometry rather than terrain — and a picture that looks like debug output is one an
    // operator stops trusting as terrain. BasicShapeMaterial exposes a Color vector parameter and a
    // Roughness scalar, so a dynamic instance is enough; no authored content is required, which is
    // the same reason this path uses instanced spheres rather than Niagara.
    if (mpm_particle_material_ == nullptr) {
        UMaterialInterface* base = LoadObject<UMaterialInterface>(
            nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
        if (base != nullptr) {
            mpm_particle_material_ = UMaterialInstanceDynamic::Create(base, this);
            mpm_particle_mesh_->SetMaterial(0, mpm_particle_material_);
            mpm_particle_appearance_.Empty(); // force the apply below on the first frame
        }
        else {
            UE_LOG(LogPhysicsCoordinator, Warning,
                   TEXT("cannot load BasicShapeMaterial — the sand will draw in the engine's "
                        "default grey. Appearance only; nothing else is affected."));
        }
    }
    if (mpm_particle_material_ != nullptr) {
        const FString wanted = CVarMpmRenderColor.GetValueOnGameThread() +
                               FString::Printf(TEXT("/%.3f"),
                                               CVarMpmRenderRoughness.GetValueOnGameThread());
        if (wanted != mpm_particle_appearance_) {
            mpm_particle_appearance_ = wanted;
            TArray<FString> parts;
            CVarMpmRenderColor.GetValueOnGameThread().ParseIntoArray(parts, TEXT(","), true);
            if (parts.Num() >= 3) {
                mpm_particle_material_->SetVectorParameterValue(
                    TEXT("Color"), FLinearColor(FCString::Atof(*parts[0]),
                                                FCString::Atof(*parts[1]),
                                                FCString::Atof(*parts[2]), 1.0f));
            }
            else {
                UE_LOG(LogPhysicsCoordinator, Warning,
                       TEXT("airsim.MpmRenderColor '%s' is not 'r,g,b' — leaving the sand colour "
                            "as it was"),
                       *CVarMpmRenderColor.GetValueOnGameThread());
            }
            mpm_particle_material_->SetScalarParameterValue(
                TEXT("Roughness"), CVarMpmRenderRoughness.GetValueOnGameThread());
        }
    }

    const float world_to_meters = getGlobalNedTransform().fromNed(1.0f);
    // The engine sphere is 100 uu across, so a unit scale is a 1 m ball; scale to the real radius.
    const float scale = static_cast<float>(frame.radius) * 2.0f *
                        CVarMpmRenderScale.GetValueOnGameThread();

    const int32 cap = FMath::Max(1, CVarMpmRenderMax.GetValueOnGameThread());
    const int32 draw = FMath::Min(static_cast<int32>(frame.count), cap);

    TArray<FTransform> transforms;
    transforms.Reserve(draw);
    for (int32 i = 0; i < draw; ++i) {
        const urdf::Vec3 solver{ frame.positions[i * 3], frame.positions[i * 3 + 1],
                                 frame.positions[i * 3 + 2] };
        transforms.Emplace(FQuat::Identity,
                           UrdfTransform::toFVector(solver, world_to_meters),
                           FVector(scale));
    }

    // ⚠ Rebuild wholesale rather than updating instance-by-instance. The particle set is a fresh
    // decimated SAMPLE each frame — instance i is not the same particle it was last frame — so
    // per-instance updates would be both slower and meaningless.
    mpm_particle_mesh_->ClearInstances();
    if (draw > 0)
        mpm_particle_mesh_->AddInstances(transforms, /*bShouldReturnIndices=*/false,
                                         /*bWorldSpace=*/true);

    if (!mpm_render_announced_) {
        mpm_render_announced_ = true;
        UE_LOG(LogPhysicsCoordinator, Log,
               TEXT("drawing %d MPM particles (sidecar published %d of %llu total, radius %.4f m)"),
               draw, static_cast<int32>(frame.count),
               static_cast<unsigned long long>(frame.total_particles), frame.radius);
    }

    const double now = FPlatformTime::Seconds();
    if (now - last_mpm_render_log_ > 15.0) {
        last_mpm_render_log_ = now;
        // ⚠ Says what fraction is on screen. A decimated render that looks complete is the same
        // class of lie as a counter reporting success about the wrong place.
        UE_LOG(LogPhysicsCoordinator, Log,
               TEXT("MPM render: %d instances = %.1f%% of the solver's %llu particles, sidecar "
                    "step %llu at t=%.2f s"),
               draw, frame.total_particles > 0 ? 100.0 * draw / frame.total_particles : 0.0,
               static_cast<unsigned long long>(frame.total_particles),
               static_cast<unsigned long long>(frame.sidecar_step), frame.sidecar_time);
    }
}

void ASimModeWorldBase::openMpmSidecarLink()
{
    using AirSimSettings = msr::airlib::AirSimSettings;
    const auto& settings = getSettings();

    const AirSimSettings::DeformableTerrainSetting* terrain = nullptr;
    for (const auto& item : settings.deformable_terrains) {
        if (item.second.enabled) {
            terrain = &item.second;
            break;
        }
    }
    if (terrain == nullptr)
        return;   // no deformable terrain declared; the ordinary case

    // ⚠ SELECTION IS THE M2 EXIT CRITERION: "selected colliders affect MPM; unselected links do
    // not". Built from the per-vehicle UrdfLinkPhysics map, and qualified exactly as
    // UrdfBotSimApi::describeColliders qualifies them, or nothing would ever match.
    mpm_selected_ids_.clear();
    for (const auto& vehicle : settings.vehicles) {
        for (const auto& link : vehicle.second->urdf_link_physics) {
            if (link.second.interact_with_mpm)
                mpm_selected_ids_.push_back(vehicle.first + "/" + link.first);
        }
    }

    if (mpm_selected_ids_.empty()) {
        // Loud, because a terrain that exists and touches nothing looks exactly like a terrain
        // that is working until somebody watches the sand not move.
        UE_LOG(LogPhysicsCoordinator, Warning,
               TEXT("DeformableTerrain '%s' is enabled but NO link opted in through "
                    "UrdfLinkPhysics.InteractWithMPM. The sidecar would receive an empty collider "
                    "set, so the link is not opened."),
               UTF8_TO_TCHAR(terrain->terrain_id.c_str()));
        return;
    }

    mpm_publisher_.reset(new msr::airlib::mpm::MpmSidecarPublisher());
    msr::airlib::mpm::MpmSidecarPublisher::Options options;
    if (!mpm_publisher_->open(options)) {
        // Not fatal: a run without sand is better than a run that refuses to start.
        UE_LOG(LogPhysicsCoordinator, Error,
               TEXT("could not open the MPM sidecar segments in %s — continuing WITHOUT deformable "
                    "terrain."),
               UTF8_TO_TCHAR(options.directory.c_str()));
        mpm_publisher_.reset();
        return;
    }

    UE_LOG(LogPhysicsCoordinator, Log,
           TEXT("MPM link open in %s: terrain '%s', %d selected collider(s), coupling %s"),
           UTF8_TO_TCHAR(options.directory.c_str()),
           UTF8_TO_TCHAR(terrain->terrain_id.c_str()),
           static_cast<int32>(mpm_selected_ids_.size()),
           TEXT("KinematicOneWay"));
    for (const std::string& id : mpm_selected_ids_)
        UE_LOG(LogPhysicsCoordinator, Log, TEXT("  MPM collider: %s"), UTF8_TO_TCHAR(id.c_str()));
}

std::vector<urdf::PhysicsColliderSet> ASimModeWorldBase::gatherColliders() const
{
    // ⚠ Stable order across calls, because the state array is indexed by registry POSITION. The
    // api provider's ordering is the scenario's, which does not change within a run.
    std::vector<urdf::PhysicsColliderSet> sets;
    for (auto& api : getApiProvider()->getVehicleSimApis()) {
        urdf::PhysicsColliderSet set;
        if (api->describeColliders(set))
            sets.push_back(std::move(set));
    }
    return sets;
}

msr::airlib::mpm::WireWorldStamp ASimModeWorldBase::currentMpmStamp() const
{
    msr::airlib::mpm::WireWorldStamp stamp;
    if (physics_scene_coordinator_ != nullptr) {
        const auto& coordinator_stamp = physics_scene_coordinator_->stamp();
        stamp.world_id = coordinator_stamp.world.id;
        stamp.world_revision = coordinator_stamp.world.revision;
        stamp.manifest_revision = coordinator_stamp.manifest_revision;
        stamp.reset_epoch = coordinator_stamp.reset_epoch;
    }
    return stamp;
}

void ASimModeWorldBase::publishMpmRegistry()
{
    if (!mpm_publisher_ || !mpm_publisher_->isOpen())
        return;

    const std::vector<urdf::PhysicsColliderSet> robots = gatherColliders();

    // ⚠ NED IN SETTINGS, SOLVER FRAME ON THE WIRE, converted exactly once — here.
    //
    // The operator declares Region.Center in global NED because every other position in a settings
    // file is global NED, and asking them to switch frames for one key is how a rover ends up
    // driving past sand that is 4 m from where the file says it is. The sidecar is frame-agnostic
    // and must stay that way, so the conversion cannot live there. Same UrdfTransform path the
    // height-field sampler uses, for the same reason: a Y mirror applied on the wrong side of this
    // boundary already cost this workstream one full debugging cycle.
    msr::airlib::mpm::WireTerrainRegion region;
    const AirSimSettings::DeformableTerrainSetting* terrain = nullptr;
    for (const auto& item : getSettings().deformable_terrains)
        if (item.second.enabled) { terrain = &item.second; break; }

    if (terrain != nullptr) {
        const float world_to_meters = getGlobalNedTransform().fromNed(1.0f);
        const msr::airlib::Vector3r ned_center(
            static_cast<float>(terrain->region.center.x),
            static_cast<float>(terrain->region.center.y),
            static_cast<float>(terrain->region.center.z));
        const FVector unreal_center = getGlobalNedTransform().fromGlobalNed(ned_center);
        const urdf::Vec3 solver_center = UrdfTransform::toUrdfVec(unreal_center, world_to_meters);
        region.center = msr::airlib::mpm::WireVec3{ solver_center.x, solver_center.y,
                                                    solver_center.z };

        // ⚠ Size is an EXTENT, not a point: it takes no origin shift, and the NED->FLU sign flips
        // on y and z are meaningless for a symmetric box. Running it through the point conversion
        // would produce negative half-extents and a patch with no volume.
        region.half_extent = msr::airlib::mpm::WireVec3{
            std::fabs(terrain->region.size.x) * 0.5,
            std::fabs(terrain->region.size.y) * 0.5,
            std::fabs(terrain->region.size.z) * 0.5
        };
        region.rigid_ticks_per_mpm_step =
            static_cast<uint32_t>(std::max(0, terrain->coupling.rigid_ticks_per_mpm_step));
        region.valid = 1;
        std::snprintf(region.terrain_id, sizeof(region.terrain_id), "%s",
                      terrain->terrain_id.c_str());

        const double sand_floor = region.center.z - region.half_extent.z;
        const double sand_top = region.center.z + region.half_extent.z;
        UE_LOG(LogPhysicsCoordinator, Log,
               TEXT("MPM terrain '%s': NED centre (%.2f %.2f %.2f) size (%.2f %.2f %.2f) -> solver "
                    "centre (%.2f %.2f %.2f), half-extent (%.2f %.2f %.2f); sand spans z=%.2f..%.2f"),
               UTF8_TO_TCHAR(terrain->terrain_id.c_str()),
               terrain->region.center.x, terrain->region.center.y, terrain->region.center.z,
               terrain->region.size.x, terrain->region.size.y, terrain->region.size.z,
               region.center.x, region.center.y, region.center.z,
               region.half_extent.x, region.half_extent.y, region.half_extent.z,
               sand_floor, sand_top);

        // ⚠ CHECK THE SAND AGAINST THE GROUND THE WHEELS ARE ACTUALLY ON. A patch whose top
        // surface sits below the level's ground is buried — the wheels roll over solid ground and
        // never touch it. One that sits above is a floating slab the rover drives under. Neither
        // shows up anywhere: the link reports healthy, the sidecar solves happily, and the sand
        // simply never moves. Same failure family as the missing ground plane and the height field
        // sampled 120 m away, so it gets a real trace rather than trust in the authored Z.
        if (GetWorld() != nullptr) {
            const float world_to_meters = getGlobalNedTransform().fromNed(1.0f);
            const FVector column = UrdfTransform::toFVector(
                urdf::Vec3{ region.center.x, region.center.y, 0.0 }, world_to_meters);
            FHitResult hit;
            const FVector start(column.X, column.Y, static_cast<float>(5000.0 * world_to_meters));
            const FVector end(column.X, column.Y, static_cast<float>(-5000.0 * world_to_meters));
            FCollisionQueryParams params(FName(TEXT("MpmRegionGroundProbe")), true);
            if (GetWorld()->LineTraceSingleByChannel(hit, start, end, ECC_Visibility, params)) {
                const double ground_z = hit.ImpactPoint.Z / world_to_meters;
                const double gap = sand_top - ground_z;
                UE_LOG(LogPhysicsCoordinator, Log,
                       TEXT("  level ground under the patch centre is at z=%.2f m; sand top is "
                            "%.2f m %s it"),
                       ground_z, std::fabs(gap), gap >= 0 ? TEXT("ABOVE") : TEXT("BELOW"));
                // ⚠ Braces are load-bearing: UE_LOG expands to a multi-statement block, so a
                // braceless if/else around it does not compile.
                if (gap < -0.02) {
                    UE_LOG(LogPhysicsCoordinator, Warning,
                           TEXT("the sand patch is BURIED: its top (z=%.2f) is below the level's "
                                "ground (z=%.2f). Wheels will roll on solid ground and never touch "
                                "it. Raise DeformableTerrains Region.Center (NED z is DOWN, so use "
                                "a MORE NEGATIVE value to lift it)."),
                           sand_top, ground_z);
                }
                else if (sand_floor - ground_z > 0.05) {
                    UE_LOG(LogPhysicsCoordinator, Warning,
                           TEXT("the sand patch FLOATS: its floor (z=%.2f) is %.2f m above the "
                                "level's ground (z=%.2f). The rover will drive underneath it."),
                           sand_floor, sand_floor - ground_z, ground_z);
                }
            }
            else {
                UE_LOG(LogPhysicsCoordinator, Warning,
                       TEXT("no level geometry under the MPM patch centre — cannot check whether "
                            "the sand is reachable."));
            }
        }
    }

    const bool ok = mpm_publisher_->publishRegistry(currentMpmStamp(), coordinatedFixedStepSeconds(),
                                                    region, robots, mpm_selected_ids_);
    mpm_registry_published_ = true;

    const size_t published = mpm_publisher_->publishedIds().size();
    UE_LOG(LogPhysicsCoordinator, Log,
           TEXT("MPM registry published: %d of %d selected collider(s) matched a real link"),
           static_cast<int32>(published), static_cast<int32>(mpm_selected_ids_.size()));

    // ⚠ A SELECTED ID THAT MATCHED NOTHING IS A TYPO, and silently pushing sand with the other
    // wheels is exactly the "looks like it works" failure this whole workstream exists to avoid.
    if (published != mpm_selected_ids_.size()) {
        for (const std::string& id : mpm_selected_ids_) {
            const auto& got = mpm_publisher_->publishedIds();
            if (std::find(got.begin(), got.end(), id) == got.end())
                UE_LOG(LogPhysicsCoordinator, Error,
                       TEXT("UrdfLinkPhysics selects '%s' for MPM, but no such link exists in any "
                            "vehicle. Check the vehicle and link names."),
                       UTF8_TO_TCHAR(id.c_str()));
        }
    }
    if (!ok)
        UE_LOG(LogPhysicsCoordinator, Warning,
               TEXT("the MPM registry hit a wire limit (collider, shape or hull-vertex ceiling). "
                    "Some geometry did NOT reach the sidecar — see MpmSidecarProtocol.hpp."));
    if (published == 0)
        UE_LOG(LogPhysicsCoordinator, Error,
               TEXT("NOT ONE selected link reached the sidecar. The sand will not move."));
}

void ASimModeWorldBase::publishMpmState()
{
    if (!mpm_publisher_ || !mpm_publisher_->isOpen() || !mpm_registry_published_)
        return;

    const auto stamp = currentMpmStamp();
    const uint64_t step = physics_scene_coordinator_ != nullptr
                              ? physics_scene_coordinator_->stamp().step_sequence
                              : 0;
    const double time = physics_scene_coordinator_ != nullptr
                            ? physics_scene_coordinator_->stamp().simulation_time_nanos * 1e-9
                            : 0.0;

    // ⚠ Read under the physics lock, exactly as the collision overlay does: describeColliders
    // walks live solver tables while the physics thread may be stepping.
    std::vector<urdf::PhysicsColliderSet> robots;
    {
        physics_world_->lock();
        robots = gatherColliders();
        physics_world_->unlock();
    }
    mpm_publisher_->publishState(stamp, step, time, robots);

    const auto health = mpm_publisher_->health(stamp);
    const double now = FPlatformTime::Seconds();

    // ⚠ A stale or wrong-epoch sidecar is said ONCE, not every tick. Plan §M2 requires the
    // condition to surface with a diagnostic; it does not require it to drown the log.
    if (health.sidecar_seen && !health.responsive && !mpm_stale_reported_) {
        mpm_stale_reported_ = true;
        UE_LOG(LogPhysicsCoordinator, Error, TEXT("%s"), UTF8_TO_TCHAR(health.describe().c_str()));
    }
    else if (health.responsive && mpm_stale_reported_) {
        mpm_stale_reported_ = false;
        UE_LOG(LogPhysicsCoordinator, Log, TEXT("MPM sidecar recovered: %s"),
               UTF8_TO_TCHAR(health.describe().c_str()));
    }

    if (now - last_mpm_health_log_ > 10.0) {
        last_mpm_health_log_ = now;
        UE_LOG(LogPhysicsCoordinator, Log, TEXT("%s"), UTF8_TO_TCHAR(health.describe().c_str()));
    }
}

void ASimModeWorldBase::drawCollisionDebugOverlay()
{
    if (!PhysicsCollisionDebugDraw::IsEnabled() || physics_world_ == nullptr)
        return;

    const float world_to_meters = getGlobalNedTransform().fromNed(1.0f);

    FVector focus = FVector::ZeroVector;
    bool have_focus = false;
    if (APlayerController* controller = GetWorld() ? GetWorld()->GetFirstPlayerController()
                                                   : nullptr) {
        if (controller->PlayerCameraManager != nullptr) {
            focus = controller->PlayerCameraManager->GetCameraLocation();
            have_focus = true;
        }
    }

    urdf::CollisionDebugFilter filter =
        PhysicsCollisionDebugDraw::MakeFilter(focus, world_to_meters, have_focus);

    // ⚠ READ UNDER THE LOCK, DRAW OUTSIDE IT. The snapshot walks mjData->geom_xpos and Box3D's live
    // shapes while the physics thread may be mid-step; the draw calls are game-thread work that has
    // no business holding the solver up. Copying the snapshots out is what lets the two be
    // separate, and is the reason CollisionDebugSnapshot is owned data rather than a view.
    std::vector<urdf::CollisionDebugSnapshot> snapshots;
    {
        physics_world_->lock();
        if (coordinated_physics_scene_ != nullptr) {
            urdf::CollisionDebugSnapshot shared;
            if (coordinated_physics_scene_->collisionDebugGeometry(filter, shared))
                snapshots.push_back(std::move(shared));
        }
        else {
            // ⚠ LEGACY IS N PRIVATE WORLDS, and that is exactly why a mixed-backend comparison can
            // only be run here: a Box3D rover and a MuJoCo rover each solve alone, so nothing has
            // to carry contact between two engines. One snapshot per vehicle keeps the backends
            // distinguishable, which a merged one would not.
            //
            // ⚠ World geometry from the FIRST vehicle that can supply it, and no other. Every
            // private world mirrors the SAME level, so asking each one would draw N identical
            // copies of one wireframe - invisible on screen, and N times the cost.
            bool world_taken = false;
            for (auto& api : getApiProvider()->getVehicleSimApis()) {
                urdf::CollisionDebugFilter one = filter;
                one.include_world = filter.include_world && !world_taken;

                urdf::CollisionDebugSnapshot snapshot;
                if (!api->collisionDebugGeometry(one, snapshot))
                    continue;
                if (one.include_world)
                    world_taken = true;
                snapshots.push_back(std::move(snapshot));
            }
        }
        physics_world_->unlock();
    }
    if (snapshots.empty())
        return;

    for (const urdf::CollisionDebugSnapshot& snapshot : snapshots)
        PhysicsCollisionDebugDraw::Draw(GetWorld(), snapshot, world_to_meters);

    // Summarised on a timer rather than per frame: the numbers are what make "it looks too big" a
    // measurement, and a 60 Hz log of them would bury everything else in Blocks.log.
    const double now = FPlatformTime::Seconds();
    if (now - last_collision_overlay_log_ > 5.0) {
        last_collision_overlay_log_ = now;
        for (const urdf::CollisionDebugSnapshot& snapshot : snapshots)
            PhysicsCollisionDebugDraw::LogSummary(snapshot);
    }
}

void ASimModeWorldBase::reset()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        if (physics_world_)
            physics_world_->reset();
    },
                                             true);

    //no need to call base reset because of our custom implementation
}

std::string ASimModeWorldBase::getDebugReport()
{
    return physics_world_ ? physics_world_->getDebugReport() : std::string();
}
