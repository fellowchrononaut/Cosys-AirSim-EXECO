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
#include <signal.h>
#include "Materials/MaterialInterface.h"
#include "Engine/World.h"
#include <algorithm>
#include <map>
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
        startMpmSidecarProcess();
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
        startMpmSidecarProcess();

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
    // ⚠ THE CHILD GOES FIRST. It reads the segments this object owns, and stopping it before the
    // world is torn down is what makes "the sidecar cannot outlive its world" true rather than
    // merely intended — the whole point of D14.
    clearMpmImpulses();
    stopMpmSidecarProcess();

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
    applyMpmImpulses();
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

/// Plan §M2 requires a timeout or epoch mismatch to PAUSE with a diagnostic, not merely to be
/// reported. Reporting was what existed: an Error line once per episode while the sim carried on
/// pushing sand with a sidecar that was no longer keeping up, or was in a world that no longer
/// existed. The verb in the exit criterion is the point.
TAutoConsoleVariable<int32> CVarMpmPauseOnStall(
    TEXT("airsim.MpmPauseOnStall"), 1,
    TEXT("Pause the simulation when the MPM sidecar stalls or reports a wrong world.\n"
         "0: report only (pre-M2 behaviour)\n1: pause, per plan §M2"),
    ECVF_Default);

/// Plan §M2: "A sidecar restart forces global reset because MPM environmental memory was lost."
/// A restarted sidecar rebuilds a PRISTINE bed while the rigid world carries on from wherever the
/// robot had driven — so the rover's history includes deforming sand that no longer exists. The two
/// halves disagree about the past, and only a global reset makes them agree again.
/// Plan M3/M4. ⚠ EXPERIMENTAL: the articulated effective-inertia question in plan 11.1 is open, so
/// a run with this enabled records LaggedImpulseTwoWay and never TwoWay.
TAutoConsoleVariable<int32> CVarMpmTwoWay(
    TEXT("airsim.MpmTwoWay"), 0,
    TEXT("Apply the sand's reaction impulses back to the rover.\n0: off (M2 behaviour)\n"
         "1: on — requires the sidecar to be running with --two-way"),
    ECVF_Default);

/// ⚠ NEITHER CONTRACT IS OBVIOUSLY RIGHT, which is why both exist. Newton reports impulses (N.s)
/// accumulated over its own frame; both rigid backends accept newtons. Dividing by the wrong
/// interval scales every force the sand applies, silently and plausibly.
TAutoConsoleVariable<int32> CVarMpmImpulseContract(
    TEXT("airsim.MpmImpulseContract"), 0,
    TEXT("How an MPM impulse becomes a rigid-body force.\n"
         "0: normalized force-hold — F = J / measured sim seconds since the last impulse, held "
         "until the next one, cleared when stale\n"
         "1: exact-once — F = J / one tick, applied for a single tick then cleared"),
    ECVF_Default);

/// Newton's own coupled-proxy solver never applies a harvested force raw — it blends it with the
/// previous one (`proxy_utils.py: blend_proxy_forces_kernel`):
///
///     force = relaxation * harvested + (1 - relaxation) * previous
///
/// ⚠ Upstream defaults this to 1.0 (raw) and documents a usable floor of 0.1, so 1.0 is kept here
/// too: a different default would be a silent physics choice made by this file rather than by an
/// operator. Lower it when the coupling oscillates — which it does, because we also still have the
/// double-support problem D10 describes, and relaxation bounds that fight without resolving it.
TAutoConsoleVariable<int32> CVarMpmReplacementPatch(
    TEXT("airsim.MpmReplacementPatch"), 0,
    TEXT("Plan D10 / RigidSurfacePolicy=AuthoredReplacementPatch. While an MPM-selected link is "
         "inside the terrain patch, suspend its collision against the MIRRORED RIGID GROUND so the "
         "sand alone carries it. 0 = off (the link rests on the rigid floor at the BED FLOOR, "
         "buried under the whole bed, and the sand can never support the vehicle)."),
    ECVF_Default);

TAutoConsoleVariable<float> CVarMpmReplacementHysteresis(
    TEXT("airsim.MpmReplacementHysteresis"), 0.05f,
    TEXT("Metres of hysteresis on the patch boundary. A link must be this far INSIDE to lose rigid "
         "support and this far OUTSIDE to regain it. Zero makes a link straddling the edge toggle "
         "every frame, and a Box3D filter change is 'almost as expensive as recreating the shape'."),
    ECVF_Default);

TAutoConsoleVariable<float> CVarMpmReplacementReach(
    TEXT("airsim.MpmReplacementReach"), 0.35f,
    TEXT("Metres above the bed's top and below its floor within which a link still counts as being "
         "in the patch. A wheel RESTING on a bed has its centre a radius proud of the surface, so a "
         "strict containment test excludes the case this feature exists for. Should exceed the "
         "largest coupled link's radius."),
    ECVF_Default);

TAutoConsoleVariable<float> CVarMpmImpulseRelaxation(
    TEXT("airsim.MpmImpulseRelaxation"), 1.0f,
    TEXT("Blend factor for the sand's reaction: 1.0 applies each frame's force raw, lower values "
         "carry more of the previous frame. Newton's coupled proxy documents 0.1 as the floor."),
    ECVF_Default);

TAutoConsoleVariable<float> CVarMpmImpulseMaxForce(
    TEXT("airsim.MpmImpulseMaxForce"), 150.0f,
    TEXT("Refuse a sand reaction above this many newtons on any one link, and clear the force. A "
         "guard against runaway energy injection, not a tuning knob — hitting it means the coupling "
         "is unstable. 0 disables the check."),
    ECVF_Default);

TAutoConsoleVariable<float> CVarMpmImpulseScale(
    TEXT("airsim.MpmImpulseScale"), 1.0f,
    TEXT("Debug multiplier on the sand's reaction. ⚠ Anything but 1.0 makes the run physically "
         "meaningless and is for bisecting instability only."),
    ECVF_Default);

TAutoConsoleVariable<int32> CVarMpmResetOnSidecarRestart(
    TEXT("airsim.MpmResetOnSidecarRestart"), 1,
    TEXT("What to do when the MPM sidecar is replaced by a fresh one mid-run.\n"
         "0: halt and let the operator decide\n1: force a global reset, per plan §M2"),
    ECVF_Default);

/// ⚠ AN EPOCH MISMATCH RIGHT AFTER OUR OWN RESET IS THE RESET PROPAGATING, NOT A FAULT. The sim
/// bumps the epoch, republishes, and for a few seconds the sidecar is still reporting the old one
/// while it tears down a scene and builds a new bed. Halting during that window would fire an
/// Error and pause the world on every successful reset — and with a lockstep firmware present, a
/// pause is exactly what must not happen. Measured rebuild: ~10-20 s at these patch sizes.
TAutoConsoleVariable<float> CVarMpmEpochGraceSeconds(
    TEXT("airsim.MpmEpochGraceSeconds"), 30.0f,
    TEXT("How long after a world-stamp change the sidecar may still report the old epoch before "
         "that counts as a fault rather than a rebuild in progress."),
    ECVF_Default);

TAutoConsoleVariable<float> CVarMpmStallSeconds(
    TEXT("airsim.MpmStallSeconds"), 3.0f,
    TEXT("How long the sidecar may stay beyond its lag budget before the sim pauses to let it "
         "catch up. Ordinary driving runs 6-16 steps behind a budget of 200, so this is about a "
         "stopped sidecar, not a slow one."),
    ECVF_Default);

/// ⚠ The published radius is ALREADY the solver's true particle radius, so drawing at scale 1 is
/// dimensionally correct and still looks nothing like the offline Newton render — because that
/// render draws every particle and this one draws a few percent of them. Sand at 6.7% density reads
/// as scattered grit, not a bed.
TAutoConsoleVariable<int32> CVarMpmRenderMatchDensity(
    TEXT("airsim.MpmRenderMatchDensity"), 1,
    TEXT("Compensate the drawn particle size for decimation, so a sampled view occupies the same "
         "volume as the solver's full particle set.\n0: draw at true radius\n1: match the bed"),
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

    const int32 cap = FMath::Max(1, CVarMpmRenderMax.GetValueOnGameThread());
    const int32 draw = FMath::Min(static_cast<int32>(frame.count), cap);

    // MATCH THE PICTURE, NOT THE PARTICLE. Two decimations stand between the solver and this view:
    // the sidecar strides its particles into shared memory, and the cap above trims again. Drawing
    // 1/N of the particles at N^(1/3) times the radius puts the same VOLUME on screen, which is
    // what makes a sampled bed look like the bed rather than like grit scattered over it.
    //
    // ⚠ A VIEW CORRECTION, and it is labelled as one: the periodic log reports the solver's true
    // radius alongside the factor applied, so nobody reads a fattened sphere as a real grain size.
    float density_scale = 1.0f;
    if (CVarMpmRenderMatchDensity.GetValueOnGameThread() != 0 && draw > 0 &&
        frame.total_particles > 0) {
        const double ratio =
            static_cast<double>(frame.total_particles) / static_cast<double>(draw);
        density_scale = static_cast<float>(FMath::Pow(ratio, 1.0 / 3.0));
    }

    // The engine sphere is 100 uu across, so a unit scale is a 1 m ball; scale to the real radius.
    const float scale = static_cast<float>(frame.radius) * 2.0f *
                        CVarMpmRenderScale.GetValueOnGameThread() * density_scale;

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
               TEXT("MPM render: %d instances = %.1f%% of the solver's %llu particles, drawn at "
                    "%.2fx the true %.4f m radius to match the bed's volume, sidecar step %llu at "
                    "t=%.2f s"),
               draw, frame.total_particles > 0 ? 100.0 * draw / frame.total_particles : 0.0,
               static_cast<unsigned long long>(frame.total_particles),
               density_scale * CVarMpmRenderScale.GetValueOnGameThread(), frame.radius,
               static_cast<unsigned long long>(frame.sidecar_step), frame.sidecar_time);
    }
}

void ASimModeWorldBase::openMpmSidecarLink()
{
    using AirSimSettings = msr::airlib::AirSimSettings;
    const auto& settings = getSettings();

    detectLockstepVehicles();

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
    // ⚠ LEGACY HAS NO EPOCH, so nothing ever told the sidecar a reset happened and the sand simply
    // carried on deformed across a reset that returned the rovers to their spawns. The stamp is the
    // ONLY reset signal on this wire, so Legacy has to supply one; a local counter bumped in
    // reset() is exactly as good, because the sidecar compares the stamp rather than interpreting
    // it.
    if (physics_scene_coordinator_ == nullptr) {
        stamp.reset_epoch = mpm_legacy_epoch_;
        return stamp;
    }
    if (physics_scene_coordinator_ != nullptr) {
        const auto& coordinator_stamp = physics_scene_coordinator_->stamp();
        stamp.world_id = coordinator_stamp.world.id;
        stamp.world_revision = coordinator_stamp.world.revision;
        stamp.manifest_revision = coordinator_stamp.manifest_revision;
        stamp.reset_epoch = coordinator_stamp.reset_epoch;
    }
    return stamp;
}

void ASimModeWorldBase::startMpmSidecarProcess()
{
    using AirSimSettings = msr::airlib::AirSimSettings;

    if (mpm_sidecar_running_)
        return;

    const AirSimSettings::DeformableTerrainSidecarSetting* sc = nullptr;
    FString terrain_name;
    for (const auto& entry : getSettings().deformable_terrains) {
        if (entry.second.enabled && entry.second.sidecar.has_sidecar &&
            entry.second.sidecar.auto_start) {
            sc = &entry.second.sidecar;
            terrain_name = UTF8_TO_TCHAR(entry.first.c_str());
            break;
        }
    }
    if (sc == nullptr)
        return; // nobody opted in — the operator owns the sidecar, as before D14

    // The script lives beside the simulator's own source, two levels above the Unreal project.
    FString script = UTF8_TO_TCHAR(sc->script.c_str());
    if (script.IsEmpty()) {
        script = FPaths::ConvertRelativePathToFull(
            FPaths::Combine(FPaths::ProjectDir(), TEXT("../../../mpm_sidecar/sidecar.py")));
    }
    const FString python = UTF8_TO_TCHAR(sc->python.c_str());

    // ⚠ SAY WHICH FILE IS MISSING. A launch that fails silently leaves a sim publishing colliders
    // into shared memory with nothing consuming them, which looks exactly like a sidecar that is
    // merely slow to build.
    if (!FPaths::FileExists(python)) {
        UE_LOG(LogPhysicsCoordinator, Error,
               TEXT("MPM sidecar AutoStart: interpreter not found at '%s' — set "
                    "DeformableTerrains.%s.Sidecar.Python to a python with the mpm_sidecar "
                    "requirements installed. Not starting."),
               *python, *terrain_name);
        return;
    }
    if (!FPaths::FileExists(script)) {
        UE_LOG(LogPhysicsCoordinator, Error,
               TEXT("MPM sidecar AutoStart: script not found at '%s'. Not starting."), *script);
        return;
    }

    FString log_file = UTF8_TO_TCHAR(sc->log_file.c_str());
    if (log_file.IsEmpty())
        log_file = FPaths::Combine(FPaths::ProjectLogDir(), TEXT("mpm_sidecar.log"));

    // ⚠ PASS OUR OWN PID. The child exits when we disappear, so an editor crash cannot strand a
    // process holding GPU memory — two such orphans were found by hand on 2026-08-26, one after 32
    // minutes.
    const uint32 own_pid = FPlatformProcess::GetCurrentProcessId();

    FString args = FString::Printf(
        TEXT("-u %s --dir /dev/shm --voxel-size %g --fps %g --density %g --particle-every %d "
             "--max-render-particles %d --parent-pid %u"),
        *script, sc->voxel_size, sc->fps, sc->density, sc->particle_every,
        sc->max_render_particles, own_pid);
    // ⚠ D13 MATERIAL, and it is not optional detail: wheel friction was the difference between a
    // 52 kg rover stalling at the toe of a mound and cresting it. Only forwarded when authored, so
    // an unset key still means "the sidecar decides" rather than "zero friction".
    if (sc->sand_friction > 0.0)
        args += FString::Printf(TEXT(" --sand-friction %g"), sc->sand_friction);
    if (sc->collider_friction > 0.0)
        args += FString::Printf(TEXT(" --collider-friction %g"), sc->collider_friction);
    if (sc->collider_friction_default > 0.0)
        args += FString::Printf(TEXT(" --collider-friction-default %g"),
                                sc->collider_friction_default);

    // ⚠ EXTRA ARGS LAST, so an operator debugging by hand can still override anything the schema
    // now models — argparse takes the final occurrence.
    if (!sc->extra_args.empty())
        args += FString::Printf(TEXT(" %s"), UTF8_TO_TCHAR(sc->extra_args.c_str()));

    // Through a shell so the child's output lands in a file the operator can tail. CreateProc's
    // pipes would need a reader thread on the game thread's side, which is a lot of machinery for
    // "write it to a log".
    const FString command = FString::Printf(TEXT("exec %s %s > %s 2>&1"), *python, *args, *log_file);
    const FString shell_args = FString::Printf(TEXT("-c \"%s\""), *command);

    mpm_sidecar_proc_ = FPlatformProcess::CreateProc(TEXT("/bin/sh"), *shell_args,
                                                     /*bLaunchDetached=*/false,
                                                     /*bLaunchHidden=*/true,
                                                     /*bLaunchReallyHidden=*/true,
                                                     &mpm_sidecar_pid_, 0, nullptr, nullptr);
    if (!mpm_sidecar_proc_.IsValid()) {
        UE_LOG(LogPhysicsCoordinator, Error,
               TEXT("MPM sidecar AutoStart: failed to launch '%s'. Not starting."), *python);
        return;
    }
    mpm_sidecar_running_ = true;

    // ⚠ NON-BLOCKING, DELIBERATELY. A 250 k-particle build is ~10 s warm and minutes on a cold Warp
    // kernel cache; waiting here would freeze PIE, and with a lockstep firmware present it would
    // breach PX4's 1 s tolerance outright. The sim runs without sand until the child reports in,
    // which is the same state as any manually started sidecar.
    UE_LOG(LogPhysicsCoordinator, Log,
           TEXT("MPM sidecar AutoStart: launched pid %u for terrain '%s' (voxel %g m, %g fps) — "
                "output at %s. The sim does not wait for it; sand appears when it reports in."),
           mpm_sidecar_pid_, *terrain_name, sc->voxel_size, sc->fps, *log_file);
}

void ASimModeWorldBase::stopMpmSidecarProcess()
{
    if (!mpm_sidecar_running_)
        return;
    mpm_sidecar_running_ = false;

    if (!mpm_sidecar_proc_.IsValid())
        return;

    // ⚠ SIGINT FIRST, not TerminateProc. The sidecar saves a pose recording and encodes any video
    // on its interrupt path; killing it outright discards both. TerminateProc is the escalation,
    // not the opening move.
    if (mpm_sidecar_pid_ != 0)
        kill(static_cast<pid_t>(mpm_sidecar_pid_), SIGINT);

    // Bounded wait, and it notices the process is gone rather than sleeping the whole budget.
    for (int i = 0; i < 50 && FPlatformProcess::IsProcRunning(mpm_sidecar_proc_); ++i)
        FPlatformProcess::Sleep(0.1f);

    if (FPlatformProcess::IsProcRunning(mpm_sidecar_proc_)) {
        UE_LOG(LogPhysicsCoordinator, Warning,
               TEXT("MPM sidecar pid %u did not exit within 5 s of SIGINT — terminating."),
               mpm_sidecar_pid_);
        FPlatformProcess::TerminateProc(mpm_sidecar_proc_, /*KillTree=*/true);
    }
    else {
        UE_LOG(LogPhysicsCoordinator, Log, TEXT("MPM sidecar pid %u stopped."), mpm_sidecar_pid_);
    }
    FPlatformProcess::CloseProc(mpm_sidecar_proc_);
    mpm_sidecar_proc_.Reset();
    mpm_sidecar_pid_ = 0;
}

double ASimModeWorldBase::currentSimTimeSeconds() const
{
    // ⚠ ONE DEFINITION, because there were two and they disagreed. publishMpmState fell back to the
    // simulation clock in Legacy while applyMpmImpulses fell back to 0.0 — so force-hold's
    // "measured interval" measured nothing, silently used the nominal MPM dt, and over-delivered
    // momentum by whatever the sidecar's lag happened to be. Measured 2026-08-26: a rover launched
    // 58 m and sand driven to z = -7127 m.
    if (physics_scene_coordinator_ != nullptr)
        return physics_scene_coordinator_->stamp().simulation_time_nanos * 1e-9;
    return static_cast<double>(msr::airlib::ClockFactory::get()->nowNanos()) * 1e-9;
}

void ASimModeWorldBase::applyMpmImpulses()
{
    const bool want = CVarMpmTwoWay.GetValueOnGameThread() != 0;

    // ⚠ CLEARING IS NOT OPTIONAL, AND IT IS NOT SYMMETRIC BETWEEN BACKENDS. MuJoCo's xfrc_applied
    // PERSISTS until overwritten; Box3D's applied force clears every step. So switching two-way off
    // — or losing the sidecar — would leave a MuJoCo rover being shoved forever by sand that
    // stopped touching it, while the same code on Box3D would quietly stop. Writing an explicit
    // zero once is what makes the two behave the same, and it is the same stale-state failure that
    // cost four bugs on 2026-08-26.
    if (!want || !mpm_impulse_reader_.isOpen()) {
        if (mpm_impulse_applied_) {
            clearMpmImpulses();
            if (!want && mpm_impulse_reader_.isOpen())
                UE_LOG(LogPhysicsCoordinator, Log,
                       TEXT("two-way off — sand reaction cleared to zero on every collider"));
        }
        if (!want)
            return;
    }

    if (!mpm_impulse_reader_.isOpen()) {
        if (!mpm_impulse_reader_.open("/dev/shm"))
            return;
        UE_LOG(LogPhysicsCoordinator, Warning,
               TEXT("TWO-WAY MPM ATTACHED. ⚠ This run is LaggedImpulseTwoWay, not TwoWay: the "
                    "impulses are a frame behind and the articulated effective inertia in plan "
                    "11.1 is unresolved. Do not report sinkage or traction from it as validated."));
    }

    msr::airlib::mpm::MpmImpulseReader::Frame frame;
    if (!mpm_impulse_reader_.read(frame, mpm_last_impulse_step_)) {
        // Nothing new. ⚠ Under exact-once the force must already be gone; under force-hold it is
        // held deliberately — but only for a bounded time, or a stopped sidecar becomes a constant
        // force nobody asked for.
        const double now = FPlatformTime::Seconds();
        if (mpm_impulse_applied_ && mpm_last_impulse_seconds_ > 0.0 &&
            now - mpm_last_impulse_seconds_ > 0.5) {
            UE_LOG(LogPhysicsCoordinator, Warning,
                   TEXT("no MPM impulse for %.2f s — clearing the sand's reaction rather than "
                        "holding a force from a frame that has passed."),
                   now - mpm_last_impulse_seconds_);
            clearMpmImpulses();
        }
        return;
    }

    // ⚠ WRONG WORLD, NO FORCE. An impulse computed against a bed from a previous epoch is not a
    // small error, it is a force from a different simulation.
    const auto stamp = currentMpmStamp();
    if (frame.stamp.world_id != stamp.world_id || frame.stamp.reset_epoch != stamp.reset_epoch) {
        if (mpm_impulse_applied_)
            clearMpmImpulses();
        return;
    }

    const double sim_now = currentSimTimeSeconds();

    // Interval this force must cover. Force-hold uses the MEASURED gap since the last impulse, so a
    // lagging sidecar delivers the right momentum rather than a fraction of it; exact-once spends
    // the whole impulse in a single tick.
    const bool exact_once = CVarMpmImpulseContract.GetValueOnGameThread() != 0;
    double interval = 0.0;
    if (exact_once) {
        interval = static_cast<double>(getPhysicsLoopPeriod()) * 1e-9;
    }
    else {
        interval = (mpm_last_impulse_sim_time_ > 0.0 && sim_now > mpm_last_impulse_sim_time_)
                       ? sim_now - mpm_last_impulse_sim_time_
                       : frame.mpm_dt;
    }
    if (interval <= 1e-9)
        interval = frame.mpm_dt > 1e-9 ? frame.mpm_dt : 1.0 / 120.0;

    // ⚠ A RUNAWAY MUST NOT BE SILENT. Lagged explicit coupling can inject energy, and the failure
    // mode is not subtle degradation — it is a rover leaving the map. This cap turns that into a
    // logged refusal, which is the difference between "the coupling is unstable here" and "the
    // simulation is broken and nobody knows why". It is a guard, not a fix: hitting it means the
    // contract or the cadence is wrong, and the run is not physically meaningful past that point.
    const float max_force = CVarMpmImpulseMaxForce.GetValueOnGameThread();
    const float scale = CVarMpmImpulseScale.GetValueOnGameThread();
    const int32 count = FMath::Min(static_cast<int32>(frame.collider_count),
                                   static_cast<int32>(mpm_impulse_targets_.size()));
    int32 pushed = 0;
    double biggest = 0.0;
    for (int32 i = 0; i < count; ++i) {
        const auto& target = mpm_impulse_targets_[i];
        if (target.api == nullptr)
            continue;
        const auto& imp = frame.colliders[i];
        // A collider the sand never touched gets an explicit zero, not a skip — see the clearing
        // note above; skipping would leave MuJoCo holding the last non-zero value.
        urdf::Wrench w;
        w.force = urdf::Vec3{ imp.linear.x / interval * scale, imp.linear.y / interval * scale,
                              imp.linear.z / interval * scale };
        w.torque = urdf::Vec3{ imp.angular.x / interval * scale, imp.angular.y / interval * scale,
                               imp.angular.z / interval * scale };
        w.at_center = true;

        // ⚠ RELAXATION, exactly as Newton's coupled proxy does it. Applying a lagged force raw
        // means every frame's correction lands in full on a state that has already moved, which is
        // how an explicit coupling pumps energy. Blending with the previous force is what turns a
        // divergent oscillation into a bounded one.
        {
            const float relax = FMath::Clamp(CVarMpmImpulseRelaxation.GetValueOnGameThread(),
                                             0.0f, 1.0f);
            if (relax < 1.0f && i < static_cast<int32>(mpm_previous_wrench_.size())) {
                const urdf::Wrench& prev = mpm_previous_wrench_[i];
                w.force = urdf::Vec3{ relax * w.force.x + (1.0f - relax) * prev.force.x,
                                      relax * w.force.y + (1.0f - relax) * prev.force.y,
                                      relax * w.force.z + (1.0f - relax) * prev.force.z };
                w.torque = urdf::Vec3{ relax * w.torque.x + (1.0f - relax) * prev.torque.x,
                                       relax * w.torque.y + (1.0f - relax) * prev.torque.y,
                                       relax * w.torque.z + (1.0f - relax) * prev.torque.z };
            }
            if (i < static_cast<int32>(mpm_previous_wrench_.size()))
                mpm_previous_wrench_[i] = w;
        }

        const double fmag = FMath::Sqrt(w.force.x * w.force.x + w.force.y * w.force.y +
                                        w.force.z * w.force.z);
        if (max_force > 0.0f && fmag > max_force) {
            if (!mpm_impulse_capped_) {
                mpm_impulse_capped_ = true;
                UE_LOG(LogPhysicsCoordinator, Error,
                       TEXT("MPM reaction of %.1f N on '%s' exceeds airsim.MpmImpulseMaxForce "
                            "(%.1f N) — REFUSING it and every one after, and clearing the sand's "
                            "force. The coupling is unstable; this run is not physically "
                            "meaningful. Try airsim.MpmImpulseContract 1, or a smaller "
                            "airsim.MpmImpulseScale to bisect."),
                       fmag, UTF8_TO_TCHAR(mpm_publisher_->publishedIds()[i].c_str()), max_force);
            }
            clearMpmImpulses();
            return;
        }
        target.api->applyLinkWrench(target.link_index, w);
        if (imp.contact_nodes > 0)
            ++pushed;
        biggest = FMath::Max(biggest, FMath::Sqrt(w.force.x * w.force.x + w.force.y * w.force.y +
                                                  w.force.z * w.force.z));
    }

    mpm_impulse_applied_ = true;
    mpm_last_impulse_step_ = frame.sidecar_step;
    mpm_last_impulse_seconds_ = FPlatformTime::Seconds();
    mpm_last_impulse_sim_time_ = sim_now;
    mpm_exact_once_pending_ = exact_once;

    const double now_log = FPlatformTime::Seconds();
    if (now_log - last_mpm_impulse_log_ > 10.0) {
        last_mpm_impulse_log_ = now_log;
        UE_LOG(LogPhysicsCoordinator, Log,
               TEXT("two-way: %d of %d collider(s) in contact, peak |F| %.2f N over a %.4f s "
                    "interval (%s), sidecar step %llu"),
               pushed, count, biggest, interval,
               exact_once ? TEXT("exact-once") : TEXT("force-hold"),
               static_cast<unsigned long long>(frame.sidecar_step));
    }
}

void ASimModeWorldBase::clearMpmImpulses()
{
    urdf::Wrench zero;
    zero.at_center = true;
    for (const auto& target : mpm_impulse_targets_) {
        if (target.api != nullptr)
            target.api->applyLinkWrench(target.link_index, zero);
    }
    mpm_impulse_applied_ = false;
    mpm_exact_once_pending_ = false;
    mpm_last_impulse_sim_time_ = 0.0;
    // ⚠ A cleared force must not be blended back in next frame — that would re-apply the very
    // thing the clear exists to remove.
    std::fill(mpm_previous_wrench_.begin(), mpm_previous_wrench_.end(), urdf::Wrench());
}

void ASimModeWorldBase::detectLockstepVehicles()
{
    // ⚠ NO dynamic_cast — Unreal builds with RTTI off, so `MavLinkVehicleSetting` cannot be
    // recovered from a `VehicleSetting*`. The vehicle TYPE string is the safe discriminator, and
    // being conservative here is the right bias: treating a MAVLink vehicle as lockstep when it
    // happens not to be costs some sand lag, while the reverse costs a flight controller.
    mpm_lockstep_vehicle_present_ = false;
    mpm_lockstep_vehicle_name_.Empty();

    using Settings = msr::airlib::AirSimSettings;
    for (const auto& entry : Settings::singleton().vehicles) {
        if (entry.second == nullptr)
            continue;
        const std::string& type = entry.second->vehicle_type;
        if (type == Settings::kVehicleTypePX4 || type == Settings::kVehicleTypeArduCopter ||
            type == Settings::kVehicleTypeArduCopterSolo) {
            mpm_lockstep_vehicle_present_ = true;
            mpm_lockstep_vehicle_name_ = UTF8_TO_TCHAR(entry.first.c_str());
            UE_LOG(LogPhysicsCoordinator, Log,
                   TEXT("vehicle '%s' is a MAVLink firmware (%s) — MPM back-pressure will not "
                        "pause this world; the sand lags instead."),
                   *mpm_lockstep_vehicle_name_, UTF8_TO_TCHAR(type.c_str()));
            break;
        }
    }
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
    mpm_registry_stamp_ = currentMpmStamp();

    // ⚠ RESOLVE ONCE, HERE. The impulse block is indexed by registry POSITION, and turning that
    // back into "which vehicle, which link" per tick would be a string lookup on the hot path. It
    // is rebuilt on every republish because a reset can rebuild the population underneath us, and a
    // cached link index into a destroyed body table is a use-after-free rather than a wrong number.
    mpm_impulse_targets_.clear();
    {
        std::map<std::string, MpmImpulseTarget> by_id;
        for (auto& api : getApiProvider()->getVehicleSimApis()) {
            urdf::PhysicsColliderSet set;
            if (!api->describeColliders(set))
                continue;
            for (const auto& collider : set.colliders)
                by_id[collider.stable_id] = MpmImpulseTarget{ api, collider.link_index };
        }
        for (const std::string& id : mpm_publisher_->publishedIds()) {
            auto found = by_id.find(id);
            mpm_impulse_targets_.push_back(found != by_id.end() ? found->second
                                                                : MpmImpulseTarget{ nullptr, 0 });
        }
    }
    mpm_previous_wrench_.assign(mpm_impulse_targets_.size(), urdf::Wrench());

    // ⚠ THE GATING'S BELIEF MUST BE DROPPED WITH THE BODIES IT DESCRIBED. resetImplementation()
    // destroys and rebuilds every solver body, so each link comes back with FULL world collision —
    // but `mpm_vehicle_suspended_` remembered which vehicles had it suspended. The coordinator
    // would then believe a vehicle's ground support was suspended while the fresh Box3D shapes
    // collide with the terrain, and skip the call that would have made it true — reading the
    // rigid floor's reaction as sand support.
    //
    // It happens to self-heal today because every vehicle respawns OUTSIDE the patch and the next
    // transition corrects it. That is a property of the current settings files, not of the code: a
    // vehicle authored to spawn inside its own bed would carry the divergence for the whole run.
    mpm_vehicle_suspended_.clear();
    mpm_ground_suspended_any_ = false;
    mpm_last_impulse_step_ = 0;
    mpm_last_impulse_sim_time_ = 0.0;

    // ⚠ MATCHING A LINK IS NOT THE SAME AS HAVING GEOMETRY, and the count below cannot tell them
    // apart. On 2026-08-26 a Legacy run reported "12 of 12 matched" while six of those colliders
    // carried shape_count = 0: correct ids, correct mass, live poses, and no collision shapes at
    // all. Newton built six bodies with no geometry, so the sand could neither touch them nor be
    // touched — and every gate we had reported success. A collider with no shapes is not a degraded
    // collider, it is an absent one wearing a name.
    {
        int32 empty = 0;
        FString names;
        for (const auto& set : robots) {
            for (const auto& collider : set.colliders) {
                if (!collider.shapes.empty())
                    continue;
                if (std::find(mpm_selected_ids_.begin(), mpm_selected_ids_.end(),
                              collider.stable_id) == mpm_selected_ids_.end())
                    continue;
                ++empty;
                if (empty <= 8) {
                    if (!names.IsEmpty()) names += TEXT(", ");
                    names += UTF8_TO_TCHAR(collider.stable_id.c_str());
                }
            }
        }
        if (empty > 0) {
            UE_LOG(LogPhysicsCoordinator, Error,
                   TEXT("%d selected collider(s) have NO COLLISION GEOMETRY and are invisible to "
                        "the sand in BOTH directions: %s%s. The backend described them with "
                        "shape_count = 0."),
                   empty, *names, empty > 8 ? TEXT(", ...") : TEXT(""));
        }
    }

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
    if (!mpm_publisher_ || !mpm_publisher_->isOpen())
        return;

    const auto stamp = currentMpmStamp();

    // ⚠ REPUBLISH WHEN THE WORLD STAMP MOVES. A global reset bumps `reset_epoch` and rebuilds the
    // population from the frozen manifest, so every collider pose goes back to its t=0 value. The
    // registry published for the PREVIOUS epoch then describes a run that has ended — the sand was
    // authored around poses that no longer hold — and the very next state block trips the sidecar's
    // epoch refusal. Republishing is the whole fix on this side, and it hands the sidecar a bed
    // built around the post-reset poses rather than the pre-reset ones.
    const bool stamp_moved =
        stamp.world_id != mpm_registry_stamp_.world_id ||
        stamp.world_revision != mpm_registry_stamp_.world_revision ||
        stamp.manifest_revision != mpm_registry_stamp_.manifest_revision ||
        stamp.reset_epoch != mpm_registry_stamp_.reset_epoch;
    if (!mpm_registry_published_ || stamp_moved) {
        if (mpm_registry_published_) {
            UE_LOG(LogPhysicsCoordinator, Log,
                   TEXT("world stamp moved (epoch %llu -> %llu) — republishing the MPM registry so "
                        "the sand is rebuilt around the post-reset poses"),
                   static_cast<unsigned long long>(mpm_registry_stamp_.reset_epoch),
                   static_cast<unsigned long long>(stamp.reset_epoch));
        }
        publishMpmRegistry();
        mpm_stamp_changed_seconds_ = FPlatformTime::Seconds();
        mpm_rebuild_wait_reported_ = false;
        // The rebuilt sidecar restarts its counters; that is expected here and must not be read as
        // a restart of a different process.
        mpm_last_acknowledged_step_ = 0;
    }
    // ⚠ LEGACY HAS NO COORDINATOR, AND THEREFORE NO STEP SEQUENCE. This used to publish a constant
    // step 0 and time 0 in that mode, which is not a degraded link — it is a DEAD one: the sidecar
    // consumes step 0 once and then sees `state.step <= acknowledged_step` forever, idling at 0 %
    // CPU while both ends report healthy. The comment at openMpmSidecarLink promised deformable
    // terrain worked in Legacy; nothing had ever run it there, because a mixed Box3D + MuJoCo
    // population is the only thing that forces Legacy, and that combination was tried for the first
    // time on 2026-08-26.
    //
    // The simulation clock is what actually advances in Legacy, so derive both from it. The step is
    // a count of physics periods rather than a solver-owned sequence — it need only be monotonic
    // and comparable, which is all the acknowledgement protocol asks of it.
    uint64_t step = 0;
    double time = 0.0;
    if (physics_scene_coordinator_ != nullptr) {
        step = physics_scene_coordinator_->stamp().step_sequence;
        time = physics_scene_coordinator_->stamp().simulation_time_nanos * 1e-9;
    }
    else {
        const auto now_nanos = msr::airlib::ClockFactory::get()->nowNanos();
        const int64_t period = FMath::Max<int64>(getPhysicsLoopPeriod(), 1);
        step = static_cast<uint64_t>(now_nanos / period);
        time = static_cast<double>(now_nanos) * 1e-9;
    }

    // ⚠ Read under the physics lock, exactly as the collision overlay does: describeColliders
    // walks live solver tables while the physics thread may be stepping.
    std::vector<urdf::PhysicsColliderSet> robots;
    {
        physics_world_->lock();
        robots = gatherColliders();
        physics_world_->unlock();
    }
    mpm_publisher_->publishState(stamp, step, time, robots);

    applyMpmGroundGating(robots);

    const auto health = mpm_publisher_->health(stamp);
    const double now = FPlatformTime::Seconds();

    applyMpmLinkPolicy(health, now);

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

void ASimModeWorldBase::applyMpmGroundGating(const std::vector<urdf::PhysicsColliderSet>& robots)
{
    // ⚠ WHAT THIS FIXES. Plan §11.4: inside an active patch a selected link must not ALSO receive
    // support from a rigid copy of the same terrain. The bed's floor is the level's ground, so a
    // wheel resting on the mirrored ground sits at the BOTTOM of the sand — measured 2026-08-26,
    // six wheels pinned at z=1.049 across 740 mm of driving through a bed spanning 0.994..1.247.
    // No work on the force path can make sand carry a vehicle the rigid floor is holding up.
    //
    // ⚠ THIS IS THE ONE SWITCH THAT CAN DROP A ROBOT THROUGH THE WORLD. Off by default, and it
    // must stay that way until a bed is known to carry the vehicle: with rigid support suspended
    // and sand that cannot hold it, the link falls to whatever is left below.
    const bool want = CVarMpmReplacementPatch.GetValueOnGameThread() != 0;
    if (!want && !mpm_ground_suspended_any_)
        return;                       // never armed and nothing to restore — the common case

    const AirSimSettings::DeformableTerrainSetting* terrain = nullptr;
    for (const auto& item : getSettings().deformable_terrains)
        if (item.second.enabled) { terrain = &item.second; break; }

    // The patch in SOLVER coordinates, the frame gatherColliders reports poses in. Converted here
    // rather than trusting the NED numbers, for the same reason the region publish does it: a Y
    // mirror on the wrong side of that boundary already cost this workstream a debugging cycle.
    urdf::Vec3 lo{}, hi{};
    if (terrain != nullptr && want) {
        const float world_to_meters = getGlobalNedTransform().fromNed(1.0f);
        const msr::airlib::Vector3r ned_center(static_cast<float>(terrain->region.center.x),
                                               static_cast<float>(terrain->region.center.y),
                                               static_cast<float>(terrain->region.center.z));
        const urdf::Vec3 c = UrdfTransform::toUrdfVec(
            getGlobalNedTransform().fromGlobalNed(ned_center), world_to_meters);
        const double hx = std::fabs(terrain->region.size.x) * 0.5;
        const double hy = std::fabs(terrain->region.size.y) * 0.5;
        const double hz = std::fabs(terrain->region.size.z) * 0.5;
        lo = urdf::Vec3{ c.x - hx, c.y - hy, c.z - hz };
        hi = urdf::Vec3{ c.x + hx, c.y + hy, c.z + hz };
    }

    const double band = FMath::Max(0.0f, CVarMpmReplacementHysteresis.GetValueOnGameThread());

    // Index the gathered poses by stable id, so a target resolves to the pose the sidecar was sent.
    std::map<std::string, urdf::Vec3> pose_by_id;
    // ⚠ AND ITS EXTENT. The floor test compared the link ORIGIN to the bed floor, so a 0.165 m
    // wheel had its BOTTOM 66 mm below the floor while its centre was still comfortably inside —
    // the guard never fired, the wheel sank past Newton's ground plane, and a collider below that
    // plane scoops particles through it: 7.8% of a Scout's bed reached -10 km on 2026-08-26. A
    // wheel is not a point, and the one place that mattered treated it as one.
    std::map<std::string, double> bound_by_id;
    for (const auto& set : robots) {
        for (const auto& c : set.colliders) {
            pose_by_id[c.stable_id] = c.position;
            double bound = 0.0;
            for (const auto& sh : c.shapes) {
                const double off = std::sqrt(sh.position.x * sh.position.x +
                                             sh.position.y * sh.position.y +
                                             sh.position.z * sh.position.z);
                double extent = std::max({ sh.radius, sh.half_length,
                                           std::fabs(sh.half_extents.x),
                                           std::fabs(sh.half_extents.y),
                                           std::fabs(sh.half_extents.z) });
                // A convex hull states its size only through its vertices; Box3D reports wheels
                // that way, so without this the dominant collider kind would bound to zero.
                for (const auto& v : sh.vertices)
                    extent = std::max(extent, std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
                bound = std::max(bound, off + extent);
            }
            bound_by_id[c.stable_id] = bound;
        }
    }

    // ⚠ ALL-OR-NOTHING PER VEHICLE, NOT PER LINK — and per link was actively harmful.
    //
    // The first version gated each link independently. A rover 0.3 m long crossing a patch edge
    // then spends the crossing with some wheels on the mirrored terrain and some suspended over
    // sand, which is worse than either extreme: measured 2026-08-26, 30 transitions during one
    // traverse and an ExoMy thrown while standing on three wheels and dangling on three. Half
    // supported is not an intermediate state between supported and unsupported; it is a lever.
    //
    // Deciding per VEHICLE turns the crossing into one clean handover. The AABB test is unchanged;
    // only the rule that consumes it is.
    std::map<msr::airlib::VehicleSimApiBase*, bool> all_inside;   // vehicle -> EVERY link inside
    for (size_t i = 0; i < mpm_impulse_targets_.size(); ++i) {
        const auto& target = mpm_impulse_targets_[i];
        if (target.api == nullptr)
            continue;

        // ⚠ HYSTERESIS FROM THE VEHICLE'S STATE, not the link's. Mixing the two would let one
        // wheel's margin fight another's and reintroduce the chatter this replaces.
        const auto held = mpm_vehicle_suspended_.find(target.api);
        const bool suspended_now = held != mpm_vehicle_suspended_.end() && held->second;
        const double m = suspended_now ? -band : band;

        bool inside = false;
        if (want && terrain != nullptr && i < mpm_selected_ids_.size()) {
            const auto found = pose_by_id.find(mpm_selected_ids_[i]);
            if (found != pose_by_id.end()) {
                const urdf::Vec3& p = found->second;
                const double reach =
                    FMath::Max(0.0f, CVarMpmReplacementReach.GetValueOnGameThread());
                // ⚠ THE REACH IS UPWARD ONLY, AND THE BED FLOOR IS A FLOOR OF LAST RESORT.
                //
                // A symmetric window let a sinking vehicle stay "inside" all the way down, so it
                // never got its rigid support back and fell out of the world — observed
                // 2026-08-26 with an ExoMy whose 1.8-voxel wheels the sand cannot carry. Plan
                // §11.4 already specifies the correct shape: "A container floor below the particle
                // bed is valid; a coincident rigid surface at the particle top is not."
                //
                // So: generous ABOVE the bed, because a wheel resting on the surface has its
                // centre a radius proud of it; hard cut AT the bed floor, because a link that has
                // sunk that far has been abandoned by the sand and the rigid ground must catch it.
                // Leaving the patch downwards is a real physical event, not an edge case, and it
                // is reported below rather than passed over in silence.
                // The floor test uses the link's LOWEST point, not its origin.
                const auto b = bound_by_id.find(mpm_selected_ids_[i]);
                const double half = b != bound_by_id.end() ? b->second : 0.0;
                inside = p.x > lo.x + m && p.x < hi.x - m &&
                         p.y > lo.y + m && p.y < hi.y - m &&
                         p.z - half > lo.z + m && p.z < hi.z + reach;
            }
        }
        auto slot = all_inside.find(target.api);
        if (slot == all_inside.end())
            all_inside.emplace(target.api, inside);
        else
            slot->second = slot->second && inside;
    }

    int32 suspended = 0, restored = 0;
    bool any_suspended = false;
    for (const auto& entry : all_inside) {
        msr::airlib::VehicleSimApiBase* api = entry.first;
        const bool want_suspend = entry.second;
        const auto held = mpm_vehicle_suspended_.find(api);
        const bool suspended_now = held != mpm_vehicle_suspended_.end() && held->second;
        if (want_suspend == suspended_now) {
            any_suspended = any_suspended || suspended_now;
            continue;
        }

        // Applied to EVERY coupled link of this vehicle in one pass, so it is never left in the
        // mixed state the per-link rule created.
        bool all_ok = true;
        for (const auto& t : mpm_impulse_targets_) {
            if (t.api != api)
                continue;
            if (!t.api->setLinkWorldCollision(t.link_index, !want_suspend))
                all_ok = false;
        }

        // ⚠ ONLY RECORD IT IF THE BACKEND ACTUALLY DID IT. Believing support was suspended when it
        // was not turns the rigid floor's reaction into "the sand is carrying the vehicle".
        if (!all_ok) {
            if (!mpm_ground_unsupported_reported_) {
                mpm_ground_unsupported_reported_ = true;
                UE_LOG(LogPhysicsCoordinator, Error,
                       TEXT("airsim.MpmReplacementPatch is ON but this vehicle's backend could not "
                            "suspend rigid ground collision on every coupled link. The wheels are "
                            "STILL resting on the mirrored terrain; any sinkage or support measured "
                            "from this run is the rigid floor, not the sand."));
            }
            continue;
        }
        // ⚠ SAY WHY IT CAME BACK. Restoring support because the vehicle DROVE OUT is routine;
        // restoring it because the vehicle SANK THROUGH THE BED is the sand failing to carry it,
        // and the two must not look the same in a log. The discriminator is whether any coupled
        // link is now at or below the bed floor.
        if (!want_suspend && suspended_now) {
            bool sank = false;
            for (size_t i = 0; i < mpm_impulse_targets_.size(); ++i) {
                if (mpm_impulse_targets_[i].api != api || i >= mpm_selected_ids_.size())
                    continue;
                const auto found = pose_by_id.find(mpm_selected_ids_[i]);
                const auto bb = bound_by_id.find(mpm_selected_ids_[i]);
                const double bhalf = bb != bound_by_id.end() ? bb->second : 0.0;
                if (found != pose_by_id.end() && found->second.z - bhalf <= lo.z)
                    sank = true;
            }
            if (sank) {
                UE_LOG(LogPhysicsCoordinator, Warning,
                       TEXT("the sand did NOT carry this vehicle: a coupled link reached the bed "
                            "floor (z=%.3f), so rigid ground support has been restored to stop it "
                            "falling out of the world. The bed cannot support this vehicle at this "
                            "resolution — check the wheel radius in voxels before reading anything "
                            "else from the run."),
                       lo.z);
            }
        }

        mpm_vehicle_suspended_[api] = want_suspend;
        want_suspend ? ++suspended : ++restored;
        any_suspended = any_suspended || want_suspend;
    }

    mpm_ground_suspended_any_ = any_suspended;
    if (suspended > 0 || restored > 0) {
        UE_LOG(LogPhysicsCoordinator, Log,
               TEXT("replacement patch: %d vehicle(s) fully entered the bed and gave up "
                    "rigid ground, %d left it and got it back (hysteresis %.3f m)"),
               suspended, restored, band);
    }
}

void ASimModeWorldBase::applyMpmLinkPolicy(const msr::airlib::mpm::MpmSidecarPublisher::Health& health,
                                          double now)
{
    // ⚠ NO SIDECAR IS NOT A FAULT. The link is optional and most runs have none; pausing a sim that
    // never asked for sand would be a regression dressed as a safety check.
    if (!health.sidecar_seen || CVarMpmPauseOnStall.GetValueOnGameThread() == 0)
        return;

    // A RESTARTED SIDECAR. Steps never go backwards inside one run, so an acknowledgement that
    // moved backwards while the world stamp did NOT change can only be a different process: the old
    // one died and a new one built a pristine bed. Our own rebuild-on-reset also restarts the
    // counter, but that comes WITH an epoch change, which is why the stamp check is the
    // discriminator rather than the counter alone.
    if (health.epoch_matches && health.fault == 0 && mpm_last_acknowledged_step_ > 0 &&
        health.acknowledged_step < mpm_last_acknowledged_step_ && !mpm_restart_handled_) {
        // ⚠ A FORCED RESET MUST NEVER RUN AWAY. Even with the epoch-scoped tracking above, this
        // path triggers a world reset from a heuristic, and a heuristic that can fire twice a
        // second is a denial of service against the operator. Rate-limited as defence in depth:
        // the root cause is fixed, and this makes the next root cause survivable.
        const double since_last_forced = now - mpm_last_forced_reset_seconds_;
        if (mpm_last_forced_reset_seconds_ > 0.0 && since_last_forced < 15.0) {
            UE_LOG(LogPhysicsCoordinator, Warning,
                   TEXT("MPM sidecar looks restarted again after only %.1f s — NOT forcing another "
                        "global reset. Something is wrong with the link rather than with the run; "
                        "check the sidecar log."),
                   since_last_forced);
            mpm_restart_handled_ = true;
            return;
        }
        mpm_last_forced_reset_seconds_ = now;

        mpm_restart_handled_ = true;
        const bool do_reset = CVarMpmResetOnSidecarRestart.GetValueOnGameThread() != 0;
        UE_LOG(LogPhysicsCoordinator, Warning,
               TEXT("the MPM sidecar was RESTARTED (acknowledged step went %llu -> %llu inside one "
                    "epoch). Its sand is pristine while this world has kept driving, so the two no "
                    "longer share a past. %s"),
               static_cast<unsigned long long>(mpm_last_acknowledged_step_),
               static_cast<unsigned long long>(health.acknowledged_step),
               do_reset ? TEXT("Forcing a global reset, per plan §M2.")
                        : TEXT("Halting instead (airsim.MpmResetOnSidecarRestart 0)."));
        mpm_last_acknowledged_step_ = 0;
        if (do_reset) {
            // The reset advances reset_epoch, which republishes the registry, which makes the new
            // sidecar rebuild against the post-reset poses. The chain closes itself.
            reset();
        }
        else {
            mpm_halted_ = true;
            mpm_paused_by_link_ = true;
            pause(true);
        }
        return;
    }
    // ⚠ ONLY TRACK ACKNOWLEDGEMENTS FROM THIS EPOCH. D9a restarts step_seq at zero in every reset
    // epoch, so a step number from a different epoch is not comparable to one from this epoch — it
    // is a different counter that happens to have the same type.
    //
    // Getting this wrong produced an infinite reset loop on 2026-08-26: during the rebuild grace
    // window the sidecar still reports the PREVIOUS epoch, and taking its acknowledgement (145) as
    // this epoch's baseline meant the rebuilt sidecar's honest first report (0) looked like a
    // different process starting — which forced another reset, which started another rebuild.
    // Epochs climbed 104, 105, 106 … about twice a second.
    if (health.epoch_matches) {
        if (health.acknowledged_step >= mpm_last_acknowledged_step_)
            mpm_last_acknowledged_step_ = health.acknowledged_step;
        if (health.acknowledged_step > 0)
            mpm_restart_handled_ = false;
    }

    // HARD HALT — a wrong world or a reported fault. Catching up cannot fix either: the sidecar is
    // simulating a scene that no longer exists, or its GPU work failed outright. Plan §M2 says a
    // sidecar restart forces a global reset for exactly this reason, so this pause does NOT lift
    // itself; the operator resets or restarts deliberately.
    // ⚠ AN ALL-ZERO STAMP IS "HAS NOT REPORTED YET", NOT "A DIFFERENT WORLD". A sidecar that has
    // written its magic but not yet its stamp reads as world 0/0 manifest 0 epoch 0, and treating
    // that as a mismatched epoch halts a run that is merely starting up. It cost a whole test run.
    const bool stamp_unreported = !health.epoch_matches && health.fault == 0 &&
                                  health.acknowledged_step == 0 && health.particle_count == 0;
    if (stamp_unreported)
        return;

    if (!health.epoch_matches || health.fault != 0) {
        // Give a rebuild room to happen. Only an epoch mismatch gets the grace — a reported FAULT
        // is the sidecar itself saying it cannot continue, and waiting on that helps nobody.
        if (!health.epoch_matches && health.fault == 0 && mpm_stamp_changed_seconds_ > 0.0 &&
            now - mpm_stamp_changed_seconds_ < CVarMpmEpochGraceSeconds.GetValueOnGameThread()) {
            if (!mpm_rebuild_wait_reported_) {
                mpm_rebuild_wait_reported_ = true;
                UE_LOG(LogPhysicsCoordinator, Log,
                       TEXT("waiting for the MPM sidecar to rebuild for the new epoch (%s)"),
                       UTF8_TO_TCHAR(health.describe().c_str()));
            }
            return;
        }
        if (!mpm_halted_) {
            mpm_halted_ = true;
            mpm_paused_by_link_ = true;
            pause(true);
            UE_LOG(LogPhysicsCoordinator, Error,
                   TEXT("PAUSED — the MPM sidecar cannot be trusted: %s. Its sand no longer "
                        "corresponds to this world, and continuing would deform a bed from a run "
                        "that has ended. Reset the world (or restart the sidecar and reset) to "
                        "resume; airsim.MpmPauseOnStall 0 disables this check."),
                   UTF8_TO_TCHAR(health.describe().c_str()));
        }
        return;
    }

    // ⚠ BACK-PRESSURE IS OFF WHEN A LOCKSTEP FIRMWARE OWNS THE CLOCK. PX4 SITL in lockstep drops
    // out after ONE SECOND without a HilActuatorControls exchange
    // (MavLinkMultirotorApi.hpp: `last_update_time_ + 1000000 < now` -> "resetting lock step
    // mode"), and a back-pressure pause lasts as long as the sidecar needs to drain — seconds. So
    // holding the clock for the sand would silently break the flight controller, which is a far
    // worse outcome than sand that lags. Two subsystems cannot both own the clock; the firmware
    // wins, and the sand degrades to report-only.
    //
    // A HARD HALT still fires above, deliberately: that run is already invalid, and quietly flying
    // a drone while deforming a bed from a world that has ended is not a kinder failure.
    if (mpm_lockstep_vehicle_present_) {
        if (!health.responsive && !mpm_lockstep_conflict_reported_) {
            mpm_lockstep_conflict_reported_ = true;
            UE_LOG(LogPhysicsCoordinator, Warning,
                   TEXT("MPM sidecar is beyond its lag budget (%llu steps), but back-pressure is "
                        "DISABLED because vehicle '%s' runs a lockstep firmware that drops out "
                        "after 1 s without an exchange. The sand will lag instead of the clock "
                        "stopping. airsim.MpmPauseOnStall does not override this."),
                   static_cast<unsigned long long>(health.lag_steps),
                   *mpm_lockstep_vehicle_name_);
        }
        if (health.responsive)
            mpm_lockstep_conflict_reported_ = false;
        return;
    }

    // BACK-PRESSURE — the sidecar is merely behind. Pausing IS the fix here: the sim stops
    // advancing, the sidecar drains its backlog, and we resume. That makes the pause
    // self-correcting rather than something an operator has to notice and undo.
    if (!health.responsive) {
        if (mpm_unresponsive_since_ <= 0.0) {
            mpm_unresponsive_since_ = now;
        }
        else if (!mpm_paused_by_link_ &&
                 now - mpm_unresponsive_since_ > CVarMpmStallSeconds.GetValueOnGameThread()) {
            mpm_paused_by_link_ = true;
            pause(true);
            UE_LOG(LogPhysicsCoordinator, Warning,
                   TEXT("PAUSED for back-pressure — the MPM sidecar has been beyond its lag budget "
                        "for %.1f s (%llu steps behind). Holding the clock so it can catch up; the "
                        "sim resumes on its own."),
                   now - mpm_unresponsive_since_,
                   static_cast<unsigned long long>(health.lag_steps));
        }
        return;
    }

    mpm_unresponsive_since_ = 0.0;
    mpm_rebuild_wait_reported_ = false;
    // ⚠ Only lift a pause WE applied. An operator who paused during a stall should stay paused.
    //
    // ⚠ AND A HALT MUST BE LIFTABLE. I first wrote this so a wrong-epoch halt never resumed, which
    // is wrong: a global reset followed by the sidecar adopting the new epoch is precisely the
    // documented recovery, and health saying "back in this world" is the evidence that it happened.
    // Refusing to resume on good health would turn the recovery path into a dead end.
    if (mpm_paused_by_link_) {
        const bool was_halt = mpm_halted_;
        mpm_paused_by_link_ = false;
        mpm_halted_ = false;
        pause(false);
        UE_LOG(LogPhysicsCoordinator, Log,
               TEXT("resumed — the MPM sidecar %s (%llu steps behind, within budget)"),
               was_halt ? TEXT("is back in this world") : TEXT("caught up"),
               static_cast<unsigned long long>(health.lag_steps));
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
    // ⚠ SAY EVERY TIME THIS RUNS. A single Backspace produced TWO epoch bumps ~1 s apart on
    // 2026-08-26, so the sand rebuilt twice for one operator reset. The sand was right: reset()
    // genuinely ran twice. The only 1-second-delayed reset in the codebase is
    // `WorldSimApi::spawnPlayer` (`sleep_for(1s); simmode_->reset()`), which is the prime suspect
    // and unproven. This line turns "it resets twice" into a timestamped count.
    UE_LOG(LogPhysicsCoordinator, Log, TEXT("ASimModeWorldBase::reset() called (frame %u)"),
           GFrameNumber);

    // Legacy's only reset signal to the MPM sidecar — see currentMpmStamp.
    //
    // ⚠ COLLAPSE MULTIPLE CALLS IN ONE FRAME. reset() arrives more than once per operator reset —
    // measured 2026-08-26 as pairs 0.8 s apart with two vehicles present — and each bump made the
    // sidecar tear down and rebuild 634 k particles again. One reset must be one epoch, so the bump
    // is keyed to the frame number rather than the call.
    if (physics_scene_coordinator_ == nullptr) {
        const uint32 frame = GFrameNumber;
        if (frame != mpm_legacy_epoch_frame_) {
            mpm_legacy_epoch_frame_ = frame;
            ++mpm_legacy_epoch_;
        }
    }

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
