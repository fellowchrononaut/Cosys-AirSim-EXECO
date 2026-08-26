#pragma once

#include "CoreMinimal.h"
#include <memory>
#include <vector>
#include "Kismet/KismetSystemLibrary.h"
#include "api/VehicleSimApiBase.hpp"
#include "AssetRegistry/AssetData.h"
#include "physics/PhysicsEngineBase.hpp"
#include "CoordinatedPhysicsScene.h"
#include "mpm/MpmImpulseReader.hpp"
#include "mpm/MpmParticleReader.hpp"
#include "mpm/MpmSidecarPublisher.hpp"
#include "physics/PhysicsSceneCoordinator.hpp"
#include "physics/World.hpp"
#include "physics/PhysicsWorld.hpp"
#include "common/StateReporterWrapper.hpp"
#include "api/ApiServerBase.hpp"
#include "SimModeBase.h"
#include "SimModeWorldBase.generated.h"

extern CORE_API uint32 GFrameNumber;

UCLASS()
class AIRSIM_API ASimModeWorldBase : public ASimModeBase
{
    GENERATED_BODY()

public:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

    virtual void reset() override;
    virtual std::string getDebugReport() override;

    virtual bool isPaused() const override;
    virtual void pause(bool is_paused) override;
    virtual void continueForTime(double seconds) override;
    virtual void continueForFrames(uint32_t frames) override;

    virtual void setWind(const msr::airlib::Vector3r& wind) const override;
    virtual void setExtForce(const msr::airlib::Vector3r& ext_force) const override;

public:
    virtual class ICoordinatedPhysicsScene* getCoordinatedPhysicsScene() const override;

protected:
    virtual void preparePhysicsScene() override;
    void startAsyncUpdator();
    void stopAsyncUpdator();
    virtual void updateDebugReport(msr::airlib::StateReporterWrapper& debug_reporter) override;

    //should be called by derived class once all api_provider_ is ready to use
    void initializeForPlay();

    //used for adding physics bodies on the fly
    virtual void registerPhysicsBody(msr::airlib::VehicleSimApiBase* physicsBody) override;

    long long getPhysicsLoopPeriod() const;
    void setPhysicsLoopPeriod(long long period);

private:
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::PhysicsEngineBase PhysicsEngineBase;
    typedef msr::airlib::ClockFactory ClockFactory;

    //create the physics engine as needed from settings
    std::unique_ptr<PhysicsEngineBase> createPhysicsEngine();

    /// Draw the shared solver's collision geometry over the level, when the console variable asks
    /// for it. A view only — nothing in the simulation reads what it produces.
    void drawCollisionDebugOverlay();

    /// Open the Newton MPM sidecar link, if any DeformableTerrains entry is enabled. Returns the
    /// selected collider ids, which is empty when nothing opted in.
    void openMpmSidecarLink();
    /// Publish topology once, after the manifest commits.
    void publishMpmRegistry();
    /// Publish per-tick collider state and report the sidecar's health.
    void publishMpmState();
    /// Act on that health, per plan §M2: a stalled sidecar pauses the clock until it catches up; a
    /// wrong-epoch or faulted one halts. Both defer to a lockstep firmware — see
    /// detectLockstepVehicles.
    /// Plan D10: suspend rigid ground collision for MPM-selected links inside the patch, restore
    /// it outside, with hysteresis. Called once per state publish, where the poses are already in
    /// hand. Toggles the backend only on an actual crossing.
    void applyMpmGroundGating(const std::vector<urdf::PhysicsColliderSet>& robots);

    void applyMpmLinkPolicy(const msr::airlib::mpm::MpmSidecarPublisher::Health& health, double now);
    /// Note whether any vehicle runs a MAVLink firmware, which owns the clock in lockstep and
    /// therefore forbids back-pressure.
    void detectLockstepVehicles();
    /// Launch the sidecar as a child of this process when a terrain opted in (plan D14). Returns
    /// immediately: the build takes seconds to minutes and the clock must not wait for it.
    void startMpmSidecarProcess();
    /// SIGINT, bounded wait, then terminate. Called on EndPlay so a run never outlives its world.
    void stopMpmSidecarProcess();
    /// Read the sand's reaction and push it onto the rover (plan M3/M4, LaggedImpulseTwoWay).
    void applyMpmImpulses();
    /// Write an explicit zero wrench to every collider. ⚠ Required, not tidy-up: MuJoCo's
    /// xfrc_applied persists until overwritten, so "stop applying" is not the same as "apply zero".
    void clearMpmImpulses();
    /// Simulation time in seconds, from the coordinator when there is one and from the simulation
    /// clock otherwise. ⚠ ONE definition — two disagreeing ones caused a runaway.
    double currentSimTimeSeconds() const;
    /// Gather every vehicle's colliders, in a stable order.
    std::vector<urdf::PhysicsColliderSet> gatherColliders() const;
    /// The world stamp the sidecar compares against, from the coordinator.
    msr::airlib::mpm::WireWorldStamp currentMpmStamp() const;

    /// Draw the sidecar's sand in the level, when the console variable asks for it.
    ///
    /// ⚠ A VIEW, exactly like the collision overlay: the particles are a decimated, lagged copy of
    /// another process's GPU state and nothing in the simulation may read them.
    void updateMpmParticleRender();

private:
    // Declared before physics_world_ so the executor/private world is destroyed first even if
    // explicit EndPlay teardown is bypassed during an exceptional startup.
    std::unique_ptr<msr::airlib::PhysicsSceneCoordinator> physics_scene_coordinator_;
    // Declared after the coordinator so it is destroyed BEFORE it: this object holds the
    // coordinator by reference, and members are destroyed in reverse declaration order.
    //
    // ⚠ The type must be COMPLETE here. UHT emits `ASimModeWorldBase::ASimModeWorldBase() {}` into
    // the generated code, which instantiates this unique_ptr's destructor in a translation unit
    // that sees only this header — so a forward declaration does not compile.
    std::unique_ptr<FCoordinatedPhysicsScene> coordinated_physics_scene_;
    /// Wall-clock of the last collision-overlay summary, so the log is timed rather than per frame.
    double last_collision_overlay_log_ = 0.0;

    /// The Newton MPM deformable-terrain link. Null when no DeformableTerrains entry is enabled,
    /// which is the ordinary case and not a fault.
    std::unique_ptr<msr::airlib::mpm::MpmSidecarPublisher> mpm_publisher_;
    /// Collider ids selected by UrdfLinkPhysics.InteractWithMPM, in publish order.
    ///
    /// ⚠ EMPTY MEANS NONE, never all. A terrain enabled with no link opted in is a configuration
    /// error worth saying out loud, not an invitation to push the whole robot into the sand.
    std::vector<std::string> mpm_selected_ids_;
    double last_mpm_health_log_ = 0.0;
    bool mpm_registry_published_ = false;
    /// World stamp as of the last registry publish. A move in any field means the registry now
    /// describes a run that has ended, so it is republished.
    msr::airlib::mpm::WireWorldStamp mpm_registry_stamp_{};
    /// ⚠ Latched so a stale sidecar is reported ONCE per episode rather than every tick. A warning
    /// at 333 Hz is a warning nobody reads.
    bool mpm_stale_reported_ = false;
    /// Wall-clock at which the sidecar first fell beyond its lag budget; 0 when it is within it.
    double mpm_unresponsive_since_ = 0.0;
    /// True while THIS code holds the pause, so an operator's own pause is never lifted for them.
    bool mpm_paused_by_link_ = false;
    /// Latched for a wrong world or a reported fault. Never lifts itself — see plan §M2's
    /// "a sidecar restart forces global reset".
    bool mpm_halted_ = false;
    /// A MAVLink firmware is present, so this world's clock is not ours to stop.
    bool mpm_lockstep_vehicle_present_ = false;
    FString mpm_lockstep_vehicle_name_;
    bool mpm_lockstep_conflict_reported_ = false;
    /// Highest acknowledgement seen this epoch. A backwards move is a different sidecar process.
    uint64_t mpm_last_acknowledged_step_ = 0;
    /// Latched so one restart triggers one reset, not one per tick while the reset propagates.
    bool mpm_restart_handled_ = false;
    /// When a reset was last forced from the restart heuristic, so it cannot run away.
    double mpm_last_forced_reset_seconds_ = 0.0;
    /// When our own world stamp last moved, so a sidecar still on the old epoch is understood as
    /// rebuilding rather than faulted.
    double mpm_stamp_changed_seconds_ = 0.0;
    bool mpm_rebuild_wait_reported_ = false;
    /// The sidecar process this simulator owns, when a terrain set Sidecar.AutoStart.
    FProcHandle mpm_sidecar_proc_;
    uint32 mpm_sidecar_pid_ = 0;
    bool mpm_sidecar_running_ = false;

    /// Where each registry slot's reaction has to go, resolved once per registry publish.
    struct MpmImpulseTarget {
        msr::airlib::VehicleSimApiBase* api = nullptr;
        size_t link_index = 0;
    };
    std::vector<MpmImpulseTarget> mpm_impulse_targets_;
    /// Last force applied per collider, for relaxation blending. Same order as the targets.
    std::vector<urdf::Wrench> mpm_previous_wrench_;
    /// Per target: is this link currently INSIDE the patch with rigid ground support suspended?
    /// Only set when the backend confirmed the change, so it never claims support was removed when
    /// it was not. Same order as the targets.
    /// Per VEHICLE, not per link: does this vehicle currently have rigid ground support
    /// suspended? Gating one link at a time left a crossing rover half supported, which threw it —
    /// see applyMpmGroundGating. Only set when the backend confirmed every coupled link.
    std::map<msr::airlib::VehicleSimApiBase*, bool> mpm_vehicle_suspended_;
    /// Any link currently suspended — lets the gating return immediately when the feature has
    /// never been armed, and still run once to RESTORE support after it is turned off mid-run.
    bool mpm_ground_suspended_any_ = false;
    bool mpm_ground_unsupported_reported_ = false;
    msr::airlib::mpm::MpmImpulseReader mpm_impulse_reader_;
    uint64_t mpm_last_impulse_step_ = 0;
    double mpm_last_impulse_seconds_ = 0.0;
    /// Simulation time at the last applied impulse, so force-hold divides by the MEASURED interval
    /// rather than by a nominal dt the sidecar may not have kept up with.
    double mpm_last_impulse_sim_time_ = 0.0;
    bool mpm_impulse_applied_ = false;
    bool mpm_exact_once_pending_ = false;
    double last_mpm_impulse_log_ = 0.0;
    /// Latched once a reaction exceeded the cap, so a runaway is reported once rather than per tick.
    bool mpm_impulse_capped_ = false;
    /// Legacy's stand-in for reset_epoch, bumped on every reset.
    uint64_t mpm_legacy_epoch_ = 0;
    /// Frame the legacy epoch last advanced on, so repeated reset() calls in one frame are one
    /// reset rather than several rebuilds of the whole particle bed.
    uint32 mpm_legacy_epoch_frame_ = 0;

    /// Read end of the sidecar's particle stream, plus the instanced mesh that draws it.
    msr::airlib::mpm::MpmParticleReader mpm_particle_reader_;
    UPROPERTY()
    class UInstancedStaticMeshComponent* mpm_particle_mesh_ = nullptr;
    /// Sand-coloured instance of the engine's BasicShapeMaterial. Appearance only — nothing in the
    /// simulation reads it, exactly like the particles it colours.
    UPROPERTY()
    class UMaterialInstanceDynamic* mpm_particle_material_ = nullptr;
    /// Last applied colour/roughness, so the material is touched only when the cvars change.
    FString mpm_particle_appearance_;
    uint64_t mpm_last_render_step_ = 0;
    double last_mpm_render_log_ = 0.0;
    bool mpm_render_announced_ = false;
    /// Wall-clock of the last NEW particle frame, so a stopped sidecar can be noticed rather than
    /// leaving its last sand bed on screen indefinitely.
    double mpm_last_frame_seconds_ = 0.0;
    /// Step of the frame cleared as stale, so re-attaching to the dead sidecar's still-present
    /// segment does not redraw it and start a five-second clear/redraw flap.
    uint64_t mpm_stale_render_step_ = 0;
    std::unique_ptr<msr::airlib::PhysicsWorld> physics_world_;
    PhysicsEngineBase* physics_engine_ = nullptr;

    /*
    300Hz seems to be minimum for non-aggressive flights
    400Hz is needed for moderately aggressive flights (such as
    high yaw rate with simultaneous back move)
    500Hz is recommended for more aggressive flights
    Lenovo P50 high-end config laptop seems to be topping out at 400Hz.
    HP Z840 desktop high-end config seems to be able to go up to 500Hz.
    To increase freq with limited CPU power, switch Barometer to constant ref mode.
    */
    long long physics_loop_period_ = 3000000LL; //3ms
};
