#include "SimModeWorldBase.h"
#include "physics/FastPhysicsEngine.hpp"
#include "physics/ExternalPhysicsEngine.hpp"
#include <exception>
#include "AirBlueprintLib.h"
#include "PhysicsTiming.h"

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

void ASimModeWorldBase::initializeForPlay()
{
    std::vector<msr::airlib::UpdatableObject*> vehicles;
    for (auto& api : getApiProvider()->getVehicleSimApis())
        vehicles.push_back(api);
    //TODO: directly accept getVehicleSimApis() using generic container

    std::unique_ptr<PhysicsEngineBase> physics_engine = createPhysicsEngine();
    physics_engine_ = physics_engine.get();
    physics_world_.reset(new msr::airlib::PhysicsWorld(std::move(physics_engine),
                                                       vehicles,
                                                       getPhysicsLoopPeriod()));
}

void ASimModeWorldBase::registerPhysicsBody(msr::airlib::VehicleSimApiBase* physicsBody)
{
    // Reset the vehicle as well before registering it
    // Similar to what happens in initializeForPlay() above
    physicsBody->reset();
    physics_world_.get()->addBody(physicsBody);
}

void ASimModeWorldBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //remove everything that we created in BeginPlay
    physics_world_.reset();

    Super::EndPlay(EndPlayReason);
}

void ASimModeWorldBase::startAsyncUpdator()
{
    physics_world_->startAsyncUpdator();
}

void ASimModeWorldBase::stopAsyncUpdator()
{
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
    return physics_world_->isPaused();
}

void ASimModeWorldBase::pause(bool is_paused)
{
    physics_world_->pause(is_paused);
    ASimModeBase::pause(is_paused);
}

void ASimModeWorldBase::continueForTime(double seconds)
{
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
    physics_engine_->setWind(wind);
}

void ASimModeWorldBase::setExtForce(const msr::airlib::Vector3r& ext_force) const
{
    physics_engine_->setExtForce(ext_force);
}

void ASimModeWorldBase::updateDebugReport(msr::airlib::StateReporterWrapper& debug_reporter)
{
    unused(debug_reporter);
    //we use custom debug reporting for this class
}

void ASimModeWorldBase::Tick(float DeltaSeconds)
{
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

    Super::Tick(DeltaSeconds);
}

void ASimModeWorldBase::reset()
{
    UAirBlueprintLib::RunCommandOnGameThread([this]() {
        physics_world_->reset();
    },
                                             true);

    //no need to call base reset because of our custom implementation
}

std::string ASimModeWorldBase::getDebugReport()
{
    return physics_world_->getDebugReport();
}
