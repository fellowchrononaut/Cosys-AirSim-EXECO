#pragma once

// I-R Phase 0 instrumentation.
//
// The A/B measurement established that LiDAR raycasting accounts for the entire real-time deficit
// (sim/wall 0.835 with LiDARs on, 1.000 with them off). It did NOT establish *why*, and the two
// candidates imply different architectures:
//
//   A. Physics-thread OCCUPANCY  - getPointCloud simply dominates the World::update() loop.
//                                  Fix: move raycasting to any other thread.
//   B. Chaos scene-lock CONTENTION - the queries fight the game thread's physics scene.
//                                  Fix: a worker thread does NOT help; needs UE async traces or GPU.
//
// Discriminator:
//   A  -> getPointCloud wall time is a large fraction of the loop interval, and the interval grows
//         in step with it. Game-thread Tick and its lock wait stay small.
//   B  -> loop interval looks reasonable relative to the work, but the GAME thread degrades: Tick
//         duration and especially the wait on physics_world_->lock() balloon.
//
// Enable: airsim.LogPhysicsTiming <seconds>   (0 = off). Prints one aggregated line per window per
// site; never per-call, which would itself perturb what it measures.

#include "CoreMinimal.h"
#include "HAL/IConsoleManager.h"
#include <chrono>

extern TAutoConsoleVariable<int32> CVarLogPhysicsTiming;

namespace AirSimPhysicsTiming
{
    using Clock = std::chrono::steady_clock;

    inline double ToMs(Clock::duration d)
    {
        return std::chrono::duration<double, std::milli>(d).count();
    }

    // Single-threaded by construction: each instance is owned by exactly one thread (the physics
    // thread for the sensor sites, the game thread for Tick), so no synchronisation is needed and
    // the probe cannot perturb the contention it is trying to measure.
    struct Window
    {
        uint64 calls = 0;
        double busy_ms = 0.0;   // time spent inside the measured region
        double busy_max = 0.0;
        double gap_ms = 0.0;    // wall interval between successive entries = the loop period
        double gap_max = 0.0;
        bool have_last = false;
        Clock::time_point last_entry{};
        Clock::time_point window_start{};

        void noteEntry(Clock::time_point now)
        {
            if (have_last) {
                const double g = ToMs(now - last_entry);
                gap_ms += g;
                gap_max = FMath::Max(gap_max, g);
            }
            last_entry = now;
            have_last = true;
            if (calls == 0) window_start = now;
        }

        void noteBusy(double ms)
        {
            ++calls;
            busy_ms += ms;
            busy_max = FMath::Max(busy_max, ms);
        }

        // Returns true and resets once `period_s` of wall time has elapsed in this window.
        bool shouldReport(Clock::time_point now, double period_s)
        {
            return calls > 1 && ToMs(now - window_start) >= period_s * 1000.0;
        }

        void reset() { *this = Window(); }
    };

    // One scoped timer; records the interval since the previous entry, then the busy duration.
    struct Scope
    {
        Window& w;
        Clock::time_point t0;

        explicit Scope(Window& window) : w(window), t0(Clock::now()) { w.noteEntry(t0); }
        ~Scope() { w.noteBusy(ToMs(Clock::now() - t0)); }
    };

    inline int32 ReportPeriodSeconds()
    {
        return CVarLogPhysicsTiming.GetValueOnAnyThread();
    }
}
