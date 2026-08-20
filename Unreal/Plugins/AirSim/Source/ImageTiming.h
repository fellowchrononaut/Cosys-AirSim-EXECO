#pragma once

// I-G Step 0a instrumentation.
//
// simGetImages costs 68-96 ms of wall time per two-image call. That number has never been broken
// down, and the right fix depends entirely on the breakdown:
//
//   Latency-bound  - most of the cost is waiting for the game thread and then for the next
//                    OnEndDraw, i.e. up to two frames. At 28-57 fps that is 35-70 ms by itself,
//                    which would account for nearly all of it.
//                    Fix: D9 fleet-synchronous capture. Pay the wait ONCE for every camera on
//                    every vehicle instead of once per RPC call. Near-linear win.
//
//   Readback-bound - most of the cost is the serial blocking ReadSurfaceData per image, and/or
//                    the extra scene renders.
//                    Fix: D9 batching barely helps; needs async readback (FRHIGPUTextureReadback,
//                    persistent staging, an N-frame latency pipeline). Much larger change.
//
// Discriminator: hold resolution fixed and vary the number of images per call.
//   Latency-bound  -> total call time is roughly FLAT in N; segments a+b dominate and do not grow.
//   Readback-bound -> total call time grows roughly LINEARLY in N; segment d dominates.
//
// Segments, matching the table in sim_issues/Sim_Issues_Plan.md:
//   a  rpc_to_game     RPC thread enters getScreenshot -> AsyncTask actually runs on game thread
//   b  game_to_draw    AsyncTask runs -> OnEndDraw fires (the capture instant)
//   c+d draw_to_done   OnEndDraw -> last readback completes  (scene render + serial readback)
//   e  done_to_return  readback complete -> getScreenshot returns (compress/copy on RPC thread)
//
// c and d are not separated here on purpose: splitting them needs a GPU timestamp query, which is
// a bigger change than this probe justifies. If the sweep says readback-bound, that is the moment
// to add one - not before.
//
// Enable: airsim.LogImageTiming <seconds>   (0 = off). Prints one aggregated line per window,
// never per call, so the probe cannot perturb what it measures. Diagnostic only: changes no
// recorded data and no timestamps that reach a client.

#include "CoreMinimal.h"
#include "HAL/IConsoleManager.h"
#include <chrono>

extern TAutoConsoleVariable<int32> CVarLogImageTiming;

namespace AirSimImageTiming
{
    using Clock = std::chrono::steady_clock;

    inline double ToMs(Clock::duration d)
    {
        return std::chrono::duration<double, std::milli>(d).count();
    }

    inline int32 ReportPeriodSeconds()
    {
        return CVarLogImageTiming.GetValueOnAnyThread();
    }

    // One capture call's segment boundaries. Written by the RPC thread (a, e), the game thread
    // (b) and the render thread (c+d), but never concurrently: each stage happens-before the next
    // via the AsyncTask -> OnEndDraw -> ENQUEUE_RENDER_COMMAND -> wait_signal_ chain, and the RPC
    // thread is blocked on wait_signal_ for the whole middle section.
    struct Call
    {
        Clock::time_point t_rpc_enter{};
        Clock::time_point t_game_task{};
        Clock::time_point t_end_draw{};
        Clock::time_point t_readback_done{};
        unsigned int images = 0;
        bool valid = false;

        // Only filled by the batched readback path (airsim.GpuReadback 1), which is the one place
        // we own the split. Design #4 gave no speed-up, but it remains a useful INSTRUMENT: it
        // separates the GPU-completion wait from the CPU copy out of staging memory, and those two
        // want opposite fixes. A wait cannot be parallelised away; a copy can.
        double lock_ms = 0.0;   // sum of Lock() across the batch - GPU wait + mapping
        double copy_ms = 0.0;   // sum of the row memcpy out of mapped staging memory
        bool have_split = false;
    };

    // Aggregates calls over a reporting window. Owned by the RPC-serving side; simGetImages is
    // serialised per RPC server, but two servers (drone/car/cv ports) can call concurrently, so
    // the accumulator is guarded.
    struct Window
    {
        uint64 calls = 0;
        uint64 images = 0;

        double a_ms = 0.0, b_ms = 0.0, cd_ms = 0.0, e_ms = 0.0, total_ms = 0.0;
        double a_max = 0.0, b_max = 0.0, cd_max = 0.0, e_max = 0.0, total_max = 0.0;

        // c+d broken down, when the batched path supplied it
        double lock_ms = 0.0, copy_ms = 0.0;
        uint64 split_calls = 0;

        Clock::time_point window_start{};
        bool started = false;

        void note(const Call& c, Clock::time_point t_return)
        {
            if (!c.valid) return;

            const double a = ToMs(c.t_game_task - c.t_rpc_enter);
            const double b = ToMs(c.t_end_draw - c.t_game_task);
            const double cd = ToMs(c.t_readback_done - c.t_end_draw);
            const double e = ToMs(t_return - c.t_readback_done);
            const double total = ToMs(t_return - c.t_rpc_enter);

            if (!started) { window_start = c.t_rpc_enter; started = true; }

            ++calls;
            images += c.images;
            a_ms += a;   a_max = FMath::Max(a_max, a);
            b_ms += b;   b_max = FMath::Max(b_max, b);
            cd_ms += cd; cd_max = FMath::Max(cd_max, cd);
            e_ms += e;   e_max = FMath::Max(e_max, e);
            total_ms += total; total_max = FMath::Max(total_max, total);

            if (c.have_split) {
                lock_ms += c.lock_ms;
                copy_ms += c.copy_ms;
                ++split_calls;
            }
        }

        bool shouldReport(Clock::time_point now, double period_s) const
        {
            return started && calls > 0 && ToMs(now - window_start) >= period_s * 1000.0;
        }

        void reset() { *this = Window(); }
    };

    // ---- Response assembly, measured SEPARATELY from the getScreenshot segments -------------
    //
    // ⚠ Why its own window rather than a field on Window: the segment report fires at the END of
    // getScreenshot, and assembly happens AFTER getScreenshot returns. Folding it into the same
    // window would report each call's assembly against the NEXT call's segments — an off-by-one
    // that averages away and is invisible in the output. A separate accumulator has no such
    // ordering relationship to defend.
    //
    // This is the term Phase A1 could not attribute: at 1080p, 223 ms of a 303 ms call sits
    // OUTSIDE getScreenshot, covering our copies + rpclib msgpack + the socket. This measures the
    // first of those three, which is the only one we own.
    struct AssemblyWindow
    {
        uint64 calls = 0, images = 0, bytes = 0;
        double ms = 0.0, max_ms = 0.0;
        Clock::time_point window_start{};
        bool started = false;

        void note(double elapsed_ms, size_t n_images, size_t n_bytes, Clock::time_point now)
        {
            if (!started) { window_start = now; started = true; }
            ++calls;
            images += n_images;
            bytes += n_bytes;
            ms += elapsed_ms;
            max_ms = FMath::Max(max_ms, elapsed_ms);
        }

        bool shouldReport(Clock::time_point now, double period_s) const
        {
            return started && calls > 0 && ToMs(now - window_start) >= period_s * 1000.0;
        }

        void reset() { *this = AssemblyWindow(); }
    };
}
