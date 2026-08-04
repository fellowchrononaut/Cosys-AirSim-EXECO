#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"

#include "AirBlueprintLib.h"
#include "Async/Async.h"
#include "HAL/IConsoleManager.h"
#include "common/AirSimSettings.hpp"

/** Diagnostic: log the batch capture instant against each result's own timestamp, plus the spread.
 *  Every image in one simGetImages batch is rendered from a single sim instant (all captures are
 *  CaptureSceneDeferred'd in one game-thread pass, and the game thread blocks until readback), but
 *  ExecuteTask stamps each result as its OWN readback completes, sequentially. Synchronous images
 *  can therefore carry timestamps spread by the readback duration - which reaches ROS unchanged via
 *  header.stamp. Turn this on to measure that spread. */
static TAutoConsoleVariable<int32> CVarLogImageTimestamps(
    TEXT("airsim.LogImageTimestamps"),
    0,
    TEXT("Log per-batch vs per-result image timestamps and their spread.\n")
    TEXT(" 0: Off (default)\n")
    TEXT(" 1: One line per batch with each result's offset from the batch instant"),
    ECVF_Default);

/** Debug override for the ImageTimestampAtCapture setting. The setting is authoritative because it
 *  changes recorded data and therefore belongs with the scenario config that gets archived
 *  alongside a dataset; a console variable would be invisible provenance. This exists only to A/B
 *  at runtime without editing settings.json.
 *   -1 = follow settings (default), 0 = force readback stamps, 1 = force capture instant. */
static TAutoConsoleVariable<int32> CVarBatchImageTimestamp(
    TEXT("airsim.BatchImageTimestamp"),
    -1,
    TEXT("Debug override for the ImageTimestampAtCapture setting in settings.json.\n")
    TEXT(" -1: Follow settings.json (default)\n")
    TEXT("  0: Force per-result stamps taken when that image's readback completes (legacy)\n")
    TEXT("  1: Force the shared capture instant, sampled with the camera poses in OnEndDraw"),
    ECVF_Default);

// I-G Step 0a. Non-static: read by ImageTiming.h's ReportPeriodSeconds().
TAutoConsoleVariable<int32> CVarLogImageTiming(
    TEXT("airsim.LogImageTiming"),
    0,
    TEXT("I-G Step 0: break simGetImages wall time into segments, every N seconds (0 = off).\n")
    TEXT("Reports a=wait for game thread, b=wait for OnEndDraw, c+d=render+readback, e=compress.\n")
    TEXT("Flat in image count => latency-bound => build D9 batching.\n")
    TEXT("Linear in image count => readback-bound => D9 will disappoint. Diagnostic only."),
    ECVF_Default);

namespace
{
    // Accumulates across every RenderRequest instance; one is constructed per capture call, so the
    // window cannot live on the object. Guarded because the MultiAgent build runs three RPC servers
    // (41451/41452/41453) whose handlers can capture concurrently.
    FCriticalSection g_image_timing_mutex;
    AirSimImageTiming::Window g_image_timing_window;
}

/** Resolve the setting against the debug override. */
static bool ShouldStampAtCaptureInstant()
{
    const int32 Override = CVarBatchImageTimestamp.GetValueOnRenderThread();
    if (Override >= 0) {
        return Override != 0;
    }
    return msr::airlib::AirSimSettings::singleton().image_timestamp_at_capture;
}

RenderRequest::RenderRequest(UGameViewportClient* game_viewport, std::function<void()>&& query_camera_pose_cb)
    : params_(nullptr), results_(nullptr), req_size_(0), wait_signal_(new msr::airlib::WorkerThreadSignal), game_viewport_(game_viewport), query_camera_pose_cb_(std::move(query_camera_pose_cb))
{
}

RenderRequest::~RenderRequest()
{
}

// read pixels from render target using render thread, then compress the result into PNG
// argument on the thread that calls this method.
void RenderRequest::getScreenshot(std::shared_ptr<RenderParams> params[], std::vector<std::shared_ptr<RenderResult>>& results, unsigned int req_size, bool use_safe_method)
{
    //TODO: is below really needed?
    for (unsigned int i = 0; i < req_size; ++i) {
        results.push_back(std::make_shared<RenderResult>());

        if (!params[i]->pixels_as_float)
            results[i]->bmp.Reset();
        else
            results[i]->bmp_float.Reset();
        results[i]->time_stamp = 0;
    }

    //make sure we are not on the rendering thread
    CheckNotBlockedOnRenderThread();

    // I-G Step 0a: segment (a) starts here, on the RPC thread.
    const bool timing_on = AirSimImageTiming::ReportPeriodSeconds() > 0;
    if (timing_on) {
        timing_ = AirSimImageTiming::Call();
        timing_.t_rpc_enter = AirSimImageTiming::Clock::now();
        timing_.images = req_size;
    }

    if (use_safe_method) {
        for (unsigned int i = 0; i < req_size; ++i) {
            if (params[i]->render_target != nullptr && params[i]->render_component != nullptr) {
                //TODO: below doesn't work right now because it must be running in game thread
                FIntPoint img_size;
                if (!params[i]->pixels_as_float) {
                    //below is documented method but more expensive because it forces flush
                    FTextureRenderTargetResource* rt_resource = params[i]->render_target->GameThread_GetRenderTargetResource();
                    auto flags = setupRenderResource(rt_resource, params[i].get(), results[i].get(), img_size);
                    if (params[i]->disable_gamma)flags.SetLinearToGamma(false);
                    rt_resource->ReadPixels(results[i]->bmp, flags);
                }
                else {
                    FTextureRenderTargetResource* rt_resource = params[i]->render_target->GetRenderTargetResource();
                    setupRenderResource(rt_resource, params[i].get(), results[i].get(), img_size);
                    rt_resource->ReadFloat16Pixels(results[i]->bmp_float);
                }
            }
        }
    }
    else {
        //wait for render thread to pick up our task
        params_ = params;
        results_ = results.data();
        req_size_ = req_size;

        // Queue up the task of querying camera pose in the game thread and synchronizing render thread with camera pose
        AsyncTask(ENamedThreads::GameThread, [this, timing_on]() {
            check(IsInGameThread());

            // I-G Step 0a: end of segment (a) - how long the game thread took to pick this up.
            if (timing_on)
                timing_.t_game_task = AirSimImageTiming::Clock::now();

            saved_DisableWorldRendering_ = game_viewport_->bDisableWorldRendering;
            game_viewport_->bDisableWorldRendering = 0;
            end_draw_handle_ = game_viewport_->OnEndDraw().AddLambda([this, timing_on] {
                check(IsInGameThread());

                // I-G Step 0a: end of segment (b) - the wait for the next rendered frame. This is
                // the boundary that decides latency-bound vs readback-bound.
                if (timing_on)
                    timing_.t_end_draw = AirSimImageTiming::Clock::now();

                // Capture instant for the whole batch, taken with the poses so time and pose agree.
                // This is the moment every image in the batch actually corresponds to; the serial
                // GPU render and readback that follow add latency, not temporal skew.
                batch_time_stamp_ = msr::airlib::ClockFactory::get()->nowNanos();

                // capture CameraPose for this frame
                query_camera_pose_cb_();

                // The completion is called immeidately after GameThread sends the
                // rendering commands to RenderThread. Hence, our ExecuteTask will
                // execute *immediately* after RenderThread renders the scene!
                RenderRequest* This = this;
                ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)
                (
                    [This](FRHICommandListImmediate& RHICmdList) {
                        This->ExecuteTask();
                    });

                game_viewport_->bDisableWorldRendering = saved_DisableWorldRendering_;

                assert(end_draw_handle_.IsValid());
                game_viewport_->OnEndDraw().Remove(end_draw_handle_);
            });

            // while we're still on GameThread, enqueue request for capture the scene!
            for (unsigned int i = 0; i < req_size_; ++i) {
                if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
                    params_[i]->render_component->CaptureSceneDeferred();
                }
            }
        });

        // wait for this task to complete
        while (!wait_signal_->waitFor(5)) {
            // log a message and continue wait
            // lamda function still references a few objects for which there is no refcount.
            // Walking away will cause memory corruption, which is much more difficult to debug.
            UE_LOG(LogTemp, Warning, TEXT("Failed: timeout waiting for screenshot"));
        }
    }

    for (unsigned int i = 0; i < req_size; ++i) {
        if (params[i]->render_target != nullptr && params[i]->render_component != nullptr) {
            if (!params[i]->pixels_as_float) {
                if (results[i]->width != 0 && results[i]->height != 0) {
                    results[i]->image_data_uint8.SetNumUninitialized(results[i]->width * results[i]->height * 3, false);
                    if (params[i]->compress)
                        UAirBlueprintLib::CompressImageArray(results[i]->width, results[i]->height, results[i]->bmp, results[i]->image_data_uint8);
                    else {
                        uint8* ptr = results[i]->image_data_uint8.GetData();
                        for (const auto& item : results[i]->bmp) {
                            *ptr++ = item.R;
                            *ptr++ = item.G;
                            *ptr++ = item.B;
                        }
                    }
                }
            }
            else {
                results[i]->image_data_float.SetNumUninitialized(results[i]->width * results[i]->height);
                float* ptr = results[i]->image_data_float.GetData();
                for (const auto& item : results[i]->bmp_float) {
                    *ptr++ = item.R.GetFloat();
                }
            }
        }
    }

    // I-G Step 0a: end of segment (e), and the window report. The safe-method path never reaches
    // OnEndDraw, so only the deferred path is measured - which is the one in use.
    if (timing_on && !use_safe_method &&
        timing_.t_game_task != AirSimImageTiming::Clock::time_point{} &&
        timing_.t_end_draw != AirSimImageTiming::Clock::time_point{} &&
        timing_.t_readback_done != AirSimImageTiming::Clock::time_point{}) {

        timing_.valid = true;
        const auto t_return = AirSimImageTiming::Clock::now();
        const int32 period = AirSimImageTiming::ReportPeriodSeconds();

        FScopeLock lock(&g_image_timing_mutex);
        g_image_timing_window.note(timing_, t_return);

        if (g_image_timing_window.shouldReport(t_return, period)) {
            const AirSimImageTiming::Window& w = g_image_timing_window;
            const double n = static_cast<double>(w.calls);
            UE_LOG(LogTemp, Log,
                   TEXT("[AirSim][imgtiming] %llu calls, %.2f img/call | total %.2f ms avg (max %.2f) | ")
                   TEXT("a wait-game %.2f (max %.2f) | b wait-draw %.2f (max %.2f) | ")
                   TEXT("c+d render+readback %.2f (max %.2f) | e compress %.2f (max %.2f) | ")
                   TEXT("latency a+b = %.0f%% of total"),
                   w.calls, w.images / n,
                   w.total_ms / n, w.total_max,
                   w.a_ms / n, w.a_max,
                   w.b_ms / n, w.b_max,
                   w.cd_ms / n, w.cd_max,
                   w.e_ms / n, w.e_max,
                   100.0 * (w.a_ms + w.b_ms) / FMath::Max(w.total_ms, KINDA_SMALL_NUMBER));
            g_image_timing_window.reset();
        }
    }
}

FReadSurfaceDataFlags RenderRequest::setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size)
{
    size = rt_resource->GetSizeXY();
    result->width = size.X;
    result->height = size.Y;
    FReadSurfaceDataFlags flags(RCM_UNorm, CubeFace_MAX);
    flags.SetLinearToGamma(false);

    return flags;
}

void RenderRequest::ExecuteTask()
{
    if (params_ != nullptr && req_size_ > 0) {
        // Readback-completion time per result, kept for the airsim.LogImageTimestamps diagnostic
        // even when airsim.BatchImageTimestamp overrides what actually reaches the response.
        TArray<msr::airlib::TTimePoint> readback_stamps;
        readback_stamps.SetNumZeroed(req_size_);

        for (unsigned int i = 0; i < req_size_; ++i) {
            if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
                FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
                auto rt_resource = params_[i]->render_target->GetRenderTargetResource();
                if (rt_resource != nullptr) {
                    const FTexture2DRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
                    FIntPoint size;
                    auto flags = setupRenderResource(rt_resource, params_[i].get(), results_[i].get(), size);

                    //should we be using ENQUEUE_UNIQUE_RENDER_COMMAND_ONEPARAMETER which was in original commit by @saihv
                    //https://github.com/Microsoft/AirSim/pull/162/commits/63e80c43812300a8570b04ed42714a3f6949e63f#diff-56b790f9394f7ca1949ddbb320d8456fR64
                    if (!params_[i]->pixels_as_float) {
                        //below is undocumented method that avoids flushing, but it seems to segfault every 2000 or so calls
                        RHICmdList.ReadSurfaceData(
                            rhi_texture,
                            FIntRect(0, 0, size.X, size.Y),
                            results_[i]->bmp,
                            flags);
                    }
                    else {
                        RHICmdList.ReadSurfaceFloatData(
                            rhi_texture,
                            FIntRect(0, 0, size.X, size.Y),
                            results_[i]->bmp_float,
                            CubeFace_PosX,
                            0,
                            0);
                    }
                }
            }
            // Readback-completion time for this image. Sampled per result, so under the legacy
            // convention a batch of images that all depict batch_time_stamp_ ends up with stamps
            // spread by the readback duration - and, worse, lagged 48-69 ms behind the instant
            // they depict. See ImageTimestampAtCapture in settings.json.
            const msr::airlib::TTimePoint readback_stamp = msr::airlib::ClockFactory::get()->nowNanos();
            results_[i]->time_stamp = (ShouldStampAtCaptureInstant() && batch_time_stamp_ != 0)
                                          ? batch_time_stamp_
                                          : readback_stamp;
            readback_stamps[(int32)i] = readback_stamp;
        }

        if (CVarLogImageTimestamps.GetValueOnRenderThread() != 0) {
            // Spread across the batch = how far apart synchronous images look downstream.
            // TTimePoint is uint64_t, so cast before subtracting to keep deltas signed.
            const int64 first = (int64)readback_stamps[0];
            const int64 last = (int64)readback_stamps[(int32)req_size_ - 1];
            FString offsets;
            for (unsigned int i = 0; i < req_size_; ++i) {
                offsets += FString::Printf(TEXT(" [%u]+%.3fms"), i,
                                           (double)((int64)readback_stamps[(int32)i] - (int64)batch_time_stamp_) * 1e-6);
            }
            UE_LOG(LogTemp, Log,
                   TEXT("[AirSim] image batch of %u: capture instant %llu, readback spread %.3f ms (first->last), offsets from capture:%s"),
                   req_size_, (unsigned long long)batch_time_stamp_,
                   (double)(last - first) * 1e-6, *offsets);
        }

        // I-G Step 0a: end of segment (c+d) - scene render plus the serial blocking readback.
        // Sampled before the signal so it excludes the RPC thread's wake-up.
        if (AirSimImageTiming::ReportPeriodSeconds() > 0)
            timing_.t_readback_done = AirSimImageTiming::Clock::now();

        req_size_ = 0;
        params_ = nullptr;
        results_ = nullptr;

        wait_signal_->signal();
    }
}
