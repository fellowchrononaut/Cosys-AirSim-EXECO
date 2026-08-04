#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"

#include "AirBlueprintLib.h"
#include "Async/Async.h"
#include "HAL/IConsoleManager.h"
#include "RHIGPUReadback.h"
#include "Async/ParallelFor.h"
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

/** I-G design #4 A/B switch. Both readback paths are compiled in so one build answers the
 *  question; flipping this needs no rebuild, which matters because a rebuild here is expensive.
 *   0 = legacy per-image blocking RHICmdList.ReadSurfaceData (current shipped behaviour)
 *   1 = batched FRHIGPUTextureReadback: all EnqueueCopy submitted, then Lock each
 *  Default 0 so behaviour is unchanged until the measurement says otherwise. */
static TAutoConsoleVariable<int32> CVarGpuReadback(
    TEXT("airsim.GpuReadback"),
    0,
    TEXT("I-G: image readback path.\n")
    TEXT(" 0: legacy ReadSurfaceData per image, one GPU sync point each (default)\n")
    TEXT(" 1: batched FRHIGPUTextureReadback - submit all copies, then drain"),
    ECVF_Default);

/** I-G: parallelise the per-pixel decode on the RPC thread.
 *
 *  The BGRA->RGB shuffle is ~2M byte-at-a-time iterations for one 1080p image, run serially for
 *  every image of every vehicle on the single RPC thread. Unlike the GPU readback - where batching
 *  bought nothing because the cost is an unavoidable copy out of staging memory - this is plain
 *  CPU work on ordinary cached memory, so spreading it across cores should scale.
 *
 *  Measured 2026-08-04. On its own segment it is a clean 4.2x (decode 7.1 -> 1.7 ms). Effect on
 *  the whole call depends on how much of it decode was:
 *    single vehicle, 1x 1080p   37.44 -> 34.60 ms   1.08x   <- the Agilex target shape
 *    fleet, 6x stereo 1080p    347.01 -> 334.27 ms  1.04x   (gain in all 3 interleaved rounds)
 *    fleet, 6x stereo VGA      129.68 -> 129.78 ms  1.00x   (VGA decode is already small)
 *    3 concurrent clients                            1.01x   (cores already saturated)
 *
 *  Under concurrent load the gain mostly disappears - but it does no harm either: sim/wall stayed
 *  1.000 throughout and there were zero errors, so ParallelFor is not stealing from the render
 *  thread. Output is bit-identical (same byte count, mean pixel 192.031 on both paths).
 *
 *  Default ON: the target workload is a SINGLE vehicle with a few high-resolution cameras, which
 *  is exactly the case that gains most, and it will matter more once fisheye cube capture makes
 *  decode six faces per camera.
 *
 *   0 = serial   1 = ParallelFor in chunks (default) */
static TAutoConsoleVariable<int32> CVarParallelImageDecode(
    TEXT("airsim.ParallelImageDecode"),
    1,
    TEXT("I-G: decode/convert captured pixels using ParallelFor.\n")
    TEXT(" 0: serial on the RPC thread\n")
    TEXT(" 1: chunked ParallelFor (default)"),
    ECVF_Default);

namespace
{
    // Below this, task overhead costs more than three byte stores per pixel would.
    constexpr int32 kDecodeMinPixelsPerChunk = 32 * 1024;
    constexpr int32 kDecodeMaxChunks = 16;

    bool ShouldParallelDecode(int32 pixel_count)
    {
        return CVarParallelImageDecode.GetValueOnAnyThread() != 0 &&
               pixel_count >= 2 * kDecodeMinPixelsPerChunk;
    }

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
                        // BGRA -> RGB. Byte-at-a-time over every pixel: ~2M iterations for one
                        // 1080p image, on the single RPC thread, serial across the whole fleet.
                        // This is segment (e), which grew from 0.2-0.7 ms to 2.3-5.1 ms once 1080p
                        // entered the mix - the visible edge of the CPU-side per-pixel cost.
                        const int32 pixel_count = results[i]->bmp.Num();
                        const FColor* src = results[i]->bmp.GetData();
                        uint8* dst = results[i]->image_data_uint8.GetData();

                        if (ShouldParallelDecode(pixel_count)) {
                            // Chunked rather than one task per pixel: task overhead would dwarf
                            // three byte stores. One chunk per worker, sized off the pixel count.
                            const int32 chunks = FMath::Min(kDecodeMaxChunks,
                                                            FMath::Max(2, pixel_count / kDecodeMinPixelsPerChunk));
                            const int32 per_chunk = FMath::DivideAndRoundUp(pixel_count, chunks);
                            ParallelFor(chunks, [src, dst, pixel_count, per_chunk](int32 chunk) {
                                const int32 begin = chunk * per_chunk;
                                const int32 end = FMath::Min(begin + per_chunk, pixel_count);
                                uint8* out = dst + static_cast<int64>(begin) * 3;
                                for (int32 p = begin; p < end; ++p) {
                                    *out++ = src[p].R;
                                    *out++ = src[p].G;
                                    *out++ = src[p].B;
                                }
                            });
                        }
                        else {
                            uint8* ptr = dst;
                            for (const auto& item : results[i]->bmp) {
                                *ptr++ = item.R;
                                *ptr++ = item.G;
                                *ptr++ = item.B;
                            }
                        }
                    }
                }
            }
            else {
                results[i]->image_data_float.SetNumUninitialized(results[i]->width * results[i]->height);
                const int32 pixel_count = results[i]->bmp_float.Num();
                const FFloat16Color* src = results[i]->bmp_float.GetData();
                float* dst = results[i]->image_data_float.GetData();

                if (ShouldParallelDecode(pixel_count)) {
                    const int32 chunks = FMath::Min(kDecodeMaxChunks,
                                                    FMath::Max(2, pixel_count / kDecodeMinPixelsPerChunk));
                    const int32 per_chunk = FMath::DivideAndRoundUp(pixel_count, chunks);
                    ParallelFor(chunks, [src, dst, pixel_count, per_chunk](int32 chunk) {
                        const int32 begin = chunk * per_chunk;
                        const int32 end = FMath::Min(begin + per_chunk, pixel_count);
                        for (int32 p = begin; p < end; ++p)
                            dst[p] = src[p].R.GetFloat();
                    });
                }
                else {
                    float* ptr = dst;
                    for (const auto& item : results[i]->bmp_float) {
                        *ptr++ = item.R.GetFloat();
                    }
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

            // c+d split, only available under airsim.GpuReadback 1. This is the number that
            // decides whether parallelising the CPU copy is worth anything.
            if (w.split_calls > 0) {
                const double sn = static_cast<double>(w.split_calls);
                UE_LOG(LogTemp, Log,
                       TEXT("[AirSim][imgtiming]   c+d split over %llu calls: Lock (GPU wait) %.2f ms ")
                       TEXT("| staging->CPU copy %.2f ms | copy = %.0f%% of c+d"),
                       w.split_calls, w.lock_ms / sn, w.copy_ms / sn,
                       100.0 * w.copy_ms / FMath::Max(w.lock_ms + w.copy_ms, KINDA_SMALL_NUMBER));
            }
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

// One log line per unique pixel format, not per frame: a batch that hits an unsupported format
// would otherwise spam the log every capture.
void RenderRequest::warnUnsupportedFormatOnce(unsigned int index, EPixelFormat format)
{
    static FCriticalSection mutex;
    static TSet<int32> warned;

    FScopeLock lock(&mutex);
    if (warned.Contains((int32)format))
        return;
    warned.Add((int32)format);

    UE_LOG(LogTemp, Warning,
           TEXT("[AirSim] airsim.GpuReadback: unsupported pixel format %d on request %u - ")
           TEXT("falling back to an empty result for it. Add a case in executeBatchedGpuReadback, ")
           TEXT("or set airsim.GpuReadback 0 to use the legacy path."),
           (int32)format, index);
}

// Legacy path: one blocking RHICmdList.ReadSurfaceData per image. Each call carries its own GPU
// sync point, which is why measured throughput COLLAPSES as images grow - 4941 MB/s at 320x240
// down to 320 MB/s at 1920x1080, ~30x below PCIe. Kept as the A/B control for I-G.
void RenderRequest::executeLegacyReadback(TArray<msr::airlib::TTimePoint>& readback_stamps)
{
    for (unsigned int i = 0; i < req_size_; ++i) {
        if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
            FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();
            auto rt_resource = params_[i]->render_target->GetRenderTargetResource();
            if (rt_resource != nullptr) {
                const FTexture2DRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
                FIntPoint size;
                auto flags = setupRenderResource(rt_resource, params_[i].get(), results_[i].get(), size);

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
        readback_stamps[(int32)i] = msr::airlib::ClockFactory::get()->nowNanos();
    }
}

// I-G design #4 (ported from the execosim branch, commit 76b843e9, and re-measured here rather
// than taken on trust).
//
// NOT asynchronous - Lock() still blocks the render thread. The win is ordering: every
// EnqueueCopy is submitted BEFORE the first Lock, so the GPU pipelines the N DMAs instead of
// servicing them one sync point at a time. Total should approach the slowest single copy rather
// than the sum.
//
// ❌ MEASURED 2026-08-04: IT DOES NOT HELP. Do not enable this.
//
// Predicted 1080p marginal 26 -> 1-2 ms. Actual, over 3 interleaved rounds of 25 samples per arm:
// legacy 19.88 ms/img vs batched 19.93 ms/img = 1.00x, a 0.05 ms difference against a 2 ms
// within-arm spread. Removing N-1 GPU sync points changed NOTHING, so the cost was never sync
// serialisation.
//
// It is the CPU copy out of GPU-visible staging memory: 8.29 MB in ~19 ms is ~436 MB/s, the
// signature of reading uncached/write-combined memory. BOTH paths do that copy per image, which is
// why batching cannot win - and why async could not either, since it would relocate the copy, not
// remove it. The lever is parallelising the per-image CPU work, not restructuring the readback.
//
// ⚠ It also CORRUPTS float image types: DepthPlanar sample mean 0.339 (legacy) vs 131.642 (this
// path), with no format warning - so the target really was PF_FloatRGBA. ReadSurfaceFloatData is
// therefore NOT equivalent to a raw staging copy; it applies a conversion this memcpy does not
// reproduce. Scene/BGRA content did match exactly.
//
// Kept, defaulted off, as the control for any future attempt. See I-G in sim_issues plan.
void RenderRequest::executeBatchedGpuReadback(TArray<msr::airlib::TTimePoint>& readback_stamps)
{
    FRHICommandListImmediate& RHICmdList = GetImmediateCommandList_ForRenderCommand();

    readbacks_.Reset();
    readbacks_.SetNum(req_size_);
    enqueued_.Reset();
    enqueued_.SetNumZeroed(req_size_);
    formats_.Reset();
    formats_.SetNumZeroed(req_size_);

    // Phase 1 - submit every copy first. No Lock in this loop; that is the whole point.
    for (unsigned int i = 0; i < req_size_; ++i) {
        if (params_[i]->render_target == nullptr || params_[i]->render_component == nullptr)
            continue;

        auto rt_resource = params_[i]->render_target->GetRenderTargetResource();
        if (rt_resource == nullptr)
            continue;

        const FTexture2DRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
        if (!rhi_texture.IsValid())
            continue;

        FIntPoint size;
        setupRenderResource(rt_resource, params_[i].get(), results_[i].get(), size);

        const EPixelFormat format = rhi_texture->GetFormat();
        if (format != PF_B8G8R8A8 && format != PF_FloatRGBA) {
            // Unknown GPU layout: decoding it would produce garbage. Mark the result invalid so
            // the caller's formatting pass skips it, and warn once rather than every frame.
            warnUnsupportedFormatOnce(i, format);
            results_[i]->width = 0;
            results_[i]->height = 0;
            continue;
        }
        formats_[(int32)i] = format;

        readbacks_[(int32)i] = MakeUnique<FRHIGPUTextureReadback>(
            FName(*FString::Printf(TEXT("AirSimReadback_%u"), i)));
        readbacks_[(int32)i]->EnqueueCopy(RHICmdList, rhi_texture);
        enqueued_[(int32)i] = true;
    }

    // Phase 2 - now drain. The first Lock waits on its fence; later ones usually return
    // immediately because their copies were submitted alongside the first.
    for (unsigned int i = 0; i < req_size_; ++i) {
        if (!enqueued_[(int32)i] || results_[i]->width <= 0 || results_[i]->height <= 0) {
            readback_stamps[(int32)i] = msr::airlib::ClockFactory::get()->nowNanos();
            continue;
        }

        // I-G diagnosis: separate the GPU-completion wait from the CPU copy. Design #4 is dead as
        // an optimisation, but this is the only place the two are separable, and they want
        // opposite fixes - a wait cannot be parallelised away, a copy can.
        const bool split_on = AirSimImageTiming::ReportPeriodSeconds() > 0;
        const auto t_lock0 = AirSimImageTiming::Clock::now();

        int32 row_pitch_in_pixels = 0;
        void* raw = readbacks_[(int32)i]->Lock(row_pitch_in_pixels);

        const auto t_lock1 = AirSimImageTiming::Clock::now();
        if (split_on) {
            timing_.lock_ms += AirSimImageTiming::ToMs(t_lock1 - t_lock0);
            timing_.have_split = true;
        }

        if (raw != nullptr) {
            const int32 w = results_[i]->width;
            const int32 h = results_[i]->height;

            // Dispatch on the ACTUAL GPU format, not the client's pixels_as_float request - those
            // can disagree, and trusting the request is how you get garbage pixels.
            if (formats_[(int32)i] == PF_B8G8R8A8) {
                results_[i]->bmp.SetNumUninitialized(w * h, false);
                const FColor* src = static_cast<const FColor*>(raw);
                FColor* dst = results_[i]->bmp.GetData();
                for (int32 y = 0; y < h; ++y)
                    FMemory::Memcpy(dst + y * w, src + y * row_pitch_in_pixels, w * sizeof(FColor));
            }
            else { //PF_FloatRGBA, guaranteed by phase 1
                results_[i]->bmp_float.SetNumUninitialized(w * h, false);
                const FFloat16Color* src = static_cast<const FFloat16Color*>(raw);
                FFloat16Color* dst = results_[i]->bmp_float.GetData();
                for (int32 y = 0; y < h; ++y)
                    FMemory::Memcpy(dst + y * w, src + y * row_pitch_in_pixels, w * sizeof(FFloat16Color));
            }
        }
        if (split_on)
            timing_.copy_ms += AirSimImageTiming::ToMs(AirSimImageTiming::Clock::now() - t_lock1);

        readbacks_[(int32)i]->Unlock();
        readback_stamps[(int32)i] = msr::airlib::ClockFactory::get()->nowNanos();
    }

    readbacks_.Reset();
}

void RenderRequest::ExecuteTask()
{
    if (params_ != nullptr && req_size_ > 0) {
        // Readback-completion time per result, kept for the airsim.LogImageTimestamps diagnostic
        // even when airsim.BatchImageTimestamp overrides what actually reaches the response.
        TArray<msr::airlib::TTimePoint> readback_stamps;
        readback_stamps.SetNumZeroed(req_size_);

        if (CVarGpuReadback.GetValueOnRenderThread() != 0)
            executeBatchedGpuReadback(readback_stamps);
        else
            executeLegacyReadback(readback_stamps);

        for (unsigned int i = 0; i < req_size_; ++i) {
            // Readback-completion time for this image. Sampled per result, so under the legacy
            // convention a batch of images that all depict batch_time_stamp_ ends up with stamps
            // spread by the readback duration - and, worse, lagged 48-69 ms behind the instant
            // they depict. See ImageTimestampAtCapture in settings.json.
            results_[i]->time_stamp = (ShouldStampAtCaptureInstant() && batch_time_stamp_ != 0)
                                          ? batch_time_stamp_
                                          : readback_stamps[(int32)i];
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
