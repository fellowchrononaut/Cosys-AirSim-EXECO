#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"
#include "PixelFormat.h"
#include "HAL/CriticalSection.h"

#include "AirBlueprintLib.h"
#include "Async/Async.h"

namespace
{
    // Single place to extend GPU readback support to more pixel formats.
    // Today AirSim only configures PF_B8G8R8A8 (8-bit BGRA color) and PF_FloatRGBA
    // (16-bit half-float RGBA) for its render targets — see PIPCamera.cpp.
    // When adding new formats (e.g. PF_R32_FLOAT for single-channel depth, or
    // PF_G16R16F for 2-channel optical flow), add a case here AND the matching
    // decode branch in RenderRequest::ExecuteTask().
    bool isSupportedReadbackFormat(EPixelFormat format, bool pixels_as_float)
    {
        if (!pixels_as_float && format == PF_B8G8R8A8) return true;   // → FColor decode
        if ( pixels_as_float && format == PF_FloatRGBA) return true;   // → FFloat16Color decode
        return false;
    }

    // Log each unique (camera_name, format, pixels_as_float) mismatch once per
    // process so a fleet of misconfigured cameras doesn't spam the log on every
    // capture tick.
    void warnUnsupportedFormatOnce(const FString& camera_name, unsigned int req_index, EPixelFormat format, bool pixels_as_float)
    {
        static FCriticalSection mutex;
        static TSet<FString> warned;

        const FString key = FString::Printf(TEXT("%s|%d|%d"),
            *camera_name, (int)format, pixels_as_float ? 1 : 0);

        FScopeLock lock(&mutex);
        if (warned.Contains(key)) return;
        warned.Add(key);

        const TCHAR* fmt_name = GPixelFormats[format].Name ? GPixelFormats[format].Name : TEXT("Unknown");
        UE_LOG(LogTemp, Warning,
            TEXT("RenderRequest: unsupported GPU readback format for camera '%s' (request_idx=%u, format=%s, pixels_as_float=%d). Camera will return empty response. Extend isSupportedReadbackFormat() to add support."),
            *camera_name, req_index, fmt_name, pixels_as_float ? 1 : 0);
    }
}

RenderRequest::RenderRequest(UGameViewportClient* game_viewport, std::function<void()>&& query_camera_pose_cb)
    : params_(nullptr), results_(nullptr), req_size_(0), game_viewport_(game_viewport),
      wait_signal_(std::make_shared<msr::airlib::WorkerThreadSignal>()),
      query_camera_pose_cb_(std::move(query_camera_pose_cb))
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
        params_ = params;
        results_ = results.data();
        req_size_ = req_size;

        // Pre-allocate one readback object per camera; reset enqueue/format trackers.
        readbacks_.Empty();
        enqueued_.Init(false, req_size_);
        formats_.Init(PF_Unknown, req_size_);
        for (unsigned int i = 0; i < req_size_; ++i) {
            readbacks_.Add(MakeUnique<FRHIGPUTextureReadback>(
                *FString::Printf(TEXT("AirSimReadback_%u"), i)));
        }

        AsyncTask(ENamedThreads::GameThread, [this]() {
            check(IsInGameThread());

            saved_DisableWorldRendering_ = game_viewport_->bDisableWorldRendering;
            game_viewport_->bDisableWorldRendering = 0;
            end_draw_handle_ = game_viewport_->OnEndDraw().AddLambda([this] {
                check(IsInGameThread());

                // Capture camera pose for this frame while still on game thread
                query_camera_pose_cb_();

                // One render command does EnqueueCopy + Lock + copy + Unlock + signal.
                // Render thread stalls during Lock (5–20ms) but RPC capture latency is
                // ~1 game tick + GPU readback rather than the 2–3 game ticks the prior
                // ticker-poll design cost. wait_signal_->signal() is guaranteed via the
                // try/catch wrapper so an exception in ExecuteTask won't hang the RPC.
                RenderRequest* This = this;
                ENQUEUE_RENDER_COMMAND(GpuReadback)
                (
                    [This](FRHICommandListImmediate& RHICmdList) {
                        try {
                            This->ExecuteTask(RHICmdList);
                        }
                        catch (const std::exception& e) {
                            UE_LOG(LogTemp, Error,
                                TEXT("RenderRequest::ExecuteTask threw std::exception: %s"),
                                UTF8_TO_TCHAR(e.what()));
                        }
                        catch (...) {
                            UE_LOG(LogTemp, Error,
                                TEXT("RenderRequest::ExecuteTask threw unknown exception"));
                        }
                        This->wait_signal_->signal();
                    });

                game_viewport_->bDisableWorldRendering = saved_DisableWorldRendering_;

                assert(end_draw_handle_.IsValid());
                game_viewport_->OnEndDraw().Remove(end_draw_handle_);
            });

            for (unsigned int i = 0; i < req_size_; ++i) {
                if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
                    params_[i]->render_component->CaptureSceneDeferred();
                }
            }
        });

        // Block RPC thread until render thread completes EnqueueCopy + Lock + copy.
        // Watchdog: if signal doesn't fire within 5s, log loudly each iteration so a
        // hung capture is visible. True cancellation needs heap-owned per-call state
        // (see TODO atop ExecuteTask) so we still wait forever — but never silently.
        while (!wait_signal_->waitFor(5)) {
            UE_LOG(LogTemp, Warning,
                TEXT("RenderRequest: capture taking >5s for %u camera(s); still waiting"),
                req_size_);
        }

        // Data already copied into results_[i]->bmp by render thread in ExecuteTask()
        req_size_ = 0;
        params_ = nullptr;
        results_ = nullptr;
        readbacks_.Empty();
        enqueued_.Empty();
        formats_.Empty();
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

// TODO(scalability): true cancellation of an in-flight capture (e.g. on timeout,
// PIE shutdown, or world destruction) requires the per-call state — params_,
// results_, readbacks_, enqueued_, formats_ — to be heap-owned via shared_ptr
// captured by the render lambda. Currently they are RenderRequest members, so
// abandoning the wait would risk use-after-free if the caller destroys the
// RenderRequest before the render command runs. See reviewer notes (High #1).
void RenderRequest::ExecuteTask(FRHICommandListImmediate& RHICmdList)
{
    if (params_ == nullptr || req_size_ == 0) return;

    // Stamp render-frame time once for the whole batch so all cameras share the
    // same timestamp — load-bearing for multi-vehicle / stereo synchronization.
    const auto ts = msr::airlib::ClockFactory::get()->nowNanos();

    // Phase 1: submit GPU DMA for every valid camera, recording which ones
    // actually got enqueued and what format the GPU texture has.
    for (unsigned int i = 0; i < req_size_; ++i) {
        results_[i]->time_stamp = ts;

        if (params_[i]->render_target == nullptr || params_[i]->render_component == nullptr) {
            continue;
        }
        auto rt_resource = params_[i]->render_target->GetRenderTargetResource();
        if (rt_resource == nullptr) {
            continue;
        }
        FIntPoint size;
        setupRenderResource(rt_resource, params_[i].get(), results_[i].get(), size);
        const FTexture2DRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
        if (!rhi_texture.IsValid()) {
            continue;
        }

        const EPixelFormat format = rhi_texture->GetFormat();
        formats_[i] = format;

        if (!isSupportedReadbackFormat(format, params_[i]->pixels_as_float)) {
            warnUnsupportedFormatOnce(params_[i]->camera_name, i, format, params_[i]->pixels_as_float);
            // Mark response invalid so the caller's formatting pass skips it
            // instead of producing garbage from the empty bmp/bmp_float.
            results_[i]->width = 0;
            results_[i]->height = 0;
            continue;
        }

        readbacks_[i]->EnqueueCopy(RHICmdList, rhi_texture);
        enqueued_[i] = true;
    }

    // Phase 2: Lock each enqueued readback. Lock() blocks the render thread until
    // that readback's GPU DMA completes — but because all EnqueueCopy calls above
    // are submitted before any Lock, the GPU pipelines the DMAs in parallel and
    // later Lock() calls usually return immediately.
    for (unsigned int i = 0; i < req_size_; ++i) {
        if (!enqueued_[i] || results_[i]->width <= 0 || results_[i]->height <= 0) {
            continue;
        }

        int32 rowPitchInPixels;
        void* rawData = readbacks_[i]->Lock(rowPitchInPixels);
        if (rawData) {
            const int32 w = results_[i]->width;
            const int32 h = results_[i]->height;
            // Dispatch on actual GPU format recorded above, not client-requested
            // pixels_as_float. Add cases here when extending isSupportedReadbackFormat().
            switch (formats_[i]) {
            case PF_B8G8R8A8:
            {
                results_[i]->bmp.SetNumUninitialized(w * h, false);
                const FColor* src = static_cast<const FColor*>(rawData);
                FColor* dst = results_[i]->bmp.GetData();
                for (int32 y = 0; y < h; ++y) {
                    FMemory::Memcpy(dst + y * w, src + y * rowPitchInPixels, w * sizeof(FColor));
                }
                break;
            }
            case PF_FloatRGBA:
            {
                results_[i]->bmp_float.SetNumUninitialized(w * h, false);
                const FFloat16Color* src = static_cast<const FFloat16Color*>(rawData);
                FFloat16Color* dst = results_[i]->bmp_float.GetData();
                for (int32 y = 0; y < h; ++y) {
                    FMemory::Memcpy(dst + y * w, src + y * rowPitchInPixels, w * sizeof(FFloat16Color));
                }
                break;
            }
            default:
                // Unreachable: Phase 1 would have rejected this format. Defensive.
                results_[i]->width = 0;
                results_[i]->height = 0;
                break;
            }
        }
        readbacks_[i]->Unlock();
    }
}
