#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"

#include "AirBlueprintLib.h"
#include "Async/Async.h"

RenderRequest::RenderRequest(UGameViewportClient* game_viewport, std::function<void()>&& query_camera_pose_cb)
    : params_(nullptr), results_(nullptr), req_size_(0), game_viewport_(game_viewport),
      query_camera_pose_cb_(std::move(query_camera_pose_cb))
{
}

RenderRequest::~RenderRequest()
{
    if (ticker_handle_.IsValid()) {
        FTSTicker::GetCoreTicker().RemoveTicker(ticker_handle_);
    }
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

        // Pre-allocate one readback object per camera
        readbacks_.Empty();
        for (unsigned int i = 0; i < req_size_; ++i) {
            readbacks_.Add(MakeUnique<FRHIGPUTextureReadback>(
                *FString::Printf(TEXT("AirSimReadback_%u"), i)));
        }

        // Promise fulfilled on render thread once collectReadbacks() finishes.
        promise_ = std::make_unique<std::promise<void>>();
        std::future<void> future = promise_->get_future();

        AsyncTask(ENamedThreads::GameThread, [this]() {
            check(IsInGameThread());

            saved_DisableWorldRendering_ = game_viewport_->bDisableWorldRendering;
            game_viewport_->bDisableWorldRendering = 0;
            end_draw_handle_ = game_viewport_->OnEndDraw().AddLambda([this] {
                check(IsInGameThread());

                // Capture camera pose for this frame while still on game thread
                query_camera_pose_cb_();

                // Submit GPU→CPU copies — render thread returns immediately after EnqueueCopy.
                RenderRequest* This = this;
                ENQUEUE_RENDER_COMMAND(SubmitReadbackCopies)
                (
                    [This](FRHICommandListImmediate& RHICmdList) {
                        This->submitCopies(RHICmdList);
                    });

                // Poll readbacks each game tick — render thread is free during this window.
                ticker_handle_ = FTSTicker::GetCoreTicker().AddTicker(
                    FTickerDelegate::CreateLambda([this](float dt) {
                        return this->checkReadbacks(dt);
                    }), 0.0f);

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

        // Block RPC thread until render thread finishes collectReadbacks().
        // Client semantics unchanged; render thread is freed during GPU readback wait.
        future.wait();

        // Data already copied into results_[i]->bmp by render thread in collectReadbacks()
        req_size_ = 0;
        params_ = nullptr;
        results_ = nullptr;
        readbacks_.Empty();
        promise_.reset();
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

void RenderRequest::submitCopies(FRHICommandListImmediate& RHICmdList)
{
    if (params_ == nullptr || req_size_ == 0) return;

    // Submit GPU DMA for all cameras and stamp render-frame time.
    // Render thread returns after this — actual DMA runs asynchronously on GPU.
    const auto ts = msr::airlib::ClockFactory::get()->nowNanos();
    for (unsigned int i = 0; i < req_size_; ++i) {
        if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
            auto rt_resource = params_[i]->render_target->GetRenderTargetResource();
            if (rt_resource != nullptr) {
                FIntPoint size;
                setupRenderResource(rt_resource, params_[i].get(), results_[i].get(), size);
                const FTexture2DRHIRef& rhi_texture = rt_resource->GetRenderTargetTexture();
                if (rhi_texture.IsValid()) {
                    readbacks_[i]->EnqueueCopy(RHICmdList, rhi_texture);
                }
            }
        }
        results_[i]->time_stamp = ts;
    }
}

bool RenderRequest::checkReadbacks(float /*dt*/)
{
    check(IsInGameThread());

    if (params_ == nullptr || req_size_ == 0) {
        ticker_handle_.Reset();
        return false;
    }

    for (unsigned int i = 0; i < req_size_; ++i) {
        if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
            if (!readbacks_[i].IsValid() || !readbacks_[i]->IsReady()) {
                return true; // not ready — poll again next tick
            }
        }
    }

    // All GPU DMAs complete — schedule Lock/copy/Unlock on render thread.
    RenderRequest* This = this;
    ENQUEUE_RENDER_COMMAND(CollectReadbacks)
    (
        [This](FRHICommandListImmediate& RHICmdList) {
            This->collectReadbacks(RHICmdList);
            This->promise_->set_value();
        });

    ticker_handle_.Reset();
    return false;
}

void RenderRequest::collectReadbacks(FRHICommandListImmediate& /*RHICmdList*/)
{
    if (params_ == nullptr || req_size_ == 0) return;

    for (unsigned int i = 0; i < req_size_; ++i) {
        if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr
            && results_[i]->width > 0 && results_[i]->height > 0) {
            int32 rowPitchInPixels;
            void* rawData = readbacks_[i]->Lock(rowPitchInPixels);
            if (rawData) {
                const int32 w = results_[i]->width;
                const int32 h = results_[i]->height;
                if (!params_[i]->pixels_as_float) {
                    results_[i]->bmp.SetNumUninitialized(w * h, false);
                    const FColor* src = static_cast<const FColor*>(rawData);
                    FColor* dst = results_[i]->bmp.GetData();
                    for (int32 y = 0; y < h; ++y) {
                        FMemory::Memcpy(dst + y * w, src + y * rowPitchInPixels, w * sizeof(FColor));
                    }
                }
                else {
                    results_[i]->bmp_float.SetNumUninitialized(w * h, false);
                    const FFloat16Color* src = static_cast<const FFloat16Color*>(rawData);
                    FFloat16Color* dst = results_[i]->bmp_float.GetData();
                    for (int32 y = 0; y < h; ++y) {
                        FMemory::Memcpy(dst + y * w, src + y * rowPitchInPixels, w * sizeof(FFloat16Color));
                    }
                }
            }
            readbacks_[i]->Unlock();
        }
    }
}
