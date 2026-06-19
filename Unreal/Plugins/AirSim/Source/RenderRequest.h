#pragma once

#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"
#include "RHIGPUReadback.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/GameViewportClient.h"
#include <memory>
#include "common/Common.hpp"
#include "common/WorkerThread.hpp"


class RenderRequest : public FRenderCommand
{
public:
    struct RenderParams {
        USceneCaptureComponent2D * const render_component;
        UTextureRenderTarget2D* render_target;
        bool pixels_as_float;
        bool compress;
        bool disable_gamma;
        // For diagnostic logging when the GPU readback format is unsupported.
        FString camera_name;

        RenderParams(USceneCaptureComponent2D * render_component_val, UTextureRenderTarget2D* render_target_val, bool pixels_as_float_val, bool compress_val, bool disable_gamma_val, const FString& camera_name_val = TEXT(""))
            : render_component(render_component_val), render_target(render_target_val), pixels_as_float(pixels_as_float_val), compress(compress_val), disable_gamma(disable_gamma_val), camera_name(camera_name_val)
        {
        }
    };
    struct RenderResult {
        TArray<uint8> image_data_uint8;
        TArray<float> image_data_float;

        TArray<FColor> bmp;
        TArray<FFloat16Color> bmp_float;

        int width;
        int height;

        msr::airlib::TTimePoint time_stamp;
    };

private:
    static FReadSurfaceDataFlags setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size);

    std::shared_ptr<RenderParams>* params_;
    std::shared_ptr<RenderResult>* results_;
    unsigned int req_size_;

    bool saved_DisableWorldRendering_ = false;
    UGameViewportClient * const game_viewport_;
    FDelegateHandle end_draw_handle_;
    TArray<TUniquePtr<FRHIGPUTextureReadback>> readbacks_;
    // Per-camera flag: true iff EnqueueCopy was actually called in ExecuteTask.
    // The Lock/copy loop only operates on enqueued cameras so one skipped camera
    // (invalid texture, unsupported pixel format) doesn't stall the batch.
    TArray<bool> enqueued_;
    // Per-camera actual GPU texture pixel format, recorded at submit time so the
    // decode dispatches on real GPU layout rather than client-requested type.
    TArray<EPixelFormat> formats_;
    // CPU-thread signal: render-thread ExecuteTask signals when all readbacks have
    // been locked and copied; RPC thread blocks on waitFor() with watchdog logging.
    std::shared_ptr<msr::airlib::WorkerThreadSignal> wait_signal_;
    std::function<void()> query_camera_pose_cb_;

public:
    RenderRequest(UGameViewportClient * game_viewport, std::function<void()>&& query_camera_pose_cb);
    ~RenderRequest();

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(RenderRequest, STATGROUP_RenderThreadCommands);
    }

    // read pixels from render target using render thread, then compress the result into PNG
    // argument on the thread that calls this method.
    void getScreenshot(
        std::shared_ptr<RenderParams> params[], std::vector<std::shared_ptr<RenderResult>>& results, unsigned int req_size, bool use_safe_method);

private:
    // Render thread: submit EnqueueCopy for all valid cameras, then Lock/copy/Unlock
    // for each enqueued camera. Render thread blocks during Lock until GPU DMA
    // completes — typically 5–20ms. Signals wait_signal_ when done.
    void ExecuteTask(FRHICommandListImmediate& RHICmdList);
};
