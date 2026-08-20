#pragma once

#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"
#include "common/WorkerThread.hpp"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/GameViewportClient.h"
#include <memory>
#include "common/Common.hpp"
#include "ImageTiming.h"
#include "CubeResample.h"


class RenderRequest : public FRenderCommand
{
public:
    struct RenderParams {
        USceneCaptureComponent2D * const render_component;
        UTextureRenderTarget2D* render_target;
        bool pixels_as_float;
        bool compress;
        bool disable_gamma;

        // Generic (non-pinhole) camera support - Phase 3b step 4, finding F1.
        //
        // The 1:1 assumption above - one capture component, one render target - is the only
        // structural thing a cube camera breaks: it needs N face captures feeding ONE output
        // target. Rather than rewrite the pair into a list, the face list is ADDED alongside it
        // and left EMPTY for every pinhole request, which is what makes the pinhole path
        // bit-identical rather than merely equivalent:
        //
        //   - the constructor is untouched, so every existing construction site builds exactly
        //     the object it built before, with two empty TArrays and a null TSharedPtr;
        //   - getScreenshot tests face_components.Num() and, finding zero, fires the same single
        //     CaptureSceneDeferred() on the same component as before;
        //   - ExecuteTask tests face_targets.Num() and, finding zero, does no resample work at
        //     all before dispatching to the same readback it dispatched to before;
        //   - render_target stays what it always was, so ReadSurfaceData is untouched (F2).
        //
        // face_targets are internal; render_target remains the OUTPUT, and the resample writes
        // into it. Filled by UnrealImageCapture only for a camera with a CameraModel block, and
        // in step 4 only for ImageType::Scene (per-modality filtering is step 5).
        TArray<USceneCaptureComponent2D*> face_components;
        TArray<UTextureRenderTarget2D*> face_targets;
        FAirSimRaymapResourcePtr raymap;

        // Which filter the resample pass runs - an EAirSimCubeResampleMode, carried as an int so
        // that this stays a plain forwarded value. Phase 3b step 5.
        //
        // The MODE and not the ImageType: RenderRequest has no business knowing AirLib's enum,
        // and CubeResample stays the only file that maps one to the other. Zero is Unsupported,
        // which is also what every pinhole request leaves it at - and a pinhole request never
        // reaches the pass anyway, because face_targets is empty.
        int32 resample_mode = 0;

        RenderParams(USceneCaptureComponent2D * render_component_val, UTextureRenderTarget2D* render_target_val, bool pixels_as_float_val, bool compress_val, bool disable_gamma_val)
            : render_component(render_component_val), render_target(render_target_val), pixels_as_float(pixels_as_float_val), compress(compress_val), disable_gamma(disable_gamma_val)
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

    // Phase 3b step 4. Cube faces -> generic camera image, into the existing output target, at
    // the top of ExecuteTask and therefore before any readback (finding F3). No-op unless a
    // request carries faces.
    void executeCubeResample();

    // I-G: the two readback paths, selected at runtime by airsim.GpuReadback. Both fill
    // results_[i]->bmp / bmp_float and record a completion stamp per image.
    void executeLegacyReadback(TArray<msr::airlib::TTimePoint>& readback_stamps);
    void executeBatchedGpuReadback(TArray<msr::airlib::TTimePoint>& readback_stamps);
    void warnUnsupportedFormatOnce(unsigned int index, EPixelFormat format);

    // B3. The batched path split in two so the drain can happen in a LATER frame than the submit.
    // Mode 1 calls them back to back; mode 2 puts a frame boundary between them.
    void submitGpuReadbacks();
    bool drainGpuReadbacks(TArray<msr::airlib::TTimePoint>& readback_stamps, bool force_blocking);

    // Post-readback tail: stamps, diagnostics, and the signal that releases getScreenshot.
    void finishTask(const TArray<msr::airlib::TTimePoint>& readback_stamps);

    // B3. Render-thread-only registry of batches waiting on their GPU fence.
    static void drainPendingDeferredReadbacks();
    static void ensureDeferredDrainHook();

    // Batched-readback scratch. Under mode 2 these OUTLIVE the submitting call and stay valid
    // until the deferred drain consumes them, so nothing here may be reset by the submit path.
    TArray<TUniquePtr<class FRHIGPUTextureReadback>> readbacks_;
    TArray<bool> enqueued_;
    TArray<EPixelFormat> formats_;

    // B3. Deferred-drain state. Only touched on the render thread.
    TArray<msr::airlib::TTimePoint> deferred_stamps_;
    int32 deferred_frames_waited_ = 0;

    std::shared_ptr<RenderParams>* params_;
    std::shared_ptr<RenderResult>* results_;
    unsigned int req_size_;

    std::shared_ptr<msr::airlib::WorkerThreadSignal> wait_signal_;

    bool saved_DisableWorldRendering_ = false;
    UGameViewportClient * const game_viewport_;
    FDelegateHandle end_draw_handle_;
    std::function<void()> query_camera_pose_cb_;

    // Instant the whole batch was captured, sampled on the game thread in OnEndDraw next to
    // query_camera_pose_cb_ (the same instant the camera poses are snapshotted). Every capture in
    // a batch is CaptureSceneDeferred'd in one game-thread pass and the game thread then blocks
    // until readback, so all images in a batch come from THIS instant regardless of how long the
    // serial GPU render and readback take. Written game thread -> read render thread, ordered by
    // the ENQUEUE_RENDER_COMMAND that follows. See airsim.BatchImageTimestamp.
    msr::airlib::TTimePoint batch_time_stamp_ = 0;

    // I-G Step 0a. Segment boundaries for one getScreenshot call; see ImageTiming.h.
    AirSimImageTiming::Call timing_{};

public:
    RenderRequest(UGameViewportClient * game_viewport, std::function<void()>&& query_camera_pose_cb);
    ~RenderRequest();

    void DoTask(ENamedThreads::Type CurrentThread, const FGraphEventRef& MyCompletionGraphEvent)
    {
        ExecuteTask();
    } 

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(RenderRequest, STATGROUP_RenderThreadCommands);
    }

    // read pixels from render target using render thread, then compress the result into PNG
    // argument on the thread that calls this method.
    void getScreenshot(
        std::shared_ptr<RenderParams> params[], std::vector<std::shared_ptr<RenderResult>>& results, unsigned int req_size, bool use_safe_method);

    void ExecuteTask();
};
