#pragma once

#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"
#include "common/WorkerThread.hpp"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/GameViewportClient.h"
#include <memory>
#include <atomic>
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

    // ⚠ Shutdown cancellation. One request passes through three states, and they are NOT
    // interchangeable - each is cancellable by a different party, and one is not cancellable at
    // all. Collapsing them is what deadlocked the editor (see RenderRequest.cpp).
    //
    //   Pending    - the game-thread AsyncTask has not run. Nothing outside getScreenshot's own
    //                stack references the request, so ANY thread may abandon it.
    //   Registered - the AsyncTask ran and added an OnEndDraw handler. Cancellable, but only from
    //                the GAME thread, because cancelling means removing that delegate.
    //   Drawing    - OnEndDraw fired and the completion command is queued on the render thread.
    //                NOT cancellable; it must be flushed to completion.
    //   Abandoned  - terminal; the waiter has gone and no one may touch the request again.
    struct CancelGate
    {
        enum EState : uint8
        {
            Pending = 0,
            Registered,
            Drawing,
            Abandoned
        };

        std::atomic<uint8> state{ Pending };
        RenderRequest* owner = nullptr;
        std::shared_ptr<msr::airlib::WorkerThreadSignal> signal;
    };

    // Set while a PIE session is tearing down. While true, getScreenshot refuses new requests and
    // gives up on outstanding ones instead of waiting forever.
    static void setShuttingDown(bool value);

    // ⚠ GAME THREAD ONLY, and it must be called BEFORE joining any thread that can be inside
    // getScreenshot. Releases every waiter it is safe to release and flushes the rest.
    static void cancelAllPending();

    // ⚠ GAME THREAD ONLY. B2 step 3. Drops the process-wide OnEndDraw registration and restores the
    // viewport flag it borrowed. Call it when the viewport is about to die (PIE EndPlay): the pump
    // holds a RAW UGameViewportClient* and must not outlive it. Idempotent, and a later enrol
    // re-registers against whatever viewport the next PIE session builds. Harmless with the pump
    // switched off - there is simply nothing registered to drop.
    static void endDrawPumpShutdown();

private:
    // ─── B2 step 3: the OnEndDraw pump ───────────────────────────────────────────────────────────
    //
    // ⚠ WHAT WAS WRONG. getScreenshot registered a per-request OnEndDraw handler that did, in
    // effect:
    //
    //     end_draw_handle_ = viewport->OnEndDraw().AddLambda([this, gate]{
    //         ... ;
    //         viewport->OnEndDraw().Remove(end_draw_handle_);   // <- FROM INSIDE ITS OWN BROADCAST
    //     });
    //
    // Removing a delegate from inside the multicast that is currently iterating it is the classic
    // UE mutate-during-broadcast fault. With ONE request in flight it survives; with two it does
    // not, and that is exactly why airsim.StreamCaptureInFlight > 1 SIGSEGV'd reading address 0
    // (the note on CVarStreamCaptureInFlight in SimModeBase.cpp records the measurement: 7.1 fps and
    // then a crash). Two concurrent requests ALSO raced on the single shared viewport flag
    // bDisableWorldRendering, each saving and restoring it without knowing about the other.
    //
    // WHAT THE PUMP DOES. One registration per viewport, never touched from inside its own
    // broadcast - the broadcast only drains a list of enrolled gates. bDisableWorldRendering is
    // REFCOUNTED, so N concurrent requests borrow and return it once between them. Requests enrol in
    // the Registered state and are all dispatched by the next drawn frame's single broadcast, which
    // is also what lets several batches share one frame instead of each waiting for its own.
    //
    // ⚠ IT IS A SWITCH, NOT A REPLACEMENT: airsim.EndDrawPump 0 restores the legacy per-request
    // registration exactly, the same way airsim.GpuReadback 0 restores the legacy readback. What
    // decides which path CANCELS a request is whether it holds a delegate handle, never a re-read of
    // the CVar - see cancelRegisteredOnGameThread.
    static void endDrawPumpEnroll(UGameViewportClient* viewport, const std::shared_ptr<CancelGate>& gate);
    static void endDrawPumpWithdraw(const std::shared_ptr<CancelGate>& gate);
    static void endDrawPumpBroadcast();

    // The per-request half of what the OnEndDraw handler does: stamp, snapshot poses, enqueue the
    // completion command. Game thread only, and only in the Drawing state. BOTH paths call it, so
    // there is exactly one copy of this body and the two arms of the A/B cannot drift.
    void onDrawnOnGameThread();

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

    // B5. One image's copy out of mapped staging memory. Safe to run concurrently for distinct i.
    void copyReadbackImage(unsigned int i, const void* raw, int32 row_pitch_in_pixels);

    // Post-readback tail: stamps, diagnostics, and the signal that releases getScreenshot.
    void finishTask(const TArray<msr::airlib::TTimePoint>& readback_stamps);

    // B3. Render-thread-only registry of batches waiting on their GPU fence.
    static void drainPendingDeferredReadbacks();
    static void ensureDeferredDrainHook();

    // Shutdown variant: drains EVERY deferred batch blocking, in one pass. The ordinary drain
    // waits for OnEndFrameRT, which never fires again once teardown has begun.
    static void drainPendingDeferredReadbacksForced();

    // Removes this request's OnEndDraw handler and restores the viewport flag it changed.
    // Game thread only; valid only in the Registered state.
    void cancelRegisteredOnGameThread();

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

    // Lives as long as the deferred lambdas that reference it, which is why it is a shared_ptr
    // and not a member value: after cancellation the RenderRequest itself is gone.
    std::shared_ptr<CancelGate> gate_;

    // ⚠ These two are the LEGACY path's state (airsim.EndDrawPump 0) and are deliberately kept.
    // Under the pump they stay untouched - end_draw_handle_ never becomes valid - and that is
    // precisely what cancelRegisteredOnGameThread uses to tell the two paths apart.
    bool saved_DisableWorldRendering_ = false;
    UGameViewportClient * const game_viewport_;
    FDelegateHandle end_draw_handle_;
    std::function<void()> query_camera_pose_cb_;

    // Whether I-G timing was on when this request entered getScreenshot. Was a lambda capture; it
    // has to be a member now that onDrawnOnGameThread is shared between the two paths.
    bool timing_enabled_ = false;

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
