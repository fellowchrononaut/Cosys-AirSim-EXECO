#include "RenderRequest.h"
#include "TextureResource.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Async/TaskGraphInterfaces.h"
#include "ImageUtils.h"

#include "AirBlueprintLib.h"
#include "Async/Async.h"
#include "HAL/IConsoleManager.h"
#include "RHIGPUReadback.h"
#include "Misc/CoreDelegates.h"
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
 *   0 = legacy per-image blocking RHICmdList.ReadSurfaceData (the original shipped behaviour)
 *   1 = batched FRHIGPUTextureReadback: all EnqueueCopy submitted, then Lock each
 *   2 = B3 DEFERRED: submit, release the render thread, drain in a later frame  <- DEFAULT
 *
 *  ⚠ DEFAULT CHANGED 0 -> 2 on 2026-08-21. It was 0 because mode 1 was an unproven experiment and
 *  mode 2 was new; a readback path stays off until measured. It has now been measured
 *  (PHASE-A1-RESULTS §23, §24, §26):
 *
 *    - sim frame rate under a 16 x 1080p ROS load: 5.6 -> 75.7 fps
 *    - GPU fence wait on the render thread: ~11 ms -> 0.03 ms
 *    - output pixel-identical to mode 0 for every modality, pinhole AND cube, verified with the
 *      sim paused so byte comparison means something
 *    - clean under 4 concurrent clients on 3 RPC ports, and under simPause
 *
 *  The ONE semantic difference: a call now costs about one extra rendered frame of latency,
 *  because the pixels are collected at the next frame boundary instead of inside the frame that
 *  drew them. Content is still the CAPTURE instant, not the drain instant, and the timestamp is
 *  unchanged - both verified. Set airsim.GpuReadback 0 to restore the old path exactly. */
static TAutoConsoleVariable<int32> CVarGpuReadback(
    TEXT("airsim.GpuReadback"),
    2,
    TEXT("I-G: image readback path.\n")
    TEXT(" 0: legacy ReadSurfaceData per image, one GPU sync point each\n")
    TEXT(" 1: batched FRHIGPUTextureReadback - submit all copies, then drain\n")
    TEXT(" 2: deferred - submit, free the render thread, drain at a later frame (DEFAULT)"),
    ECVF_Default);

/** B2 step 3. Which OnEndDraw registration a capture uses. Default 1 (the pump).
 *
 *   1 = ONE process-wide registration owned by the pump, drained per drawn frame  <- DEFAULT
 *   0 = LEGACY: every request adds its own handler and removes it from inside that handler's own
 *       broadcast. Kept, and kept working, so this is an A/B in one build rather than a rewrite.
 *
 *  ⚠ WHY THE DEFAULT IS 1 RATHER THAN 0. Mode 0 is not a safe fallback that happens to be slower -
 *  it is undefined behaviour that survives only while exactly one request is ever in flight. It is
 *  the direct cause of the airsim.StreamCaptureInFlight > 1 SIGSEGV. Mode 1 is a bug fix, and the
 *  switch exists to make the fix falsifiable, not because mode 0 is a supported configuration.
 *
 *  ⚠ WHAT DOES NOT CHANGE, EITHER WAY. Both modes run the SAME onDrawnOnGameThread() body: same
 *  capture instant, same pose snapshot, same completion command, same results. simGetImages is
 *  unaffected in signature, in semantics and in returned bytes; classic RPC clients cannot tell the
 *  two modes apart on a single in-flight request, which is the pre-B2 case. The step-3 exit
 *  criterion checks exactly that. */
static TAutoConsoleVariable<int32> CVarEndDrawPump(
    TEXT("airsim.EndDrawPump"),
    1,
    TEXT("B2: OnEndDraw registration.\n")
    TEXT(" 0: legacy - one delegate per request, removed from inside its own broadcast (UB with >1)\n")
    TEXT(" 1: pump - one registration for the process, drained per drawn frame (DEFAULT)"),
    ECVF_Default);

/** B3. Whether mode 2 also defers CUBE (fisheye) requests. Default ON.
 *
 *  This was briefly gated OFF, on the belief that deferral corrupted fisheye SurfaceNormals. That
 *  was the wrong diagnosis: the corruption was a readback DISPATCH bug that hit the pinhole path
 *  just as hard, and it is fixed in drainGpuReadbacks. With the sim paused, SurfaceNormals is now
 *  byte-identical over 8 captures in all three modes, cube and pinhole alike. Kept as a switch so
 *  the cube path can be isolated without a rebuild if it is ever suspect again. */
static TAutoConsoleVariable<int32> CVarGpuReadbackCube(
    TEXT("airsim.GpuReadbackCube"),
    1,
    TEXT("B3: 1 = airsim.GpuReadback 2 defers CUBE requests too (default). 0 = cube stays blocking."),
    ECVF_Default);

/** B3 investigation. Per-request trace of what the cube resample actually bound. */
static TAutoConsoleVariable<int32> CVarLogCubeResample(
    TEXT("airsim.LogCubeResample"),
    0,
    TEXT("B3 test: log the output/face textures each cube resample binds, and whether it ran."),
    ECVF_Default);

/** B5. Parallelise the staging->CPU copy across the images in a batch.
 *
 *  Measured 2026-08-21 at 16 x 1080p: the copy is 80 ms of a ~380 ms c+d under ROS saturation and
 *  ~52 ms of ~171 ms with a light client - the single largest REMOVABLE term left, now that B3 has
 *  taken the GPU fence wait to 0.03 ms. It is a plain memcpy out of staging memory, per image and
 *  independent, so it parallelises the same way airsim.ParallelImageDecode already does (4.2x on
 *  its own segment). Serialising 16 independent memcpys on the render thread is pure waste.
 *
 *  ⚠ Only the COPY is parallel. Lock and Unlock stay serial on the render thread: they touch RHI
 *  state and the fence, and nothing about this change is worth a threading bug in the RHI. */
static TAutoConsoleVariable<int32> CVarParallelReadbackCopy(
    TEXT("airsim.ParallelReadbackCopy"),
    1,
    TEXT("B5: 1 = copy the batch's images out of staging memory in parallel (default)\n")
    TEXT("    0 = serial, one image at a time"),
    ECVF_Default);

/** B3. How many rendered frames a deferred batch may wait for its GPU fence before the drain
 *  gives up and blocks. A batch that never reports IsReady() would otherwise hang the calling
 *  RPC thread forever, since getScreenshot waits on wait_signal_ with no deadline. */
static TAutoConsoleVariable<int32> CVarGpuReadbackMaxFrames(
    TEXT("airsim.GpuReadbackMaxFrames"),
    4,
    TEXT("B3: frames a deferred readback may wait before forcing a blocking drain."),
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

// ⚠ Shutdown cancellation registry.
//
// WHY THIS EXISTS. getScreenshot used to wait on wait_signal_ in an unbounded loop, with a comment
// explaining that walking away would free objects the render thread still references. That is true
// of ONE of the three states a request can be in, and the loop did not distinguish them. The cost,
// measured 2026-08-21 with the Phase E capture driver running: pressing Stop froze the editor
// permanently, with 159 threads parked in futex waits. Both halves of the cycle, from gdb:
//
//   Thread 1 (game)   std::thread::join()  <- ASimModeBase::stopStreamCapture <- EndPlay
//   Thread 169        WorkerThreadSignal::waitFor <- RenderRequest::getScreenshot
//                                                 <- getImagesAllVehicles <- streamCaptureLoop
//
// The worker waits for the game thread to service its AsyncTask; the game thread waits for the
// worker to exit. Neither can move. A pump-and-join inside EndPlay cannot fix it either: the
// worker is ultimately waiting for OnEndDraw, which only fires when a FRAME IS DRAWN, and EndPlay
// runs inside UEditorEngine::Tick (EditorEngine.cpp:2430) - no further frame can be drawn until
// that Tick returns. FlushRenderingCommands does not help, because the completion command is
// enqueued INSIDE OnEndDraw; if OnEndDraw has not fired there is nothing queued to flush.
//
// So cancellation is per-state (see CancelGate in the header), and the transitions are CAS'd
// because the waiter races the game thread for the right to abandon.
namespace
{
    std::atomic<bool> g_shutting_down{ false };
    FCriticalSection g_gate_mutex;
    TArray<std::shared_ptr<RenderRequest::CancelGate>> g_live_gates;
}

void RenderRequest::setShuttingDown(bool value)
{
    g_shutting_down.store(value, std::memory_order_release);
}

void RenderRequest::cancelRegisteredOnGameThread()
{
    check(IsInGameThread());

    // ⚠ Which path registered this request is recorded by whether it holds a delegate handle, NOT by
    // re-reading airsim.EndDrawPump. The CVar can be flipped between enrol and cancel, and a request
    // must always be cancelled the way it was registered - otherwise a legacy request leaks its
    // delegate and a pumped one leaks its share of the refcounted viewport flag.
    if (game_viewport_ != nullptr && end_draw_handle_.IsValid()) {
        game_viewport_->OnEndDraw().Remove(end_draw_handle_);
        end_draw_handle_.Reset();
        game_viewport_->bDisableWorldRendering = saved_DisableWorldRendering_;
        return;
    }

    // B2 step 3. A pumped request owns no delegate - it owns a slot in the waiting list and a share
    // of the refcounted viewport flag. Withdrawing returns both, and does nothing at all if the
    // broadcast already drained this gate (in which case the request is Drawing, not Registered, and
    // cancelAllPending will not have called us).
    endDrawPumpWithdraw(gate_);
}

void RenderRequest::cancelAllPending()
{
    check(IsInGameThread());

    TArray<std::shared_ptr<CancelGate>> gates;
    {
        FScopeLock lock(&g_gate_mutex);
        gates = g_live_gates;
    }

    int32 pending = 0, registered = 0, drawing = 0;
    for (const std::shared_ptr<CancelGate>& gate : gates) {
        // Pending: the AsyncTask has not run. It will find the gate Abandoned and return without
        // dereferencing the request, so releasing the waiter now is safe.
        uint8 expected = CancelGate::Pending;
        if (gate->state.compare_exchange_strong(expected, CancelGate::Abandoned)) {
            ++pending;
            gate->signal->signal();
            continue;
        }

        // Registered: an OnEndDraw handler is installed and no frame has drawn. Removing that
        // delegate is exactly what the handler would have done, and this IS the game thread, so
        // it cannot race the handler.
        expected = CancelGate::Registered;
        if (gate->state.compare_exchange_strong(expected, CancelGate::Abandoned)) {
            ++registered;
            if (gate->owner != nullptr)
                gate->owner->cancelRegisteredOnGameThread();
            gate->signal->signal();
            continue;
        }

        ++drawing;
    }

    // Drawing requests already have SceneDrawCompletion queued; the flush below runs it. Under
    // airsim.GpuReadback 2 that call only SUBMITS the copies and parks the batch for OnEndFrameRT,
    // which will not fire again - hence the forced drain, enqueued after it so it runs after it.
    ENQUEUE_RENDER_COMMAND(AirSimCancelDrainDeferred)
    ([](FRHICommandListImmediate&) { RenderRequest::drainPendingDeferredReadbacksForced(); });
    FlushRenderingCommands();

    if (pending + registered + drawing > 0)
        UE_LOG(LogTemp, Warning,
               TEXT("[AirSim] shutdown: cancelled %d queued and %d registered image request(s), flushed %d already drawing"),
               pending, registered, drawing);
}

// ─────────────────────────────────────────────────────────────────────────────────────────────────
// B2 step 3. ONE OnEndDraw registration for the whole process. See the block comment above
// endDrawPumpEnroll in RenderRequest.h for what was wrong with the per-request one.
//
// Thread rules, all of them load-bearing:
//   * enrol / withdraw / broadcast / shutdown are GAME THREAD ONLY. The mutex is not there to make
//     them cross-thread safe - it is there because the waiting list and the refcount are touched
//     from several places on that thread and a half-updated pair leaks the viewport flag.
//   * the delegate is added and removed only in enrol and shutdown, i.e. NEVER from inside the
//     broadcast. That single property is the whole bug fix.
namespace
{
    FCriticalSection g_pump_mutex;
    UGameViewportClient* g_pump_viewport = nullptr;
    FDelegateHandle g_pump_handle;
    TArray<std::shared_ptr<RenderRequest::CancelGate>> g_pump_waiting;

    // How many enrolled requests are currently borrowing bDisableWorldRendering, and what it was
    // before the first of them borrowed it.
    int32 g_pump_render_refs = 0;
    bool g_pump_saved_disable_world_rendering = false;

    /** Give the viewport flag back once the last borrower is gone. Caller holds g_pump_mutex. */
    void pumpReleaseRenderFlag_Locked()
    {
        if (g_pump_render_refs <= 0)
            return;
        if (--g_pump_render_refs == 0 && g_pump_viewport != nullptr)
            g_pump_viewport->bDisableWorldRendering = g_pump_saved_disable_world_rendering ? 1 : 0;
    }
}

void RenderRequest::endDrawPumpEnroll(UGameViewportClient* viewport, const std::shared_ptr<CancelGate>& gate)
{
    check(IsInGameThread());
    if (viewport == nullptr || !gate)
        return;

    FScopeLock lock(&g_pump_mutex);

    // A new PIE session builds a new viewport client. Rebind here - on the game thread and outside
    // any broadcast, the only place it is safe to mutate the delegate list.
    if (g_pump_viewport != viewport) {
        if (g_pump_viewport != nullptr && g_pump_handle.IsValid())
            g_pump_viewport->OnEndDraw().Remove(g_pump_handle);
        g_pump_viewport = viewport;
        g_pump_handle = viewport->OnEndDraw().AddStatic(&RenderRequest::endDrawPumpBroadcast);
        // The previous viewport's borrowers died with it; there is nothing to give back to a
        // destroyed object, and the count must not carry over into the new one.
        g_pump_render_refs = 0;
    }

    if (g_pump_render_refs == 0) {
        g_pump_saved_disable_world_rendering = viewport->bDisableWorldRendering != 0;
        viewport->bDisableWorldRendering = 0;
    }
    ++g_pump_render_refs;

    g_pump_waiting.Add(gate);
}

void RenderRequest::endDrawPumpWithdraw(const std::shared_ptr<CancelGate>& gate)
{
    check(IsInGameThread());
    if (!gate)
        return;

    FScopeLock lock(&g_pump_mutex);
    // ⚠ Release the flag ONLY if this gate was still waiting. A gate the broadcast already drained
    // is gone from the list and its refcount share was returned in bulk there; decrementing again
    // would restore bDisableWorldRendering underneath a request still using it. Remove()'s return
    // count is what keeps the two paths mutually exclusive.
    if (g_pump_waiting.Remove(gate) > 0)
        pumpReleaseRenderFlag_Locked();
}

void RenderRequest::endDrawPumpBroadcast()
{
    check(IsInGameThread());

    TArray<std::shared_ptr<CancelGate>> drawing;
    {
        FScopeLock lock(&g_pump_mutex);
        if (g_pump_waiting.Num() == 0)
            return;   // the common case while nothing is capturing: one lock and out

        // ⚠ Swap the list out under the lock and dispatch OUTSIDE it. Dispatch runs a caller-
        // supplied pose callback and an ENQUEUE_RENDER_COMMAND; holding a lock across either is how
        // a capture-time deadlock gets built.
        drawing = MoveTemp(g_pump_waiting);
        g_pump_waiting.Reset();

        // Every enrolled request is dispatched by this one broadcast, so the flag they collectively
        // borrowed goes back now - once, not once per request.
        if (g_pump_viewport != nullptr && g_pump_render_refs > 0)
            g_pump_viewport->bDisableWorldRendering = g_pump_saved_disable_world_rendering ? 1 : 0;
        g_pump_render_refs = 0;
    }

    for (const std::shared_ptr<CancelGate>& gate : drawing) {
        // Registered -> Drawing. Past this point the request belongs to the render thread and can
        // only be flushed, never abandoned. Both sides of this CAS run on the game thread, so it
        // cannot lose the race; a failure means cancelAllPending already claimed the request.
        uint8 expected = CancelGate::Registered;
        if (!gate->state.compare_exchange_strong(expected, CancelGate::Drawing))
            continue;

        if (gate->owner != nullptr)
            gate->owner->onDrawnOnGameThread();
    }
}

void RenderRequest::endDrawPumpShutdown()
{
    check(IsInGameThread());

    FScopeLock lock(&g_pump_mutex);
    if (g_pump_viewport != nullptr) {
        if (g_pump_handle.IsValid())
            g_pump_viewport->OnEndDraw().Remove(g_pump_handle);
        if (g_pump_render_refs > 0)
            g_pump_viewport->bDisableWorldRendering = g_pump_saved_disable_world_rendering ? 1 : 0;
    }
    g_pump_handle.Reset();
    g_pump_viewport = nullptr;
    g_pump_waiting.Reset();
    g_pump_render_refs = 0;
}

// ⚠ ONE COPY, called by BOTH the pump and the legacy path. This is the body that used to be inline
// in getScreenshot's OnEndDraw lambda, moved verbatim - which is what makes airsim.EndDrawPump a
// genuine A/B of the REGISTRATION and nothing else.
void RenderRequest::onDrawnOnGameThread()
{
    check(IsInGameThread());

    // I-G Step 0a: end of segment (b) - the wait for the next rendered frame. This is the boundary
    // that decides latency-bound vs readback-bound.
    if (timing_enabled_)
        timing_.t_end_draw = AirSimImageTiming::Clock::now();

    // Capture instant for the whole batch, taken with the poses so time and pose agree. This is the
    // moment every image in the batch actually corresponds to; the serial GPU render and readback
    // that follow add latency, not temporal skew.
    batch_time_stamp_ = msr::airlib::ClockFactory::get()->nowNanos();

    // capture CameraPose for this frame
    query_camera_pose_cb_();

    // The completion is called immediately after GameThread sends the rendering commands to
    // RenderThread. Hence our ExecuteTask will execute *immediately* after RenderThread renders the
    // scene.
    RenderRequest* This = this;
    ENQUEUE_RENDER_COMMAND(SceneDrawCompletion)
    (
        [This](FRHICommandListImmediate& RHICmdList) {
            This->ExecuteTask();
        });
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

        // ⚠ RenderResult leaves these uninitialised. Every normal path overwrites them in
        // setupRenderResource, but the cancellation path below returns without ever reaching it,
        // and the formatting loop at the tail keys off `width != 0`. Garbage here would size a
        // SetNumUninitialized from a stack value.
        results[i]->width = 0;
        results[i]->height = 0;
    }

    // Refuse outright once teardown has begun. Without this, a worker that passed its own
    // stop-flag check a microsecond before cancelAllPending ran would register a gate that
    // nothing will ever cancel, and the join would hang exactly as before.
    if (g_shutting_down.load(std::memory_order_acquire)) {
        UE_LOG(LogTemp, Warning,
               TEXT("[AirSim] image request of %d refused: the session is shutting down"), (int)req_size);
        return;
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

        gate_ = std::make_shared<CancelGate>();
        gate_->owner = this;
        gate_->signal = wait_signal_;
        {
            FScopeLock lock(&g_gate_mutex);
            g_live_gates.Add(gate_);
        }

        // Queue up the task of querying camera pose in the game thread and synchronizing render thread with camera pose
        // ⚠ `gate` is captured BY VALUE and every access before the CAS goes through it, never
        // through `this`: if the request was abandoned, `this` is a destroyed stack object.
        AsyncTask(ENamedThreads::GameThread, [this, timing_on, gate = gate_]() {
            check(IsInGameThread());

            uint8 expected = CancelGate::Pending;
            if (!gate->state.compare_exchange_strong(expected, CancelGate::Registered))
                return; // Abandoned while queued - the waiter is gone and owns nothing here.

            // I-G Step 0a: end of segment (a) - how long the game thread took to pick this up.
            if (timing_on)
                timing_.t_game_task = AirSimImageTiming::Clock::now();

            timing_enabled_ = timing_on;

            if (CVarEndDrawPump.GetValueOnGameThread() != 0) {
                // B2 step 3. Enrol with the process-wide pump. It performs the Registered ->
                // Drawing CAS and calls onDrawnOnGameThread() on the next drawn frame.
                endDrawPumpEnroll(game_viewport_, gate);
            }
            else {
                // ⚠ LEGACY (airsim.EndDrawPump 0). Preserved verbatim, self-removal and all, so the
                // pump can be A/B'd in one build. It is UNSAFE with more than one request in flight
                // - the Remove below runs inside this very delegate's broadcast. Do not "clean this
                // up": the fault IS the point of keeping it.
                saved_DisableWorldRendering_ = game_viewport_->bDisableWorldRendering;
                game_viewport_->bDisableWorldRendering = 0;
                end_draw_handle_ = game_viewport_->OnEndDraw().AddLambda([this, gate] {
                    check(IsInGameThread());

                    // Past this point the request belongs to the render thread and can no longer be
                    // abandoned - only flushed. Both sides of this CAS run on the game thread, so it
                    // cannot lose the race; it is here to make the ownership handover explicit.
                    uint8 draw_expected = CancelGate::Registered;
                    if (!gate->state.compare_exchange_strong(draw_expected, CancelGate::Drawing))
                        return;

                    onDrawnOnGameThread();

                    game_viewport_->bDisableWorldRendering = saved_DisableWorldRendering_;

                    assert(end_draw_handle_.IsValid());
                    game_viewport_->OnEndDraw().Remove(end_draw_handle_);
                });
            }

            // while we're still on GameThread, enqueue request for capture the scene!
            for (unsigned int i = 0; i < req_size_; ++i) {
                if (params_[i]->render_target != nullptr && params_[i]->render_component != nullptr) {
                    // Phase 3b step 4 (F1). A generic-camera request renders its cube faces
                    // INSTEAD of the pinhole capture, not as well as it: the pinhole render would
                    // be overwritten by the resample, so firing it would buy a discarded frame.
                    // face_components is empty for every pinhole request, so an ordinary request
                    // pays one TArray::Num() compare and then does exactly what it did before.
                    // All of these are still queued in the same single game-thread pass, so
                    // finding F9 - one world state, one sim instant per batch - still holds, and
                    // now holds across the faces of a camera as well as across cameras.
                    if (params_[i]->face_components.Num() > 0) {
                        for (USceneCaptureComponent2D* face_component : params_[i]->face_components) {
                            if (face_component != nullptr)
                                face_component->CaptureSceneDeferred();
                        }
                    }
                    else {
                        params_[i]->render_component->CaptureSceneDeferred();
                    }
                }
            }
        });

        // wait for this task to complete
        //
        // The original loop here was unbounded, because "the lambda still references a few objects
        // for which there is no refcount - walking away will cause memory corruption". That holds
        // only once the request reaches Drawing. Before that, the gate lets us leave safely, and
        // cancelAllPending signals us so we do not sit out the 5 s timeout to find that out.
        bool cancelled = false;
        for (;;) {
            const bool signalled = wait_signal_->waitFor(5);
            if (signalled) {
                cancelled = gate_->state.load(std::memory_order_acquire) == CancelGate::Abandoned;
                break;
            }

            // Timed out. If we are tearing down and nobody has picked the request up yet, claim it
            // and leave; if it is already Registered or Drawing, cancelAllPending owns the outcome
            // and will either release us or flush the request to completion.
            if (g_shutting_down.load(std::memory_order_acquire)) {
                uint8 expected = CancelGate::Pending;
                if (gate_->state.compare_exchange_strong(expected, CancelGate::Abandoned)) {
                    cancelled = true;
                    break;
                }
            }

            UE_LOG(LogTemp, Warning, TEXT("Failed: timeout waiting for screenshot"));
        }

        {
            FScopeLock lock(&g_gate_mutex);
            g_live_gates.Remove(gate_);
        }

        if (cancelled) {
            // Results keep their zeroed width/height, so callers see empty images rather than a
            // half-formatted buffer. Nothing below this point may run: the render targets are
            // being torn down.
            UE_LOG(LogTemp, Warning,
                   TEXT("[AirSim] image request of %d cancelled during shutdown"), (int)req_size);
            return;
        }
    }

    for (unsigned int i = 0; i < req_size; ++i) {
        if (params[i]->render_target != nullptr && params[i]->render_component != nullptr) {
            if (!params[i]->pixels_as_float) {
                if (results[i]->width != 0 && results[i]->height != 0) {
                    results[i]->image_data_uint8.SetNumUninitialized(results[i]->width * results[i]->height * 3, false);

                    // ⚠ Fail LOUD and BLACK rather than quietly returning uninitialised heap.
                    //
                    // image_data_uint8 is SetNumUninitialized and the copy below is bounded by
                    // bmp.Num(), so a short or empty bmp leaves part or all of the buffer holding
                    // whatever the allocator last had there - which in practice is the PREVIOUS
                    // capture. That is how a SurfaceNormals request returned Segmentation pixels
                    // (2026-08-21): a plausible, well-formed, completely wrong image, with no
                    // error anywhere. It also hands raw process memory to an RPC client.
                    //
                    // The dispatch bug that caused it is fixed in drainGpuReadbacks; this is the
                    // guard that makes the NEXT one visible instead of convincing.
                    const int32 expected_px = results[i]->width * results[i]->height;
                    if (results[i]->bmp.Num() < expected_px) {
                        static FCriticalSection warn_mutex;
                        static bool warned = false;
                        {
                            FScopeLock lock(&warn_mutex);
                            if (!warned) {
                                warned = true;
                                UE_LOG(LogTemp, Error,
                                       TEXT("[AirSim] readback produced %d of %d expected pixels - returning BLACK. ")
                                       TEXT("A readback path filled the wrong buffer for this request."),
                                       results[i]->bmp.Num(), expected_px);
                            }
                        }
                        FMemory::Memzero(results[i]->image_data_uint8.GetData(),
                                         results[i]->image_data_uint8.Num());
                    }
                    else if (params[i]->compress)
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
// ⚠ BOTH of this comment's original conclusions were RETRACTED on 2026-08-20 (PHASE-A1-RESULTS
// §22.2). They are recorded here because both argued against the fix that turned out to work.
//
// RETRACTED 1 - "it is the CPU copy out of staging memory, 8.29 MB in ~19 ms = ~436 MB/s ... and
// why async could not [win] either, since it would relocate the copy, not remove it."
// The copy is not 19 ms. Measured with the Lock/copy split that this function now records,
// 8.29 MB takes 3.25 ms - 2.56 GB/s. The 19 ms was the GPU FENCE WAIT, which the 2026-08-04 A/B
// had no way to separate from the copy. The two want opposite fixes, so the misattribution led
// directly to the wrong conclusion: deferring the readback does not relocate a copy, it deletes
// an ~11 ms render-thread stall. That is airsim.GpuReadback 2, and it works.
//
// RETRACTED 2 - "it also CORRUPTS float image types: DepthPlanar mean 0.339 vs 131.642."
// Does not reproduce. Warmed and interleaved over 4 rounds, DepthPlanar / DepthPerspective /
// DisparityNormalized agree between modes 0, 1 and 2 to 3 decimal places. The original figure is
// consistent with the known COLD-FRAME defect: a lazily created render target's first capture
// returns a stale frame, and whichever mode was measured first absorbed it. A cold
// DepthPerspective reads mean 0.356 against a warm 1.183 - the same shape as the 0.339 quoted.
//
// What DOES still hold: batching alone buys nothing (47.8 vs 48.1 ms/call at 1080p), and now it
// has a mechanism. Batching removes N-1 sync points, but there was only ever ONE fence wait to
// remove, so it had nothing to win.
//
// Kept, defaulted off, as the A/B control. See I-G in sim_issues plan.
void RenderRequest::executeBatchedGpuReadback(TArray<msr::airlib::TTimePoint>& readback_stamps)
{
    submitGpuReadbacks();
    drainGpuReadbacks(readback_stamps, /*force_blocking=*/true);
}

// B3 phase 1. Submit every copy and RETURN. Shared by mode 1 (which drains immediately) and
// mode 2 (which drains in a later frame) - one submit path, so the two modes cannot drift apart.
void RenderRequest::submitGpuReadbacks()
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

}

// B5. The per-image copy out of staging memory, lifted out of the drain loop so it can run
// either serially or inside a ParallelFor. Touches only results_[i] and the mapped pointer for
// image i, so concurrent invocations for different i share nothing.
void RenderRequest::copyReadbackImage(unsigned int i, const void* raw, int32 row_pitch_in_pixels)
{
    if (raw == nullptr)
        return;

    const int32 w = results_[i]->width;
    const int32 h = results_[i]->height;

    // ⚠ BOTH the GPU format AND the client's request matter, and they DISAGREE routinely.
    //
    // The old code dispatched on the GPU format alone, with a comment claiming that
    // trusting the request "is how you get garbage pixels". The opposite was true. A
    // SurfaceNormals target is created with auto_format + force_linear_gamma, which
    // resolves to PF_FloatRGBA, while the client asks for uint8. The format-only dispatch
    // then filled bmp_float and left bmp EMPTY - and the uint8 formatter in getScreenshot
    // reads results[i]->bmp unconditionally, walking off an empty TArray.
    //
    // That out-of-bounds read is why a SurfaceNormals request came back holding the
    // PREVIOUS capture's pixels: measured 2026-08-21 with the sim paused, SurfaceNormals
    // returned mean 225.14 against Segmentation's 225.04 on the cube path, and 253.33
    // against 253.38 on the pinhole path. Plausible images, entirely wrong content. The
    // legacy path never had this because RHIReadSurfaceData converts float -> FColor
    // itself.
    //
    // So: convert whenever the pair disagrees. RCM_UNorm + SetLinearToGamma(false) is what
    // setupRenderResource asks the legacy path for, and ToFColor(false) is its equivalent.
    const bool want_float = (params_ != nullptr && params_[i] != nullptr)
                                ? params_[i]->pixels_as_float
                                : (formats_[(int32)i] == PF_FloatRGBA);

    if (formats_[(int32)i] == PF_B8G8R8A8) {
        const FColor* src = static_cast<const FColor*>(raw);
        if (!want_float) {
            results_[i]->bmp.SetNumUninitialized(w * h, false);
            FColor* dst = results_[i]->bmp.GetData();
            for (int32 y = 0; y < h; ++y)
                FMemory::Memcpy(dst + y * w, src + y * row_pitch_in_pixels, w * sizeof(FColor));
        }
        else {
            results_[i]->bmp_float.SetNumUninitialized(w * h, false);
            FFloat16Color* dst = results_[i]->bmp_float.GetData();
            for (int32 y = 0; y < h; ++y)
                for (int32 x = 0; x < w; ++x)
                    dst[y * w + x] = FFloat16Color(FLinearColor(src[y * row_pitch_in_pixels + x]));
        }
    }
    else { //PF_FloatRGBA, guaranteed by phase 1
        const FFloat16Color* src = static_cast<const FFloat16Color*>(raw);
        if (want_float) {
            results_[i]->bmp_float.SetNumUninitialized(w * h, false);
            FFloat16Color* dst = results_[i]->bmp_float.GetData();
            for (int32 y = 0; y < h; ++y)
                FMemory::Memcpy(dst + y * w, src + y * row_pitch_in_pixels, w * sizeof(FFloat16Color));
        }
        else {
            results_[i]->bmp.SetNumUninitialized(w * h, false);
            FColor* dst = results_[i]->bmp.GetData();
            for (int32 y = 0; y < h; ++y) {
                for (int32 x = 0; x < w; ++x) {
                    const FFloat16Color& c = src[y * row_pitch_in_pixels + x];
                    dst[y * w + x] = FLinearColor(c.R.GetFloat(), c.G.GetFloat(),
                                                  c.B.GetFloat(), c.A.GetFloat()).ToFColor(false);
                }
            }
        }
    }
}

// B3 phase 2. Drain the submitted copies into results_.
//
// force_blocking=true  - mode 1 and the deferred fallback: Lock unconditionally, which waits on
//                        the GPU fence. This is the ~11 ms stall measured in PHASE-A1-RESULTS §22.
// force_blocking=false - mode 2: only drain when every readback reports IsReady(), so Lock finds
//                        its fence already signalled and returns without waiting. Returns false if
//                        the batch is not ready yet, leaving state untouched for the next frame.
bool RenderRequest::drainGpuReadbacks(TArray<msr::airlib::TTimePoint>& readback_stamps, bool force_blocking)
{
    if (!force_blocking) {
        for (unsigned int i = 0; i < req_size_; ++i) {
            if (!enqueued_[(int32)i])
                continue;
            if (readbacks_[(int32)i].IsValid() && !readbacks_[(int32)i]->IsReady())
                return false;          // not this frame; nothing has been consumed yet
        }
    }

    const bool split_on = AirSimImageTiming::ReportPeriodSeconds() > 0;

    // B5 phase 1 - Lock every image first, serially. Lock/Unlock touch RHI state and the GPU
    // fence, so they stay on the render thread alone; only the memcpy that follows is parallel.
    // After B3 the fence is already signalled, so these Locks cost ~0.03 ms for the whole batch.
    TArray<const void*> mapped;
    TArray<int32> pitches;
    mapped.SetNumZeroed((int32)req_size_);
    pitches.SetNumZeroed((int32)req_size_);

    const auto t_lock0 = AirSimImageTiming::Clock::now();
    for (unsigned int i = 0; i < req_size_; ++i) {
        if (!enqueued_[(int32)i] || results_[i]->width <= 0 || results_[i]->height <= 0)
            continue;
        int32 row_pitch_in_pixels = 0;
        mapped[(int32)i] = readbacks_[(int32)i]->Lock(row_pitch_in_pixels);
        pitches[(int32)i] = row_pitch_in_pixels;
    }
    if (split_on)
        timing_.lock_ms += AirSimImageTiming::ToMs(AirSimImageTiming::Clock::now() - t_lock0);

    // B5 phase 2 - the copies. Independent per image: each writes only its own results_[i] buffer.
    const auto t_copy0 = AirSimImageTiming::Clock::now();
    const bool parallel = CVarParallelReadbackCopy.GetValueOnRenderThread() != 0 && req_size_ > 1;
    if (parallel) {
        ParallelFor((int32)req_size_, [this, &mapped, &pitches](int32 i) {
            copyReadbackImage((unsigned int)i, mapped[i], pitches[i]);
        });
    }
    else {
        for (unsigned int i = 0; i < req_size_; ++i)
            copyReadbackImage(i, mapped[(int32)i], pitches[(int32)i]);
    }
    if (split_on)
        timing_.copy_ms += AirSimImageTiming::ToMs(AirSimImageTiming::Clock::now() - t_copy0);
    if (split_on)
        timing_.have_split = true;

    // B5 phase 3 - Unlock, serial again, and stamp.
    for (unsigned int i = 0; i < req_size_; ++i) {
        if (mapped[(int32)i] != nullptr)
            readbacks_[(int32)i]->Unlock();
        readback_stamps[(int32)i] = msr::airlib::ClockFactory::get()->nowNanos();
    }

    readbacks_.Reset();
    return true;
}

// Phase 3b step 4. Six pinhole cube faces -> one image of the calibrated camera, written into
// the request's EXISTING output render target (finding F2), so ReadSurfaceData and everything
// downstream of it need no change.
//
// WHY HERE. The resample must run after the deferred face captures have rendered and before the
// readback. CaptureSceneDeferred() queues into Unreal's own deferred-capture list, processed
// during the scene render, while ExecuteTask is enqueued from OnEndDraw - reasoning about the
// interleaving of two queueing mechanisms is fragile. ExecuteTask already runs on the render
// thread immediately after the scene render, and it is ours, so dispatching here makes the
// ordering trivially correct with no new synchronisation. That is finding F3, and it is why
// this call sits at the TOP of ExecuteTask, before the airsim.GpuReadback dispatch and not
// inside either readback path.
//
// A batch with no generic camera in it runs req_size_ integer compares here and nothing else.
void RenderRequest::executeCubeResample()
{
    if (!AirSimCubeResampleEnabled())
        return;

    FRHICommandListImmediate* cmd_list = nullptr;

    for (unsigned int i = 0; i < req_size_; ++i) {
        const RenderParams* params = params_[i].get();
        if (params == nullptr || params->face_targets.Num() == 0)
            continue; //pinhole request: this is the whole cost of the feature for it

        if (params->render_target == nullptr || !params->raymap.IsValid() || !params->raymap->ready)
            continue;

        FTextureRenderTargetResource* output_resource = params->render_target->GetRenderTargetResource();
        if (output_resource == nullptr)
            continue;
        FRHITexture* output_texture = output_resource->GetRenderTargetTexture();
        if (output_texture == nullptr)
            continue;

        FRHITexture* face_textures[kAirSimCubeResampleMaxFaces] = {};
        const int32 face_count = FMath::Min(params->face_targets.Num(), kAirSimCubeResampleMaxFaces);
        bool faces_ready = face_count > 0;
        for (int32 face = 0; face < face_count; ++face) {
            FRHITexture* face_texture = nullptr;
            UTextureRenderTarget2D* face_target = params->face_targets[face];
            if (face_target != nullptr) {
                FTextureRenderTargetResource* face_resource = face_target->GetRenderTargetResource();
                if (face_resource != nullptr)
                    face_texture = face_resource->GetRenderTargetTexture();
            }
            if (face_texture == nullptr) {
                faces_ready = false;
                break;
            }
            face_textures[face] = face_texture;
        }
        if (CVarLogCubeResample.GetValueOnRenderThread() != 0) {
            FString faces;
            for (int32 f = 0; f < face_count; ++f)
                faces += FString::Printf(TEXT(" f%d=%s"), f,
                                         face_textures[f] ? *face_textures[f]->GetName().ToString() : TEXT("NULL"));
            UE_LOG(LogTemp, Log,
                   TEXT("[AirSim][cube] req[%u] mode=%d out=%s %dx%d ready=%d faces:%s"),
                   i, params->resample_mode,
                   *output_texture->GetName().ToString(),
                   output_texture->GetSizeX(), output_texture->GetSizeY(),
                   faces_ready ? 1 : 0, *faces);
        }

        if (!faces_ready)
            continue; //leave the target alone rather than write half an image into it

        if (cmd_list == nullptr)
            cmd_list = &GetImmediateCommandList_ForRenderCommand();

        AirSimCubeResample_RenderThread(*cmd_list, face_textures, face_count, output_texture, *params->raymap,
                                        static_cast<EAirSimCubeResampleMode>(params->resample_mode));
    }
}

// B3. Deferred batches waiting for their GPU fence.
//
// ⚠ Render thread ONLY. Every writer is on the render thread - ExecuteTask arrives via
// ENQUEUE_RENDER_COMMAND and the drain runs from OnEndFrameRT - so no lock is needed. That is
// load-bearing: the MultiAgent build runs four RPC servers whose handlers capture concurrently,
// and their RenderRequests would otherwise race here.
namespace
{
    TArray<RenderRequest*> g_deferred_pending;
    FDelegateHandle g_deferred_hook;
}

void RenderRequest::ensureDeferredDrainHook()
{
    if (!g_deferred_hook.IsValid())
        g_deferred_hook = FCoreDelegates::OnEndFrameRT.AddStatic(&RenderRequest::drainPendingDeferredReadbacks);
}

// One pass per rendered frame. A batch whose fence is still unsignalled stays in the list and is
// retried next frame, until CVarGpuReadbackMaxFrames - after which it drains blocking rather than
// leaving the caller waiting on wait_signal_ with no deadline.
void RenderRequest::drainPendingDeferredReadbacks()
{
    check(IsInRenderingThread());
    for (int32 i = g_deferred_pending.Num() - 1; i >= 0; --i) {
        RenderRequest* req = g_deferred_pending[i];
        if (req == nullptr) {
            g_deferred_pending.RemoveAt(i);
            continue;
        }
        ++req->deferred_frames_waited_;
        const bool force = req->deferred_frames_waited_ >= FMath::Max(1, CVarGpuReadbackMaxFrames.GetValueOnRenderThread());
        if (req->drainGpuReadbacks(req->deferred_stamps_, force)) {
            g_deferred_pending.RemoveAt(i);
            req->finishTask(req->deferred_stamps_);
        }
    }
}

// Shutdown counterpart of the above. The ordinary drain retries across frames and only forces the
// blocking Lock after CVarGpuReadbackMaxFrames; during teardown there are no more frames, so every
// batch drains blocking in this one pass. Each finishTask signals a caller still parked in
// getScreenshot, which is the whole point - they are what stopStreamCapture is about to join.
void RenderRequest::drainPendingDeferredReadbacksForced()
{
    check(IsInRenderingThread());
    for (int32 i = g_deferred_pending.Num() - 1; i >= 0; --i) {
        RenderRequest* req = g_deferred_pending[i];
        g_deferred_pending.RemoveAt(i);
        if (req == nullptr)
            continue;
        req->drainGpuReadbacks(req->deferred_stamps_, /*force_blocking=*/true);
        req->finishTask(req->deferred_stamps_);
    }
}

void RenderRequest::ExecuteTask()
{
    if (params_ != nullptr && req_size_ > 0) {
        // Phase 3b step 4 (F3): resample BEFORE the readback dispatch below. No-op for a batch
        // that contains no generic camera.
        executeCubeResample();

        // Readback-completion time per result, kept for the airsim.LogImageTimestamps diagnostic
        // even when airsim.BatchImageTimestamp overrides what actually reaches the response.
        TArray<msr::airlib::TTimePoint> readback_stamps;
        readback_stamps.SetNumZeroed(req_size_);

        const int32 readback_mode = CVarGpuReadback.GetValueOnRenderThread();

        // B3. Submit the copies and hand the render thread back. finishTask - and with it the
        // signal that unblocks the caller - happens in drainPendingDeferredReadbacks, a later
        // frame, once the GPU fence is signalled and Lock costs nothing.
        //
        // The CALLER still blocks: getScreenshot waits on wait_signal_ either way. What changes
        // is that the RENDER THREAD no longer spends ~11 ms (§22.1) parked on a fence inside the
        // frame it just drew, which is the cost that shows up as lost FPS.
        // Cube requests defer like any other unless airsim.GpuReadbackCube is turned off.
        // Pinhole requests carry no faces, so they pay one TArray::Num() compare.
        bool has_cube_faces = false;
        for (unsigned int i = 0; i < req_size_ && !has_cube_faces; ++i) {
            if (params_[i] != nullptr &&
                (params_[i]->face_targets.Num() > 0 || params_[i]->face_components.Num() > 0))
                has_cube_faces = true;
        }

        const bool cube_blocks_deferral =
            has_cube_faces && CVarGpuReadbackCube.GetValueOnRenderThread() == 0;

        if (readback_mode == 2 && !cube_blocks_deferral) {
            deferred_stamps_.SetNumZeroed(req_size_);
            deferred_frames_waited_ = 0;
            submitGpuReadbacks();
            ensureDeferredDrainHook();
            g_deferred_pending.Add(this);
            return;
        }

        // mode 2 on a cube batch falls through to the batched blocking path, not the legacy one:
        // it is the same submit/drain code, just drained in the same frame.
        if (readback_mode != 0)
            executeBatchedGpuReadback(readback_stamps);
        else
            executeLegacyReadback(readback_stamps);

        finishTask(readback_stamps);
    }
}

// Everything after the pixels land: per-image stamps, the timestamp diagnostic, the segment
// boundary, and the signal that releases getScreenshot. Split out of ExecuteTask so the deferred
// path can run it from a different frame than the one that submitted.
void RenderRequest::finishTask(const TArray<msr::airlib::TTimePoint>& readback_stamps)
{
    if (params_ == nullptr || req_size_ == 0)
        return;
    {
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
