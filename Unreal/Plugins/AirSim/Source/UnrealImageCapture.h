#pragma once

#include "CoreMinimal.h"
#include "PIPCamera.h"
#include "UnrealClient.h"
#include "common/ImageCaptureBase.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "RenderRequest.h"

#include <stdexcept>
#include <string>

class AIRSIM_API UnrealImageCapture : public msr::airlib::ImageCaptureBase
{
public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

    UnrealImageCapture(const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras);
    virtual ~UnrealImageCapture();

    virtual void getImages(const std::vector<ImageRequest>& requests, std::vector<ImageResponse>& responses) const override;

    /// A camera and the ABSOLUTE index of its response. Absolute, so one callback is correct both
    /// for a single vehicle and for a fleet batch whose `responses` spans several vehicles.
    struct CameraResponsePair
    {
        APIPCamera* camera = nullptr;
        size_t response_idx = 0;
    };

    /// Resolve requests into render params WITHOUT rendering, appending to the caller's vectors.
    ///
    /// ⚠ This is the ONLY place render params are built, and it must stay that way. It carries the
    /// Phase 3b generic-camera rig (cube resample mode, raymap, per-face components, the NativeGEER
    /// Scene exception, the size-match fallback). A second copy for the batch path would drift, and
    /// the symptom would be fisheye cameras silently falling back to a pinhole render in batch mode
    /// only — geometry that looks perfectly well-formed and is the wrong projection.
    ///
    /// `gameViewport` is in/out: the first camera to resolve sets it, and the fleet path reuses it
    /// across vehicles so every camera in the batch renders into one frame.
    void collectRenderParams(const std::vector<ImageRequest>& requests,
                             std::vector<ImageResponse>& responses,
                             std::vector<std::shared_ptr<RenderRequest::RenderParams>>& render_params,
                             std::vector<CameraResponsePair>& camera_response_pairs,
                             UGameViewportClient*& gameViewport,
                             bool use_safe_method) const;

    /// The declared camera model for one camera, or nullptr if that camera does not exist.
    /// ⚠ Does not throw: the publish seam runs for every image on every path, including ones where
    /// a missing camera is already being reported some other way, and an exception there would take
    /// down a capture that had otherwise succeeded.
    const msr::airlib::cameras::CameraModelParams* cameraModelFor(const std::string& camera_name) const;

    /// Horizontal FOV for one camera/image-type, or NaN. Used to derive pinhole intrinsics for a
    /// camera that declared no CameraModel block.
    double fovDegreesFor(const std::string& camera_name, int image_type) const;

    /// ⚠ GAME THREAD ONLY. Enable every capture component these requests will need, so that the
    /// later collectRenderParams finds them already on and never activates one off-thread.
    ///
    /// WHY THIS EXISTS: enabling a capture component calls UActorComponent::Activate(), which
    /// mutates UE's TICK TASK MANAGER — game-thread-only state. collectRenderParams runs on capture
    /// worker threads AND on RPC handler threads, so the first request for a not-yet-enabled
    /// (camera, type) raced the game thread and SIGSEGV'd in FTickTaskLevel::RemoveTickFunction
    /// (measured 2026-08-21). Doing it once, up front, on the right thread removes the race for the
    /// driver path.
    void ensureCameraTypesEnabled(const std::vector<ImageRequest>& requests) const;

    /// Copy one render result into one response. Returns the bytes copied, for the assembly timer.
    static size_t fillResponseFromResult(const ImageRequest& request,
                                         const RenderRequest::RenderResult& result,
                                         ImageResponse& response);

    /// Feed the `[AirSim][imgassembly]` window. Gated behind airsim.LogImageTiming.
    static void noteAssembly(double elapsed_ms, size_t images, size_t bytes);

private:
    /// Resolve a camera name or throw std::invalid_argument naming what IS available.
    /// Never use cameras_->at() directly — see the definition for why.
    APIPCamera* requireCamera(const std::string& camera_name) const;

    void getSceneCaptureImage(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
                              std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, bool use_safe_method) const;

    void addScreenCaptureHandler(UWorld* world);
    bool getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng);

    bool updateCameraVisibility(APIPCamera* camera, const msr::airlib::ImageCaptureBase::ImageRequest& request);

private:
    const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras_;
    std::vector<uint8_t> last_compressed_png_;
};
