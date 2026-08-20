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
