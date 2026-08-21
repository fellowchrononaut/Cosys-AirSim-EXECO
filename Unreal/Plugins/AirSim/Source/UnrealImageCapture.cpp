#include "UnrealImageCapture.h"
#include <limits>
#include "Engine/World.h"
#include "ImageUtils.h"

#include "RenderRequest.h"
#include "ImageTiming.h"
#include "common/ClockFactory.hpp"

namespace
{
    // ⚠ Guarded: MultiAgent runs four RPC servers whose handlers can assemble concurrently.
    FCriticalSection g_assembly_mutex;
    AirSimImageTiming::AssemblyWindow g_assembly_window;
}

UnrealImageCapture::UnrealImageCapture(const common_utils::UniqueValueMap<std::string, APIPCamera*>* cameras)
    : cameras_(cameras)
{
    //TODO: explore screenshot option
    //addScreenCaptureHandler(camera->GetWorld());
}

UnrealImageCapture::~UnrealImageCapture()
{
}

void UnrealImageCapture::getImages(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
                                   std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses) const
{
    if (cameras_->valsSize() == 0) {
        for (unsigned int i = 0; i < requests.size(); ++i) {
            responses.push_back(ImageResponse());
            responses[responses.size() - 1].message = "camera is not set";
        }
    }
    else
        getSceneCaptureImage(requests, responses, false);
}

/// Resolve a camera name, or throw something a human can act on.
///
/// ⚠ Replaces a bare cameras_->at(name). std::map::at throws std::out_of_range whose what() is
/// just "map::at: key not found" — no camera name, no list of what exists — and on the RECORDING
/// thread nothing catches it at all, so the editor aborted outright (confirmed from a core dump,
/// 2026-08-18). The same lookup is reached from simGetImages, where the message was the only clue
/// a client got and it named neither the camera asked for nor the ones available.
///
/// Naming the alternatives matters more than it looks: the usual cause is a camera that exists on
/// one vehicle and not another, and the fix is invisible until you can see both lists.
APIPCamera* UnrealImageCapture::requireCamera(const std::string& camera_name) const
{
    const auto& map = cameras_->getMap();
    const auto it = map.find(camera_name);
    if (it != map.end() && it->second != nullptr)
        return it->second;

    std::string available;
    for (const auto& p : map) {
        if (!available.empty())
            available += ", ";
        available += p.first.empty() ? "\"\" (default)" : "'" + p.first + "'";
    }
    if (available.empty())
        available = "(none - this vehicle has no cameras)";

    throw std::invalid_argument(
        "no camera named " +
        (camera_name.empty() ? std::string("\"\" (the default camera)") : "'" + camera_name + "'") +
        " on this vehicle; available: " + available);
}

const msr::airlib::cameras::CameraModelParams* UnrealImageCapture::cameraModelFor(
    const std::string& camera_name) const
{
    const auto& map = cameras_->getMap();
    const auto it = map.find(camera_name);
    if (it == map.end() || it->second == nullptr)
        return nullptr;
    return &it->second->cameraModelParams();
}

double UnrealImageCapture::fovDegreesFor(const std::string& camera_name, int image_type) const
{
    const auto& map = cameras_->getMap();
    const auto it = map.find(camera_name);
    if (it == map.end() || it->second == nullptr)
        return std::numeric_limits<double>::quiet_NaN();
    return it->second->fovDegreesFor(image_type);
}

void UnrealImageCapture::collectRenderParams(
    const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses,
    std::vector<std::shared_ptr<RenderRequest::RenderParams>>& render_params,
    std::vector<CameraResponsePair>& camera_response_pairs,
    UGameViewportClient*& gameViewport,
    bool use_safe_method) const
{
    // ⚠ EXTRACTED SO THERE IS EXACTLY ONE COPY OF THIS. It carries the whole Phase 3b generic-camera
    // rig — cube resample mode, raymap, per-face components, the NativeGEER Scene exception and the
    // size-match fallback. A fleet-batch path with its own second copy would drift from this one,
    // and the symptom would be fisheye cameras quietly falling back to a pinhole render in batch
    // mode only. Both getSceneCaptureImage and WorldSimApi::getImagesAllVehicles call this.
    bool visibilityChanged = false;
    for (unsigned int i = 0; i < requests.size(); ++i) {
        APIPCamera* camera = requireCamera(requests.at(i).camera_name);
        //TODO: may be we should have these methods non-const?
        if (requests[i].image_type == ImageType::Annotation) {
            if (camera->GetAnnotationNameExist(requests[i].annotation_name)){
                visibilityChanged = const_cast<UnrealImageCapture*>(this)->updateCameraVisibility(camera, requests[i]) || visibilityChanged;
			}
        }
        else {
            visibilityChanged = const_cast<UnrealImageCapture*>(this)->updateCameraVisibility(camera, requests[i]) || visibilityChanged;
        }
    }

    if (use_safe_method && visibilityChanged) {
        // We don't do game/render thread synchronization for safe method.
        // We just blindly sleep for 200ms (the old way)
        std::this_thread::sleep_for(std::chrono::duration<double>(0.2));
    }

    for (unsigned int i = 0; i < requests.size(); ++i) {

        APIPCamera* camera = requireCamera(requests.at(i).camera_name);

        if (gameViewport == nullptr) {
            gameViewport = camera->GetWorld()->GetGameViewport();
        }

        // ⚠ APPEND-AND-BACK, never responses.at(i). In a fleet batch this vector already holds
        // earlier vehicles' entries, so indexing by the per-vehicle request index would overwrite
        // another vehicle's response — silently, and only for the second vehicle onward.
        const size_t response_index = responses.size();
        responses.push_back(ImageResponse());
        ImageResponse& response = responses.back();
        UTextureRenderTarget2D* textureTarget = nullptr;
        USceneCaptureComponent2D* capture = nullptr;
        if (requests[i].image_type == ImageType::Annotation) {
            if (camera->GetAnnotationNameExist(requests[i].annotation_name)) {
                capture = camera->getCaptureComponent(requests[i].image_type, false, requests[i].annotation_name);
                if (capture == nullptr) {
                    response.message = "Can't take screenshot because none camera type is not active";
                }
                else if (capture->TextureTarget == nullptr) {
                    response.message = "Can't take screenshot because texture target is null";
                }
                else
                    textureTarget = capture->TextureTarget;
            }
            else {
                response.message = "Can't take screenshot because none annotation name does not exist for this camera";
            }
        }
        else {
            capture = camera->getCaptureComponent(requests[i].image_type, false, requests[i].annotation_name);
            if (capture == nullptr) {
                response.message = "Can't take screenshot because none camera type is not active";
            }
            else if (capture->TextureTarget == nullptr) {
                response.message = "Can't take screenshot because texture target is null";
            }
            else
                textureTarget = capture->TextureTarget;
        }
        
        bool disable_gamma = false;
        if (requests[i].image_type == ImageCaptureBase::ImageType::Segmentation || requests[i].image_type == ImageCaptureBase::ImageType::Annotation)disable_gamma = true;
        auto params = std::make_shared<RenderRequest::RenderParams>(capture, textureTarget, requests[i].pixels_as_float, requests[i].compress, disable_gamma);

        //Generic (non-pinhole) camera - Phase 3b step 4, extended to the other modalities in
        //step 5. Everything below is skipped by the hasCameraModel() test for every camera that
        //has no CameraModel block, which leaves params exactly the object this line used to push:
        //two empty TArrays, a null raymap and mode 0, which RenderRequest reads as "pinhole,
        //behave as before".
        //
        //Which ImageTypes go through the cube path, and how each is filtered, is
        //AirSimCubeResampleModeForImageType and nowhere else - see the table in CubeResample.cpp
        //for every row and its reason. Unsupported here means what it meant in step 4: fall
        //through to the ordinary pinhole render. It is honest rather than empty - it is what that
        //ImageType has always returned, not a silently mis-filtered fisheye - and it is why
        //optical flow, disparity and DepthVis are still on the pinhole path.
        const EAirSimCubeResampleMode resample_mode =
            AirSimCubeResampleModeForImageType(static_cast<int32>(requests[i].image_type));

        //The mode table indexes AirLib's ImageType BY VALUE, in a header that deliberately does
        //not include AirLib. Anchor both ends of that so a reordered enum is a compile error and
        //not four silently mis-filtered modalities.
        static_assert(static_cast<int>(ImageType::Scene) == 0 &&
                          static_cast<int>(ImageType::Segmentation) == 5 &&
                          static_cast<int>(ImageType::Annotation) == 11 &&
                          static_cast<int>(ImageType::Count) == 12,
                      "AirSimCubeResampleModeForImageType's table is keyed on these values");

        //NativeGEER Scene must produce an actual FSceneView for this output target so NanoGS can
        //match it and, in Gate B, shade its raymap directly. Other modalities remain on the cube
        //path; a colocated camera with RenderBackend=Cube is completely unchanged.
        const bool native_geer_scene = camera->usesNativeGeerBackend() &&
            requests[i].image_type == ImageType::Scene;
        if (capture != nullptr && textureTarget != nullptr && camera->hasCameraModel() &&
            !native_geer_scene &&
            resample_mode != EAirSimCubeResampleMode::Unsupported) {

            //The raymap is ONE RAY PER OUTPUT PIXEL, and buildRaymapResource already refuses any
            //camera whose CameraModel resolution differs from its Scene CaptureSettings - so the
            //Scene target's size IS the raymap's size. An ImageType with its own, different
            //CaptureSettings would resample the top-left CORNER of the raymap into a smaller
            //image and look entirely plausible, and since CaptureSetting defaults to 256x144 that
            //is the normal case for a settings.json that only sizes Scene. Fall back to the
            //pinhole render instead, which is exactly what an Unsupported type does.
            UTextureRenderTarget2D* scene_target = camera->getRenderTarget(ImageType::Scene, false);
            const bool size_matches = scene_target != nullptr &&
                                      textureTarget->SizeX == scene_target->SizeX &&
                                      textureTarget->SizeY == scene_target->SizeY;

            const FAirSimRaymapResourcePtr& raymap = camera->getRaymapResource();
            const int face_count = camera->getCubeFaceCount();
            if (size_matches && raymap.IsValid() && face_count > 0) {
                TArray<USceneCaptureComponent2D*> face_components;
                TArray<UTextureRenderTarget2D*> face_targets;
                face_components.Reserve(face_count);
                face_targets.Reserve(face_count);

                for (int face = 0; face < face_count; ++face) {
                    USceneCaptureComponent2D* face_capture = camera->getFaceCaptureComponent(requests[i].image_type, face);
                    UTextureRenderTarget2D* face_target = camera->getFaceRenderTarget(requests[i].image_type, face);
                    if (face_capture == nullptr || face_target == nullptr) {
                        face_components.Reset();
                        face_targets.Reset();
                        break;
                    }
                    face_components.Add(face_capture);
                    face_targets.Add(face_target);
                }

                //an incomplete rig falls back to the pinhole path rather than to half a cube
                if (face_components.Num() == face_count) {
                    params->face_components = MoveTemp(face_components);
                    params->face_targets = MoveTemp(face_targets);
                    params->raymap = raymap;
                    params->resample_mode = static_cast<int32>(resample_mode);
                }
            }
        }

        render_params.push_back(params);
        camera_response_pairs.push_back(CameraResponsePair{ camera, response_index });
    }
}

size_t UnrealImageCapture::fillResponseFromResult(
    const msr::airlib::ImageCaptureBase::ImageRequest& request,
    const RenderRequest::RenderResult& result,
    msr::airlib::ImageCaptureBase::ImageResponse& response)
{
    response.camera_name = request.camera_name;
    response.time_stamp = result.time_stamp;

    // ⚠ MEASURED, AND DELIBERATELY LEFT AS A COPY. Phase A1 timed this at ~2600 MB/s: ~16 ms of a
    // 303 ms 1080p six-image call, i.e. 7%. The 93% is rpclib's msgpack encode plus the socket.
    // Making this a move would mean changing RenderResult's buffers from TArray to std::vector, and
    // it would buy 7%. See ExecoSimArchitecture/PHASE-A1-RESULTS.md §9 before revisiting.
    const size_t bytes = static_cast<size_t>(result.image_data_uint8.Num()) +
                         static_cast<size_t>(result.image_data_float.Num()) * sizeof(float);
    response.image_data_uint8 = std::vector<uint8_t>(
        result.image_data_uint8.GetData(),
        result.image_data_uint8.GetData() + result.image_data_uint8.Num());
    response.image_data_float = std::vector<float>(
        result.image_data_float.GetData(),
        result.image_data_float.GetData() + result.image_data_float.Num());

    response.pixels_as_float = request.pixels_as_float;
    response.compress = request.compress;
    response.width = result.width;
    response.height = result.height;
    response.image_type = request.image_type;
    response.annotation_name = request.annotation_name;
    return bytes;
}

void UnrealImageCapture::noteAssembly(double elapsed_ms, size_t images, size_t bytes)
{
    const AirSimImageTiming::Clock::time_point now = AirSimImageTiming::Clock::now();
    const double period = static_cast<double>(AirSimImageTiming::ReportPeriodSeconds());
    if (period <= 0.0) return;

    bool report = false;
    AirSimImageTiming::AssemblyWindow snapshot;
    {
        FScopeLock lock(&g_assembly_mutex);
        g_assembly_window.note(elapsed_ms, images, bytes, now);
        if (g_assembly_window.shouldReport(now, period)) {
            snapshot = g_assembly_window;
            g_assembly_window.reset();
            report = true;
        }
    }
    if (report) {
        const double n = static_cast<double>(snapshot.calls);
        const double mb = static_cast<double>(snapshot.bytes) / 1e6;
        UE_LOG(LogTemp, Log,
               TEXT("[AirSim][imgassembly] %llu calls, %.2f img/call | copy %.2f ms avg "
                    "(max %.2f) | %.1f MB total, %.1f MB/call | %.0f MB/s while copying"),
               snapshot.calls, snapshot.images / n, snapshot.ms / n, snapshot.max_ms,
               mb, mb / n, snapshot.ms > 0.0 ? mb / (snapshot.ms / 1000.0) : 0.0);
    }
}

void UnrealImageCapture::getSceneCaptureImage(const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests,
                                              std::vector<msr::airlib::ImageCaptureBase::ImageResponse>& responses, bool use_safe_method) const
{
    std::vector<std::shared_ptr<RenderRequest::RenderParams>> render_params;
    std::vector<std::shared_ptr<RenderRequest::RenderResult>> render_results;
    std::vector<CameraResponsePair> camera_response_pairs;
    UGameViewportClient* gameViewport = nullptr;

    collectRenderParams(requests, responses, render_params, camera_response_pairs,
                        gameViewport, use_safe_method);

    if (nullptr == gameViewport) {
        return;
    }

    // ⚠ Poses come from camera_response_pairs, not from re-resolving names against `requests`.
    // The pair list carries an absolute index into `responses`, which is what makes the identical
    // callback correct for a single vehicle and for a fleet batch spanning several.
    auto query_camera_pose_cb = [&camera_response_pairs, &responses]() {
        for (const CameraResponsePair& pair : camera_response_pairs) {
            if (pair.camera == nullptr || pair.response_idx >= responses.size()) continue;
            const auto camera_pose = pair.camera->getPose();
            responses[pair.response_idx].camera_position = camera_pose.position;
            responses[pair.response_idx].camera_orientation = camera_pose.orientation;
        }
    };
    RenderRequest render_request{ gameViewport, std::move(query_camera_pose_cb) };

    render_request.getScreenshot(render_params.data(), render_results, render_params.size(), use_safe_method);

    const bool timing_on = AirSimImageTiming::ReportPeriodSeconds() > 0;
    const AirSimImageTiming::Clock::time_point t_start =
        timing_on ? AirSimImageTiming::Clock::now() : AirSimImageTiming::Clock::time_point{};
    size_t assembled_bytes = 0;

    for (unsigned int i = 0; i < requests.size() && i < render_results.size(); ++i) {
        assembled_bytes += fillResponseFromResult(requests.at(i), *render_results[i], responses.at(i));

        if (use_safe_method) {
            // Currently, we don't have a way to synchronize image capturing and camera pose when
            // the safe method is used.
            APIPCamera* camera = requireCamera(requests.at(i).camera_name);
            const msr::airlib::Pose pose = camera->getPose();
            responses.at(i).camera_position = pose.position;
            responses.at(i).camera_orientation = pose.orientation;
        }
    }

    if (timing_on)
        noteAssembly(AirSimImageTiming::ToMs(AirSimImageTiming::Clock::now() - t_start),
                     requests.size(), assembled_bytes);
}

void UnrealImageCapture::ensureCameraTypesEnabled(
    const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& requests) const
{
    check(IsInGameThread());
    for (const auto& request : requests) {
        const auto& map = cameras_->getMap();
        const auto it = map.find(request.camera_name);
        if (it == map.end() || it->second == nullptr)
            continue;                       //unknown camera: reported elsewhere, not fatal here
        APIPCamera* camera = it->second;
        if (request.image_type == ImageType::Annotation &&
            !camera->GetAnnotationNameExist(request.annotation_name))
            continue;
        if (!camera->getCameraTypeEnabled(request.image_type, request.annotation_name))
            camera->setCameraTypeEnabled(request.image_type, true, request.annotation_name);
    }
}

/// ⚠ CALLED FROM NON-GAME THREADS (capture workers, RPC handlers). setCameraTypeEnabled ends in
/// UActorComponent::Activate(), which mutates the tick task manager and is game-thread-only — so
/// reaching that branch off-thread is a race that SIGSEGV'd the editor on 2026-08-21. The driver
/// path now pre-enables on the game thread (see ensureCameraTypesEnabled) so this early-outs.
/// ⚠ THE RPC PATH IS STILL EXPOSED: a simGetImages for a (camera, type) never requested before can
/// still reach the Activate branch from an RPC thread. Narrow, pre-existing, and not fixed here.
bool UnrealImageCapture::updateCameraVisibility(APIPCamera* camera, const msr::airlib::ImageCaptureBase::ImageRequest& request)
{
    bool visibilityChanged = false;
    if (!camera->getCameraTypeEnabled(request.image_type, request.annotation_name)) {
        camera->setCameraTypeEnabled(request.image_type, true, request.annotation_name);
        visibilityChanged = true;
    }

    return visibilityChanged;
}

bool UnrealImageCapture::getScreenshotScreen(ImageType image_type, std::vector<uint8_t>& compressedPng)
{
    FScreenshotRequest::RequestScreenshot(false); // This is an async operation
    return true;
}

void UnrealImageCapture::addScreenCaptureHandler(UWorld* world)
{
    static bool is_installed = false;

    if (!is_installed) {
        UGameViewportClient* ViewportClient = world->GetGameViewport();
        ViewportClient->OnScreenshotCaptured().Clear();
        ViewportClient->OnScreenshotCaptured().AddLambda(
            [this](int32 SizeX, int32 SizeY, const TArray<FColor>& Bitmap) {
                // Make sure that all alpha values are opaque.
                TArray<FColor>& RefBitmap = const_cast<TArray<FColor>&>(Bitmap);
                for (auto& Color : RefBitmap)
                    Color.A = 255;

                TArray<uint8_t> last_compressed_png;
                FImageUtils::CompressImageArray(SizeX, SizeY, RefBitmap, last_compressed_png);
                last_compressed_png_ = std::vector<uint8_t>(last_compressed_png.GetData(), last_compressed_png.GetData() + last_compressed_png.Num());
            });

        is_installed = true;
    }
}
