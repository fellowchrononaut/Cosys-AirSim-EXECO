#include "PIPCamera.h"
#include "UObject/ConstructorHelpers.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Camera/CameraComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "ImageUtils.h"
#include "Annotation/AnnotationComponent.h"
#include "Annotation/ObjectAnnotator.h"
#include <string>
#include <exception>
#include "AirBlueprintLib.h"
#include "HAL/IConsoleManager.h"
#include "EngineUtils.h"
#include "Engine/Engine.h"
#include "cameras/Raymap.hpp"

/** I-G diagnostic and candidate fix.
 *
 *  A 1-image simGetImages call costs ~215 ms of render+readback on a 6-vehicle / 24-camera rig,
 *  while `b wait-draw` stays at ~6 ms. Small b with huge c+d means the GAME thread is not blocked
 *  but the RENDER thread is deep - the signature of per-frame capture work piling up in the render
 *  queue, with our readback command waiting behind it.
 *
 *  setCaptureUpdate() sets bCaptureEveryFrame = !nodisplay, so every capture component that has
 *  ever been enabled keeps re-rendering the scene each frame. simGetImages does NOT need that: it
 *  issues its own CaptureSceneDeferred() for exactly the components requested.
 *
 *  The NoDisplay view mode ('-') would clear the flag, but it also disables viewport world
 *  rendering, which confounds the measurement - and the key is relocated on non-QWERTY layouts
 *  anyway (see ui_issues U-10). This CVar isolates the flag alone.
 *
 *   -1 = leave alone (default, current behaviour)
 *    0 = force OFF on every capture component of every camera
 *    1 = force ON
 *
 *  ⚠ Turning it off makes SubWindows and anything else relying on continuous capture go stale.
 *  The main viewport is unaffected: it renders through the UCineCameraComponent, not a capture. */
static TAutoConsoleVariable<int32> CVarCaptureEveryFrame(
    TEXT("airsim.CaptureEveryFrame"),
    -1,
    TEXT("I-G: force bCaptureEveryFrame on all camera capture components.\n")
    TEXT(" -1: leave alone (default)\n")
    TEXT("  0: force off - simGetImages still works, it captures on demand\n")
    TEXT("  1: force on"),
    ECVF_Default);

namespace
{
    // Applies the CVar to every APIPCamera in the running game world. Registered as an on-changed
    // callback so it takes effect the moment the value is set from the console.
    void ApplyCaptureEveryFrameCVar(IConsoleVariable* var)
    {
        const int32 value = var->GetInt();
        if (value < 0 || GEngine == nullptr)
            return;

        UWorld* world = nullptr;
        for (const FWorldContext& ctx : GEngine->GetWorldContexts()) {
            if (ctx.World() != nullptr &&
                (ctx.WorldType == EWorldType::Game || ctx.WorldType == EWorldType::PIE)) {
                world = ctx.World();
                break;
            }
        }
        if (world == nullptr)
            return;

        int32 cameras = 0;
        for (TActorIterator<APIPCamera> it(world); it; ++it) {
            it->setAllCapturesEveryFrame(value != 0);
            ++cameras;
        }
        UE_LOG(LogTemp, Log, TEXT("[AirSim] airsim.CaptureEveryFrame=%d applied to %d cameras"),
               value, cameras);
    }

    struct FCaptureEveryFrameCVarBinder
    {
        FCaptureEveryFrameCVarBinder()
        {
            CVarCaptureEveryFrame.AsVariable()->SetOnChangedCallback(
                FConsoleVariableDelegate::CreateStatic(&ApplyCaptureEveryFrameCVar));
        }
    };
    FCaptureEveryFrameCVarBinder g_capture_every_frame_binder;
}

void APIPCamera::setAllCapturesEveryFrame(bool enabled)
{
    for (USceneCaptureComponent2D* capture : captures_) {
        if (capture != nullptr) {
            capture->bCaptureEveryFrame = enabled;
            capture->bCaptureOnMovement = enabled;
        }
    }
}

//CinemAirSim
APIPCamera::APIPCamera(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer
                .SetDefaultSubobjectClass<UCineCameraComponent>(TEXT("CameraComponent")))
{
    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder(TEXT("Material'/AirSim/HUDAssets/CameraSensorNoise.CameraSensorNoise'"));
    if (mat_finder.Succeeded()) {
        noise_material_static_ = mat_finder.Object;
    }
    else
        UAirBlueprintLib::LogMessageString("Cannot create noise material for the PIPCamera",
                                           "",
                                           LogDebugLevel::Failure);

    static ConstructorHelpers::FObjectFinder<UMaterial> dist_mat_finder(TEXT("Material'/AirSim/HUDAssets/CameraDistortion.CameraDistortion'"));
    if (dist_mat_finder.Succeeded()) {
        distortion_material_static_ = dist_mat_finder.Object;
        distortion_param_collection_ = Cast<UMaterialParameterCollection>(StaticLoadObject(UMaterialParameterCollection::StaticClass(), NULL, TEXT("'/AirSim/HUDAssets/DistortionParams.DistortionParams'")));
    }
    else{
        UAirBlueprintLib::LogMessageString("Cannot create distortion material for the PIPCamera",
                                           "", LogDebugLevel::Failure);
    }

	static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder2(TEXT("Material'/AirSim/HUDAssets/CameraSensorLensDistortion.CameraSensorLensDistortion'"));
	if (mat_finder2.Succeeded())
	{
		lens_distortion_material_static_ = mat_finder2.Object;
	}
	else{
		UAirBlueprintLib::LogMessageString("Cannot create lens distortion material for the PIPCamera", "", LogDebugLevel::Failure);
    }

	static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder3(TEXT("Material'/AirSim/HUDAssets/CameraSensorLensDistortionInvert.CameraSensorLensDistortionInvert'"));
	if (mat_finder3.Succeeded())
	{
		lens_distortion_invert_material_static_ = mat_finder3.Object;
	}
	else
		UAirBlueprintLib::LogMessageString("Cannot create inverted lens distortion material for the PIPCamera",
			"", LogDebugLevel::Failure);

    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder4(TEXT("Material'/AirSim/HUDAssets/CameraSensorMotionBlur.CameraSensorMotionBlur'"));
    if (mat_finder4.Succeeded())
    {
        motion_blur_material_static_ = mat_finder4.Object;
    }
    else
        UAirBlueprintLib::LogMessageString("Cannot create fake motion blur material for the PIPCamera",
            "", LogDebugLevel::Failure);

    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder5(TEXT("Material'/AirSim/HUDAssets/CameraSensorRadialBlur.CameraSensorRadialBlur'"));
    if (mat_finder5.Succeeded())
    {
        radial_blur_material_static_ = mat_finder5.Object;
    }
    else
        UAirBlueprintLib::LogMessageString("Cannot create radial blur material for the PIPCamera",
            "", LogDebugLevel::Failure);

    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder6(TEXT("Material'/AirSim/HUDAssets/CameraSensorGuassianBlur.CameraSensorGuassianBlur'"));
    if (mat_finder6.Succeeded())
    {
        guassian_blur_material_static_ = mat_finder6.Object;
    }
    else
        UAirBlueprintLib::LogMessageString("Cannot create guassian blur material for the PIPCamera",
            "", LogDebugLevel::Failure);


    PrimaryActorTick.bCanEverTick = true;

    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Scene), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DepthPlanar), EPixelFormat::PF_DepthStencil); // not used. init_auto_format is called in setupCameraFromSettings()
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DepthPerspective), EPixelFormat::PF_DepthStencil); // not used for same reason as above
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DepthVis), EPixelFormat::PF_DepthStencil); // not used for same reason as above
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::DisparityNormalized), EPixelFormat::PF_DepthStencil); // not used for same reason as above
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Segmentation), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::SurfaceNormals), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Infrared), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::OpticalFlow), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::OpticalFlowVis), EPixelFormat::PF_B8G8R8A8);
    image_type_to_pixel_format_map_.Add(Utils::toNumeric(ImageType::Lighting), EPixelFormat::PF_B8G8R8A8);

    object_filter_ = FObjectFilter();

    static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedMesh(TEXT("StaticMesh'/AirSim/Models/AnnotationSphere.AnnotationSphere'"));
    if (loadedMesh.Succeeded())
    {
        annotation_sphere_static_ = loadedMesh.Object;
    }
}

void APIPCamera::PostInitializeComponents()
{
    Super::PostInitializeComponents();

    //CinemAirSim
    camera_ = UAirBlueprintLib::GetActorComponent<UCineCameraComponent>(this, TEXT("CameraComponent"));
    captures_.Init(nullptr, imageTypeCount());
    render_targets_.Init(nullptr, imageTypeCount());
    detections_.Init(nullptr, imageTypeCount());

    captures_[Utils::toNumeric(ImageType::Scene)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SceneCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthPlanar)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthPlanarCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthPerspective)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthPerspectiveCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthVis)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthVisCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DisparityNormalized)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DisparityNormalizedCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Segmentation)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SegmentationCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Infrared)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("InfraredCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::SurfaceNormals)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("NormalsCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::OpticalFlow)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("OpticalFlowCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::OpticalFlowVis)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("OpticalFlowVisCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Lighting)] =
        UAirBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("LightingCaptureComponent"));

    for (unsigned int i = 0; i < imageTypeCount(); ++i) {
        detections_[i] = NewObject<UDetectionComponent>(this);
        if (detections_[i]) {
            detections_[i]->SetupAttachment(captures_[i]);
            detections_[i]->RegisterComponent();
            detections_[i]->Deactivate();
        }
    }
    //set initial focal length
    camera_->CurrentFocalLength = 11.9;

    FObjectAnnotator::SetViewForAnnotationRender(captures_[Utils::toNumeric(ImageType::Segmentation)]->ShowFlags);
    captures_[Utils::toNumeric(ImageType::Segmentation)]->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;

    captures_[Utils::toNumeric(ImageType::Lighting)]->ShowFlags.SetLighting(true);
    captures_[Utils::toNumeric(ImageType::Lighting)]->ShowFlags.SetMaterials(false);
    captures_[Utils::toNumeric(ImageType::Lighting)]->ShowFlags.SetPostProcessing(false);
}

void APIPCamera::BeginPlay()
{
    Super::BeginPlay();

    noise_materials_.AddZeroed(imageTypeCount() + 1);
    distortion_materials_.AddZeroed(imageTypeCount() + 1);
	lens_distortion_materials_.AddZeroed(imageTypeCount() + 1);
    fake_motion_blur_materials_.AddZeroed(imageTypeCount() + 1);
    radial_blur_materials_.AddZeroed(imageTypeCount() + 1);
    guassian_blur_materials_.AddZeroed(imageTypeCount() + 1);

    //by default all image types are disabled
    camera_type_enabled_.assign(imageTypeCount(), false);

    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        //use final color for all calculations
        if(image_type == Utils::toNumeric(ImageType::Scene) || image_type == Utils::toNumeric(ImageType::Lighting)) {
            captures_[image_type]->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
        }
        else {
            captures_[image_type]->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        }      
        render_targets_[image_type] = NewObject<UTextureRenderTarget2D>();
    }

    //We set all cameras to start as nodisplay
    //This improves performance because the capture components are no longer updating every frame and only update while requesting an image
    onViewModeChanged(true);

    gimbal_stabilization_ = 0;
    gimbald_rotator_ = this->GetActorRotation();
    this->SetActorTickEnabled(false);
    

    if (distortion_param_collection_)
        distortion_param_instance_ = this->GetWorld()->GetParameterCollectionInstance(distortion_param_collection_);
}


msr::airlib::AirSimSettings::CameraSetting APIPCamera::getParams() const
{
    return sensor_params_;
}

msr::airlib::ProjectionMatrix APIPCamera::getProjectionMatrix() const
{
    msr::airlib::ProjectionMatrix mat;

    // TODO: This is always the case in current request, might need to change to include annotation if needed
	ImageType image_type = ImageType::Scene;

    //TODO: avoid the need to override const cast here
    const_cast<APIPCamera*>(this)->setCameraTypeEnabled(image_type, true);
    const USceneCaptureComponent2D* capture = const_cast<APIPCamera*>(this)->getCaptureComponent(image_type, false);
    if (capture) {
        FMatrix proj_mat_transpose;

        FIntPoint render_target_size(capture->TextureTarget->GetSurfaceWidth(), capture->TextureTarget->GetSurfaceHeight());
        float x_axis_multiplier = 1.0f;
        float y_axis_multiplier = render_target_size.X / (float)render_target_size.Y;

        if (render_target_size.X < render_target_size.Y) {
            // if the viewport is taller than it is wide
            x_axis_multiplier = render_target_size.Y / static_cast<float>(render_target_size.X);
            y_axis_multiplier = 1.0f;
        }

        if (capture->ProjectionType == ECameraProjectionMode::Orthographic) {
            check((int32)ERHIZBuffer::IsInverted);
            const float OrthoWidth = capture->OrthoWidth / 2.0f;
            const float OrthoHeight = capture->OrthoWidth / 2.0f * x_axis_multiplier / y_axis_multiplier;

            const float NearPlane = 0;
            const float FarPlane = WORLD_MAX / 8.0f;

            const float ZScale = 1.0f / (FarPlane - NearPlane);
            const float ZOffset = -NearPlane;

            proj_mat_transpose = FReversedZOrthoMatrix(
                OrthoWidth,
                OrthoHeight,
                ZScale,
                ZOffset);
        }
        else {
            float halfFov = Utils::degreesToRadians(capture->FOVAngle) / 2;
            if ((int32)ERHIZBuffer::IsInverted) {
                proj_mat_transpose = FReversedZPerspectiveMatrix(
                    halfFov,
                    halfFov,
                    x_axis_multiplier,
                    y_axis_multiplier,
                    GNearClippingPlane,
                    GNearClippingPlane);
            }
            else {
                //The FPerspectiveMatrix() constructor actually returns the transpose of the perspective matrix.
                proj_mat_transpose = FPerspectiveMatrix(
                    halfFov,
                    halfFov,
                    x_axis_multiplier,
                    y_axis_multiplier,
                    GNearClippingPlane,
                    GNearClippingPlane);
            }
        }

        //Takes a vector from NORTH-EAST-DOWN coordinates (AirSim) to EAST-UP-SOUTH coordinates (Unreal). Leaves W coordinate unchanged.
        FMatrix coordinateChangeTranspose = FMatrix(
            FPlane(0, 0, -1, 0),
            FPlane(1, 0, 0, 0),
            FPlane(0, -1, 0, 0),
            FPlane(0, 0, 0, 1));

        FMatrix projMatTransposeInAirSim = coordinateChangeTranspose * proj_mat_transpose;

        //Copy the result to an airlib::ProjectionMatrix while taking transpose.
        for (auto row = 0; row < 4; ++row)
            for (auto col = 0; col < 4; ++col)
                mat.matrix[col][row] = projMatTransposeInAirSim.M[row][col];
    }
    else
        mat.setTo(Utils::nan<float>());

    return mat;
}

void APIPCamera::Tick(float DeltaTime)
{
    if (gimbal_stabilization_ > 0) {
        FRotator rotator = this->GetActorRotation();
        if (!std::isnan(gimbald_rotator_.Pitch))
            rotator.Pitch = gimbald_rotator_.Pitch * gimbal_stabilization_ +
                            rotator.Pitch * (1 - gimbal_stabilization_);
        if (!std::isnan(gimbald_rotator_.Roll))
            rotator.Roll = gimbald_rotator_.Roll * gimbal_stabilization_ +
                           rotator.Roll * (1 - gimbal_stabilization_);
        if (!std::isnan(gimbald_rotator_.Yaw))
            rotator.Yaw = gimbald_rotator_.Yaw * gimbal_stabilization_ +
                          rotator.Yaw * (1 - gimbal_stabilization_);

        this->SetActorRotation(rotator);
    }
    if (sensor_params_.draw_sensor) {
        UAirBlueprintLib::DrawPoint(this->GetWorld(), this->GetActorTransform().GetLocation(), 5, FColor::Black, false, 0.3);
        UAirBlueprintLib::DrawCoordinateSystem(this->GetWorld(), this->GetActorLocation(), this->GetActorRotation(), 25, false, 0.3, 10);
    }
}

void APIPCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    int image_count_to_delete = static_cast<int>(Utils::toNumeric(ImageType::Count));
    if (noise_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (noise_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(noise_materials_[image_type + 1]);
        }
        if (noise_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(noise_materials_[0]);
    }

	if (lens_distortion_materials_.Num()) {
		for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
			if (lens_distortion_materials_[image_type + 1])
				captures_[image_type]->PostProcessSettings.RemoveBlendable(lens_distortion_materials_[image_type + 1]);
		}
		if (lens_distortion_materials_[0])
			camera_->PostProcessSettings.RemoveBlendable(lens_distortion_materials_[0]);
	}

    if (radial_blur_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (radial_blur_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(radial_blur_materials_[image_type + 1]);
        }
        if (radial_blur_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(radial_blur_materials_[0]);
    }

    if (guassian_blur_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (guassian_blur_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(guassian_blur_materials_[image_type + 1]);
        }
        if (guassian_blur_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(guassian_blur_materials_[0]);
    }

    if (fake_motion_blur_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (fake_motion_blur_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(fake_motion_blur_materials_[image_type + 1]);
        }
        if (fake_motion_blur_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(fake_motion_blur_materials_[0]);
    }

    noise_material_static_ = nullptr;
	lens_distortion_material_static_ = nullptr;
	lens_distortion_invert_material_static_ = nullptr;
    annotation_sphere_static_ = nullptr;
    guassian_blur_material_static_ = nullptr;
    radial_blur_material_static_ = nullptr;
    motion_blur_material_static_ = nullptr;
    noise_materials_.Empty();
	lens_distortion_materials_.Empty();
    radial_blur_materials_.Empty();
    guassian_blur_materials_.Empty();
    fake_motion_blur_materials_.Empty();

    if (distortion_materials_.Num()) {
        for (int image_type = 0; image_type < image_count_to_delete - 3; ++image_type) {
            if (distortion_materials_[image_type + 1])
            {
                if (captures_[image_type] != NULL) {
                    captures_[image_type]->PostProcessSettings.RemoveBlendable(distortion_materials_[image_type + 1]);
                }
            }                
        }
        if (distortion_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(distortion_materials_[0]);
    }

    distortion_material_static_ = nullptr;
    distortion_materials_.Empty();

	annotator_name_to_index_map_.Empty();
    sphere_annotation_component_map_.Empty();


    int camera_full_count = static_cast<int>(cameraCaptureCount());
    for (int current_camera = 0; current_camera < camera_full_count; ++current_camera) {
        //use final color for all calculations
        if (current_camera == Utils::toNumeric(ImageType::Segmentation) || current_camera >= image_count_to_delete - 2) {
            captures_[current_camera]->ShowOnlyComponents.Empty();
        }        
        captures_[current_camera] = nullptr;
        render_targets_[current_camera] = nullptr;
        detections_[current_camera] = nullptr;
    }
    captures_.Empty();
	render_targets_.Empty();
	detections_.Empty();

    //empty already for every camera without a CameraModel block, so this loop does not run
    for (int32 face_index = 0; face_index < face_captures_.Num(); ++face_index) {
        face_captures_[face_index] = nullptr;
        face_render_targets_[face_index] = nullptr;
    }
    face_captures_.Empty();
    face_render_targets_.Empty();

    //null already for every camera without a CameraModel block, so this returns immediately
    AirSimReleaseRaymap(raymap_);

    Super::EndPlay(EndPlayReason);
}

unsigned int APIPCamera::imageTypeCount()
{
    return Utils::toNumeric(ImageType::Count) - 1;
}

unsigned int APIPCamera::cameraCaptureCount()
{
    return static_cast<unsigned int>(captures_.Num());
}

void APIPCamera::showToScreen()
{
    camera_->SetVisibility(true);
    camera_->Activate();
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    controller->SetViewTarget(this);
    UAirBlueprintLib::LogMessage(TEXT("Camera: "), GetName(), LogDebugLevel::Informational);
}

void APIPCamera::disableAll()
{
    disableMain();
    disableAllPIP();
}

bool APIPCamera::getCameraTypeEnabled(ImageType type, std::string annotation_name) const
{
    if (type == ImageType::Annotation) {
        return camera_type_enabled_[annotator_name_to_index_map_[FString(annotation_name.c_str())]];
    }
    else {
        return camera_type_enabled_[Utils::toNumeric(type)];
    }    
}

bool APIPCamera::GetAnnotationNameExist(std::string annotation_name)
{
    if (annotator_name_to_index_map_.Contains(FString(annotation_name.c_str())))
        return true;
    else
        return false;
}

void APIPCamera::setCameraTypeEnabled(ImageType type, bool enabled, std::string annotation_name)
{
    enableCaptureComponent(type, enabled, annotation_name);
}

void APIPCamera::setCameraOrientation(const FRotator& rotator)
{
    if (gimbal_stabilization_ > 0) {
        gimbald_rotator_.Pitch = rotator.Pitch;
        gimbald_rotator_.Roll = rotator.Roll;
        gimbald_rotator_.Yaw = rotator.Yaw;
    }
    this->SetActorRelativeRotation(rotator);
}


void APIPCamera::setCaptureUpdate(USceneCaptureComponent2D* capture, bool nodisplay)
{
    capture->bCaptureEveryFrame = !nodisplay;
    capture->bCaptureOnMovement = !nodisplay;
    capture->bAlwaysPersistRenderingState = true;
}

void APIPCamera::setCameraTypeUpdate(ImageType type, bool nodisplay, std::string annotation_name)
{
    USceneCaptureComponent2D* capture = getCaptureComponent(type, false, annotation_name);
    if (capture != nullptr)
        setCaptureUpdate(capture, nodisplay);
}

void APIPCamera::setCameraPose(const msr::airlib::Pose& relative_pose)
{
    FTransform pose = ned_transform_->fromRelativeNed(relative_pose);

    FVector position = pose.GetLocation();
    this->SetActorRelativeLocation(pose.GetLocation());

    FRotator rotator = pose.GetRotation().Rotator();
    if (gimbal_stabilization_ > 0) {
        gimbald_rotator_.Pitch = rotator.Pitch;
        gimbald_rotator_.Roll = rotator.Roll;
        gimbald_rotator_.Yaw = rotator.Yaw;
    }
    else {
        this->SetActorRelativeRotation(rotator);
    }
}

void APIPCamera::setCameraFoV(float fov_degrees)
{
    for (unsigned int image_type = 0; image_type < cameraCaptureCount(); ++image_type) {
        captures_[image_type]->FOVAngle = fov_degrees;
    }
    camera_->SetFieldOfView(fov_degrees);
}

msr::airlib::CameraInfo APIPCamera::getCameraInfo() const
{
    msr::airlib::CameraInfo camera_info;

    camera_info.pose.position = ned_transform_->toLocalNed(this->GetActorLocation());
    camera_info.pose.orientation = ned_transform_->toNed(this->GetActorRotation().Quaternion());
    camera_info.fov = camera_->FieldOfView;
    camera_info.proj_mat = getProjectionMatrix();
    return camera_info;
}

std::vector<float> APIPCamera::getDistortionParams() const
{
    std::vector<float> param_values(5, 0.0);

    auto getParamValue = [this](const auto& name, float& val) {
        distortion_param_instance_->GetScalarParameterValue(FName(name), val);
    };

    getParamValue(TEXT("K1"), param_values[0]);
    getParamValue(TEXT("K2"), param_values[1]);
    getParamValue(TEXT("K3"), param_values[2]);
    getParamValue(TEXT("P1"), param_values[3]);
    getParamValue(TEXT("P2"), param_values[4]);

    return param_values;
}

void APIPCamera::setDistortionParam(const std::string& param_name, float value)
{
    distortion_param_instance_->SetScalarParameterValue(FName(param_name.c_str()), value);
}

void APIPCamera::updateInstanceSegmentationAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList, bool only_hide) {
    if(!only_hide)
        captures_[Utils::toNumeric(ImageType::Segmentation)]->ShowOnlyComponents = ComponentList;
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    for(TWeakObjectPtr<UPrimitiveComponent> component : ComponentList) {
        captures_[Utils::toNumeric(ImageType::Scene)]->HiddenComponents.AddUnique(component);
        captures_[Utils::toNumeric(ImageType::Lighting)]->HiddenComponents.AddUnique(component);
        controller->HiddenPrimitiveComponents.AddUnique(component);
	}
}

void APIPCamera::updateAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList, FString annotation_name, bool only_hide) {
    if (!only_hide) {
        captures_[annotator_name_to_index_map_[annotation_name]]->ShowOnlyComponents = ComponentList;
        if (sphere_annotation_component_map_.Contains(annotation_name))
            captures_[annotator_name_to_index_map_[annotation_name]]->ShowOnlyComponents.Add(sphere_annotation_component_map_[annotation_name]);
    }   
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();

    for (TWeakObjectPtr<UPrimitiveComponent> component : ComponentList) {
        captures_[Utils::toNumeric(ImageType::Scene)]->HiddenComponents.AddUnique(component);
        captures_[Utils::toNumeric(ImageType::Lighting)]->HiddenComponents.AddUnique(component);
        controller->HiddenPrimitiveComponents.AddUnique(component);
    }
}

void APIPCamera::addAnnotationCamera(FString name, FObjectAnnotator::AnnotatorType type, float max_view_distance)
{
    USceneCaptureComponent2D* new_capture = NewObject<USceneCaptureComponent2D>(this, USceneCaptureComponent2D ::StaticClass(), *name);

    new_capture->bAutoActivate = false;
    new_capture->bCaptureEveryFrame = true;
    new_capture->bCaptureOnMovement = true;
    new_capture->SetRelativeRotation(FRotator(0, 0, 0));
    new_capture->SetRelativeLocation(FVector(0, 0, 0));
    new_capture->AttachToComponent(this->RootComponent, FAttachmentTransformRules::KeepRelativeTransform);
	new_capture->RegisterComponent();
    new_capture->Deactivate();

    if (max_view_distance > 0) {
        FString sphereName  = name + "_hidden_sphere";
        UStaticMeshComponent* annotation_sphere = NewObject<UStaticMeshComponent>(this, FName(*sphereName));
        annotation_sphere->SetupAttachment(RootComponent);
        annotation_sphere->RegisterComponent();
        annotation_sphere->SetStaticMesh(annotation_sphere_static_);
        annotation_sphere->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        annotation_sphere->SetRelativeScale3D(FVector(max_view_distance * 2, max_view_distance * 2, max_view_distance * 2));

        FString annotatedSphereName = name + "_annotation_sphere";
        UAnnotationComponent* AnnotationComponent = NewObject<UAnnotationComponent>(annotation_sphere, FName(*annotatedSphereName));
        AnnotationComponent->SetupAttachment(annotation_sphere);
        AnnotationComponent->RegisterComponent();
        AnnotationComponent->MarkRenderStateDirty();
        UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(AnnotationComponent);

        sphere_annotation_component_map_.Add(name, PrimitiveComponent);
	}

    captures_.Add(new_capture);

    render_targets_.Add(NewObject<UTextureRenderTarget2D>());
    int render_index = render_targets_.Num() - 1;
    if (type == FObjectAnnotator::AnnotatorType::RGB || type == FObjectAnnotator::AnnotatorType::InstanceSegmentation) {
        render_targets_[render_index]->TargetGamma = 1;
    }

    FObjectAnnotator::SetViewForAnnotationRender(captures_[render_index]->ShowFlags);
    captures_[render_index]->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;

    camera_type_enabled_.push_back(false);   

    annotator_name_to_index_map_.Add(TCHAR_TO_UTF8(*name), render_index);

    detections_.Add(NewObject<UDetectionComponent>(this));
    if (detections_[render_index]) {
        detections_[render_index]->SetupAttachment(captures_[render_index]);
        detections_[render_index]->RegisterComponent();
        detections_[render_index]->Deactivate();
    }

    captures_[render_index]->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

    setCaptureUpdate(captures_[render_index], true);    

    if (sensor_params_.capture_settings.at(Utils::toNumeric(ImageType::Annotation)).ignore_marked)captures_[render_index]->HiddenActors.Append(ignore_actors_);
    
    captures_[render_index]->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;

    updateCaptureComponentSetting(captures_[render_index], render_targets_[render_index],
        false, EPixelFormat::PF_B8G8R8A8,
        sensor_params_.capture_settings.at(Utils::toNumeric(ImageType::Annotation)),
        *ned_transform_, false);

    copyCameraSettingsToSceneCapture(camera_, captures_[render_index]);	
}

void APIPCamera::setupCameraFromSettings(const APIPCamera::CameraSetting& camera_setting, const NedTransform& ned_transform)
{
    //TODO: should we be ignoring position and orientation settings here?

    //TODO: can we eliminate storing NedTransform?

    ned_transform_ = &ned_transform;

    sensor_params_ = camera_setting;

    gimbal_stabilization_ = Utils::clip(camera_setting.gimbal.stabilization, 0.0f, 1.0f);
    if (gimbal_stabilization_ > 0) {
        this->SetActorTickEnabled(true);
        gimbald_rotator_.Pitch = camera_setting.gimbal.rotation.pitch;
        gimbald_rotator_.Roll = camera_setting.gimbal.rotation.roll;
        gimbald_rotator_.Yaw = camera_setting.gimbal.rotation.yaw;
    }
    else
        this->SetActorTickEnabled(false);

    if (sensor_params_.draw_sensor) {
        this->SetActorTickEnabled(true);
    }

    if (sensor_params_.external) {
        this->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
    }

	static const FName lidar_ignore_tag = TEXT("MarkedIgnore");
    for (TActorIterator<AActor> ActorIterator(this->GetWorld()); ActorIterator; ++ActorIterator)
    {
        AActor* Actor = *ActorIterator;
        if (Actor && Actor != this && Actor->Tags.Contains(lidar_ignore_tag))ignore_actors_.Add(Actor);
    }


    int image_count = static_cast<int>(Utils::toNumeric(ImageType::Count));
    for (int image_type = -1; image_type < image_count - 1; ++image_type) {
        const auto& capture_setting = camera_setting.capture_settings.at(image_type);
        const auto& noise_setting = camera_setting.noise_settings.at(image_type);

        if (image_type >= 0) { //scene capture components
            auto pixel_format_override = camera_setting.ue_setting.pixel_format_override_settings.find(image_type);
            EPixelFormat pixel_format = EPixelFormat::PF_Unknown;
            if (pixel_format_override != camera_setting.ue_setting.pixel_format_override_settings.end()) {
                pixel_format = static_cast<EPixelFormat>(pixel_format_override->second.pixel_format);
            }
            pixel_format = (pixel_format == EPixelFormat::PF_Unknown ? image_type_to_pixel_format_map_[image_type] : pixel_format);

            switch (Utils::toEnum<ImageType>(image_type)) {
            case ImageType::Scene:
            case ImageType::Infrared:
                updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type], false, pixel_format, capture_setting, ned_transform, false);
                break;
            case ImageType::Lighting:
                updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type], false, pixel_format, capture_setting, ned_transform, false);
            case ImageType::Segmentation:
                updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type], false, pixel_format, capture_setting, ned_transform, false);
                render_targets_[image_type]->TargetGamma = 1;
            case ImageType::SurfaceNormals:
                updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type], true, pixel_format, capture_setting, ned_transform, true);
                break;
            case ImageType::Annotation:
                break;
            default:
                updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type], true, pixel_format, capture_setting, ned_transform, false);
                break;
            }
            if(capture_setting.ignore_marked)captures_[image_type]->HiddenActors.Append(ignore_actors_);
            setDistortionMaterial(image_type, captures_[image_type], captures_[image_type]->PostProcessSettings);
            setNoiseMaterial(image_type, captures_[image_type], captures_[image_type]->PostProcessSettings, noise_setting);
            copyCameraSettingsToSceneCapture(camera_, captures_[image_type]); //CinemAirSim
            if(image_type == Utils::toNumeric(ImageType::Scene) || image_type == Utils::toNumeric(ImageType::Lighting)) {
                if (capture_setting.lumen_gi_enabled) {
                    captures_[image_type]->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = 1;
                    captures_[image_type]->PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::Lumen;                    
                }
                else {
                    captures_[image_type]->PostProcessSettings.bOverride_DynamicGlobalIlluminationMethod = 1;
                    captures_[image_type]->PostProcessSettings.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::None;
                }
                if (capture_setting.lumen_reflections_enabled) {
                    captures_[image_type]->PostProcessSettings.bOverride_ReflectionMethod = 1;
                    captures_[image_type]->PostProcessSettings.ReflectionMethod = EReflectionMethod::Lumen;
                }
                else {
                    captures_[image_type]->PostProcessSettings.bOverride_ReflectionMethod = 1;
					captures_[image_type]->PostProcessSettings.ReflectionMethod = EReflectionMethod::None;
                }
                captures_[image_type]->PostProcessSettings.LumenSurfaceCacheResolution = 1;
                captures_[image_type]->PostProcessSettings.LumenFinalGatherQuality = capture_setting.lumen_final_quality;
                captures_[image_type]->PostProcessSettings.LumenSceneDetail = capture_setting.lumen_scene_detail;
                captures_[image_type]->PostProcessSettings.LumenSceneLightingQuality = capture_setting.lumen_scene_lightning_quality;
                captures_[image_type]->bUseRayTracingIfEnabled = 1;
			}
        }
        else { //camera component
            updateCameraSetting(camera_, capture_setting, ned_transform);
            setDistortionMaterial(image_type, camera_, camera_->PostProcessSettings);
            setNoiseMaterial(image_type, camera_, camera_->PostProcessSettings, noise_setting);
            copyCameraSettingsToAllSceneCapture(camera_); //CinemAirSim
        }
    }
    if (camera_setting.capture_settings.at(Utils::toNumeric(ImageType::Scene)).force_update){
        setCameraTypeEnabled(ImageType::Scene, true);
        setCameraTypeUpdate(ImageType::Scene, false);
    }

    //C2 / step 4: configuration time is the only time the raymap is built. Returns immediately
    //for a camera with no CameraModel block.
    buildRaymapResource();
}

void APIPCamera::updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target,
                                               bool auto_format, const EPixelFormat& pixel_format, const CaptureSetting& setting, const NedTransform& ned_transform,
                                               bool force_linear_gamma)
{
    if (auto_format) {
        render_target->InitAutoFormat(setting.width, setting.height); //256 X 144, X 480
    }
    else {
        render_target->InitCustomFormat(setting.width, setting.height, pixel_format, force_linear_gamma);
    }

    if (!std::isnan(setting.target_gamma))
		render_target->TargetGamma = setting.target_gamma;

    capture->ProjectionType = static_cast<ECameraProjectionMode::Type>(setting.projection_mode);

    if (!std::isnan(setting.fov_degrees))
        capture->FOVAngle = setting.fov_degrees;
    if (capture->ProjectionType == ECameraProjectionMode::Orthographic && !std::isnan(setting.ortho_width))
        capture->OrthoWidth = ned_transform.fromNed(setting.ortho_width);

    updateCameraPostProcessingSetting(capture->PostProcessSettings, setting);
}

//CinemAirSim
void APIPCamera::updateCameraSetting(UCineCameraComponent* camera, const CaptureSetting& setting, const NedTransform& ned_transform)
{
    //if (!std::isnan(setting.target_gamma))
    //    camera-> = setting.target_gamma;

    camera->SetProjectionMode(static_cast<ECameraProjectionMode::Type>(setting.projection_mode));

    if (!std::isnan(setting.fov_degrees))
        camera->SetFieldOfView(setting.fov_degrees);
    if (camera->ProjectionMode == ECameraProjectionMode::Orthographic && !std::isnan(setting.ortho_width))
        camera->SetOrthoWidth(ned_transform.fromNed(setting.ortho_width));

    updateCameraPostProcessingSetting(camera->PostProcessSettings, setting);
}

msr::airlib::Pose APIPCamera::getPose() const
{
    return ned_transform_->toLocalNed(this->GetActorTransform());
}

void APIPCamera::updateCameraPostProcessingSetting(FPostProcessSettings& obj, const CaptureSetting& setting)
{
    if (!std::isnan(setting.motion_blur_amount)) {
        obj.bOverride_MotionBlurAmount = 1;
        obj.MotionBlurAmount = setting.motion_blur_amount;
    }
    if (!std::isnan(setting.motion_blur_max))
	{
		obj.bOverride_MotionBlurMax = 1;
		obj.MotionBlurMax = setting.motion_blur_max;
	} 
    if (!std::isnan(setting.motion_blur_target_fps))
    {
        obj.bOverride_MotionBlurTargetFPS = 1;
        obj.MotionBlurTargetFPS = setting.motion_blur_target_fps;
    }

    if (setting.auto_exposure_method >= 0) {
        obj.bOverride_AutoExposureMethod = 1;
        obj.AutoExposureMethod = Utils::toEnum<EAutoExposureMethod>(setting.auto_exposure_method);
    }
    if (!std::isnan(setting.auto_exposure_bias)) {
        obj.bOverride_AutoExposureBias = 1;
        obj.AutoExposureBias = setting.auto_exposure_bias;
    }
    
    obj.bOverride_AutoExposureApplyPhysicalCameraExposure = 1;
    obj.AutoExposureApplyPhysicalCameraExposure = setting.auto_exposure_apply_physical_camera_exposure ? 1 : 0;
    
    if (!std::isnan(setting.auto_exposure_min_brightness)) {
        obj.bOverride_AutoExposureMinBrightness = 1;
        obj.AutoExposureMinBrightness = setting.auto_exposure_min_brightness;
    }
    if (!std::isnan(setting.auto_exposure_max_brightness)) {
        obj.bOverride_AutoExposureMaxBrightness = 1;
        obj.AutoExposureMaxBrightness = setting.auto_exposure_max_brightness;
    }
    if (!std::isnan(setting.auto_exposure_speed_up)) {
        obj.bOverride_AutoExposureSpeedUp = 1;
        obj.AutoExposureSpeedUp = setting.auto_exposure_speed_up;
    }
    if (!std::isnan(setting.auto_exposure_speed_down)) {
        obj.bOverride_AutoExposureSpeedDown = 1;
        obj.AutoExposureSpeedDown = setting.auto_exposure_speed_down;
    }
    if (!std::isnan(setting.auto_exposure_low_percent)) {
        obj.bOverride_AutoExposureLowPercent = 1;
        obj.AutoExposureLowPercent = setting.auto_exposure_low_percent;
    }
    if (!std::isnan(setting.auto_exposure_high_percent)) {
        obj.bOverride_AutoExposureHighPercent = 1;
        obj.AutoExposureHighPercent = setting.auto_exposure_high_percent;
    }
    if (!std::isnan(setting.auto_exposure_histogram_log_min)) {
        obj.bOverride_HistogramLogMin = 1;
        obj.HistogramLogMin = setting.auto_exposure_histogram_log_min;
    }
    if (!std::isnan(setting.auto_exposure_histogram_log_max)) {
        obj.bOverride_HistogramLogMax = 1;
        obj.HistogramLogMax = setting.auto_exposure_histogram_log_max;
    }

    if (!std::isnan(setting.bloom_intensity)) {
        obj.bOverride_BloomIntensity = 1;
        obj.BloomIntensity = setting.bloom_intensity;
    }
    if (!std::isnan(setting.bloom_threshold)) {
        obj.bOverride_BloomThreshold = 1;
        obj.BloomThreshold = setting.bloom_threshold;
    }

    if (!std::isnan(setting.chromatic_aberration_intensity)) {
        obj.bOverride_SceneFringeIntensity = 1;
        obj.SceneFringeIntensity = setting.chromatic_aberration_intensity;
    }
    if (!std::isnan(setting.chromatic_aberration_start_offset)) {
        obj.bOverride_ChromaticAberrationStartOffset = 1;
        obj.ChromaticAberrationStartOffset = setting.chromatic_aberration_start_offset;
    }

    if (!std::isnan(setting.camera_shutter_speed)) {
        obj.bOverride_CameraShutterSpeed = 1;
        obj.CameraShutterSpeed = setting.camera_shutter_speed;
    }
    if (!std::isnan(setting.camera_iso)) {
        obj.bOverride_CameraISO = 1;
        obj.CameraISO = setting.camera_iso;
    }
    if (!std::isnan(setting.camera_aperture)) {
        obj.bOverride_DepthOfFieldFstop = 1;
        obj.DepthOfFieldFstop = setting.camera_aperture;
    }
    if (!std::isnan(setting.camera_max_aperture)) {
        obj.bOverride_DepthOfFieldMinFstop = 1;
        obj.DepthOfFieldMinFstop = setting.camera_max_aperture;
    }
    if (!std::isnan(setting.camera_num_blades)) {
        obj.bOverride_DepthOfFieldBladeCount = 1;
        obj.DepthOfFieldBladeCount = static_cast<int32>(setting.camera_num_blades);
    }

    if (!std::isnan(setting.lens_flare_intensity)) {
        obj.bOverride_LensFlareIntensity = 1;
        obj.LensFlareIntensity = setting.lens_flare_intensity;
    }
    if (!std::isnan(setting.lens_flare_bokeh_size)) {
        obj.bOverride_LensFlareBokehSize = 1;
        obj.LensFlareBokehSize = setting.lens_flare_bokeh_size;
    }
    if (!std::isnan(setting.lens_flare_threshold)) {
        obj.bOverride_LensFlareThreshold = 1;
        obj.LensFlareThreshold = setting.lens_flare_threshold;
    }

    if (!std::isnan(setting.depth_of_field_sensor_width)) {
        obj.bOverride_DepthOfFieldSensorWidth = 1;
        obj.DepthOfFieldSensorWidth = setting.depth_of_field_sensor_width;
    }
    if (!std::isnan(setting.depth_of_field_squeeze_factor)) {
        obj.bOverride_DepthOfFieldSqueezeFactor = 1;
        obj.DepthOfFieldSqueezeFactor = setting.depth_of_field_squeeze_factor;
    }
    if (!std::isnan(setting.depth_of_field_focal_distance)) {
        obj.bOverride_DepthOfFieldFocalDistance = 1;
        obj.DepthOfFieldFocalDistance = setting.depth_of_field_focal_distance;
    }
    if (!std::isnan(setting.depth_of_field_depth_blur_amount)) {
        obj.bOverride_DepthOfFieldDepthBlurAmount = 1;
        obj.DepthOfFieldDepthBlurAmount = setting.depth_of_field_depth_blur_amount;
    }
    if (!std::isnan(setting.depth_of_field_depth_blur_radius)) {
        obj.bOverride_DepthOfFieldDepthBlurRadius = 1;
        obj.DepthOfFieldDepthBlurRadius = setting.depth_of_field_depth_blur_radius;
    }
    if (!std::isnan(setting.depth_of_field_use_hair_depth)) {
        obj.bOverride_DepthOfFieldUseHairDepth = 1;
        obj.DepthOfFieldUseHairDepth = setting.depth_of_field_use_hair_depth ? 1 : 0;
    }

}

void APIPCamera::setDistortionMaterial(int image_type, UObject* outer, FPostProcessSettings& obj)
{
    UMaterialInstanceDynamic* distortion_material = UMaterialInstanceDynamic::Create(distortion_material_static_, outer);
    distortion_materials_[image_type + 1] = distortion_material;
    obj.AddBlendable(distortion_material, 1.0f);
}

void APIPCamera::setNoiseMaterial(int image_type, UObject* outer, FPostProcessSettings& obj, const NoiseSetting& settings)
{
    if (!settings.Enabled)
        return;

    UMaterialInstanceDynamic* noise_material = UMaterialInstanceDynamic::Create(noise_material_static_, outer);
    noise_materials_[image_type + 1] = noise_material;

    noise_material->SetScalarParameterValue("HorzWaveStrength", settings.HorzWaveStrength);
    noise_material->SetScalarParameterValue("RandSpeed", settings.RandSpeed);
    noise_material->SetScalarParameterValue("RandSize", settings.RandSize);
    noise_material->SetScalarParameterValue("RandDensity", settings.RandDensity);
    noise_material->SetScalarParameterValue("RandContrib", settings.RandContrib);
    noise_material->SetScalarParameterValue("HorzWaveContrib", settings.HorzWaveContrib);
    noise_material->SetScalarParameterValue("HorzWaveVertSize", settings.HorzWaveVertSize);
    noise_material->SetScalarParameterValue("HorzWaveScreenSize", settings.HorzWaveScreenSize);
    noise_material->SetScalarParameterValue("HorzNoiseLinesContrib", settings.HorzNoiseLinesContrib);
    noise_material->SetScalarParameterValue("HorzNoiseLinesDensityY", settings.HorzNoiseLinesDensityY);
    noise_material->SetScalarParameterValue("HorzNoiseLinesDensityXY", settings.HorzNoiseLinesDensityXY);
    noise_material->SetScalarParameterValue("HorzDistortionStrength", settings.HorzDistortionStrength);
    noise_material->SetScalarParameterValue("HorzDistortionContrib", settings.HorzDistortionContrib);

    obj.AddBlendable(noise_material, 1.0f);

	if (settings.LensDistortionEnable) {

		UMaterialInstanceDynamic* lens_distortion_material_;

		if (settings.LensDistortionInvert) {
			lens_distortion_material_ = UMaterialInstanceDynamic::Create(lens_distortion_invert_material_static_, outer);
		}
		else {
			lens_distortion_material_ = UMaterialInstanceDynamic::Create(lens_distortion_material_static_, outer);
		}

		lens_distortion_materials_[image_type + 1] = lens_distortion_material_;


		lens_distortion_material_->SetScalarParameterValue("AreaFalloff", settings.LensDistortionAreaFalloff);
		lens_distortion_material_->SetScalarParameterValue("AreaRadius", settings.LensDistortionAreaRadius);
		lens_distortion_material_->SetScalarParameterValue("Intensity", settings.LensDistortionIntensity);


		obj.AddBlendable(lens_distortion_material_, 1.0f);
	}

    if (settings.FakeMotionBlurEnable) {
        UMaterialInstanceDynamic* motion_blur_material = UMaterialInstanceDynamic::Create(motion_blur_material_static_, outer);
        fake_motion_blur_materials_[image_type + 1] = motion_blur_material;
        
        motion_blur_material->SetScalarParameterValue("MotionBlurDirectionX", settings.FakeMotionBlurDirectionX);
        motion_blur_material->SetScalarParameterValue("MotionBlurDirectionY", settings.FakeMotionBlurDirectionY);
        motion_blur_material->SetScalarParameterValue("MotionBlurMovementSpeed", settings.FakeMotionBlurMovementSpeed);
        motion_blur_material->SetScalarParameterValue("MotionBlurShutterSpeed", settings.FakeMotionBlurShutterSpeed);
        motion_blur_material->SetScalarParameterValue("MotionBlurFocalLength", settings.FakeMotionBlurFocalLength);
        motion_blur_material->SetScalarParameterValue("MotionBlurMovementSpeed", settings.FakeMotionBlurSamples);

        obj.AddBlendable(motion_blur_material, 1.0f);
    }
    if (settings.RadialBlurEnable) {
        UMaterialInstanceDynamic* radial_blur_material = UMaterialInstanceDynamic::Create(radial_blur_material_static_, outer);
        radial_blur_materials_[image_type + 1] = radial_blur_material;

        radial_blur_material->SetScalarParameterValue("RadialBlurDistance", settings.RadialBlurDistance);
        radial_blur_material->SetScalarParameterValue("RadialBlurRadius", settings.RadialBlurRadius);
        radial_blur_material->SetScalarParameterValue("RadialBlurDensity", settings.RadialBlurDensity);

        obj.AddBlendable(radial_blur_material, 1.0f);
    }
    if (settings.GuassianBlurEnable) {
        UMaterialInstanceDynamic* guassian_blur_material = UMaterialInstanceDynamic::Create(guassian_blur_material_static_, outer);
        guassian_blur_materials_[image_type + 1] = guassian_blur_material;

        guassian_blur_material->SetScalarParameterValue("GuassianDirections", settings.GuassianBlurDirections);
        guassian_blur_material->SetScalarParameterValue("GuassianQuality", settings.GuassianBlurQuality);
        guassian_blur_material->SetScalarParameterValue("GuassianSize", settings.GuassianBlurSize);

        obj.AddBlendable(guassian_blur_material, 1.0f);
    }
}

void APIPCamera::enableCaptureComponent(const APIPCamera::ImageType type, bool is_enabled, std::string annotation_name)
{
    USceneCaptureComponent2D* capture = getCaptureComponent(type, false, annotation_name);
    if (capture != nullptr) {
        UDetectionComponent* detection = getDetectionComponent(type, false, annotation_name);
        if (is_enabled) {
            //do not make unnecessary calls to Activate() which otherwise causes crash in Unreal
            if (!capture->IsActive() || capture->TextureTarget == nullptr) {
                capture->TextureTarget = getRenderTarget(type, false, annotation_name);
                capture->Activate();
                if (detection != nullptr) {
                    detection->texture_target_ = capture->TextureTarget;
                    detection->Activate();
                }
            }
        }
        else {
            if (capture->IsActive() || capture->TextureTarget != nullptr) {
                capture->Deactivate();
                capture->TextureTarget = nullptr;
                if (detection != nullptr) {
                    detection->Deactivate();
                    detection->texture_target_ = nullptr;
                }
            }
        }
        //C2 / F5: the cube face rig follows camera_type_enabled_ exactly rather than inventing a
        //second mechanism. Without a CameraModel block this is one bool test and returns.
        setFaceRigEnabled(type, is_enabled);

        if (type == ImageType::Annotation)
			camera_type_enabled_[annotator_name_to_index_map_[FString(annotation_name.c_str())]] = is_enabled;
        else
            camera_type_enabled_[Utils::toNumeric(type)] = is_enabled;
    }
    //else nothing to enable
}

// -------------------------------------------------------------------------------------------
// Generic (non-pinhole) camera cube face rig - Phase 3b step 3.
//
// D13: six USceneCaptureComponent2D, not one USceneCaptureComponentCube. The cube component
// renders all six faces and cannot skip one, has a locked 90 degree field of view and a
// hardcoded near plane, and on Vulkan cannot be read back per face as uint8 at all. Six 2D
// captures cost more components and buy back all four.
//
// FACE ORIENTATION CONVENTION - stated here because getting it wrong produces a plausible
// image that is wrong everywhere, and the error is then blamed on the camera model or on the
// resample. Step 2 measured a cube component's faces arriving rotated 90 degrees against a 2D
// capture of the same pose; that is the failure this convention and the shared-edge check exist
// to rule out.
//
// All axes below are Unreal component axes: +X forward, +Y right, +Z up. A scene capture looks
// down its own +X, its image right is its own +Y and its image up is its own +Z. Faces are
// defined in the CAMERA frame and attached to camera_ at zero relative offset, so every face
// shares the pinhole capture's origin and a face's world pose is the camera pose composed with
// the fixed relative rotation below.
//
//   idx  name    forward   image right   image up   relative rotator (pitch, yaw, roll)
//    0   Front     +X          +Y           +Z         (  0,    0, 0)
//    1   Right     +Y          -X           +Z         (  0,   90, 0)
//    2   Left      -Y          +X           +Z         (  0,  -90, 0)
//    3   Up        +Z          +Y           -X         ( 90,    0, 0)
//    4   Down      -Z          +Y           +X         (-90,    0, 0)
//    5   Back      -X          -Y           +Z         (  0,  180, 0)
//
// In words: the four side faces keep camera-up as image up, and Up and Down are reached by
// pitching the front face, so the front face's image right (+Y) remains image right on all six.
// Back is deliberately last, so "Faces": 5 - legal for a camera whose field of view is at most
// 180 degrees, since no ray of such a camera reaches the rear face - is exactly the first five
// rows of the same table and needs no second ordering.
//
// The convention is checkable, not assumed: each face is an exact pinhole render sharing one
// origin, so along a shared cube edge two faces sample the same directions to within half a
// face texel. A discontinuity there means wrong orientation, wrong field of view, or a
// non-shared origin. airsim.CubeFaceDump measures all twelve edges - see CubeFaceDump.cpp.
//
// Nothing here is agnostic-hostile to a later cube-backed equirectangular path: the faces are
// handed out through getFaceCaptureComponent / getFaceRenderTarget, which say nothing about
// where a face came from.
// -------------------------------------------------------------------------------------------

const TCHAR* APIPCamera::getCubeFaceName(int face)
{
    switch (face) {
    case 0:
        return TEXT("Front");
    case 1:
        return TEXT("Right");
    case 2:
        return TEXT("Left");
    case 3:
        return TEXT("Up");
    case 4:
        return TEXT("Down");
    case 5:
        return TEXT("Back");
    default:
        return TEXT("Invalid");
    }
}

FRotator APIPCamera::getCubeFaceRotation(int face)
{
    switch (face) {
    case 0:
        return FRotator(0.0f, 0.0f, 0.0f); //Front, +X
    case 1:
        return FRotator(0.0f, 90.0f, 0.0f); //Right, +Y
    case 2:
        return FRotator(0.0f, -90.0f, 0.0f); //Left, -Y
    case 3:
        return FRotator(90.0f, 0.0f, 0.0f); //Up, +Z
    case 4:
        return FRotator(-90.0f, 0.0f, 0.0f); //Down, -Z
    default:
        return FRotator(0.0f, 180.0f, 0.0f); //Back, -X
    }
}

bool APIPCamera::hasCameraModel() const
{
    return sensor_params_.camera_model.enabled;
}

int APIPCamera::getCubeFaceCount() const
{
    if (!sensor_params_.camera_model.enabled)
        return 0;

    //"Faces": Auto resolves to six here. Choosing five automatically needs the model's largest
    //incidence angle, which is the same computation the automatic face resolution of design
    //section 6 needs - both belong to step 7. An explicit "Faces": 5 is honoured now.
    return sensor_params_.camera_model.faces == 5 ? 5 : kCubeFaceCount;
}

int APIPCamera::getCubeFaceResolution() const
{
    if (!sensor_params_.camera_model.enabled)
        return 0;

    if (sensor_params_.camera_model.cube_face_resolution > 0)
        return sensor_params_.camera_model.cube_face_resolution;

    //Documented default for "CubeFaceResolution": 0, until step 7 derives it from the angular
    //density at the face centre (design section 6). The rule is one face texel per output texel
    //along the longer output axis. Section 6's value is width * (pi/2) / fov, so this default is
    //at or above it for every field of view of 90 degrees or more - it therefore never
    //undersamples a camera wide enough to need this path at all - and it oversamples a 180
    //degree lens by 2x linear. Oversampling costs render and readback time, undersampling costs
    //image quality that cannot be recovered later, so the default errs the way section 6 says to
    //err. Set CubeFaceResolution explicitly to trade it back.
    //
    //A Raymap model carries no Width or Height in settings, so it lands on the 64 floor and must
    //set CubeFaceResolution explicitly until step 4 loads the map itself.
    const int widest = FMath::Max(static_cast<int>(sensor_params_.camera_model.model.width),
                                  static_cast<int>(sensor_params_.camera_model.model.height));
    return FMath::Clamp(widest, 64, 2048);
}

void APIPCamera::ensureFaceRig(const ImageType type)
{
    //A naive rig is imageTypeCount() * 6 = 66 components per camera. This builds a face set only
    //for a camera that asked for a camera model, and only for the ImageTypes that camera is
    //actually asked for - which UnrealImageCapture::updateCameraVisibility does exactly once per
    //(camera, ImageType), on the first image request.
    if (!sensor_params_.camera_model.enabled)
        return;
    if (type == ImageType::Annotation)
        return; //annotation cameras stay on the pinhole path

    //NewObject and RegisterComponent are game-thread only, and this function is reached from an
    //rpclib worker: WorldSimApi::getImages -> UnrealImageCapture::updateCameraVisibility ->
    //setCameraTypeEnabled -> setFaceRigEnabled. The pinhole path survives that because it only
    //toggles components that already exist; building them off-thread does not. Measured
    //2026-08-05 - USceneCaptureComponent::RegisterDelegates ensures on the race, then the frame
    //dies on "Assertion failed: !bPostTickComponentUpdate [LevelTick.cpp:905]".
    //
    //It went unnoticed until now only because every earlier session ran airsim.CubeFaceDump - a
    //console command, so game thread - before its first image request, which warmed the rig.
    if (!IsInGameThread()) {
        UAirBlueprintLib::RunCommandOnGameThread([this, type]() { ensureFaceRig(type); }, true);
        return;
    }

    const unsigned int image_type = Utils::toNumeric(type);
    if (image_type >= imageTypeCount())
        return;

    if (face_captures_.Num() == 0) {
        face_captures_.Init(nullptr, static_cast<int32>(imageTypeCount()) * kCubeFaceCount);
        face_render_targets_.Init(nullptr, static_cast<int32>(imageTypeCount()) * kCubeFaceCount);
    }

    const int32 base = static_cast<int32>(image_type) * kCubeFaceCount;
    if (face_captures_[base] != nullptr)
        return; //already built for this ImageType

    USceneCaptureComponent2D* parent = captures_[image_type];
    UTextureRenderTarget2D* parent_target = render_targets_[image_type];
    if (parent == nullptr || parent_target == nullptr)
        return;

    //Mirror whatever the pinhole target ended up as, rather than repeating the auto-format /
    //custom-format switch of setupCameraFromSettings: GetFormat() resolves OverrideFormat or
    //RenderTargetFormat, so this stays correct if that switch changes.
    const EPixelFormat parent_format = parent_target->GetFormat();
    if (parent_format == EPixelFormat::PF_Unknown) {
        UE_LOG(LogTemp, Warning,
               TEXT("[AirSim] cube face rig skipped for %s image type %d: pinhole render target has no format yet"),
               *GetName(), static_cast<int32>(image_type));
        return;
    }

    const int resolution = getCubeFaceResolution();
    const int face_count = getCubeFaceCount();
    //camera_ is what copyCameraSettingsToSceneCapture aligns every pinhole capture to, so
    //attaching there is what makes "shared origin" true rather than approximately true
    USceneComponent* attach_to = camera_;
    if (attach_to == nullptr)
        attach_to = this->RootComponent;

    for (int face = 0; face < face_count; ++face) {
        const FString component_name = FString::Printf(TEXT("CubeFace_%d_%s"), static_cast<int32>(image_type), getCubeFaceName(face));
        USceneCaptureComponent2D* capture = NewObject<USceneCaptureComponent2D>(this, USceneCaptureComponent2D::StaticClass(), *component_name);

        capture->bAutoActivate = false;
        capture->SetRelativeLocation(FVector::ZeroVector); //shared origin with the pinhole capture
        capture->SetRelativeRotation(getCubeFaceRotation(face));
        capture->AttachToComponent(attach_to, FAttachmentTransformRules::KeepRelativeTransform);
        capture->RegisterComponent();

        //After RegisterComponent, never before: USceneCaptureComponent::OnRegister calls
        //UpdateShowFlags, which overwrites ShowFlags wholesale from the archetype. Assigned
        //earlier, Segmentation's annotation show flags and Lighting's would be silently lost and
        //those faces would render the wrong thing while looking fine.
        capture->ShowFlags = parent->ShowFlags;
        capture->CaptureSource = parent->CaptureSource;
        capture->PrimitiveRenderMode = parent->PrimitiveRenderMode;
        capture->ShowOnlyComponents = parent->ShowOnlyComponents;
        capture->HiddenComponents = parent->HiddenComponents;
        capture->ShowOnlyActors = parent->ShowOnlyActors;
        capture->HiddenActors = parent->HiddenActors;
        capture->PostProcessSettings = parent->PostProcessSettings; //carries the blendables, so depth and segmentation materials come with it
        capture->PostProcessBlendWeight = parent->PostProcessBlendWeight;

        //Phase 3b step 5, design section 5 O3: NON-PINHOLE DEPTH IS RANGE ALONG THE RAY.
        //
        //DepthPlanar has no meaning without an image plane, and per-face planar depth is measured
        //against THAT FACE's axis, so values from two faces are not comparable and interpolating
        //them across a boundary is meaningless. Range is: all six faces share one origin (the
        //SetRelativeLocation(FVector::ZeroVector) above), so range is a single continuous
        //function of direction over the whole sphere.
        //
        //DepthPerspectiveMaterial already emits exactly that - it is VectorLength of the
        //camera-relative world position - so the correction is not a shader conversion at all,
        //it is giving the DepthPlanar FACES the perspective material. No asset load, no new
        //material, and both depth types then return range, which is what O3 resolved they should.
        //
        //This is a copy onto components that exist only for a camera with a CameraModel block:
        //captures_[DepthPlanar] - every ordinary pinhole camera in AirSim - is untouched, and so
        //is the parent's own struct. It does replace this face's distortion and noise blendables
        //with DepthPerspective's, which are the same materials configured the same way for the
        //same camera.
        if (type == ImageType::DepthPlanar) {
            USceneCaptureComponent2D* perspective = captures_[Utils::toNumeric(ImageType::DepthPerspective)];
            if (perspective != nullptr)
                capture->PostProcessSettings.WeightedBlendables = perspective->PostProcessSettings.WeightedBlendables;
        }

        //C3, measured 2026-08-05: kill the screen-space lens effects on the FACE captures only.
        //Each of these is computed per view, radially about ITS OWN frame centre, so a 90-degree
        //cube face applies a full lens vignette inside what is really one patch of a wider image -
        //and the darkening peaks exactly at the face boundary, i.e. exactly at the seams. The
        //argument is the same one that keeps lens distortion off the faces; the difference is that
        //vignette is on by default, so it was already in every non-pinhole image.
        //
        //Numbers behind this: the pinhole-through-cube test (test 1) reads 1.047 LSB mean with
        //these on and 0.047 with them off, 97.6 percent of pixels bit-exact; the projection gate
        //(test 2) moved 1.260 -> 0.449 px on the yaw -70 reference. Vignette is the large term,
        //bloom about 0.09 LSB. Ambient occlusion, screen-space reflections and diffuse indirect
        //were measured and are NOT contributors, so they are deliberately left alone.
        //
        //This is a copy: PostProcessSettings is a value member, and these components exist only
        //for a camera with a CameraModel block. captures_[image_type] - every ordinary pinhole
        //camera in AirSim - is not touched, and neither is the parent's own struct.
        capture->PostProcessSettings.bOverride_VignetteIntensity = true;
        capture->PostProcessSettings.VignetteIntensity = 0.0f;
        capture->PostProcessSettings.bOverride_BloomIntensity = true;
        capture->PostProcessSettings.BloomIntensity = 0.0f;
        capture->PostProcessSettings.bOverride_SceneFringeIntensity = true;
        capture->PostProcessSettings.SceneFringeIntensity = 0.0f; //chromatic aberration: radial, same argument
        capture->PostProcessSettings.bOverride_FilmGrainIntensity = true;
        capture->PostProcessSettings.FilmGrainIntensity = 0.0f; //per-face grain would not even be consistent between faces
        capture->bUseRayTracingIfEnabled = parent->bUseRayTracingIfEnabled;
        capture->MaxViewDistanceOverride = parent->MaxViewDistanceOverride;
        capture->LODDistanceFactor = parent->LODDistanceFactor;

        //a cube face is exactly 90 degrees on a square target, whatever the output camera is
        capture->ProjectionType = ECameraProjectionMode::Perspective;
        capture->FOVAngle = 90.0f;

        //never per frame: re-rendering capture components every frame was the I-G defect
        capture->bCaptureEveryFrame = false;
        capture->bCaptureOnMovement = false;
        capture->bAlwaysPersistRenderingState = true;
        capture->SetVisibility(true); //CaptureScene and CaptureSceneDeferred both test IsVisible()
        capture->Deactivate();

        UTextureRenderTarget2D* target = NewObject<UTextureRenderTarget2D>();
        target->ClearColor = parent_target->ClearColor;
        target->InitCustomFormat(static_cast<uint32>(resolution), static_cast<uint32>(resolution),
                                 parent_format, parent_target->bForceLinearGamma);
        target->TargetGamma = parent_target->TargetGamma;

        face_captures_[base + face] = capture;
        face_render_targets_[base + face] = target;
    }

    //One line per rig actually built. Its absence is the runtime half of the non-invasiveness
    //check: no CameraModel block, no line, no components.
    UE_LOG(LogTemp, Log,
           TEXT("[AirSim] cube face rig built: %s image type %d, %d faces at %dx%d, format %d"),
           *GetName(), static_cast<int32>(image_type), face_count, resolution, resolution,
           static_cast<int32>(parent_format));
}

void APIPCamera::setFaceRigEnabled(const ImageType type, bool is_enabled)
{
    if (!sensor_params_.camera_model.enabled)
        return; //no CameraModel block: one bool test, and nothing is ever built
    if (type == ImageType::Annotation)
        return;

    if (is_enabled)
        ensureFaceRig(type);

    if (face_captures_.Num() == 0)
        return;

    const unsigned int image_type = Utils::toNumeric(type);
    if (image_type >= imageTypeCount())
        return;

    const int32 base = static_cast<int32>(image_type) * kCubeFaceCount;
    for (int face = 0; face < kCubeFaceCount; ++face) {
        USceneCaptureComponent2D* capture = face_captures_[base + face];
        if (capture == nullptr)
            continue; //a five-face rig leaves the last slot empty

        //the same activation dance enableCaptureComponent does for the pinhole capture, for the
        //same reason: repeated Activate() calls crash Unreal
        if (is_enabled) {
            if (!capture->IsActive() || capture->TextureTarget == nullptr) {
                capture->TextureTarget = face_render_targets_[base + face];
                capture->Activate();
            }
        }
        else if (capture->IsActive() || capture->TextureTarget != nullptr) {
            capture->Deactivate();
            capture->TextureTarget = nullptr;
        }
    }
}

USceneCaptureComponent2D* APIPCamera::getFaceCaptureComponent(const ImageType type, int face)
{
    if (face_captures_.Num() == 0 || face < 0 || face >= kCubeFaceCount || type == ImageType::Annotation)
        return nullptr;

    const unsigned int image_type = Utils::toNumeric(type);
    if (image_type >= imageTypeCount())
        return nullptr;

    return face_captures_[static_cast<int32>(image_type) * kCubeFaceCount + face];
}

UTextureRenderTarget2D* APIPCamera::getFaceRenderTarget(const ImageType type, int face)
{
    if (face_render_targets_.Num() == 0 || face < 0 || face >= kCubeFaceCount || type == ImageType::Annotation)
        return nullptr;

    const unsigned int image_type = Utils::toNumeric(type);
    if (image_type >= imageTypeCount())
        return nullptr;

    return face_render_targets_[static_cast<int32>(image_type) * kCubeFaceCount + face];
}

const FAirSimRaymapResourcePtr& APIPCamera::getRaymapResource() const
{
    return raymap_;
}

// -------------------------------------------------------------------------------------------
// The raymap on the GPU - and the ONE place the optical to Unreal axis change happens.
//
// AirLib builds the raymap in the calibration's own frame: the right-handed OPTICAL frame,
// +x right along an image row, +y down an image column, +z forward out of the lens. That is the
// frame OpenCV, Kalibr and ScanNet++ express cx, cy and k1..k4 in, and staying in it is what
// lets tools/raymap_check.py check our rays against those tools rather than against ourselves.
// The conventions block at the top of AirLib/include/cameras/CameraModel.hpp is authoritative.
//
// The cube faces are in the Unreal CAMERA frame: +X forward, +Y right, +Z up. So
//
//     Unreal X (forward)  =  optical  z
//     Unreal Y (right)    =  optical  x
//     Unreal Z (up)       =  optical -y
//
// which is a handedness flip, as it must be - the optical frame is right handed and Unreal's is
// left handed. It is applied here, once, as each ray is copied into the GPU buffer, and nothing
// downstream re-applies it: CubeResample.usf assumes the buffer is already in the Unreal camera
// frame and uses the face table stated above getCubeFaceRotation() in this file. If an image
// ever comes out mirrored or rotated, this function and that table are the two places to look,
// and they must agree.
//
// Origins take the same change of basis plus the metres to centimetres scale Unreal works in.
// Every camera model in the v1 set is central, so every origin is exactly zero and the scale is
// unobservable today. It is applied anyway: the first non-central raymap would otherwise be
// wrong by a factor of 100 in a way that reads like a calibration error. Per ADR-001 the origin
// channels are stored whether or not anything reads them - and the cube path does not, because
// all six faces share one origin by construction, so a non-central model needs the native
// raymap path rather than this one.
// -------------------------------------------------------------------------------------------

void APIPCamera::buildRaymapResource()
{
    //setupCameraFromSettings can run more than once for a camera; never leak the previous one
    if (raymap_.IsValid())
        AirSimReleaseRaymap(raymap_);

    if (!sensor_params_.camera_model.enabled)
        return; //no CameraModel block: nothing allocated, no render command enqueued, no log line

    msr::airlib::cameras::Raymap map;
    msr::airlib::cameras::RaymapStats stats;
    std::string error;
    if (!msr::airlib::cameras::buildRaymap(sensor_params_.camera_model.model, map, stats, error)) {
        UE_LOG(LogTemp, Error,
               TEXT("[AirSim] %s: raymap build failed (%s). Camera stays on the pinhole path."),
               *GetName(), UTF8_TO_TCHAR(error.c_str()));
        return;
    }

    //The output image IS the camera model's image: fx, fy, cx and cy are expressed in the
    //model's own pixels, so a Scene target of a different size is a different camera. Rescaling
    //the raymap to fit would publish plausible, wrong intrinsics - the exact silent failure
    //design section 7.3 warns about - so refuse instead, loudly, and leave the camera pinhole.
    const CaptureSetting& scene_setting = sensor_params_.capture_settings.at(Utils::toNumeric(ImageType::Scene));
    if (map.width != scene_setting.width || map.height != scene_setting.height) {
        UE_LOG(LogTemp, Error,
               TEXT("[AirSim] %s: CameraModel is %ux%u but its Scene CaptureSettings is %ux%u. ")
               TEXT("These must match. Camera stays on the pinhole path until they do."),
               *GetName(), map.width, map.height, scene_setting.width, scene_setting.height);
        return;
    }

    const int32 channels = static_cast<int32>(msr::airlib::cameras::Raymap::kChannels);
    TArray<float> values;
    values.SetNumUninitialized(static_cast<int32>(map.width) * static_cast<int32>(map.height) * channels);

    for (unsigned int y = 0; y < map.height; ++y) {
        for (unsigned int x = 0; x < map.width; ++x) {
            const msr::airlib::cameras::Ray ray = map.at(x, y);
            const int32 base = (static_cast<int32>(y) * static_cast<int32>(map.width) + static_cast<int32>(x)) * channels;

            values[base + 0] = static_cast<float>(ray.oz * 100.0);
            values[base + 1] = static_cast<float>(ray.ox * 100.0);
            values[base + 2] = static_cast<float>(-ray.oy * 100.0);
            //a texel the model could not unproject keeps its all-zero direction, which is what
            //the shader tests for; do not normalise or otherwise repair it here
            values[base + 3] = static_cast<float>(ray.dz);
            values[base + 4] = static_cast<float>(ray.dx);
            values[base + 5] = static_cast<float>(-ray.dy);
        }
    }

    const double megabytes = static_cast<double>(values.Num()) * sizeof(float) / (1024.0 * 1024.0);

    raymap_ = AirSimCreateRaymapResource();
    AirSimUploadRaymap(raymap_, MoveTemp(values), map.width, map.height, map.central);

    //One line per raymap actually uploaded. Its absence is the other half of the runtime
    //non-invasiveness check, alongside the cube face rig line: no CameraModel block, no line.
    UE_LOG(LogTemp, Log,
           TEXT("[AirSim] raymap uploaded: %s model %s %ux%u, 6 floats/texel, %.1f MB, ")
           TEXT("%llu of %llu texels outside the model's valid domain, central=%s"),
           *GetName(),
           UTF8_TO_TCHAR(msr::airlib::cameras::toString(sensor_params_.camera_model.model.type)),
           map.width, map.height, megabytes,
           static_cast<unsigned long long>(stats.invalid),
           static_cast<unsigned long long>(stats.texels),
           map.central ? TEXT("true") : TEXT("false"));
}

UTextureRenderTarget2D* APIPCamera::getRenderTarget(const APIPCamera::ImageType type, bool if_active, std::string annotation_name)
{
    unsigned int image_type = Utils::toNumeric(type);
	if (type == ImageType::Annotation)
    {        
        if (!if_active || camera_type_enabled_[annotator_name_to_index_map_[FString(annotation_name.c_str())]])
            return render_targets_[annotator_name_to_index_map_[FString(annotation_name.c_str())]];
    }
    else {
        if (!if_active || camera_type_enabled_[image_type])
            return render_targets_[image_type];
    }    
    return nullptr;
}

UDetectionComponent* APIPCamera::getDetectionComponent(const ImageType type, bool if_active, std::string annotation_name) const
{
    if (type == ImageType::Annotation) {
        if (!if_active || camera_type_enabled_[annotator_name_to_index_map_[FString(annotation_name.c_str())]])
            return detections_[annotator_name_to_index_map_[FString(annotation_name.c_str())]];
    }
    else {
        unsigned int image_type = Utils::toNumeric(type);
        if (!if_active || camera_type_enabled_[image_type])
            return detections_[image_type];
    }
    return nullptr;
}

USceneCaptureComponent2D* APIPCamera::getCaptureComponent(const APIPCamera::ImageType type, bool if_active, std::string annotation_name)
{
    if (type == ImageType::Annotation) {
        if (!if_active || camera_type_enabled_[annotator_name_to_index_map_[FString(annotation_name.c_str())]])
            return captures_[annotator_name_to_index_map_[FString(annotation_name.c_str())]];
    }
    else {
        unsigned int image_type = Utils::toNumeric(type);
        if (!if_active || camera_type_enabled_[image_type])
            return captures_[image_type];
    }
    return nullptr;
}

void APIPCamera::disableAllPIP()
{
    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
		if (Utils::toEnum<ImageType>(image_type) == ImageType::Annotation)
		{
			for (auto& annotator : annotator_name_to_index_map_)
			{
				enableCaptureComponent(ImageType::Annotation, false, TCHAR_TO_UTF8(*annotator.Key));
			}
		}
        else {
            enableCaptureComponent(Utils::toEnum<ImageType>(image_type), false);
        }
    }
}

void APIPCamera::disableMain()
{
    camera_->Deactivate();
    camera_->SetVisibility(false);
    //APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    //if (controller && controller->GetViewTarget() == this)
    //    controller->SetViewTarget(nullptr);
}

void APIPCamera::onViewModeChanged(bool nodisplay)
{
    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        if (Utils::toEnum<ImageType>(image_type) == ImageType::Annotation)
        {
            for (auto& annotator : annotator_name_to_index_map_)
            {
                USceneCaptureComponent2D* capture = getCaptureComponent(ImageType::Annotation, false, TCHAR_TO_UTF8(*annotator.Key));
                if (capture) {
                    setCaptureUpdate(capture, nodisplay);
                }
            }
        }
        else
        {
            // I-G root cause. ImageType::Scene used to be excluded here, with no comment saying
            // why, so Scene capture components never got bCaptureEveryFrame = false and kept
            // re-rendering the whole scene EVERY FRAME - even though the caller ten lines up says:
            //
            //     "We set all cameras to start as nodisplay. This improves performance because the
            //      capture components are no longer updating every frame and only update while
            //      requesting an image"
            //
            // With 6 vehicles x 9 cameras that is 55 scene renders per frame, and a simGetImages
            // readback ends up queued behind all of them: measured c+d 215 ms with b wait-draw only
            // 6 ms - the game thread was fine, the render thread was 200 ms deep.
            //
            // Including Scene here cuts a fleet cycle 1535 -> 242 ms (6.35x), images verified
            // unchanged. Nothing needs Scene to capture every frame:
            //   - the main viewport renders through camera_ (UCineCameraComponent) + SetViewTarget,
            //     not through this capture component - see showToScreen()
            //   - simGetImages issues its own CaptureSceneDeferred() for what it requests
            //   - SubWindows DO need it, and set it themselves in
            //     ASimHUD::updateWidgetSubwindowVisibility via setCameraTypeUpdate(type, false)
            //
            // airsim.CaptureEveryFrame still forces either state at runtime for A/B.
            USceneCaptureComponent2D* capture = getCaptureComponent(static_cast<ImageType>(image_type), false);
            if (capture) {
                setCaptureUpdate(capture, nodisplay);
            }
        }
    }
}

//CinemAirSim methods
std::vector<std::string> APIPCamera::getPresetLensSettings() const
{
    std::vector<std::string> vector;
    const TArray<FNamedLensPreset> lens_presets = UCineCameraSettings::GetLensPresets();
    for (const FNamedLensPreset& preset : lens_presets) {
        std::ostringstream current_lens_string;
        std::string name = (TCHAR_TO_UTF8(*preset.Name));

        current_lens_string << "Name: " << name << ";\n\t MinFocalLength: " << preset.LensSettings.MinFocalLength << "; \t MaxFocalLength: " << preset.LensSettings.MaxFocalLength;
        current_lens_string << "\n\t Min FStop: " << preset.LensSettings.MinFStop << "; \t Max Fstop: " << preset.LensSettings.MaxFStop;
        vector.push_back(current_lens_string.str());
    }
    return vector;
}

std::string APIPCamera::getLensSettings() const
{
    const FCameraLensSettings current_lens_params = camera_->LensSettings;

    std::ostringstream current_lens_string;

    const FString lens_preset_name = camera_->GetLensPresetName();
    std::string name = (TCHAR_TO_UTF8(*lens_preset_name));

    current_lens_string << "Name: " << name;
    current_lens_string << ";\n\t MinFocalLength: " << current_lens_params.MinFocalLength;
    current_lens_string << "; \t MaxFocalLength: " << current_lens_params.MaxFocalLength;
    current_lens_string << "\n\t Min FStop: " << current_lens_params.MinFStop;
    current_lens_string << "; \t Max Fstop: " << current_lens_params.MaxFStop;
    current_lens_string << "\n\t Diaphragm Blade Count: " << current_lens_params.DiaphragmBladeCount;
    current_lens_string << "\n\t Minimum focus distance: " << current_lens_params.MinimumFocusDistance;

    return current_lens_string.str();
}

void APIPCamera::setPresetLensSettings(std::string preset_string)
{
    const FString preset(preset_string.c_str());
    camera_->SetLensPresetByName(preset);
    copyCameraSettingsToAllSceneCapture(camera_);
}

std::vector<std::string> APIPCamera::getPresetFilmbackSettings() const
{
    std::vector<std::string> vector_all_presets;
    TArray<FNamedFilmbackPreset> lens_presets = UCineCameraSettings::GetFilmbackPresets();
    for (const FNamedFilmbackPreset& preset : lens_presets) {
        std::ostringstream preset_string;
        std::string name = (TCHAR_TO_UTF8(*preset.Name));

        preset_string << "Name: " << name << ";\n\t Sensor Width: " << preset.FilmbackSettings.SensorWidth << "; \t Sensor Height: " << preset.FilmbackSettings.SensorHeight;
        preset_string << "\n\t Sensor Aspect Ratio: " << preset.FilmbackSettings.SensorAspectRatio;
        vector_all_presets.push_back(preset_string.str());
    }
    return vector_all_presets;
}

void APIPCamera::setPresetFilmbackSettings(std::string preset_string)
{
    const FString preset(preset_string.c_str());
    camera_->SetFilmbackPresetByName(preset);
    copyCameraSettingsToAllSceneCapture(camera_);
}

std::string APIPCamera::getFilmbackSettings() const
{
    FCameraFilmbackSettings current_filmback_settings = camera_->Filmback;

    const FString filmback_present_name = camera_->GetFilmbackPresetName();
    std::ostringstream current_filmback_string;
    std::string name = (TCHAR_TO_UTF8(*filmback_present_name));

    current_filmback_string << "Name: " << name << ";\n\t Sensor Width: " << current_filmback_settings.SensorWidth << "; \t Sensor Height: " << current_filmback_settings.SensorHeight;
    current_filmback_string << "\n\t Sensor Aspect Ratio: " << current_filmback_settings.SensorAspectRatio;
    return current_filmback_string.str();
}

float APIPCamera::setFilmbackSettings(float sensor_width, float sensor_height)
{
    camera_->Filmback.SensorWidth = sensor_width;
    camera_->Filmback.SensorHeight = sensor_height;

    copyCameraSettingsToAllSceneCapture(camera_);

    return camera_->Filmback.SensorAspectRatio;
}

float APIPCamera::getFocalLength() const
{
    return camera_->CurrentFocalLength;
}

void APIPCamera::setFocalLength(float focal_length)
{
    camera_->CurrentFocalLength = focal_length;
    copyCameraSettingsToAllSceneCapture(camera_);
}

void APIPCamera::enableManualFocus(bool enable)
{
    if (enable) {
        camera_->FocusSettings.FocusMethod = ECameraFocusMethod::Manual;
    }
    else {
        camera_->FocusSettings.FocusMethod = ECameraFocusMethod::Disable;
    }
    copyCameraSettingsToAllSceneCapture(camera_);
}

float APIPCamera::getFocusDistance() const
{
    return camera_->FocusSettings.ManualFocusDistance;
}

void APIPCamera::setFocusDistance(float focus_distance)
{
    camera_->FocusSettings.ManualFocusDistance = focus_distance;
    copyCameraSettingsToAllSceneCapture(camera_);
}

float APIPCamera::getFocusAperture() const
{
    return camera_->CurrentAperture;
}

void APIPCamera::setFocusAperture(float focus_aperture)
{
    camera_->CurrentAperture = focus_aperture;
    copyCameraSettingsToAllSceneCapture(camera_);
}

void APIPCamera::enableFocusPlane(bool enable)
{
#if WITH_EDITOR
    camera_->FocusSettings.bDrawDebugFocusPlane = enable;
#endif
}

std::string APIPCamera::getCurrentFieldOfView() const
{
    std::ostringstream field_of_view_string;
    field_of_view_string << "Current Field Of View:\n\tHorizontal Field Of View: " << camera_->GetHorizontalFieldOfView() << ";\n\t Vertical Field Of View: " << camera_->GetVerticalFieldOfView();
    return field_of_view_string.str();
}

void APIPCamera::copyCameraSettingsToAllSceneCapture(UCameraComponent* camera)
{
    int image_count = static_cast<int>(cameraCaptureCount());
    for (int image_type = image_count - 1; image_type >= 0; image_type--) {
        copyCameraSettingsToSceneCapture(camera_, captures_[image_type]);
    }
}

void APIPCamera::copyCameraSettingsToSceneCapture(UCameraComponent* src, USceneCaptureComponent2D* dst)
{
    if (src && dst) {
        dst->SetWorldLocationAndRotation(src->GetComponentLocation(), src->GetComponentRotation());

        FMinimalViewInfo camera_view_info;
        src->GetCameraView(/*DeltaTime =*/0.0f, camera_view_info);

        const FPostProcessSettings& src_pp_settings = camera_view_info.PostProcessSettings;
        FPostProcessSettings& dst_pp_settings = dst->PostProcessSettings;

        FWeightedBlendables dst_weighted_blendables = dst_pp_settings.WeightedBlendables;

        // Copy all of the post processing settings
        dst_pp_settings = src_pp_settings;

        // But restore the original blendables
        dst_pp_settings.WeightedBlendables = dst_weighted_blendables;
    }
}

//end CinemAirSim methods
