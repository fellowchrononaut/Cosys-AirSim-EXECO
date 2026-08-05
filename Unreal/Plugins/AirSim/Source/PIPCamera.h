#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Camera/CameraActor.h"
#include "Materials/Material.h"
#include "Runtime/Core/Public/PixelFormat.h"
#include "Annotation/ObjectAnnotator.h"
#include "common/ImageCaptureBase.hpp"
#include "common/common_utils/Utils.hpp"
#include "Components/StaticMeshComponent.h"
#include <Engine/StaticMesh.h>
#include "common/AirSimSettings.hpp"
#include "NedTransform.h"
#include "DetectionComponent.h"
#include "CubeResample.h"

//CinemAirSim
#include <CineCameraActor.h>
#include <CineCameraComponent.h>
#include "Materials/MaterialParameterCollection.h"
#include "Materials/MaterialParameterCollectionInstance.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "PIPCamera.generated.h"

UCLASS()
class AIRSIM_API APIPCamera : public ACineCameraActor //CinemAirSim
{
    GENERATED_BODY()    

public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
    typedef msr::airlib::AirSimSettings AirSimSettings;
    typedef AirSimSettings::CameraSetting CameraSetting;

    APIPCamera(const FObjectInitializer& ObjectInitializer); //CinemAirSim

    virtual void PostInitializeComponents() override;
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    void showToScreen();
    void disableAll();
    void disableAllPIP();
    void disableMain();
    void onViewModeChanged(bool nodisplay);

    //CinemAirSim methods
    std::vector<std::string> getPresetLensSettings() const;
    void setPresetLensSettings(std::string preset_string);
    std::vector<std::string> getPresetFilmbackSettings() const;
    void setPresetFilmbackSettings(std::string preset_string);
    std::string getLensSettings() const;
    std::string getFilmbackSettings() const;
    float setFilmbackSettings(float sensor_width, float sensor_height);
    float getFocalLength() const;
    void setFocalLength(float focal_length);
    void enableManualFocus(bool enable);
    float getFocusDistance() const;
    void setFocusDistance(float focus_distance);
    float getFocusAperture() const;
    void setFocusAperture(float focus_aperture);
    void enableFocusPlane(bool enable);
    std::string getCurrentFieldOfView() const;
    //end CinemAirSim methods

    void setCameraTypeEnabled(ImageType type, bool enabled, std::string annotation_name = "");
    bool getCameraTypeEnabled(ImageType type, std::string annotation_name = "") const;
    AirSimSettings::CameraSetting getParams() const;
    void setCaptureUpdate(USceneCaptureComponent2D* capture, bool nodisplay);

    // I-G. Force bCaptureEveryFrame on every capture component this camera owns, without touching
    // viewport rendering the way the NoDisplay view mode does. Driven by airsim.CaptureEveryFrame;
    // see PIPCamera.cpp. simGetImages does not need per-frame capture - it issues its own
    // CaptureSceneDeferred - so turning this off is a candidate fix, not only a diagnostic.
    void setAllCapturesEveryFrame(bool enabled);
    void setCameraTypeUpdate(ImageType type, bool nodisplay, std::string annotation_name = "");
    void setCameraOrientation(const FRotator& rotator);
    void updateInstanceSegmentationAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList, bool only_hide=false);
    bool GetAnnotationNameExist(std::string annotation_name);
    void updateAnnotation(TArray<TWeakObjectPtr<UPrimitiveComponent> >& ComponentList, FString annotation_name, bool only_hide = false);
    void addAnnotationCamera(FString name, FObjectAnnotator::AnnotatorType type, float max_view_distance = -1.0f);
    void setupCameraFromSettings(const APIPCamera::CameraSetting& camera_setting, const NedTransform& ned_transform);
    void setCameraPose(const msr::airlib::Pose& relative_pose);
    void setCameraFoV(float fov_degrees);
    msr::airlib::CameraInfo getCameraInfo() const;
    std::vector<float> getDistortionParams() const;
    void setDistortionParam(const std::string& param_name, float value);

    msr::airlib::ProjectionMatrix getProjectionMatrix() const;

    USceneCaptureComponent2D* getCaptureComponent(const ImageType type, bool if_active, std::string annotation_name = "");
    UTextureRenderTarget2D* getRenderTarget(const ImageType type, bool if_active, std::string annotation_name = "");
    UDetectionComponent* getDetectionComponent(const ImageType type, bool if_active, std::string annotation_name = "") const;

    // Generic (non-pinhole) camera support - Phase 3b step 3. The cube face rig is six
    // USceneCaptureComponent2D per ImageType (decision D13), built only for a camera whose
    // settings carry a CameraModel block and only for the ImageTypes that camera actually uses.
    // Absent that block hasCameraModel() is false, the two arrays below stay empty and not one
    // component or render target is ever constructed. The face-orientation convention is stated
    // in full above getCubeFaceRotation() in PIPCamera.cpp - read it before consuming a face.
    bool hasCameraModel() const;
    int getCubeFaceCount() const;
    int getCubeFaceResolution() const;
    static const TCHAR* getCubeFaceName(int face);
    USceneCaptureComponent2D* getFaceCaptureComponent(const ImageType type, int face);
    UTextureRenderTarget2D* getFaceRenderTarget(const ImageType type, int face);

    // Phase 3b step 4. The camera's raymap as a GPU resource, built once at configuration time.
    // Invalid - and therefore silently the pinhole path - for every camera without a CameraModel
    // block, and also for a camera whose CameraModel and Scene CaptureSettings resolutions
    // disagree, which buildRaymapResource() refuses loudly rather than resampling something that
    // is not the calibrated camera.
    const FAirSimRaymapResourcePtr& getRaymapResource() const;

    msr::airlib::Pose getPose() const;

private: //members
    UPROPERTY()
    UMaterialParameterCollection* distortion_param_collection_;
    UPROPERTY()
    UMaterialParameterCollectionInstance* distortion_param_instance_;

    UPROPERTY()
    TArray<USceneCaptureComponent2D*> captures_;
    UPROPERTY()
    TArray<UTextureRenderTarget2D*> render_targets_;
    UPROPERTY()
    TArray<UDetectionComponent*> detections_;

    // Cube face rig, flat and indexed image_type * kCubeFaceCount + face. Both stay Num() == 0
    // for a camera with no CameraModel block, which is where the non-invasiveness of this
    // feature is enforced: an empty TArray allocates nothing.
    UPROPERTY()
    TArray<USceneCaptureComponent2D*> face_captures_;
    UPROPERTY()
    TArray<UTextureRenderTarget2D*> face_render_targets_;

    // Per-camera raymap on the GPU (step 4). Not a UPROPERTY: it holds RHI references, not
    // UObjects. Stays null for a camera with no CameraModel block, so an ordinary camera pays
    // one null pointer.
    FAirSimRaymapResourcePtr raymap_;

    UPROPERTY() UStaticMesh* annotation_sphere_static_;

    //CinemAirSim
    UPROPERTY()
    UCineCameraComponent* camera_;
    //TMap<int, UMaterialInstanceDynamic*> noise_materials_;
    //below is needed because TMap doesn't work with UPROPERTY, but we do have -ve index
    UPROPERTY()
    TArray<UMaterialInstanceDynamic*> noise_materials_;
    UPROPERTY()
    TArray<UMaterialInstanceDynamic*> distortion_materials_;
    UPROPERTY()
    UMaterial* noise_material_static_;
    UPROPERTY()
    UMaterial* distortion_material_static_;

    // TODO: See if this can be replaced/removed
    UPROPERTY() UMaterial* lens_distortion_material_static_;
	UPROPERTY() UMaterial* lens_distortion_invert_material_static_;
	UPROPERTY() TArray<UMaterialInstanceDynamic*> lens_distortion_materials_;

    UPROPERTY() UMaterial* motion_blur_material_static_;
    UPROPERTY() UMaterial* radial_blur_material_static_;
    UPROPERTY() UMaterial* guassian_blur_material_static_;

    UPROPERTY() TArray<UMaterialInstanceDynamic*> fake_motion_blur_materials_;
    UPROPERTY() TArray<UMaterialInstanceDynamic*> guassian_blur_materials_;
    UPROPERTY() TArray<UMaterialInstanceDynamic*> radial_blur_materials_;

    TMap<FString, int> annotator_name_to_index_map_;
    TMap<FString, TWeakObjectPtr<UPrimitiveComponent>> sphere_annotation_component_map_;

    std::vector<bool> camera_type_enabled_;
    FRotator gimbald_rotator_;
    float gimbal_stabilization_;
    const NedTransform* ned_transform_;
    TMap<int, EPixelFormat> image_type_to_pixel_format_map_;

    FObjectFilter object_filter_;

    msr::airlib::AirSimSettings::CameraSetting sensor_params_;

    TArray<AActor*> ignore_actors_;
private: //methods
    typedef common_utils::Utils Utils;
    typedef AirSimSettings::CaptureSetting CaptureSetting;
    typedef AirSimSettings::NoiseSetting NoiseSetting;

    static constexpr int kCubeFaceCount = 6;

    static unsigned int imageTypeCount();
    unsigned int cameraCaptureCount();
    void enableCaptureComponent(const ImageType type, bool is_enabled, std::string annotation_name = "");
    static FRotator getCubeFaceRotation(int face);
    void ensureFaceRig(const ImageType type);
    void setFaceRigEnabled(const ImageType type, bool is_enabled);
    void buildRaymapResource();
    static void updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target,
                                              bool auto_format, const EPixelFormat& pixel_format, const CaptureSetting& setting, const NedTransform& ned_transform,
                                              bool force_linear_gamma);
    void setNoiseMaterial(int image_type, UObject* outer, FPostProcessSettings& obj, const NoiseSetting& settings);
    void setDistortionMaterial(int image_type, UObject* outer, FPostProcessSettings& obj);
    static void updateCameraPostProcessingSetting(FPostProcessSettings& obj, const CaptureSetting& setting);
    //CinemAirSim
    static void updateCameraSetting(UCineCameraComponent* camera, const CaptureSetting& setting, const NedTransform& ned_transform);
    void copyCameraSettingsToAllSceneCapture(UCameraComponent* camera);
    void copyCameraSettingsToSceneCapture(UCameraComponent* src, USceneCaptureComponent2D* dst);
    //end CinemAirSim
};
