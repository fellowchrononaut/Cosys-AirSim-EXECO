#include "StartupCameraSettingsEditor.h"

#include "common/common_utils/json.hpp"

#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"

#include "Misc/MessageDialog.h"
#include "Misc/Paths.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SMultiLineEditableTextBox.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace airsim_startup
{
namespace
{
    using CameraSettingsJson = nlohmann::json;

    const char* CameraSettingsCollectionKey(EStartupCameraSettingsCollection collection)
    {
        return collection == EStartupCameraSettingsCollection::Capture ? "CaptureSettings" : "NoiseSettings";
    }

    FString CameraSettingsCollectionLabel(EStartupCameraSettingsCollection collection)
    {
        return UTF8_TO_TCHAR(CameraSettingsCollectionKey(collection));
    }

    bool ParseCameraSettingsDocument(const FString& text, CameraSettingsJson& document, FString& error)
    {
        try
        {
            document = CameraSettingsJson::parse(TCHAR_TO_UTF8(*text));
            if (!document.is_object())
            {
                error = TEXT("Settings root must be a JSON object.");
                return false;
            }
            return true;
        }
        catch (const std::exception& exception)
        {
            error = UTF8_TO_TCHAR(exception.what());
            return false;
        }
    }

    CameraSettingsJson* FindCameraSettingsCamera(CameraSettingsJson& document, const FString& vehicle_name,
                                                 const FString& camera_name, FString& error)
    {
        if (vehicle_name.IsEmpty() || camera_name.IsEmpty())
        {
            error = TEXT("A vehicle and camera must be selected.");
            return nullptr;
        }
        if (!document.contains("Vehicles"))
        {
            error = TEXT("Settings document has no Vehicles object.");
            return nullptr;
        }
        if (!document["Vehicles"].is_object())
        {
            error = TEXT("Settings Vehicles must be a JSON object.");
            return nullptr;
        }
        const std::string vehicle_key = TCHAR_TO_UTF8(*vehicle_name);
        if (!document["Vehicles"].contains(vehicle_key))
        {
            error = FString::Printf(TEXT("Vehicle '%s' was not found."), *vehicle_name);
            return nullptr;
        }
        if (!document["Vehicles"][vehicle_key].is_object())
        {
            error = FString::Printf(TEXT("Vehicle '%s' must be a JSON object."), *vehicle_name);
            return nullptr;
        }
        CameraSettingsJson& vehicle = document["Vehicles"][vehicle_key];
        if (!vehicle.contains("Cameras"))
        {
            error = FString::Printf(TEXT("Vehicle '%s' has no Cameras object."), *vehicle_name);
            return nullptr;
        }
        if (!vehicle["Cameras"].is_object())
        {
            error = FString::Printf(TEXT("Vehicle '%s' Cameras must be a JSON object."), *vehicle_name);
            return nullptr;
        }
        const std::string camera_key = TCHAR_TO_UTF8(*camera_name);
        if (!vehicle["Cameras"].contains(camera_key))
        {
            error = FString::Printf(TEXT("Camera '%s' was not found on vehicle '%s'."), *camera_name, *vehicle_name);
            return nullptr;
        }
        if (!vehicle["Cameras"][camera_key].is_object())
        {
            error = FString::Printf(TEXT("Camera '%s' must be a JSON object."), *camera_name);
            return nullptr;
        }
        return &vehicle["Cameras"][camera_key];
    }

    CameraSettingsJson* FindCameraSettingsArray(CameraSettingsJson& document, const FString& vehicle_name,
                                                const FString& camera_name, EStartupCameraSettingsCollection collection,
                                                bool create, FString& error)
    {
        CameraSettingsJson* camera = FindCameraSettingsCamera(document, vehicle_name, camera_name, error);
        if (camera == nullptr)
            return nullptr;
        const char* key = CameraSettingsCollectionKey(collection);
        if (!camera->contains(key))
        {
            if (!create)
                return nullptr;
            (*camera)[key] = CameraSettingsJson::array();
        }
        if (!(*camera)[key].is_array())
        {
            error = FString::Printf(TEXT("%s must be a JSON array."), *CameraSettingsCollectionLabel(collection));
            return nullptr;
        }
        return &(*camera)[key];
    }

    bool TryGetCameraImageType(const CameraSettingsJson& entry, int32& image_type)
    {
        if (!entry.contains("ImageType"))
        {
            image_type = 0;
            return true;
        }
        if (!entry["ImageType"].is_number_integer())
            return false;
        try
        {
            image_type = entry["ImageType"].get<int32>();
            return true;
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    bool ValidateCameraEntries(const CameraSettingsJson& array, EStartupCameraSettingsCollection collection, FString& error)
    {
        for (int32 index = 0; index < static_cast<int32>(array.size()); ++index)
        {
            const CameraSettingsJson& entry = array[index];
            if (!entry.is_object())
            {
                error = FString::Printf(TEXT("%s entry %d must be a JSON object."), *CameraSettingsCollectionLabel(collection), index);
                return false;
            }
            int32 image_type = 0;
            if (!TryGetCameraImageType(entry, image_type))
            {
                error = FString::Printf(TEXT("%s entry %d ImageType must be an int32 when present."), *CameraSettingsCollectionLabel(collection), index);
                return false;
            }
        }
        return true;
    }

    bool HasCameraImageType(const CameraSettingsJson& array, int32 image_type, int32 ignored_index,
                            EStartupCameraSettingsCollection collection, FString& error)
    {
        for (int32 index = 0; index < static_cast<int32>(array.size()); ++index)
        {
            if (index == ignored_index)
                continue;
            int32 effective_type = 0;
            if (!TryGetCameraImageType(array[index], effective_type))
            {
                error = FString::Printf(TEXT("%s entry %d ImageType must be an int32 when present."), *CameraSettingsCollectionLabel(collection), index);
                return false;
            }
            if (effective_type == image_type)
                return true;
        }
        return false;
    }

    bool ParseCameraInteger(const FString& text, int32& value)
    {
        const std::string raw = TCHAR_TO_UTF8(*text);
        if (raw.empty() || std::any_of(raw.begin(), raw.end(), [](unsigned char character) { return std::isspace(character) != 0; }))
            return false;
        char* end = nullptr;
        errno = 0;
        const long parsed = std::strtol(raw.c_str(), &end, 10);
        if (errno == ERANGE || end == raw.c_str() || *end != '\0' || parsed < std::numeric_limits<int32>::min() || parsed > std::numeric_limits<int32>::max())
            return false;
        value = static_cast<int32>(parsed);
        return true;
    }

    bool ParseCameraNumber(const FString& text, double& value)
    {
        const std::string raw = TCHAR_TO_UTF8(*text);
        if (raw.empty() || std::any_of(raw.begin(), raw.end(), [](unsigned char character) { return std::isspace(character) != 0; }))
            return false;
        char* end = nullptr;
        errno = 0;
        value = std::strtod(raw.c_str(), &end);
        return errno != ERANGE && end != raw.c_str() && *end == '\0' && std::isfinite(value);
    }

    FString CameraImageTypeLabel(int32 image_type)
    {
        static const int32 values[] = {-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        static const TCHAR* labels[] = {
            TEXT("Main camera (-1)"), TEXT("Scene (0)"), TEXT("DepthPlanar (1)"), TEXT("DepthPerspective (2)"),
            TEXT("DepthVis (3)"), TEXT("DisparityNormalized (4)"), TEXT("Segmentation (5)"), TEXT("SurfaceNormals (6)"),
            TEXT("Infrared (7)"), TEXT("OpticalFlow (8)"), TEXT("OpticalFlowVis (9)"), TEXT("Lighting (10)"), TEXT("Annotation (11)")};
        for (int32 index = 0; index < UE_ARRAY_COUNT(values); ++index)
            if (values[index] == image_type)
                return labels[index];
        return FString::Printf(TEXT("Unknown (%d)"), image_type);
    }

    FString SerializeCameraSettingsDocument(const CameraSettingsJson& document)
    {
        return UTF8_TO_TCHAR(document.dump(2).c_str());
    }

    struct FCameraFieldDefinition
    {
        const TCHAR* Label;
        const TCHAR* Key;
        EStartupCameraSettingType Type;
        const TCHAR* Group;
    };

#define CAMERA_INT(label, key, group) {TEXT(label), TEXT(key), EStartupCameraSettingType::Integer, TEXT(group)}
#define CAMERA_NUMBER(label, key, group) {TEXT(label), TEXT(key), EStartupCameraSettingType::Number, TEXT(group)}
#define CAMERA_BOOL(label, key, group) {TEXT(label), TEXT(key), EStartupCameraSettingType::Boolean, TEXT(group)}
#define CAMERA_IMAGE(label, key, group) {TEXT(label), TEXT(key), EStartupCameraSettingType::ImageType, TEXT(group)}
#define CAMERA_PROJECTION(label, key, group) {TEXT(label), TEXT(key), EStartupCameraSettingType::ProjectionMode, TEXT(group)}

    const FCameraFieldDefinition GCaptureFields[] = {
        CAMERA_IMAGE("ImageType", "ImageType", "Core capture"),
        CAMERA_INT("Width", "Width", "Core capture"), CAMERA_INT("Height", "Height", "Core capture"),
        CAMERA_NUMBER("FOV degrees", "FOV_Degrees", "Core capture"), CAMERA_BOOL("Force update", "ForceUpdate", "Core capture"),
        CAMERA_NUMBER("Target gamma", "TargetGamma", "Core capture"), CAMERA_PROJECTION("Projection mode", "ProjectionMode", "Core capture"),
        CAMERA_NUMBER("Ortho width", "OrthoWidth", "Core capture"), CAMERA_BOOL("Ignore marked", "IgnoreMarked", "Core capture"),
        CAMERA_BOOL("Lumen GI enabled", "LumenGIEnable", "Lumen"), CAMERA_BOOL("Lumen reflections enabled", "LumenReflectionEnable", "Lumen"),
        CAMERA_NUMBER("Lumen final quality", "LumenFinalQuality", "Lumen"), CAMERA_NUMBER("Lumen scene detail", "LumenSceneDetail", "Lumen"),
        CAMERA_NUMBER("Lumen scene lightning detail", "LumenSceneLightningDetail", "Lumen"),
        CAMERA_INT("Auto exposure method", "AutoExposureMethod", "Exposure"), CAMERA_NUMBER("Auto exposure compensation", "AutoExposureCompensation", "Exposure"),
        CAMERA_BOOL("Apply physical camera exposure", "AutoExposureApplyPhysicalCameraExposure", "Exposure"),
        CAMERA_NUMBER("Auto exposure min brightness", "AutoExposureMinBrightness", "Exposure"), CAMERA_NUMBER("Auto exposure max brightness", "AutoExposureMaxBrightness", "Exposure"),
        CAMERA_NUMBER("Auto exposure speed up", "AutoExposureSpeedUp", "Exposure"), CAMERA_NUMBER("Auto exposure speed down", "AutoExposureSpeedDown", "Exposure"),
        CAMERA_NUMBER("Auto exposure low percent", "AutoExposureLowPercent", "Exposure"), CAMERA_NUMBER("Auto exposure high percent", "AutoExposureHighPercent", "Exposure"),
        CAMERA_NUMBER("Auto exposure histogram log min", "AutoExposureHistogramLogMin", "Exposure"), CAMERA_NUMBER("Auto exposure histogram log max", "AutoExposureHistogramLogMax", "Exposure"),
        CAMERA_NUMBER("Motion blur amount", "MotionBlurAmount", "Motion and bloom"), CAMERA_NUMBER("Motion blur max", "MotionBlurMax", "Motion and bloom"),
        CAMERA_NUMBER("Motion blur target FPS", "MotionBlurTargetFPS", "Motion and bloom"), CAMERA_NUMBER("Bloom intensity", "BloomIntensity", "Motion and bloom"),
        CAMERA_NUMBER("Bloom threshold", "BloomThreshold", "Motion and bloom"), CAMERA_NUMBER("Chromatic aberration intensity", "ChromaticAberrationIntensity", "Motion and bloom"),
        CAMERA_NUMBER("Chromatic aberration start offset", "ChromaticAberrationStartOffset", "Motion and bloom"),
        CAMERA_NUMBER("Camera shutter speed", "CameraShutterSpeed", "Physical camera"), CAMERA_NUMBER("Camera ISO", "CameraISO", "Physical camera"),
        CAMERA_NUMBER("Camera aperture", "CameraAperture", "Physical camera"), CAMERA_NUMBER("Camera max aperture", "CameraMaxAperture", "Physical camera"),
        CAMERA_NUMBER("Camera number of blades", "CameraNumBlades", "Physical camera"), CAMERA_NUMBER("Lens flare intensity", "LensFlareIntensity", "Lens flare"),
        CAMERA_NUMBER("Lens flare bokeh size", "LensFlareBokehSize", "Lens flare"), CAMERA_NUMBER("Lens flare threshold", "LensFlareThreshold", "Lens flare"),
        CAMERA_NUMBER("DOF sensor width", "DepthOfFieldSensorWidth", "Depth of field"), CAMERA_NUMBER("DOF squeeze factor", "DepthOfFieldSqueezeFactor", "Depth of field"),
        CAMERA_NUMBER("DOF focal distance", "DepthOfFieldFocalDistance", "Depth of field"), CAMERA_NUMBER("DOF depth blur amount", "DepthOfFieldDepthBlurAmount", "Depth of field"),
        CAMERA_NUMBER("DOF depth blur radius", "DepthOfFieldDepthBlurRadius", "Depth of field"), CAMERA_NUMBER("DOF use hair depth", "DepthOfFieldUseHairDepth", "Depth of field")
    };

    const FCameraFieldDefinition GNoiseFields[] = {
        CAMERA_BOOL("Enabled", "Enabled", "Noise"), CAMERA_IMAGE("ImageType", "ImageType", "Noise"),
        CAMERA_NUMBER("Horizontal wave strength", "HorzWaveStrength", "Horizontal wave"), CAMERA_NUMBER("Random speed", "RandSpeed", "Horizontal wave"),
        CAMERA_NUMBER("Random size", "RandSize", "Horizontal wave"), CAMERA_NUMBER("Random density", "RandDensity", "Horizontal wave"),
        CAMERA_NUMBER("Random contribution", "RandContrib", "Horizontal wave"), CAMERA_NUMBER("Horizontal wave contribution", "HorzWaveContrib", "Horizontal wave"),
        CAMERA_NUMBER("Horizontal wave vertical size", "HorzWaveVertSize", "Horizontal wave"), CAMERA_NUMBER("Horizontal wave screen size", "HorzWaveScreenSize", "Horizontal wave"),
        CAMERA_NUMBER("Horizontal noise lines contribution", "HorzNoiseLinesContrib", "Horizontal noise lines"), CAMERA_NUMBER("Horizontal noise lines density Y", "HorzNoiseLinesDensityY", "Horizontal noise lines"),
        CAMERA_NUMBER("Horizontal noise lines density XY", "HorzNoiseLinesDensityXY", "Horizontal noise lines"), CAMERA_NUMBER("Horizontal distortion strength", "HorzDistortionStrength", "Horizontal distortion"),
        CAMERA_NUMBER("Horizontal distortion contribution", "HorzDistortionContrib", "Horizontal distortion"),
        CAMERA_BOOL("Lens distortion enabled", "LensDistortionEnable", "Lens distortion"), CAMERA_NUMBER("Lens distortion area falloff", "LensDistortionAreaFalloff", "Lens distortion"),
        CAMERA_NUMBER("Lens distortion area radius", "LensDistortionAreaRadius", "Lens distortion"), CAMERA_NUMBER("Lens distortion intensity", "LensDistortionIntensity", "Lens distortion"),
        CAMERA_BOOL("Lens distortion invert", "LensDistortionInvert", "Lens distortion"),
        CAMERA_BOOL("Fake motion blur enabled", "FakeMotionBlurEnable", "Fake motion blur"), CAMERA_NUMBER("Fake motion blur direction X", "FakeMotionBlurDirectionX", "Fake motion blur"),
        CAMERA_NUMBER("Fake motion blur direction Y", "FakeMotionBlurDirectionY", "Fake motion blur"), CAMERA_NUMBER("Fake motion blur movement speed", "FakeMotionBlurMovementSpeed", "Fake motion blur"),
        CAMERA_NUMBER("Fake motion blur shutter speed", "FakeMotionBlurShutterSpeed", "Fake motion blur"), CAMERA_NUMBER("Fake motion blur focal length", "FakeMotionBlurFocalLength", "Fake motion blur"),
        CAMERA_INT("Fake motion blur samples", "FakeMotionBlurSamples", "Fake motion blur"),
        CAMERA_BOOL("Radial blur enabled", "RadialBlurEnable", "Radial blur"), CAMERA_NUMBER("Radial blur distance", "RadialBlurDistance", "Radial blur"),
        CAMERA_NUMBER("Radial blur radius", "RadialBlurRadius", "Radial blur"), CAMERA_NUMBER("Radial blur density", "RadialBlurDensity", "Radial blur"),
        CAMERA_BOOL("Guassian blur enabled", "GuassianBlurEnable", "Guassian blur"), CAMERA_NUMBER("Guassian blur directions", "GuassianBlurDirections", "Guassian blur"),
        CAMERA_NUMBER("Guassian blur quality", "GuassianBlurQuality", "Guassian blur"), CAMERA_NUMBER("Guassian blur size", "GuassianBlurSize", "Guassian blur")
    };

    struct FCameraModelFieldDefinition
    {
        const TCHAR* Label;
        const TCHAR* Key;
        EStartupCameraModelFieldType Type;
        const TCHAR* Group;
        const TCHAR* ModelType;
    };

#define MODEL_INT(label, key, group, model) {TEXT(label), TEXT(key), EStartupCameraModelFieldType::Integer, TEXT(group), TEXT(model)}
#define MODEL_NUMBER(label, key, group, model) {TEXT(label), TEXT(key), EStartupCameraModelFieldType::Number, TEXT(group), TEXT(model)}
#define MODEL_STRING(label, key, group, model) {TEXT(label), TEXT(key), EStartupCameraModelFieldType::String, TEXT(group), TEXT(model)}
#define MODEL_TYPE(label, key, group, model) {TEXT(label), TEXT(key), EStartupCameraModelFieldType::Type, TEXT(group), TEXT(model)}
#define MODEL_FACES(label, key, group, model) {TEXT(label), TEXT(key), EStartupCameraModelFieldType::Faces, TEXT(group), TEXT(model)}
#define MODEL_BACKEND(label, key, group, model) {TEXT(label), TEXT(key), EStartupCameraModelFieldType::Backend, TEXT(group), TEXT(model)}

    const FCameraModelFieldDefinition GCameraModelFields[] = {
        MODEL_TYPE("Type", "Type", "Model", ""),
        MODEL_INT("Width", "Width", "Calibration", "not-raymap"), MODEL_INT("Height", "Height", "Calibration", "not-raymap"),
        MODEL_NUMBER("fx", "fx", "Calibration", "not-raymap"), MODEL_NUMBER("fy", "fy", "Calibration", "not-raymap"),
        MODEL_NUMBER("cx", "cx", "Calibration", "not-raymap"), MODEL_NUMBER("cy", "cy", "Calibration", "not-raymap"),
        MODEL_NUMBER("FOV degrees", "FOV_Degrees", "Calibration", "Pinhole"),
        MODEL_NUMBER("k1", "k1", "Kannala-Brandt", "KannalaBrandt"), MODEL_NUMBER("k2", "k2", "Kannala-Brandt", "KannalaBrandt"),
        MODEL_NUMBER("k3", "k3", "Kannala-Brandt", "KannalaBrandt"), MODEL_NUMBER("k4", "k4", "Kannala-Brandt", "KannalaBrandt"),
        MODEL_NUMBER("xi", "xi", "Double Sphere", "DoubleSphere"), MODEL_NUMBER("alpha", "alpha", "Double Sphere", "DoubleSphere"),
        MODEL_STRING("Path", "Path", "Raymap", "Raymap"),
        MODEL_INT("Cube face resolution (0 = auto)", "CubeFaceResolution", "Rendering", ""),
        MODEL_FACES("Faces", "Faces", "Rendering", ""), MODEL_BACKEND("Render backend", "RenderBackend", "Rendering", "")
    };

#undef MODEL_INT
#undef MODEL_NUMBER
#undef MODEL_STRING
#undef MODEL_TYPE
#undef MODEL_FACES
#undef MODEL_BACKEND

#undef CAMERA_INT
#undef CAMERA_NUMBER
#undef CAMERA_BOOL
#undef CAMERA_IMAGE
#undef CAMERA_PROJECTION
}

bool FStartupCameraSettingsModel::ListEntries(const FString& document_text, const FString& vehicle_name,
                                              const FString& camera_name, EStartupCameraSettingsCollection collection,
                                              TArray<FStartupCameraEntryInfo>& entries, FString& error)
{
    error.Reset();
    entries.Reset();
    CameraSettingsJson document;
    if (!ParseCameraSettingsDocument(document_text, document, error))
        return false;
    CameraSettingsJson* array = FindCameraSettingsArray(document, vehicle_name, camera_name, collection, false, error);
    if (array == nullptr)
    {
        if (error.IsEmpty())
        {
            entries.Reset();
            return true;
        }
        return false;
    }
    if (!ValidateCameraEntries(*array, collection, error))
        return false;
    TArray<FStartupCameraEntryInfo> listed_entries;
    for (int32 index = 0; index < static_cast<int32>(array->size()); ++index)
    {
        FStartupCameraEntryInfo info;
        info.Index = index;
        TryGetCameraImageType((*array)[index], info.ImageType);
        info.Label = FString::Printf(TEXT("[%d] %s"), index, *CameraImageTypeLabel(info.ImageType));
        listed_entries.Add(info);
    }
    entries = MoveTemp(listed_entries);
    return true;
}

bool FStartupCameraSettingsModel::AddEntry(const FString& document_text, const FString& vehicle_name,
                                           const FString& camera_name, EStartupCameraSettingsCollection collection,
                                           int32 image_type, FString& updated_text, FString& error)
{
    error.Reset();
    CameraSettingsJson document;
    if (!ParseCameraSettingsDocument(document_text, document, error))
        return false;
    CameraSettingsJson* array = FindCameraSettingsArray(document, vehicle_name, camera_name, collection, true, error);
    if (array == nullptr)
        return false;
    if (!ValidateCameraEntries(*array, collection, error))
        return false;
    if (HasCameraImageType(*array, image_type, INDEX_NONE, collection, error))
    {
        error = FString::Printf(TEXT("ImageType %d already exists in %s."), image_type, *CameraSettingsCollectionLabel(collection));
        return false;
    }
    CameraSettingsJson entry = CameraSettingsJson::object();
    entry["ImageType"] = image_type;
    array->push_back(entry);
    updated_text = SerializeCameraSettingsDocument(document);
    return true;
}

bool FStartupCameraSettingsModel::DuplicateEntry(const FString& document_text, const FString& vehicle_name,
                                                 const FString& camera_name, EStartupCameraSettingsCollection collection,
                                                 int32 source_index, int32 target_image_type,
                                                 FString& updated_text, FString& error)
{
    error.Reset();
    CameraSettingsJson document;
    if (!ParseCameraSettingsDocument(document_text, document, error))
        return false;
    CameraSettingsJson* array = FindCameraSettingsArray(document, vehicle_name, camera_name, collection, false, error);
    if (array == nullptr)
    {
        if (error.IsEmpty()) error = TEXT("The settings collection is absent.");
        return false;
    }
    if (!ValidateCameraEntries(*array, collection, error))
        return false;
    if (source_index < 0 || source_index >= static_cast<int32>(array->size()) || !(*array)[source_index].is_object())
    {
        error = TEXT("Select a valid settings entry first.");
        return false;
    }
    if (HasCameraImageType(*array, target_image_type, INDEX_NONE, collection, error))
    {
        error = FString::Printf(TEXT("ImageType %d already exists in %s."), target_image_type, *CameraSettingsCollectionLabel(collection));
        return false;
    }
    CameraSettingsJson duplicate = (*array)[source_index];
    duplicate["ImageType"] = target_image_type;
    array->push_back(duplicate);
    updated_text = SerializeCameraSettingsDocument(document);
    return true;
}

bool FStartupCameraSettingsModel::DeleteEntry(const FString& document_text, const FString& vehicle_name,
                                              const FString& camera_name, EStartupCameraSettingsCollection collection,
                                              int32 index, FString& updated_text, FString& error)
{
    error.Reset();
    CameraSettingsJson document;
    if (!ParseCameraSettingsDocument(document_text, document, error))
        return false;
    CameraSettingsJson* array = FindCameraSettingsArray(document, vehicle_name, camera_name, collection, false, error);
    if (array == nullptr)
    {
        if (error.IsEmpty()) error = TEXT("The settings collection is absent.");
        return false;
    }
    if (!ValidateCameraEntries(*array, collection, error))
        return false;
    if (index < 0 || index >= static_cast<int32>(array->size()) || !(*array)[index].is_object())
    {
        error = TEXT("Select a valid settings entry first.");
        return false;
    }
    array->erase(array->begin() + index);
    if (array->empty())
    {
        CameraSettingsJson* camera = FindCameraSettingsCamera(document, vehicle_name, camera_name, error);
        if (camera == nullptr)
            return false;
        camera->erase(CameraSettingsCollectionKey(collection));
    }
    updated_text = SerializeCameraSettingsDocument(document);
    return true;
}

bool FStartupCameraSettingsModel::MutateField(const FString& document_text, const FString& vehicle_name,
                                              const FString& camera_name, EStartupCameraSettingsCollection collection,
                                              int32 index, const TCHAR* key, EStartupCameraSettingType type,
                                              int32 enum_value, bool omit, bool keep_current, const FString& scalar_text,
                                              bool clear, FString& updated_text, FString& error)
{
    error.Reset();
    if (key == nullptr || *key == TEXT('\0'))
    {
        error = TEXT("A field key is required.");
        return false;
    }
    CameraSettingsJson document;
    if (!ParseCameraSettingsDocument(document_text, document, error))
        return false;
    CameraSettingsJson* array = FindCameraSettingsArray(document, vehicle_name, camera_name, collection, false, error);
    if (array == nullptr || index < 0 || index >= static_cast<int32>(array->size()) || !(*array)[index].is_object())
    {
        if (error.IsEmpty()) error = TEXT("Select a valid settings entry first.");
        return false;
    }
    if (!ValidateCameraEntries(*array, collection, error))
        return false;
    if (clear || omit)
    {
        if (type == EStartupCameraSettingType::ImageType && HasCameraImageType(*array, 0, index, collection, error))
        {
            error = FString::Printf(TEXT("ImageType 0 already exists in %s."), *CameraSettingsCollectionLabel(collection));
            return false;
        }
        (*array)[index].erase(TCHAR_TO_UTF8(key));
        updated_text = SerializeCameraSettingsDocument(document);
        return true;
    }
    if (keep_current)
    {
        updated_text = document_text;
        return true;
    }
    CameraSettingsJson& entry = (*array)[index];
    const std::string json_key = TCHAR_TO_UTF8(key);
    if (type == EStartupCameraSettingType::Boolean)
    {
        if (enum_value != 0 && enum_value != 1)
        {
            error = TEXT("Boolean selection must be 0 or 1.");
            return false;
        }
        entry[json_key] = enum_value != 0;
    }
    else if (type == EStartupCameraSettingType::ImageType)
    {
        if (!ValidateCameraEntries(*array, collection, error))
            return false;
        if (HasCameraImageType(*array, enum_value, index, collection, error))
        {
            error = FString::Printf(TEXT("ImageType %d already exists in %s."), enum_value, *CameraSettingsCollectionLabel(collection));
            return false;
        }
        entry[json_key] = enum_value;
    }
    else if (type == EStartupCameraSettingType::ProjectionMode)
    {
        const bool perspective = scalar_text.Equals(TEXT("perspective"), ESearchCase::CaseSensitive);
        const bool orthographic = scalar_text.Equals(TEXT("orthographic"), ESearchCase::CaseSensitive);
        if (!perspective && !orthographic)
        {
            error = TEXT("ProjectionMode must be perspective or orthographic.");
            return false;
        }
        entry[json_key] = perspective ? "perspective" : "orthographic";
    }
    else if (type == EStartupCameraSettingType::Integer)
    {
        int32 value = 0;
        if (!ParseCameraInteger(scalar_text, value)) { error = TEXT("Integer must be a valid int32."); return false; }
        if ((FCString::Strcmp(key, TEXT("Width")) == 0 || FCString::Strcmp(key, TEXT("Height")) == 0) && value < 0)
        { error = TEXT("Width and Height cannot be negative."); return false; }
        entry[json_key] = value;
    }
    else if (type == EStartupCameraSettingType::Number)
    {
        double value = 0.0;
        if (!ParseCameraNumber(scalar_text, value)) { error = TEXT("Number must be finite."); return false; }
        entry[json_key] = value;
    }
    else
    {
        error = TEXT("Unsupported camera setting field type.");
        return false;
    }
    updated_text = SerializeCameraSettingsDocument(document);
    return true;
}

bool FStartupCameraSettingsModel::ApplyEntry(const FString& document_text, const FString& vehicle_name,
                                             const FString& camera_name, EStartupCameraSettingsCollection collection,
                                             int32 index, const FString& replacement_text,
                                             FString& updated_text, FString& error)
{
    error.Reset();
    CameraSettingsJson replacement;
    if (!ParseCameraSettingsDocument(replacement_text, replacement, error) || !replacement.is_object())
    {
        error = TEXT("Selected settings JSON must be an object.");
        return false;
    }
    CameraSettingsJson document;
    if (!ParseCameraSettingsDocument(document_text, document, error))
        return false;
    CameraSettingsJson* array = FindCameraSettingsArray(document, vehicle_name, camera_name, collection, false, error);
    if (array == nullptr || index < 0 || index >= static_cast<int32>(array->size()) || !(*array)[index].is_object())
    {
        if (error.IsEmpty()) error = TEXT("Select a valid settings entry first.");
        return false;
    }
    if (!ValidateCameraEntries(*array, collection, error))
        return false;
    (*array)[index] = replacement;
    updated_text = SerializeCameraSettingsDocument(document);
    return true;
}

struct SStartupCameraSettingsEditor::FState
{
    FReadDocument ReadDocument;
    FWriteDocument WriteDocument;
    FStatus Status;
    FSelectedVehicle SelectedVehicle;
    FSelectedCamera SelectedCamera;
    EStartupCameraSettingsCollection Collection = EStartupCameraSettingsCollection::Capture;
    int32 SelectedIndex = INDEX_NONE;
    bool Updating = false;

    TArray<TSharedPtr<FStartupCameraEntryInfo>> Entries;
    TArray<TSharedPtr<FString>> ImageChoices;
    TArray<int32> ImageValues;
    TArray<bool> ImageOmit;
    TArray<bool> ImageKeep;
    TSharedPtr<SComboBox<TSharedPtr<FString>>> AddImageType;
    TSharedPtr<SComboBox<TSharedPtr<FString>>> DuplicateImageType;
    TSharedPtr<SListView<TSharedPtr<FStartupCameraEntryInfo>>> EntryList;
    TSharedPtr<SWidgetSwitcher> CollectionSwitcher;
    TSharedPtr<SMultiLineEditableTextBox> Advanced;
    TSharedPtr<SMultiLineEditableTextBox> ModelAdvanced;
    TSharedPtr<STextBlock> InfoText;
    TSharedPtr<STextBlock> ModelMismatchText;
    TSharedPtr<SVerticalBox> CaptureFields;
    TSharedPtr<SVerticalBox> NoiseFields;
    TSharedPtr<SVerticalBox> ModelFields;
    FString ModelType;

    struct FFieldRow
    {
        FCameraFieldDefinition Definition;
        TArray<TSharedPtr<FString>> Choices;
        TArray<int32> Values;
        TArray<bool> Omit;
        TArray<bool> Keep;
        TSharedPtr<SComboBox<TSharedPtr<FString>>> Combo;
        TSharedPtr<SEditableTextBox> Text;
    };
    TArray<TSharedPtr<FFieldRow>> CaptureRows;
    TArray<TSharedPtr<FFieldRow>> NoiseRows;

    struct FModelFieldRow
    {
        FCameraModelFieldDefinition Definition;
        TArray<TSharedPtr<FString>> Choices;
        TSharedPtr<SComboBox<TSharedPtr<FString>>> Combo;
        TSharedPtr<SEditableTextBox> Text;
        TSharedPtr<SWidget> Line;
        TSharedPtr<STextBlock> GroupHeader;
    };
    TArray<TSharedPtr<FModelFieldRow>> ModelRows;

    void Report(const FString& message) const
    {
        if (Status)
            Status(message);
    }

    FString VehicleName() const { return SelectedVehicle ? SelectedVehicle() : FString(); }
    FString CameraName() const { return SelectedCamera ? SelectedCamera() : FString(); }

    bool Read(const FString& failure_prefix, FString& text) const
    {
        if (!ReadDocument)
        {
            Report(failure_prefix + TEXT(" document callback is unavailable."));
            return false;
        }
        text = ReadDocument();
        return true;
    }

    static const FCameraFieldDefinition* Definitions(EStartupCameraSettingsCollection collection, int32& count)
    {
        if (collection == EStartupCameraSettingsCollection::Capture)
        {
            count = UE_ARRAY_COUNT(GCaptureFields);
            return GCaptureFields;
        }
        count = UE_ARRAY_COUNT(GNoiseFields);
        return GNoiseFields;
    }

    void BuildBooleanChoices(FFieldRow& row, const CameraSettingsJson* value)
    {
        row.Choices.Reset(); row.Values.Reset(); row.Omit.Reset(); row.Keep.Reset();
        row.Choices.Add(MakeShared<FString>(TEXT("Auto (omitted)"))); row.Values.Add(0); row.Omit.Add(true); row.Keep.Add(false);
        row.Choices.Add(MakeShared<FString>(TEXT("True"))); row.Values.Add(1); row.Omit.Add(false); row.Keep.Add(false);
        row.Choices.Add(MakeShared<FString>(TEXT("False"))); row.Values.Add(0); row.Omit.Add(false); row.Keep.Add(false);
        if (row.Combo.IsValid())
            row.Combo->RefreshOptions();
        (void)value;
    }

    void BuildImageChoices(FFieldRow& row, const CameraSettingsJson* value)
    {
        row.Choices.Reset(); row.Values.Reset(); row.Omit.Reset(); row.Keep.Reset();
        int32 authored = 0;
        const bool has_authored = value != nullptr && value->contains("ImageType") && value->at("ImageType").is_number_integer() && TryGetCameraImageType(*value, authored);
        row.Choices.Add(MakeShared<FString>(TEXT("Auto (omitted)"))); row.Values.Add(0); row.Omit.Add(true); row.Keep.Add(false);
        if (has_authored && authored != 0)
        {
            const bool recognized = authored >= -1 && authored <= 11;
            if (!recognized)
            {
                row.Choices.Add(MakeShared<FString>(FString::Printf(TEXT("Keep current: %d"), authored)));
                row.Values.Add(authored); row.Omit.Add(false); row.Keep.Add(true);
            }
        }
        static const int32 values[] = {-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        for (const int32 image_type : values)
        {
            row.Choices.Add(MakeShared<FString>(CameraImageTypeLabel(image_type)));
            row.Values.Add(image_type); row.Omit.Add(false); row.Keep.Add(false);
        }
        if (row.Combo.IsValid())
            row.Combo->RefreshOptions();
    }

    void BuildProjectionChoices(FFieldRow& row, const CameraSettingsJson* value)
    {
        row.Choices.Reset(); row.Values.Reset(); row.Omit.Reset(); row.Keep.Reset();
        FString authored;
        if (value != nullptr && value->is_string())
            authored = UTF8_TO_TCHAR(value->get<std::string>().c_str());
        if (!authored.IsEmpty() && authored != TEXT("perspective") && authored != TEXT("orthographic"))
        {
            row.Choices.Add(MakeShared<FString>(FString::Printf(TEXT("Keep current: %s"), *authored)));
            row.Values.Add(0); row.Omit.Add(false); row.Keep.Add(true);
        }
        row.Choices.Add(MakeShared<FString>(TEXT("Auto (omitted)"))); row.Values.Add(0); row.Omit.Add(true); row.Keep.Add(false);
        row.Choices.Add(MakeShared<FString>(TEXT("perspective"))); row.Values.Add(0); row.Omit.Add(false); row.Keep.Add(false);
        row.Choices.Add(MakeShared<FString>(TEXT("orthographic"))); row.Values.Add(1); row.Omit.Add(false); row.Keep.Add(false);
        if (row.Combo.IsValid())
            row.Combo->RefreshOptions();
    }

    void BuildRows(EStartupCameraSettingsCollection collection, TSharedPtr<SVerticalBox> box, TArray<TSharedPtr<FFieldRow>>& rows, TWeakPtr<FState> weak_state)
    {
        int32 count = 0;
        const FCameraFieldDefinition* definitions = Definitions(collection, count);
        FString last_group;
        for (int32 index = 0; index < count; ++index)
        {
            TSharedPtr<FFieldRow> row = MakeShared<FFieldRow>();
            row->Definition = definitions[index];
            rows.Add(row);
            TWeakPtr<FFieldRow> weak_row = row;
            TSharedRef<SHorizontalBox> line = SNew(SHorizontalBox);
            if (last_group != row->Definition.Group)
            {
                box->AddSlot().AutoHeight().Padding(4.0f, 8.0f, 3.0f, 2.0f)[SNew(STextBlock).Text(FText::FromString(row->Definition.Group))];
                last_group = row->Definition.Group;
            }
            line->AddSlot().FillWidth(0.42f).Padding(3.0f)[SNew(STextBlock).Text(FText::FromString(row->Definition.Label))];
            if (row->Definition.Type == EStartupCameraSettingType::Boolean || row->Definition.Type == EStartupCameraSettingType::ImageType || row->Definition.Type == EStartupCameraSettingType::ProjectionMode)
            {
                row->Combo = SNew(SComboBox<TSharedPtr<FString>>)
                    .OptionsSource(&row->Choices)
                    .OnGenerateWidget_Lambda([](TSharedPtr<FString> item) { return SNew(STextBlock).Text(FText::FromString(*item)); })
                    .OnSelectionChanged_Lambda([weak_state, weak_row](TSharedPtr<FString> item, ESelectInfo::Type)
                    {
                        if (TSharedPtr<FState> state = weak_state.Pin())
                            if (TSharedPtr<FFieldRow> field = weak_row.Pin())
                                state->Mutate(field, field->Choices.IndexOfByKey(item), FString(), false);
                    })
                    [SNew(STextBlock).Text_Lambda([weak_row]() { if (TSharedPtr<FFieldRow> row = weak_row.Pin()) if (row->Combo.IsValid() && row->Combo->GetSelectedItem().IsValid()) return FText::FromString(*row->Combo->GetSelectedItem()); return FText::FromString(TEXT("Auto (omitted)")); })];
                line->AddSlot().FillWidth(0.50f).Padding(3.0f)[row->Combo.ToSharedRef()];
            }
            else
            {
                row->Text = SNew(SEditableTextBox)
                    .HintText(FText::FromString(TEXT("Auto (omitted)")))
                    .OnTextCommitted_Lambda([weak_state, weak_row](const FText& text, ETextCommit::Type)
                    {
                        if (TSharedPtr<FState> state = weak_state.Pin())
                            if (TSharedPtr<FFieldRow> field = weak_row.Pin())
                                state->Mutate(field, 0, text.ToString(), false);
                    });
                line->AddSlot().FillWidth(0.50f).Padding(3.0f)[row->Text.ToSharedRef()];
            }
            line->AddSlot().AutoWidth().Padding(3.0f)[SNew(SButton).Text(FText::FromString(TEXT("Clear"))).OnClicked_Lambda([weak_state, weak_row]() { if (TSharedPtr<FState> state = weak_state.Pin()) if (TSharedPtr<FFieldRow> row = weak_row.Pin()) state->Mutate(row, 0, FString(), true); return FReply::Handled(); })];
            box->AddSlot().AutoHeight().Padding(2.0f)[line];
        }
    }

    bool ModelRowVisible(const FModelFieldRow& row) const
    {
        if (ModelType.IsEmpty())
            return row.Definition.Type == EStartupCameraModelFieldType::Type;
        if (row.Definition.ModelType[0] == TEXT('\0'))
            return true;
        if (FCString::Stricmp(row.Definition.ModelType, TEXT("not-raymap")) == 0)
            return !ModelType.Equals(TEXT("Raymap"), ESearchCase::IgnoreCase);
        return ModelType.Equals(row.Definition.ModelType, ESearchCase::IgnoreCase);
    }

    void BuildModelRows(TSharedPtr<SVerticalBox> box, TWeakPtr<FState> weak_state)
    {
        FString last_group;
        for (int32 index = 0; index < UE_ARRAY_COUNT(GCameraModelFields); ++index)
        {
            TSharedPtr<FModelFieldRow> row = MakeShared<FModelFieldRow>();
            row->Definition = GCameraModelFields[index];
            ModelRows.Add(row);
            TWeakPtr<FModelFieldRow> weak_row = row;
            TSharedRef<SHorizontalBox> line = SNew(SHorizontalBox);
            row->Line = line;
            if (last_group != row->Definition.Group)
            {
                TSharedPtr<STextBlock> header;
                box->AddSlot().AutoHeight().Padding(4.0f, 8.0f, 3.0f, 2.0f)
                    [SAssignNew(header, STextBlock).Text(FText::FromString(row->Definition.Group))];
                row->GroupHeader = header;
                last_group = row->Definition.Group;
            }
            line->AddSlot().FillWidth(0.34f).Padding(3.0f)
                [SNew(STextBlock).Text(FText::FromString(row->Definition.Label))];
            const bool is_combo = row->Definition.Type == EStartupCameraModelFieldType::Type ||
                                  row->Definition.Type == EStartupCameraModelFieldType::Faces ||
                                  row->Definition.Type == EStartupCameraModelFieldType::Backend;
            if (is_combo)
            {
                if (row->Definition.Type == EStartupCameraModelFieldType::Type)
                {
                    row->Choices = {MakeShared<FString>(TEXT("Pinhole")), MakeShared<FString>(TEXT("KannalaBrandt")),
                                    MakeShared<FString>(TEXT("DoubleSphere")), MakeShared<FString>(TEXT("Raymap"))};
                }
                else if (row->Definition.Type == EStartupCameraModelFieldType::Faces)
                {
                    row->Choices = {MakeShared<FString>(TEXT("Auto")), MakeShared<FString>(TEXT("5")), MakeShared<FString>(TEXT("6"))};
                }
                else
                {
                    row->Choices = {MakeShared<FString>(TEXT("Cube")), MakeShared<FString>(TEXT("NativeGEER"))};
                }
                row->Combo = SNew(SComboBox<TSharedPtr<FString>>)
                    .OptionsSource(&row->Choices)
                    .OnGenerateWidget_Lambda([](TSharedPtr<FString> item) { return SNew(STextBlock).Text(FText::FromString(*item)); })
                    .OnSelectionChanged_Lambda([weak_state, weak_row](TSharedPtr<FString> item, ESelectInfo::Type)
                    {
                        if (TSharedPtr<FState> state = weak_state.Pin())
                            if (!state->Updating)
                                if (TSharedPtr<FModelFieldRow> field = weak_row.Pin())
                                    state->MutateModel(field, item.IsValid() ? *item : FString(), false);
                    })
                    [SNew(STextBlock).Text_Lambda([weak_state, weak_row]() {
                        if (TSharedPtr<FState> state = weak_state.Pin())
                            if (TSharedPtr<FModelFieldRow> field = weak_row.Pin())
                                if (field->Combo.IsValid() && field->Combo->GetSelectedItem().IsValid())
                                    return FText::FromString(*field->Combo->GetSelectedItem());
                        return FText::FromString(TEXT("Select"));
                    })];
                line->AddSlot().FillWidth(0.56f).Padding(3.0f)[row->Combo.ToSharedRef()];
            }
            else
            {
                row->Text = SNew(SEditableTextBox)
                    .HintText(FText::FromString(TEXT("Auto (omitted)")))
                    .OnTextCommitted_Lambda([weak_state, weak_row](const FText& value, ETextCommit::Type)
                    {
                        if (TSharedPtr<FState> state = weak_state.Pin())
                            if (!state->Updating)
                                if (TSharedPtr<FModelFieldRow> field = weak_row.Pin())
                                    state->MutateModel(field, value.ToString(), false);
                    });
                line->AddSlot().FillWidth(0.56f).Padding(3.0f)[row->Text.ToSharedRef()];
                if (row->Definition.Key == FString(TEXT("Path")))
                    line->AddSlot().AutoWidth().Padding(3.0f)[SNew(SButton).Text(FText::FromString(TEXT("Browse...")))
                        .OnClicked_Lambda([weak_state, weak_row]() {
                            if (TSharedPtr<FState> state = weak_state.Pin())
                                if (TSharedPtr<FModelFieldRow> field = weak_row.Pin())
                                    state->BrowseRaymap(field);
                            return FReply::Handled();
                        })];
            }
            if (row->Definition.Type != EStartupCameraModelFieldType::Type)
                line->AddSlot().AutoWidth().Padding(3.0f)[SNew(SButton).Text(FText::FromString(TEXT("Clear")))
                    .OnClicked_Lambda([weak_state, weak_row]() {
                        if (TSharedPtr<FState> state = weak_state.Pin())
                            if (TSharedPtr<FModelFieldRow> field = weak_row.Pin())
                                state->MutateModel(field, FString(), true);
                        return FReply::Handled();
                    })];
            box->AddSlot().AutoHeight().Padding(2.0f)[line];
        }
    }

    CameraSettingsJson* SelectedObject(CameraSettingsJson& document, FString& error) const
    {
        CameraSettingsJson* array = FindCameraSettingsArray(document, VehicleName(), CameraName(), Collection, false, error);
        if (array == nullptr)
            return nullptr;
        if (SelectedIndex < 0 || SelectedIndex >= static_cast<int32>(array->size()) || !(*array)[SelectedIndex].is_object())
        {
            error = TEXT("Select a camera settings entry first.");
            return nullptr;
        }
        return &(*array)[SelectedIndex];
    }

    void RefreshControls()
    {
        FString text;
        if (!Read(TEXT("Camera settings refresh refused:"), text)) return;
        CameraSettingsJson document;
        FString error;
        if (!ParseCameraSettingsDocument(text, document, error)) { Report(TEXT("Camera settings refresh refused: ") + error); return; }
        CameraSettingsJson* item = SelectedObject(document, error);
        Updating = true;
        TArray<TSharedPtr<FFieldRow>>& rows = Collection == EStartupCameraSettingsCollection::Capture ? CaptureRows : NoiseRows;
        for (const TSharedPtr<FFieldRow>& row : rows)
        {
            const CameraSettingsJson* value = item && item->contains(TCHAR_TO_UTF8(row->Definition.Key)) ? &item->at(TCHAR_TO_UTF8(row->Definition.Key)) : nullptr;
            if (row->Definition.Type == EStartupCameraSettingType::Boolean)
            {
                BuildBooleanChoices(*row, value);
                int32 selected = value && value->is_boolean() ? (value->get<bool>() ? 1 : 2) : 0;
                row->Combo->SetSelectedItem(row->Choices[selected]);
            }
            else if (row->Definition.Type == EStartupCameraSettingType::ImageType)
            {
                BuildImageChoices(*row, item);
                int32 selected = 0; int32 authored = 0;
                if (value && TryGetCameraImageType(*item, authored))
                    for (int32 index = 0; index < row->Values.Num(); ++index) if (!row->Omit[index] && row->Values[index] == authored) { selected = index; break; }
                row->Combo->SetSelectedItem(row->Choices[selected]);
            }
            else if (row->Definition.Type == EStartupCameraSettingType::ProjectionMode)
            {
                BuildProjectionChoices(*row, value);
                int32 selected = 0;
                if (value && value->is_string())
                {
                    const FString authored = UTF8_TO_TCHAR(value->get<std::string>().c_str());
                    for (int32 index = 0; index < row->Choices.Num(); ++index) if (*row->Choices[index] == authored) { selected = index; break; }
                }
                row->Combo->SetSelectedItem(row->Choices[selected]);
            }
            else if (row->Text.IsValid())
            {
                FString value_text;
                if (value && value->is_number()) value_text = UTF8_TO_TCHAR(value->dump().c_str());
                row->Text->SetText(FText::FromString(value_text));
            }
        }
        if (Advanced.IsValid()) Advanced->SetText(FText::FromString(item ? UTF8_TO_TCHAR(item->dump(2).c_str()) : TEXT("{}\n")));
        if (InfoText.IsValid()) InfoText->SetText(FText::FromString(item ? FString::Printf(TEXT("Entry %d"), SelectedIndex) : TEXT("No camera settings entry selected")));
        Updating = false;
    }

    void RefreshEntries()
    {
        // An empty camera collection is a valid profile state. The parent vehicle editor calls
        // RefreshFromDocument while a vehicle is selected but before any camera exists; asking
        // the model to list entries with an empty camera name would turn that normal state into a
        // status error. In the launcher the status callback marks validation stale, which made a
        // valid vehicle-only (or sensor-only) profile impossible to launch.
        if (VehicleName().IsEmpty() || CameraName().IsEmpty())
        {
            Entries.Reset();
            SelectedIndex = INDEX_NONE;
            if (EntryList.IsValid())
            {
                EntryList->RequestListRefresh();
                EntryList->SetSelection(nullptr);
            }
            RefreshControls();
            return;
        }
        FString text;
        if (!Read(TEXT("Camera settings list refused:"), text)) return;
        TArray<FStartupCameraEntryInfo> listed;
        FString error;
        if (!FStartupCameraSettingsModel::ListEntries(text, VehicleName(), CameraName(), Collection, listed, error))
        {
            Entries.Reset(); SelectedIndex = INDEX_NONE; if (EntryList.IsValid()) EntryList->RequestListRefresh();
            Report(TEXT("Camera settings list refused: ") + error); RefreshControls(); return;
        }
        const int32 old_index = SelectedIndex;
        Entries.Reset();
        for (const FStartupCameraEntryInfo& info : listed) Entries.Add(MakeShared<FStartupCameraEntryInfo>(info));
        if (Entries.Num() == 0) SelectedIndex = INDEX_NONE;
        else if (old_index >= 0 && old_index < Entries.Num()) SelectedIndex = old_index;
        else SelectedIndex = 0;
        Updating = true;
        if (EntryList.IsValid()) { EntryList->RequestListRefresh(); if (SelectedIndex >= 0) EntryList->SetSelection(Entries[SelectedIndex]); else EntryList->SetSelection(nullptr); }
        Updating = false;
        RefreshControls();
    }

    void RefreshFromDocument() { RefreshEntries(); }

    void Publish(const FString& updated, const FString& message)
    {
        if (!WriteDocument) { Report(TEXT("Camera settings write callback is unavailable; change was not applied.")); return; }
        Updating = true; WriteDocument(updated); Updating = false; Report(message); RefreshEntries();
    }

    void Mutate(const TSharedPtr<FFieldRow>& row, int32 choice, const FString& text, bool clear)
    {
        if (Updating || !row.IsValid()) return;
        if ((row->Definition.Type == EStartupCameraSettingType::Boolean || row->Definition.Type == EStartupCameraSettingType::ImageType || row->Definition.Type == EStartupCameraSettingType::ProjectionMode) && (choice < 0 || choice >= row->Choices.Num())) { Report(TEXT("Invalid camera settings choice.")); return; }
        FString source; if (!Read(TEXT("Camera settings edit refused:"), source)) return;
        FString updated, error;
        const int32 enum_value = (choice >= 0 && choice < row->Values.Num()) ? row->Values[choice] : 0;
        const bool omit = choice >= 0 && choice < row->Omit.Num() && row->Omit[choice];
        const bool keep = choice >= 0 && choice < row->Keep.Num() && row->Keep[choice];
        if (!FStartupCameraSettingsModel::MutateField(source, VehicleName(), CameraName(), Collection, SelectedIndex, row->Definition.Key, row->Definition.Type, enum_value, omit, keep, text, clear, updated, error)) { Report(TEXT("Camera settings edit refused: ") + error); return; }
        Publish(updated, TEXT("Camera setting changed; press Validate."));
    }

    int32 SelectedChoice(const TSharedPtr<SComboBox<TSharedPtr<FString>>>& combo, const TArray<TSharedPtr<FString>>& choices) const { return combo.IsValid() ? choices.IndexOfByKey(combo->GetSelectedItem()) : INDEX_NONE; }

    void AddEntry()
    {
        const int32 choice = SelectedChoice(AddImageType, ImageChoices);
        if (choice < 0 || choice >= ImageValues.Num() || ImageOmit[choice]) { Report(TEXT("Add requires an explicit ImageType.")); return; }
        FString source; if (!Read(TEXT("Camera settings add refused:"), source)) return;
        FString updated, error;
        if (!FStartupCameraSettingsModel::AddEntry(source, VehicleName(), CameraName(), Collection, ImageValues[choice], updated, error)) { Report(TEXT("Camera settings add refused: ") + error); return; }
        Publish(updated, TEXT("Camera settings entry added; press Validate."));
    }

    void DuplicateEntry()
    {
        const int32 choice = SelectedChoice(DuplicateImageType, ImageChoices);
        if (choice < 0 || choice >= ImageValues.Num() || ImageOmit[choice]) { Report(TEXT("Duplicate requires an explicit target ImageType.")); return; }
        FString source; if (!Read(TEXT("Camera settings duplicate refused:"), source)) return;
        FString updated, error;
        if (!FStartupCameraSettingsModel::DuplicateEntry(source, VehicleName(), CameraName(), Collection, SelectedIndex, ImageValues[choice], updated, error)) { Report(TEXT("Camera settings duplicate refused: ") + error); return; }
        Publish(updated, TEXT("Camera settings entry duplicated; press Validate."));
    }

    void DeleteEntry()
    {
        if (SelectedIndex == INDEX_NONE || FMessageDialog::Open(EAppMsgType::YesNo, FText::FromString(TEXT("Delete the selected camera settings entry?"))) != EAppReturnType::Yes) return;
        FString source; if (!Read(TEXT("Camera settings delete refused:"), source)) return;
        FString updated, error;
        if (!FStartupCameraSettingsModel::DeleteEntry(source, VehicleName(), CameraName(), Collection, SelectedIndex, updated, error)) { Report(TEXT("Camera settings delete refused: ") + error); return; }
        Publish(updated, TEXT("Camera settings entry deleted; press Validate."));
    }

    void ApplyAdvanced()
    {
        if (!Advanced.IsValid()) return;
        FString source; if (!Read(TEXT("Camera settings JSON apply refused:"), source)) return;
        FString updated, error;
        if (!FStartupCameraSettingsModel::ApplyEntry(source, VehicleName(), CameraName(), Collection, SelectedIndex, Advanced->GetText().ToString(), updated, error)) { Report(TEXT("Camera settings JSON apply refused: ") + error); return; }
        Publish(updated, TEXT("Camera settings JSON applied; press Validate."));
    }

    void RefreshModel()
    {
        // No authored camera is a valid state. Model controls remain visible as an editor affordance,
        // but passive refresh must not report the empty selection as an error to the parent launcher
        // (its status callback would otherwise invalidate an already successful document check).
        if (VehicleName().IsEmpty() || CameraName().IsEmpty())
        {
            ModelType.Reset();
            for (const TSharedPtr<FModelFieldRow>& row : ModelRows)
            {
                if (row->Combo.IsValid())
                    row->Combo->SetSelectedItem(nullptr);
                if (row->Text.IsValid())
                    row->Text->SetText(FText::GetEmpty());
                if (row->Line.IsValid())
                    row->Line->SetVisibility(row->Definition.Type == EStartupCameraModelFieldType::Type ? EVisibility::Visible : EVisibility::Collapsed);
                if (row->GroupHeader.IsValid())
                    row->GroupHeader->SetVisibility(row->Definition.Type == EStartupCameraModelFieldType::Type ? EVisibility::Visible : EVisibility::Collapsed);
            }
            if (ModelAdvanced.IsValid())
                ModelAdvanced->SetText(FText::FromString(TEXT("{}")));
            if (ModelMismatchText.IsValid())
                ModelMismatchText->SetText(FText::GetEmpty());
            return;
        }
        FString source; if (!Read(TEXT("Camera model refresh refused:"), source)) return;
        FString model, error; if (!FStartupCameraSettingsModel::GetCameraModel(source, VehicleName(), CameraName(), model, error)) { Report(TEXT("Camera model refresh refused: ") + error); return; }
        ModelType.Reset();
        CameraSettingsJson model_json;
        if (!model.IsEmpty())
        {
            try
            {
                model_json = CameraSettingsJson::parse(TCHAR_TO_UTF8(*model));
                if (model_json.contains("Type") && model_json["Type"].is_string())
                    ModelType = UTF8_TO_TCHAR(model_json["Type"].get<std::string>().c_str());
            }
            catch (...) { /* GetCameraModel only returns parser-produced JSON. */ }
        }
        Updating = true;
        for (const TSharedPtr<FModelFieldRow>& row : ModelRows)
        {
            if (!row.IsValid()) continue;
            const char* key = TCHAR_TO_UTF8(row->Definition.Key);
            const CameraSettingsJson* value = model_json.is_object() && model_json.contains(key) ? &model_json.at(key) : nullptr;
            if (row->Combo.IsValid())
            {
                int32 selected = INDEX_NONE;
                if (value != nullptr)
                {
                    FString authored;
                    if (value->is_string()) authored = UTF8_TO_TCHAR(value->get<std::string>().c_str());
                    for (int32 choice = 0; choice < row->Choices.Num(); ++choice)
                        if (authored.Equals(*row->Choices[choice], ESearchCase::IgnoreCase)) { selected = choice; break; }
                    if (row->Definition.Type == EStartupCameraModelFieldType::Faces && value->is_number_integer())
                    {
                        int32 faces = 0;
                        if (ParseCameraInteger(UTF8_TO_TCHAR(value->dump().c_str()), faces))
                            selected = faces == 5 ? 1 : faces == 6 ? 2 : 0;
                    }
                }
                row->Combo->SetSelectedItem(selected >= 0 ? row->Choices[selected] : nullptr);
            }
            else if (row->Text.IsValid())
            {
                FString value_text;
                if (value != nullptr && value->is_string()) value_text = UTF8_TO_TCHAR(value->get<std::string>().c_str());
                else if (value != nullptr && value->is_number()) value_text = UTF8_TO_TCHAR(value->dump().c_str());
                row->Text->SetText(FText::FromString(value_text));
            }
            if (row->Line.IsValid())
                row->Line->SetVisibility(ModelRowVisible(*row) ? EVisibility::Visible : EVisibility::Collapsed);
            if (row->GroupHeader.IsValid())
                row->GroupHeader->SetVisibility(ModelRowVisible(*row) ? EVisibility::Visible : EVisibility::Collapsed);
        }
        Updating = false;
        if (ModelAdvanced.IsValid()) ModelAdvanced->SetText(FText::FromString(model.IsEmpty() ? TEXT("{}") : model));
        FString mismatch;
        FStartupCameraSettingsModel::CameraModelResolutionMismatch(source, VehicleName(), CameraName(), mismatch);
        if (ModelMismatchText.IsValid())
            ModelMismatchText->SetText(FText::FromString(mismatch));
        if (!mismatch.IsEmpty()) Report(mismatch);
    }

    void MutateModel(const TSharedPtr<FModelFieldRow>& row, const FString& value, bool clear)
    {
        if (Updating || !row.IsValid()) return;
        FString source; if (!Read(TEXT("Camera model edit refused:"), source)) return;
        FString updated, error;
        if (row->Definition.Type == EStartupCameraModelFieldType::Type)
        {
            if (!FStartupCameraSettingsModel::ChangeCameraModelType(source, VehicleName(), CameraName(), value, updated, error))
            {
                Report(TEXT("Camera model type change refused: ") + error);
                RefreshModel();
                return;
            }
        }
        else if (!FStartupCameraSettingsModel::MutateCameraModelField(source, VehicleName(), CameraName(), row->Definition.Key,
                                                                       row->Definition.Type, value, clear, updated, error))
        {
            Report(TEXT("Camera model edit refused: ") + error);
            RefreshModel();
            return;
        }
        Publish(updated, TEXT("Camera model changed; press Validate."));
        RefreshModel();
    }

    void BrowseRaymap(const TSharedPtr<FModelFieldRow>& row)
    {
        if (Updating || !row.IsValid()) return;
        IDesktopPlatform* desktop = FDesktopPlatformModule::Get();
        if (desktop == nullptr) { Report(TEXT("Raymap browse is unavailable in this build.")); return; }
        TArray<FString> files;
        if (!desktop->OpenFileDialog(nullptr, TEXT("Choose raymap file"), FPaths::LaunchDir(), TEXT(""),
                                     TEXT("Raymap Files (*.raymap;*.bin)|*.raymap;*.bin|All Files (*.*)|*.*"),
                                     EFileDialogFlags::None, files) || files.Num() == 0)
            return;
        MutateModel(row, files[0], false);
    }

    void AddModel(const FString& model_type = TEXT("Pinhole"))
    {
        FString source; if (!Read(TEXT("Camera model add refused:"), source)) return; FString updated, error;
        if (!FStartupCameraSettingsModel::AddCameraModel(source, VehicleName(), CameraName(), model_type, updated, error)) { Report(TEXT("Camera model add refused: ") + error); return; }
        Publish(updated, TEXT("CameraModel added; edit the structured fields or raw model JSON.")); RefreshModel();
    }

    void AddRaymapModel()
    {
        IDesktopPlatform* desktop = FDesktopPlatformModule::Get();
        if (desktop == nullptr) { Report(TEXT("Raymap browse is unavailable in this build.")); return; }
        TArray<FString> files;
        if (!desktop->OpenFileDialog(nullptr, TEXT("Choose raymap file"), FPaths::LaunchDir(), TEXT(""),
                                     TEXT("Raymap Files (*.raymap;*.bin)|*.raymap;*.bin|All Files (*.*)|*.*"),
                                     EFileDialogFlags::None, files) || files.Num() == 0)
            return;
        FString source; if (!Read(TEXT("Camera model add refused:"), source)) return;
        FString updated, error;
        if (!FStartupCameraSettingsModel::AddCameraModel(source, VehicleName(), CameraName(), TEXT("Raymap"), files[0], updated, error))
        { Report(TEXT("Raymap model add refused: ") + error); return; }
        Publish(updated, TEXT("Raymap model added.")); RefreshModel();
    }
    void RemoveModel()
    {
        FString source; if (!Read(TEXT("Camera model remove refused:"), source)) return; FString updated, error;
        if (!FStartupCameraSettingsModel::RemoveCameraModel(source, VehicleName(), CameraName(), updated, error)) { Report(TEXT("Camera model remove refused: ") + error); return; }
        Publish(updated, TEXT("CameraModel removed.")); if (ModelAdvanced.IsValid()) ModelAdvanced->SetText(FText::FromString(TEXT("{}"))); RefreshModel();
    }
    void ApplyModel()
    {
        if (!ModelAdvanced.IsValid()) return; FString source; if (!Read(TEXT("Camera model apply refused:"), source)) return; FString updated, error;
        if (!FStartupCameraSettingsModel::ApplyCameraModel(source, VehicleName(), CameraName(), ModelAdvanced->GetText().ToString(), updated, error)) { Report(TEXT("Camera model apply refused: ") + error); return; }
        Publish(updated, TEXT("CameraModel applied; press Validate."));
        RefreshModel();
    }

    void SyncModelSize()
    {
        FString source; if (!Read(TEXT("Camera model size sync refused:"), source)) return;
        FString updated, error;
        if (!FStartupCameraSettingsModel::SyncCameraModelSizeFromScene(source, VehicleName(), CameraName(), updated, error))
        { Report(TEXT("Camera model size sync refused: ") + error); return; }
        Publish(updated, TEXT("Camera model size synchronized from Scene CaptureSettings."));
        RefreshModel();
    }

    void SetCollection(EStartupCameraSettingsCollection collection)
    {
        if (Updating) return;
        Collection = collection; SelectedIndex = INDEX_NONE;
        if (CollectionSwitcher.IsValid()) CollectionSwitcher->SetActiveWidgetIndex(Collection == EStartupCameraSettingsCollection::Capture ? 0 : 1);
        RefreshEntries();
    }
};

void SStartupCameraSettingsEditor::Construct(const FArguments& args)
{
    State = MakeShared<FState>();
    TSharedPtr<FState> state = State;
    state->ReadDocument = args._ReadDocument; state->WriteDocument = args._WriteDocument; state->Status = args._Status;
    state->SelectedVehicle = args._SelectedVehicle; state->SelectedCamera = args._SelectedCamera;
    state->CaptureFields = SNew(SVerticalBox); state->NoiseFields = SNew(SVerticalBox); state->ModelFields = SNew(SVerticalBox);
    TWeakPtr<FState> weak_state = state;
    state->BuildRows(EStartupCameraSettingsCollection::Capture, state->CaptureFields, state->CaptureRows, weak_state);
    state->BuildRows(EStartupCameraSettingsCollection::Noise, state->NoiseFields, state->NoiseRows, weak_state);
    state->BuildModelRows(state->ModelFields, weak_state);
    state->ImageChoices.Add(MakeShared<FString>(TEXT("Select ImageType"))); state->ImageValues.Add(0); state->ImageOmit.Add(true); state->ImageKeep.Add(false);
    static const int32 image_values[] = {-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    for (const int32 image_type : image_values) { state->ImageChoices.Add(MakeShared<FString>(CameraImageTypeLabel(image_type))); state->ImageValues.Add(image_type); state->ImageOmit.Add(false); state->ImageKeep.Add(false); }
    TSharedRef<SWidgetSwitcher> switcher = SAssignNew(state->CollectionSwitcher, SWidgetSwitcher)
        // The launcher provides one main vertical scroll area. Avoid nested scroll boxes here,
        // which otherwise leave the long camera form in a cramped half-height viewport.
        + SWidgetSwitcher::Slot()[state->CaptureFields.ToSharedRef()]
        + SWidgetSwitcher::Slot()[state->NoiseFields.ToSharedRef()]
        + SWidgetSwitcher::Slot()[SNew(SVerticalBox)
            + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(STextBlock).Text(FText::FromString(TEXT("CameraModel: fisheye/raymap calibration and NativeGEER/Cube rendering. NativeGEER is the NanoGS/splat path; Cube is the compatibility path.")))]
            + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SHorizontalBox)
                + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Enable Pinhole model"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->AddModel(TEXT("Pinhole")); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("New KB fisheye"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->AddModel(TEXT("KannalaBrandt")); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("New Double Sphere"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->AddModel(TEXT("DoubleSphere")); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("New Raymap"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->AddRaymapModel(); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Remove Model"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->RemoveModel(); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Sync Model Size from Scene"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->SyncModelSize(); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Refresh Model"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->RefreshModel(); return FReply::Handled(); })]]
            + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SAssignNew(state->ModelMismatchText, STextBlock).ColorAndOpacity(FSlateColor(FLinearColor(1.0f, 0.65f, 0.1f))).Text(FText::GetEmpty())]
            + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[state->ModelFields.ToSharedRef()]
            + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(STextBlock).Text(FText::FromString(TEXT("Raw CameraModel JSON (advanced)")))]
            + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SBox).HeightOverride(220.0f)[SAssignNew(state->ModelAdvanced, SMultiLineEditableTextBox).AutoWrapText(false)]]
            + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SButton).Text(FText::FromString(TEXT("Apply CameraModel JSON"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->ApplyModel(); return FReply::Handled(); })]];
    state->AddImageType = SNew(SComboBox<TSharedPtr<FString>>).OptionsSource(&state->ImageChoices).OnGenerateWidget_Lambda([](TSharedPtr<FString> item) { return SNew(STextBlock).Text(FText::FromString(*item)); })[SNew(STextBlock).Text_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) if (state->AddImageType.IsValid() && state->AddImageType->GetSelectedItem().IsValid()) return FText::FromString(*state->AddImageType->GetSelectedItem()); return FText::FromString(TEXT("Select ImageType")); })];
    state->DuplicateImageType = SNew(SComboBox<TSharedPtr<FString>>).OptionsSource(&state->ImageChoices).OnGenerateWidget_Lambda([](TSharedPtr<FString> item) { return SNew(STextBlock).Text(FText::FromString(*item)); })[SNew(STextBlock).Text_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) if (state->DuplicateImageType.IsValid() && state->DuplicateImageType->GetSelectedItem().IsValid()) return FText::FromString(*state->DuplicateImageType->GetSelectedItem()); return FText::FromString(TEXT("Select target ImageType")); })];
    ChildSlot[SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(STextBlock).Text(FText::FromString(TEXT("Camera Capture, Noise & Model settings")))]
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SHorizontalBox)
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Capture"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->SetCollection(EStartupCameraSettingsCollection::Capture); return FReply::Handled(); })]
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Noise"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->SetCollection(EStartupCameraSettingsCollection::Noise); return FReply::Handled(); })]
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Model"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) { if (state->CollectionSwitcher.IsValid()) state->CollectionSwitcher->SetActiveWidgetIndex(2); state->RefreshModel(); } return FReply::Handled(); })]
            + SHorizontalBox::Slot().FillWidth(1.0f).Padding(8.0f)[SAssignNew(state->InfoText, STextBlock).Text(FText::FromString(TEXT("No camera settings entry selected")))] ]
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(0.35f).Padding(2.0f)[state->AddImageType.ToSharedRef()]
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Add"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->AddEntry(); return FReply::Handled(); })]
            + SHorizontalBox::Slot().FillWidth(0.35f).Padding(2.0f)[state->DuplicateImageType.ToSharedRef()]
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Duplicate"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->DuplicateEntry(); return FReply::Handled(); })]
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Delete"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->DeleteEntry(); return FReply::Handled(); })] ]
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SBox).HeightOverride(120.0f)[SAssignNew(state->EntryList, SListView<TSharedPtr<FStartupCameraEntryInfo>>).ListItemsSource(&state->Entries).OnSelectionChanged_Lambda([weak_state](TSharedPtr<FStartupCameraEntryInfo> item, ESelectInfo::Type) { if (TSharedPtr<FState> state = weak_state.Pin()) { if (state->Updating) return; state->SelectedIndex = item.IsValid() ? item->Index : INDEX_NONE; state->RefreshControls(); } }).OnGenerateRow_Lambda([](TSharedPtr<FStartupCameraEntryInfo> item, const TSharedRef<STableViewBase>& owner) { return SNew(STableRow<TSharedPtr<FStartupCameraEntryInfo>>, owner)[SNew(STextBlock).Text(FText::FromString(item.IsValid() ? item->Label : TEXT("Invalid")))]; })]]
        // Every section contributes its full desired height to the launcher's one main scroll.
        // This prevents the raw editor and its buttons from being arranged over long form rows.
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[switcher]
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SVerticalBox) + SVerticalBox::Slot().AutoHeight()[SNew(STextBlock).Text(FText::FromString(TEXT("Selected entry raw JSON")))] + SVerticalBox::Slot().AutoHeight()[SNew(SBox).HeightOverride(180.0f)[SAssignNew(state->Advanced, SMultiLineEditableTextBox).AutoWrapText(false)]]]
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SHorizontalBox)
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Refresh / Revert"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->RefreshFromDocument(); return FReply::Handled(); })]
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Apply object JSON"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> state = weak_state.Pin()) state->ApplyAdvanced(); return FReply::Handled(); })] ]];
}

void SStartupCameraSettingsEditor::RefreshFromDocument()
{
    if (State.IsValid()) State->RefreshFromDocument();
}

namespace
{
    using ModelJson = nlohmann::json;

    bool TryGetInt32(const ModelJson& value, int32& result)
    {
        if (!value.is_number_integer()) return false;
        try
        {
            const int64 parsed = value.get<int64>();
            if (parsed < std::numeric_limits<int32>::min() || parsed > std::numeric_limits<int32>::max()) return false;
            result = static_cast<int32>(parsed);
            return true;
        }
        catch (...) { return false; }
    }

    ModelJson* FindModel(ModelJson& document, const FString& vehicle, const FString& camera, FString& error)
    {
        ModelJson* camera_object = FindCameraSettingsCamera(document, vehicle, camera, error);
        if (camera_object == nullptr) return nullptr;
        if (!camera_object->contains("CameraModel")) return nullptr;
        if (!(*camera_object)["CameraModel"].is_object()) {
            error = TEXT("CameraModel must be a JSON object.");
            return nullptr;
        }
        return &(*camera_object)["CameraModel"];
    }

    bool ValidateModel(const ModelJson& model, FString& error)
    {
        if (!model.is_object()) {
            error = TEXT("CameraModel must be a JSON object.");
            return false;
        }
        if (!model.contains("Type") || !model["Type"].is_string()) {
            error = TEXT("CameraModel Type is required.");
            return false;
        }
        std::string type = model["Type"].get<std::string>();
        std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (type != "pinhole" && type != "kannalabrandt" && type != "doublesphere" && type != "raymap") {
            error = TEXT("CameraModel Type must be Pinhole, KannalaBrandt, DoubleSphere or Raymap.");
            return false;
        }
        auto finite_number = [&](const char* key) {
            return !model.contains(key) || (model[key].is_number() && std::isfinite(model[key].get<double>()));
        };
        for (const char* key : {"fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4", "xi", "alpha", "FOV_Degrees"})
            if (!finite_number(key)) {
                error = FString::Printf(TEXT("CameraModel %s must be finite."), UTF8_TO_TCHAR(key));
                return false;
            }
        if (model.contains("Width")) {
            int32 width;
            if (!TryGetInt32(model["Width"], width) || width < 0) {
                error = TEXT("CameraModel Width must be a nonnegative int32.");
                return false;
            }
        }
        if (model.contains("Height")) {
            int32 height;
            if (!TryGetInt32(model["Height"], height) || height < 0) {
                error = TEXT("CameraModel Height must be a nonnegative int32.");
                return false;
            }
        }
        if (model.contains("CubeFaceResolution")) {
            int32 resolution;
            if (!TryGetInt32(model["CubeFaceResolution"], resolution) || resolution < 0) {
                error = TEXT("CubeFaceResolution must be a nonnegative int32.");
                return false;
            }
        }
        if (type == "raymap") {
            if (!model.contains("Path") || !model["Path"].is_string() || model["Path"].get<std::string>().empty()) {
                error = TEXT("Raymap CameraModel needs a nonempty Path.");
                return false;
            }
        } else {
            int32 width = 0, height = 0;
            if (!model.contains("Width") || !TryGetInt32(model["Width"], width) || width <= 0 ||
                !model.contains("Height") || !TryGetInt32(model["Height"], height) || height <= 0) {
                error = TEXT("CameraModel needs non-zero Width and Height.");
                return false;
            }
            const bool focal = model.contains("fx") && model.contains("fy") && model["fx"].is_number() && model["fy"].is_number();
            const bool fov = model.contains("FOV_Degrees") && model["FOV_Degrees"].is_number() && model["FOV_Degrees"].get<double>() > 0.0 && model["FOV_Degrees"].get<double>() < 180.0;
            if (type == "pinhole" && !focal && !fov) {
                error = TEXT("Pinhole needs fx/fy or FOV_Degrees in (0,180).");
                return false;
            }
            if (type != "pinhole" || focal) {
                for (const char* key : {"fx", "fy"}) {
                    if (!model.contains(key) || !model[key].is_number()) {
                        error = TEXT("CameraModel needs numeric fx and fy.");
                        return false;
                    }
                }
                if (model["fx"].get<double>() == 0.0 || model["fy"].get<double>() == 0.0) {
                    error = TEXT("CameraModel fx and fy must be non-zero.");
                    return false;
                }
                for (const char* key : {"cx", "cy"}) {
                    if ((type != "pinhole" && !model.contains(key)) || (model.contains(key) && !model[key].is_number())) {
                        error = FString::Printf(TEXT("CameraModel %s must be numeric."), UTF8_TO_TCHAR(key));
                        return false;
                    }
                }
            }
        }
        if (type == "doublesphere" && model.contains("alpha") && (!model["alpha"].is_number() || model["alpha"].get<double>() < 0.0 || model["alpha"].get<double>() >= 1.0)) {
            error = TEXT("DoubleSphere alpha must be in [0,1).");
            return false;
        }
        if (model.contains("Faces")) {
            bool valid_faces = false;
            if (model["Faces"].is_number_integer()) {
                int32 faces;
                valid_faces = TryGetInt32(model["Faces"], faces) && (faces == 0 || faces == 5 || faces == 6);
            }
            else if (model["Faces"].is_string()) {
                std::string faces = model["Faces"].get<std::string>();
                std::transform(faces.begin(), faces.end(), faces.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                valid_faces = faces == "auto" || faces == "5" || faces == "6";
            }
            if (!valid_faces) {
                error = TEXT("Faces must be Auto, 5 or 6.");
                return false;
            }
        }
        if (model.contains("RenderBackend")) {
            if (!model["RenderBackend"].is_string()) {
                error = TEXT("RenderBackend must be Cube or NativeGEER.");
                return false;
            }
            std::string backend = model["RenderBackend"].get<std::string>();
            std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (backend != "cube" && backend != "nativegeer") {
                error = TEXT("RenderBackend must be Cube or NativeGEER.");
                return false;
            }
        }
        if (model.contains("SplatOnly") && !model["SplatOnly"].is_boolean()) {
            error = TEXT("CameraModel SplatOnly must be boolean.");
            return false;
        }
        if (model.contains("SplatOnly") && model.contains("RenderBackend")) {
            std::string backend = model["RenderBackend"].get<std::string>();
            std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (model["SplatOnly"].get<bool>() != (backend == "nativegeer")) {
                error = TEXT("CameraModel SplatOnly conflicts with RenderBackend.");
                return false;
            }
        }
        return true;
    }
}

    bool CanonicalModelType(const FString& value, std::string& canonical)
    {
        canonical = TCHAR_TO_UTF8(*value);
        std::transform(canonical.begin(), canonical.end(), canonical.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (canonical == "pinhole") canonical = "Pinhole";
        else if (canonical == "kannalabrandt") canonical = "KannalaBrandt";
        else if (canonical == "doublesphere") canonical = "DoubleSphere";
        else if (canonical == "raymap") canonical = "Raymap";
        else return false;
        return true;
    }

    void CanonicalizeModelEnums(ModelJson& model)
    {
        if (model.contains("Type") && model["Type"].is_string())
        {
            std::string canonical;
            if (CanonicalModelType(UTF8_TO_TCHAR(model["Type"].get<std::string>().c_str()), canonical))
                model["Type"] = canonical;
        }
        if (model.contains("Faces"))
        {
            if (model["Faces"].is_string())
            {
                std::string faces = model["Faces"].get<std::string>();
                std::transform(faces.begin(), faces.end(), faces.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (faces == "auto") model["Faces"] = "Auto";
                else if (faces == "5") model["Faces"] = "5";
                else if (faces == "6") model["Faces"] = "6";
            }
            else if (model["Faces"].is_number_integer() && model["Faces"] == 0)
                model["Faces"] = "Auto";
        }
        if (model.contains("RenderBackend") && model["RenderBackend"].is_string())
        {
            std::string backend = model["RenderBackend"].get<std::string>();
            std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (backend == "nativegeer") model["RenderBackend"] = "NativeGEER";
            else if (backend == "cube") model["RenderBackend"] = "Cube";
        }
    }

bool FStartupCameraSettingsModel::GetCameraModel(const FString& text, const FString& vehicle, const FString& camera, FString& model_text, FString& error)
{
    model_text.Reset();
    ModelJson document;
    if (!ParseCameraSettingsDocument(text, document, error)) return false;
    ModelJson* model = FindModel(document, vehicle, camera, error);
    if (!error.IsEmpty()) return false;
    model_text = model ? UTF8_TO_TCHAR(model->dump(2).c_str()) : TEXT("");
    return true;
}

bool FStartupCameraSettingsModel::ChangeCameraModelType(const FString& text, const FString& vehicle, const FString& camera,
                                                        const FString& type_text, FString& updated, FString& error)
{
    ModelJson document;
    if (!ParseCameraSettingsDocument(text, document, error)) return false;
    ModelJson* model = FindModel(document, vehicle, camera, error);
    if (model == nullptr) {
        if (error.IsEmpty()) error = TEXT("CameraModel is not enabled.");
        return false;
    }
    std::string canonical;
    if (!CanonicalModelType(type_text, canonical)) {
        error = TEXT("Unsupported CameraModel Type.");
        return false;
    }
    (*model)["Type"] = canonical;
    if (canonical == "Raymap")
    {
        for (const char* key : {"Width", "Height", "fx", "fy", "cx", "cy", "FOV_Degrees", "k1", "k2", "k3", "k4", "xi", "alpha"})
            model->erase(key);
        if (!model->contains("Path") || !(*model)["Path"].is_string() || model->at("Path").get<std::string>().empty())
        {
            error = TEXT("Choose a raymap Path before selecting Raymap.");
            return false;
        }
    }
    else
    {
        int32 existing_width = 0, existing_height = 0;
        const int width = model->contains("Width") && TryGetInt32((*model)["Width"], existing_width) && existing_width > 0 ? existing_width : 640;
        const int height = model->contains("Height") && TryGetInt32((*model)["Height"], existing_height) && existing_height > 0 ? existing_height : 480;
        (*model)["Width"] = width;
        (*model)["Height"] = height;
        if (!model->contains("fx") || !(*model)["fx"].is_number() || (*model)["fx"].get<double>() == 0.0)
            (*model)["fx"] = width / 2.0;
        if (!model->contains("fy") || !(*model)["fy"].is_number() || (*model)["fy"].get<double>() == 0.0)
            (*model)["fy"] = width / 2.0;
        if (!model->contains("cx") || !(*model)["cx"].is_number())
            (*model)["cx"] = (width - 1) / 2.0;
        if (!model->contains("cy") || !(*model)["cy"].is_number())
            (*model)["cy"] = (height - 1) / 2.0;
        model->erase("Path");
        if (canonical == "Pinhole") {
            for (const char* key : {"k1", "k2", "k3", "k4", "xi", "alpha"}) model->erase(key);
        }
        else if (canonical == "KannalaBrandt") {
            model->erase("xi");
            model->erase("alpha");
            for (const char* key : {"k1", "k2", "k3", "k4"})
                if (!model->contains(key)) (*model)[key] = 0.0;
        }
        else {
            for (const char* key : {"k1", "k2", "k3", "k4"}) model->erase(key);
            if (!model->contains("xi")) (*model)["xi"] = 0.0;
            if (!model->contains("alpha")) (*model)["alpha"] = 0.5;
        }
    }
    if (!ValidateModel(*model, error)) return false;
    updated = SerializeCameraSettingsDocument(document);
    return true;
}

bool FStartupCameraSettingsModel::AddCameraModel(const FString& text, const FString& vehicle, const FString& camera, const FString& type_text, FString& updated, FString& error)
{
    return AddCameraModel(text, vehicle, camera, type_text, FString(), updated, error);
}

bool FStartupCameraSettingsModel::AddCameraModel(const FString& text, const FString& vehicle, const FString& camera,
                                                 const FString& type_text, const FString& raymap_path,
                                                 FString& updated, FString& error)
{
    ModelJson document;
    if (!ParseCameraSettingsDocument(text, document, error)) return false;
    ModelJson* camera_object = FindCameraSettingsCamera(document, vehicle, camera, error);
    if (camera_object == nullptr) return false;
    std::string type;
    if (!CanonicalModelType(type_text, type)) {
        error = TEXT("Unsupported CameraModel Type.");
        return false;
    }
    if (camera_object->contains("CameraModel")) {
        error = TEXT("CameraModel already exists.");
        return false;
    }
    int width = 640, height = 480;
    if (camera_object->contains("CaptureSettings") && (*camera_object)["CaptureSettings"].is_array())
        for (const auto& c : (*camera_object)["CaptureSettings"])
            if (c.is_object() && (!c.contains("ImageType") || c["ImageType"] == 0))
            {
                int32 capture_width = 0, capture_height = 0;
                if (c.contains("Width") && TryGetInt32(c["Width"], capture_width) && capture_width > 0) width = capture_width;
                if (c.contains("Height") && TryGetInt32(c["Height"], capture_height) && capture_height > 0) height = capture_height;
                break;
            }
    ModelJson model = {{"Type", type}, {"Width", width}, {"Height", height}, {"fx", width / 2.0},
                       {"fy", width / 2.0}, {"cx", (width - 1) / 2.0}, {"cy", (height - 1) / 2.0},
                       {"CubeFaceResolution", 0}, {"Faces", "Auto"}, {"RenderBackend", "Cube"}};
    if (type == "KannalaBrandt") {
        model.update({{"k1", 0.0}, {"k2", 0.0}, {"k3", 0.0}, {"k4", 0.0}});
    } else if (type == "DoubleSphere") {
        model.update({{"xi", 0.0}, {"alpha", 0.5}});
    } else if (type == "Raymap") {
        if (raymap_path.IsEmpty()) {
            error = TEXT("Raymap creation requires a Path.");
            return false;
        }
        for (const char* key : {"Width", "Height", "fx", "fy", "cx", "cy"}) model.erase(key);
        model["Path"] = TCHAR_TO_UTF8(*raymap_path);
    }
    if (!ValidateModel(model, error)) return false;
    (*camera_object)["CameraModel"] = model;
    updated = SerializeCameraSettingsDocument(document);
    return true;
}

bool FStartupCameraSettingsModel::MutateCameraModelField(const FString& text, const FString& vehicle, const FString& camera, const TCHAR* key, EStartupCameraModelFieldType field_type, const FString& scalar, bool clear, FString& updated, FString& error)
{
    ModelJson document;
    if (!ParseCameraSettingsDocument(text, document, error)) return false;
    ModelJson* model = FindModel(document, vehicle, camera, error);
    if (!model) {
        if (error.IsEmpty()) error = TEXT("CameraModel is not enabled.");
        return false;
    }
    const std::string json_key = TCHAR_TO_UTF8(key);
    if (clear) model->erase(json_key);
    else if (field_type == EStartupCameraModelFieldType::String) (*model)[json_key] = TCHAR_TO_UTF8(*scalar);
    else if (field_type == EStartupCameraModelFieldType::Faces)
    {
        if (scalar.Equals(TEXT("Auto"), ESearchCase::IgnoreCase)) (*model)[json_key] = "Auto";
        else if (scalar == TEXT("5") || scalar == TEXT("6")) (*model)[json_key] = TCHAR_TO_UTF8(*scalar);
        else {
            error = TEXT("Faces must be Auto, 5 or 6.");
            return false;
        }
    }
    else if (field_type == EStartupCameraModelFieldType::Backend)
    {
        if (scalar.Equals(TEXT("Cube"), ESearchCase::IgnoreCase)) (*model)[json_key] = "Cube";
        else if (scalar.Equals(TEXT("NativeGEER"), ESearchCase::IgnoreCase)) (*model)[json_key] = "NativeGEER";
        else {
            error = TEXT("RenderBackend must be Cube or NativeGEER.");
            return false;
        }
        model->erase("SplatOnly");
    }
    else if (field_type == EStartupCameraModelFieldType::Integer)
    {
        int32 value;
        if (!ParseCameraInteger(scalar, value) || value < 0) {
            error = TEXT("CameraModel integer must be nonnegative.");
            return false;
        }
        (*model)[json_key] = value;
    }
    else
    {
        double value;
        if (!ParseCameraNumber(scalar, value)) {
            error = TEXT("CameraModel number must be finite.");
            return false;
        }
        (*model)[json_key] = value;
    }
    CanonicalizeModelEnums(*model);
    if (!ValidateModel(*model, error)) return false;
    updated = SerializeCameraSettingsDocument(document);
    return true;
}

bool FStartupCameraSettingsModel::RemoveCameraModel(const FString& text, const FString& vehicle, const FString& camera, FString& updated, FString& error)
{
    ModelJson document;
    if (!ParseCameraSettingsDocument(text, document, error)) return false;
    ModelJson* object = FindCameraSettingsCamera(document, vehicle, camera, error);
    if (!object) return false;
    object->erase("CameraModel");
    updated = SerializeCameraSettingsDocument(document);
    return true;
}

bool FStartupCameraSettingsModel::ApplyCameraModel(const FString& text, const FString& vehicle, const FString& camera, const FString& replacement, FString& updated, FString& error)
{
    ModelJson model;
    try {
        model = ModelJson::parse(TCHAR_TO_UTF8(*replacement));
    }
    catch (...) {
        error = TEXT("CameraModel JSON is malformed.");
        return false;
    }
    if (model.contains("SplatOnly"))
    {
        if (!model["SplatOnly"].is_boolean()) {
            error = TEXT("CameraModel SplatOnly must be boolean.");
            return false;
        }
        if (model.contains("RenderBackend"))
        {
            if (!model["RenderBackend"].is_string()) {
                error = TEXT("RenderBackend must be Cube or NativeGEER.");
                return false;
            }
            std::string backend = model["RenderBackend"].get<std::string>();
            std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (model["SplatOnly"].get<bool>() != (backend == "nativegeer")) {
                error = TEXT("CameraModel SplatOnly conflicts with RenderBackend.");
                return false;
            }
        }
        else model["RenderBackend"] = model["SplatOnly"].get<bool>() ? "NativeGEER" : "Cube";
        model.erase("SplatOnly");
    }
    CanonicalizeModelEnums(model);
    if (!ValidateModel(model, error)) return false;
    ModelJson document;
    if (!ParseCameraSettingsDocument(text, document, error)) return false;
    ModelJson* object = FindCameraSettingsCamera(document, vehicle, camera, error);
    if (!object) return false;
    (*object)["CameraModel"] = model;
    updated = SerializeCameraSettingsDocument(document);
    return true;
}

bool FStartupCameraSettingsModel::CameraModelResolutionMismatch(const FString& text, const FString& vehicle, const FString& camera, FString& message)
{
    message.Reset(); ModelJson document; FString error;
    if (!ParseCameraSettingsDocument(text, document, error)) { message = error; return false; }
    ModelJson* model = FindModel(document, vehicle, camera, error);
    int32 model_width = 0, model_height = 0;
    if (!model || !model->contains("Width") || !model->contains("Height") ||
        !TryGetInt32((*model)["Width"], model_width) || !TryGetInt32((*model)["Height"], model_height)) return false;
    ModelJson* object = FindCameraSettingsCamera(document, vehicle, camera, error);
    if (!object || !object->contains("CaptureSettings") || !(*object)["CaptureSettings"].is_array()) return false;
    for (const auto& capture : (*object)["CaptureSettings"])
    {
        int32 capture_width = 0, capture_height = 0;
        if (capture.is_object() && (!capture.contains("ImageType") || capture["ImageType"] == 0) && capture.contains("Width") && capture.contains("Height") &&
            TryGetInt32(capture["Width"], capture_width) && TryGetInt32(capture["Height"], capture_height) &&
            (capture_width != model_width || capture_height != model_height))
        { message = TEXT("CameraModel Width/Height differs from Scene CaptureSettings; runtime falls back to pinhole."); return true; }
    }
    return false;
}

bool FStartupCameraSettingsModel::ValidateCameraModel(const FString& model_text, FString& error)
{
    error.Reset();
    ModelJson model;
    try { model = ModelJson::parse(TCHAR_TO_UTF8(*model_text)); }
    catch (...) { error = TEXT("CameraModel JSON is malformed."); return false; }
    return ValidateModel(model, error);
}

bool FStartupCameraSettingsModel::SyncCameraModelSizeFromScene(const FString& text, const FString& vehicle, const FString& camera, FString& updated, FString& error)
{
    ModelJson document;
    if (!ParseCameraSettingsDocument(text, document, error)) return false;
    ModelJson* object = FindCameraSettingsCamera(document, vehicle, camera, error);
    if (!object || !object->contains("CameraModel") || !(*object)["CameraModel"].is_object()) { if (error.IsEmpty()) error = TEXT("CameraModel is not enabled."); return false; }
    if ((*object)["CameraModel"].contains("Type") && (*object)["CameraModel"]["Type"].is_string() &&
        FString(UTF8_TO_TCHAR((*object)["CameraModel"]["Type"].get<std::string>().c_str())).Equals(TEXT("Raymap"), ESearchCase::IgnoreCase))
    { error = TEXT("Raymap models use dimensions from the raymap file; Scene size sync is unavailable."); return false; }
    if (!object->contains("CaptureSettings") || !(*object)["CaptureSettings"].is_array()) { error = TEXT("Scene CaptureSettings is absent."); return false; }
    for (const auto& capture : (*object)["CaptureSettings"]) {
        if (capture.is_object() && (!capture.contains("ImageType") || capture["ImageType"] == 0) && capture.contains("Width") && capture.contains("Height") && capture["Width"].is_number_integer() && capture["Height"].is_number_integer()) {
            int32 width = 0, height = 0;
            if (!TryGetInt32(capture["Width"], width) || !TryGetInt32(capture["Height"], height) || width <= 0 || height <= 0)
                continue;
            (*object)["CameraModel"]["Width"] = width;
            (*object)["CameraModel"]["Height"] = height;
            updated = SerializeCameraSettingsDocument(document);
            return true;
        }
    }
    error = TEXT("Scene CaptureSettings has no usable Scene Width/Height.");
    return false;
}
}
