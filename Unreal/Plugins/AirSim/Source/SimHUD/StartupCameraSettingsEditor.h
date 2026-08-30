#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

#include <functional>

namespace airsim_startup
{
    enum class EStartupCameraSettingsCollection { Capture, Noise };
    enum class EStartupCameraSettingType { Integer, Number, Boolean, ImageType, ProjectionMode };
    enum class EStartupCameraModelFieldType { Integer, Number, String, Type, Faces, Backend };

    struct FStartupCameraEntryInfo
    {
        int32 Index = INDEX_NONE;
        int32 ImageType = 0;
        FString Label;
    };

    /** Pure document-preserving operations used by the camera settings widget and tests. */
    class FStartupCameraSettingsModel
    {
    public:
        static bool ListEntries(const FString& document_text, const FString& vehicle_name,
                                const FString& camera_name, EStartupCameraSettingsCollection collection,
                                TArray<FStartupCameraEntryInfo>& entries, FString& error);
        static bool AddEntry(const FString& document_text, const FString& vehicle_name,
                             const FString& camera_name, EStartupCameraSettingsCollection collection,
                             int32 image_type, FString& updated_text, FString& error);
        static bool DuplicateEntry(const FString& document_text, const FString& vehicle_name,
                                   const FString& camera_name, EStartupCameraSettingsCollection collection,
                                   int32 source_index, int32 target_image_type,
                                   FString& updated_text, FString& error);
        static bool DeleteEntry(const FString& document_text, const FString& vehicle_name,
                                const FString& camera_name, EStartupCameraSettingsCollection collection,
                                int32 index, FString& updated_text, FString& error);
        static bool MutateField(const FString& document_text, const FString& vehicle_name,
                                const FString& camera_name, EStartupCameraSettingsCollection collection,
                                int32 index, const TCHAR* key, EStartupCameraSettingType type,
                                int32 enum_value, bool omit, bool keep_current, const FString& scalar_text,
                                bool clear, FString& updated_text, FString& error);
        static bool ApplyEntry(const FString& document_text, const FString& vehicle_name,
                               const FString& camera_name, EStartupCameraSettingsCollection collection,
                               int32 index, const FString& replacement_text,
                               FString& updated_text, FString& error);

        static bool GetCameraModel(const FString& document_text, const FString& vehicle_name,
                                   const FString& camera_name, FString& model_text, FString& error);
        static bool AddCameraModel(const FString& document_text, const FString& vehicle_name,
                                   const FString& camera_name, const FString& type,
                                   FString& updated_text, FString& error);
        static bool AddCameraModel(const FString& document_text, const FString& vehicle_name,
                                   const FString& camera_name, const FString& type, const FString& raymap_path,
                                   FString& updated_text, FString& error);
        static bool MutateCameraModelField(const FString& document_text, const FString& vehicle_name,
                                           const FString& camera_name, const TCHAR* key,
                                           EStartupCameraModelFieldType type, const FString& scalar_text,
                                           bool clear, FString& updated_text, FString& error);
        static bool ChangeCameraModelType(const FString& document_text, const FString& vehicle_name,
                                          const FString& camera_name, const FString& type,
                                          FString& updated_text, FString& error);
        static bool RemoveCameraModel(const FString& document_text, const FString& vehicle_name,
                                      const FString& camera_name, FString& updated_text, FString& error);
        static bool ApplyCameraModel(const FString& document_text, const FString& vehicle_name,
                                     const FString& camera_name, const FString& replacement_text,
                                     FString& updated_text, FString& error);
        static bool CameraModelResolutionMismatch(const FString& document_text, const FString& vehicle_name,
                                                  const FString& camera_name, FString& message);
        static bool ValidateCameraModel(const FString& model_text, FString& error);
        static bool SyncCameraModelSizeFromScene(const FString& document_text, const FString& vehicle_name,
                                                 const FString& camera_name, FString& updated_text, FString& error);
    };

    class SStartupCameraSettingsEditor : public SCompoundWidget
    {
    public:
        using FReadDocument = std::function<FString()>;
        using FWriteDocument = std::function<void(const FString&)>;
        using FStatus = std::function<void(const FString&)>;
        using FSelectedVehicle = std::function<FString()>;
        using FSelectedCamera = std::function<FString()>;

        SLATE_BEGIN_ARGS(SStartupCameraSettingsEditor) {}
            SLATE_ARGUMENT(FReadDocument, ReadDocument)
            SLATE_ARGUMENT(FWriteDocument, WriteDocument)
            SLATE_ARGUMENT(FStatus, Status)
            SLATE_ARGUMENT(FSelectedVehicle, SelectedVehicle)
            SLATE_ARGUMENT(FSelectedCamera, SelectedCamera)
        SLATE_END_ARGS()

        void Construct(const FArguments& args);
        void RefreshFromDocument();

    public:
        struct FState;

    private:
        TSharedPtr<FState> State;
    };
}
