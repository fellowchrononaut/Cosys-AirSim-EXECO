#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

#include <functional>

namespace airsim_startup
{
    /**
     * Small, document-preserving editor for one named per-vehicle collection.  The parent
     * vehicle editor owns the selected vehicle; this widget only receives callbacks and never
     * reads the document during Construct.
     */
    class SStartupSensorEditor : public SCompoundWidget
    {
    public:
        using FReadDocument = std::function<FString()>;
        using FWriteDocument = std::function<void(const FString&)>;
        using FStatus = std::function<void(const FString&)>;
        using FSelectedVehicle = std::function<FString()>;

        SLATE_BEGIN_ARGS(SStartupSensorEditor) {}
            SLATE_ARGUMENT(FReadDocument, ReadDocument)
            SLATE_ARGUMENT(FWriteDocument, WriteDocument)
            SLATE_ARGUMENT(FStatus, Status)
            SLATE_ARGUMENT(FSelectedVehicle, SelectedVehicle)
            SLATE_ARGUMENT(FString, CollectionName)
        SLATE_END_ARGS()

        void Construct(const FArguments& args);
        void RefreshFromDocument();

    private:
        struct FState;
        TSharedPtr<FState> State;
    };
}
