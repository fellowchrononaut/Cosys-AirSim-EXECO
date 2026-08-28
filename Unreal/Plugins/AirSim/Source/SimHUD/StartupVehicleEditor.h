#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

#include <functional>

namespace airsim_startup
{
    /**
     * Native editor for the authored Vehicles object. The launcher owns the raw document; this
     * widget only receives callbacks for reading it and publishing a pretty-printed replacement.
     */
    class SStartupVehicleEditor : public SCompoundWidget
    {
    public:
        using FReadDocument = std::function<FString()>;
        using FWriteDocument = std::function<void(const FString&)>;
        using FStatus = std::function<void(const FString&)>;

        SLATE_BEGIN_ARGS(SStartupVehicleEditor) {}
            SLATE_ARGUMENT(FReadDocument, ReadDocument)
            SLATE_ARGUMENT(FWriteDocument, WriteDocument)
            SLATE_ARGUMENT(FStatus, Status)
        SLATE_END_ARGS()

        void Construct(const FArguments& args);
        void RefreshFromDocument();

    private:
        struct FState;
        TSharedPtr<FState> State;
        FReadDocument ReadDocument;
        FWriteDocument WriteDocument;
        FStatus Status;
    };
}
