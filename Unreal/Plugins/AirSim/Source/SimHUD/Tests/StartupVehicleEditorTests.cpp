#include "../StartupVehicleEditor.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"

namespace airsim_startup
{
IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStartupVehicleEditorDoesNotReadDuringConstruct,
    "AirSim.StartupLauncher.VehicleEditor.DoesNotReadDuringConstruct",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FStartupVehicleEditorDoesNotReadDuringConstruct::RunTest(const FString&)
{
    int32 read_count = 0;
    FString document = TEXT("{\"Vehicles\":{\"Drone\":{\"VehicleType\":\"simpleflight\"}}}");
    TSharedPtr<SStartupVehicleEditor> editor = SNew(SStartupVehicleEditor)
        .ReadDocument([&read_count, &document]()
        {
            ++read_count;
            return document;
        })
        .WriteDocument([&document](const FString& updated)
        {
            document = updated;
        })
        .Status([](const FString&) {});

    TestEqual(TEXT("Construct performs no document reads"), read_count, 0);
    editor->RefreshFromDocument();
    TestTrue(TEXT("Explicit refresh performs a document read"), read_count > 0);
    return true;
}
}

#endif
