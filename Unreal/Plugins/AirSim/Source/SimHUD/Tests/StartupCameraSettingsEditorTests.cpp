#include "../StartupCameraSettingsEditor.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "common/common_utils/json.hpp"

namespace airsim_startup
{
namespace
{
    using CameraSettingsTestJson = nlohmann::json;

    FString CameraSettingsFixture()
    {
        return TEXT(R"JSON({
            "Vehicles": {
                "Drone": {
                    "VehicleType": "simpleflight",
                    "Cameras": {
                        "front": {
                            "PublishToRos": true,
                            "Compress": false,
                            "Annotation": "keep-me",
                            "CaptureSettings": [
                                {"ImageType": 0, "Width": 320, "FOV_Degrees": 90.0, "CustomCapture": {"token": 7}},
                                {"Width": 640, "Sibling": "untouched"}
                            ],
                            "NoiseSettings": [
                                {"Enabled": true, "ImageType": 0, "CustomNoise": "preserve"}
                            ]
                        }
                    }
                },
                "Other": {"VehicleType": "physxcar"}
            },
            "UnknownTopLevel": {"value": 42}
        })JSON");
    }

    CameraSettingsTestJson ParseCameraSettingsTestJson(const FString& text)
    {
        return CameraSettingsTestJson::parse(TCHAR_TO_UTF8(*text));
    }

    bool Mutate(const FString& source, EStartupCameraSettingsCollection collection, int32 index,
                const TCHAR* key, EStartupCameraSettingType type, int32 enum_value,
                bool omit, bool keep_current, const FString& scalar, bool clear,
                FString& output, FString& error)
    {
        return FStartupCameraSettingsModel::MutateField(source, TEXT("Drone"), TEXT("front"), collection,
                                                        index, key, type, enum_value, omit, keep_current,
                                                        scalar, clear, output, error);
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStartupCameraSettingsEditorModelOperations,
    "AirSim.StartupLauncher.CameraSettingsEditor.ModelOperations",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FStartupCameraSettingsEditorModelOperations::RunTest(const FString&)
{
    const FString original = CameraSettingsFixture();
    FString updated = TEXT("sentinel");
    FString error = TEXT("stale error");

    const FString absent_collection = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{}}}}})JSON");
    TestTrue(TEXT("Add creates an absent collection array"), FStartupCameraSettingsModel::AddEntry(
        absent_collection, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Capture, 5, updated, error));
    CameraSettingsTestJson document = ParseCameraSettingsTestJson(updated);
    TestEqual(TEXT("Absent collection receives one entry"), document["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"].size(), static_cast<size_t>(1));
    TestEqual(TEXT("Added ImageType is authored"), document["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"][0]["ImageType"].get<int32>(), 5);

    updated = TEXT("sentinel");
    error.Reset();
    TestTrue(TEXT("Duplicate to a new ImageType succeeds"), FStartupCameraSettingsModel::DuplicateEntry(
        original, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Capture, 0, 3, updated, error));
    document = ParseCameraSettingsTestJson(updated);
    TestEqual(TEXT("Duplicate is appended"), document["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"].size(), static_cast<size_t>(3));
    TestEqual(TEXT("Duplicate changes ImageType"), document["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"][2]["ImageType"].get<int32>(), 3);
    TestEqual(TEXT("Duplicate preserves unknown nested data"), document["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"][2]["CustomCapture"]["token"].get<int32>(), 7);
    TestTrue(TEXT("Duplicate preserves camera siblings"), document["Vehicles"]["Drone"]["Cameras"]["front"].contains("PublishToRos"));
    TestTrue(TEXT("Duplicate preserves compression sibling"), document["Vehicles"]["Drone"]["Cameras"]["front"].contains("Compress"));
    TestTrue(TEXT("Duplicate preserves annotation sibling"), document["Vehicles"]["Drone"]["Cameras"]["front"].contains("Annotation"));
    TestTrue(TEXT("Duplicate preserves top-level siblings"), document.contains("UnknownTopLevel"));

    FString list_source = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CaptureSettings":[{"ImageType":11},{"ImageType":99},{"Width":1}]}}}}})JSON");
    TArray<FStartupCameraEntryInfo> entries;
    TestTrue(TEXT("List accepts authored and omitted ImageType"), FStartupCameraSettingsModel::ListEntries(
        list_source, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Capture, entries, error));
    TestEqual(TEXT("List returns all entries"), entries.Num(), 3);
    TestTrue(TEXT("Known ImageType label is present"), entries[0].Label.Contains(TEXT("Annotation")));
    TestTrue(TEXT("Unknown ImageType label is present"), entries[1].Label.Contains(TEXT("Unknown (99)")));
    TestTrue(TEXT("Missing ImageType defaults to zero"), entries[2].ImageType == 0);

    FString delete_source = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CaptureSettings":[{"ImageType":1,"Name":"first"},{"ImageType":2,"Name":"second"},{"ImageType":3,"Name":"third"}]}}}}})JSON");
    TestTrue(TEXT("Delete preserves remaining order"), FStartupCameraSettingsModel::DeleteEntry(
        delete_source, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Capture, 1, updated, error));
    document = ParseCameraSettingsTestJson(updated);
    TestEqual(TEXT("First remaining entry stays first"), UTF8_TO_TCHAR(document["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"][0]["Name"].get<std::string>().c_str()), TEXT("first"));
    TestEqual(TEXT("Second remaining entry shifts into deleted slot"), UTF8_TO_TCHAR(document["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"][1]["Name"].get<std::string>().c_str()), TEXT("third"));
    const FString one_entry = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CaptureSettings":[{"ImageType":8}]}}}}})JSON");
    TestTrue(TEXT("Delete last entry removes collection key"), FStartupCameraSettingsModel::DeleteEntry(
        one_entry, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Capture, 0, updated, error));
    TestFalse(TEXT("Last-entry deletion removes CaptureSettings"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"].contains("CaptureSettings"));

    TestTrue(TEXT("Integer mutation succeeds"), Mutate(original, EStartupCameraSettingsCollection::Capture, 0, TEXT("Width"), EStartupCameraSettingType::Integer, 0, false, false, TEXT("800"), false, updated, error));
    TestTrue(TEXT("Finite number mutation succeeds"), Mutate(updated, EStartupCameraSettingsCollection::Capture, 0, TEXT("FOV_Degrees"), EStartupCameraSettingType::Number, 0, false, false, TEXT("91.5"), false, updated, error));
    TestTrue(TEXT("Boolean mutation uses enum value"), Mutate(updated, EStartupCameraSettingsCollection::Noise, 0, TEXT("Enabled"), EStartupCameraSettingType::Boolean, 0, false, false, FString(), false, updated, error));
    TestTrue(TEXT("Projection mutation accepts orthographic"), Mutate(updated, EStartupCameraSettingsCollection::Capture, 0, TEXT("ProjectionMode"), EStartupCameraSettingType::ProjectionMode, 0, false, false, TEXT("orthographic"), false, updated, error));
    document = ParseCameraSettingsTestJson(updated);
    TestEqual(TEXT("Integer is serialized as integer"), document["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"][0]["Width"].get<int32>(), 800);
    TestEqual(TEXT("Number is serialized as number"), document["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"][0]["FOV_Degrees"].get<double>(), 91.5);
    TestFalse(TEXT("Boolean false is authored"), document["Vehicles"]["Drone"]["Cameras"]["front"]["NoiseSettings"][0]["Enabled"].get<bool>());
    TestEqual(TEXT("Projection mode is serialized"), UTF8_TO_TCHAR(document["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"][0]["ProjectionMode"].get<std::string>().c_str()), TEXT("orthographic"));

    TestTrue(TEXT("Clear omits selected field"), Mutate(updated, EStartupCameraSettingsCollection::Capture, 0, TEXT("Width"), EStartupCameraSettingType::Integer, 0, false, false, FString(), true, updated, error));
    TestFalse(TEXT("Clear removes only selected field"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CaptureSettings"][0].contains("Width"));

    const FString image_type_only = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"NoiseSettings":[{"ImageType":5}]}}}}})JSON");
    TestTrue(TEXT("ImageType omit succeeds when zero remains available"), Mutate(image_type_only, EStartupCameraSettingsCollection::Noise, 0, TEXT("ImageType"), EStartupCameraSettingType::ImageType, 0, true, false, FString(), false, updated, error));
    TestFalse(TEXT("ImageType omit removes ImageType"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["NoiseSettings"][0].contains("ImageType"));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStartupCameraSettingsEditorModelValidation,
    "AirSim.StartupLauncher.CameraSettingsEditor.ModelValidation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FStartupCameraSettingsEditorModelValidation::RunTest(const FString&)
{
    FString error;
    FString output = TEXT("sentinel");
    const FString original = CameraSettingsFixture();

    TestFalse(TEXT("ProjectionMode rejects unsupported values"), Mutate(original, EStartupCameraSettingsCollection::Capture, 0, TEXT("ProjectionMode"), EStartupCameraSettingType::ProjectionMode, 0, false, false, TEXT("fisheye"), false, output, error));
    TestEqual(TEXT("Projection refusal leaves output unchanged"), output, TEXT("sentinel"));

    TestFalse(TEXT("ImageType mutation refuses effective collision"), Mutate(original, EStartupCameraSettingsCollection::Capture, 0, TEXT("ImageType"), EStartupCameraSettingType::ImageType, 0, false, false, FString(), false, output, error));
    TestEqual(TEXT("ImageType refusal leaves output unchanged"), output, TEXT("sentinel"));

    TestFalse(TEXT("Duplicate refuses effective ImageType collision with omitted field"), FStartupCameraSettingsModel::DuplicateEntry(
        original, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Capture, 0, 0, output, error));
    TestEqual(TEXT("Duplicate refusal leaves output unchanged"), output, TEXT("sentinel"));

    const FString omitted_image_type = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CaptureSettings":[{"Width":1}]}}}}})JSON");
    TestFalse(TEXT("Add refuses duplicate effective ImageType when existing field is omitted"), FStartupCameraSettingsModel::AddEntry(
        omitted_image_type, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Capture, 0, output, error));
    TestEqual(TEXT("Omitted ImageType duplicate leaves output unchanged"), output, TEXT("sentinel"));

    const FString image_type_clear_collision = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CaptureSettings":[{"ImageType":1},{"Width":1}]}}}}})JSON");
    TestFalse(TEXT("ImageType clear refuses effective zero collision"), Mutate(image_type_clear_collision, EStartupCameraSettingsCollection::Capture, 0, TEXT("ImageType"), EStartupCameraSettingType::ImageType, 0, false, false, FString(), true, output, error));
    TestEqual(TEXT("ImageType clear refusal leaves output unchanged"), output, TEXT("sentinel"));

    TestFalse(TEXT("Boolean mutation rejects an invalid enum value"), Mutate(original, EStartupCameraSettingsCollection::Noise, 0, TEXT("Enabled"), EStartupCameraSettingType::Boolean, 2, false, false, FString(), false, output, error));
    TestEqual(TEXT("Invalid boolean refusal leaves output unchanged"), output, TEXT("sentinel"));

    const FString wrong_collection = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CaptureSettings":{}}}}}})JSON");
    TestFalse(TEXT("Wrong collection type is refused"), FStartupCameraSettingsModel::AddEntry(
        wrong_collection, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Capture, 0, output, error));
    TestEqual(TEXT("Wrong collection refusal leaves output unchanged"), output, TEXT("sentinel"));

    const FString non_object_entry = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CaptureSettings":[1]}}}}}})JSON");
    TArray<FStartupCameraEntryInfo> entries;
    TestFalse(TEXT("Non-object entry is refused"), FStartupCameraSettingsModel::ListEntries(
        non_object_entry, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Capture, entries, error));
    TestEqual(TEXT("Entry output is cleared on refusal"), entries.Num(), 0);
    TestFalse(TEXT("Add refuses non-object entries"), FStartupCameraSettingsModel::AddEntry(
        non_object_entry, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Capture, 2, output, error));

    const FString replacement = TEXT(R"JSON({"ImageType":5,"Replacement":true})JSON");
    TestTrue(TEXT("Apply accepts a raw object"), FStartupCameraSettingsModel::ApplyEntry(
        original, TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Noise, 0, replacement, output, error));
    CameraSettingsTestJson document = ParseCameraSettingsTestJson(output);
    TestTrue(TEXT("Apply replaces selected entry"), document["Vehicles"]["Drone"]["Cameras"]["front"]["NoiseSettings"][0].contains("Replacement"));
    TestTrue(TEXT("Apply preserves other vehicle"), document["Vehicles"].contains("Other"));
    TestTrue(TEXT("Absent collection lists successfully"), FStartupCameraSettingsModel::ListEntries(
        TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{}}}}})JSON"), TEXT("Drone"), TEXT("front"), EStartupCameraSettingsCollection::Noise, entries, error));
    TestEqual(TEXT("Absent collection lists empty"), entries.Num(), 0);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStartupCameraSettingsEditorDoesNotReadDuringConstruct,
    "AirSim.StartupLauncher.CameraSettingsEditor.DoesNotReadDuringConstruct",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FStartupCameraSettingsEditorDoesNotReadDuringConstruct::RunTest(const FString&)
{
    int32 read_count = 0;
    FString document = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{}}}}})JSON");
    TSharedPtr<SStartupCameraSettingsEditor> editor = SNew(SStartupCameraSettingsEditor)
        .ReadDocument([&read_count, &document]() { ++read_count; return document; })
        .WriteDocument([&document](const FString& updated) { document = updated; })
        .Status([](const FString&) {})
        .SelectedVehicle([]() { return FString(TEXT("Drone")); })
        .SelectedCamera([]() { return FString(TEXT("front")); });

    TestEqual(TEXT("Construct performs no document reads"), read_count, 0);
    editor->RefreshFromDocument();
    TestTrue(TEXT("Explicit refresh performs a document read"), read_count > 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStartupCameraSettingsEditorRefreshWithoutCameraIsQuiet,
    "AirSim.StartupLauncher.CameraSettingsEditor.RefreshWithoutCameraIsQuiet",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FStartupCameraSettingsEditorRefreshWithoutCameraIsQuiet::RunTest(const FString&)
{
    int32 status_count = 0;
    int32 write_count = 0;
    const FString document = TEXT(R"JSON({"Vehicles":{"Scout":{"VehicleType":"simpleflight"}}})JSON");
    TSharedPtr<SStartupCameraSettingsEditor> editor = SNew(SStartupCameraSettingsEditor)
        .ReadDocument([&document]() { return document; })
        .WriteDocument([&write_count](const FString&) { ++write_count; })
        .Status([&status_count](const FString&) { ++status_count; })
        .SelectedVehicle([]() { return FString(TEXT("Scout")); })
        .SelectedCamera([]() { return FString(); });

    editor->RefreshFromDocument();
    TestEqual(TEXT("Refreshing a vehicle with no camera is quiet"), status_count, 0);
    TestEqual(TEXT("Refreshing a vehicle with no camera does not write"), write_count, 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FStartupCameraSettingsEditorCameraModelOperations,
    "AirSim.StartupLauncher.CameraSettingsEditor.CameraModelOperations",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FStartupCameraSettingsEditorCameraModelOperations::RunTest(const FString&)
{
    const FString source = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"Annotation":"keep","CaptureSettings":[{"ImageType":0,"Width":800,"Height":600}]}}}}})JSON");
    FString updated, error;
    const TCHAR* types[] = {TEXT("Pinhole"), TEXT("KannalaBrandt"), TEXT("DoubleSphere"), TEXT("Raymap")};
    for (const TCHAR* type : types)
    {
        updated.Reset(); error.Reset();
        const bool added = FCString::Stricmp(type, TEXT("Raymap")) == 0
                               ? FStartupCameraSettingsModel::AddCameraModel(source, TEXT("Drone"), TEXT("front"), type, TEXT("camera.raymap"), updated, error)
                               : FStartupCameraSettingsModel::AddCameraModel(source, TEXT("Drone"), TEXT("front"), type, updated, error);
        TestTrue(FString::Printf(TEXT("Add %s model"), type), added);
        TestTrue(TEXT("Camera sibling survives model add"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"].contains("Annotation"));
    }

    const FString raymap = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CameraModel":{"Type":"raymap","Path":"old.raymap","Unknown":7},"CaptureSettings":[{"ImageType":0,"Width":800,"Height":600}]}}}}})JSON");
    TestTrue(TEXT("Raymap path mutation succeeds"), FStartupCameraSettingsModel::MutateCameraModelField(raymap, TEXT("Drone"), TEXT("front"), TEXT("Path"), EStartupCameraModelFieldType::String, TEXT("new.raymap"), false, updated, error));
    TestEqual(TEXT("Raymap path is editable"), UTF8_TO_TCHAR(ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"]["Path"].get<std::string>().c_str()), TEXT("new.raymap"));
    TestTrue(TEXT("Unknown model field survives mutation"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"].contains("Unknown"));

    const FString typed = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CameraModel":{"Type":"Pinhole","Width":800,"Height":600,"fx":400,"fy":400,"cx":399.5,"cy":299.5,"Unknown":9}}}}}})JSON");
    TestTrue(TEXT("Type switch creates a valid DoubleSphere template"), FStartupCameraSettingsModel::ChangeCameraModelType(typed, TEXT("Drone"), TEXT("front"), TEXT("DoubleSphere"), updated, error));
    TestEqual(TEXT("Type switch updates Type"), UTF8_TO_TCHAR(ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"]["Type"].get<std::string>().c_str()), TEXT("DoubleSphere"));
    TestTrue(TEXT("Type switch preserves unknown model fields"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"].contains("Unknown"));

    TestTrue(TEXT("FOV-only pinhole validates"), FStartupCameraSettingsModel::ValidateCameraModel(TEXT(R"JSON({"Type":"Pinhole","Width":640,"Height":480,"FOV_Degrees":90})JSON"), error));
    TestTrue(TEXT("Double Sphere alpha accepts zero"), FStartupCameraSettingsModel::ValidateCameraModel(TEXT(R"JSON({"Type":"DoubleSphere","Width":640,"Height":480,"fx":300,"fy":300,"cx":319.5,"cy":239.5,"alpha":0})JSON"), error));
    TestTrue(TEXT("Double Sphere omitted alpha uses parser default"), FStartupCameraSettingsModel::ValidateCameraModel(TEXT(R"JSON({"Type":"DoubleSphere","Width":640,"Height":480,"fx":300,"fy":300,"cx":319.5,"cy":239.5})JSON"), error));
    TestFalse(TEXT("Kannala-Brandt missing centers is rejected"), FStartupCameraSettingsModel::ValidateCameraModel(TEXT(R"JSON({"Type":"KannalaBrandt","Width":640,"Height":480,"fx":300,"fy":300})JSON"), error));
    TestFalse(TEXT("Double Sphere missing centers is rejected"), FStartupCameraSettingsModel::ValidateCameraModel(TEXT(R"JSON({"Type":"DoubleSphere","Width":640,"Height":480,"fx":300,"fy":300,"alpha":0.5})JSON"), error));
    TestTrue(TEXT("Type case is accepted"), FStartupCameraSettingsModel::ValidateCameraModel(TEXT(R"JSON({"Type":"dOuBlEsPhErE","Width":640,"Height":480,"fx":300,"fy":300,"cx":319.5,"cy":239.5,"alpha":0.5})JSON"), error));
    TestFalse(TEXT("Legacy SplatOnly conflict is rejected"), FStartupCameraSettingsModel::ApplyCameraModel(source, TEXT("Drone"), TEXT("front"), TEXT(R"JSON({"Type":"Pinhole","Width":640,"Height":480,"fx":300,"fy":300,"SplatOnly":true,"RenderBackend":"Cube"})JSON"), updated, error));
    TestTrue(TEXT("Legacy SplatOnly-only migration succeeds"), FStartupCameraSettingsModel::ApplyCameraModel(source, TEXT("Drone"), TEXT("front"), TEXT(R"JSON({"Type":"Pinhole","Width":640,"Height":480,"fx":300,"fy":300,"SplatOnly":true})JSON"), updated, error));
    TestTrue(TEXT("Legacy SplatOnly key is removed"), !ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"].contains("SplatOnly"));
    TestEqual(TEXT("Legacy SplatOnly writes NativeGEER"), UTF8_TO_TCHAR(ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"]["RenderBackend"].get<std::string>().c_str()), TEXT("NativeGEER"));
    TestTrue(TEXT("Consistent dual backend keys migrate"), FStartupCameraSettingsModel::ApplyCameraModel(source, TEXT("Drone"), TEXT("front"), TEXT(R"JSON({"Type":"Pinhole","Width":640,"Height":480,"fx":300,"fy":300,"SplatOnly":true,"RenderBackend":"nativegeer"})JSON"), updated, error));
    TestTrue(TEXT("Consistent migration removes SplatOnly"), !ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"].contains("SplatOnly"));
    TestEqual(TEXT("Consistent migration canonicalizes backend"), UTF8_TO_TCHAR(ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"]["RenderBackend"].get<std::string>().c_str()), TEXT("NativeGEER"));
    const FString apply_siblings = TEXT(R"JSON({"UnknownTop":42,"Vehicles":{"Drone":{"Cameras":{"front":{"CameraSibling":true,"CameraModel":{"Type":"Pinhole","Width":640,"Height":480,"fx":300,"fy":300,"ModelSibling":7}}}}}})JSON");
    TestTrue(TEXT("Apply model preserves unknown model key and siblings"), FStartupCameraSettingsModel::ApplyCameraModel(apply_siblings, TEXT("Drone"), TEXT("front"), TEXT(R"JSON({"Type":"Pinhole","Width":640,"Height":480,"fx":300,"fy":300,"NewModelKey":9})JSON"), updated, error));
    TestTrue(TEXT("Apply preserves top-level sibling"), ParseCameraSettingsTestJson(updated).contains("UnknownTop"));
    TestTrue(TEXT("Apply preserves camera sibling"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"].contains("CameraSibling"));
    TestTrue(TEXT("Apply preserves replacement model key"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"].contains("NewModelKey"));

    const FString mismatch = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CameraModel":{"Type":"Pinhole","Width":640,"Height":480,"fx":300,"fy":300},"CaptureSettings":[{"ImageType":0,"Width":800,"Height":600}]}}}}})JSON");
    FString message;
    TestTrue(TEXT("Resolution mismatch is reported"), FStartupCameraSettingsModel::CameraModelResolutionMismatch(mismatch, TEXT("Drone"), TEXT("front"), message));
    TestTrue(TEXT("Mismatch text is meaningful"), message.Contains(TEXT("differs")));
    TestTrue(TEXT("Model size sync succeeds"), FStartupCameraSettingsModel::SyncCameraModelSizeFromScene(mismatch, TEXT("Drone"), TEXT("front"), updated, error));
    TestEqual(TEXT("Sync copies scene width"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"]["Width"].get<int32>(), 800);
    TestEqual(TEXT("Sync copies scene height"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"]["CameraModel"]["Height"].get<int32>(), 600);
    const FString invalid_scene = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CameraModel":{"Type":"Pinhole","Width":640,"Height":480,"fx":300,"fy":300},"CaptureSettings":[{"ImageType":0,"Width":0,"Height":600}]}}}}})JSON");
    TestFalse(TEXT("Sync refuses zero scene size"), FStartupCameraSettingsModel::SyncCameraModelSizeFromScene(invalid_scene, TEXT("Drone"), TEXT("front"), updated, error));
    const FString negative_scene = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CameraModel":{"Type":"Pinhole","Width":640,"Height":480,"fx":300,"fy":300},"CaptureSettings":[{"ImageType":0,"Width":-1,"Height":600}]}}}}})JSON");
    TestFalse(TEXT("Sync refuses negative scene size"), FStartupCameraSettingsModel::SyncCameraModelSizeFromScene(negative_scene, TEXT("Drone"), TEXT("front"), updated, error));
    const FString non_integer_scene = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CameraModel":{"Type":"Pinhole","Width":640,"Height":480,"fx":300,"fy":300},"CaptureSettings":[{"ImageType":0,"Width":800.5,"Height":600}]}}}}})JSON");
    TestFalse(TEXT("Sync refuses non-integer scene size"), FStartupCameraSettingsModel::SyncCameraModelSizeFromScene(non_integer_scene, TEXT("Drone"), TEXT("front"), updated, error));
    const FString raymap_model = TEXT(R"JSON({"Vehicles":{"Drone":{"Cameras":{"front":{"CameraModel":{"Type":"Raymap","Path":"camera.raymap"},"CaptureSettings":[{"ImageType":0,"Width":800,"Height":600}]}}}}})JSON");
    TestFalse(TEXT("Sync refuses Raymap dimensions"), FStartupCameraSettingsModel::SyncCameraModelSizeFromScene(raymap_model, TEXT("Drone"), TEXT("front"), updated, error));
    TestTrue(TEXT("Remove model lifecycle succeeds"), FStartupCameraSettingsModel::RemoveCameraModel(raymap_model, TEXT("Drone"), TEXT("front"), updated, error));
    TestFalse(TEXT("Remove model erases CameraModel"), ParseCameraSettingsTestJson(updated)["Vehicles"]["Drone"]["Cameras"]["front"].contains("CameraModel"));
    return true;
}
}

#endif
