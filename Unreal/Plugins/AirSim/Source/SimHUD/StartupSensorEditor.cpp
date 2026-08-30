#include "StartupSensorEditor.h"
#include "StartupCameraSettingsEditor.h"

#include "Misc/MessageDialog.h"
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
#include "common/common_utils/json.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace airsim_startup
{
namespace
{
    using Json = nlohmann::json;

    enum class ENestedFieldType { Number, String, Boolean, SensorType };

    struct FNestedFieldDefinition
    {
        const TCHAR* label;
        const TCHAR* key;
        ENestedFieldType type;
    };

    struct FNestedFieldRow
    {
        FNestedFieldDefinition definition;
        TArray<TSharedPtr<FString>> choices;
        TArray<int32> values;
        TArray<bool> omit;
        TArray<bool> keep;
        TSharedPtr<SComboBox<TSharedPtr<FString>>> combo;
        TSharedPtr<SEditableTextBox> text;
    };

    const FNestedFieldDefinition GCameraFields[] = {
        {TEXT("X"), TEXT("X"), ENestedFieldType::Number},
        {TEXT("Y"), TEXT("Y"), ENestedFieldType::Number},
        {TEXT("Z"), TEXT("Z"), ENestedFieldType::Number},
        {TEXT("Pitch"), TEXT("Pitch"), ENestedFieldType::Number},
        {TEXT("Roll"), TEXT("Roll"), ENestedFieldType::Number},
        {TEXT("Yaw"), TEXT("Yaw"), ENestedFieldType::Number},
        {TEXT("External"), TEXT("External"), ENestedFieldType::Boolean},
        {TEXT("External local"), TEXT("ExternalLocal"), ENestedFieldType::Boolean},
        {TEXT("Draw sensor"), TEXT("DrawSensor"), ENestedFieldType::Boolean},
        {TEXT("Link"), TEXT("Link"), ENestedFieldType::String}
    };

    const FNestedFieldDefinition GSensorFields[] = {
        {TEXT("SensorType"), TEXT("SensorType"), ENestedFieldType::SensorType},
        {TEXT("Enabled"), TEXT("Enabled"), ENestedFieldType::Boolean},
        {TEXT("X"), TEXT("X"), ENestedFieldType::Number},
        {TEXT("Y"), TEXT("Y"), ENestedFieldType::Number},
        {TEXT("Z"), TEXT("Z"), ENestedFieldType::Number},
        {TEXT("Pitch"), TEXT("Pitch"), ENestedFieldType::Number},
        {TEXT("Roll"), TEXT("Roll"), ENestedFieldType::Number},
        {TEXT("Yaw"), TEXT("Yaw"), ENestedFieldType::Number},
        {TEXT("Link"), TEXT("Link"), ENestedFieldType::String},
        {TEXT("External"), TEXT("External"), ENestedFieldType::Boolean},
        {TEXT("External local"), TEXT("ExternalLocal"), ENestedFieldType::Boolean},
        {TEXT("Draw sensor"), TEXT("DrawSensor"), ENestedFieldType::Boolean}
    };

    const TCHAR* GSensorTypeLabels[] = {
        TEXT("Barometer (1)"), TEXT("Imu (2)"), TEXT("Gps (3)"), TEXT("Magnetometer (4)"),
        TEXT("Distance (5)"), TEXT("Lidar (6)"), TEXT("Echo (7)"), TEXT("GPULidar (8)"),
        TEXT("SensorTemplate (9)"), TEXT("MarlocUwb (10)"), TEXT("Wifi (11)")
    };
    const int32 GSensorTypeValues[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

    bool ParseDocument(const FString& text, Json& document, FString& error)
    {
        try
        {
            document = Json::parse(TCHAR_TO_UTF8(*text));
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

    Json* FindVehicle(Json& document, const FString& name)
    {
        if (name.IsEmpty() || !document.contains("Vehicles") || !document["Vehicles"].is_object())
            return nullptr;
        const std::string key = TCHAR_TO_UTF8(*name);
        if (!document["Vehicles"].contains(key) || !document["Vehicles"][key].is_object())
            return nullptr;
        return &document["Vehicles"][key];
    }

    Json* FindCollection(Json& document, const FString& vehicle_name, const FString& collection_name)
    {
        Json* vehicle = FindVehicle(document, vehicle_name);
        if (vehicle == nullptr || !vehicle->contains(TCHAR_TO_UTF8(*collection_name)))
            return nullptr;
        Json& collection = (*vehicle)[TCHAR_TO_UTF8(*collection_name)];
        return collection.is_object() ? &collection : nullptr;
    }

    bool ParseNumber(const FString& text, double& result)
    {
        const std::string raw = TCHAR_TO_UTF8(*text);
        char* end = nullptr;
        errno = 0;
        result = std::strtod(raw.c_str(), &end);
        return errno != ERANGE && end != raw.c_str() && *end == '\0' && std::isfinite(result);
    }
}

struct SStartupSensorEditor::FState
{
    FReadDocument Read;
    FWriteDocument Write;
    FStatus Status;
    FSelectedVehicle SelectedVehicle;
    FString CollectionName;
    bool IsCamera = false;
    bool Updating = false;

    TArray<TSharedPtr<FString>> Names;
    TSharedPtr<FString> Selected;
    TArray<TSharedPtr<FString>> SensorTypeChoices;
    TArray<int32> SensorTypeValues;
    TSharedPtr<SListView<TSharedPtr<FString>>> List;
    TSharedPtr<SEditableTextBox> AddName;
    TSharedPtr<SEditableTextBox> DuplicateName;
    TSharedPtr<SEditableTextBox> RenameName;
    TSharedPtr<SComboBox<TSharedPtr<FString>>> AddSensorType;
    TSharedPtr<SComboBox<TSharedPtr<FString>>> AddEnabled;
    TArray<TSharedPtr<FString>> AddEnabledChoices;
    TSharedPtr<SMultiLineEditableTextBox> Advanced;
    TSharedPtr<SVerticalBox> Fields;
    TSharedPtr<STextBlock> InfoText;
    TArray<TSharedPtr<FNestedFieldRow>> Rows;
    TSharedPtr<SStartupCameraSettingsEditor> CameraSettings;
    TSharedPtr<SWidgetSwitcher> CameraSubview;

    void Report(const FString& message) const
    {
        if (Status)
            Status(message);
    }

    FString VehicleName() const
    {
        return SelectedVehicle ? SelectedVehicle() : FString();
    }

    bool ReadJson(Json& document, FString& error) const
    {
        if (!Read)
        {
            error = TEXT("Nested collection document callback is unavailable.");
            return false;
        }
        return ParseDocument(Read(), document, error);
    }

    Json* SelectedObject(Json& document) const
    {
        Json* collection = FindCollection(document, VehicleName(), CollectionName);
        if (collection == nullptr || !Selected.IsValid())
            return nullptr;
        const std::string key = TCHAR_TO_UTF8(**Selected);
        if (!collection->contains(key) || !(*collection)[key].is_object())
            return nullptr;
        return &(*collection)[key];
    }

    void Publish(Json& document, const FString& message)
    {
        if (!Write)
        {
            Report(TEXT("Nested collection document callback is unavailable; change was not applied."));
            return;
        }
        Updating = true;
        const FString serialized = UTF8_TO_TCHAR(document.dump(2).c_str());
        Write(serialized);
        Updating = false;
        Report(message);
        RebuildNames();
    }

    void BuildBooleanChoices(FNestedFieldRow& row)
    {
        row.choices.Reset();
        row.values.Reset();
        row.omit.Reset();
        row.keep.Reset();
        row.choices.Add(MakeShared<FString>(TEXT("Auto (omitted)")));
        row.choices.Add(MakeShared<FString>(TEXT("True")));
        row.choices.Add(MakeShared<FString>(TEXT("False")));
        row.values.Add(0); row.values.Add(1); row.values.Add(0);
        row.omit.Add(true); row.omit.Add(false); row.omit.Add(false);
        row.keep.Add(false); row.keep.Add(false); row.keep.Add(false);
    }

    void BuildSensorTypeChoices(FNestedFieldRow& row, const Json* value)
    {
        row.choices.Reset(); row.values.Reset(); row.omit.Reset(); row.keep.Reset();
        int32 authored = 0;
        const bool has_authored = value != nullptr && value->is_number_integer();
        if (has_authored)
            authored = value->get<int32>();
        if (has_authored && authored > 0 && authored <= 11)
        {
            row.choices.Add(MakeShared<FString>(TEXT("Keep current")));
            row.values.Add(authored); row.omit.Add(false); row.keep.Add(true);
        }
        else if (has_authored)
        {
            row.choices.Add(MakeShared<FString>(FString::Printf(TEXT("Keep current: %d"), authored)));
            row.values.Add(authored); row.omit.Add(false); row.keep.Add(true);
        }
        else
        {
            row.choices.Add(MakeShared<FString>(TEXT("Auto (omitted)")));
            row.values.Add(0); row.omit.Add(true); row.keep.Add(false);
        }
        for (int32 index = 0; index < UE_ARRAY_COUNT(GSensorTypeValues); ++index)
        {
            row.choices.Add(MakeShared<FString>(GSensorTypeLabels[index]));
            row.values.Add(GSensorTypeValues[index]); row.omit.Add(false); row.keep.Add(false);
        }
    }

    void RebuildNames()
    {
        Json document;
        FString error;
        if (!ReadJson(document, error))
        {
            Report(TEXT("Nested collection refresh refused: ") + error);
            return;
        }
        Json* vehicle = FindVehicle(document, VehicleName());
        const FString previous = Selected.IsValid() ? *Selected : FString();
        Names.Reset();
        if (vehicle != nullptr && vehicle->contains(TCHAR_TO_UTF8(*CollectionName)))
        {
            Json& collection = (*vehicle)[TCHAR_TO_UTF8(*CollectionName)];
            if (!collection.is_object())
            {
                Names.Reset();
                Selected.Reset();
                Updating = true;
                if (List.IsValid())
                {
                    List->RequestListRefresh();
                    List->SetSelection(nullptr);
                }
                Updating = false;
                RefreshControls();
                if (IsCamera && CameraSettings.IsValid())
                    CameraSettings->RefreshFromDocument();
                Report(CollectionName + TEXT(" must be a JSON object."));
                return;
            }
            for (auto it = collection.begin(); it != collection.end(); ++it)
                Names.Add(MakeShared<FString>(UTF8_TO_TCHAR(it.key().c_str())));
        }
        Selected.Reset();
        for (const TSharedPtr<FString>& name : Names)
            if (*name == previous) { Selected = name; break; }
        if (!Selected.IsValid() && Names.Num() > 0)
            Selected = Names[0];
        Updating = true;
        if (List.IsValid()) { List->RequestListRefresh(); List->SetSelection(Selected); }
        Updating = false;
        RefreshControls();
        if (IsCamera && CameraSettings.IsValid())
            CameraSettings->RefreshFromDocument();
    }

    void RefreshControls()
    {
        Json document;
        FString error;
        if (!ReadJson(document, error))
            return;
        Updating = true;
        Json* collection = FindCollection(document, VehicleName(), CollectionName);
        Json* item = SelectedObject(document);
        for (const TSharedPtr<FNestedFieldRow>& row : Rows)
        {
            Json* value = item ? (item->contains(TCHAR_TO_UTF8(row->definition.key)) ? &(*item)[TCHAR_TO_UTF8(row->definition.key)] : nullptr) : nullptr;
            if (row->definition.type == ENestedFieldType::Boolean)
            {
                const int32 index = value && value->is_boolean() ? (value->get<bool>() ? 1 : 2) : 0;
                if (row->combo.IsValid() && index >= 0 && index < row->choices.Num())
                    row->combo->SetSelectedItem(row->choices[index]);
            }
            else if (row->definition.type == ENestedFieldType::SensorType)
            {
                BuildSensorTypeChoices(*row, value);
                if (row->combo.IsValid())
                {
                    row->combo->RefreshOptions();
                    int32 index = 0;
                    if (value && value->is_number_integer())
                        for (int32 i = 0; i < row->values.Num(); ++i)
                            if (!row->keep[i] && row->values[i] == value->get<int32>()) { index = i; break; }
                    if (index >= 0 && index < row->choices.Num())
                        row->combo->SetSelectedItem(row->choices[index]);
                }
            }
            else if (row->text.IsValid())
            {
                FString text;
                if (value && row->definition.type == ENestedFieldType::String && value->is_string())
                    text = UTF8_TO_TCHAR(value->get<std::string>().c_str());
                else if (value && row->definition.type == ENestedFieldType::Number && value->is_number())
                    text = UTF8_TO_TCHAR(value->dump().c_str());
                row->text->SetText(FText::FromString(text));
            }
        }
        if (Advanced.IsValid())
            Advanced->SetText(FText::FromString(item ? UTF8_TO_TCHAR(item->dump(2).c_str()) : TEXT("{}\n")));
        int32 capture_count = 0, noise_count = 0;
        FString sensor_type_label = TEXT("SensorType omitted");
        if (IsCamera && item != nullptr)
        {
            if (item->contains("CaptureSettings") && ((*item)["CaptureSettings"].is_object() || (*item)["CaptureSettings"].is_array()))
                capture_count = static_cast<int32>((*item)["CaptureSettings"].size());
            if (item->contains("NoiseSettings") && ((*item)["NoiseSettings"].is_object() || (*item)["NoiseSettings"].is_array()))
                noise_count = static_cast<int32>((*item)["NoiseSettings"].size());
        }
        else if (!IsCamera && item != nullptr && item->contains("SensorType") && (*item)["SensorType"].is_number_integer())
        {
            const int32 sensor_type = (*item)["SensorType"].get<int32>();
            for (int32 index = 0; index < UE_ARRAY_COUNT(GSensorTypeValues); ++index)
            {
                if (GSensorTypeValues[index] == sensor_type)
                {
                    sensor_type_label = GSensorTypeLabels[index];
                    break;
                }
            }
            if (sensor_type_label == TEXT("SensorType omitted"))
                sensor_type_label = FString::Printf(TEXT("SensorType %d (unknown)"), sensor_type);
        }
        if (InfoText.IsValid())
        {
            InfoText->SetText(FText::FromString(
                IsCamera
                    ? FString::Printf(TEXT("Camera: %s   CaptureSettings: %d   NoiseSettings: %d"), Selected.IsValid() ? **Selected : TEXT("none"), capture_count, noise_count)
                    : FString::Printf(TEXT("Sensor: %s   Type: %s"), Selected.IsValid() ? **Selected : TEXT("none"), *sensor_type_label)));
        }
        Updating = false;
        (void)collection;
    }

    void MutateField(const TSharedPtr<FNestedFieldRow>& row, int32 choice, const FString& text, bool clear)
    {
        if (Updating) return;
        Json document; FString error;
        if (!ReadJson(document, error)) { Report(TEXT("Nested edit refused: ") + error); return; }
        Json* item = SelectedObject(document);
        if (item == nullptr) { Report(TEXT("Select a nested object first.")); return; }
        const std::string key = TCHAR_TO_UTF8(row->definition.key);
        if (row->definition.type == ENestedFieldType::Boolean)
        {
            if (choice < 0 || choice >= row->choices.Num()) { Report(TEXT("Invalid boolean choice.")); return; }
            if (clear || row->omit[choice]) item->erase(key);
            else (*item)[key] = choice == 1;
        }
        else if (row->definition.type == ENestedFieldType::SensorType)
        {
            if (choice < 0 || choice >= row->choices.Num()) { Report(TEXT("Invalid SensorType choice.")); return; }
            if (row->keep[choice]) return;
            if (row->omit[choice]) item->erase(key); else (*item)[key] = row->values[choice];
        }
        else if (clear || text.IsEmpty())
            item->erase(key);
        else if (row->definition.type == ENestedFieldType::String)
            (*item)[key] = std::string(TCHAR_TO_UTF8(*text));
        else
        {
            double number = 0.0;
            if (!ParseNumber(text, number)) { Report(TEXT("Nested number must be finite.")); return; }
            (*item)[key] = number;
        }
        Publish(document, TEXT("Nested setting changed; press Validate."));
    }

    void AddObject()
    {
        FString name = AddName.IsValid() ? AddName->GetText().ToString() : FString();
        name.TrimStartAndEndInline();
        int32 type_index = AddSensorType.IsValid() ? SensorTypeChoices.IndexOfByKey(AddSensorType->GetSelectedItem()) : INDEX_NONE;
        const int32 enabled_index = AddEnabled.IsValid() ? AddEnabledChoices.IndexOfByKey(AddEnabled->GetSelectedItem()) : INDEX_NONE;
        if (name.IsEmpty() || (!IsCamera && (type_index <= 0 || type_index >= SensorTypeValues.Num() || enabled_index < 1 || enabled_index > 2)))
        { Report(IsCamera ? TEXT("Add requires a nonempty unique camera name.") : TEXT("Add requires a nonempty unique name and explicit SensorType.")); return; }
        Json document; FString error;
        if (!ReadJson(document, error)) { Report(TEXT("Nested add refused: ") + error); return; }
        Json* vehicle = FindVehicle(document, VehicleName());
        if (vehicle == nullptr) { Report(TEXT("Select a vehicle first.")); return; }
        if (!vehicle->contains(TCHAR_TO_UTF8(*CollectionName))) (*vehicle)[TCHAR_TO_UTF8(*CollectionName)] = Json::object();
        if (!(*vehicle)[TCHAR_TO_UTF8(*CollectionName)].is_object()) { Report(CollectionName + TEXT(" must be a JSON object.")); return; }
        Json& collection = (*vehicle)[TCHAR_TO_UTF8(*CollectionName)];
        const std::string key = TCHAR_TO_UTF8(*name);
        if (collection.contains(key)) { Report(TEXT("Nested name already exists.")); return; }
        Json object = Json::object();
        if (IsCamera)
        {
            // CameraSetting uses NaN to mean "use the component default".  A newly
            // authored camera has no component default yet, so make its pose explicit;
            // otherwise dynamic spawning hands Unreal a NaN transform and can crash.
            object["X"] = 0.0;
            object["Y"] = 0.0;
            object["Z"] = 0.0;
            object["Pitch"] = 0.0;
            object["Roll"] = 0.0;
            object["Yaw"] = 0.0;
        }
        else
        {
            object["SensorType"] = SensorTypeValues[type_index];
            object["Enabled"] = enabled_index == 1;
        }
        collection[key] = object; Selected = MakeShared<FString>(name);
        Publish(document, TEXT("Nested object added; press Validate."));
    }

    void DuplicateObject()
    {
        FString name = DuplicateName.IsValid() ? DuplicateName->GetText().ToString() : FString(); name.TrimStartAndEndInline();
        Json document; FString error;
        if (!ReadJson(document, error)) { Report(TEXT("Nested duplicate refused: ") + error); return; }
        Json* collection = FindCollection(document, VehicleName(), CollectionName); Json* source = SelectedObject(document);
        if (!collection || !source || name.IsEmpty() || collection->contains(TCHAR_TO_UTF8(*name))) { Report(TEXT("Duplicate requires a selected object and unique nonempty name.")); return; }
        (*collection)[TCHAR_TO_UTF8(*name)] = *source; Selected = MakeShared<FString>(name);
        Publish(document, TEXT("Nested object duplicated; press Validate."));
    }

    void RenameObject()
    {
        FString name = RenameName.IsValid() ? RenameName->GetText().ToString() : FString(); name.TrimStartAndEndInline();
        Json document; FString error;
        if (!ReadJson(document, error)) { Report(TEXT("Nested rename refused: ") + error); return; }
        Json* collection = FindCollection(document, VehicleName(), CollectionName);
        if (!collection || !Selected.IsValid() || name.IsEmpty() || collection->contains(TCHAR_TO_UTF8(*name))) { Report(TEXT("Rename requires a unique nonempty name.")); return; }
        const std::string old_key = TCHAR_TO_UTF8(**Selected); const std::string new_key = TCHAR_TO_UTF8(*name);
        if (!collection->contains(old_key)) { Report(TEXT("Select a nested object first.")); return; }
        (*collection)[new_key] = (*collection)[old_key]; collection->erase(old_key); Selected = MakeShared<FString>(name);
        Publish(document, TEXT("Nested object renamed; press Validate."));
    }

    void DeleteObject()
    {
        if (!Selected.IsValid()) { Report(TEXT("Select a nested object first.")); return; }
        if (FMessageDialog::Open(EAppMsgType::YesNo, FText::FromString(FString::Printf(TEXT("Delete %s '%s'?"), *CollectionName, **Selected))) != EAppReturnType::Yes) return;
        Json document; FString error;
        if (!ReadJson(document, error)) { Report(TEXT("Nested delete refused: ") + error); return; }
        Json* vehicle = FindVehicle(document, VehicleName()); Json* collection = FindCollection(document, VehicleName(), CollectionName);
        if (!vehicle || !collection) { Report(CollectionName + TEXT(" must be a JSON object.")); return; }
        collection->erase(TCHAR_TO_UTF8(**Selected));
        // For sensors, an explicitly empty object is meaningful: it suppresses AirSim's
        // sim-mode default IMU/GPS/etc. population. Keep the empty Sensors object after deleting
        // its last entry so the structured editor can express a true zero-sensor vehicle. Camera
        // collections retain the historical omission semantics because absent Cameras already
        // means no authored cameras.
        if (collection->empty() && IsCamera)
            vehicle->erase(TCHAR_TO_UTF8(*CollectionName));
        Selected.Reset(); Publish(document, TEXT("Nested object deleted; press Validate."));
    }

    void ApplyAdvanced()
    {
        Json replacement; FString error;
        if (!Advanced.IsValid() || !ParseDocument(Advanced->GetText().ToString(), replacement, error) || !replacement.is_object()) { Report(TEXT("Nested JSON must be an object.")); return; }
        Json document; if (!ReadJson(document, error)) { Report(TEXT("Nested JSON apply refused: ") + error); return; }
        Json* item = SelectedObject(document); if (!item) { Report(TEXT("Select a nested object first.")); return; }
        *item = replacement; Publish(document, TEXT("Nested JSON applied; press Validate."));
    }

    void Refresh() { RebuildNames(); }
};

void SStartupSensorEditor::Construct(const FArguments& args)
{
    State = MakeShared<FState>();
    State->Read = args._ReadDocument;
    State->Write = args._WriteDocument;
    State->Status = args._Status;
    State->SelectedVehicle = args._SelectedVehicle;
    State->CollectionName = args._CollectionName;
    State->IsCamera = State->CollectionName.Equals(TEXT("Cameras"), ESearchCase::IgnoreCase);
    TSharedPtr<FState> state = State;
    state->SensorTypeChoices.Add(MakeShared<FString>(TEXT("Select SensorType")));
    state->SensorTypeValues.Add(0);
    for (int32 index = 0; index < UE_ARRAY_COUNT(GSensorTypeValues); ++index)
    {
        state->SensorTypeChoices.Add(MakeShared<FString>(GSensorTypeLabels[index]));
        state->SensorTypeValues.Add(GSensorTypeValues[index]);
    }
    state->AddEnabledChoices.Add(MakeShared<FString>(TEXT("Select Enabled")));
    state->AddEnabledChoices.Add(MakeShared<FString>(TEXT("True")));
    state->AddEnabledChoices.Add(MakeShared<FString>(TEXT("False")));
    const FNestedFieldDefinition* definitions = state->IsCamera ? GCameraFields : GSensorFields;
    const int32 definition_count = state->IsCamera ? UE_ARRAY_COUNT(GCameraFields) : UE_ARRAY_COUNT(GSensorFields);
    state->Fields = SNew(SVerticalBox);
    for (int32 index = 0; index < definition_count; ++index)
    {
        TSharedPtr<FNestedFieldRow> row = MakeShared<FNestedFieldRow>(); row->definition = definitions[index];
        if (row->definition.type == ENestedFieldType::Boolean) state->BuildBooleanChoices(*row);
        state->Rows.Add(row);
        TSharedRef<SHorizontalBox> line = SNew(SHorizontalBox);
        line->AddSlot().FillWidth(0.38f).Padding(3.0f)[SNew(STextBlock).Text(FText::FromString(row->definition.label))];
        TWeakPtr<FState> weak_state = state; TWeakPtr<FNestedFieldRow> weak_row = row;
        if (row->definition.type == ENestedFieldType::Boolean || row->definition.type == ENestedFieldType::SensorType)
        {
            row->combo = SNew(SComboBox<TSharedPtr<FString>>)
                .OptionsSource(&row->choices)
                .OnGenerateWidget_Lambda([](TSharedPtr<FString> item) { return SNew(STextBlock).Text(FText::FromString(*item)); })
                .OnSelectionChanged_Lambda([weak_state, weak_row](TSharedPtr<FString> item, ESelectInfo::Type)
                {
                    TSharedPtr<FState> pinned_state = weak_state.Pin(); TSharedPtr<FNestedFieldRow> pinned_row = weak_row.Pin();
                    if (pinned_state.IsValid() && pinned_row.IsValid()) pinned_state->MutateField(pinned_row, pinned_row->choices.IndexOfByKey(item), FString(), false);
                })
                [SNew(STextBlock).Text_Lambda([weak_row]() { TSharedPtr<FNestedFieldRow> row = weak_row.Pin(); return row.IsValid() && row->combo->GetSelectedItem().IsValid() ? FText::FromString(*row->combo->GetSelectedItem()) : FText::FromString(TEXT("Auto (omitted)")); })];
            line->AddSlot().FillWidth(0.52f).Padding(3.0f)[row->combo.ToSharedRef()];
        }
        else
        {
            row->text = SNew(SEditableTextBox).HintText(FText::FromString(TEXT("Auto (omitted)"))).OnTextCommitted_Lambda([weak_state, weak_row](const FText& text, ETextCommit::Type)
            {
                TSharedPtr<FState> pinned_state = weak_state.Pin(); TSharedPtr<FNestedFieldRow> pinned_row = weak_row.Pin();
                if (pinned_state.IsValid() && pinned_row.IsValid()) pinned_state->MutateField(pinned_row, 0, text.ToString(), false);
            });
            line->AddSlot().FillWidth(0.52f).Padding(3.0f)[row->text.ToSharedRef()];
        }
        line->AddSlot().AutoWidth().Padding(3.0f)[SNew(SButton).Text(FText::FromString(TEXT("Clear"))).OnClicked_Lambda([weak_state, weak_row]() { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) if (TSharedPtr<FNestedFieldRow> pinned_row = weak_row.Pin()) pinned_state->MutateField(pinned_row, 0, FString(), true); return FReply::Handled(); })];
        state->Fields->AddSlot().AutoHeight().Padding(2.0f)[line];
    }
    TWeakPtr<FState> weak_state = state;
    TSharedRef<SVerticalBox> root = SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(STextBlock).Text(FText::FromString(state->IsCamera ? TEXT("Camera object controls; use Capture, Noise & Model for camera settings.") : TEXT("Named vehicle Sensors; type-specific settings remain in the raw object editor.")))]
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(0.28f)[SAssignNew(state->AddName, SEditableTextBox).HintText(FText::FromString(TEXT("New name")))]
            + SHorizontalBox::Slot().FillWidth(0.22f)[SAssignNew(state->AddSensorType, SComboBox<TSharedPtr<FString>>).OptionsSource(&state->SensorTypeChoices).OnGenerateWidget_Lambda([](TSharedPtr<FString> item) { return SNew(STextBlock).Text(FText::FromString(*item)); })[SNew(STextBlock).Text(FText::FromString(TEXT("Select SensorType")))]]
            + SHorizontalBox::Slot().FillWidth(0.18f)[SAssignNew(state->AddEnabled, SComboBox<TSharedPtr<FString>>).OptionsSource(&state->AddEnabledChoices).OnGenerateWidget_Lambda([](TSharedPtr<FString> item) { return SNew(STextBlock).Text(FText::FromString(*item)); })[SNew(STextBlock).Text(FText::FromString(TEXT("Select Enabled")))]]
            + SHorizontalBox::Slot().AutoWidth()[SNew(SButton).Text(FText::FromString(TEXT("Add"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) pinned_state->AddObject(); return FReply::Handled(); })]
            + SHorizontalBox::Slot().AutoWidth()[SNew(SButton).Text(FText::FromString(TEXT("Delete"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) pinned_state->DeleteObject(); return FReply::Handled(); })]]
        + SVerticalBox::Slot().FillHeight(0.18f).Padding(3.0f)[SNew(SBox).MinDesiredHeight(120.0f)[SAssignNew(state->List, SListView<TSharedPtr<FString>>).ListItemsSource(&state->Names).OnSelectionChanged_Lambda([weak_state](TSharedPtr<FString> name, ESelectInfo::Type) { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) { if (pinned_state->Updating) return; pinned_state->Selected = name; pinned_state->RefreshControls(); if (pinned_state->IsCamera && pinned_state->CameraSettings.IsValid()) pinned_state->CameraSettings->RefreshFromDocument(); } }).OnGenerateRow_Lambda([](TSharedPtr<FString> name, const TSharedRef<STableViewBase>& owner) { return SNew(STableRow<TSharedPtr<FString>>, owner)[SNew(STextBlock).Text(FText::FromString(*name))]; })]]
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(0.28f)[SAssignNew(state->DuplicateName, SEditableTextBox).HintText(FText::FromString(TEXT("Duplicate as...")))]
            + SHorizontalBox::Slot().AutoWidth()[SNew(SButton).Text(FText::FromString(TEXT("Duplicate"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) pinned_state->DuplicateObject(); return FReply::Handled(); })]
            + SHorizontalBox::Slot().FillWidth(0.28f)[SAssignNew(state->RenameName, SEditableTextBox).HintText(FText::FromString(TEXT("Rename as...")))]
            + SHorizontalBox::Slot().AutoWidth()[SNew(SButton).Text(FText::FromString(TEXT("Rename"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) pinned_state->RenameObject(); return FReply::Handled(); })]]
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SAssignNew(state->InfoText, STextBlock).Text(FText::FromString(TEXT("No nested object selected")))]
        // The parent launcher provides the single main vertical scrollbar. Do not constrain the
        // long camera/sensor form to a small nested scroll viewport.
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[state->Fields.ToSharedRef()]
        + SVerticalBox::Slot().FillHeight(0.28f).Padding(3.0f)[SNew(SVerticalBox) + SVerticalBox::Slot().AutoHeight()[SNew(STextBlock).Text(FText::FromString(TEXT("Selected object JSON")))] + SVerticalBox::Slot().FillHeight(1.0f)[SNew(SBox).MinDesiredHeight(180.0f)[SAssignNew(state->Advanced, SMultiLineEditableTextBox).AutoWrapText(false)]]]
        + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SHorizontalBox)
            + SHorizontalBox::Slot().AutoWidth()[SNew(SButton).Text(FText::FromString(TEXT("Refresh / Revert"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) pinned_state->RefreshControls(); return FReply::Handled(); })]
            + SHorizontalBox::Slot().AutoWidth()[SNew(SButton).Text(FText::FromString(TEXT("Apply object JSON"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) pinned_state->ApplyAdvanced(); return FReply::Handled(); })]];
    if (state->IsCamera)
    {
        state->CameraSettings = SAssignNew(state->CameraSettings, SStartupCameraSettingsEditor)
            .ReadDocument(state->Read)
            .WriteDocument(state->Write)
            .Status(state->Status)
            .SelectedVehicle(state->SelectedVehicle)
            .SelectedCamera([weak_state]() { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) return pinned_state->Selected.IsValid() ? *pinned_state->Selected : FString(); return FString(); });
        state->CameraSubview = SNew(SWidgetSwitcher)
            + SWidgetSwitcher::Slot()[root]
            + SWidgetSwitcher::Slot()[state->CameraSettings.ToSharedRef()];
        ChildSlot[SNew(SVerticalBox)
            + SVerticalBox::Slot().AutoHeight().Padding(3.0f)[SNew(SHorizontalBox)
                + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Camera object"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) if (pinned_state->CameraSubview.IsValid()) pinned_state->CameraSubview->SetActiveWidgetIndex(0); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)[SNew(SButton).Text(FText::FromString(TEXT("Capture, Noise & Model"))).OnClicked_Lambda([weak_state]() { if (TSharedPtr<FState> pinned_state = weak_state.Pin()) if (pinned_state->CameraSubview.IsValid()) pinned_state->CameraSubview->SetActiveWidgetIndex(1); return FReply::Handled(); })]]
            + SVerticalBox::Slot().FillHeight(1.0f)[state->CameraSubview.ToSharedRef()]];
    }
    else
        ChildSlot[root];
    if (state->AddSensorType.IsValid() && state->IsCamera)
        state->AddSensorType->SetVisibility(EVisibility::Collapsed);
    if (state->AddEnabled.IsValid() && state->IsCamera)
        state->AddEnabled->SetVisibility(EVisibility::Collapsed);
}

void SStartupSensorEditor::RefreshFromDocument()
{
    if (State.IsValid())
        State->Refresh();
}
}
