#include "StartupVehicleEditor.h"

#include "Misc/MessageDialog.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SMultiLineEditableTextBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SBox.h"
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
    enum class EFieldType { String, Integer, Number, Boolean };
    struct FFieldDefinition { const TCHAR* label; const TCHAR* key; EFieldType type; };
    struct FFieldRow {
        FFieldDefinition definition;
        TArray<TSharedPtr<FString>> choices;
        TSharedPtr<SEditableTextBox> text;
        TSharedPtr<SComboBox<TSharedPtr<FString>>> combo;
        TSharedPtr<SBox> container;
    };
    const TCHAR* GVehicleTypes[] = {
        TEXT("px4multirotor"), TEXT("arducoptersolo"), TEXT("simpleflight"), TEXT("arducopter"),
        TEXT("physxcar"), TEXT("boxcar"), TEXT("cphusky"), TEXT("pioneer"), TEXT("ardurover"),
        TEXT("computervision"), TEXT("urdfbot")
    };
    const FFieldDefinition GCoreFields[] = {
        {TEXT("Pawn path"), TEXT("PawnPath"), EFieldType::String}, {TEXT("Default vehicle state"), TEXT("DefaultVehicleState"), EFieldType::String},
        {TEXT("X"), TEXT("X"), EFieldType::Number}, {TEXT("Y"), TEXT("Y"), EFieldType::Number}, {TEXT("Z"), TEXT("Z"), EFieldType::Number},
        {TEXT("Yaw"), TEXT("Yaw"), EFieldType::Number}, {TEXT("Pitch"), TEXT("Pitch"), EFieldType::Number}, {TEXT("Roll"), TEXT("Roll"), EFieldType::Number},
        {TEXT("Allow API always"), TEXT("AllowAPIAlways"), EFieldType::Boolean}, {TEXT("Auto create"), TEXT("AutoCreate"), EFieldType::Boolean},
        {TEXT("Collision passthrough"), TEXT("EnableCollisionPassthrough"), EFieldType::Boolean}, {TEXT("Enable trace"), TEXT("EnableTrace"), EFieldType::Boolean},
        {TEXT("Enable collisions"), TEXT("EnableCollisions"), EFieldType::Boolean}, {TEXT("Is FPV vehicle"), TEXT("IsFpvVehicle"), EFieldType::Boolean}
    };
    const FFieldDefinition GRcFields[] = {
        {TEXT("RC remote control ID"), TEXT("RC.RemoteControlID"), EFieldType::Integer},
        {TEXT("RC allow API disconnected"), TEXT("RC.AllowAPIWhenDisconnected"), EFieldType::Boolean}
    };
    const FFieldDefinition GUrdfFields[] = {
        {TEXT("URDF file (required)"), TEXT("UrdfFile"), EFieldType::String}, {TEXT("Physics engine"), TEXT("PhysicsEngine"), EFieldType::String},
        {TEXT("Newton sidecar vehicle"), TEXT("NewtonSidecarVehicle"), EFieldType::String}, {TEXT("URDF fixed base"), TEXT("UrdfFixedBase"), EFieldType::Boolean}
    };
    bool Parse(const FString& text, Json& document, FString& error)
    {
        try { document = Json::parse(TCHAR_TO_UTF8(*text)); if (!document.is_object()) { error = TEXT("Settings root must be a JSON object."); return false; } return true; }
        catch (const std::exception& e) { error = UTF8_TO_TCHAR(e.what()); return false; }
    }
    Json* Find(Json& object, const FString& path)
    {
        TArray<FString> parts; path.ParseIntoArray(parts, TEXT("."), true); Json* current = &object;
        for (const FString& part : parts) { const std::string key = TCHAR_TO_UTF8(*part); if (!current->is_object() || !current->contains(key)) return nullptr; current = &(*current)[key]; }
        return current;
    }
    void Set(Json& object, const FString& path, const Json& value)
    {
        TArray<FString> parts; path.ParseIntoArray(parts, TEXT("."), true); Json* current = &object;
        for (int32 i = 0; i + 1 < parts.Num(); ++i) { const std::string key = TCHAR_TO_UTF8(*parts[i]); if (!current->is_object()) *current = Json::object(); current = &(*current)[key]; }
        if (parts.Num() > 0) (*current)[TCHAR_TO_UTF8(*parts.Last())] = value;
    }
    bool Clear(Json& object, const FString& path)
    {
        TArray<FString> parts; path.ParseIntoArray(parts, TEXT("."), true); if (parts.Num() == 0) return false;
        Json* current = &object; TArray<Json*> parents; parents.Add(current);
        for (int32 i = 0; i + 1 < parts.Num(); ++i) { const std::string key = TCHAR_TO_UTF8(*parts[i]); if (!current->is_object() || !current->contains(key)) return false; current = &(*current)[key]; parents.Add(current); }
        const std::string leaf = TCHAR_TO_UTF8(*parts.Last()); if (!current->is_object() || !current->contains(leaf)) return false; current->erase(leaf);
        for (int32 i = parts.Num() - 1; i > 0; --i) { Json* parent = parents[i - 1]; const std::string key = TCHAR_TO_UTF8(*parts[i - 1]); if (parent->is_object() && parent->contains(key) && (*parent)[key].is_object() && (*parent)[key].empty()) parent->erase(key); else break; }
        return true;
    }
}

struct SStartupVehicleEditor::FState
{
    FReadDocument Read; FWriteDocument Write; FStatus Status;
    TArray<TSharedPtr<FString>> Names; TSharedPtr<FString> Selected;
    TSharedPtr<SListView<TSharedPtr<FString>>> List; TSharedPtr<SEditableTextBox> AddName, DuplicateName, RenameName;
    TSharedPtr<SComboBox<TSharedPtr<FString>>> AddType, SelectedType; TArray<TSharedPtr<FString>> TypeChoices, SelectedTypeChoices;
    TSharedPtr<SMultiLineEditableTextBox> Advanced; TSharedPtr<SVerticalBox> Fields; TSharedPtr<STextBlock> InfoText;
    TArray<TSharedPtr<FFieldRow>> Rows; bool Updating = false;

    void Report(const FString& message) { if (Status) Status(message); }
    bool ReadJson(Json& document, FString& error) const
    {
        if (!Read)
        {
            error = TEXT("Vehicle document callback is unavailable.");
            return false;
        }
        return Parse(Read(), document, error);
    }
    Json* Vehicle(Json& document) const
    {
        if (!Selected.IsValid() || Selected->IsEmpty() || !document.contains("Vehicles") || !document["Vehicles"].is_object()) return nullptr;
        const std::string name = TCHAR_TO_UTF8(**Selected); if (!document["Vehicles"].contains(name) || !document["Vehicles"][name].is_object()) return nullptr; return &document["Vehicles"][name];
    }
    void Publish(Json& document, const FString& message)
    {
        // Keep the parent launcher from treating our programmatic document update as a
        // user edit.  The parent owns the raw text and can therefore also update its
        // validation-stale state in the write callback.
        Updating = true;
        if (!Write)
        {
            Updating = false;
            Report(TEXT("Vehicle document callback is unavailable; change was not applied."));
            return;
        }
        const FString serialized = UTF8_TO_TCHAR(document.dump(2).c_str());
        Write(serialized);
        Updating = false;

        Report(message);
        Refresh();
    }
    void MutateField(const TSharedPtr<FFieldRow>& row, int32 choice, const FString& text, bool clear)
    {
        if (Updating) return; Json document; FString error; if (!ReadJson(document, error)) { Report(TEXT("Vehicle edit refused: ") + error); return; }
        Json* vehicle = Vehicle(document); if (!vehicle) { Report(TEXT("Select a vehicle first.")); return; }
        const EFieldType type = row->definition.type; const FString& key = row->definition.key;
        if (type == EFieldType::Boolean && (choice < 0 || choice >= row->choices.Num())) { Report(TEXT("Invalid vehicle boolean choice.")); return; }
        if (clear || (type == EFieldType::Boolean && choice == 0)) Clear(*vehicle, key);
        else if (type == EFieldType::Boolean) Set(*vehicle, key, choice == 1);
        else if (type == EFieldType::String) { if (text.IsEmpty()) Clear(*vehicle, key); else Set(*vehicle, key, std::string(TCHAR_TO_UTF8(*text))); }
        else if (type == EFieldType::Integer) { const std::string raw = TCHAR_TO_UTF8(*text); char* end = nullptr; errno = 0; const long value = std::strtol(raw.c_str(), &end, 10); if (errno == ERANGE || end == raw.c_str() || *end != '\0' || value < std::numeric_limits<int32>::min() || value > std::numeric_limits<int32>::max()) { Report(TEXT("Vehicle integer must be a valid int32.")); return; } Set(*vehicle, key, static_cast<int32>(value)); }
        else { const std::string raw = TCHAR_TO_UTF8(*text); char* end = nullptr; const double value = std::strtod(raw.c_str(), &end); if (end == raw.c_str() || *end != '\0' || !std::isfinite(value)) { Report(TEXT("Vehicle number must be finite.")); return; } Set(*vehicle, key, value); }
        Publish(document, TEXT("Vehicle setting changed; press Validate."));
    }
    void MutateType(TSharedPtr<FString> choice)
    {
        if (Updating || !choice.IsValid()) return; const int32 index = SelectedTypeChoices.IndexOfByKey(choice); if (index <= 0 || index >= SelectedTypeChoices.Num()) return;
        Json document; FString error; if (!ReadJson(document, error)) { Report(TEXT("Vehicle type change refused: ") + error); return; } Json* vehicle = Vehicle(document); if (!vehicle) { Report(TEXT("Select a vehicle first.")); return; }
        (*vehicle)["VehicleType"] = std::string(TCHAR_TO_UTF8(**SelectedTypeChoices[index])); Publish(document, TEXT("Vehicle type changed; press Validate."));
    }
    void RefreshControls()
    {
        Json document; FString error; if (!ReadJson(document, error)) return; Updating = true; Json* vehicle = Vehicle(document);
        SelectedTypeChoices.Reset(); FString type; if (vehicle && vehicle->contains("VehicleType") && (*vehicle)["VehicleType"].is_string()) type = UTF8_TO_TCHAR((*vehicle)["VehicleType"].get<std::string>().c_str());
        SelectedTypeChoices.Add(MakeShared<FString>(type.IsEmpty() ? TEXT("Keep current VehicleType") : FString::Printf(TEXT("Keep current: %s"), *type))); int32 selected_type = 0;
        for (const TCHAR* known : GVehicleTypes) { SelectedTypeChoices.Add(MakeShared<FString>(known)); if (type == known) selected_type = SelectedTypeChoices.Num() - 1; }
        if (SelectedType.IsValid()) { SelectedType->RefreshOptions(); SelectedType->SetSelectedItem(SelectedTypeChoices[selected_type]); }
        for (const TSharedPtr<FFieldRow>& row : Rows) { Json* value = vehicle ? Find(*vehicle, row->definition.key) : nullptr; if (row->definition.type == EFieldType::Boolean) { const int32 index = value && value->is_boolean() ? (value->get<bool>() ? 1 : 2) : 0; if (index < row->choices.Num()) row->combo->SetSelectedItem(row->choices[index]); } else { FString text; if (value && ((row->definition.type == EFieldType::String && value->is_string()) || (row->definition.type == EFieldType::Integer && value->is_number_integer()) || (row->definition.type == EFieldType::Number && value->is_number()))) text = row->definition.type == EFieldType::String ? UTF8_TO_TCHAR(value->get<std::string>().c_str()) : UTF8_TO_TCHAR(value->dump().c_str()); row->text->SetText(FText::FromString(text)); } }
        FString lowered_type = type.ToLower();
        const bool is_urdf = lowered_type == TEXT("urdfbot");
        for (int32 i = 0; i < Rows.Num(); ++i)
            if (Rows[i]->container.IsValid())
                Rows[i]->container->SetVisibility(i >= GCoreFieldsNum + 2 && is_urdf ? EVisibility::Visible : i >= GCoreFieldsNum + 2 ? EVisibility::Collapsed : EVisibility::Visible);
        if (Advanced.IsValid()) Advanced->SetText(FText::FromString(vehicle ? UTF8_TO_TCHAR(vehicle->dump(2).c_str()) : TEXT("{}\n")));
        if (InfoText.IsValid()) { int32 cameras = 0, sensors = 0; if (vehicle && vehicle->contains("Cameras") && (*vehicle)["Cameras"].is_object()) cameras = static_cast<int32>((*vehicle)["Cameras"].size()); if (vehicle && vehicle->contains("Sensors") && (*vehicle)["Sensors"].is_object()) sensors = static_cast<int32>((*vehicle)["Sensors"].size()); InfoText->SetText(FText::FromString(Selected.IsValid() ? FString::Printf(TEXT("Selected: %s   Cameras: %d   Sensors: %d"), **Selected, cameras, sensors) : TEXT("No vehicle selected"))); }
        Updating = false;
    }
    void RebuildNames()
    {
        Json document;
        FString error;
        if (!ReadJson(document, error))
        {
            Report(TEXT("Vehicle list refresh refused: ") + error);
            return;
        }

        if (!document.contains("Vehicles"))
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
            return;
        }

        if (!document["Vehicles"].is_object())
        {
            Report(TEXT("Vehicles must be a JSON object."));
            return;
        }

        const FString previous_name = Selected.IsValid() ? *Selected : FString();
        Names.Reset();
        for (auto it = document["Vehicles"].begin(); it != document["Vehicles"].end(); ++it)
        {
            Names.Add(MakeShared<FString>(UTF8_TO_TCHAR(it.key().c_str())));
        }

        // Rebind to the new shared item rather than retaining a pointer into the
        // previous array.  This keeps the list and detail panel synchronized after
        // every add, duplicate, rename, delete, and external refresh.
        Selected.Reset();
        for (const TSharedPtr<FString>& name : Names)
        {
            if (*name == previous_name)
            {
                Selected = name;
                break;
            }
        }
        if (!Selected.IsValid() && Names.Num() > 0)
        {
            Selected = Names[0];
        }

        Updating = true;
        if (List.IsValid())
        {
            List->RequestListRefresh();
            List->SetSelection(Selected);
        }
        Updating = false;
        RefreshControls();
    }
    void AddVehicle()
    {
        FString name = AddName.IsValid() ? AddName->GetText().ToString() : FString();
        name.TrimStartAndEndInline();
        const TSharedPtr<FString> type = AddType.IsValid() ? AddType->GetSelectedItem() : nullptr;
        if (name.IsEmpty() || !type.IsValid() || type->IsEmpty())
        {
            Report(TEXT("Add requires a nonempty unique name and explicit VehicleType."));
            return;
        }

        Json document;
        FString error;
        if (!ReadJson(document, error))
        {
            Report(TEXT("Vehicle add refused: ") + error);
            return;
        }
        if (!document.contains("Vehicles"))
        {
            document["Vehicles"] = Json::object();
        }
        if (!document["Vehicles"].is_object())
        {
            Report(TEXT("Vehicles must be a JSON object."));
            return;
        }

        const std::string key = TCHAR_TO_UTF8(*name);
        if (document["Vehicles"].contains(key))
        {
            Report(TEXT("Vehicle name already exists."));
            return;
        }

        document["Vehicles"][key] = Json::object({
            {"VehicleType", std::string(TCHAR_TO_UTF8(**type))}
        });
        Selected = MakeShared<FString>(name);
        Publish(document, TEXT("Vehicle added; press Validate."));
    }

    void DuplicateVehicle()
    {
        FString name = DuplicateName.IsValid() ? DuplicateName->GetText().ToString() : FString();
        name.TrimStartAndEndInline();

        Json document;
        FString error;
        if (!ReadJson(document, error))
        {
            Report(TEXT("Vehicle duplicate refused: ") + error);
            return;
        }
        Json* source = Vehicle(document);
        if (!source || name.IsEmpty() || !document.contains("Vehicles") ||
            !document["Vehicles"].is_object() ||
            document["Vehicles"].contains(TCHAR_TO_UTF8(*name)))
        {
            Report(TEXT("Duplicate requires a selected vehicle and unique nonempty name."));
            return;
        }

        // Assignment copies the complete object, including keys unknown to this
        // editor (Cameras, Sensors, firmware, and backend-specific settings).
        document["Vehicles"][TCHAR_TO_UTF8(*name)] = *source;
        Selected = MakeShared<FString>(name);
        Publish(document, TEXT("Vehicle duplicated; press Validate."));
    }

    void RenameVehicle()
    {
        FString name = RenameName.IsValid() ? RenameName->GetText().ToString() : FString();
        name.TrimStartAndEndInline();
        Json document;
        FString error;
        if (!ReadJson(document, error))
        {
            Report(TEXT("Vehicle rename refused: ") + error);
            return;
        }

        if (!Selected.IsValid() || name.IsEmpty() || !document.contains("Vehicles") ||
            !document["Vehicles"].is_object())
        {
            Report(TEXT("Rename requires a unique nonempty name."));
            return;
        }
        const std::string old_key = TCHAR_TO_UTF8(**Selected);
        const std::string new_key = TCHAR_TO_UTF8(*name);
        if (!document["Vehicles"].contains(old_key) || document["Vehicles"].contains(new_key))
        {
            Report(TEXT("Rename requires a unique nonempty name."));
            return;
        }

        document["Vehicles"][new_key] = document["Vehicles"][old_key];
        document["Vehicles"].erase(old_key);
        Selected = MakeShared<FString>(name);
        Publish(document, TEXT("Vehicle renamed; press Validate."));
    }

    void DeleteVehicle()
    {
        if (!Selected.IsValid())
        {
            Report(TEXT("Select a vehicle first."));
            return;
        }
        const EAppReturnType::Type answer = FMessageDialog::Open(
            EAppMsgType::YesNo,
            FText::FromString(FString::Printf(TEXT("Delete vehicle '%s'?"), **Selected)));
        if (answer != EAppReturnType::Yes)
        {
            return;
        }

        Json document;
        FString error;
        if (!ReadJson(document, error))
        {
            Report(TEXT("Vehicle delete refused: ") + error);
            return;
        }
        if (!document.contains("Vehicles") || !document["Vehicles"].is_object())
        {
            Report(TEXT("Vehicles must be a JSON object."));
            return;
        }

        document["Vehicles"].erase(TCHAR_TO_UTF8(**Selected));
        if (document["Vehicles"].empty())
        {
            document.erase("Vehicles");
        }
        Selected.Reset();
        Publish(document, TEXT("Vehicle deleted; press Validate."));
    }

    void ApplyAdvanced() { Json replacement; FString error; if (!Parse(Advanced->GetText().ToString(), replacement, error) || !replacement.is_object()) { Report(TEXT("Vehicle JSON must be an object.")); return; } Json document; if (!ReadJson(document, error)) { Report(TEXT("Vehicle JSON apply refused: ") + error); return; } Json* vehicle = Vehicle(document); if (!vehicle) { Report(TEXT("Select a vehicle first.")); return; } *vehicle = replacement; Publish(document, TEXT("Vehicle JSON applied; press Validate.")); }
    void Refresh() { RebuildNames(); }
    static constexpr int32 GCoreFieldsNum = 14;
};

void SStartupVehicleEditor::Construct(const FArguments& args)
{
    ReadDocument = args._ReadDocument; WriteDocument = args._WriteDocument; Status = args._Status; State = MakeShared<FState>(); State->Read = ReadDocument; State->Write = WriteDocument; State->Status = Status;
    TSharedPtr<FState> state = State;
    for (const TCHAR* type : GVehicleTypes) state->TypeChoices.Add(MakeShared<FString>(type));
    auto add_rows = [&](const FFieldDefinition* definitions, int32 count)
    {
        for (int32 index = 0; index < count; ++index)
        {
            TSharedPtr<FFieldRow> row = MakeShared<FFieldRow>();
            row->definition = definitions[index];
            if (row->definition.type == EFieldType::Boolean)
            {
                row->choices.Add(MakeShared<FString>(TEXT("Auto (omitted)")));
                row->choices.Add(MakeShared<FString>(TEXT("True")));
                row->choices.Add(MakeShared<FString>(TEXT("False")));
            }
            state->Rows.Add(row);
        }
    };
    add_rows(GCoreFields, UE_ARRAY_COUNT(GCoreFields)); add_rows(GRcFields, UE_ARRAY_COUNT(GRcFields)); add_rows(GUrdfFields, UE_ARRAY_COUNT(GUrdfFields));
    state->Fields = SNew(SVerticalBox); FString group;
    for (const TSharedPtr<FFieldRow>& row : state->Rows) {
        const FString next_group = FCString::Strncmp(row->definition.key, TEXT("RC."), 3) == 0 ? TEXT("RC") : FCString::Strncmp(row->definition.key, TEXT("Urdf"), 4) == 0 ? TEXT("URDF (urdfbot only)") : TEXT("Core");
        if (group != next_group) { group = next_group; if (group != TEXT("URDF (urdfbot only)")) state->Fields->AddSlot().AutoHeight().Padding(3.0f, 8.0f, 3.0f, 2.0f)[SNew(STextBlock).Text(FText::FromString(group))]; }
        TSharedRef<SHorizontalBox> line = SNew(SHorizontalBox);
        line->AddSlot().FillWidth(0.38f).Padding(3.0f)
        [SNew(STextBlock).Text(FText::FromString(row->definition.label))];
        TWeakPtr<FState> weak_state = state; TWeakPtr<FFieldRow> weak_row = row;
        if (row->definition.type == EFieldType::Boolean)
        {
            row->combo = SNew(SComboBox<TSharedPtr<FString>>)
                .OptionsSource(&row->choices)
                .OnGenerateWidget_Lambda([](TSharedPtr<FString> item)
                {
                    return SNew(STextBlock).Text(FText::FromString(*item));
                })
                .OnSelectionChanged_Lambda(
                    [weak_state, weak_row](TSharedPtr<FString> item, ESelectInfo::Type)
                    {
                        TSharedPtr<FState> pinned_state = weak_state.Pin();
                        TSharedPtr<FFieldRow> pinned_row = weak_row.Pin();
                        if (pinned_state.IsValid() && pinned_row.IsValid())
                        {
                            const int32 choice = pinned_row->choices.IndexOfByKey(item);
                            pinned_state->MutateField(pinned_row, choice, FString(), false);
                        }
                    })
                [SNew(STextBlock).Text_Lambda([weak_row]()
                {
                    TSharedPtr<FFieldRow> pinned_row = weak_row.Pin();
                    if (pinned_row.IsValid() && pinned_row->combo->GetSelectedItem().IsValid())
                    {
                        return FText::FromString(*pinned_row->combo->GetSelectedItem());
                    }
                    return FText::FromString(TEXT("Auto (omitted)"));
                })];
            line->AddSlot().FillWidth(0.52f).Padding(3.0f)[row->combo.ToSharedRef()];
        }
        else
        {
            row->text = SNew(SEditableTextBox)
                .HintText(FText::FromString(TEXT("Auto (omitted)")))
                .OnTextCommitted_Lambda(
                    [weak_state, weak_row](const FText& text, ETextCommit::Type)
                    {
                        TSharedPtr<FState> pinned_state = weak_state.Pin();
                        TSharedPtr<FFieldRow> pinned_row = weak_row.Pin();
                        if (pinned_state.IsValid() && pinned_row.IsValid())
                        {
                            pinned_state->MutateField(pinned_row, 0, text.ToString(), false);
                        }
                    });
            line->AddSlot().FillWidth(0.52f).Padding(3.0f)[row->text.ToSharedRef()];
        }

        line->AddSlot().AutoWidth().Padding(3.0f)
        [SNew(SButton)
            .Text(FText::FromString(TEXT("Clear")))
            .OnClicked_Lambda([weak_state, weak_row]()
            {
                TSharedPtr<FState> pinned_state = weak_state.Pin();
                TSharedPtr<FFieldRow> pinned_row = weak_row.Pin();
                if (pinned_state.IsValid() && pinned_row.IsValid())
                {
                    pinned_state->MutateField(pinned_row, 0, FString(), true);
                }
                return FReply::Handled();
            })];

        row->container = SNew(SBox)[line];
        state->Fields->AddSlot().AutoHeight().Padding(2.0f)[row->container.ToSharedRef()];
    }
    TWeakPtr<FState> weak_state = state;
    // Keep the callbacks weak: State owns the widgets, so a strong widget callback
    // would create a State -> widget -> callback -> State ownership cycle.
    TSharedRef<SHorizontalBox> add_controls = SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(0.32f).Padding(2.0f)
        [SAssignNew(state->AddName, SEditableTextBox)
            .HintText(FText::FromString(TEXT("New vehicle name")))]
        + SHorizontalBox::Slot().FillWidth(0.32f).Padding(2.0f)
        [SAssignNew(state->AddType, SComboBox<TSharedPtr<FString>>)
            .OptionsSource(&state->TypeChoices)
            .OnGenerateWidget_Lambda([](TSharedPtr<FString> item)
            {
                return SNew(STextBlock).Text(FText::FromString(*item));
            })
            [SNew(STextBlock).Text(FText::FromString(TEXT("Select VehicleType")))]]
        + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)
        [SNew(SButton)
            .Text(FText::FromString(TEXT("Add")))
            .OnClicked_Lambda([weak_state]()
            {
                if (TSharedPtr<FState> pinned_state = weak_state.Pin())
                {
                    pinned_state->AddVehicle();
                }
                return FReply::Handled();
            })]
        + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)
        [SNew(SButton)
            .Text(FText::FromString(TEXT("Delete")))
            .OnClicked_Lambda([weak_state]()
            {
                if (TSharedPtr<FState> pinned_state = weak_state.Pin())
                {
                    pinned_state->DeleteVehicle();
                }
                return FReply::Handled();
            })];

    TSharedRef<SVerticalBox> root = SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(6.0f)[SNew(STextBlock).Text(FText::FromString(TEXT("Vehicles preserve unknown keys, Cameras, Sensors, firmware, URDF, and backend options.")))]
        + SVerticalBox::Slot().AutoHeight().Padding(4.0f)[add_controls]
        + SVerticalBox::Slot().FillHeight(0.22f).Padding(4.0f)
        [SAssignNew(state->List, SListView<TSharedPtr<FString>>)
            .ListItemsSource(&state->Names)
            .OnSelectionChanged_Lambda(
                [weak_state](TSharedPtr<FString> name, ESelectInfo::Type)
                {
                    if (TSharedPtr<FState> pinned_state = weak_state.Pin())
                    {
                        if (pinned_state->Updating)
                        {
                            return;
                        }
                        pinned_state->Selected = name;
                        pinned_state->RefreshControls();
                    }
                })
            .OnGenerateRow_Lambda(
                [](TSharedPtr<FString> name, const TSharedRef<STableViewBase>& owner)
                {
                    return SNew(STableRow<TSharedPtr<FString>>, owner)
                        [SNew(STextBlock).Text(FText::FromString(*name))];
                })]
        + SVerticalBox::Slot().AutoHeight().Padding(4.0f)
        [SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(0.32f).Padding(2.0f)
            [SAssignNew(state->DuplicateName, SEditableTextBox)
                .HintText(FText::FromString(TEXT("Duplicate as...")))]
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)
            [SNew(SButton)
                .Text(FText::FromString(TEXT("Duplicate")))
                .OnClicked_Lambda([weak_state]()
                {
                    if (TSharedPtr<FState> pinned_state = weak_state.Pin())
                    {
                        pinned_state->DuplicateVehicle();
                    }
                    return FReply::Handled();
                })]
            + SHorizontalBox::Slot().FillWidth(0.32f).Padding(2.0f)
            [SAssignNew(state->RenameName, SEditableTextBox)
                .HintText(FText::FromString(TEXT("Rename as...")))]
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)
            [SNew(SButton)
                .Text(FText::FromString(TEXT("Rename")))
                .OnClicked_Lambda([weak_state]()
                {
                    if (TSharedPtr<FState> pinned_state = weak_state.Pin())
                    {
                        pinned_state->RenameVehicle();
                    }
                    return FReply::Handled();
                })]]
        + SVerticalBox::Slot().AutoHeight().Padding(4.0f)[SAssignNew(state->InfoText, STextBlock).Text(FText::FromString(TEXT("No vehicle selected")))]
        + SVerticalBox::Slot().AutoHeight().Padding(4.0f)
        [SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(0.38f).Padding(3.0f)
            [SNew(STextBlock).Text(FText::FromString(TEXT("VehicleType (recognized replacement)")))]
            + SHorizontalBox::Slot().FillWidth(0.52f).Padding(3.0f)
            [SAssignNew(state->SelectedType, SComboBox<TSharedPtr<FString>>)
                .OptionsSource(&state->SelectedTypeChoices)
                .OnGenerateWidget_Lambda([](TSharedPtr<FString> item)
                {
                    return SNew(STextBlock).Text(FText::FromString(*item));
                })
                .OnSelectionChanged_Lambda(
                    [weak_state](TSharedPtr<FString> item, ESelectInfo::Type)
                    {
                        if (TSharedPtr<FState> pinned_state = weak_state.Pin())
                        {
                            pinned_state->MutateType(item);
                        }
                    })
                [SNew(STextBlock).Text(FText::FromString(TEXT("Select vehicle")))]]]
        + SVerticalBox::Slot().FillHeight(0.4f).Padding(4.0f)[SNew(SScrollBox) + SScrollBox::Slot()[state->Fields.ToSharedRef()]]
        + SVerticalBox::Slot().FillHeight(0.3f).Padding(4.0f)
        [SNew(SVerticalBox)
            + SVerticalBox::Slot().AutoHeight()
            [SNew(STextBlock).Text(FText::FromString(
                TEXT("Selected vehicle JSON (advanced escape hatch)")))]
            + SVerticalBox::Slot().FillHeight(1.0f)
            [SAssignNew(state->Advanced, SMultiLineEditableTextBox)
                .AutoWrapText(false)]]
        + SVerticalBox::Slot().AutoHeight().Padding(4.0f)
        [SNew(SHorizontalBox)
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)
            [SNew(SButton)
                .Text(FText::FromString(TEXT("Refresh / Revert")))
                .OnClicked_Lambda([weak_state]()
                {
                    if (TSharedPtr<FState> pinned_state = weak_state.Pin())
                    {
                        pinned_state->RefreshControls();
                    }
                    return FReply::Handled();
                })]
            + SHorizontalBox::Slot().AutoWidth().Padding(2.0f)
            [SNew(SButton)
                .Text(FText::FromString(TEXT("Apply vehicle JSON")))
                .OnClicked_Lambda([weak_state]()
                {
                    if (TSharedPtr<FState> pinned_state = weak_state.Pin())
                    {
                        pinned_state->ApplyAdvanced();
                    }
                    return FReply::Handled();
                })]];
    ChildSlot[root];
}

void SStartupVehicleEditor::RefreshFromDocument() { if (State.IsValid()) State->RebuildNames(); }
}
