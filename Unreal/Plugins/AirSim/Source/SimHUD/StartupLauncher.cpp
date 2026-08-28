#include "StartupLauncher.h"

#include "CoreMinimal.h"
#include "DesktopPlatformModule.h"
#include "HAL/IConsoleManager.h"
#include "IDesktopPlatform.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Framework/Application/SlateApplication.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SMultiLineEditableTextBox.h"
#include "Widgets/Layout/SSpacer.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SSearchBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"
#include "StartupCVarOverrides.h"
#include "StartupVehicleEditor.h"
#include "Widgets/SWindow.h"
#include "common/AirSimSettings.hpp"
#include "common/Settings.hpp"
#include "common/common_utils/json.hpp"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <functional>
#include <initializer_list>
#include <vector>

namespace airsim_startup
{
namespace
{
    enum class ECommonSettingType { String, Integer, Number, Boolean, Enum };

    struct FCommonSettingDescriptor
    {
        const TCHAR* label;
        const TCHAR* path;
        const TCHAR* group;
        ECommonSettingType type;
        std::initializer_list<const TCHAR*> options;
    };

    struct FCommonSettingRow
    {
        FCommonSettingDescriptor descriptor;
        TArray<TSharedPtr<FString>> options;
        TSharedPtr<SEditableTextBox> text;
        TSharedPtr<SComboBox<TSharedPtr<FString>>> combo;
    };

    // This is intentionally a small, authored catalog. It is UI metadata, not a second AirSim
    // schema: the live AirLib parser remains the authority and unknown keys are never touched.
    const FCommonSettingDescriptor GCommonSettings[] = {
        {TEXT("Simulation mode"), TEXT("SimMode"), TEXT("General"), ECommonSettingType::Enum,
            {TEXT("Multirotor"), TEXT("Car"), TEXT("SkidVehicle"), TEXT("ComputerVision"), TEXT("MultiAgent")}},
        {TEXT("Default Environment"), TEXT("Default Environment"), TEXT("General"), ECommonSettingType::String, {}},
        {TEXT("View mode"), TEXT("ViewMode"), TEXT("General"), ECommonSettingType::Enum,
            {TEXT("Fpv"), TEXT("GroundObserver"), TEXT("FlyWithMe"), TEXT("Manual"), TEXT("SpringArmChase"), TEXT("Backup"), TEXT("NoDisplay"), TEXT("Front")}},
        {TEXT("Physics engine"), TEXT("PhysicsEngineName"), TEXT("Simulation"), ECommonSettingType::String, {}},
        {TEXT("Local host IP"), TEXT("LocalHostIp"), TEXT("Simulation"), ECommonSettingType::String, {}},
        {TEXT("API server port"), TEXT("ApiServerPort"), TEXT("Simulation"), ECommonSettingType::Integer, {}},
        {TEXT("Record UI visible"), TEXT("RecordUIVisible"), TEXT("Simulation"), ECommonSettingType::Boolean, {}},
        {TEXT("Engine sound"), TEXT("EngineSound"), TEXT("Simulation"), ECommonSettingType::Boolean, {}},
        {TEXT("Enable RPC"), TEXT("EnableRpc"), TEXT("Simulation"), ECommonSettingType::Boolean, {}},
        {TEXT("Move world origin"), TEXT("MoveWorldOrigin"), TEXT("Simulation"), ECommonSettingType::Boolean, {}},
        {TEXT("Initial instance segmentation"), TEXT("InitialInstanceSegmentation"), TEXT("Simulation"), ECommonSettingType::Boolean, {}},
        {TEXT("Log messages visible"), TEXT("LogMessagesVisible"), TEXT("Simulation"), ECommonSettingType::Boolean, {}},
        {TEXT("Show LOS debug lines"), TEXT("ShowLosDebugLines"), TEXT("Simulation"), ECommonSettingType::Boolean, {}},
        {TEXT("Clock type"), TEXT("ClockType"), TEXT("Simulation"), ECommonSettingType::Enum,
            {TEXT("ScalableClock"), TEXT("SteppableClock")}},
        {TEXT("Clock speed"), TEXT("ClockSpeed"), TEXT("Simulation"), ECommonSettingType::Number, {}},
        {TEXT("Image timestamp at capture"), TEXT("ImageTimestampAtCapture"), TEXT("Simulation"), ECommonSettingType::Boolean, {}},
        {TEXT("LiDAR deskew"), TEXT("LidarDeskew"), TEXT("Simulation"), ECommonSettingType::Boolean, {}},
        {TEXT("Speed unit factor"), TEXT("SpeedUnitFactor"), TEXT("Simulation"), ECommonSettingType::Number, {}},
        {TEXT("Speed unit label"), TEXT("SpeedUnitLabel"), TEXT("Simulation"), ECommonSettingType::String, {}},
        {TEXT("Origin latitude"), TEXT("OriginGeopoint.Latitude"), TEXT("Environment"), ECommonSettingType::Number, {}},
        {TEXT("Origin longitude"), TEXT("OriginGeopoint.Longitude"), TEXT("Environment"), ECommonSettingType::Number, {}},
        {TEXT("Origin altitude"), TEXT("OriginGeopoint.Altitude"), TEXT("Environment"), ECommonSettingType::Number, {}},
        {TEXT("Time of day enabled"), TEXT("TimeOfDay.Enabled"), TEXT("Environment"), ECommonSettingType::Boolean, {}},
        {TEXT("Start date/time"), TEXT("TimeOfDay.StartDateTime"), TEXT("Environment"), ECommonSettingType::String, {}},
        {TEXT("Celestial clock speed"), TEXT("TimeOfDay.CelestialClockSpeed"), TEXT("Environment"), ECommonSettingType::Number, {}},
        {TEXT("Start date/time DST"), TEXT("TimeOfDay.StartDateTimeDst"), TEXT("Environment"), ECommonSettingType::Boolean, {}},
        {TEXT("Update interval (seconds)"), TEXT("TimeOfDay.UpdateIntervalSecs"), TEXT("Environment"), ECommonSettingType::Number, {}},
        {TEXT("Move sun"), TEXT("TimeOfDay.MoveSun"), TEXT("Environment"), ECommonSettingType::Boolean, {}}
    };

    struct FCVarChoiceDefinition
    {
        const TCHAR* label;
        int32 value;
        bool omit;
    };

    struct FCVarChoice
    {
        FString label;
        int32 value = 0;
        bool omit = false;
    };

    struct FCVarHint
    {
        const TCHAR* name;
        std::initializer_list<FCVarChoiceDefinition> values;
        bool boolean;
    };

    // Only declarations whose help text explicitly names the legal values are listed here. An
    // integer default of 0/1 alone is deliberately not enough to infer a boolean.
    const FCVarHint GCVarHints[] = {
        {TEXT("airsim.LidarDeskew"), {{TEXT("inherited"), -1, true}, {TEXT("false"), 0, false}, {TEXT("true"), 1, false}}, true},
        {TEXT("airsim.LogLidarSweep"), {{TEXT("inherited"), 0, true}, {TEXT("false"), 0, false}, {TEXT("true"), 1, false}}, true},
        {TEXT("airsim.BatchImageTimestamp"), {{TEXT("inherited"), -1, true}, {TEXT("readback"), 0, false}, {TEXT("capture"), 1, false}}, false},
        {TEXT("airsim.GpuReadback"), {{TEXT("inherited"), 2, true}, {TEXT("legacy"), 0, false}, {TEXT("batched"), 1, false}, {TEXT("deferred"), 2, false}}, false},
        {TEXT("airsim.EndDrawPump"), {{TEXT("inherited"), 1, true}, {TEXT("legacy"), 0, false}, {TEXT("pump"), 1, false}}, false},
        {TEXT("airsim.GpuReadbackCube"), {{TEXT("inherited"), 1, true}, {TEXT("false"), 0, false}, {TEXT("true"), 1, false}}, true},
        {TEXT("airsim.ParallelReadbackCopy"), {{TEXT("inherited"), 1, true}, {TEXT("false"), 0, false}, {TEXT("true"), 1, false}}, true},
        {TEXT("airsim.ParallelImageDecode"), {{TEXT("inherited"), 1, true}, {TEXT("false"), 0, false}, {TEXT("true"), 1, false}}, true},
        {TEXT("airsim.NewtonSidecarAutoStart"), {{TEXT("inherited"), 0, true}, {TEXT("false"), 0, false}, {TEXT("true"), 1, false}}, true},
        {TEXT("airsim.MpmRenderParticles"), {{TEXT("inherited"), 1, true}, {TEXT("false"), 0, false}, {TEXT("true"), 1, false}}, true},
        {TEXT("airsim.MpmPauseOnStall"), {{TEXT("inherited"), 1, true}, {TEXT("report only"), 0, false}, {TEXT("pause"), 1, false}}, false},
        {TEXT("airsim.MpmTwoWay"), {{TEXT("inherited"), 0, true}, {TEXT("false"), 0, false}, {TEXT("true"), 1, false}}, true},
        {TEXT("airsim.MpmResetOnSidecarRestart"), {{TEXT("inherited"), 1, true}, {TEXT("halt"), 0, false}, {TEXT("reset"), 1, false}}, false},
        {TEXT("airsim.MpmRenderMatchDensity"), {{TEXT("inherited"), 1, true}, {TEXT("false"), 0, false}, {TEXT("true"), 1, false}}, true},
        {TEXT("gs.ShowClusterBounds"), {{TEXT("inherited"), 0, true}, {TEXT("false"), 0, false}, {TEXT("true"), 1, false}}, true},
        {TEXT("gs.GeerNearCullMode"), {{TEXT("inherited"), 0, true}, {TEXT("auto"), 0, false}, {TEXT("axial"), 1, false}, {TEXT("Euclidean"), 2, false}}, false},
        {TEXT("gs.GeerPBF"), {{TEXT("inherited"), 1, true}, {TEXT("EWA"), 0, false}, {TEXT("PBF"), 1, false}}, false},
        {TEXT("gs.GeerEval"), {{TEXT("inherited"), 1, true}, {TEXT("legacy EWA"), 0, false}, {TEXT("respect asset"), 1, false}, {TEXT("force GEER"), 2, false}}, false}
    };

    const FCVarHint* FindCVarHint(const FString& name)
    {
        for (const FCVarHint& hint : GCVarHints)
            if (name == hint.name)
                return &hint;
        return nullptr;
    }

    std::vector<std::string> SplitJsonPath(const FString& path)
    {
        std::vector<std::string> parts;
        FString remaining = path;
        while (!remaining.IsEmpty()) {
            FString part;
            if (!remaining.Split(TEXT("."), &part, &remaining)) {
                part = remaining;
                remaining.Empty();
            }
            parts.push_back(TCHAR_TO_UTF8(*part));
        }
        return parts;
    }

    nlohmann::json* FindJsonPath(nlohmann::json& document, const FString& path)
    {
        nlohmann::json* current = &document;
        for (const std::string& part : SplitJsonPath(path)) {
            if (!current->is_object() || !current->contains(part))
                return nullptr;
            current = &(*current)[part];
        }
        return current;
    }

    void SetJsonPath(nlohmann::json& document, const FString& path, const nlohmann::json& value)
    {
        nlohmann::json* current = &document;
        const std::vector<std::string> parts = SplitJsonPath(path);
        for (size_t index = 0; index + 1 < parts.size(); ++index) {
            if (!current->is_object())
                *current = nlohmann::json::object();
            current = &(*current)[parts[index]];
        }
        if (parts.empty())
            return;
        if (!current->is_object())
            *current = nlohmann::json::object();
        (*current)[parts.back()] = value;
    }

    bool ClearJsonPath(nlohmann::json& document, const FString& path)
    {
        const std::vector<std::string> parts = SplitJsonPath(path);
        if (parts.empty())
            return false;
        nlohmann::json* current = &document;
        std::vector<nlohmann::json*> parents;
        parents.push_back(current);
        for (size_t index = 0; index + 1 < parts.size(); ++index) {
            if (!current->is_object() || !current->contains(parts[index]))
                return false;
            current = &(*current)[parts[index]];
            parents.push_back(current);
        }
        if (!current->is_object() || !current->contains(parts.back()))
            return false;
        current->erase(parts.back());
        // Remove only ancestors made empty by this erase. Unknown siblings remain untouched.
        for (size_t index = parts.size() - 1; index > 0; --index) {
            nlohmann::json* parent = parents[index - 1];
            const std::string& child = parts[index - 1];
            if (parent->is_object() && parent->contains(child) && (*parent)[child].is_object() && (*parent)[child].empty())
                parent->erase(child);
            else
                break;
        }
        return true;
    }

    FString ToFString(const std::string& value)
    {
        return UTF8_TO_TCHAR(value.c_str());
    }

    bool ValidateText(const FString& text, FString& error)
    {
        try {
            const std::string raw = TCHAR_TO_UTF8(*text);
            const nlohmann::json document = nlohmann::json::parse(raw);
            if (!document.is_object()) {
                error = TEXT("Settings root must be a JSON object.");
                return false;
            }
            const bool has_simmode = document.contains("SimMode") && document["SimMode"].is_string();
            msr::airlib::AirSimSettings::initializeSettings(raw);
            // The normal launcher asks the user for SimMode when it is omitted. Validation must
            // remain non-interactive, so use MultiAgent only for this preflight and explain it in
            // the feedback; the real launch performs its normal callback again.
            const std::string validation_simmode = has_simmode
                ? document["SimMode"].get<std::string>() : "MultiAgent";
            msr::airlib::AirSimSettings::singleton().load([&validation_simmode]() { return validation_simmode; });
            if (!msr::airlib::AirSimSettings::singleton().error_messages.empty()) {
                std::string messages = "AirSim parser errors:";
                for (const auto& message : msr::airlib::AirSimSettings::singleton().error_messages)
                    messages += "\n- " + message;
                error = ToFString(messages);
                return false;
            }
            const auto startup_it = document.find("StartupCVars");
            if (startup_it != document.end()) {
                if (!startup_it->is_object()) {
                    error = TEXT("StartupCVars must be a JSON object.");
                    return false;
                }
                // AirSimSettings has already populated the shared Settings singleton above. Reuse
                // the runtime validator so UI preflight and launch enforce identical registry/type
                // rules without applying any values during validation.
                airsim_startup::ValidateStartupCVars();
            }
            std::string success = "JSON and AirSim parser checks passed.";
            if (!has_simmode)
                success += " Validation used SimMode=MultiAgent because SimMode is omitted; launch will use the normal chooser.";
            for (const auto& warning : msr::airlib::AirSimSettings::singleton().warning_messages)
                success += "\nWarning: " + warning;
            error = ToFString(success);
            return true;
        }
        catch (const std::exception& exception) {
            error = ToFString(std::string("JSON error: ") + exception.what());
            return false;
        }
    }
}

bool RunStartupLauncher(std::string& settings_text, bool& cancelled)
{
    cancelled = false;
    if (!FSlateApplication::IsInitialized())
        return false;
    FString initial_text = TEXT("{}\n");
    // Seed the editor with the first conventional file, while still allowing Open to select any
    // profile. DesktopPlatform can be unavailable in packaged builds; the raw editor remains usable.
    const FString candidates[] = {
        FString(msr::airlib::Settings::getExecutableFullPath("settings.json").c_str()),
        FPaths::Combine(FPaths::LaunchDir(), TEXT("settings.json")),
        FString(msr::airlib::Settings::getUserDirectoryFullPath("settings.json").c_str())
    };
    for (const FString& candidate : candidates) {
        if (FPaths::FileExists(candidate) && FFileHelper::LoadFileToString(initial_text, *candidate))
            break;
    }

    TSharedPtr<SWindow> window;
    TSharedPtr<SMultiLineEditableTextBox> editor;
    TSharedPtr<STextBlock> feedback;
    TSharedPtr<SButton> launch_button;
    bool accepted = false;
    bool validation_passed = false;

    struct FCVarRow
    {
        FString name;
        FString type;
        FString effective;
        FString defaults;
        const FCVarHint* hint = nullptr;
        TArray<TSharedPtr<FCVarChoice>> hint_options;
    };
    TArray<TSharedPtr<FCVarRow>> all_cvars;
    TArray<TSharedPtr<FCVarRow>> filtered_cvars;
    TSharedPtr<SSearchBox> cvar_search;
    TSharedPtr<SListView<TSharedPtr<FCVarRow>>> cvar_list;
    TSharedPtr<SEditableTextBox> override_name;
    TSharedPtr<SEditableTextBox> override_value;
    using FCVarChoicePtr = TSharedPtr<FCVarChoice>;
    TArray<FCVarChoicePtr> cvar_hint_options;
    TSharedPtr<SComboBox<FCVarChoicePtr>> cvar_hint_combo;
    bool cvar_hint_inherited = false;
    bool cvar_hint_sync_guard = false;
    FString validation_error;
    bool sync_guard = false;
    TSharedPtr<SWidgetSwitcher> view_switcher;
    TSharedPtr<SStartupVehicleEditor> vehicle_editor;
    TSharedPtr<SVerticalBox> common_form;
    std::function<void()> refresh_common;
    std::function<void()> refresh_vehicles;

    window = SNew(SWindow)
        .Title(FText::FromString(TEXT("AirSim Startup Launcher")))
        .ClientSize(FVector2D(980.0f, 720.0f))
        .SupportsMaximize(true)
        .SupportsMinimize(false);

    auto set_feedback = [&]() {
        if (feedback.IsValid())
            feedback->SetText(FText::FromString(validation_error));
        if (launch_button.IsValid())
            launch_button->SetEnabled(validation_passed);
    };

    TArray<TSharedPtr<FCommonSettingRow>> common_rows;
    for (const FCommonSettingDescriptor& descriptor : GCommonSettings) {
        TSharedPtr<FCommonSettingRow> row = MakeShared<FCommonSettingRow>();
        row->descriptor = descriptor;
        row->options.Add(MakeShared<FString>(TEXT("Auto (omitted)")));
        for (const TCHAR* option : descriptor.options)
            row->options.Add(MakeShared<FString>(option));
        if (descriptor.type == ECommonSettingType::Boolean) {
            row->options.Add(MakeShared<FString>(TEXT("True")));
            row->options.Add(MakeShared<FString>(TEXT("False")));
        }
        common_rows.Add(row);
    }

    auto mutate_common = [&](const TSharedPtr<FCommonSettingRow>& row, int32 selected_index, const FString& committed_text, bool clear) {
        if (sync_guard)
            return;
        try {
            nlohmann::json document = nlohmann::json::parse(TCHAR_TO_UTF8(*editor->GetText().ToString()));
            if (!document.is_object())
                throw std::invalid_argument("settings root must be an object");
            const FCommonSettingDescriptor& descriptor = row->descriptor;
            if ((descriptor.type == ECommonSettingType::Boolean || descriptor.type == ECommonSettingType::Enum) &&
                (selected_index < 0 || selected_index >= row->options.Num()))
                throw std::invalid_argument("invalid common setting choice");
            if (clear || ((descriptor.type == ECommonSettingType::Boolean || descriptor.type == ECommonSettingType::Enum) && selected_index == 0)) {
                ClearJsonPath(document, descriptor.path);
            }
            else {
                nlohmann::json value;
                bool has_value = true;
                if (descriptor.type == ECommonSettingType::Boolean) {
                    value = selected_index == 1;
                }
                else if (descriptor.type == ECommonSettingType::Enum) {
                    const FString& selected_option = *row->options[selected_index];
                    value = std::string(TCHAR_TO_UTF8(*selected_option));
                }
                else if (descriptor.type == ECommonSettingType::String) {
                    if (committed_text.IsEmpty()) {
                        ClearJsonPath(document, descriptor.path);
                        has_value = false;
                    }
                    else {
                        value = std::string(TCHAR_TO_UTF8(*committed_text));
                    }
                }
                else if (descriptor.type == ECommonSettingType::Integer) {
                    const std::string raw = TCHAR_TO_UTF8(*committed_text);
                    char* end = nullptr; errno = 0;
                    const long parsed = std::strtol(raw.c_str(), &end, 10);
                    if (errno == ERANGE || end == raw.c_str() || *end != '\0' || parsed < std::numeric_limits<int32>::min() || parsed > std::numeric_limits<int32>::max())
                        throw std::invalid_argument("integer value expected");
                    value = static_cast<int32>(parsed);
                }
                else {
                    const std::string raw = TCHAR_TO_UTF8(*committed_text);
                    char* end = nullptr; const double parsed = std::strtod(raw.c_str(), &end);
                    if (end == raw.c_str() || *end != '\0' || !std::isfinite(parsed))
                        throw std::invalid_argument("finite number expected");
                    value = parsed;
                }
                if (has_value)
                    SetJsonPath(document, descriptor.path, value);
            }
            sync_guard = true;
            editor->SetText(FText::FromString(UTF8_TO_TCHAR(document.dump(2).c_str())));
            sync_guard = false;
            if (refresh_common)
                refresh_common();
            validation_passed = false;
            validation_error = TEXT("Common setting updated in raw JSON; press Validate.");
            set_feedback();
        }
        catch (const std::exception& exception) {
            sync_guard = false;
            validation_passed = false;
            validation_error = ToFString(std::string("Cannot update common setting: ") + exception.what());
            set_feedback();
        }
    };

    auto clear_common = [&](const TSharedPtr<FCommonSettingRow>& row) {
        mutate_common(row, 0, FString(), true);
    };

    refresh_common = [&]() {
        if (!editor.IsValid())
            return;
        try {
            nlohmann::json document = nlohmann::json::parse(TCHAR_TO_UTF8(*editor->GetText().ToString()));
            if (!document.is_object())
                return;
            sync_guard = true;
            for (const TSharedPtr<FCommonSettingRow>& row : common_rows) {
                nlohmann::json* value = FindJsonPath(document, row->descriptor.path);
                if (row->descriptor.type == ECommonSettingType::Boolean || row->descriptor.type == ECommonSettingType::Enum) {
                    int32 selected = 0;
                    if (value != nullptr) {
                        if (row->descriptor.type == ECommonSettingType::Boolean && value->is_boolean())
                            selected = value->get<bool>() ? 1 : 2;
                        else if (row->descriptor.type == ECommonSettingType::Enum && value->is_string()) {
                            const FString authored = UTF8_TO_TCHAR(value->get<std::string>().c_str());
                            for (int32 index = 1; index < row->options.Num(); ++index)
                                if (*row->options[index] == authored) { selected = index; break; }
                        }
                    }
                    if (row->combo.IsValid())
                        row->combo->SetSelectedItem(row->options[selected]);
                }
                else if (row->text.IsValid()) {
                    FString text;
                    if (value != nullptr && ((row->descriptor.type == ECommonSettingType::String && value->is_string()) ||
                        (row->descriptor.type == ECommonSettingType::Integer && value->is_number_integer()) ||
                        (row->descriptor.type == ECommonSettingType::Number && value->is_number())))
                        text = UTF8_TO_TCHAR(value->dump().c_str());
                    if (row->descriptor.type == ECommonSettingType::String && value != nullptr && value->is_string())
                        text = UTF8_TO_TCHAR(value->get<std::string>().c_str());
                    row->text->SetText(FText::FromString(text));
                }
            }
            sync_guard = false;
        }
        catch (const std::exception&) {
            sync_guard = false;
        }
    };

    auto validate = [&]() {
        validation_error.Empty();
        const bool valid = ValidateText(editor->GetText().ToString(), validation_error);
        validation_passed = valid;
        if (valid && refresh_common)
            refresh_common();
        if (valid && refresh_vehicles)
            refresh_vehicles();
        set_feedback();
        return valid;
    };

    auto open_file = [&]() {
        IDesktopPlatform* desktop = FDesktopPlatformModule::Get();
        if (desktop == nullptr) {
            validation_error = TEXT("File picker unavailable in this build; paste JSON into the editor.");
            set_feedback();
            return;
        }
        TArray<FString> files;
        if (desktop->OpenFileDialog(nullptr, TEXT("Open AirSim settings JSON"), FPaths::LaunchDir(), TEXT("settings.json"),
                                    TEXT("JSON Files (*.json)|*.json|All Files (*.*)|*.*"), EFileDialogFlags::None, files) && files.Num() > 0) {
            FString loaded;
            if (FFileHelper::LoadFileToString(loaded, *files[0])) {
                sync_guard = true;
                editor->SetText(FText::FromString(loaded));
                sync_guard = false;
                validate();
            }
        }
    };

    auto save_as = [&]() {
        IDesktopPlatform* desktop = FDesktopPlatformModule::Get();
        if (desktop == nullptr) {
            validation_error = TEXT("File picker unavailable in this build; use the editor and Launch without saving.");
            set_feedback();
            return;
        }
        TArray<FString> files;
        if (desktop->SaveFileDialog(nullptr, TEXT("Save AirSim settings JSON"), FPaths::LaunchDir(), TEXT("settings.json"),
                                    TEXT("JSON Files (*.json)|*.json|All Files (*.*)|*.*"), EFileDialogFlags::None, files) && files.Num() > 0) {
            if (!FFileHelper::SaveStringToFile(editor->GetText().ToString(), *files[0])) {
                validation_error = TEXT("Could not save the selected settings file.");
                set_feedback();
            }
            else {
                validation_error = TEXT("Saved settings file.");
                set_feedback();
            }
        }
    };

    auto add_cvars = [&](const TCHAR* prefix) {
        IConsoleManager::Get().ForEachConsoleObjectThatStartsWith(
            FConsoleObjectVisitor::CreateLambda([&](const TCHAR* name, IConsoleObject* object) {
                IConsoleVariable* variable = object ? object->AsVariable() : nullptr;
                if (!variable)
                    return;
                TSharedPtr<FCVarRow> row = MakeShared<FCVarRow>();
                row->name = name;
                row->type = variable->IsVariableInt() ? TEXT("integer") : variable->IsVariableFloat() ? TEXT("number") : variable->IsVariableString() ? TEXT("string") : TEXT("unsupported");
                row->effective = variable->GetString();
                row->defaults = variable->GetDefaultValue();
                row->hint = FindCVarHint(row->name);
                if (row->hint != nullptr) {
                    for (const FCVarChoiceDefinition& definition : row->hint->values) {
                        TSharedPtr<FCVarChoice> choice = MakeShared<FCVarChoice>();
                        choice->label = definition.label;
                        choice->value = definition.value;
                        choice->omit = definition.omit;
                        row->hint_options.Add(choice);
                    }
                }
                all_cvars.Add(row);
            }), prefix);
    };
    add_cvars(TEXT("airsim."));
    add_cvars(TEXT("gs."));
    all_cvars.Sort([](const TSharedPtr<FCVarRow>& left, const TSharedPtr<FCVarRow>& right) { return left->name < right->name; });
    filtered_cvars = all_cvars;

    auto apply_override = [&]() {
        const FString name = override_name->GetText().ToString();
        const FString raw_value = override_value->GetText().ToString();
        TSharedPtr<FCVarRow> row;
        for (const TSharedPtr<FCVarRow>& candidate : all_cvars)
            if (candidate->name == name) { row = candidate; break; }
        if (!row.IsValid() || row->type == TEXT("unsupported")) {
            validation_error = TEXT("Choose a supported AirSim/NanoGS CVar from the list.");
            validation_passed = false;
            set_feedback();
            return;
        }
        try {
            nlohmann::json document = nlohmann::json::parse(TCHAR_TO_UTF8(*editor->GetText().ToString()));
            if (!document.is_object()) throw std::invalid_argument("settings root must be an object");
            if (!document.contains("StartupCVars")) document["StartupCVars"] = nlohmann::json::object();
            if (!document["StartupCVars"].is_object()) throw std::invalid_argument("StartupCVars must be an object");
            if (cvar_hint_inherited && row->hint != nullptr) {
                document["StartupCVars"].erase(TCHAR_TO_UTF8(*name));
                if (document["StartupCVars"].empty())
                    document.erase("StartupCVars");
                cvar_hint_inherited = false;
                sync_guard = true;
                editor->SetText(FText::FromString(UTF8_TO_TCHAR(document.dump(2).c_str())));
                sync_guard = false;
                validation_passed = false;
                validation_error = TEXT("CVar override cleared from raw JSON; press Validate.");
                set_feedback();
                return;
            }
            const std::string value = TCHAR_TO_UTF8(*raw_value);
            if (row->type == TEXT("integer")) {
                char* end = nullptr; errno = 0; const long parsed = std::strtol(value.c_str(), &end, 10);
                if (errno == ERANGE || end == value.c_str() || *end != '\0' ||
                    parsed < std::numeric_limits<int32>::min() || parsed > std::numeric_limits<int32>::max())
                    throw std::invalid_argument("int32 value expected");
                document["StartupCVars"][TCHAR_TO_UTF8(*name)] = static_cast<int32>(parsed);
            }
            else if (row->type == TEXT("number")) {
                char* end = nullptr; const double parsed = std::strtod(value.c_str(), &end);
                if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed)) throw std::invalid_argument("finite number expected");
                document["StartupCVars"][TCHAR_TO_UTF8(*name)] = parsed;
            }
            else
                document["StartupCVars"][TCHAR_TO_UTF8(*name)] = value;
            sync_guard = true;
            editor->SetText(FText::FromString(UTF8_TO_TCHAR(document.dump(2).c_str())));
            sync_guard = false;
            validation_passed = false;
            validation_error = TEXT("Override added to raw JSON; press Validate.");
            set_feedback();
        }
        catch (const std::exception& exception) {
            validation_error = ToFString(std::string("Cannot add override: ") + exception.what());
            validation_passed = false;
            set_feedback();
        }
    };

    auto remove_override = [&]() {
        const FString name = override_name->GetText().ToString();
        if (name.IsEmpty()) {
            validation_error = TEXT("Select or enter a CVar name first.");
            validation_passed = false;
            set_feedback();
            return;
        }
        try {
            nlohmann::json document = nlohmann::json::parse(TCHAR_TO_UTF8(*editor->GetText().ToString()));
            if (document.contains("StartupCVars") && document["StartupCVars"].is_object())
                document["StartupCVars"].erase(TCHAR_TO_UTF8(*name));
            if (document.contains("StartupCVars") && document["StartupCVars"].is_object() && document["StartupCVars"].empty())
                document.erase("StartupCVars");
            sync_guard = true;
            editor->SetText(FText::FromString(UTF8_TO_TCHAR(document.dump(2).c_str())));
            sync_guard = false;
            validation_passed = false;
            validation_error = TEXT("Override removed from raw JSON; press Validate.");
            set_feedback();
        }
        catch (const std::exception& exception) {
            validation_error = ToFString(std::string("Cannot remove override: ") + exception.what());
            validation_passed = false;
            set_feedback();
        }
    };

    TSharedRef<SVerticalBox> vehicle_view = SNew(SVerticalBox)
        + SVerticalBox::Slot().FillHeight(1.0f).Padding(4.0f)
        [SAssignNew(vehicle_editor, SStartupVehicleEditor)
            .ReadDocument([&]() { return editor.IsValid() ? editor->GetText().ToString() : initial_text; })
            .WriteDocument([&](const FString& text) {
                sync_guard = true;
                editor->SetText(FText::FromString(text));
                sync_guard = false;
                validation_passed = false;
                validation_error = TEXT("Vehicle changed raw JSON; press Validate.");
                set_feedback();
            })
            .Status([&](const FString& message) {
                validation_error = message;
                validation_passed = false;
                set_feedback();
            })];
    refresh_vehicles = [&]() {
        if (vehicle_editor.IsValid())
            vehicle_editor->RefreshFromDocument();
    };

    // Build the common form before the switcher so each row has a stable widget and can be
    // refreshed from the one raw document after Validate/Open JSON.
    common_form = SNew(SVerticalBox);
    FString current_group;
    for (const TSharedPtr<FCommonSettingRow>& row : common_rows) {
        if (!current_group.Equals(row->descriptor.group)) {
            current_group = row->descriptor.group;
            common_form->AddSlot().AutoHeight().Padding(4.0f, 10.0f, 4.0f, 3.0f)
                [SNew(STextBlock).Text(FText::FromString(current_group))];
        }
        TSharedRef<SHorizontalBox> controls = SNew(SHorizontalBox);
        controls->AddSlot().FillWidth(0.34f).Padding(3.0f)
            [SNew(STextBlock).Text(FText::FromString(row->descriptor.label))];
        if (row->descriptor.type == ECommonSettingType::Boolean || row->descriptor.type == ECommonSettingType::Enum) {
            row->combo = SNew(SComboBox<TSharedPtr<FString>>)
                .OptionsSource(&row->options)
                .OnGenerateWidget_Lambda([](TSharedPtr<FString> option) { return SNew(STextBlock).Text(FText::FromString(*option)); })
                .OnSelectionChanged_Lambda([&, row](TSharedPtr<FString> option, ESelectInfo::Type) {
                    int32 selected = row->options.IndexOfByKey(option);
                    mutate_common(row, selected, FString(), false);
                })
                [SNew(STextBlock).Text_Lambda([row]() {
                    return row->combo.IsValid() && row->combo->GetSelectedItem().IsValid()
                        ? FText::FromString(*row->combo->GetSelectedItem()) : FText::FromString(TEXT("Auto (omitted)"));
                })];
            controls->AddSlot().FillWidth(0.55f).Padding(3.0f)[row->combo.ToSharedRef()];
        }
        else {
            row->text = SNew(SEditableTextBox)
                .HintText(FText::FromString(TEXT("Auto (omitted)")))
                .OnTextCommitted_Lambda([&, row](const FText& text, ETextCommit::Type) {
                    mutate_common(row, 0, text.ToString(), false);
                });
            controls->AddSlot().FillWidth(0.55f).Padding(3.0f)[row->text.ToSharedRef()];
        }
        controls->AddSlot().AutoWidth().Padding(3.0f)
            [SNew(SButton).Text(FText::FromString(TEXT("Clear"))).OnClicked_Lambda([&, row]() { clear_common(row); return FReply::Handled(); })];
        common_form->AddSlot().AutoHeight().Padding(2.0f)[controls];
    }

    TSharedRef<SVerticalBox> raw_view = SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(12.0f)
        [SNew(STextBlock).Text(FText::FromString(TEXT("Complete settings document. Unknown and dynamic keys are preserved.")))]
        + SVerticalBox::Slot().FillHeight(1.0f).Padding(12.0f)
        [SAssignNew(editor, SMultiLineEditableTextBox).Text(FText::FromString(initial_text)).AutoWrapText(false)
            .OnTextChanged_Lambda([&](const FText&) {
                if (!sync_guard) {
                    validation_passed = false;
                    validation_error = TEXT("Text changed; press Validate again.");
                    set_feedback();
                }
            })];

    TSharedRef<SVerticalBox> cvar_view = SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(12.0f)
        [SNew(STextBlock).Text(FText::FromString(TEXT("Project CVars (airsim.* and gs.*): effective / default. Hints appear only where source declarations name legal values.")))]
        + SVerticalBox::Slot().AutoHeight().Padding(12.0f)
        [SAssignNew(cvar_search, SSearchBox).HintText(FText::FromString(TEXT("Search project CVars"))).OnTextChanged_Lambda([&](const FText& query) {
            filtered_cvars.Reset();
            const FString needle = query.ToString();
            for (const TSharedPtr<FCVarRow>& row : all_cvars)
                if (needle.IsEmpty() || row->name.Contains(needle, ESearchCase::IgnoreCase, ESearchDir::FromStart)) filtered_cvars.Add(row);
            if (cvar_list.IsValid()) cvar_list->RequestListRefresh();
        })]
        + SVerticalBox::Slot().FillHeight(1.0f).Padding(12.0f)
        [SAssignNew(cvar_list, SListView<TSharedPtr<FCVarRow>>)
            .ListItemsSource(&filtered_cvars)
            .OnSelectionChanged_Lambda([&](TSharedPtr<FCVarRow> row, ESelectInfo::Type) {
                if (!row.IsValid()) return;
                override_name->SetText(FText::FromString(row->name));
                FString value = row->effective;
                cvar_hint_options = row->hint_options;
                bool authored = false;
                bool authored_integer = false;
                int32 authored_value = 0;
                try {
                    const nlohmann::json document = nlohmann::json::parse(TCHAR_TO_UTF8(*editor->GetText().ToString()));
                    if (document.contains("StartupCVars") && document["StartupCVars"].is_object()) {
                        const auto it = document["StartupCVars"].find(TCHAR_TO_UTF8(*row->name));
                        if (it != document["StartupCVars"].end()) {
                            authored = true;
                            value = UTF8_TO_TCHAR(it->is_string() ? it->get<std::string>().c_str() : it->dump().c_str());
                            if (it->is_number_integer()) {
                                authored_integer = true;
                                authored_value = it->get<int32>();
                            }
                        }
                    }
                }
                catch (const std::exception&) {}
                cvar_hint_inherited = false;
                cvar_hint_sync_guard = true;
                FCVarChoicePtr selected_hint = nullptr;
                if (cvar_hint_combo.IsValid()) {
                    if (row->hint != nullptr) {
                        // An unauthored override inherits by omission. If raw JSON contains a
                        // value outside the curated choices, leave the hint unselected and keep
                        // the exact typed value visible instead of presenting a destructive
                        // "inherited" action for data the form does not understand.
                        if (!authored)
                            selected_hint = cvar_hint_options.Num() > 0 ? cvar_hint_options[0] : nullptr;
                        else if (authored_integer) {
                            for (const FCVarChoicePtr& candidate : cvar_hint_options) {
                                if (!candidate->omit && candidate->value == authored_value) {
                                    selected_hint = candidate;
                                    break;
                                }
                            }
                        }
                    }
                    cvar_hint_combo->SetSelectedItem(selected_hint);
                    cvar_hint_combo->RefreshOptions();
                }
                override_value->SetText(FText::FromString(value));
                cvar_hint_sync_guard = false;
                cvar_hint_inherited = selected_hint.IsValid() && selected_hint->omit;
            })
            .OnGenerateRow_Lambda([](TSharedPtr<FCVarRow> row, const TSharedRef<STableViewBase>& owner) {
                return SNew(STableRow<TSharedPtr<FCVarRow>>, owner)
                    [SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("%s  [%s]  effective=%s  default=%s%s"), *row->name, *row->type, *row->effective, *row->defaults, row->hint != nullptr ? TEXT("  [hinted]") : TEXT(""))))];
            })]
        + SVerticalBox::Slot().AutoHeight().Padding(12.0f)
        [SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(0.28f).Padding(3.0f)
                [SAssignNew(override_name, SEditableTextBox).HintText(FText::FromString(TEXT("CVar name (from list)")))]
            + SHorizontalBox::Slot().FillWidth(0.28f).Padding(3.0f)
                [SAssignNew(override_value, SEditableTextBox).HintText(FText::FromString(TEXT("Override value")))
                    .OnTextChanged_Lambda([&](const FText&) { if (!cvar_hint_sync_guard) cvar_hint_inherited = false; })]
            + SHorizontalBox::Slot().FillWidth(0.24f).Padding(3.0f)
                [SAssignNew(cvar_hint_combo, SComboBox<FCVarChoicePtr>)
                    .OptionsSource(&cvar_hint_options)
                    .OnGenerateWidget_Lambda([](FCVarChoicePtr option) { return SNew(STextBlock).Text(FText::FromString(option->label)); })
                    .OnSelectionChanged_Lambda([&](FCVarChoicePtr option, ESelectInfo::Type) {
                        if (cvar_hint_sync_guard)
                            return;
                        cvar_hint_inherited = option.IsValid() && option->omit;
                        if (option.IsValid() && !cvar_hint_inherited) {
                            override_value->SetText(FText::FromString(FString::FromInt(option->value)));
                        }
                    })
                    [SNew(STextBlock).Text_Lambda([&]() {
                        return cvar_hint_combo.IsValid() && cvar_hint_combo->GetSelectedItem().IsValid()
                            ? FText::FromString(cvar_hint_combo->GetSelectedItem()->label)
                            : FText::FromString(TEXT("No semantic hint"));
                    })]]
            + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                [SNew(SButton).Text(FText::FromString(TEXT("Add override"))).OnClicked_Lambda([&]() { apply_override(); return FReply::Handled(); })]
            + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                [SNew(SButton).Text(FText::FromString(TEXT("Remove override"))).OnClicked_Lambda([&]() { remove_override(); return FReply::Handled(); })]];

    window->SetContent(
        SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(8.0f)
            [SNew(SHorizontalBox)
                + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                    [SNew(SButton).Text(FText::FromString(TEXT("Common Settings"))).OnClicked_Lambda([&]() { view_switcher->SetActiveWidgetIndex(0); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                    [SNew(SButton).Text(FText::FromString(TEXT("Vehicles"))).OnClicked_Lambda([&]() { view_switcher->SetActiveWidgetIndex(1); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                    [SNew(SButton).Text(FText::FromString(TEXT("Project CVars"))).OnClicked_Lambda([&]() { view_switcher->SetActiveWidgetIndex(2); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                    [SNew(SButton).Text(FText::FromString(TEXT("Raw JSON"))).OnClicked_Lambda([&]() { view_switcher->SetActiveWidgetIndex(3); return FReply::Handled(); })]]
        + SVerticalBox::Slot().FillHeight(1.0f).Padding(4.0f)
            [SAssignNew(view_switcher, SWidgetSwitcher)
                + SWidgetSwitcher::Slot()[SNew(SScrollBox) + SScrollBox::Slot()[common_form.ToSharedRef()]]
                + SWidgetSwitcher::Slot()[vehicle_view]
                + SWidgetSwitcher::Slot()[cvar_view]
                + SWidgetSwitcher::Slot()[raw_view]]
        + SVerticalBox::Slot().AutoHeight().Padding(8.0f)
            [SAssignNew(feedback, STextBlock).Text(FText::FromString(TEXT("Press Validate before Launch.")))]
        + SVerticalBox::Slot().AutoHeight().Padding(8.0f)
            [SNew(SHorizontalBox)
                + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                    [SNew(SButton).Text(FText::FromString(TEXT("Open JSON"))).OnClicked_Lambda([&]() { open_file(); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                    [SNew(SButton).Text(FText::FromString(TEXT("Validate"))).OnClicked_Lambda([&]() { validate(); return FReply::Handled(); })]
                + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                    [SNew(SButton).Text(FText::FromString(TEXT("Save As"))).OnClicked_Lambda([&]() { save_as(); return FReply::Handled(); })]
                + SHorizontalBox::Slot().FillWidth(1.0f)[SNew(SSpacer)]
                + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                    [SAssignNew(launch_button, SButton).Text(FText::FromString(TEXT("Launch"))).IsEnabled(false).OnClicked_Lambda([&]() {
                        if (validate()) {
                            settings_text = TCHAR_TO_UTF8(*editor->GetText().ToString());
                            accepted = true;
                            FSlateApplication::Get().RequestDestroyWindow(window.ToSharedRef());
                        }
                        return FReply::Handled();
                    })]
                + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
                    [SNew(SButton).Text(FText::FromString(TEXT("Cancel"))).OnClicked_Lambda([&]() {
                        cancelled = true;
                        FSlateApplication::Get().RequestDestroyWindow(window.ToSharedRef());
                        return FReply::Handled();
                    })]]);

    refresh_common();
    refresh_vehicles();

    window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([&](const TSharedRef<SWindow>&) {
        // Alt-F4/window-manager close is cancellation, never an implicit default-settings launch.
        if (!accepted)
            cancelled = true;
    }));
    FSlateApplication::Get().AddModalWindow(window.ToSharedRef(), nullptr);
    return accepted;
}
}
