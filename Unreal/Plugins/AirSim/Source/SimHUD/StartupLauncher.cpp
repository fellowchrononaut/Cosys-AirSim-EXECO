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
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SSearchBox.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"
#include "StartupCVarOverrides.h"
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

namespace airsim_startup
{
namespace
{
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
    };
    TArray<TSharedPtr<FCVarRow>> all_cvars;
    TArray<TSharedPtr<FCVarRow>> filtered_cvars;
    TSharedPtr<SSearchBox> cvar_search;
    TSharedPtr<SListView<TSharedPtr<FCVarRow>>> cvar_list;
    TSharedPtr<SEditableTextBox> override_name;
    TSharedPtr<SEditableTextBox> override_value;
    FString validation_error;

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

    auto validate = [&]() {
        validation_error.Empty();
        const bool valid = ValidateText(editor->GetText().ToString(), validation_error);
        validation_passed = valid;
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
                editor->SetText(FText::FromString(loaded));
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
            editor->SetText(FText::FromString(UTF8_TO_TCHAR(document.dump(2).c_str())));
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
            editor->SetText(FText::FromString(UTF8_TO_TCHAR(document.dump(2).c_str())));
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

    window->SetContent(
        SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(12.0f)
        [SNew(STextBlock).Text(FText::FromString(TEXT("Edit raw JSON. Unknown/documentation keys are preserved; StartupCVars accepts exact registered CVars.")))]
        + SVerticalBox::Slot().FillHeight(1.0f).Padding(12.0f)
         [SAssignNew(editor, SMultiLineEditableTextBox).Text(FText::FromString(initial_text)).AutoWrapText(false)
         .OnTextChanged_Lambda([&](const FText&) {
             validation_passed = false;
             validation_error = TEXT("Text changed; press Validate again.");
             set_feedback();
         })]
        + SVerticalBox::Slot().AutoHeight().Padding(12.0f)
        [SNew(STextBlock).Text(FText::FromString(TEXT("Project CVars (airsim.* and gs.*): effective / default. Search, then enter a name and value to add a StartupCVars override.")))]
        + SVerticalBox::Slot().AutoHeight().Padding(12.0f)
        [SAssignNew(cvar_search, SSearchBox).HintText(FText::FromString(TEXT("Search project CVars"))).OnTextChanged_Lambda([&](const FText& query) {
            filtered_cvars.Reset();
            const FString needle = query.ToString();
            for (const TSharedPtr<FCVarRow>& row : all_cvars)
                if (needle.IsEmpty() || row->name.Contains(needle, ESearchCase::IgnoreCase, ESearchDir::FromStart)) filtered_cvars.Add(row);
            if (cvar_list.IsValid()) cvar_list->RequestListRefresh();
        })]
        + SVerticalBox::Slot().FillHeight(0.35f).Padding(12.0f)
        [SAssignNew(cvar_list, SListView<TSharedPtr<FCVarRow>>)
         .ListItemsSource(&filtered_cvars)
         .OnSelectionChanged_Lambda([&](TSharedPtr<FCVarRow> row, ESelectInfo::Type) {
             if (row.IsValid()) {
                 override_name->SetText(FText::FromString(row->name));
                 FString value = row->effective;
                 // Keep an authored value visible when editing an existing profile; effective
                 // falls back to the registered value for new overrides.
                 try {
                     const nlohmann::json document = nlohmann::json::parse(TCHAR_TO_UTF8(*editor->GetText().ToString()));
                     if (document.contains("StartupCVars") && document["StartupCVars"].is_object()) {
                         const auto it = document["StartupCVars"].find(TCHAR_TO_UTF8(*row->name));
                         if (it != document["StartupCVars"].end())
                             value = UTF8_TO_TCHAR(it->is_string() ? it->get<std::string>().c_str() : it->dump().c_str());
                     }
                 }
                 catch (const std::exception&) {
                     // Validation feedback remains responsible for malformed raw JSON.
                 }
                 override_value->SetText(FText::FromString(value));
             }
         })
         .OnGenerateRow_Lambda([](TSharedPtr<FCVarRow> row, const TSharedRef<STableViewBase>& owner) {
             return SNew(STableRow<TSharedPtr<FCVarRow>>, owner)
                 [SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("%s  [%s]  effective=%s  default=%s"), *row->name, *row->type, *row->effective, *row->defaults)))];
         })]
        + SVerticalBox::Slot().AutoHeight().Padding(12.0f)
        [SNew(SHorizontalBox)
         + SHorizontalBox::Slot().FillWidth(0.5f).Padding(3.0f)
           [SAssignNew(override_name, SEditableTextBox).HintText(FText::FromString(TEXT("CVar name (from list)")))]
         + SHorizontalBox::Slot().FillWidth(0.5f).Padding(3.0f)
           [SAssignNew(override_value, SEditableTextBox).HintText(FText::FromString(TEXT("Override value")))]
         + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
           [SNew(SButton).Text(FText::FromString(TEXT("Add override"))).OnClicked_Lambda([&]() { apply_override(); return FReply::Handled(); })]
         + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
           [SNew(SButton).Text(FText::FromString(TEXT("Remove override"))).OnClicked_Lambda([&]() { remove_override(); return FReply::Handled(); })]]
        + SVerticalBox::Slot().AutoHeight().Padding(12.0f)
        [SAssignNew(feedback, STextBlock).Text(FText::FromString(TEXT("Press Validate before Launch.")))]
        + SVerticalBox::Slot().AutoHeight().Padding(12.0f)
        [SNew(SHorizontalBox)
         + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
           [SNew(SButton).Text(FText::FromString(TEXT("Open JSON"))).OnClicked_Lambda([&]() { open_file(); return FReply::Handled(); })]
         + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
           [SNew(SButton).Text(FText::FromString(TEXT("Validate"))).OnClicked_Lambda([&]() { validate(); return FReply::Handled(); })]
         + SHorizontalBox::Slot().AutoWidth().Padding(3.0f)
           [SNew(SButton).Text(FText::FromString(TEXT("Save As"))).OnClicked_Lambda([&]() { save_as(); return FReply::Handled(); })]
         + SHorizontalBox::Slot().FillWidth(1.0f)
           [SNew(SSpacer)]
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

    window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([&](const TSharedRef<SWindow>&) {
        // Alt-F4/window-manager close is cancellation, never an implicit default-settings launch.
        if (!accepted)
            cancelled = true;
    }));
    FSlateApplication::Get().AddModalWindow(window.ToSharedRef(), nullptr);
    return accepted;
}
}
