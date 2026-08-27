#include "NewtonSidecarPanel.h"

#if WITH_EDITOR

#include "Framework/Docking/TabManager.h"
#include "Widgets/Docking/SDockTab.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"
#include "Misc/MessageDialog.h"
#include "Misc/Paths.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformProcess.h"
#include "Interfaces/IPluginManager.h"
#include "NewtonSidecarLaunch.h"
#include "WorkspaceMenuStructure.h"
#include "WorkspaceMenuStructureModule.h"

#define LOCTEXT_NAMESPACE "NewtonSidecarPanel"

const FName FNewtonSidecarPanel::TabId(TEXT("NewtonSidecarStatus"));

namespace
{
/// Where the blocks live. ⚠ Hard-coded to match the sidecar's default and the plugin's own
/// `Options::directory`; when the panel grows controls this must come from the same settings the
/// backend uses, or the panel will report on a different sidecar than the one that is running.
const TCHAR* kSegmentDirectory = TEXT("/dev/shm");

/// ⚠ SHELLED OUT TO tools/sidecar_ctl.sh RATHER THAN REIMPLEMENTED. That script already knows
/// things this panel would have to learn the hard way: that `pgrep -f` matches the launching shell
/// and the checking command itself and so reported the sidecar ALIVE for an hour while none ran;
/// that the segments must be deleted BEFORE the new process starts or the renderer reads a stale
/// particle block once and then clears it; and that `nohup env ... &` returns before the process
/// exists, so a liveness check must wait for it to appear before watching for it to vanish. A
/// second implementation of all that, in C++, would drift from the first one silently.
///
/// ⚠ AND THE PATH COMES FROM ResolveNewtonSidecarControlScript(), shared with the runtime's
/// auto-start. This file used to derive it from the plugin directory while SimModeWorldBase
/// derived it from the project directory — two copies of "four levels up" that stay correct until
/// somebody moves a folder, and then fail in only one of the two places.
} // namespace

void FNewtonSidecarPanel::Register()
{
    // ⚠ IDEMPOTENT ON PURPOSE. Two delegates race to call this and either may win; registering the
    // same nomad tab twice is an ensure, and an ensure raised by a diagnostic panel is noise in the
    // log the panel exists to make readable.
    static bool bRegistered = false;
    if (bRegistered)
        return;
    bRegistered = true;

    FGlobalTabmanager::Get()
        ->RegisterNomadTabSpawner(
            TabId,
            FOnSpawnTab::CreateLambda([](const FSpawnTabArgs&) -> TSharedRef<SDockTab> {
                return SNew(SDockTab)
                    .TabRole(ETabRole::NomadTab)
                    [SNew(SNewtonSidecarPanel)];
            }))
        .SetDisplayName(LOCTEXT("TabTitle", "Newton Sidecar"))
        .SetTooltipText(LOCTEXT("TabTooltip",
                                "What the Newton MPM sidecar's shared-memory wire is reporting."))
        // ⚠ THE LEVEL EDITOR CATEGORY, NOT DeveloperToolsMisc. On UE 5.6 the developer-tools
        // categories render under the TOOLS menu, not Window, so a tab filed there is registered,
        // reachable by the menu search, and invisible everywhere an operator would think to look.
        // Reported 2026-08-27 as simply "can't find it".
        .SetGroup(WorkspaceMenu::GetMenuStructure().GetLevelEditorCategory());

    // ⚠ SAY THAT IT REGISTERED. "The tab is missing" has two completely different causes — never
    // registered, or registered somewhere the menu does not show — and without this line the log
    // cannot tell them apart.
    UE_LOG(LogTemp, Log,
           TEXT("[AirSim] Newton Sidecar status panel registered: Window > Level Editor > "
                "Newton Sidecar (tab id %s)"),
           *TabId.ToString());
}

void FNewtonSidecarPanel::Unregister()
{
    FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(TabId);
}

void SNewtonSidecarPanel::Construct(const FArguments&)
{
    // ⚠ TWICE A SECOND. Fast enough that a stalled wire is obvious within a moment, slow enough
    // that the four map/read/unmap round trips per sample are free next to a frame.
    RegisterActiveTimer(0.5f, FWidgetActiveTimerDelegate::CreateSP(this, &SNewtonSidecarPanel::Sample));

    TSharedRef<SVerticalBox> Lines = SNew(SVerticalBox);
    for (int32 i = 0; i < 12; ++i) {
        Lines->AddSlot().AutoHeight().Padding(6.0f, 2.0f)
            [SNew(STextBlock)
                 .Text(this, &SNewtonSidecarPanel::StatusLine, i)
                 .Font(FCoreStyle::GetDefaultFontStyle("Mono", 9))];
    }

    TSharedRef<SHorizontalBox> Buttons = SNew(SHorizontalBox);
    for (const TCHAR* Verb : {TEXT("Start"), TEXT("Stop"), TEXT("Restart")}) {
        const FString VerbString(Verb);
        Buttons->AddSlot().AutoWidth().Padding(4.0f, 4.0f)
            [SNew(SButton)
                 .Text(FText::FromString(VerbString))
                 .OnClicked(this, &SNewtonSidecarPanel::OnControl, VerbString)];
    }
    Buttons->AddSlot().AutoWidth().Padding(12.0f, 6.0f)
        [SNew(SCheckBox)
             .ToolTipText(LOCTEXT("AutoStartTip",
                                  "Start the sidecar automatically when the loaded settings select "
                                  "PhysicsEngine \"NewtonSidecar\". Off by default: there are runs "
                                  "where the editor is deliberately up with no sidecar behind it. "
                                  "It launches the same script this panel's Start button does, and "
                                  "that script refuses if one is already running."))
             .IsChecked_Lambda([this]() {
                 IConsoleVariable* V = CVar(TEXT("airsim.NewtonSidecarAutoStart"));
                 return (V && V->GetInt() != 0) ? ECheckBoxState::Checked
                                                : ECheckBoxState::Unchecked;
             })
             .OnCheckStateChanged_Lambda([this](ECheckBoxState State) {
                 if (IConsoleVariable* V = CVar(TEXT("airsim.NewtonSidecarAutoStart")))
                     V->Set(State == ECheckBoxState::Checked ? 1 : 0);
             })
             [SNew(STextBlock).Text(LOCTEXT("AutoStart", "Auto-start with settings"))]];

    Buttons->AddSlot().FillWidth(1.0f).Padding(8.0f, 8.0f)
        [SNew(STextBlock)
             .Text_Lambda([this]() { return FText::FromString(LastActionMessage); })
             .Font(FCoreStyle::GetDefaultFontStyle("Mono", 8))];

    // ---- the editable profile ---------------------------------------------------------------
    LoadProfile();

    TSharedRef<SVerticalBox> Settings = SNew(SVerticalBox);
    Settings->AddSlot().AutoHeight().Padding(6.0f, 8.0f, 6.0f, 2.0f)
        [SNew(STextBlock)
             .Text(LOCTEXT("SettingsHeading",
                           "sand and solver settings   -   EVERY ONE OF THESE NEEDS A SIDECAR "
                           "RESTART, and the sand is rebuilt when it happens"))
             .AutoWrapText(true)];

    for (const FParam& P : Params()) {
        const FString Key = P.Key;
        const bool bInteger = P.bInteger;
        Settings->AddSlot().AutoHeight().Padding(6.0f, 1.0f)
            [SNew(SHorizontalBox)
             + SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
                   [SNew(STextBlock)
                        .Text(FText::FromString(P.Label))
                        .ToolTipText(FText::FromString(P.Tooltip))
                        .Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))]
             + SHorizontalBox::Slot().FillWidth(0.45f)
                   [SNew(SNumericEntryBox<double>)
                        .AllowSpin(false)
                        .ToolTipText(FText::FromString(P.Tooltip))
                        .Value_Lambda([this, Key]() -> TOptional<double> {
                            const double* V = ProfileValues.Find(Key);
                            return V ? TOptional<double>(*V) : TOptional<double>();
                        })
                        .OnValueCommitted_Lambda([this, Key, bInteger](double NewValue, ETextCommit::Type) {
                            // ⚠ HELD IN MEMORY UNTIL Save. Writing the file on every keystroke
                            // would rewrite the operator's profile while they were still deciding,
                            // and a half-typed "0.0" is a valid number.
                            ProfileValues.Add(Key, bInteger ? FMath::RoundToDouble(NewValue) : NewValue);
                            ProfileMessage = TEXT("edited - press Save, then Restart to apply");
                            bProfileDirty = true;
                        })]];
    }

    TSharedRef<SHorizontalBox> ProfileButtons = SNew(SHorizontalBox);
    ProfileButtons->AddSlot().AutoWidth().Padding(6.0f, 6.0f)
        [SNew(SButton)
             .Text(LOCTEXT("Save", "Save"))
             .ToolTipText(LOCTEXT("SaveTip",
                                  "Write these values to settings/d15/sidecar_profile.json. The "
                                  "running sidecar is NOT affected until it restarts."))
             .OnClicked_Lambda([this]() { SaveProfile(); return FReply::Handled(); })];
    ProfileButtons->AddSlot().AutoWidth().Padding(2.0f, 6.0f)
        [SNew(SButton)
             .Text(LOCTEXT("Reload", "Reload"))
             .ToolTipText(LOCTEXT("ReloadTip", "Discard edits and re-read the file from disk."))
             .OnClicked_Lambda([this]() { LoadProfile(); return FReply::Handled(); })];
    ProfileButtons->AddSlot().FillWidth(1.0f).Padding(8.0f, 8.0f)
        [SNew(STextBlock)
             .Text_Lambda([this]() { return FText::FromString(ProfileMessage); })
             .Font(FCoreStyle::GetDefaultFontStyle("Mono", 8))
             .AutoWrapText(true)];
    Settings->AddSlot().AutoHeight()[ProfileButtons];

    // ---- render settings: console variables, applied on the next frame ----------------------
    TSharedRef<SVerticalBox> Render = SNew(SVerticalBox);
    Render->AddSlot().AutoHeight().Padding(6.0f, 10.0f, 6.0f, 2.0f)
        [SNew(STextBlock)
             .Text(LOCTEXT("RenderHeading",
                           "sand appearance   -   NO RESTART NEEDED, these take effect on the next "
                           "frame and change nothing the solver does"))
             .AutoWrapText(true)];

    for (const FRenderParam& R : RenderParams()) {
        // ⚠ RESOLVED ONCE, HERE, AND CAPTURED BY POINTER. The editors below run their getters on
        // EVERY REPAINT; looking the variable up by name from inside them turned into hundreds of
        // lookups a second and Unreal said so — "Console object named 'airsim.MpmRenderScale'
        // shows many (500) FindConsoleObject() calls". A console variable's address is stable for
        // the life of the process once registered, and these are registered by the AirSim module
        // at load, long before this panel exists.
        IConsoleVariable* const V = CVar(R.CVar);
        TSharedPtr<SWidget> Editor;

        // ⚠ ONLY WHAT WAS TYPED OR DELIBERATELY LEFT. SNumericEntryBox also commits on focus loss,
        // and an empty box parses as ZERO — so clicking into "Max drawn instances" and clicking
        // away would set it to 0 and the sand would vanish, indistinguishable from the renderer
        // being broken. That is not hypothetical: the sand did vanish on 2026-08-27 and this was
        // the first thing suspected.
        auto Typed = [](ETextCommit::Type Type) {
            return Type == ETextCommit::OnEnter || Type == ETextCommit::OnUserMovedFocus;
        };

        switch (R.Kind) {
        case FRenderParam::EKind::Bool:
            Editor = SNew(SCheckBox)
                         .IsChecked_Lambda([V]() {
                             return (V && V->GetInt() != 0) ? ECheckBoxState::Checked
                                                            : ECheckBoxState::Unchecked;
                         })
                         .OnCheckStateChanged_Lambda([V](ECheckBoxState State) {
                             if (V) V->Set(State == ECheckBoxState::Checked ? 1 : 0);
                         });
            break;
        case FRenderParam::EKind::Int:
            Editor = SNew(SNumericEntryBox<int32>)
                         .AllowSpin(false)
                         .Value_Lambda([V]() -> TOptional<int32> {
                             return V ? TOptional<int32>(V->GetInt()) : TOptional<int32>();
                         })
                         .OnValueCommitted_Lambda([V, Typed](int32 NewValue, ETextCommit::Type Type) {
                             if (V && Typed(Type)) V->Set(NewValue);
                         });
            break;
        case FRenderParam::EKind::Text:
            Editor = SNew(SEditableTextBox)
                         .Text_Lambda([V]() {
                             return V ? FText::FromString(V->GetString()) : FText::GetEmpty();
                         })
                         .OnTextCommitted_Lambda([V, Typed](const FText& NewText,
                                                            ETextCommit::Type Type) {
                             if (V && Typed(Type) && !NewText.IsEmptyOrWhitespace())
                                 V->Set(*NewText.ToString());
                         });
            break;
        default:
            Editor = SNew(SNumericEntryBox<float>)
                         .AllowSpin(false)
                         .Value_Lambda([V]() -> TOptional<float> {
                             return V ? TOptional<float>(V->GetFloat()) : TOptional<float>();
                         })
                         .OnValueCommitted_Lambda([V, Typed](float NewValue, ETextCommit::Type Type) {
                             if (V && Typed(Type)) V->Set(NewValue);
                         });
            break;
        }

        Render->AddSlot().AutoHeight().Padding(6.0f, 1.0f)
            [SNew(SHorizontalBox)
             + SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
                   [SNew(STextBlock)
                        .Text(FText::FromString(R.Label))
                        .ToolTipText(FText::FromString(R.Tooltip))
                        .Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))]
             + SHorizontalBox::Slot().FillWidth(0.45f).VAlign(VAlign_Center)
                   [Editor.ToSharedRef()]];
    }

    Render->AddSlot().AutoHeight().Padding(6.0f, 6.0f)
        [SNew(SButton)
             .Text(LOCTEXT("ResetAppearance", "Reset appearance to defaults"))
             .ToolTipText(LOCTEXT("ResetAppearanceTip",
                                  "Put every setting in this section back to the value its console "
                                  "variable was declared with. Nothing here affects the solver, so "
                                  "this is always safe."))
             .OnClicked_Lambda([this]() {
                 // ⚠ RECOVERABLE FROM INSIDE THE PANEL. A zeroed size or instance cap makes the
                 // sand disappear, which looks exactly like the renderer being broken — and
                 // hunting the right console variable to undo it is not something a panel should
                 // make necessary.
                 for (const FRenderParam& R : RenderParams())
                     if (IConsoleVariable* V = CVar(R.CVar))
                         V->Set(*V->GetDefaultValue());
                 return FReply::Handled();
             })];

    ChildSlot
        [SNew(SBorder).Padding(4.0f)
             [SNew(SVerticalBox)
              + SVerticalBox::Slot().AutoHeight()[Buttons]
              + SVerticalBox::Slot().FillHeight(1.0f)
                    [SNew(SScrollBox)
                     + SScrollBox::Slot()[Lines]
                     + SScrollBox::Slot()[Settings]
                     + SScrollBox::Slot()[Render]]]];
}

const TArray<SNewtonSidecarPanel::FRenderParam>& SNewtonSidecarPanel::RenderParams()
{
    using EKind = FRenderParam::EKind;
    // ⚠ THE CONSOLE VARIABLES THAT ALREADY EXISTED, surfaced rather than duplicated. Each is the
    // authority for its own setting; the panel reads and writes the same variable the console
    // does, so `airsim.MpmRenderScale 2` typed into the console and the box below can never
    // disagree.
    static const TArray<FRenderParam> Table = {
        {TEXT("airsim.MpmRenderParticles"), TEXT("Draw the sand"),
         TEXT("Draw the sidecar's grains in the level at all. Appearance only - the solver keeps "
              "running either way."), EKind::Bool},
        {TEXT("airsim.MpmRenderScale"), TEXT("Grain size multiplier"),
         TEXT("Multiplier on the drawn particle size. The sidecar reports the REAL radius; this "
              "only makes the sand easier to see and changes nothing the solver does."),
         EKind::Float},
        {TEXT("airsim.MpmRenderMatchDensity"), TEXT("Match bed density"),
         TEXT("Compensate the drawn size for decimation, so a sampled view occupies the same "
              "volume as the solver's full particle set. Off draws grains at their true radius, "
              "which on a few percent of the particles reads as scattered grit rather than a bed."),
         EKind::Bool},
        {TEXT("airsim.MpmRenderMaxInstances"), TEXT("Max drawn instances"),
         TEXT("Hard cap on instances drawn. Rebuilding instance transforms is game-thread work, so "
              "this is what keeps a large patch from costing more than the simulation does.\n"
              "NOTE: how many grains are PUBLISHED is decided by the sidecar; this caps how many "
              "of those are drawn."), EKind::Int},
        {TEXT("airsim.MpmRenderRoughness"), TEXT("Material roughness"),
         TEXT("Dry sand is near 1; lower it for a wet, packed look."), EKind::Float},
        {TEXT("airsim.MpmRenderColor"), TEXT("Colour (linear R,G,B)"),
         TEXT("Linear RGB of the sand, comma separated, e.g. 0.72,0.56,0.33."), EKind::Text},
    };
    return Table;
}

const TArray<SNewtonSidecarPanel::FParam>& SNewtonSidecarPanel::Params()
{
    // ⚠ KEYED BY ARGPARSE DESTINATION, because that is what the sidecar's --profile loader accepts
    // and it REFUSES a key it does not recognise. A panel that invented its own names would write
    // a file the sidecar rejects outright, which is a better failure than a silently ignored one
    // but still a failure this table exists to avoid.
    static const TArray<FParam> Table = {
        {TEXT("voxel_size"), TEXT("Voxel size (m)"),
         TEXT("MPM grid resolution. Decides how many particles the bed has: 0.065 gives 141k on "
              "this patch, 0.05 gives 296k. Smaller is finer AND much slower."), false},
        {TEXT("density"), TEXT("Sand density (kg/m3)"),
         TEXT("Mass per cubic metre of bed."), false},
        {TEXT("sand_friction"), TEXT("Sand internal friction"),
         TEXT("Sets the angle of repose, atan(mu). 0.5 is about 27 degrees."), false},
        {TEXT("ground_friction"), TEXT("Ground friction"),
         TEXT("Friction of the fallback ground plane, used only when the level is not mirrored."),
         false},
        {TEXT("ground_z"), TEXT("Ground height (m, solver frame)"),
         TEXT("Where the floor is in the SIDECAR's frame. On Blocks the AirSim NED origin sits at "
              "solver z = 0.640, so solver_z = 0.640 - ned_z."), false},
        {TEXT("patch_x"), TEXT("Patch centre X (m)"), TEXT("Centre of the sand bed."), false},
        {TEXT("patch_y"), TEXT("Patch centre Y (m)"), TEXT("Centre of the sand bed."), false},
        {TEXT("patch_z"), TEXT("Patch centre Z (m)"),
         TEXT("Centre of the bed. The bed rests ON the floor, so this is normally "
              "ground_z + patch_depth/2."), false},
        {TEXT("patch_size"), TEXT("Patch half-size X (m)"), TEXT("Half-extent along X."), false},
        {TEXT("patch_size_y"), TEXT("Patch half-size Y (m)"), TEXT("Half-extent along Y."), false},
        {TEXT("patch_depth"), TEXT("Patch depth (m)"), TEXT("Full depth of the bed."), false},
        {TEXT("fps"), TEXT("Solver rate (Hz)"),
         TEXT("How much SIMULATED time each solve buys. A real physics change, not a speed knob: "
              "at 50 Hz a solve is 20 ms of sim, so the collider travels twice as far per solve as "
              "at 100. Once that approaches a voxel the sand stops seeing the motion in between."),
         false},
        {TEXT("own_vehicle_proxy_iterations"), TEXT("Coupling iterations"),
         TEXT("Fixed-point iterations between the rigid and MPM solvers. 2 is the probe reference; "
              "1 halves the per-solve cost. Force numbers measured at 1 are NOT comparable with "
              "newton_probes/results/README.md, which is all at 2."), true},
        {TEXT("own_vehicle_substeps"), TEXT("MuJoCo substeps"),
         TEXT("Rigid solver substeps per MPM solve."), true},
    };
    return Table;
}

IConsoleVariable* SNewtonSidecarPanel::CVar(const FString& Name)
{
    if (IConsoleVariable** Found = CVarCache.Find(Name))
        return *Found;
    IConsoleVariable* V = IConsoleManager::Get().FindConsoleVariable(*Name);
    // ⚠ Cached even when null. A name that does not resolve will not resolve later either — the
    // variables are registered by the AirSim module at load, well before this panel exists — and
    // caching the miss stops a typo costing a lookup per repaint forever.
    CVarCache.Add(Name, V);
    return V;
}

FString SNewtonSidecarPanel::ProfilePath()
{
    const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("AirSim"));
    if (!Plugin.IsValid())
        return FString();
    return FPaths::ConvertRelativePathToFull(FPaths::Combine(
        Plugin->GetBaseDir(), TEXT("../../../../settings/d15/sidecar_profile.json")));
}

void SNewtonSidecarPanel::LoadProfile()
{
    ProfileValues.Empty();
    bProfileDirty = false;

    FString Text;
    const FString Path = ProfilePath();
    if (Path.IsEmpty() || !FFileHelper::LoadFileToString(Text, *Path)) {
        ProfileMessage = FString::Printf(TEXT("cannot read %s"), *Path);
        return;
    }
    TSharedPtr<FJsonObject> Root;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Text);
    if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid()) {
        ProfileMessage = FString::Printf(TEXT("%s is not valid JSON"), *Path);
        return;
    }
    for (const FParam& P : Params()) {
        double Value = 0.0;
        if (Root->TryGetNumberField(P.Key, Value))
            ProfileValues.Add(P.Key, Value);
    }
    ProfileMessage = FString::Printf(TEXT("loaded %s"), *FPaths::GetCleanFilename(Path));
}

bool SNewtonSidecarPanel::SaveProfile()
{
    const FString Path = ProfilePath();
    // ⚠ READ-MODIFY-WRITE, never write-from-scratch. The file may carry keys this panel does not
    // show — a probe pinning something, a setting added later — and rewriting it from the table
    // above would delete them without saying so.
    TSharedPtr<FJsonObject> Root = MakeShared<FJsonObject>();
    FString Existing;
    if (FFileHelper::LoadFileToString(Existing, *Path)) {
        const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Existing);
        TSharedPtr<FJsonObject> Parsed;
        if (FJsonSerializer::Deserialize(Reader, Parsed) && Parsed.IsValid())
            Root = Parsed;
    }
    for (const TPair<FString, double>& Pair : ProfileValues)
        Root->SetNumberField(Pair.Key, Pair.Value);

    FString Out;
    const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
        TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&Out);
    FJsonSerializer::Serialize(Root.ToSharedRef(), Writer);
    if (!FFileHelper::SaveStringToFile(Out, *Path)) {
        ProfileMessage = FString::Printf(TEXT("could not write %s"), *Path);
        return false;
    }
    // ⚠ SAVED IS NOT APPLIED, and the panel keeps saying so until a restart. The running sidecar
    // built its model from the file as it was when it started; between Save and Restart the numbers
    // on screen and the numbers in the solver disagree, and that gap is exactly where "I changed it
    // and nothing happened" comes from.
    ProfileMessage = TEXT("saved - NOT APPLIED until the sidecar restarts (press Restart)");
    bProfileDirty = true;
    return true;
}

FString SNewtonSidecarPanel::ControlScriptPath()
{
    return ResolveNewtonSidecarControlScript();
}

FReply SNewtonSidecarPanel::OnControl(FString Verb)
{
    // ⚠ CONFIRMED, BECAUSE EVERY ONE OF THESE DISCARDS THE BED. Stopping or restarting the sidecar
    // tears down its Newton model, and the sand's deformation IS its state — a bed that has been
    // driven through for twenty minutes cannot be recovered. Typing the command in a terminal is
    // deliberate enough on its own; a button two pixels from a status readout is not.
    const FText Question = FText::FromString(FString::Printf(
        TEXT("%s the Newton sidecar?\n\nThe sand is rebuilt from scratch and the robot returns to "
             "its spawn. Anything driven so far is discarded."),
        *Verb));
    if (FMessageDialog::Open(EAppMsgType::YesNo, Question) != EAppReturnType::Yes)
        return FReply::Handled();

    // ⚠ THROUGH THE SHARED LAUNCHER, and it claims ownership: a sidecar started from this button
    // is stopped when the editor exits, and is handed the editor's pid so it exits by itself if
    // the editor crashes instead. A sidecar started from a terminal is left alone by both.
    if (!RunNewtonSidecarControl(Verb, /*bTakeOwnership=*/true, LastActionMessage))
        return FReply::Handled();

    // ⚠ Start and Restart read the profile from disk, so the file and the solver agree again;
    // Stop leaves the edit pending, because nothing has read it yet.
    if (!Verb.Equals(TEXT("Stop"), ESearchCase::IgnoreCase)) {
        bProfileDirty = false;
        ActionRequestedSeconds = FPlatformTime::Seconds();
        LastStartupSeconds = -1.0;
    }
    else {
        ActionRequestedSeconds = -1.0;
    }
    return FReply::Handled();
}

double SNewtonSidecarPanel::Rate(uint64 Now, uint64 Then, double Seconds)
{
    // ⚠ A NEGATIVE DELTA IS A RESTART, NOT A RATE. Every counter here restarts at 0 when the
    // sidecar rebuilds its model; reporting -3800/s would look like a catastrophe and is just a
    // new model. Same rule as tools/sidecar_monitor.py, deliberately.
    if (Seconds <= 0.0 || Now < Then)
        return -1.0;
    return static_cast<double>(Now - Then) / Seconds;
}

EActiveTimerReturnType SNewtonSidecarPanel::Sample(double CurrentTime, float)
{
    Previous = Status;
    const bool bHad = Status.anyBlockPresent();
    Status = msr::airlib::mpm::readSidecarStatus(TCHAR_TO_UTF8(kSegmentDirectory));

    if (bHavePrevious) {
        const double Seconds = CurrentTime - PreviousSeconds;
        ParticleRate = Rate(Status.particles.step, Previous.particles.step, Seconds);
        PoseRate = Rate(Status.poses.step, Previous.poses.step, Seconds);
        CommandRate = Rate(Status.commands.step, Previous.commands.step, Seconds);
        // ⚠ SIMULATED TIME AGAINST WALL TIME. This is the number that says whether the sand is
        // keeping up with the operator; 1.00 means the world advances as fast as the clock.
        SimTimeRatio = (Seconds > 0.0 && Status.poses.time >= Previous.poses.time)
                           ? (Status.poses.time - Previous.poses.time) / Seconds
                           : -1.0;
    }
    // ⚠ A LIVE WIRE, NOT A PRESENT ONE, ENDS THE WAIT. A block exists the moment the sidecar
    // creates it, well before it has anything to put in it; declaring success then would report
    // "ready" over a bed that has not been built.
    if (ActionRequestedSeconds >= 0.0 && (PoseRate > 0.0 || ParticleRate > 0.0)) {
        LastStartupSeconds = CurrentTime - ActionRequestedSeconds;
        ActionRequestedSeconds = -1.0;
    }
    // ⚠ AND A WAIT THAT NEVER ENDS IS ITS OWN ANSWER. The panel cannot see a failed launch, so
    // after longer than any build takes it stops implying one is in progress.
    else if (ActionRequestedSeconds >= 0.0 && CurrentTime - ActionRequestedSeconds > 180.0) {
        LastActionMessage =
            TEXT("no live wire 180 s after the request - check logs/d15_step3_sidecar.log; the "
                 "sidecar may have failed to start");
        ActionRequestedSeconds = -1.0;
    }

    PreviousSeconds = CurrentTime;
    bHavePrevious = bHad;
    return EActiveTimerReturnType::Continue;
}

FString SNewtonSidecarPanel::StartupPhase() const
{
    if (ActionRequestedSeconds < 0.0) {
        if (LastStartupSeconds > 0.0)
            return FString::Printf(TEXT("ready - the wire went live %.1f s after the request"),
                                   LastStartupSeconds);
        return FString();
    }

    const double Waited = FPlatformTime::Seconds() - ActionRequestedSeconds;

    // ⚠ THREE PHASES, ALL INFERRED FROM THE WIRE, and the panel says which evidence it is using.
    // It cannot see the process: "python is still importing warp", "the model is building" and
    // "the launch failed" are indistinguishable to it until something appears in /dev/shm.
    if (!Status.particles.present && !Status.poses.present)
        return FString::Printf(
            TEXT("STARTING - %.0f s. No blocks in /dev/shm yet: the script deletes them before it "
                 "starts, so this covers python launching, warp initialising and the model "
                 "building. The panel cannot tell those apart - it reads the wire, not the "
                 "process."),
            Waited);

    if (PoseRate <= 0.0 && ParticleRate <= 0.0)
        return FString::Printf(
            TEXT("BUILDING - %.0f s. The blocks exist but no counter has advanced yet. A first "
                 "build of a session also compiles the mesh-contact CUDA kernels; later ones hit "
                 "warp's cache and are much faster."),
            Waited);

    return FString::Printf(TEXT("running - %.0f s since the request"), Waited);
}

FText SNewtonSidecarPanel::StatusLine(int32 Which) const
{
    using namespace msr::airlib::mpm;

    auto RateText = [](double R) {
        return R < 0.0 ? FString(TEXT("   ...")) : FString::Printf(TEXT("%6.1f"), R);
    };

    switch (Which) {
    case 0: {
        const FString Phase = StartupPhase();
        if (!Phase.IsEmpty())
            return FText::FromString(Phase);
        if (bProfileDirty)
            return LOCTEXT("Dirty",
                           "SETTINGS EDITED - the running sidecar is still using the values it "
                           "started with. Press Restart to apply them.");
        return FText::FromString(FString::Printf(
            TEXT("wire directory %s      protocol v%u"), kSegmentDirectory, Status.expected_version));
    }

    case 1:
        if (!Status.anyBlockPresent())
            return LOCTEXT("NoBlocks",
                           "NO SEGMENTS AT ALL - no sidecar has run against this directory. "
                           "Start one with tools/sidecar_ctl.sh start");
        return FText::GetEmpty();

    case 2:
        // ⚠ THE FIRST THING WORTH SAYING, because a version mismatch makes every number below it
        // meaningless and looks exactly like a physics bug from the viewport.
        if (Status.particles.version_mismatch || Status.poses.version_mismatch ||
            Status.commands.version_mismatch || Status.level.version_mismatch)
            return LOCTEXT("Mismatch",
                           "PROTOCOL MISMATCH - a block was written by a different build. Stop "
                           "both ends, delete the segments, and restart them together.");
        return FText::GetEmpty();

    case 3:
        return LOCTEXT("WireHeading", "wire");

    case 4:
        if (!Status.particles.present)
            return LOCTEXT("NoSand", "  sand      no particle block");
        return FText::FromString(FString::Printf(
            TEXT("  sand      %s fps   %u of %llu particles, r=%.4f m%s"),
            *RateText(ParticleRate), Status.particles.published,
            static_cast<unsigned long long>(Status.particles.total), Status.particles.radius,
            (ParticleRate == 0.0 ? TEXT("   NOT PUBLISHING - the renderer will clear the bed")
                                 : TEXT(""))));

    case 5:
        if (!Status.poses.present)
            return LOCTEXT("NoPoses", "  poses     no pose block");
        return FText::FromString(FString::Printf(
            TEXT("  poses     %s Hz    '%s'  %u links, %u joints%s"), *RateText(PoseRate),
            *FString(Status.poses.vehicle_name.c_str()), Status.poses.link_count,
            Status.poses.joint_count,
            (PoseRate == 0.0 ? TEXT("   NOT PUBLISHING - the robot will freeze") : TEXT(""))));

    case 6:
        if (!Status.poses.present)
            return FText::GetEmpty();
        return FText::FromString(FString::Printf(
            TEXT("            root (%+7.3f %+7.3f %+7.3f)   sim t=%8.2f s"), Status.poses.root[0],
            Status.poses.root[1], Status.poses.root[2], Status.poses.time));

    case 7:
        if (!Status.poses.present || SimTimeRatio < 0.0)
            return FText::GetEmpty();
        return FText::FromString(FString::Printf(
            TEXT("            %.2fx real time   (1.00 = the world advances as fast as your clock)"),
            SimTimeRatio));

    case 8:
        if (!Status.commands.present)
            return LOCTEXT("NoCommands",
                           "  commands  no command block - the simulator has not started");
        return FText::FromString(FString::Printf(
            TEXT("  commands  %s Hz    %llu step(s) unacknowledged%s"), *RateText(CommandRate),
            static_cast<unsigned long long>(Status.commands.step >
                                                    Status.poses.acknowledged_command_step
                                                ? Status.commands.step -
                                                      Status.poses.acknowledged_command_step
                                                : 0),
            (CommandRate == 0.0 ? TEXT("   SIM NOT COMMANDING - is PIE running?") : TEXT(""))));

    case 9:
        if (!Status.commands.present)
            return FText::GetEmpty();
        // ⚠ THE SESSION ID IS SHOWN, not hidden as an implementation detail. It is the only thing
        // on this wire that distinguishes one PIE session from the next: every counter restarts
        // from the same base each Play, which is why stopping and replaying without pressing
        // BackSpace used to leave the sidecar running the previous session's bed.
        return FText::FromString(FString::Printf(
            TEXT("            session %016llx   reset epoch %llu   %u moving actor pose(s)"),
            static_cast<unsigned long long>(Status.commands.session_id),
            static_cast<unsigned long long>(Status.commands.reset_epoch),
            Status.commands.kinematic_count));

    case 10:
        if (!Status.level.present)
            return LOCTEXT("NoLevel",
                           "  level     not mirrored - the sidecar is on its flat fallback ground");
        return FText::FromString(FString::Printf(
            TEXT("  level     rev %u   %u static bodies, %u moving, %u triangles%s"),
            Status.level.revision, Status.level.body_count, Status.level.kinematic_count,
            Status.level.triangle_count,
            (Status.level.truncated ? TEXT("   TRUNCATED - the level has holes") : TEXT(""))));

    case 11:
        // ⚠ THE DISAGREEMENT THAT LOOKS LIKE A PHYSICS BUG. When these two revisions differ the
        // sidecar REFUSES every mirrored-actor pose, so each one is frozen at its build pose while
        // both ends report healthy.
        if (Status.kinematic_revision_matches())
            return FText::GetEmpty();
        return FText::FromString(FString::Printf(
            TEXT("  REGISTRATION MISMATCH: the simulator is sending revision %u, the sidecar built "
                 "%u. Every mirrored actor is frozen at its build pose."),
            Status.commands.kinematic_revision, Status.level.revision));

    default:
        return FText::GetEmpty();
    }
}

#undef LOCTEXT_NAMESPACE

#endif // WITH_EDITOR
