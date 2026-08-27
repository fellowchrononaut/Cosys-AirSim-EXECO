// A read-only editor panel showing what the Newton MPM sidecar's wire actually says.
//
// ⚠ WHY READ-ONLY FIRST. Every control this panel could grow — restart, change the voxel size,
// force a rebuild — costs a model rebuild or a process restart, and both are disruptive enough
// that a panel which does them by accident is worse than no panel. A status readout cannot break a
// run, and it immediately replaces the terminal for the question that was asked twenty times on
// 2026-08-27: "is the sidecar alive, and is the wire moving".
//
// ⚠ PROCESS AND WIRE ARE DIFFERENT QUESTIONS AND ARE NEVER CONFLATED HERE. `pgrep` reported the
// sidecar ALIVE for over an hour while none was running, and a sidecar blocked in an 11 s rebuild
// is alive at 0 % CPU with a frozen particle block. This panel answers the stronger question — are
// the counters ADVANCING — by comparing consecutive samples, and says so in those terms.
#pragma once

#include "CoreMinimal.h"

#if WITH_EDITOR

#include "Widgets/SCompoundWidget.h"
#include "Input/Reply.h"
#include "HAL/IConsoleManager.h"

#include "mpm/MpmSidecarStatus.hpp"

/// Registers and unregisters the panel's tab. Called from the module's startup/shutdown.
class FNewtonSidecarPanel
{
public:
    static void Register();
    static void Unregister();

    /// The tab id, also used by the menu entry.
    static const FName TabId;
};

/// The widget itself: samples the wire on a timer and renders one line per block.
class SNewtonSidecarPanel : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SNewtonSidecarPanel) {}
    SLATE_END_ARGS()

    void Construct(const FArguments& InArgs);

private:
    /// ⚠ SAMPLED ON A TIMER, NOT EVERY TICK. Each sample opens, maps, reads and unmaps four
    /// segments; doing that at frame rate would put four syscall round trips per block on the game
    /// thread to render text a human reads twice a second.
    EActiveTimerReturnType Sample(double CurrentTime, float DeltaTime);

    /// Per-second rate of a monotonic counter, or -1 when it cannot be computed yet.
    static double Rate(uint64 Now, uint64 Then, double Seconds);

    msr::airlib::mpm::SidecarStatus Status;
    msr::airlib::mpm::SidecarStatus Previous;
    bool bHavePrevious = false;
    double PreviousSeconds = 0.0;
    double ParticleRate = -1.0;
    double PoseRate = -1.0;
    double CommandRate = -1.0;
    double SimTimeRatio = -1.0;

    FText StatusLine(int32 Which) const;

    /// ⚠ THE BUTTONS DO NOT REPORT SUCCESS — THE WIRE DOES. Stopping takes up to 15 s (SIGTERM,
    /// then SIGKILL) and starting takes an 11 s model rebuild, so a handler that waited for either
    /// would freeze the editor for the length of the thing it triggered. These launch the control
    /// script detached and return immediately; whether it worked shows up in the counters above,
    /// which is the only evidence that was ever worth trusting anyway.
    FReply OnControl(FString Verb);
    static FString ControlScriptPath();

    /// One editable sidecar parameter, as it appears in settings/d15/sidecar_profile.json.
    ///
    /// ⚠ EVERY ONE OF THESE COSTS A RESTART, and the panel says so rather than pretending
    /// otherwise. The Newton model is immutable once finalised — voxel size and patch geometry
    /// decide how many particles exist, and the coupling iterations and fps are baked into the
    /// solver at construction — so none of them can be changed under a running solve. A control
    /// that silently did nothing until the next restart would be worse than no control.
    struct FParam {
        FString Key;          ///< argparse destination, e.g. "voxel_size"
        FString Label;
        FString Tooltip;
        bool bInteger = false;
    };
    static const TArray<FParam>& Params();

    /// One render setting, which is a CONSOLE VARIABLE and not a sidecar parameter.
    ///
    /// ⚠ THESE NEED NO RESTART, AND THE DISTINCTION IS THE WHOLE REASON THEY ARE A SEPARATE
    /// SECTION. Everything above rebuilds the Newton model; everything here changes how the
    /// already-published grains are DRAWN and takes effect on the next frame. Mixing the two in
    /// one list would teach the operator that every control costs eleven seconds and a lost bed,
    /// which would make them stop using the cheap ones.
    struct FRenderParam {
        FString CVar;
        FString Label;
        FString Tooltip;
        enum class EKind { Float, Int, Bool, Text } Kind = EKind::Float;
    };
    static const TArray<FRenderParam>& RenderParams();

    /// ⚠ RESOLVED ONCE AND REMEMBERED. Every editor lambda below runs on EVERY REPAINT, so calling
    /// FindConsoleVariable from them turned into hundreds of name lookups a second and Unreal said
    /// so: "Console object named 'airsim.MpmRenderScale' shows many (500) FindConsoleObject()
    /// calls". A console variable's address is stable for the life of the process once registered,
    /// so the only correct number of lookups is one.
    IConsoleVariable* CVar(const FString& Name);
    TMap<FString, IConsoleVariable*> CVarCache;

    static FString ProfilePath();
    void LoadProfile();
    bool SaveProfile();

    TMap<FString, double> ProfileValues;
    FString ProfileMessage;
    /// ⚠ Set when a value has been edited but not yet applied by a restart, so the panel can say
    /// which of the numbers on screen the RUNNING sidecar is actually using — the file and the
    /// process disagree between Save and Restart, and that gap is where "I changed it and nothing
    /// happened" comes from.
    bool bProfileDirty = false;

    /// ⚠ WHAT THE PANEL CAN AND CANNOT SEE WHILE THE SIDECAR STARTS. It reads the WIRE, not the
    /// process, so between pressing Start and the first published frame it genuinely does not know
    /// whether python is compiling Warp kernels, the model is building, or the launch failed
    /// outright. Reporting "starting" for all three is honest; reporting "started" would not be.
    /// What it can do is time it, and say when it has waited longer than a build ever takes.
    ///
    /// Seconds (FPlatformTime) at which Start or Restart was last requested, or -1 when idle.
    double ActionRequestedSeconds = -1.0;
    /// How long the last start took to reach a live wire, for the line that reports it.
    double LastStartupSeconds = -1.0;
    FString StartupPhase() const;

    /// What the last launch attempt said, for the case where the script could not be run at all —
    /// a wrong path shows as nothing happening, which is indistinguishable from a sidecar that
    /// ignored the request.
    FString LastActionMessage;
};

#endif // WITH_EDITOR
