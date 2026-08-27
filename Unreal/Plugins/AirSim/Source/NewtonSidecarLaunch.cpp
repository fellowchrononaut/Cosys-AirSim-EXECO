#include "NewtonSidecarLaunch.h"

#include "HAL/IConsoleManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/Paths.h"

DEFINE_LOG_CATEGORY_STATIC(LogNewtonSidecarLaunch, Log, All);

namespace
{
/// ⚠ Set only by a launch this process made. See the header on why a terminal-started sidecar must
/// survive the editor.
bool GOwnsSidecar = false;
} // namespace

FString ResolveNewtonSidecarControlScript()
{
    static IConsoleVariable* Override =
        IConsoleManager::Get().FindConsoleVariable(TEXT("airsim.NewtonSidecarControlScript"));
    if (Override != nullptr) {
        const FString Value = Override->GetString();
        if (!Value.IsEmpty())
            return Value;
    }
    // The SIMVAL repo is four directories above the Unreal project, the same anchor
    // startMpmSidecarProcess uses to find mpm_sidecar/sidecar.py.
    return FPaths::ConvertRelativePathToFull(
        FPaths::Combine(FPaths::ProjectDir(), TEXT("../../../../tools/sidecar_ctl.sh")));
}

bool RunNewtonSidecarControl(const FString& Verb, bool bTakeOwnership, FString& OutMessage)
{
    const FString Script = ResolveNewtonSidecarControlScript();
    if (!FPaths::FileExists(Script)) {
        OutMessage = FString::Printf(
            TEXT("cannot run '%s' - set airsim.NewtonSidecarControlScript to tools/sidecar_ctl.sh"),
            *Script);
        UE_LOG(LogNewtonSidecarLaunch, Error, TEXT("%s"), *OutMessage);
        return false;
    }

    // ⚠ OUR OWN PID GOES WITH IT, ON START. The sidecar watches that pid and exits when it
    // disappears, which is the only thing that covers an editor CRASH — the clean-exit path below
    // never runs then. The runner turns PARENT_PID into --parent-pid; a sidecar started from a
    // terminal passes nothing and outlives everything, which is what a probe run needs.
    FString Params = Verb.ToLower();
    if (bTakeOwnership && !Verb.Equals(TEXT("Stop"), ESearchCase::IgnoreCase))
        Params += FString::Printf(TEXT(" PARENT_PID=%u"), FPlatformProcess::GetCurrentProcessId());

    uint32 ProcessId = 0;
    const FProcHandle Handle = FPlatformProcess::CreateProc(
        *Script, *Params, /*bLaunchDetached=*/true, /*bLaunchHidden=*/false,
        /*bLaunchReallyHidden=*/false, &ProcessId, 0, nullptr, nullptr);
    if (!Handle.IsValid()) {
        OutMessage = FString::Printf(TEXT("could not launch %s %s"), *Script, *Params);
        UE_LOG(LogNewtonSidecarLaunch, Error, TEXT("%s"), *OutMessage);
        return false;
    }
    FPlatformProcess::CloseProc(const_cast<FProcHandle&>(Handle));

    if (Verb.Equals(TEXT("Stop"), ESearchCase::IgnoreCase))
        GOwnsSidecar = false;
    else if (bTakeOwnership)
        GOwnsSidecar = true;

    OutMessage = FString::Printf(
        TEXT("%s requested (pid %u) - watch the counters, they are the evidence"), *Verb,
        ProcessId);
    UE_LOG(LogNewtonSidecarLaunch, Log, TEXT("sidecar %s%s"), *OutMessage,
           GOwnsSidecar ? TEXT("; this editor owns it and will stop it on exit") : TEXT(""));
    return true;
}

void StopNewtonSidecarIfOwned()
{
    if (!GOwnsSidecar)
        return;
    GOwnsSidecar = false;
    FString Message;
    UE_LOG(LogNewtonSidecarLaunch, Log,
           TEXT("editor is exiting and it started the sidecar - stopping it"));
    RunNewtonSidecarControl(TEXT("Stop"), /*bTakeOwnership=*/false, Message);
}
