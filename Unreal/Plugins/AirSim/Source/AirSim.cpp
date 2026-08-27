// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "AirSim.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Modules/ModuleInterface.h"
#include "Interfaces/IPluginManager.h"
#include "ShaderCore.h"
#include "Misc/CoreDelegates.h"
#include "NewtonSidecarLaunch.h"

#if WITH_EDITOR
#include "NewtonSidecarPanel.h"
#endif

class FAirSim : public IModuleInterface
{
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};

IMPLEMENT_MODULE(FAirSim, AirSim)

void FAirSim::StartupModule()
{
    //plugin startup
    UE_LOG(LogTemp, Log, TEXT("StartupModule: AirSim plugin"));

#if WITH_EDITOR
    // ⚠ EDITOR ONLY. A status panel is a development tool; compiling it into a packaged build
    // would drag UnrealEd into a runtime module for something no shipped game can open.
    //
    // ⚠ AND DEFERRED TO OnPostEngineInit, NOT DONE HERE. Registering a nomad tab spawner from
    // StartupModule loads the editor style and workspace-menu modules, and those construct class
    // default objects — during plugin startup the UObject system is not ready for that, so the
    // editor dies before it opens with a CastChecked<UPackage> failure inside
    // FEditorStyleModule::StartupModule. Measured 2026-08-27: the editor crashed on launch with
    // exactly that stack, and the panel is a diagnostic tool, so taking the editor down with it is
    // the worst possible failure mode for it to have.
    // ⚠ TWO ENTRY POINTS, AND Register() IS IDEMPOTENT. OnPostEngineInit alone did not fire for
    // this module on UE 5.6 — the log carried "StartupModule: AirSim plugin" and never the
    // registration line — and rather than guess why, bind to both and let whichever runs first do
    // the work. Each logs which one it was, so the next reader gets the answer instead of the
    // guess.
    FCoreDelegates::OnPostEngineInit.AddStatic(&FNewtonSidecarPanel::Register);
    FCoreDelegates::OnFEngineLoopInitComplete.AddStatic(&FNewtonSidecarPanel::Register);
    UE_LOG(LogTemp, Log,
           TEXT("[AirSim] Newton Sidecar panel registration bound to OnPostEngineInit and "
                "OnFEngineLoopInitComplete"));
#endif

    //Phase 3b step 4: the resample shader lives in Plugins/AirSim/Shaders/Private and is
    //addressed as /Plugin/AirSim/Private/CubeResample.usf. Same mechanism, same virtual path
    //shape, as the NanoGS plugin uses. Mapping a shader directory costs nothing at run time and
    //nothing is compiled from it unless a shader in it is referenced.
    TSharedPtr<IPlugin> plugin = IPluginManager::Get().FindPlugin(TEXT("AirSim"));
    if (plugin.IsValid()) {
        AddShaderSourceDirectoryMapping(TEXT("/Plugin/AirSim"),
                                        FPaths::Combine(plugin->GetBaseDir(), TEXT("Shaders")));
    }
    else {
        UE_LOG(LogTemp, Warning,
               TEXT("[AirSim] plugin base directory not found; /Plugin/AirSim shaders unavailable ")
               TEXT("(generic camera models will fall back to the pinhole path)"));
    }
}

void FAirSim::ShutdownModule()
{
    // ⚠ ONLY IF THIS EDITOR STARTED IT. A sidecar launched from a terminal — for a probe run, or
    // to keep a bed alive across editor restarts — must survive an editor that had nothing to do
    // with it. And this covers the CLEAN exit only: a crash never runs it, which is why an
    // editor-started sidecar is also handed --parent-pid and exits on its own when that pid goes.
    StopNewtonSidecarIfOwned();
#if WITH_EDITOR
    FNewtonSidecarPanel::Unregister();
#endif
    //plugin shutdown
}