// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "AirSim.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Modules/ModuleInterface.h"
#include "Interfaces/IPluginManager.h"
#include "ShaderCore.h"

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
    //plugin shutdown
}