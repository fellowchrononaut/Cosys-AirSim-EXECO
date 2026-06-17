// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class SDockTab;
class FSpawnTabArgs;

class FNanoGSEditorModule : public IModuleInterface
{
public:
	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	/** Tab ID for the dockable "NanoGS Lighting" panel, opened via the Window menu. */
	static const FName LightingPanelTabId;

private:
	/** Handle for registered asset type actions */
	TArray<TSharedPtr<class IAssetTypeActions>> RegisteredAssetTypeActions;

	TSharedRef<SDockTab> SpawnLightingPanelTab(const FSpawnTabArgs& Args);
};
