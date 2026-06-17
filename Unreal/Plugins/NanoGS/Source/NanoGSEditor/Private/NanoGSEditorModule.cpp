// Copyright Epic Games, Inc. All Rights Reserved.

#include "NanoGSEditorModule.h"
#include "GaussianSplatAssetTypeActions.h"
#include "GaussianSplatThumbnailRenderer.h"
#include "GaussianSplatAsset.h"
#include "NanoGSLightingSettings.h"
#include "AssetToolsModule.h"
#include "IAssetTools.h"
#include "ThumbnailRendering/ThumbnailManager.h"
#include "Widgets/Docking/SDockTab.h"
#include "Framework/Docking/TabManager.h"
#include "PropertyEditorModule.h"
#include "IDetailsView.h"

#define LOCTEXT_NAMESPACE "FNanoGSEditorModule"

const FName FNanoGSEditorModule::LightingPanelTabId(TEXT("NanoGSLightingPanel"));

void FNanoGSEditorModule::StartupModule()
{
	// Register asset type actions
	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();

	// Register Gaussian Splat asset type
	TSharedPtr<IAssetTypeActions> GaussianSplatAssetActions = MakeShareable(new FAssetTypeActions_GaussianSplatAsset());
	AssetTools.RegisterAssetTypeActions(GaussianSplatAssetActions.ToSharedRef());
	RegisteredAssetTypeActions.Add(GaussianSplatAssetActions);

	// Register custom thumbnail renderer for Gaussian Splat assets
	UThumbnailManager::Get().RegisterCustomRenderer(
		UGaussianSplatAsset::StaticClass(),
		UGaussianSplatThumbnailRenderer::StaticClass());

	// Register the dockable "NanoGS Lighting" panel under the editor's Window menu, so the
	// lighting CVars can be tuned without navigating to Project Settings every time.
	FGlobalTabmanager::Get()->RegisterNomadTabSpawner(
		LightingPanelTabId,
		FOnSpawnTab::CreateRaw(this, &FNanoGSEditorModule::SpawnLightingPanelTab))
		.SetDisplayName(LOCTEXT("NanoGSLightingPanelTitle", "NanoGS Lighting"))
		.SetTooltipText(LOCTEXT("NanoGSLightingPanelTooltip", "Tune NanoGS screen-space splat lighting"))
		.SetMenuType(ETabSpawnerMenuType::Enabled);

	UE_LOG(LogTemp, Log, TEXT("GaussianSplattingEditor module started."));
}

void FNanoGSEditorModule::ShutdownModule()
{
	// Unregister asset type actions
	if (FModuleManager::Get().IsModuleLoaded("AssetTools"))
	{
		IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
		for (auto& Action : RegisteredAssetTypeActions)
		{
			AssetTools.UnregisterAssetTypeActions(Action.ToSharedRef());
		}
	}
	RegisteredAssetTypeActions.Empty();

	if (FGlobalTabmanager::Get()->HasTabSpawner(LightingPanelTabId))
	{
		FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(LightingPanelTabId);
	}

	UE_LOG(LogTemp, Log, TEXT("GaussianSplattingEditor module shutdown."));
}

TSharedRef<SDockTab> FNanoGSEditorModule::SpawnLightingPanelTab(const FSpawnTabArgs& Args)
{
	FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

	FDetailsViewArgs DetailsViewArgs;
	DetailsViewArgs.bAllowSearch = false;
	DetailsViewArgs.bShowOptions = false;
	DetailsViewArgs.NameAreaSettings = FDetailsViewArgs::HideNameArea;
	DetailsViewArgs.ViewIdentifier = TEXT("NanoGSLightingPanel");

	TSharedRef<IDetailsView> DetailsView = PropertyEditorModule.CreateDetailView(DetailsViewArgs);
	// Edit the live singleton CDO directly, same object Project Settings edits — keeps both in sync.
	DetailsView->SetObject(GetMutableDefault<UNanoGSLightingSettings>());

	return SNew(SDockTab)
		.TabRole(ETabRole::NomadTab)
		[
			DetailsView
		];
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FNanoGSEditorModule, NanoGSEditor)
