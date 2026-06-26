// Copyright Epic Games, Inc. All Rights Reserved.

#include "NanoGSLightingSettings.h"
#include "HAL/IConsoleManager.h"

UNanoGSLightingSettings::UNanoGSLightingSettings()
{
	CategoryName = TEXT("Plugins");
}

namespace
{
	void SetCVarFloat(const TCHAR* Name, float Value)
	{
		if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(Name))
		{
			// SetByConsole so editing the panel behaves like typing the command (last edit wins,
			// whether it came from this UI or the console).
			CVar->Set(Value, ECVF_SetByConsole);
		}
	}

	void SetCVarInt(const TCHAR* Name, int32 Value)
	{
		if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(Name))
		{
			CVar->Set(Value, ECVF_SetByConsole);
		}
	}
}

void UNanoGSLightingSettings::ApplyToCVars() const
{
	SetCVarInt  (TEXT("gs.LightingGeometryMode"),  (int32)GeometryMode);
	SetCVarInt  (TEXT("gs.LightingDebugView"),     bShowReconstructedNormals ? 1 : 0);
	SetCVarFloat(TEXT("gs.LightingBlend"),         LightingBlend);
	SetCVarFloat(TEXT("gs.AmbientIntensity"),      AmbientIntensity);
	SetCVarFloat(TEXT("gs.LightIntensityScale"),   LightIntensityScale);
	SetCVarFloat(TEXT("gs.LightResponseCeiling"),  LightResponseCeiling);
	SetCVarFloat(TEXT("gs.LightWrap"),             LightWrap);
	SetCVarFloat(TEXT("gs.RelightRatioMin"),       RelightRatioMin);
	SetCVarFloat(TEXT("gs.RelightRatioMax"),       RelightRatioMax);
	SetCVarInt  (TEXT("gs.UseRelightRatio"),       bUseRelightRatio ? 1 : 0);
	SetCVarInt  (TEXT("gs.NormalConfidenceFade"),  bNormalConfidenceFade ? 1 : 0);
	SetCVarInt  (TEXT("gs.DepthProximityWeighting"), bDepthProximityWeighting ? 1 : 0);
	SetCVarInt  (TEXT("gs.NormalSampleStep"),      NormalSampleStep);
	SetCVarInt  (TEXT("gs.NormalSmoothRadius"),    NormalSmoothRadius);
	SetCVarFloat(TEXT("gs.NormalSmoothDepthSigma"), NormalSmoothDepthSigma);
}

void UNanoGSLightingSettings::PostInitProperties()
{
	Super::PostInitProperties();
	// Push persisted config values to the CVars. If the CVars aren't registered yet (very early
	// init), FindConsoleVariable returns null and we skip — FNanoGSModule::OnPostEngineInit
	// re-applies once the module is fully up, so this is just an early best-effort.
	ApplyToCVars();
}

#if WITH_EDITOR

FText UNanoGSLightingSettings::GetSectionText() const
{
	return NSLOCTEXT("NanoGS", "NanoGSLightingSectionText", "NanoGS Lighting");
}

static void ApplyPreset(UNanoGSLightingSettings& S, ENanoGSLightingPreset Preset)
{
	switch (Preset)
	{
	case ENanoGSLightingPreset::Off:
		S.LightingBlend = 0.0f;
		break;
	case ENanoGSLightingPreset::Subtle:
		S.LightingBlend = 0.3f;  S.AmbientIntensity = 0.15f; S.LightIntensityScale = 0.10f;
		S.NormalSampleStep = 6;  S.NormalSmoothRadius = 2;   S.NormalSmoothDepthSigma = 0.02f;
		break;
	case ENanoGSLightingPreset::Balanced:
		S.LightingBlend = 0.5f;  S.AmbientIntensity = 0.10f; S.LightIntensityScale = 0.10f;
		S.NormalSampleStep = 4;  S.NormalSmoothRadius = 2;   S.NormalSmoothDepthSigma = 0.02f;
		break;
	case ENanoGSLightingPreset::Strong:
		S.LightingBlend = 0.8f;  S.AmbientIntensity = 0.05f; S.LightIntensityScale = 0.15f;
		S.NormalSampleStep = 3;  S.NormalSmoothRadius = 3;   S.NormalSmoothDepthSigma = 0.03f;
		break;
	default:
		break;  // Custom: leave values untouched
	}
}

void UNanoGSLightingSettings::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	const FName PropName = PropertyChangedEvent.Property ? PropertyChangedEvent.Property->GetFName() : NAME_None;

	if (PropName == GET_MEMBER_NAME_CHECKED(UNanoGSLightingSettings, Preset))
	{
		// Choosing a preset (other than Custom) fills in the values.
		if (Preset != ENanoGSLightingPreset::Custom)
		{
			ApplyPreset(*this, Preset);
		}
	}
	else if (PropName != NAME_None)
	{
		// Manually tweaking any value means we're no longer on a named preset.
		Preset = ENanoGSLightingPreset::Custom;
	}

	ApplyToCVars();
	SaveConfig();

	Super::PostEditChangeProperty(PropertyChangedEvent);
}

#endif // WITH_EDITOR
