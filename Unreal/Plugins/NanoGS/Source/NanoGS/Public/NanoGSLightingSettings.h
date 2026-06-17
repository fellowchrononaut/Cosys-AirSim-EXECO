// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "NanoGSLightingSettings.generated.h"

/** Quick-start presets for NanoGS screen-space lighting. Choosing one fills the values below. */
UENUM()
enum class ENanoGSLightingPreset : uint8
{
	Custom    UMETA(DisplayName = "Custom (manual)"),
	Off       UMETA(DisplayName = "Off"),
	Subtle    UMETA(DisplayName = "Subtle"),
	Balanced  UMETA(DisplayName = "Balanced"),
	Strong    UMETA(DisplayName = "Strong"),
};

/** Geometry source used to reconstruct normals for screen-space lighting. */
UENUM()
enum class ENanoGSLightingGeometryMode : uint8
{
	SplatDepth      UMETA(DisplayName = "Splat Depth (no setup needed)"),
	ProxyMesh       UMETA(DisplayName = "Proxy Mesh CustomDepth (clean, needs scene setup)"),
	PerSplatNormal  UMETA(DisplayName = "Per-Splat Normal (clean, no setup needed)"),
};

/**
 * Project Settings page (Edit > Project Settings > Plugins > NanoGS Lighting) for the
 * Gaussian-splat screen-space dynamic lighting. These values drive the gs.* console variables
 * the renderer reads, so the editor UI and the console stay in sync.
 */
UCLASS(config = Game, defaultconfig, meta = (DisplayName = "NanoGS Lighting"))
class NANOGS_API UNanoGSLightingSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UNanoGSLightingSettings();

	//~ UDeveloperSettings
	virtual FName GetCategoryName() const override { return FName(TEXT("Plugins")); }
#if WITH_EDITOR
	virtual FText GetSectionText() const override;
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	virtual void PostInitProperties() override;

	/** Pick a preset to fill in sensible values, or leave on Custom to tune everything by hand. */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting")
	ENanoGSLightingPreset Preset = ENanoGSLightingPreset::Custom;

	/** Geometry source for lighting normals. Splat Depth needs no scene setup but is noisy
	 *  (see Normal Smoothing below). Proxy Mesh gives clean normals from a hidden mesh co-located
	 *  with the splat — enable "Render CustomDepth Pass" on that mesh's component for this to work.
	 *  Per-Splat Normal uses each splat's own shape (no external mesh, no screen-space noise).
	 *  (gs.LightingGeometryMode) */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting")
	ENanoGSLightingGeometryMode GeometryMode = ENanoGSLightingGeometryMode::SplatDepth;

	/** 0 = unlit (original splat appearance), 1 = fully lit by the scene's lights. (gs.LightingBlend) */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting",
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float LightingBlend = 0.0f;

	/** Ambient fill added on the shadowed side of splats. (gs.AmbientIntensity) */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting",
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float AmbientIntensity = 0.1f;

	/** Sensitivity mapping a scene light's intensity to its effect on splats. Raise to brighten. (gs.LightIntensityScale) */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting",
		meta = (ClampMin = "0.0", ClampMax = "2.0", UIMin = "0.0", UIMax = "1.0"))
	float LightIntensityScale = 0.1f;

	/** Per-light brightness ceiling. 1.0 = original behavior (each light asymptotes to full
	 *  brightness and no brighter). Raise above 1.0 to let strong lights keep getting brighter
	 *  instead of plateauing. (gs.LightResponseCeiling) */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting",
		meta = (ClampMin = "0.1", ClampMax = "8.0", UIMin = "1.0", UIMax = "4.0"))
	float LightResponseCeiling = 1.0f;

	/** Clamp floor for the brightness-preserving relight ratio. Splat colors are already a baked,
	 *  lit appearance, not raw albedo — this lets directional shading vary that baked brightness
	 *  within [Min, Max] instead of multiplying by the absolute light level (which double-shades
	 *  and darkens). Lower = darker shadows allowed. (gs.RelightRatioMin) */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting",
		meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float RelightRatioMin = 0.6f;

	/** Clamp ceiling for the brightness-preserving relight ratio. Higher = brighter highlights
	 *  allowed. (gs.RelightRatioMax) */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting",
		meta = (ClampMin = "1.0", ClampMax = "4.0", UIMin = "1.0", UIMax = "3.0"))
	float RelightRatioMax = 1.6f;

	/** Normal baseline (pixels): the surface slope is measured between samples this far apart.
	 *  Higher = flatter, less noisy surfaces (the main fix for the "noisy SfM mesh" look). (gs.NormalSampleStep) */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting|Normal Smoothing",
		meta = (ClampMin = "1", ClampMax = "16", UIMin = "1", UIMax = "12"))
	int32 NormalSampleStep = 3;

	/** Bilateral depth-smoothing kernel radius (pixels). 0 = off. (gs.NormalSmoothRadius) */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting|Normal Smoothing",
		meta = (ClampMin = "0", ClampMax = "6", UIMin = "0", UIMax = "6"))
	int32 NormalSmoothRadius = 2;

	/** Edge preservation: depth-similarity sigma as a fraction of view depth. Smaller keeps
	 *  silhouettes/creases sharper; larger smooths more. (gs.NormalSmoothDepthSigma) */
	UPROPERTY(EditAnywhere, config, Category = "NanoGS Lighting|Normal Smoothing",
		meta = (ClampMin = "0.0", ClampMax = "0.2", UIMin = "0.0", UIMax = "0.1"))
	float NormalSmoothDepthSigma = 0.02f;

	/** Push the current values into the gs.* console variables (which the renderer reads). */
	void ApplyToCVars() const;
};
