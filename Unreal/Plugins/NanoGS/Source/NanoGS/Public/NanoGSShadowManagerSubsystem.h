// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "NanoGSShadowManagerSubsystem.generated.h"

class USceneCaptureComponent2D;
class USceneCaptureComponentCube;
class UTextureRenderTarget2D;
class UTextureRenderTargetCube;
class UPrimitiveComponent;
class ULightComponent;

/**
 * gs.ShadowMode = Proxy Mesh: one pool slot's capture state. Directional/Spot use a single
 * USceneCaptureComponent2D (one shadow frustum); Point lights use a USceneCaptureComponentCube
 * (all 6 faces in one component, public API, no manual per-face work needed).
 *
 * LightWorldPos/LightDirection are stored so the render thread can match this capture back to the
 * corresponding FGaussianLight entry GatherSceneLighting already selected (NanoGS.cpp) — there's
 * no shared stable identity between a game-thread ULightComponent and a render-thread
 * FLightSceneProxy, so matching is done by comparing position (local lights) or direction
 * (directional) with a small epsilon. This is a known, documented approximation (see
 * Dynamic-Lighting-Notes/nanogs-dynamic-lighting.md).
 */
USTRUCT()
struct FNanoGSShadowCapture
{
	GENERATED_BODY()

	UPROPERTY() TObjectPtr<USceneCaptureComponent2D> Capture2D = nullptr;
	UPROPERTY() TObjectPtr<USceneCaptureComponentCube> CaptureCube = nullptr;
	UPROPERTY() TObjectPtr<UTextureRenderTarget2D> RenderTarget2D = nullptr;
	UPROPERTY() TObjectPtr<UTextureRenderTargetCube> RenderTargetCube = nullptr;

	bool bAssigned = false;     // true if a real light is bound to this slot this frame
	bool bIsCube = false;       // point light (cube) vs directional/spot (2D)
	bool bIsDirectional = false;
	FVector LightWorldPos = FVector::ZeroVector;   // local lights: world position for matching
	FVector LightDirection = FVector::ZeroVector;  // directional: travel direction for matching

	// On-demand recapture: bCaptureEveryFrame is OFF (a full extra scene render per light per
	// frame is wasteful for the common case of static lights/geometry) — CaptureScene() is only
	// called when the bound light's transform actually moved since last tick, or this is the first
	// time this slot was bound. Doesn't yet account for the tagged mesh itself moving — a known
	// limitation, not handled in this pass.
	bool bHasCaptured = false;
	FVector LastLightPos = FVector(NAN, NAN, NAN);
	FRotator LastLightRot = FRotator(NAN, NAN, NAN);
	// 2D (directional/spot) only. ViewMatrix alone gives view-space Z, the same linear-depth
	// convention SCS_SceneDepth writes into the capture texture (matches it regardless of whether
	// the projection is ortho or perspective) — used for the shadow depth comparison. ViewProjMatrix
	// (View * Proj) is separately needed to find which UV to sample in the first place.
	FMatrix44f ViewMatrix = FMatrix44f::Identity;
	FMatrix44f ViewProjMatrix = FMatrix44f::Identity;
};

/**
 * gs.ShadowMode = Proxy Mesh manager. Scans the world each tick (game thread) for actors carrying
 * UNanoGSShadowCasterComponent, and for the gs.ShadowMaxLights nearest lights with "Cast Shadows"
 * enabled, maintains a depth-only USceneCaptureComponent2D/Cube pointed from that light at those
 * tagged primitives only (via ShowOnlyComponents) — giving Gaussian splats a shadow term without
 * touching UE's own shadow-map/VSM internals (see the plan doc for why that's out of scope).
 */
UCLASS()
class NANOGS_API UNanoGSShadowManagerSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	//~ Begin UTickableWorldSubsystem Interface
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	// FTickableGameObject::IsTickableInEditor() defaults to false — without this override, the
	// engine's tickable-object dispatcher (Tickable.cpp) never calls Tick() at all while just
	// viewing the level in the editor viewport (only Play-In-Editor counts as a "game world"
	// otherwise). NanoGS is used live in-editor, not just in PIE, so this must be true.
	virtual bool IsTickableInEditor() const override { return true; }
	virtual bool DoesSupportWorldType(EWorldType::Type WorldType) const override;
	//~ End UTickableWorldSubsystem Interface

	/** This frame's active (bAssigned) shadow captures. Game-thread only — call from
	 *  FGaussianSplatViewExtension::BeginRenderViewFamily (also game thread) to snapshot before
	 *  handing off to the render thread. */
	void GetActiveCaptures(TArray<FNanoGSShadowCapture>& OutCaptures) const;

private:
	void RefreshShadowCasterPrimitives();
	void EnsureCapture2D(FNanoGSShadowCapture& Slot);
	void EnsureCaptureCube(FNanoGSShadowCapture& Slot);
	void BindDirectional(FNanoGSShadowCapture& Slot, ULightComponent* Light);
	void BindSpot(FNanoGSShadowCapture& Slot, ULightComponent* Light);
	void BindPoint(FNanoGSShadowCapture& Slot, ULightComponent* Light);
	void SyncShowOnlyComponents(FNanoGSShadowCapture& Slot);
	bool ShouldRecapture(FNanoGSShadowCapture& Slot, const FVector& NewPos, const FRotator& NewRot);

	UPROPERTY()
	TArray<TObjectPtr<UPrimitiveComponent>> ShadowCasterPrimitives;

	UPROPERTY()
	TArray<FNanoGSShadowCapture> CapturePool2D;   // directional/spot, sized to gs.ShadowMaxLights

	UPROPERTY()
	TArray<FNanoGSShadowCapture> CapturePoolCube; // point lights, smaller pool (6x render cost each)

	// Combined bounds of tagged shadow-caster primitives, used to fit the directional ortho frustum.
	FVector CachedBoundsOrigin = FVector::ZeroVector;
	float CachedBoundsRadius = 1000.f;
	bool bHasCachedBounds = false;
};
