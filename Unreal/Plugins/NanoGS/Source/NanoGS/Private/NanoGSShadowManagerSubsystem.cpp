// Copyright Epic Games, Inc. All Rights Reserved.

#include "NanoGSShadowManagerSubsystem.h"
#include "NanoGSShadowCasterComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SceneCaptureComponentCube.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/TextureRenderTargetCube.h"
#include "Components/PrimitiveComponent.h"
#include "Components/LightComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/PointLightComponent.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Engine/World.h"

namespace
{
	// Fixed shadow-map resolution for v1; not yet exposed as a setting.
	constexpr int32 ShadowMapResolution = 1024;
}

void UNanoGSShadowManagerSubsystem::Tick(float DeltaTime)
{
	{
		static bool bLoggedFirstTick = false;
		if (!bLoggedFirstTick)
		{
			UE_LOG(LogTemp, Log, TEXT("[NanoGSShadowManager] Tick is running (world: %s, type: %d)"),
				GetWorld() ? *GetWorld()->GetName() : TEXT("null"), GetWorld() ? (int32)GetWorld()->WorldType.GetValue() : -1);
			bLoggedFirstTick = true;
		}
	}

	IConsoleVariable* CVarMode = IConsoleManager::Get().FindConsoleVariable(TEXT("gs.ShadowMode"));
	const int32 ShadowMode = CVarMode ? CVarMode->GetInt() : 0;

	if (ShadowMode != 1)  // Proxy Mesh only — Off/Splat-Self-Shadow don't use this manager
	{
		for (FNanoGSShadowCapture& Slot : CapturePool2D)
		{
			Slot.bAssigned = false;
			if (Slot.Capture2D) Slot.Capture2D->SetVisibility(false);
		}
		for (FNanoGSShadowCapture& Slot : CapturePoolCube)
		{
			Slot.bAssigned = false;
			if (Slot.CaptureCube) Slot.CaptureCube->SetVisibility(false);
		}
		return;
	}

	RefreshShadowCasterPrimitives();

	IConsoleVariable* CVarMaxLights = IConsoleManager::Get().FindConsoleVariable(TEXT("gs.ShadowMaxLights"));
	const int32 MaxLights = CVarMaxLights ? FMath::Clamp(CVarMaxLights->GetInt(), 0, 4) : 4;
	// Point lights cost 6x (cubemap) — keep their pool smaller than the 2D (directional/spot) pool.
	const int32 MaxCubeLights = FMath::Min(MaxLights, 2);

	UWorld* World = GetWorld();
	if (!World) return;

	// Reference position for distance scoring. ViewLocationsRenderedLastFrame works in both PIE
	// and editor viewports (unlike PlayerController/local-player camera lookups, which assume a
	// running game).
	FVector RefPos = FVector::ZeroVector;
	if (World->ViewLocationsRenderedLastFrame.Num() > 0)
	{
		RefPos = World->ViewLocationsRenderedLastFrame[0];
	}

	struct FCandidate { ULightComponent* Light; float Score; };
	TArray<FCandidate> Candidates;
	for (TObjectIterator<ULightComponent> It; It; ++It)
	{
		ULightComponent* Light = *It;
		if (!Light || !IsValid(Light)) continue;
		if (Light->GetWorld() != World) continue;
		if (!Light->IsRegistered() || !Light->IsVisible()) continue;
		if (!Light->CastShadows) continue;  // mirrors the same "Cast Shadows" property the render-thread side reads
		Candidates.Add({Light, FVector::Dist(Light->GetComponentLocation(), RefPos)});
	}
	Candidates.Sort([](const FCandidate& A, const FCandidate& B) { return A.Score < B.Score; });

	{
		static int32 LastLoggedCandidates = -1;
		static int32 LastLoggedPrimitives = -1;
		if (Candidates.Num() != LastLoggedCandidates || ShadowCasterPrimitives.Num() != LastLoggedPrimitives)
		{
			UE_LOG(LogTemp, Log, TEXT("[NanoGSShadowManager] Tick: %d candidate shadow-casting lights, %d tagged shadow-caster primitives"),
				Candidates.Num(), ShadowCasterPrimitives.Num());
			LastLoggedCandidates = Candidates.Num();
			LastLoggedPrimitives = ShadowCasterPrimitives.Num();
		}
	}

	for (FNanoGSShadowCapture& Slot : CapturePool2D)   Slot.bAssigned = false;
	for (FNanoGSShadowCapture& Slot : CapturePoolCube) Slot.bAssigned = false;

	int32 Next2D = 0, NextCube = 0;
	for (const FCandidate& Cand : Candidates)
	{
		ULightComponent* Light = Cand.Light;
		const bool bIsSpot = Light->IsA<USpotLightComponent>();
		const bool bIsPoint = !bIsSpot && Light->IsA<UPointLightComponent>();
		const bool bIsDirectional = Light->IsA<UDirectionalLightComponent>();

		if (bIsPoint)
		{
			if (NextCube >= MaxCubeLights) continue;
			if (NextCube >= CapturePoolCube.Num()) CapturePoolCube.AddDefaulted();
			FNanoGSShadowCapture& Slot = CapturePoolCube[NextCube++];
			EnsureCaptureCube(Slot);
			BindPoint(Slot, Light);
		}
		else if (bIsSpot || bIsDirectional)
		{
			if (Next2D >= MaxLights) continue;
			if (Next2D >= CapturePool2D.Num()) CapturePool2D.AddDefaulted();
			FNanoGSShadowCapture& Slot = CapturePool2D[Next2D++];
			EnsureCapture2D(Slot);
			if (bIsDirectional) BindDirectional(Slot, Light);
			else                BindSpot(Slot, Light);
		}
	}

	for (FNanoGSShadowCapture& Slot : CapturePool2D)
	{
		if (!Slot.bAssigned && Slot.Capture2D) Slot.Capture2D->SetVisibility(false);
	}
	for (FNanoGSShadowCapture& Slot : CapturePoolCube)
	{
		if (!Slot.bAssigned && Slot.CaptureCube) Slot.CaptureCube->SetVisibility(false);
	}
}

TStatId UNanoGSShadowManagerSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UNanoGSShadowManagerSubsystem, STATGROUP_Tickables);
}

bool UNanoGSShadowManagerSubsystem::DoesSupportWorldType(EWorldType::Type WorldType) const
{
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE || WorldType == EWorldType::Editor;
}

void UNanoGSShadowManagerSubsystem::GetActiveCaptures(TArray<FNanoGSShadowCapture>& OutCaptures) const
{
	OutCaptures.Reset();
	for (const FNanoGSShadowCapture& Slot : CapturePool2D)
	{
		if (Slot.bAssigned) OutCaptures.Add(Slot);
	}
	for (const FNanoGSShadowCapture& Slot : CapturePoolCube)
	{
		if (Slot.bAssigned) OutCaptures.Add(Slot);
	}
}

void UNanoGSShadowManagerSubsystem::RefreshShadowCasterPrimitives()
{
	ShadowCasterPrimitives.Reset();
	FBox CombinedBounds(ForceInit);

	for (TActorIterator<AActor> It(GetWorld()); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor || !Actor->FindComponentByClass<UNanoGSShadowCasterComponent>()) continue;

		TArray<UPrimitiveComponent*> Prims;
		Actor->GetComponents(Prims);
		for (UPrimitiveComponent* Prim : Prims)
		{
			if (!Prim) continue;
			ShadowCasterPrimitives.Add(Prim);
			CombinedBounds += Prim->Bounds.GetBox();
		}
	}

	bHasCachedBounds = ShadowCasterPrimitives.Num() > 0 && CombinedBounds.IsValid;
	if (bHasCachedBounds)
	{
		CachedBoundsOrigin = CombinedBounds.GetCenter();
		CachedBoundsRadius = FMath::Max(CombinedBounds.GetExtent().Size(), 100.f);
	}
}

bool UNanoGSShadowManagerSubsystem::ShouldRecapture(FNanoGSShadowCapture& Slot, const FVector& NewPos, const FRotator& NewRot)
{
	constexpr float PosEpsilon = 1.f;   // cm
	constexpr float RotEpsilonDeg = 0.1f;

	const bool bChanged = !Slot.bHasCaptured
		|| !NewPos.Equals(Slot.LastLightPos, PosEpsilon)
		|| !NewRot.Equals(Slot.LastLightRot, RotEpsilonDeg);

	if (bChanged)
	{
		Slot.LastLightPos = NewPos;
		Slot.LastLightRot = NewRot;
		Slot.bHasCaptured = true;
	}
	return bChanged;
}

void UNanoGSShadowManagerSubsystem::SyncShowOnlyComponents(FNanoGSShadowCapture& Slot)
{
	auto Sync = [this](USceneCaptureComponent2D* C) { if (C) { C->ShowOnlyComponents.Reset(); for (UPrimitiveComponent* P : ShadowCasterPrimitives) { C->ShowOnlyComponents.Add(P); } } };
	auto SyncCube = [this](USceneCaptureComponentCube* C) { if (C) { C->ShowOnlyComponents.Reset(); for (UPrimitiveComponent* P : ShadowCasterPrimitives) { C->ShowOnlyComponents.Add(P); } } };
	Sync(Slot.Capture2D);
	SyncCube(Slot.CaptureCube);
}

void UNanoGSShadowManagerSubsystem::EnsureCapture2D(FNanoGSShadowCapture& Slot)
{
	if (Slot.Capture2D) return;

	Slot.RenderTarget2D = NewObject<UTextureRenderTarget2D>(this);
	Slot.RenderTarget2D->RenderTargetFormat = RTF_R32f;
	Slot.RenderTarget2D->InitAutoFormat(ShadowMapResolution, ShadowMapResolution);
	Slot.RenderTarget2D->UpdateResourceImmediate(true);

	Slot.Capture2D = NewObject<USceneCaptureComponent2D>(this);
	Slot.Capture2D->CaptureSource = SCS_SceneDepth;
	Slot.Capture2D->TextureTarget = Slot.RenderTarget2D;
	// On-demand only (see ShouldRecapture) — a full extra scene render every frame for every
	// shadow-casting light is wasteful when lights/geometry are mostly static.
	Slot.Capture2D->bCaptureEveryFrame = false;
	Slot.Capture2D->bCaptureOnMovement = false;
	Slot.Capture2D->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
	Slot.Capture2D->RegisterComponentWithWorld(GetWorld());
}

void UNanoGSShadowManagerSubsystem::EnsureCaptureCube(FNanoGSShadowCapture& Slot)
{
	if (Slot.CaptureCube) return;

	Slot.RenderTargetCube = NewObject<UTextureRenderTargetCube>(this);
	Slot.RenderTargetCube->ClearColor = FLinearColor::Black;
	Slot.RenderTargetCube->Init(ShadowMapResolution, PF_R32_FLOAT);
	Slot.RenderTargetCube->UpdateResourceImmediate(true);

	Slot.CaptureCube = NewObject<USceneCaptureComponentCube>(this);
	Slot.CaptureCube->CaptureSource = SCS_SceneDepth;
	Slot.CaptureCube->TextureTarget = Slot.RenderTargetCube;
	// On-demand only (see ShouldRecapture) — especially important for cube captures (6x cost).
	Slot.CaptureCube->bCaptureEveryFrame = false;
	Slot.CaptureCube->bCaptureOnMovement = false;
	Slot.CaptureCube->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
	Slot.CaptureCube->RegisterComponentWithWorld(GetWorld());

	Slot.bIsCube = true;
}

void UNanoGSShadowManagerSubsystem::BindDirectional(FNanoGSShadowCapture& Slot, ULightComponent* Light)
{
	SyncShowOnlyComponents(Slot);

	const FVector LightDir = Light->GetDirection().GetSafeNormal();  // direction light travels
	const float Radius = bHasCachedBounds ? CachedBoundsRadius : 1000.f;
	const FVector Origin = bHasCachedBounds ? CachedBoundsOrigin : Light->GetComponentLocation();
	// Position the capture back along -LightDir (toward the light source) so it looks across the
	// tagged geometry's bounds toward the light's travel direction.
	const FVector CapturePos = Origin - LightDir * (Radius * 2.f);
	const FRotator CaptureRot = LightDir.Rotation();

	Slot.Capture2D->SetWorldLocationAndRotation(CapturePos, CaptureRot);
	Slot.Capture2D->ProjectionType = ECameraProjectionMode::Orthographic;
	Slot.Capture2D->OrthoWidth = FMath::Max(Radius * 2.2f, 100.f);
	Slot.Capture2D->bOverride_CustomNearClippingPlane = true;
	Slot.Capture2D->CustomNearClippingPlane = 10.f;
	Slot.Capture2D->MaxViewDistanceOverride = Radius * 4.f;
	Slot.Capture2D->SetVisibility(true);

	const FMatrix View = FLookAtMatrix(CapturePos, CapturePos + LightDir, FVector::UpVector);
	const FMatrix Proj = FReversedZOrthoMatrix(Slot.Capture2D->OrthoWidth * 0.5f, Slot.Capture2D->OrthoWidth * 0.5f, 1.f / (Radius * 4.f), 10.f);
	Slot.ViewMatrix = FMatrix44f(View);
	Slot.ViewProjMatrix = FMatrix44f(View * Proj);

	if (ShouldRecapture(Slot, CapturePos, CaptureRot))
	{
		Slot.Capture2D->CaptureScene();
	}

	Slot.bAssigned = true;
	Slot.bIsCube = false;
	Slot.bIsDirectional = true;
	Slot.LightDirection = LightDir;
}

void UNanoGSShadowManagerSubsystem::BindSpot(FNanoGSShadowCapture& Slot, ULightComponent* Light)
{
	SyncShowOnlyComponents(Slot);

	USpotLightComponent* Spot = CastChecked<USpotLightComponent>(Light);
	const FVector LightPos = Spot->GetComponentLocation();
	const FVector LightDir = Spot->GetDirection().GetSafeNormal();
	const float OuterConeDeg = Spot->OuterConeAngle;
	const float FarClip = FMath::Max(Spot->AttenuationRadius, 100.f);

	Slot.Capture2D->SetWorldLocationAndRotation(LightPos, LightDir.Rotation());
	Slot.Capture2D->ProjectionType = ECameraProjectionMode::Perspective;
	Slot.Capture2D->FOVAngle = FMath::Clamp(OuterConeDeg * 2.f, 1.f, 170.f);
	Slot.Capture2D->bOverride_CustomNearClippingPlane = true;
	Slot.Capture2D->CustomNearClippingPlane = 10.f;
	Slot.Capture2D->MaxViewDistanceOverride = FarClip;
	Slot.Capture2D->SetVisibility(true);

	const float HalfFOV = FMath::DegreesToRadians(Slot.Capture2D->FOVAngle) * 0.5f;
	const FMatrix View = FLookAtMatrix(LightPos, LightPos + LightDir, FVector::UpVector);
	const FMatrix Proj = FReversedZPerspectiveMatrix(HalfFOV, HalfFOV, 1.f, 1.f, 10.f, FarClip);
	Slot.ViewMatrix = FMatrix44f(View);
	Slot.ViewProjMatrix = FMatrix44f(View * Proj);

	if (ShouldRecapture(Slot, LightPos, LightDir.Rotation()))
	{
		Slot.Capture2D->CaptureScene();
	}

	Slot.bAssigned = true;
	Slot.bIsCube = false;
	Slot.bIsDirectional = false;
	Slot.LightWorldPos = LightPos;
}

void UNanoGSShadowManagerSubsystem::BindPoint(FNanoGSShadowCapture& Slot, ULightComponent* Light)
{
	SyncShowOnlyComponents(Slot);

	UPointLightComponent* Point = CastChecked<UPointLightComponent>(Light);
	const FVector LightPos = Point->GetComponentLocation();
	Slot.CaptureCube->SetWorldLocation(LightPos);
	Slot.CaptureCube->MaxViewDistanceOverride = FMath::Max(Point->AttenuationRadius, 100.f);
	Slot.CaptureCube->SetVisibility(true);

	if (ShouldRecapture(Slot, LightPos, FRotator::ZeroRotator))
	{
		Slot.CaptureCube->CaptureScene();
	}

	// Cube capture has no single view/projection — sampled by direction in the shader instead.
	Slot.bAssigned = true;
	Slot.bIsCube = true;
	Slot.bIsDirectional = false;
	Slot.LightWorldPos = LightPos;
}
