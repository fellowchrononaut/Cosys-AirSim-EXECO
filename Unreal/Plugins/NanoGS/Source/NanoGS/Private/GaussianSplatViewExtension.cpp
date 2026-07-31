// Copyright Epic Games, Inc. All Rights Reserved.

#include "GaussianSplatViewExtension.h"
#include "GaussianSplatSceneProxy.h"
#include "RenderGraphBuilder.h"
#include "NanoGSShadowManagerSubsystem.h"
#include "Engine/World.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/TextureRenderTargetCube.h"
#include "SceneInterface.h"

FGaussianSplatViewExtension* FGaussianSplatViewExtension::Instance = nullptr;

FGaussianSplatViewExtension::FGaussianSplatViewExtension(const FAutoRegister& AutoRegister)
	: FSceneViewExtensionBase(AutoRegister)
{
	Instance = this;
}

FGaussianSplatViewExtension::~FGaussianSplatViewExtension()
{
	if (Instance == this)
	{
		Instance = nullptr;
	}
}

FGaussianSplatViewExtension* FGaussianSplatViewExtension::Get()
{
	return Instance;
}

void FGaussianSplatViewExtension::GetRegisteredProxies(TArray<FGaussianSplatSceneProxy*>& OutProxies) const
{
	FScopeLock Lock(&ProxyLock);
	OutProxies = RegisteredProxies;
}

bool FGaussianSplatViewExtension::IsActiveThisFrame_Internal(const FSceneViewExtensionContext& Context) const
{
	FScopeLock Lock(&ProxyLock);
	return RegisteredProxies.Num() > 0;
}

void FGaussianSplatViewExtension::SetupViewFamily(FSceneViewFamily& InViewFamily)
{
}

void FGaussianSplatViewExtension::BeginRenderViewFamily(FSceneViewFamily& InViewFamily)
{
	// Game thread: snapshot gs.ShadowMode=ProxyMesh's active captures into render-thread-safe data
	// (no UObject pointers) before render-thread work for this frame begins. GatherSceneLighting
	// (NanoGS.cpp) reads this snapshot via GetShadowCaptureSnapshot().
	TArray<FNanoGSShadowRenderData> NewSnapshot;
	UWorld* World = InViewFamily.Scene ? InViewFamily.Scene->GetWorld() : nullptr;
	if (World)
	{
		if (UNanoGSShadowManagerSubsystem* Manager = World->GetSubsystem<UNanoGSShadowManagerSubsystem>())
		{
			TArray<FNanoGSShadowCapture> Active;
			Manager->GetActiveCaptures(Active);
			for (const FNanoGSShadowCapture& C : Active)
			{
				FNanoGSShadowRenderData RD;
				RD.bIsCube = C.bIsCube;
				RD.bIsDirectional = C.bIsDirectional;
				RD.LightWorldPos = C.LightWorldPos;
				RD.LightDirection = C.LightDirection;
				RD.ViewMatrix = C.ViewMatrix;
				RD.ViewProjMatrix = C.ViewProjMatrix;

				if (C.bIsCube && C.RenderTargetCube)
				{
					if (FTextureRenderTargetResource* Res = C.RenderTargetCube->GameThread_GetRenderTargetResource())
					{
						RD.DepthTexture = Res->TextureRHI;
					}
				}
				else if (!C.bIsCube && C.RenderTarget2D)
				{
					if (FTextureRenderTargetResource* Res = C.RenderTarget2D->GameThread_GetRenderTargetResource())
					{
						RD.DepthTexture = Res->TextureRHI;
					}
				}

				if (RD.DepthTexture.IsValid())
				{
					NewSnapshot.Add(RD);
				}
			}
		}
	}

	FScopeLock Lock(&ShadowCaptureLock);
	ShadowCaptureSnapshot = MoveTemp(NewSnapshot);
}

void FGaussianSplatViewExtension::GetShadowCaptureSnapshot(TArray<FNanoGSShadowRenderData>& OutCaptures) const
{
	FScopeLock Lock(&ShadowCaptureLock);
	OutCaptures = ShadowCaptureSnapshot;
}

void FGaussianSplatViewExtension::RegisterProxy(FGaussianSplatSceneProxy* Proxy)
{
	if (Proxy)
	{
		FScopeLock Lock(&ProxyLock);
		RegisteredProxies.AddUnique(Proxy);
	}
}

void FGaussianSplatViewExtension::UnregisterProxy(FGaussianSplatSceneProxy* Proxy)
{
	if (Proxy)
	{
		FScopeLock Lock(&ProxyLock);
		RegisteredProxies.Remove(Proxy);
	}
}

void FGaussianSplatViewExtension::PreRenderView_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& InView)
{
}

void FGaussianSplatViewExtension::PreRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily)
{
}

void FGaussianSplatViewExtension::PostRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily)
{
}

void FGaussianSplatViewExtension::PostRenderBasePassDeferred_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& InView, const FRenderTargetBindingSlots& RenderTargets, TRDGUniformBufferRef<FSceneTextureUniformParameters> SceneTextures)
{
	// Rendering is handled by PostOpaqueRenderDelegate in FGaussianSplattingModule
	// This hook is too early in the pipeline (before lighting) for Gaussian splats
}

void FGaussianSplatViewExtension::PrePostProcessPass_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView& View, const FPostProcessingInputs& Inputs)
{
}

