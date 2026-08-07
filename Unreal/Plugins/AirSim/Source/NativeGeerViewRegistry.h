// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "CoreMinimal.h"
#include "CubeResample.h"

class FRenderTarget;

// Immutable render-thread snapshot for one Scene output explicitly configured with
// CameraModel.RenderBackend = NativeGEER. The FSceneView supplies the live camera transform;
// copying an actor transform at settings time would become stale as soon as the vehicle moves.
struct AIRSIM_API FAirSimNativeGeerView
{
    FString camera_name;
    FIntPoint output_extent = FIntPoint::ZeroValue;
    FAirSimRaymapResourcePtr raymap;
    uint64 registration_serial = 0;
    bool central = true;
};

// Game-thread entry points. Both enqueue their mutation so the registry and its TSharedPtr
// lifetimes are owned entirely by the render thread. output_target is an identity key only; the
// queued lambdas never dereference it.
AIRSIM_API void AirSimRegisterNativeGeerView(FRenderTarget* output_target,
                                             FString camera_name,
                                             FIntPoint output_extent,
                                             const FAirSimRaymapResourcePtr& raymap,
                                             bool central);
AIRSIM_API void AirSimUnregisterNativeGeerView(FRenderTarget* output_target);

// Render-thread lookup used by NanoGS. Exact FSceneViewFamily::RenderTarget identity is the Gate-A
// hypothesis; a cube-face or ordinary pinhole target cannot match unless AirSim registered it.
AIRSIM_API bool AirSimFindNativeGeerView_RenderThread(const FRenderTarget* output_target,
                                                      FAirSimNativeGeerView& out_view);
