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
    FShaderResourceViewRHIRef inverse_direction_srv;
    FIntPoint inverse_direction_grid = FIntPoint::ZeroValue;
    FVector3f common_origin_camera_cm = FVector3f::ZeroVector;
    // Smallest optical-axis component over the raymap's valid unit directions. NanoGS uses it to
    // pick the near-cull criterion automatically: a lens whose rays all point forward can use the
    // reference's axial (view-space z) test, while one that sees past 90 deg must use the
    // Euclidean one, because for those rays axial distance is no longer a distance at all.
    float min_ray_forward_component = 1.0f;
    uint64 registration_serial = 0;
    bool central = true;
    bool inverse_direction_ready = false;
};

// Game-thread entry points. Both enqueue their mutation so the registry and its TSharedPtr
// lifetimes are owned entirely by the render thread. output_target is an identity key only; the
// queued lambdas never dereference it.
AIRSIM_API void AirSimRegisterNativeGeerView(FRenderTarget* output_target,
                                             FString camera_name,
                                             FIntPoint output_extent,
                                             const FAirSimRaymapResourcePtr& raymap,
                                             bool central,
                                             float min_ray_forward_component);
AIRSIM_API void AirSimUnregisterNativeGeerView(FRenderTarget* output_target);

// Render-thread lookup used by NanoGS. Exact FSceneViewFamily::RenderTarget identity is the Gate-A
// hypothesis; a cube-face or ordinary pinhole target cannot match unless AirSim registered it.
AIRSIM_API bool AirSimFindNativeGeerView_RenderThread(const FRenderTarget* output_target,
                                                      FAirSimNativeGeerView& out_view);
