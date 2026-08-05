// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

// Generic (non-pinhole) camera resample - Phase 3b step 4. The GPU side of the camera model:
// a camera's raymap as a GPU resource, and the one full-screen pass that turns six pinhole cube
// faces into one image of the calibrated camera.
//
// Everything here is inert for a camera with no CameraModel block in settings.json. Nothing
// allocates, nothing uploads, and RenderRequest never reaches the dispatch, because a pinhole
// request carries no faces. That is the non-invasiveness contract, enforced by data rather than
// by a flag: RenderParams::face_targets is empty and the loop that would resample skips it.
//
// The shader class itself is private to CubeResample.cpp so that this header stays cheap for
// RenderRequest.h to include - it pulls in no shader or render-graph headers.

#include "CoreMinimal.h"
#include "RHIResources.h"
#include "Templates/SharedPointer.h"

class FRHICommandListImmediate;

// Six faces, and the shader has six texture bindings to match - a dynamic index into a texture
// array is not portable and the face is chosen per pixel. Keep in step with SampleCubeFace in
// Shaders/Private/CubeResample.usf.
static constexpr int32 kAirSimCubeResampleMaxFaces = 6;

// One camera's raymap on the GPU.
//
// FORMAT: a structured buffer of float, stride 4 bytes, SIX consecutive floats per texel, laid
// out row major (y outer, x inner): ox, oy, oz, dx, dy, dz. That is ADR-001's representation
// verbatim - the same six channels, in the same order, as the AIRRAYM1 dump format that
// tools/raymap_check.py validates - so a raymap that checks out offline is byte-for-byte what
// the GPU reads. See the format rationale in CubeResample.cpp.
//
// The values are in the UNREAL CAMERA frame and in centimetres, not the calibration's optical
// frame: the axis change is applied once, on the CPU, in APIPCamera::buildRaymapResource.
struct FAirSimRaymapResource
{
    FBufferRHIRef buffer;
    FShaderResourceViewRHIRef srv;
    uint32 width = 0;
    uint32 height = 0;
    //all origins identical. A fetch hint for the shader; never a change of format (ADR-001).
    bool central = true;
    //written on the render thread by the upload command, read on the render thread by the
    //resample. No barrier is needed: the upload is enqueued at camera configuration time and
    //every ExecuteTask is enqueued later, so the render thread runs them in that order.
    bool ready = false;
};

using FAirSimRaymapResourcePtr = TSharedPtr<FAirSimRaymapResource, ESPMode::ThreadSafe>;

// Allocates the (empty) resource on the calling thread. Cheap; no RHI object is created yet.
FAirSimRaymapResourcePtr AirSimCreateRaymapResource();

// Enqueues the upload. values is width * height * 6 floats, already in the Unreal camera frame.
void AirSimUploadRaymap(const FAirSimRaymapResourcePtr& resource, TArray<float> values,
                        uint32 width, uint32 height, bool central);

// Enqueues the release and clears the caller's reference.
void AirSimReleaseRaymap(FAirSimRaymapResourcePtr& resource);

// airsim.CubeResample - 0 skips the pass so its cost can be measured by difference. Render
// thread only.
bool AirSimCubeResampleEnabled();

// The pass. Render thread only, and it must run after the face captures have rendered and
// before the output target is read back - which is exactly where ExecuteTask sits (finding F3).
// Returns false if it declined to run, in which case the output target is left untouched.
bool AirSimCubeResample_RenderThread(FRHICommandListImmediate& cmd_list,
                                     FRHITexture* const* face_textures, int32 face_count,
                                     FRHITexture* output_texture,
                                     const FAirSimRaymapResource& raymap);
