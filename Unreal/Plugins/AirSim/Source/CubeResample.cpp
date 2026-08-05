// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Generic (non-pinhole) camera resample - Phase 3b step 4. See CubeResample.h for what this is
// and CubeResample.usf for the geometry.
//
// WHY A PIXEL SHADER AND NOT A COMPUTE SHADER
// The output is the EXISTING render_targets_[image_type] (finding F2), so that ReadSurfaceData
// and everything downstream of it - compression, image_data_uint8, RPC, ROS - needs no change at
// all. That target is created by UTextureRenderTarget2D::InitCustomFormat and is a render target,
// not a UAV: PF_B8G8R8A8 as a storage image is not guaranteed on Vulkan, and adding
// bCanCreateUAV would change how the PINHOLE path's targets are created. A raster pass binds the
// target the way it was already made to be bound, so nothing about the pinhole target changes.
//
// WHY A STRUCTURED BUFFER AND NOT TWO FLOAT4 TEXTURES
// The raymap is consumed one texel per output pixel, never filtered - output pixel (x, y) reads
// raymap texel (x, y) and nothing else. A texture buys sampler hardware we would have to switch
// off, and either wastes two floats per texel on padding (two RGBA32F textures store eight
// floats where ADR-001 asks for six) or splits the six channels across formats. A structured
// buffer of float stores exactly six per texel in exactly ADR-001's order, which is also the
// order of the AIRRAYM1 dump tools/raymap_check.py validates, so what the GPU reads is what the
// offline check checked.
//
// Cost: 6 floats * 4 bytes = 24 bytes per output pixel. 1280x1024 is 31.5 MB per camera. That is
// real, and it is the price ADR-001 names: the origin channels are constant across every v1
// model. If the fleet-scaling benchmark ever charges for it, ADR-001's own invalidation clause
// says keep the format and add a bCentralCamera fast path - the shader already skips the origin
// FETCH, so the remaining saving is the 12 bytes of storage, and that is a separate decision.

#include "CubeResample.h"

#include "CommonRenderResources.h"
#include "GlobalShader.h"
#include "HAL/IConsoleManager.h"
#include "PipelineStateCache.h"
#include "PixelShaderUtils.h"
#include "RHICommandList.h"
#include "RHIResources.h"
#include "RHIStaticStates.h"
#include "RenderingThread.h"
#include "ShaderParameterStruct.h"

/** Phase 3b step 4. Skips the resample pass while still rendering the cube faces, so the pass'
 *  own cost is the difference between the two arms of one A/B run rather than a number read off
 *  a profiler. Compare the c+d segment of airsim.LogImageTiming with this at 1 and at 0.
 *
 *  At 0 the output image is whatever was last in the target and is NOT meaningful - this is a
 *  timing switch, not a fallback. It has no effect at all on a camera with no CameraModel block,
 *  because such a camera never reaches this file. */
static TAutoConsoleVariable<int32> CVarAirSimCubeResample(
    TEXT("airsim.CubeResample"),
    1,
    TEXT("Phase 3b: run the cube -> generic camera resample pass.\n")
    TEXT(" 0: skip the pass (faces still render; output image is undefined - timing arm only)\n")
    TEXT(" 1: resample (default)"),
    ECVF_RenderThreadSafe);

class FAirSimCubeResamplePS : public FGlobalShader
{
    DECLARE_GLOBAL_SHADER(FAirSimCubeResamplePS);
    SHADER_USE_PARAMETER_STRUCT(FAirSimCubeResamplePS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_SRV(StructuredBuffer<float>, Raymap)
        SHADER_PARAMETER_TEXTURE(Texture2D, FaceTexture0)
        SHADER_PARAMETER_TEXTURE(Texture2D, FaceTexture1)
        SHADER_PARAMETER_TEXTURE(Texture2D, FaceTexture2)
        SHADER_PARAMETER_TEXTURE(Texture2D, FaceTexture3)
        SHADER_PARAMETER_TEXTURE(Texture2D, FaceTexture4)
        SHADER_PARAMETER_TEXTURE(Texture2D, FaceTexture5)
        SHADER_PARAMETER_SAMPLER(SamplerState, FaceSampler)
        SHADER_PARAMETER(FIntPoint, RaymapSize)
        SHADER_PARAMETER(uint32, FaceCount)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

IMPLEMENT_GLOBAL_SHADER(FAirSimCubeResamplePS, "/Plugin/AirSim/Private/CubeResample.usf", "MainPS", SF_Pixel);

FAirSimRaymapResourcePtr AirSimCreateRaymapResource()
{
    return MakeShared<FAirSimRaymapResource, ESPMode::ThreadSafe>();
}

void AirSimUploadRaymap(const FAirSimRaymapResourcePtr& resource, TArray<float> values,
                        uint32 width, uint32 height, bool central)
{
    if (!resource.IsValid() || values.Num() == 0)
        return;

    ENQUEUE_RENDER_COMMAND(AirSimUploadRaymap)
    (
        [resource, values, width, height, central](FRHICommandListImmediate& cmd_list) {
            const uint32 stride = static_cast<uint32>(sizeof(float));
            const uint32 bytes = static_cast<uint32>(values.Num()) * stride;

            FRHIBufferCreateDesc desc = FRHIBufferCreateDesc::Create(
                                            TEXT("AirSimCameraRaymap"),
                                            bytes,
                                            stride,
                                            BUF_Static | BUF_ShaderResource | BUF_StructuredBuffer)
                                            .SetInitialState(ERHIAccess::SRVMask);
            resource->buffer = cmd_list.CreateBuffer(desc);

            void* dest = cmd_list.LockBuffer(resource->buffer, 0, bytes, RLM_WriteOnly);
            FMemory::Memcpy(dest, values.GetData(), bytes);
            cmd_list.UnlockBuffer(resource->buffer);

            resource->srv = cmd_list.CreateShaderResourceView(
                resource->buffer,
                FRHIViewDesc::CreateBufferSRV()
                    .SetType(FRHIViewDesc::EBufferType::Structured)
                    .SetStride(stride));

            resource->width = width;
            resource->height = height;
            resource->central = central;
            resource->ready = true;
        });
}

void AirSimReleaseRaymap(FAirSimRaymapResourcePtr& resource)
{
    if (!resource.IsValid())
        return;

    //the render command keeps the last reference alive until it runs, so the RHI objects are
    //released on the render thread and never from under a pass that is still using them
    FAirSimRaymapResourcePtr released = resource;
    resource.Reset();

    ENQUEUE_RENDER_COMMAND(AirSimReleaseRaymap)
    (
        [released](FRHICommandListImmediate&) {
            released->ready = false;
            released->srv.SafeRelease();
            released->buffer.SafeRelease();
        });
}

bool AirSimCubeResampleEnabled()
{
    return CVarAirSimCubeResample.GetValueOnRenderThread() != 0;
}

bool AirSimCubeResample_RenderThread(FRHICommandListImmediate& cmd_list,
                                     FRHITexture* const* face_textures, int32 face_count,
                                     FRHITexture* output_texture,
                                     const FAirSimRaymapResource& raymap)
{
    if (face_textures == nullptr || output_texture == nullptr || face_count <= 0)
        return false;
    if (!raymap.ready || !raymap.srv.IsValid() || raymap.width == 0 || raymap.height == 0)
        return false;

    const int32 bound_faces = FMath::Min(face_count, kAirSimCubeResampleMaxFaces);

    //a five-face rig has no Back face. Bind face 0 in its slot so every declared parameter has a
    //valid resource - SHADER_USE_PARAMETER_STRUCT binds everything - and let the shader's
    //Face >= FaceCount test make sure it is never sampled.
    FRHITexture* bound[kAirSimCubeResampleMaxFaces] = {};
    for (int32 face = 0; face < kAirSimCubeResampleMaxFaces; ++face) {
        FRHITexture* texture = face < bound_faces ? face_textures[face] : nullptr;
        bound[face] = texture != nullptr ? texture : face_textures[0];
        if (bound[face] == nullptr)
            return false;
    }

    TShaderMapRef<FAirSimCubeResamplePS> pixel_shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
    if (!pixel_shader.IsValid())
        return false;

    const FIntPoint output_size = output_texture->GetDesc().Extent;
    if (output_size.X <= 0 || output_size.Y <= 0)
        return false;

    for (int32 face = 0; face < bound_faces; ++face) {
        cmd_list.Transition(FRHITransitionInfo(face_textures[face], ERHIAccess::Unknown, ERHIAccess::SRVGraphics));
    }
    cmd_list.Transition(FRHITransitionInfo(output_texture, ERHIAccess::Unknown, ERHIAccess::RTV));

    //DontLoad because the pass writes every pixel of the target: a full-screen triangle over the
    //whole viewport, with no discard and no blending.
    FRHIRenderPassInfo pass_info(output_texture, ERenderTargetActions::DontLoad_Store);
    cmd_list.BeginRenderPass(pass_info, TEXT("AirSimCubeResample"));
    {
        FAirSimCubeResamplePS::FParameters parameters;
        parameters.Raymap = raymap.srv;
        parameters.FaceTexture0 = bound[0];
        parameters.FaceTexture1 = bound[1];
        parameters.FaceTexture2 = bound[2];
        parameters.FaceTexture3 = bound[3];
        parameters.FaceTexture4 = bound[4];
        parameters.FaceTexture5 = bound[5];
        //bilinear is the Scene rule of design section 5. Segmentation and Annotation need
        //nearest and depth needs converting before it is filtered at all, which is why step 4
        //resamples Scene only.
        parameters.FaceSampler = TStaticSamplerState<SF_Bilinear, AM_Clamp, AM_Clamp, AM_Clamp>::GetRHI();
        parameters.RaymapSize = FIntPoint(static_cast<int32>(raymap.width), static_cast<int32>(raymap.height));
        parameters.FaceCount = static_cast<uint32>(bound_faces);

        FPixelShaderUtils::DrawFullscreenPixelShader(
            cmd_list,
            GetGlobalShaderMap(GMaxRHIFeatureLevel),
            pixel_shader,
            parameters,
            FIntRect(0, 0, output_size.X, output_size.Y));
    }
    cmd_list.EndRenderPass();

    //leave the target in the state a scene capture would have left it in, so the readback that
    //follows sees nothing unusual
    cmd_list.Transition(FRHITransitionInfo(output_texture, ERHIAccess::RTV, ERHIAccess::SRVMask));
    return true;
}
