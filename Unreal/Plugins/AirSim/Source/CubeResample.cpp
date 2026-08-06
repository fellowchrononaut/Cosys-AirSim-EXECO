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

/** Phase 3b step 5. Which way the DepthRange mode resolves a 4-tap gather that straddles a depth
 *  discontinuity. Bilinear across a silhouette invents a surface that is in NEITHER tap - a ramp
 *  of geometry hanging in space at every object edge - so the built-in rule blends only while the
 *  four taps agree and falls back to the nearest tap when they do not.
 *
 *  This is a DIAGNOSTIC, not a data knob: it makes the choice measurable instead of arguable. If
 *  the threshold ever has to differ per dataset it moves into settings.json, which is archived
 *  with the data, not here. */
static TAutoConsoleVariable<int32> CVarAirSimCubeResampleDepthFilter(
    TEXT("airsim.CubeResampleDepthFilter"),
    -1,
    TEXT("Phase 3b: depth 4-tap resolution rule.\n")
    TEXT(" -1: follow the built-in rule (default) - blend unless max/min exceeds the edge ratio\n")
    TEXT("  0: nearest tap always (no interpolation anywhere)\n")
    TEXT("  1: blend always (the naive filter; leaves ramps at every silhouette)"),
    ECVF_RenderThreadSafe);

/** Phase 3b step 5. The max/min ratio across the four taps above which they are treated as
 *  straddling a discontinuity rather than sampling one surface.
 *
 *  Why 1.05. One texel of a 640-square face spans (pi/2)/640 = 0.14 degrees at the face centre,
 *  which is where the angular density is COARSEST (design section 6). A single surface at
 *  incidence i changes range across one texel by about 1 + dtheta * tan(i): 1.4 percent at 80
 *  degrees, 2.8 percent at 85. At the production rig's 320-square faces those double. So 1.05
 *  passes every genuine surface out to ~87 degrees of incidence at 640 and ~84 at 320.
 *  A real silhouette, meanwhile, is a FACTOR and not a percent - two objects that are visibly
 *  separated differ by 1.3x at the very least and usually more than 2x. 1.05 therefore sits about
 *  2x above the worst legitimate slope and 6x below the cheapest real edge.
 *
 *  Diagnostic only, for sweeping the choice during validation. Not a per-dataset knob. */
static TAutoConsoleVariable<float> CVarAirSimCubeResampleDepthEdgeRatio(
    TEXT("airsim.CubeResampleDepthEdgeRatio"),
    1.05f,
    TEXT("Phase 3b: depth 4-tap max/min ratio above which the taps are treated as an edge.\n")
    TEXT(" 1.05: default. Values below 1 are clamped to 1 (= nearest everywhere)"),
    ECVF_RenderThreadSafe);

/** Phase 3b step 5, section 3.3. Whether the face captures' depth is PLANAR and has to be turned
 *  into range along the ray, per tap, before it is filtered.
 *
 *  The built-in rule is 0 (no conversion), because DepthPerspectiveMaterial emits the LENGTH of
 *  the camera-relative world position - range from the camera origin - and step 5 routes the
 *  DepthPlanar face captures through that same material (APIPCamera::ensureFaceRig). All six
 *  faces share one origin, so those values are already commensurable across faces and already the
 *  quantity design section 5 O3 asks for.
 *
 *  1 applies range = planar * sqrt(1 + a*a + b*b) per tap. It exists so the material question is
 *  settled by ONE RUN rather than by a rebuild: if a flat wall reads constant across a face
 *  instead of rising as 1/cos(theta), the faces are planar and this is the correction. */
static TAutoConsoleVariable<int32> CVarAirSimCubeResampleDepthPlanarToRange(
    TEXT("airsim.CubeResampleDepthPlanarToRange"),
    -1,
    TEXT("Phase 3b: treat cube-face depth as planar and convert it to range along the ray.\n")
    TEXT(" -1: follow the built-in rule (default) - the faces already carry range\n")
    TEXT("  0: same as -1\n")
    TEXT("  1: convert per tap with sqrt(1 + a*a + b*b) before filtering"),
    ECVF_RenderThreadSafe);

/** Phase 3b step 5, section 3.4. Whether SurfaceNormals face values are in a per-FACE frame and
 *  must be rotated into the camera frame before they are blended.
 *
 *  The built-in rule is 0 (already a common frame). NormalsMaterial reads the scene texture
 *  PPI_WorldNormal, which is the GBuffer's WORLD-space normal, so every face encodes the same
 *  vector for the same surface and only the renormalisation after blending matters.
 *
 *  1 rotates each tap by its face's (F, R, U) basis, assuming the encoding is (R, U, F). That
 *  convention is UNVERIFIED - it only becomes relevant if the measurement in the step 5 brief
 *  finds that two faces disagree about one flat surface, and it must be re-derived if so. */
static TAutoConsoleVariable<int32> CVarAirSimCubeResampleNormalsFrame(
    TEXT("airsim.CubeResampleNormalsFrame"),
    -1,
    TEXT("Phase 3b: rotate SurfaceNormals taps out of face space before blending.\n")
    TEXT(" -1: follow the built-in rule (default) - normals are world space, no rotation\n")
    TEXT("  0: same as -1\n")
    TEXT("  1: rotate each tap by its face basis (unverified encoding; see CubeResample.cpp)"),
    ECVF_RenderThreadSafe);

// -------------------------------------------------------------------------------------------
// The mode table. THE single source of truth (step 5 section 3.1).
//
// The Scene-only restriction of step 4 lived in two places - the ImageType test in
// UnrealImageCapture and the Annotation early return in APIPCamera::ensureFaceRig - and adding
// four modalities by editing both is how they drift apart. Everything a reader needs to know
// about what is supported and why is in this one function.
//
// Unsupported means EXACTLY what it meant before step 5: the request falls through to the
// ordinary pinhole render. It is not an error and it is not a black frame.
// -------------------------------------------------------------------------------------------
EAirSimCubeResampleMode AirSimCubeResampleModeForImageType(int32 image_type)
{
    switch (image_type) {
    case 0: //Scene
        //shipped in step 4, and its generated shader code must not change - the permutation
        //exists so that this is checkable rather than asserted
        return EAirSimCubeResampleMode::Bilinear;

    case 1: //DepthPlanar
        //design section 5 O3: non-pinhole depth is RANGE ALONG THE RAY. DepthPlanar has no
        //meaning without an image plane, so its FACE captures are routed through the perspective
        //material (ensureFaceRig) and it returns the same image DepthPerspective does
        return EAirSimCubeResampleMode::DepthRange;

    case 2: //DepthPerspective
        //already range-like, so nothing to convert. Deliberately the same output as DepthPlanar
        //for a generic camera; see the note to step 6 in the handoff
        return EAirSimCubeResampleMode::DepthRange;

    case 3: //DepthVis
        //a visualisation material applied to FACE depth: black-to-white over 0-100 m in the
        //face's own frame. The mapping is not invertible, so the face's colour cannot be
        //corrected into the output camera's after the fact. Future work
        return EAirSimCubeResampleMode::Unsupported;

    case 4: //DisparityNormalized
        //disparity is f*b/Z. A fisheye has no f, so there is nothing to normalise against
        return EAirSimCubeResampleMode::Unsupported;

    case 5: //Segmentation
        //an ID buffer - Cosys-AirSim renders it as flat per-instance colours through the
        //annotation show flags, so bilinear does not blur an edge, it INVENTS AN INSTANCE that
        //is not in the scene
        return EAirSimCubeResampleMode::Nearest;

    case 6: //SurfaceNormals
        //filter, then renormalise: the mean of two unit vectors is not a unit vector. Frame
        //handling per airsim.CubeResampleNormalsFrame above
        return EAirSimCubeResampleMode::Normals;

    case 7: //Infrared
        //InfraredMaterial is PPI_CustomStencil divided down, so this is an ID-derived buffer and
        //morally Nearest - but that is read off the material, not measured. Promote this row to
        //Nearest only once the step 5 ID-exactness test has been run for Infrared too
        return EAirSimCubeResampleMode::Unsupported;

    case 8:  //OpticalFlow
    case 9:  //OpticalFlowVis
        //design section 5: flow is defined in image space and the image space is exactly what
        //changed. Resampled per-face pinhole flow is not fisheye flow, and a buffer that looks
        //right and is wrong is worse than no buffer
        return EAirSimCubeResampleMode::Unsupported;

    case 10: //Lighting
        //behaves as Scene. Cheap; take it
        return EAirSimCubeResampleMode::Bilinear;

    case 11: //Annotation
        //captures_ is indexed BY ANNOTATION NAME for this type, not by ImageType, so the face
        //rig's image_type * 6 + face indexing does not address it at all. That is a different
        //rig-indexing problem, and it is why APIPCamera::ensureFaceRig keeps its early return
        return EAirSimCubeResampleMode::Unsupported;

    default:
        return EAirSimCubeResampleMode::Unsupported;
    }
}

class FAirSimCubeResamplePS : public FGlobalShader
{
    DECLARE_GLOBAL_SHADER(FAirSimCubeResamplePS);
    SHADER_USE_PARAMETER_STRUCT(FAirSimCubeResamplePS, FGlobalShader);

    // Step 5 section 3.2: a PERMUTATION over the mode, not a uniform branch. Two reasons, and the
    // second is the binding one. (1) Per-pixel uniform branching over four filters costs
    // registers in every mode, including Scene. (2) It lets us state that the Scene shader is the
    // step 4 shader: RESAMPLE_MODE 0 excludes every line step 5 added, by the preprocessor, so
    // the claim is checkable with r.DumpShaderDebugInfo rather than asserted.
    class FModeDim : SHADER_PERMUTATION_INT("RESAMPLE_MODE", 4);
    using FPermutationDomain = TShaderPermutationDomain<FModeDim>;

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
        //Below here: referenced only by the modes that need them, so they are absent from the
        //Scene permutation's parameter map and are simply ignored when it is bound. Filling them
        //unconditionally keeps one code path in AirSimCubeResample_RenderThread.
        SHADER_PARAMETER(FIntPoint, FaceSize)
        SHADER_PARAMETER(uint32, DepthPlanarToRange)
        SHADER_PARAMETER(uint32, DepthFilterMode)
        SHADER_PARAMETER(float, DepthEdgeRatio)
        SHADER_PARAMETER(uint32, NormalsFaceSpace)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

IMPLEMENT_GLOBAL_SHADER(FAirSimCubeResamplePS, "/Plugin/AirSim/Private/CubeResample.usf", "MainPS", SF_Pixel);

//RESAMPLE_MODE is the enum value minus one, so the enum's order IS the shader's. Both ends of
//that are load-bearing and neither is obvious from the other file.
static_assert(static_cast<int32>(EAirSimCubeResampleMode::Bilinear) == 1 &&
                  static_cast<int32>(EAirSimCubeResampleMode::Nearest) == 2 &&
                  static_cast<int32>(EAirSimCubeResampleMode::DepthRange) == 3 &&
                  static_cast<int32>(EAirSimCubeResampleMode::Normals) == 4,
              "EAirSimCubeResampleMode order defines RESAMPLE_MODE in CubeResample.usf");

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
                                     const FAirSimRaymapResource& raymap,
                                     EAirSimCubeResampleMode mode)
{
    if (face_textures == nullptr || output_texture == nullptr || face_count <= 0)
        return false;
    if (!raymap.ready || !raymap.srv.IsValid() || raymap.width == 0 || raymap.height == 0)
        return false;
    //UnrealImageCapture never fills faces for an unsupported type, so this is belt and braces -
    //but declining here leaves the target untouched, which for an unsupported type would be a
    //stale image rather than a pinhole one. Keep the two ends in agreement.
    if (mode == EAirSimCubeResampleMode::Unsupported)
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

    FAirSimCubeResamplePS::FPermutationDomain permutation;
    permutation.Set<FAirSimCubeResamplePS::FModeDim>(static_cast<int32>(mode) - 1);

    TShaderMapRef<FAirSimCubeResamplePS> pixel_shader(GetGlobalShaderMap(GMaxRHIFeatureLevel), permutation);
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
        //The sampler follows the mode. Only Bilinear wants hardware filtering; the other three
        //do their filtering explicitly in the shader, from point taps, because each of them has
        //to look at the four values BEFORE deciding what to do with them.
        parameters.FaceSampler = mode == EAirSimCubeResampleMode::Bilinear
                                     ? TStaticSamplerState<SF_Bilinear, AM_Clamp, AM_Clamp, AM_Clamp>::GetRHI()
                                     : TStaticSamplerState<SF_Point, AM_Clamp, AM_Clamp, AM_Clamp>::GetRHI();
        parameters.RaymapSize = FIntPoint(static_cast<int32>(raymap.width), static_cast<int32>(raymap.height));
        parameters.FaceCount = static_cast<uint32>(bound_faces);

        //All faces are one square target of getCubeFaceResolution() (ensureFaceRig), so face 0's
        //extent is every face's. Read from the RHI rather than re-derived, so it cannot drift.
        parameters.FaceSize = face_textures[0]->GetDesc().Extent;

        //Diagnostics, all on the project's -1 follow / 0 / 1 override shape: -1 is the built-in
        //rule and changes nothing.
        const int32 planar_to_range = CVarAirSimCubeResampleDepthPlanarToRange.GetValueOnRenderThread();
        parameters.DepthPlanarToRange = planar_to_range > 0 ? 1u : 0u;

        //shader semantics: 0 nearest, 1 blend, 2 the rule. -1 and anything unrecognised are the rule.
        const int32 depth_filter = CVarAirSimCubeResampleDepthFilter.GetValueOnRenderThread();
        parameters.DepthFilterMode = depth_filter == 0 ? 0u : (depth_filter == 1 ? 1u : 2u);

        //below 1 the test max > min * ratio is true everywhere, which is nearest everywhere - a
        //legal arm of the sweep, but clamp so a negative value cannot invert the test
        parameters.DepthEdgeRatio = FMath::Max(1.0f, CVarAirSimCubeResampleDepthEdgeRatio.GetValueOnRenderThread());

        const int32 normals_frame = CVarAirSimCubeResampleNormalsFrame.GetValueOnRenderThread();
        parameters.NormalsFaceSpace = normals_frame > 0 ? 1u : 0u;

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
