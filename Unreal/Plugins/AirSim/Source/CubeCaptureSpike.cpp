// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

/** ⚠ THROWAWAY PROBE — Phase 3b step 2, open question C1. NOT the design. ⚠
 *
 *  This file exists to answer one question by measurement: does a USceneCaptureComponentCube see
 *  NanoGS Gaussian splats? Reading the code says it should (see the notes below), but on this
 *  project code-reading has been wrong in both directions and every measured item landed on
 *  prediction, so the answer has to be measured. Step 3 builds the real rig; this file is then
 *  deleted or folded into it. Do not build on it, do not extend it, do not copy its patterns.
 *
 *  It is a single self-contained .cpp with no header and no call sites: nothing else in the plugin
 *  references it. At airsim.CubeCaptureSpike 0 (the default) not one line of it executes, so
 *  behaviour is bit-identical to a build without the file.
 *
 *  WHAT THE CODE READING SAYS, so the measurement has something to disagree with
 *  ----------------------------------------------------------------------------
 *  1. NanoGS does NOT draw splats in FGaussianSplatViewExtension::PrePostProcessPass_RenderThread
 *     — that override is empty (GaussianSplatViewExtension.cpp:141). Splats are drawn from
 *     FNanoGSModule::OnPostOpaqueRender_RenderThread, registered via
 *     RegisterPostOpaqueRenderDelegate (NanoGS.cpp:504).
 *  2. FRendererModule::RenderPostOpaqueExtensions (SceneRendering.cpp:5216) loops over every view
 *     in the family and is called unconditionally from the deferred renderer
 *     (DeferredShadingRenderer.cpp:3217) — it is not gated on capture source or bResolveScene.
 *  3. A cube capture with r.SceneCapture.CubeSinglePass=1 (the default) renders all six faces as
 *     six views of ONE view family into a 3x2 tiled atlas, then copies tile->face slice
 *     (SceneCaptureRendering.cpp:1742+).
 *  => reading predicts splats appear in all six faces, for every capture source, because the draw
 *     happens post-opaque into scene colour, upstream of the post-process chain entirely.
 *  Mode 3 below is the control that separates "post-opaque ran" from "post-process chain ran".
 *
 *  WHY THE READBACK IS FLOAT16 AND THE TARGET IS PF_FloatRGBA
 *  ---------------------------------------------------------
 *  On Vulkan — this project's RHI — FVulkanDynamicRHI::RHIReadSurfaceData (the FColor/uint8 path
 *  AirSim uses for Scene and Segmentation) cannot read a cube face at all:
 *    VulkanRenderTarget.cpp:128  checkf(!IsTextureCube() || CubeFace == CubeFace_MAX, "Cube faces
 *                                not supported yet.")   <- asserts on a real face index
 *    VulkanRenderTarget.cpp:142  ETextureDimension::TextureCube falls to default: -> memzero
 *                                                                  <- silently returns BLACK
 *  Only RHIReadSurfaceFloatData handles cubes (VulkanRenderTarget.cpp:373, baseArrayLayer =
 *  CubeFace + 6*ArrayIndex), and it ensure()s the storage format is VK_FORMAT_R16G16B16A16_SFLOAT.
 *  So a probe that used the uint8 path would read black no matter what rendered, and we would have
 *  "measured" a false negative. Hence PF_FloatRGBA + ReadFloat16Pixels throughout.
 *
 *  USAGE — see the step 2 handoff report for the full recipe.
 *    airsim.CubeCaptureSpike 1   run once with SCS_FinalToneCurveHDR  (AirSim Scene / Lighting)
 *    airsim.CubeCaptureSpike 2   run once with SCS_FinalColorLDR      (all other ImageTypes)
 *    airsim.CubeCaptureSpike 3   run once with SCS_SceneColorHDR      (control, no post-process)
 *  Each run captures from the current player viewpoint and writes
 *  PNG files under Saved/CubeCaptureSpike/ plus one [CubeSpike] log line per face.
 */

#include "CoreMinimal.h"
#include "HAL/IConsoleManager.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SceneCaptureComponentCube.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/TextureRenderTargetCube.h"
#include "TextureResource.h"
#include "UObject/StrongObjectPtr.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "GameFramework/PlayerController.h"
#include "Camera/PlayerCameraManager.h"
#include "PIPCamera.h"

/** Throwaway probe for Phase 3b step 2 / open question C1. Inert at 0, which is the default.
 *   0 = off (default; nothing in this file runs)
 *   1 = one-shot cube+2D capture with SCS_FinalToneCurveHDR (AirSim's Scene / Lighting source)
 *   2 = one-shot cube+2D capture with SCS_FinalColorLDR     (AirSim's source for every other type)
 *   3 = one-shot cube+2D capture with SCS_SceneColorHDR     (control: post-processing disabled) */
static TAutoConsoleVariable<int32> CVarCubeCaptureSpike(
    TEXT("airsim.CubeCaptureSpike"),
    0,
    TEXT("C1 spike: capture one cube + one 2D reference from the player view, then dump and log.\n")
    TEXT(" 0: off (default)\n")
    TEXT(" 1: SCS_FinalToneCurveHDR - what AirSim uses for Scene and Lighting\n")
    TEXT(" 2: SCS_FinalColorLDR     - what AirSim uses for every other ImageType\n")
    TEXT(" 3: SCS_SceneColorHDR     - control, post-processing off"),
    ECVF_Default);

namespace
{
    // Small on purpose: the question is "did splats appear", not "how does it look at 1080p".
    constexpr int32 kFaceSize = 256;

    const TCHAR* CubeFaceName(int32 face)
    {
        switch (face) {
        case CubeFace_PosX: return TEXT("PosX");
        case CubeFace_NegX: return TEXT("NegX");
        case CubeFace_PosY: return TEXT("PosY");
        case CubeFace_NegY: return TEXT("NegY");
        case CubeFace_PosZ: return TEXT("PosZ");
        case CubeFace_NegZ: return TEXT("NegZ");
        default: return TEXT("????");
        }
    }

    ESceneCaptureSource SourceForMode(int32 mode, FString& out_name)
    {
        switch (mode) {
        case 2:  out_name = TEXT("SCS_FinalColorLDR");     return SCS_FinalColorLDR;
        case 3:  out_name = TEXT("SCS_SceneColorHDR");     return SCS_SceneColorHDR;
        default: out_name = TEXT("SCS_FinalToneCurveHDR"); return SCS_FinalToneCurveHDR;
        }
    }

    // Same world lookup the airsim.CaptureEveryFrame diagnostic uses (PIPCamera.cpp).
    UWorld* FindGameWorld()
    {
        if (GEngine == nullptr)
            return nullptr;

        for (const FWorldContext& ctx : GEngine->GetWorldContexts()) {
            if (ctx.World() != nullptr &&
                (ctx.WorldType == EWorldType::Game || ctx.WorldType == EWorldType::PIE)) {
                return ctx.World();
            }
        }
        return nullptr;
    }

    // Capture from wherever the user is looking, so they can fly to the splats first and then set
    // the CVar. Falls back to the first APIPCamera when there is no player camera manager.
    bool FindViewpoint(UWorld* world, FVector& out_location, FRotator& out_rotation, FString& out_from)
    {
        if (APlayerController* pc = world->GetFirstPlayerController()) {
            if (pc->PlayerCameraManager != nullptr) {
                out_location = pc->PlayerCameraManager->GetCameraLocation();
                out_rotation = pc->PlayerCameraManager->GetCameraRotation();
                out_from = TEXT("player camera manager");
                return true;
            }
        }

        for (TActorIterator<APIPCamera> it(world); it; ++it) {
            out_location = it->GetActorLocation();
            out_rotation = it->GetActorRotation();
            out_from = FString::Printf(TEXT("APIPCamera %s"), *it->GetName());
            return true;
        }

        return false;
    }

    struct FFaceStats
    {
        double mean_luma = 0.0;
        float max_luma = 0.0f;
        double nonzero_fraction = 0.0;
        int32 pixels = 0;
    };

    FFaceStats ComputeStats(const TArray<FFloat16Color>& pixels)
    {
        FFaceStats stats;
        stats.pixels = pixels.Num();
        if (stats.pixels == 0)
            return stats;

        double sum = 0.0;
        int64 nonzero = 0;
        for (const FFloat16Color& p : pixels) {
            const float luma = 0.2126f * p.R.GetFloat() + 0.7152f * p.G.GetFloat() + 0.0722f * p.B.GetFloat();
            sum += luma;
            stats.max_luma = FMath::Max(stats.max_luma, luma);
            if (luma > 1e-4f)
                ++nonzero;
        }
        stats.mean_luma = sum / stats.pixels;
        stats.nonzero_fraction = static_cast<double>(nonzero) / stats.pixels;
        return stats;
    }

    // Explicit, documented mapping so the PNG and the logged numbers describe the same thing:
    // clamp linear to [0,1] then sRGB-encode. Blows out HDR highlights; that is fine, the PNG is
    // only ever meant to answer "are the splats there" by eye.
    void WritePng(const TArray<FFloat16Color>& pixels, int32 size, const FString& path)
    {
        if (pixels.Num() < size * size)
            return;

        TArray<FColor> bgra;
        bgra.Reserve(size * size);
        for (int32 i = 0; i < size * size; ++i) {
            const FLinearColor lin(
                FMath::Clamp(pixels[i].R.GetFloat(), 0.0f, 1.0f),
                FMath::Clamp(pixels[i].G.GetFloat(), 0.0f, 1.0f),
                FMath::Clamp(pixels[i].B.GetFloat(), 0.0f, 1.0f),
                1.0f);
            bgra.Add(lin.ToFColor(true));
        }

        TArray64<uint8> png;
        FImageUtils::PNGCompressImageArray(size, size, TArrayView64<const FColor>(bgra.GetData(), bgra.Num()), png);
        if (!FFileHelper::SaveArrayToFile(png, *path)) {
            UE_LOG(LogTemp, Warning, TEXT("[CubeSpike] could not write %s"), *path);
        }
    }

    void RunCubeCaptureSpike(int32 mode)
    {
        UWorld* world = FindGameWorld();
        if (world == nullptr) {
            UE_LOG(LogTemp, Warning, TEXT("[CubeSpike] no Game/PIE world - run this in play, not in the editor viewport"));
            return;
        }

        FVector location;
        FRotator rotation;
        FString viewpoint_from;
        if (!FindViewpoint(world, location, rotation, viewpoint_from)) {
            UE_LOG(LogTemp, Warning, TEXT("[CubeSpike] no player camera manager and no APIPCamera - nowhere to capture from"));
            return;
        }

        FString source_name;
        const ESceneCaptureSource source = SourceForMode(mode, source_name);

        IConsoleVariable* single_pass_cvar = IConsoleManager::Get().FindConsoleVariable(TEXT("r.SceneCapture.CubeSinglePass"));
        const int32 single_pass = single_pass_cvar ? single_pass_cvar->GetInt() : -1;

        UE_LOG(LogTemp, Log,
               TEXT("[CubeSpike] mode=%d source=%s face=%d r.SceneCapture.CubeSinglePass=%d viewpoint=%s loc=%s rot=%s"),
               mode, *source_name, kFaceSize, single_pass, *viewpoint_from, *location.ToString(), *rotation.ToString());

        // TStrongObjectPtr keeps these alive for the (synchronous) duration without a UPROPERTY.
        TStrongObjectPtr<UTextureRenderTargetCube> cube_target(NewObject<UTextureRenderTargetCube>(world));
        cube_target->ClearColor = FLinearColor::Black;
        cube_target->Init(kFaceSize, PF_FloatRGBA);      // PF_FloatRGBA: the only cube format Vulkan can read back
        cube_target->UpdateResourceImmediate(true);

        TStrongObjectPtr<USceneCaptureComponentCube> cube_capture(NewObject<USceneCaptureComponentCube>(world));
        cube_capture->CaptureSource = source;
        cube_capture->TextureTarget = cube_target.Get();
        // bCaptureRotation=true makes the faces follow the component's rotation, so PosX is the
        // face looking straight ahead - directly comparable with the 2D reference below.
        cube_capture->bCaptureRotation = true;
        cube_capture->bCaptureEveryFrame = false;
        cube_capture->bCaptureOnMovement = false;
        // Register before posing, the order NanoGSShadowManagerSubsystem uses for a manually
        // created (owner-less) capture component.
        cube_capture->RegisterComponentWithWorld(world);
        cube_capture->SetWorldLocationAndRotation(location, rotation);
        cube_capture->SetVisibility(true);

        // 2D reference at the same pose with a 90 degree FOV. Phase 3a proved splats appear here,
        // so it is the control: if the 2D image has no splats either, the run says nothing about
        // cube captures and should be repeated somewhere the splats are actually visible.
        TStrongObjectPtr<UTextureRenderTarget2D> ref_target(NewObject<UTextureRenderTarget2D>(world));
        ref_target->ClearColor = FLinearColor::Black;
        ref_target->InitCustomFormat(kFaceSize, kFaceSize, PF_FloatRGBA, true);
        ref_target->UpdateResourceImmediate(true);

        TStrongObjectPtr<USceneCaptureComponent2D> ref_capture(NewObject<USceneCaptureComponent2D>(world));
        ref_capture->CaptureSource = source;
        ref_capture->TextureTarget = ref_target.Get();
        ref_capture->FOVAngle = 90.0f;
        ref_capture->bCaptureEveryFrame = false;
        ref_capture->bCaptureOnMovement = false;
        ref_capture->RegisterComponentWithWorld(world);
        ref_capture->SetWorldLocationAndRotation(location, rotation);
        ref_capture->SetVisibility(true);

        // Immediate, not deferred: a probe wants the pixels before it returns. The real rig will
        // use CaptureSceneDeferred() like the rest of AirSim does (finding F9).
        ref_capture->CaptureScene();
        cube_capture->CaptureScene();

        const FString dir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("CubeCaptureSpike"));
        IFileManager::Get().MakeDirectory(*dir, true);
        const FString tag = FString::Printf(TEXT("mode%d_%s"), mode, *source_name);

        if (FTextureRenderTargetResource* ref_resource = ref_target->GameThread_GetRenderTargetResource()) {
            TArray<FFloat16Color> pixels;
            const bool ok = ref_resource->ReadFloat16Pixels(pixels);
            const FFaceStats s = ComputeStats(pixels);
            UE_LOG(LogTemp, Log,
                   TEXT("[CubeSpike] %s  2D-REFERENCE   read=%s px=%d mean=%.6f max=%.4f nonzero=%.2f%%"),
                   *tag, ok ? TEXT("ok") : TEXT("FAILED"), s.pixels, s.mean_luma, s.max_luma, s.nonzero_fraction * 100.0);
            WritePng(pixels, kFaceSize, FPaths::Combine(dir, tag + TEXT("_2Dreference.png")));
        }

        if (FTextureRenderTargetResource* cube_resource = cube_target->GameThread_GetRenderTargetResource()) {
            for (int32 face = 0; face < (int32)CubeFace_MAX; ++face) {
                TArray<FFloat16Color> pixels;
                // The cube face selector lives in the read flags; see the Vulkan note at the top.
                const FReadSurfaceDataFlags flags(RCM_UNorm, (ECubeFace)face);
                const bool ok = cube_resource->ReadFloat16Pixels(pixels, flags);
                const FFaceStats s = ComputeStats(pixels);
                UE_LOG(LogTemp, Log,
                       TEXT("[CubeSpike] %s  CUBE face %d %s read=%s px=%d mean=%.6f max=%.4f nonzero=%.2f%%"),
                       *tag, face, CubeFaceName(face), ok ? TEXT("ok") : TEXT("FAILED"), s.pixels,
                       s.mean_luma, s.max_luma, s.nonzero_fraction * 100.0);
                WritePng(pixels, kFaceSize, FPaths::Combine(dir, FString::Printf(TEXT("%s_cube%d_%s.png"), *tag, face, CubeFaceName(face))));
            }
        }

        UE_LOG(LogTemp, Log, TEXT("[CubeSpike] %s done - images in %s"), *tag, *dir);

        // Tear the rig down so nothing keeps rendering after the one-shot.
        cube_capture->DestroyComponent();
        ref_capture->DestroyComponent();
    }

    void OnCubeCaptureSpikeChanged(IConsoleVariable* var)
    {
        const int32 mode = var->GetInt();
        if (mode <= 0)
            return;

        RunCubeCaptureSpike(mode);
    }

    struct FCubeCaptureSpikeCVarBinder
    {
        FCubeCaptureSpikeCVarBinder()
        {
            CVarCubeCaptureSpike.AsVariable()->SetOnChangedCallback(
                FConsoleVariableDelegate::CreateStatic(&OnCubeCaptureSpikeChanged));
        }
    };
    FCubeCaptureSpikeCVarBinder g_cube_capture_spike_binder;
}
