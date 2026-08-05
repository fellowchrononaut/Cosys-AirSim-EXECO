// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Cube face debug dump and render-cost bench - Phase 3b step 3.
//
// Two console variables, both inert at their default of 0, so a build with this file behaves
// identically to a build without it until someone types one of them:
//
//   airsim.CubeFaceDump 1     one-shot: capture every face of every camera that has a
//                             CameraModel block, write one PNG per face under
//                             Saved/CubeFaces, log per-face statistics, and measure all twelve
//                             shared cube edges.
//   airsim.CubeFaceBench 20   time 1 face, 5 faces and 6 faces per camera over N iterations.
//
// WHY THE EDGE CHECK IS THE POINT
// Six faces that look plausible prove nothing. Each face is an exact pinhole render and all six
// share one origin, so along a shared cube edge the two faces sample the same directions to
// within half a face texel - a discontinuity there is a bug in the rig, not in the scene. Step 2
// measured cube-component faces arriving rotated 90 degrees against a 2D capture of the same
// pose, which is exactly the failure this measures. The convention under test is stated in full
// above APIPCamera::getCubeFaceRotation in PIPCamera.cpp.
//
// The comparison is not equality. The two texel rows either side of a cube edge straddle it by
// half a texel each, so they sample directions about one texel apart and differ by roughly one
// texel of local image gradient. So each edge is reported against a reference measured inside
// each face - the same border line against its immediate neighbour line - and continuity means
// the cross-face difference is of the same order as that reference. A wrong orientation does not
// produce a slightly larger number, it produces an unrelated image.
//
// WHAT THIS TOOL CANNOT TELL YOU - read before acting on a large number
//
// It measures ONE thing: whether two faces agree along a shared edge. It cannot say why they do
// not, and it used to claim it could. Two limits, both learned the hard way:
//
//  1. A LARGE NUMBER ON SPLAT CONTENT IS EXPECTED AND IS NOT A BUG. Under cube capture each face
//     linearises EWA with its own pinhole projection, so a Gaussian straddling a face boundary
//     gets different 2D covariances either side and seams BY CONSTRUCTION.
//     CAMERA-MODEL-DESIGN.md section 2.2 predicts it and section 8 item 6 calls it "data, not a
//     defect to chase". This is also the mechanism behind the Isaac Sim NuRec fisheye artifacts
//     (issue 399). Record the magnitude as the EWA baseline; do not go looking for a rig fault.
//     3DGEER splats and Unreal MESH geometry have no such mechanism, so a seam on either of
//     those IS worth chasing - mesh is the case with a known-correct answer.
//
//  2. AN EDGE WITH NO LOCAL GRADIENT CANNOT BE JUDGED AT ALL. When both faces show flat sky or
//     flat floor along an edge, the within-face reference is ~0 and the ratio it feeds is
//     meaningless - a fraction of an intensity level over nothing at all. Such edges are now
//     reported "no gradient" and given no verdict, instead of being passed as continuous, which
//     was equally unearned in the other direction.
//
// So the verdict vocabulary is deliberately narrow: "continuous", "no gradient", and
// "MISMATCH (cause not diagnosed here)". Orientation, field of view and a non-shared origin all
// produce a mismatch and this tool cannot separate them; so does splat content, legitimately.
//
// Scene only. Depth and segmentation targets are float or gamma-special and ReadPixels would
// quantise them into nonsense; per-modality handling is step 5's, and this is a debug tool.
//
// The bench measures the render side only - CaptureScene plus FlushRenderingCommands, no
// readback and no simGetImages call overhead - so its numbers are NOT comparable to the
// whole-call figures of Overall_Project_Plan section 1.5. It answers the relative question,
// N faces against 1 face, on the real rig. The section 1.5 comparison is a simGetImages
// measurement with airsim.LogImageTiming, taken separately.

#include "CoreMinimal.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TextureResource.h"
#include "RenderingThread.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "PIPCamera.h"

static TAutoConsoleVariable<int32> CVarCubeFaceDump(
    TEXT("airsim.CubeFaceDump"),
    0,
    TEXT("Step 3: dump the cube faces of every camera that has a CameraModel block.\n")
    TEXT(" 0: off (default, nothing in CubeFaceDump.cpp runs)\n")
    TEXT(" 1: capture all faces, write PNGs to Saved/CubeFaces, log the twelve edge checks"),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarCubeFaceBench(
    TEXT("airsim.CubeFaceBench"),
    0,
    TEXT("Step 3: time N iterations of 1, 5 and 6 face captures per camera model camera.\n")
    TEXT(" 0: off (default)\n")
    TEXT(" N: N timed iterations per arm, render side only, no readback"),
    ECVF_Default);

namespace
{
    // Image-space edges of a face, named as they appear in the written PNG.
    enum EFaceEdge
    {
        EdgeTop = 0,
        EdgeBottom,
        EdgeLeft,
        EdgeRight
    };

    struct FEdgePair
    {
        int32 face_a;
        EFaceEdge edge_a;
        int32 face_b;
        EFaceEdge edge_b;
        bool reversed; //traverse face B's edge backwards
        const TCHAR* label;
    };

    // The twelve edges of the cube, derived from the face table in PIPCamera.cpp. Face indices
    // are 0 Front, 1 Right, 2 Left, 3 Up, 4 Down, 5 Back. Writing a ray on face f as
    // F + a*R - b*U with a and b running over (-1, 1), the left edge is a = -1, the right edge
    // a = +1, the top edge b = -1 and the bottom edge b = +1; two edges match when they span the
    // same directions, and "reversed" records that they do so in opposite parameter order. Four
    // of the twelve are reversed and that is a property of a cube net, not a bug.
    const FEdgePair kEdgePairs[] = {
        { 0, EdgeRight, 1, EdgeLeft, false, TEXT("Front|Right") },
        { 1, EdgeRight, 5, EdgeLeft, false, TEXT("Right|Back ") },
        { 5, EdgeRight, 2, EdgeLeft, false, TEXT("Back |Left ") },
        { 2, EdgeRight, 0, EdgeLeft, false, TEXT("Left |Front") },
        { 0, EdgeTop, 3, EdgeBottom, false, TEXT("Front|Up   ") },
        { 0, EdgeBottom, 4, EdgeTop, false, TEXT("Front|Down ") },
        { 5, EdgeTop, 3, EdgeTop, true, TEXT("Back |Up   ") },
        { 5, EdgeBottom, 4, EdgeBottom, true, TEXT("Back |Down ") },
        { 1, EdgeTop, 3, EdgeRight, true, TEXT("Right|Up   ") },
        { 2, EdgeTop, 3, EdgeLeft, false, TEXT("Left |Up   ") },
        { 1, EdgeBottom, 4, EdgeRight, false, TEXT("Right|Down ") },
        { 2, EdgeBottom, 4, EdgeLeft, true, TEXT("Left |Down ") }
    };

    // Mean absolute difference, 0-255 units, below which an edge carries too little local
    // gradient for the cross-face comparison to mean anything. One intensity level over a whole
    // border line: quantisation and mild dither reach this, image structure does not.
    constexpr double kMinEdgeReference = 1.0;

    struct FCubeFaceDumpStats
    {
        double mean_luma = 0.0;
        double max_luma = 0.0;
        double nonzero_fraction = 0.0;
        int32 pixels = 0;
    };

    // Same world lookup the airsim.CaptureEveryFrame diagnostic uses (PIPCamera.cpp).
    UWorld* FindCubeFaceGameWorld()
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

    FCubeFaceDumpStats ComputeStats(const TArray<FColor>& pixels)
    {
        FCubeFaceDumpStats stats;
        stats.pixels = pixels.Num();
        if (stats.pixels == 0)
            return stats;

        double sum = 0.0;
        int64 nonzero = 0;
        for (const FColor& p : pixels) {
            const double luma = 0.2126 * p.R + 0.7152 * p.G + 0.0722 * p.B;
            sum += luma;
            stats.max_luma = FMath::Max(stats.max_luma, luma);
            if (luma > 0.5)
                ++nonzero;
        }
        stats.mean_luma = sum / stats.pixels;
        stats.nonzero_fraction = static_cast<double>(nonzero) / stats.pixels;
        return stats;
    }

    void WritePng(const TArray<FColor>& pixels, int32 width, int32 height, const FString& path)
    {
        if (pixels.Num() < width * height)
            return;

        TArray<FColor> opaque;
        opaque.Reserve(width * height);
        for (int32 i = 0; i < width * height; ++i) {
            FColor c = pixels[i];
            c.A = 255;
            opaque.Add(c);
        }

        TArray64<uint8> png;
        FImageUtils::PNGCompressImageArray(width, height, TArrayView64<const FColor>(opaque.GetData(), opaque.Num()), png);
        if (!FFileHelper::SaveArrayToFile(png, *path)) {
            UE_LOG(LogTemp, Warning, TEXT("[CubeFace] could not write %s"), *path);
        }
    }

    // One line of an image edge. inset 0 is the border line itself, inset 1 the line immediately
    // inside it - which is what gives the within-face reference the edge check is compared to.
    void ExtractEdge(const TArray<FColor>& pixels, int32 size, EFaceEdge edge, int32 inset,
                     bool reversed, TArray<FColor>& out)
    {
        out.Reset();
        if (pixels.Num() < size * size || inset < 0 || inset >= size)
            return;

        out.Reserve(size);
        for (int32 i = 0; i < size; ++i) {
            const int32 t = reversed ? (size - 1 - i) : i;
            int32 x = 0;
            int32 y = 0;
            switch (edge) {
            case EdgeTop:
                x = t;
                y = inset;
                break;
            case EdgeBottom:
                x = t;
                y = size - 1 - inset;
                break;
            case EdgeLeft:
                x = inset;
                y = t;
                break;
            default: //EdgeRight
                x = size - 1 - inset;
                y = t;
                break;
            }
            out.Add(pixels[y * size + x]);
        }
    }

    // Mean absolute difference over RGB, in 0..255 units.
    double MeanAbsDiff(const TArray<FColor>& a, const TArray<FColor>& b)
    {
        const int32 count = FMath::Min(a.Num(), b.Num());
        if (count == 0)
            return -1.0;

        double sum = 0.0;
        for (int32 i = 0; i < count; ++i) {
            sum += FMath::Abs(static_cast<double>(a[i].R) - static_cast<double>(b[i].R));
            sum += FMath::Abs(static_cast<double>(a[i].G) - static_cast<double>(b[i].G));
            sum += FMath::Abs(static_cast<double>(a[i].B) - static_cast<double>(b[i].B));
        }
        return sum / (3.0 * count);
    }

    // Capture the faces of one camera for one ImageType. Returns how many were captured.
    int32 CaptureFaces(APIPCamera* camera, APIPCamera::ImageType image_type, int32 face_limit)
    {
        int32 captured = 0;
        const int32 face_count = FMath::Min(camera->getCubeFaceCount(), face_limit);
        for (int32 face = 0; face < face_count; ++face) {
            USceneCaptureComponent2D* capture = camera->getFaceCaptureComponent(image_type, face);
            if (capture == nullptr)
                continue;
            //Immediate, not deferred: a debug tool wants the pixels before it returns. The real
            //path will use CaptureSceneDeferred, which is what makes all faces in one request
            //share a single simulation instant (finding F9).
            capture->CaptureScene();
            ++captured;
        }
        return captured;
    }

    void RunCubeFaceDump()
    {
        UWorld* world = FindCubeFaceGameWorld();
        if (world == nullptr) {
            UE_LOG(LogTemp, Warning, TEXT("[CubeFace] no Game/PIE world - run this in play, not in the editor viewport"));
            return;
        }

        const APIPCamera::ImageType image_type = APIPCamera::ImageType::Scene;
        const FString dir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("CubeFaces"));
        IFileManager::Get().MakeDirectory(*dir, true);

        int32 cameras_seen = 0;
        int32 cameras_with_model = 0;

        for (TActorIterator<APIPCamera> it(world); it; ++it) {
            APIPCamera* camera = *it;
            ++cameras_seen;
            if (!camera->hasCameraModel())
                continue;
            ++cameras_with_model;

            //Builds the rig if this is the first request for this ImageType, exactly as an image
            //request would: this is the lazy path, not a second one.
            camera->setCameraTypeEnabled(image_type, true);

            const int32 face_count = camera->getCubeFaceCount();
            const int32 size = camera->getCubeFaceResolution();
            UE_LOG(LogTemp, Log, TEXT("[CubeFace] %s: %d faces at %dx%d"), *camera->GetName(), face_count, size, size);

            const int32 captured = CaptureFaces(camera, image_type, face_count);
            if (captured != face_count) {
                UE_LOG(LogTemp, Warning, TEXT("[CubeFace] %s: only %d of %d faces captured"),
                       *camera->GetName(), captured, face_count);
            }

            TArray<TArray<FColor>> face_pixels;
            face_pixels.SetNum(face_count);

            for (int32 face = 0; face < face_count; ++face) {
                UTextureRenderTarget2D* target = camera->getFaceRenderTarget(image_type, face);
                if (target == nullptr)
                    continue;

                FTextureRenderTargetResource* resource = target->GameThread_GetRenderTargetResource();
                if (resource == nullptr)
                    continue;

                const bool ok = resource->ReadPixels(face_pixels[face]);
                const FCubeFaceDumpStats stats = ComputeStats(face_pixels[face]);
                UE_LOG(LogTemp, Log,
                       TEXT("[CubeFace] %s face %d %s read=%s px=%d mean=%.3f max=%.1f nonzero=%.2f%%"),
                       *camera->GetName(), face, APIPCamera::getCubeFaceName(face),
                       ok ? TEXT("ok") : TEXT("FAILED"), stats.pixels, stats.mean_luma,
                       stats.max_luma, stats.nonzero_fraction * 100.0);

                const FString path = FPaths::Combine(dir, FString::Printf(TEXT("%s_scene_face%d_%s.png"),
                                                                          *camera->GetName(), face,
                                                                          APIPCamera::getCubeFaceName(face)));
                WritePng(face_pixels[face], size, size, path);
            }

            //The shared-edge check. cross is the difference across the cube edge; refA and refB
            //are the same border line against its neighbour line inside each face, i.e. the local
            //image gradient over one texel. Continuous means cross is of the same order as those.
            UE_LOG(LogTemp, Log, TEXT("[CubeFace] %s shared-edge check (cross vs within-face reference, 0-255 units)"),
                   *camera->GetName());
            for (const FEdgePair& pair : kEdgePairs) {
                if (pair.face_a >= face_count || pair.face_b >= face_count)
                    continue; //a five-face rig has no Back face and so four fewer edges
                if (face_pixels[pair.face_a].Num() == 0 || face_pixels[pair.face_b].Num() == 0)
                    continue;

                TArray<FColor> line_a, line_b, inner_a, inner_b;
                ExtractEdge(face_pixels[pair.face_a], size, pair.edge_a, 0, false, line_a);
                ExtractEdge(face_pixels[pair.face_a], size, pair.edge_a, 1, false, inner_a);
                ExtractEdge(face_pixels[pair.face_b], size, pair.edge_b, 0, pair.reversed, line_b);
                ExtractEdge(face_pixels[pair.face_b], size, pair.edge_b, 1, pair.reversed, inner_b);

                const double cross = MeanAbsDiff(line_a, line_b);
                const double ref_a = MeanAbsDiff(line_a, inner_a);
                const double ref_b = MeanAbsDiff(line_b, inner_b);
                const double reference = FMath::Max(ref_a, ref_b);

                //Below this the edge carries no local image gradient - flat sky against flat
                //sky - and the ratio is a fraction of an intensity level divided by nothing.
                //Refuse to judge it. Passing it as continuous was as unearned as failing it.
                const bool judgeable = reference >= kMinEdgeReference;
                const bool continuous = cross <= 3.0 * reference + 1.0;

                //No cause is named. This measures agreement, not its explanation: orientation,
                //field of view and a non-shared origin all land here, and so does splat content,
                //which is expected to seam by construction. See the header of this file.
                const TCHAR* verdict = !judgeable ? TEXT("no gradient - not judged")
                                                  : (continuous ? TEXT("continuous")
                                                                : TEXT("MISMATCH (cause not diagnosed here)"));

                UE_LOG(LogTemp, Log,
                       TEXT("[CubeFace]   %s %s cross=%7.3f  refA=%7.3f refB=%7.3f  ratio=%6.2f  %s"),
                       *camera->GetName(), pair.label, cross, ref_a, ref_b,
                       reference > 0.0 ? cross / reference : 0.0,
                       verdict);
            }
        }

        UE_LOG(LogTemp, Log, TEXT("[CubeFace] done: %d cameras, %d with a CameraModel block, images in %s"),
               cameras_seen, cameras_with_model, *dir);
    }

    void RunCubeFaceBench(int32 iterations)
    {
        UWorld* world = FindCubeFaceGameWorld();
        if (world == nullptr) {
            UE_LOG(LogTemp, Warning, TEXT("[CubeFaceBench] no Game/PIE world"));
            return;
        }

        const APIPCamera::ImageType image_type = APIPCamera::ImageType::Scene;

        TArray<APIPCamera*> cameras;
        for (TActorIterator<APIPCamera> it(world); it; ++it) {
            if (it->hasCameraModel()) {
                it->setCameraTypeEnabled(image_type, true);
                cameras.Add(*it);
            }
        }

        if (cameras.Num() == 0) {
            UE_LOG(LogTemp, Warning, TEXT("[CubeFaceBench] no camera has a CameraModel block - nothing to time"));
            return;
        }

        const int32 face_count = cameras[0]->getCubeFaceCount();
        const int32 resolution = cameras[0]->getCubeFaceResolution();
        UE_LOG(LogTemp, Log, TEXT("[CubeFaceBench] %d cameras, %d faces at %dx%d, %d iterations per arm"),
               cameras.Num(), face_count, resolution, resolution, iterations);

        const int32 arms[] = { 1, 5, 6 };
        const int32 arm_count = static_cast<int32>(UE_ARRAY_COUNT(arms));
        for (int32 arm_index = 0; arm_index < arm_count; ++arm_index) {
            const int32 faces_per_camera = FMath::Min(arms[arm_index], face_count);
            if (arm_index > 0 && faces_per_camera == FMath::Min(arms[arm_index - 1], face_count))
                continue; //a five-face rig would otherwise time the same arm twice

            //one discarded warm-up, so shader and render-state setup is not charged to iteration 1
            for (APIPCamera* camera : cameras)
                CaptureFaces(camera, image_type, faces_per_camera);
            FlushRenderingCommands();

            double total_ms = 0.0;
            double min_ms = TNumericLimits<double>::Max();
            double max_ms = 0.0;
            for (int32 i = 0; i < iterations; ++i) {
                const double start = FPlatformTime::Seconds();
                for (APIPCamera* camera : cameras)
                    CaptureFaces(camera, image_type, faces_per_camera);
                FlushRenderingCommands();
                const double elapsed_ms = (FPlatformTime::Seconds() - start) * 1000.0;

                total_ms += elapsed_ms;
                min_ms = FMath::Min(min_ms, elapsed_ms);
                max_ms = FMath::Max(max_ms, elapsed_ms);
            }

            const double mean_ms = iterations > 0 ? total_ms / iterations : 0.0;
            const int32 renders = faces_per_camera * cameras.Num();
            UE_LOG(LogTemp, Log,
                   TEXT("[CubeFaceBench] %d face(s) x %d camera(s) = %2d renders: mean %.2f ms (min %.2f, max %.2f), %.2f ms per render, %.1f Hz"),
                   faces_per_camera, cameras.Num(), renders, mean_ms, min_ms, max_ms,
                   renders > 0 ? mean_ms / renders : 0.0,
                   mean_ms > 0.0 ? 1000.0 / mean_ms : 0.0);
        }
    }

    void OnCubeFaceDumpChanged(IConsoleVariable* var)
    {
        if (var->GetInt() <= 0)
            return;

        RunCubeFaceDump();
    }

    void OnCubeFaceBenchChanged(IConsoleVariable* var)
    {
        const int32 iterations = var->GetInt();
        if (iterations <= 0)
            return;

        RunCubeFaceBench(iterations);
    }

    struct FCubeFaceCVarBinder
    {
        FCubeFaceCVarBinder()
        {
            CVarCubeFaceDump.AsVariable()->SetOnChangedCallback(
                FConsoleVariableDelegate::CreateStatic(&OnCubeFaceDumpChanged));
            CVarCubeFaceBench.AsVariable()->SetOnChangedCallback(
                FConsoleVariableDelegate::CreateStatic(&OnCubeFaceBenchChanged));
        }
    };
    FCubeFaceCVarBinder g_cube_face_cvar_binder;
}
