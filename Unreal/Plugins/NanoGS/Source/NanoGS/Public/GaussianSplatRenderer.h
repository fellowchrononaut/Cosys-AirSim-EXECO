// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RenderGraphResources.h"
#include "RenderGraphBuilder.h"
#include "SceneView.h"

// One scene light, flattened for the composite shader. World space.
struct FGaussianLight
{
	FVector3f Position    = FVector3f::ZeroVector; // world; unused for directional
	float     InvRadius   = 0.f;                   // 1/attenuationRadius; 0 = directional (no falloff)
	FVector3f Color       = FVector3f::ZeroVector; // tint, normalized to [0,1] max-component
	float     Type        = 0.f;                   // 0 = directional, 1 = point, 2 = spot
	FVector3f Direction   = FVector3f(0,0,-1);     // world dir the light travels (dir/spot)
	float     CosOuter    = -1.f;                  // spot outer cone cosine (<= -1 means "no cone")
	float     CosInner    = 1.f;                   // spot inner cone cosine (for smooth edge)
	float     Intensity   = 1.f;                   // brightness (max-component of HDR GetColor()); drives the slider
};

// Per-frame scene lighting gathered on the render thread.
struct FGaussianSceneLighting
{
	static constexpr int32 MaxLights = 16;
	FGaussianLight Lights[MaxLights];
	int32     NumLights      = 0;
	FVector3f AmbientColor   = FVector3f(0.1f, 0.1f, 0.1f);
	float     LightingBlend  = 0.f;   // 0 = unlit (original), 1 = fully lit
	float     IntensityScale = 0.3f;  // global sensitivity: contribution = tint * Intensity * IntensityScale
	float     ResponseCeiling = 1.f;  // per-light contribution ceiling; raise past 1 for brighter lights
	float     RelightRatioMin = 0.6f; // clamp floor for the brightness-preserving relight ratio
	float     RelightRatioMax = 1.6f; // clamp ceiling for the brightness-preserving relight ratio
	// Bilateral depth smoothing for normal reconstruction (removes per-splat depth scatter)
	int32     NormalSmoothRadius = 2;     // kernel radius in pixels; 0 = off
	float     NormalSmoothFrac   = 0.02f; // depth-similarity sigma as fraction of view depth
	int32     NormalSampleStep   = 3;     // central-difference baseline (pixels) for the normal

	// Geometry source for lighting reconstruction. 0 = screen-space splat depth (default,
	// noisy but needs no scene setup). 1 = a hidden proxy mesh's CustomDepth (clean geometry,
	// requires the user to enable "Render CustomDepth Pass" on a co-located mesh).
	int32     GeometryMode = 0;
	// Engine's per-frame DeviceZ->linear-view-Z transform (FSceneView::InvDeviceZToWorldZTransform),
	// needed to linearize CustomDepth (GeometryMode == 1 only; unused for splat depth-accum).
	FVector4f InvDeviceZToWorldZTransform = FVector4f(0, 0, 0, 0);
};

class FGaussianSplatSceneProxy;
class FGaussianSplatGPUResources;
struct FGaussianGlobalAccumulator;

/**
 * Handles the rendering of Gaussian Splats
 * Orchestrates compute passes for view calculation, sorting, and final rendering
 */
class NANOGS_API FGaussianSplatRenderer
{
public:
	FGaussianSplatRenderer();
	~FGaussianSplatRenderer();

	/**
	 * Dispatch the view data calculation compute shader
	 * @param bUseLODRendering If true, skip splats covered by parent LOD clusters
	 */
	static void DispatchCalcViewData(
		FRHICommandListImmediate& RHICmdList,
		const FSceneView& View,
		FGaussianSplatGPUResources* GPUResources,
		const FMatrix& LocalToWorld,
		int32 SplatCount,
		int32 SHOrder,
		float OpacityScale,
		float SplatScale,
		bool bUseLODRendering = false
	);

	/**
	 * Dispatch the distance calculation compute shader
	 */
	static void DispatchCalcDistances(
		FRHICommandListImmediate& RHICmdList,
		FGaussianSplatGPUResources* GPUResources,
		int32 SplatCount
	);

	/**
	 * Dispatch radix sort for back-to-front ordering
	 */
	static void DispatchRadixSort(
		FRHICommandListImmediate& RHICmdList,
		FGaussianSplatGPUResources* GPUResources,
		int32 SplatCount
	);

	/**
	 * Dispatch radix sort with indirect dispatch (GPU-driven sort count)
	 * Uses SortIndirectArgsBuffer for CountCS/ScatterCS dispatch dimensions
	 * and SortParamsBuffer for Count/NumTiles read by shaders.
	 * Only sorts the visible splats after compaction.
	 */
	static void DispatchRadixSortIndirect(
		FRHICommandListImmediate& RHICmdList,
		FGaussianSplatGPUResources* GPUResources
	);

	/**
	 * Draw the Gaussian splats
	 */
	static void DrawSplats(
		FRHICommandListImmediate& RHICmdList,
		const FSceneView& View,
		FGaussianSplatGPUResources* GPUResources,
		int32 SplatCount
	);

	/**
	 * Dispatch cluster culling compute shader (Nanite-style optimization)
	 * Tests cluster bounding spheres against view frustum
	 * @param ErrorThreshold Screen-space error threshold in pixels for LOD selection
	 * @param bUseLODRendering If true, track unique LOD clusters for later rendering
	 * @return Number of visible clusters (for statistics)
	 */
	static int32 DispatchClusterCulling(
		FRHICommandListImmediate& RHICmdList,
		const FSceneView& View,
		FGaussianSplatGPUResources* GPUResources,
		const FMatrix& LocalToWorld,
		float ErrorThreshold,
		bool bUseLODRendering = false
	);

	// NOTE: DispatchCalcLODViewDataGPUDriven and DispatchUpdateDrawArgs have been removed
	// in the unified approach. LOD splats are now processed by DispatchCalcViewData.

	//----------------------------------------------------------------------
	// Splat Compaction (GPU-driven work reduction)
	//----------------------------------------------------------------------

	/**
	 * Dispatch the splat compaction compute shader
	 * Builds a compact list of visible splat indices using atomics
	 */
	static void DispatchCompactSplats(
		FRHICommandListImmediate& RHICmdList,
		FGaussianSplatGPUResources* GPUResources,
		int32 TotalSplatCount,
		int32 OriginalSplatCount,
		bool bUseLODRendering
	);

	/**
	 * Dispatch the prepare indirect args compute shader
	 * Prepares indirect dispatch and draw arguments from visible splat count
	 */
	static void DispatchPrepareIndirectArgs(
		FRHICommandListImmediate& RHICmdList,
		FGaussianSplatGPUResources* GPUResources
	);

	/**
	 * Dispatch CalcViewData with compaction (indirect dispatch)
	 * Only processes visible splats from compacted list
	 */
	static void DispatchCalcViewDataCompacted(
		FRHICommandListImmediate& RHICmdList,
		const FSceneView& View,
		FGaussianSplatGPUResources* GPUResources,
		const FMatrix& LocalToWorld,
		int32 SplatCount,
		int32 OriginalSplatCount,
		int32 SHOrder,
		float OpacityScale,
		float SplatScale
	);

	/**
	 * Dispatch CalcDistances with indirect dispatch
	 * Only processes visible splats
	 */
	static void DispatchCalcDistancesIndirect(
		FRHICommandListImmediate& RHICmdList,
		FGaussianSplatGPUResources* GPUResources
	);

	//----------------------------------------------------------------------
	// Global Accumulator dispatch (one-draw-call path)
	//----------------------------------------------------------------------

	/**
	 * Dispatch CalcViewData writing into GlobalAccumulator->GlobalViewDataBuffer
	 * at GlobalBaseOffset, instead of the per-proxy ViewDataBuffer.
	 */
	static void DispatchCalcViewDataGlobal(
		FRHICommandListImmediate& RHICmdList,
		const FSceneView& View,
		FGaussianSplatGPUResources* GPUResources,
		const FMatrix& LocalToWorld,
		int32 SplatCount,
		int32 SHOrder,
		float OpacityScale,
		float SplatScale,
		bool bUseLODRendering,
		uint32 GlobalBaseOffset,
		FGaussianGlobalAccumulator* GlobalAccumulator
	);

	/**
	 * Dispatch CalcDistances over the full global ViewDataBuffer.
	 * Must be called after all Phase-1 CalcViewData dispatches.
	 */
	static void DispatchCalcDistancesGlobal(
		FRHICommandListImmediate& RHICmdList,
		FGaussianGlobalAccumulator* GlobalAccumulator,
		int32 TotalSplatCount
	);

	/**
	 * Dispatch radix sort over the full global distance/key buffers.
	 */
	static void DispatchRadixSortGlobal(
		FRHICommandListImmediate& RHICmdList,
		FGaussianGlobalAccumulator* GlobalAccumulator,
		int32 TotalSplatCount
	);

	/**
	 * Draw all splats using global sorted keys and ViewData.
	 * Borrows the IndexBuffer from the first valid proxy.
	 */
	static void DrawSplatsGlobal(
		FRHICommandListImmediate& RHICmdList,
		const FSceneView& View,
		FGaussianGlobalAccumulator* GlobalAccumulator,
		FBufferRHIRef IndexBuffer,
		int32 TotalSplatCount,
		int32 DebugMode,
		bool bOutputDepth = false,   // also write the alpha-weighted depth MRT (RT2) for lighting (GeometryMode 0/1)
		bool bOutputNormal = false   // also write the alpha-weighted normal MRT (RT2) for lighting (GeometryMode 2)
	);

	//----------------------------------------------------------------------
	// Global Accumulator + Nanite Compaction dispatch (one-draw-call path)
	// Phase sequence: GatherVisibleCount × N → PrefixSumVisibleCounts ×1 →
	//   CalcViewDataCompactedGlobal × N → CalcDistancesGlobalIndirect ×1 →
	//   RadixSortGlobalIndirect ×1 → DrawSplatsGlobalIndirect ×1
	//----------------------------------------------------------------------

	/**
	 * Copy GPUResources->VisibleSplatCountBuffer[0] into
	 * GlobalAccumulator->GlobalVisibleCountArrayBuffer[ProxyIndex].
	 * Dispatch: (1,1,1) per proxy, after DispatchCompactSplats.
	 */
	static void DispatchGatherVisibleCount(
		FRHICommandListImmediate& RHICmdList,
		FGaussianSplatGPUResources* GPUResources,
		FGaussianGlobalAccumulator* GlobalAccumulator,
		int32 ProxyIndex
	);

	/**
	 * Compute exclusive prefix sums over GlobalVisibleCountArray and write
	 * all indirect dispatch/draw args for Phase-3 passes.
	 * Dispatch: (1,1,1) once, after all GatherVisibleCount dispatches.
	 */
	static void DispatchPrefixSumVisibleCounts(
		FRHICommandListImmediate& RHICmdList,
		FGaussianGlobalAccumulator* GlobalAccumulator,
		int32 ProxyCount,
		uint32 MaxRenderBudget
	);

	/**
	 * CalcViewData for proxy ProxyIndex, writing into GlobalViewDataBuffer
	 * at offset GlobalBaseOffsetsBuffer[ProxyIndex].
	 * Uses IndirectDispatchArgsBuffer from GPUResources (set by PrepareIndirectArgs).
	 */
	static void DispatchCalcViewDataCompactedGlobal(
		FRHICommandListImmediate& RHICmdList,
		const FSceneView& View,
		FGaussianSplatGPUResources* GPUResources,
		const FMatrix& LocalToWorld,
		int32 SplatCount,
		int32 OriginalSplatCount,
		int32 SHOrder,
		float OpacityScale,
		float SplatScale,
		int32 ProxyIndex,
		FGaussianGlobalAccumulator* GlobalAccumulator,
		uint32 MaxRenderBudget
	);

	/**
	 * CalcDistances over the global ViewDataBuffer using indirect dispatch
	 * (count from GlobalCalcDistIndirectArgsBuffer written by PrefixSumCS).
	 */
	static void DispatchCalcDistancesGlobalIndirect(
		FRHICommandListImmediate& RHICmdList,
		FGaussianGlobalAccumulator* GlobalAccumulator
	);

	/**
	 * Radix sort over the global distance/key buffers using indirect dispatch
	 * (count/numTiles from GlobalSortParamsBuffer written by PrefixSumCS).
	 */
	static void DispatchRadixSortGlobalIndirect(
		FRHICommandListImmediate& RHICmdList,
		FGaussianGlobalAccumulator* GlobalAccumulator
	);

	/**
	 * Draw all visible splats using GlobalDrawIndirectArgsBuffer
	 * (instance count written by PrefixSumCS, not a CPU constant).
	 */
	static void DrawSplatsGlobalIndirect(
		FRHICommandListImmediate& RHICmdList,
		const FSceneView& View,
		FGaussianGlobalAccumulator* GlobalAccumulator,
		FBufferRHIRef IndexBuffer,
		int32 DebugMode,
		bool bOutputDepth = false,   // also write the alpha-weighted depth MRT (RT2) for lighting (GeometryMode 0/1)
		bool bOutputNormal = false   // also write the alpha-weighted normal MRT (RT2) for lighting (GeometryMode 2)
	);

	/**
	 * Composite the intermediate sRGB-blended splat texture onto SceneColor.
	 * Converts from sRGB to linear color space during compositing.
	 * This ensures gaussian splat alpha blending happens in sRGB space
	 * (matching 3DGS training) while still integrating with UE's linear pipeline.
	 * Also applies screen-space dynamic lighting when Lighting.LightingBlend > 0.
	 */
	static void CompositeToSceneColor(
		FRHICommandListImmediate& RHICmdList,
		const FSceneView& View,
		FTextureRHIRef IntermediateTexture,
		FTextureRHIRef DepthAccumTexture,           // alpha-weighted depth MRT (RG32F); null = no lighting
		FTextureRHIRef CustomDepthTexture,          // proxy-mesh CustomDepth; null = GeometryMode 1 unavailable
		FTextureRHIRef NormalAccumTexture,           // alpha-weighted per-splat normal MRT; null = GeometryMode 2 unavailable
		const FGaussianSceneLighting& Lighting      // gathered scene lights
	);

private:
	/** Calculate next power of 2 */
	static uint32 NextPowerOfTwo(uint32 Value);

	/**
	 * Extract frustum planes from view-projection matrix
	 * Planes are in world space, normalized with normal pointing inward
	 * Order: Left, Right, Bottom, Top, Near, Far
	 */
	static void ExtractFrustumPlanes(const FMatrix& ViewProjection, FVector4f OutPlanes[6]);
};
