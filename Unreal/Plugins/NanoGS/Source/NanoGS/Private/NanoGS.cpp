// Copyright Epic Games, Inc. All Rights Reserved.

#include "NanoGS.h"
#include "GaussianSplatViewExtension.h"
#include "GaussianSplatRenderer.h"
#include "GaussianSplatSceneProxy.h"
#include "GaussianGlobalAccumulator.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"
#include "SceneViewExtension.h"
#include "Misc/CoreDelegates.h"
#include "Engine/Engine.h"
#include "UnrealClient.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "SceneView.h"
#include "ScreenPass.h"
#include "ScenePrivate.h"
#include "LightSceneProxy.h"    // FLightSceneProxy full definition (GetLightType, GetRadius, etc.)
#include "SceneManagement.h"    // FLightRenderParameters (SpotAngles for spot cone reconstruction)
#include "NanoGSLightingSettings.h" // Project Settings page that drives the gs.* lighting CVars
#include "SceneTextureParameters.h" // FSceneTextureUniformParameters::CustomDepthTexture (GeometryMode 1)
#include "NativeGeerViewRegistry.h"

#define LOCTEXT_NAMESPACE "FNanoGSModule"

namespace
{
	//Gate-A instrumentation is bounded to one line per registry generation even when a capture
	//component renders continuously. Mutated only by the render-thread callback below.
	TSet<uint64> GNanoGSLoggedNativeGeerRegistrations;
	TSet<uint64> GNanoGSLoggedNativeGeerRefusals;
	TSet<uint64> GNanoGSLoggedNativeGeerActive;
}

// Pass 2 parameter struct: declares IntermediateTexture, DepthAccumTexture, and CustomDepthTexture
// as RDG-tracked shader resource inputs so that RDG inserts the required RTV→SRV (or, for
// CustomDepthTexture, whatever barrier its last writer left pending) between Pass 1 and this pass.
BEGIN_SHADER_PARAMETER_STRUCT(FGaussianCompositePassParameters, )
	SHADER_PARAMETER_RDG_TEXTURE(Texture2D, IntermediateTexture)
	SHADER_PARAMETER_RDG_TEXTURE(Texture2D, DepthAccumTexture)   // read-after-write barrier for depth accum
	SHADER_PARAMETER_RDG_TEXTURE(Texture2D, CustomDepthTexture)  // proxy-mesh depth (GeometryMode 1 only)
	SHADER_PARAMETER_RDG_TEXTURE(Texture2D, NormalAccumTexture)  // per-splat normal accum (GeometryMode 2 only)
	RENDER_TARGET_BINDING_SLOTS()
END_SHADER_PARAMETER_STRUCT()

// Pass 1b (gs.DepthProximityWeighting, GeometryMode 2 only): re-rasterizes splats a second time,
// after Pass 1's DepthAccumTexture is fully resolved, writing ONLY a depth-proximity-weighted
// NormalAccum. DepthAccumTexture here is a read (RTV->SRV barrier); the render targets are
// throwaway color/velocity scratch (the per-splat PS always writes those two, regardless of
// permutation) plus the real NormalAccumTexture.
BEGIN_SHADER_PARAMETER_STRUCT(FGaussianNormalProximityPassParameters, )
	SHADER_PARAMETER_RDG_TEXTURE(Texture2D, DepthAccumTexture)
	RENDER_TARGET_BINDING_SLOTS()
END_SHADER_PARAMETER_STRUCT()

//----------------------------------------------------------------------
// Console Variables for Gaussian Splatting
//----------------------------------------------------------------------

/** Show cluster debug visualization (Nanite-style coloring) */
TAutoConsoleVariable<int32> CVarShowClusterBounds(
	TEXT("gs.ShowClusterBounds"),
	0,
	TEXT("Debug visualization for Gaussian Splat clusters (Nanite-style).\n")
	TEXT("When enabled, shows cluster colors on black background (like Nanite debug view).\n")
	TEXT(" 0: Off (default)\n")
	TEXT(" 1: Show cluster colors (each cluster gets a unique random color)"),
	ECVF_RenderThreadSafe);

/** Maximum number of splats the global accumulator will allocate working buffers for.
 *  Caps VRAM usage for ViewData/sort/histogram buffers. If total visible splats exceed
 *  this budget (after Nanite LOD compaction), excess splats are simply not rendered.
 *  When budget is active, closer assets get priority (farther assets culled first).
 *  Default: 0 (unlimited). Example: 3M budget uses ~195 MB working buffers. */
TAutoConsoleVariable<int32> CVarMaxRenderBudget(
	TEXT("gs.MaxRenderBudget"),
	0,
	TEXT("Maximum number of splats to render per frame (render budget).\n")
	TEXT("Caps global accumulator buffer allocation and GPU-side visible count.\n")
	TEXT("When budget is exceeded, farther assets are culled first (closer assets have priority).\n")
	TEXT("Default: 0 (unlimited). Set to a positive value (e.g. 3000000) to limit splat count."),
	ECVF_RenderThreadSafe);

/** Debug: Force a specific LOD level for debugging LOD hierarchy */
TAutoConsoleVariable<int32> CVarDebugForceLODLevel(
	TEXT("gs.DebugForceLODLevel"),
	-1,
	TEXT("Force rendering of a specific LOD level for debugging (only affects Nanite-enabled assets).\n")
	TEXT("This ignores normal LOD selection and forces all clusters to use specified level.\n")
	TEXT(" -1: Auto - normal LOD selection based on distance/error (default)\n")
	TEXT("  0: Force leaf clusters only - render original splats (finest detail)\n")
	TEXT("  1+: Force specific LOD level (1 = first parent level, 2 = second, etc.)\n")
	TEXT("Note: Higher levels have fewer, coarser splats. Max level depends on asset size.\n")
	TEXT("Use with gs.ShowClusterBounds 2 to visualize which LOD level is being rendered."),
	ECVF_RenderThreadSafe);

/** 3DGEER exact per-ray splat evaluation (see 3DGEERNanoGS.md). Per-asset via
 *  UGaussianSplatAsset::GeerMode; this CVar is the global override for A/B testing. */
TAutoConsoleVariable<int32> CVarGeerEval(
	TEXT("gs.GeerEval"),
	1,
	TEXT("3DGEER exact per-ray Gaussian evaluation (per-asset GeerEval property).\n")
	TEXT("Frames containing a GEER asset also sort by Euclidean camera distance.\n")
	TEXT(" 0: Force legacy EWA falloff + clip-z sort everywhere (kill switch)\n")
	TEXT(" 1: Respect the per-asset flag (default)\n")
	TEXT(" 2: Force GEER evaluation for ALL splat assets (A/B testing)"),
	ECVF_RenderThreadSafe);

/** Footprint inflation for GEER eval: the EWA quad edge is ~2.83 sigma but GEER integrates to
 *  3 sigma, and the EWA footprint itself is approximate. Tune down after Phase 2 (PBF). */
TAutoConsoleVariable<float> CVarGeerQuadInflation(
	TEXT("gs.GeerQuadInflation"),
	1.4f,
	TEXT("Multiplier on the EWA quad axes when gs.GeerEval is on (footprint only; 1.0 = off)."),
	ECVF_RenderThreadSafe);

/** Canonical-space cutoff radius for GEER evaluation, in sigma. The reference rasteriser bounds
 *  every Gaussian at 3 sigma (forward.cu:882 "cutoff = 3.0f") and culls rays outside those bounds
 *  before shading — in ALL association modes, PBF and EWA alike. Phase 1 has no geometric bound
 *  (the quad is EWA-derived and Phase 2 replaces it with the exact PBF rect), so without this the
 *  exp() tail is shaded across the whole quad with a nonzero pedestal at the edge, where the
 *  legacy falloff reached exactly zero. Applied per ray, so it is shape-agnostic and will remain
 *  correct once Phase 2 swaps the quad for a PBF rect. */
TAutoConsoleVariable<float> CVarGeerCutoff(
	TEXT("gs.GeerCutoff"),
	3.0f,
	TEXT("GEER canonical cutoff radius in sigma (forward.cu uses 3.0). 0 = no cutoff."),
	ECVF_RenderThreadSafe);

/** Near-plane cull for GEER splats, in cm. The reference drops any Gaussian whose view-space z
 *  is <= near_threshold (auxiliary.h in_frustum, default 0.2 m) and never shades it. We cull
 *  only clipPos.w <= 0, so splats in front of but very close to the camera survive — and for
 *  those the GEER density degenerates: p is tiny, so |d_obj x p_obj| is tiny, so power ~ 0 over
 *  the WHOLE quad, i.e. full opacity everywhere. That is correct GEER behaviour for a camera
 *  effectively inside a Gaussian, which is exactly why the reference culls instead of shading.
 *  EWA never showed this because its falloff is in quad-local coords and always decays.
 *  Applies to GEER splats only; classic assets are untouched. */
TAutoConsoleVariable<float> CVarGeerNearCull(
	TEXT("gs.GeerNearCull"),
	20.0f,
	TEXT("Cull GEER splats closer than this view-space depth in cm (reference: 0.2 m = 20 cm). 0 = off."),
	ECVF_RenderThreadSafe);

/** Exact GEER footprint: replace the EWA-derived quad with the Particle Bounding Frustum, the
 *  tight bound on the rays that pierce the splat's cutoff-sigma ellipsoid (forward.cu computePBF,
 *  pinhole branch). The EWA quad is sized from the PROJECTED 2D covariance, so for splats
 *  elongated along the view direction it is too SMALL and clips the density with a hard straight
 *  edge; PBF derives the bound from the 3D covariance instead. This visibly changes the image for
 *  those splats — that is the fix, not a regression. Phase 2 gated PBF on both the analytic scene
 *  and a real checkpoint, so it is the GEER default; classic splats still use their EWA quad.
 *  The bound follows gs.GeerCutoff so the geometric and per-ray 3-sigma bounds cannot disagree. */
TAutoConsoleVariable<int32> CVarGeerPBF(
	TEXT("gs.GeerPBF"),
	1,
	TEXT("GEER exact footprint (Particle Bounding Frustum) instead of the inflated EWA quad.\n")
	TEXT(" 0: EWA quad x gs.GeerQuadInflation (diagnostic fallback)\n")
	TEXT(" 1: PBF rect, bounded at gs.GeerCutoff sigma and clamped to the viewport (default)"),
	ECVF_RenderThreadSafe);

/** DIAGNOSTIC (2026-08-07): split the Euclidean sort key away from the GEER evaluation, which
 *  gs.GeerEval otherwise switches together. Against the reference oracle at test view 23 our
 *  render breaks into hard-edged shards while legacy EWA at the same pose is structurally
 *  correct, and the two paths differ in exactly four things: the falloff, this sort key, the AA
 *  opacity scaling, and the near cull. With gs.GeerEval 2 and this 0 you get the GEER falloff
 *  with the legacy clip-z order, which is the bisect that separates shading from ordering.
 *  Keep at 1 for normal use — Euclidean is what forward.cu sorts by (depths[] in preprocess),
 *  and it is also what makes cube-face order consistent for Phase 3. */
TAutoConsoleVariable<int32> CVarGeerSort(
	TEXT("gs.GeerSort"),
	1,
	TEXT("Sort key used when a GEER asset is visible.\n")
	TEXT(" 0: legacy clip-z key (diagnostic — isolates ordering from shading)\n")
	TEXT(" 1: Euclidean camera->splat distance, matching forward.cu (default)"),
	ECVF_RenderThreadSafe);

/** Splat hardware depth-write policy. The GEER reference composites splats without letting one
 *  splat reject another through a depth buffer, while legacy NanoGS writes each accepted
 *  fragment's centre depth. Auto mode preserves the legacy path for classic-only views and turns
 *  splat depth writes off whenever at least one visible proxy takes the GEER evaluation path.
 *  Scene depth is still tested and the temporal-responsive stencil mask is still written. */
TAutoConsoleVariable<int32> CVarGeerDepthWrite(
	TEXT("gs.GeerDepthWrite"),
	-1,
	TEXT("Hardware depth-write policy for the splat pass.\n")
	TEXT("-1: auto — write for classic-only views, read-only when GEER is visible (default)\n")
	TEXT(" 0: test existing scene depth, but do not let splats write depth\n")
	TEXT(" 1: force splat centre-depth writes (legacy NanoGS diagnostic)"),
	ECVF_RenderThreadSafe);

/** DIAGNOSTIC (2026-08-07): disable the GEER antialiasing opacity scaling (forward.cu omni_hvar).
 *  It is ~1 for ordinary splats but drops sharply for splats thinner than sqrt(h_var) = 0.032 cm,
 *  so on a real checkpoint it is a per-splat opacity change that the analytic scene never
 *  exercised. Part of the same four-way bisect as gs.GeerSort. */
TAutoConsoleVariable<int32> CVarGeerAAOpacity(
	TEXT("gs.GeerAAOpacity"),
	1,
	TEXT("GEER antialiasing opacity scaling (forward.cu omni_hvar). 0 = off (diagnostic)."),
	ECVF_RenderThreadSafe);

/** Diagnostic: visualise the GEER canonical radius instead of shading. Intended for the
 *  single-splat analytic scene (arm 1), where there is no blending to confuse the readout —
 *  it makes the density field's iso-contours directly measurable in pixels, so an effective
 *  sigma error or a transposed basis is a number rather than an impression. */
TAutoConsoleVariable<int32> CVarGeerDebugView(
	TEXT("gs.GeerDebugView"),
	0,
	TEXT("Visualise GEER evaluation instead of shading it.\n")
	TEXT(" 0: Off (normal shading)\n")
	TEXT(" 1: Canonical radius r — green ring at r=1 sigma, blue ring at r=2, red ramps 0..3\n")
	TEXT(" 2: DIAGNOSTIC — the sort KEY (ViewDistance, cm, ramped over 10 m)\n")
	TEXT(" 3: DIAGNOSTIC — the sort RESULT (position in the sorted draw order, 0..1)\n")
	TEXT("    2 and 3 should both read as a smooth depth ramp. 2 smooth + 3 noisy means the\n")
	TEXT("    key is right and the sort is not; 2 already noisy means the key value is wrong."),
	ECVF_RenderThreadSafe);

/** Diagnostic for cube-face residual attribution. Modes 1-4 switch RT0 to additive blending and
 * accumulate one scalar per GEER fragment; mode 5 exposes the ordinary over-composite's final
 * opacity. The composite pass writes a pure grayscale frame on black, so mesh shading and scene
 * post colour cannot be mistaken for a splat residual. Off by default and inert in mode 0. */
TAutoConsoleVariable<int32> CVarGeerResidualDebug(
	TEXT("gs.GeerResidualDebug"),
	0,
	TEXT("Per-face GEER coverage/evaluation residual diagnostic.\n")
	TEXT(" 0: Off (normal rendering)\n")
	TEXT(" 1: Additive candidate count (rasterized quad/PBF footprint)\n")
	TEXT(" 2: Additive finite-power/cutoff survivor count\n")
	TEXT(" 3: Additive final contributor count (alpha >= 1/255)\n")
	TEXT(" 4: Additive sum of accepted per-splat alpha\n")
	TEXT(" 5: Final over-composited opacity, 1-product(1-alpha), order independent\n")
	TEXT("Modes 1-4 are log2(1+x)/16 encoded for display; mode 5 is linear."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<float> CVarGSLightingBlend(
	TEXT("gs.LightingBlend"),
	0.0f,
	TEXT("Blend factor for scene directional lighting on Gaussian splats.\n")
	TEXT(" 0: Unlit (original appearance, default)\n")
	TEXT(" 1: Fully lit by the dominant directional light\n")
	TEXT("Intermediate values blend between the two."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<float> CVarGSAmbientIntensity(
	TEXT("gs.AmbientIntensity"),
	0.1f,
	TEXT("Ambient light intensity added to shadowed side of splats when gs.LightingBlend > 0."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<float> CVarGSLightIntensityScale(
	TEXT("gs.LightIntensityScale"),
	0.1f,
	TEXT("Global sensitivity mapping a scene light's intensity to its effect on splats.\n")
	TEXT("Per-light response = 1 - exp(-intensity * this scale), so it never blows out (asymptotes to 1).\n")
	TEXT("Local-light intensity is in ~candela (UE unit factor divided out); directional in ~lux.\n")
	TEXT("Raise to make lights reach full brightness at lower intensities; lower to soften. Default 0.1."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<float> CVarGSLightResponseCeiling(
	TEXT("gs.LightResponseCeiling"),
	1.0f,
	TEXT("Per-light contribution ceiling. Per-light response = Ceiling * (1 - exp(-intensity * gs.LightIntensityScale)),\n")
	TEXT("so 1.0 reproduces the original behavior (asymptotes to 1) and raising it lets strong lights keep\n")
	TEXT("getting brighter past that point instead of plateauing. Default 1.0."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<float> CVarGSRelightRatioMin(
	TEXT("gs.RelightRatioMin"),
	0.6f,
	TEXT("Clamp floor for the brightness-preserving relight ratio (actual shading / neutral-orientation\n")
	TEXT("shading). Splat colors are already a baked, lit appearance, not raw albedo — this ratio lets\n")
	TEXT("directional shading vary the baked brightness within a modest range instead of multiplying\n")
	TEXT("by the absolute light level (which double-shades and darkens). Default 0.6."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<float> CVarGSRelightRatioMax(
	TEXT("gs.RelightRatioMax"),
	1.6f,
	TEXT("Clamp ceiling for the brightness-preserving relight ratio. See gs.RelightRatioMin. Default 1.6."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<int32> CVarGSUseRelightRatio(
	TEXT("gs.UseRelightRatio"),
	1,
	TEXT("1: Brightness-preserving ratio blend (default) — multiplies by actual/neutral shading,\n")
	TEXT("   clamped to [gs.RelightRatioMin, gs.RelightRatioMax], avoiding double-shading the splat's\n")
	TEXT("   already-baked color.\n")
	TEXT("0: Old direct multiply by the absolute light level. Simpler, but darkens/blows out more\n")
	TEXT("   easily since it ignores that the baked color already has some lighting in it. Kept as\n")
	TEXT("   an opt-out for comparison."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<int32> CVarGSNormalConfidenceFade(
	TEXT("gs.NormalConfidenceFade"),
	1,
	TEXT("GeometryMode 2 (per-splat normal) only. 1 (default): weight the per-splat normal\n")
	TEXT("accumulation by each splat's confidence — low for near-isotropic splats whose 'thinnest\n")
	TEXT("axis' choice is essentially a coin-flip, which otherwise causes salt-and-pepper dark\n")
	TEXT("speckling where unstable normals randomly face away from lights. Pixels dominated by\n")
	TEXT("low-confidence splats fade back toward the unlit splat color instead of trusting a noisy\n")
	TEXT("normal. 0: ignore confidence (original behavior)."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<int32> CVarGSShadowMode(
	TEXT("gs.ShadowMode"),
	0,
	TEXT("Per-light shadows for splats. A light only casts a shadow if BOTH this is non-zero AND\n")
	TEXT("the light's own 'Cast Shadows' property is enabled (same checkbox the rest of the scene\n")
	TEXT("respects) — up to gs.ShadowMaxLights of them, prioritized the same way as the existing\n")
	TEXT("light-selection scoring (directional first, then closest/brightest locals).\n")
	TEXT(" 0: Off (default). No shadow rendering/sampling at all.\n")
	TEXT(" 1: Proxy Mesh — shadow depth comes from a user-designated mesh (NOT YET IMPLEMENTED).\n")
	TEXT(" 2: Splat Self-Shadow — shadow depth comes from the splats' own geometry (NOT YET IMPLEMENTED)."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<int32> CVarGSShadowMaxLights(
	TEXT("gs.ShadowMaxLights"),
	4,
	TEXT("Max simultaneous shadow-casting lights (fixed shader resource budget). If more lights are\n")
	TEXT("eligible (gs.ShadowMode on + their own 'Cast Shadows' enabled) than this, the same\n")
	TEXT("priority order used for general light selection picks which ones actually get a shadow\n")
	TEXT("this frame. Default 4."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<float> CVarGSLightWrap(
	TEXT("gs.LightWrap"),
	0.3f,
	TEXT("Wrap lighting (Penner/Half-Lambert generalization): ndl_wrapped = (ndl + Wrap) / (1 + Wrap),\n")
	TEXT("applied to the two-sided abs(NdotL) term. Lifts the floor near the terminator so it fades\n")
	TEXT("into shadow instead of bottoming out at a hard zero (which otherwise looks like a stark dark\n")
	TEXT("ring/line with bright lobes on both sides, since splats use two-sided abs(NdotL) to stay\n")
	TEXT("robust to ambiguous/flipped normals). 0 = off (original hard terminator). Default 0.3."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<int32> CVarGSDepthProximityWeighting(
	TEXT("gs.DepthProximityWeighting"),
	0,
	TEXT("GeometryMode 2 (per-splat normal) only. 1: re-rasterizes splats a SECOND time, after the\n")
	TEXT("depth-accumulation MRT from the first pass is fully resolved, weighting each splat's normal\n")
	TEXT("contribution by how close its own depth is to the now-known reconstructed surface depth.\n")
	TEXT("Suppresses background/secondary splats (visible through a semi-transparent foreground\n")
	TEXT("splat) from polluting that pixel's normal with a stable-but-wrong orientation — a different\n")
	TEXT("failure mode than gs.NormalConfidenceFade's near-isotropic-axis noise. Roughly doubles the\n")
	TEXT("splat-draw cost for GeometryMode 2. 0 (default): off, normal accumulation happens once in\n")
	TEXT("the same pass as depth/color."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<float> CVarGSDepthProximitySigma(
	TEXT("gs.DepthProximitySigma"),
	0.05f,
	TEXT("Depth-similarity sigma (fraction of view depth) for gs.DepthProximityWeighting's per-splat\n")
	TEXT("proximity falloff. Smaller = more aggressively rejects splats whose depth differs from the\n")
	TEXT("resolved surface depth. Default 0.05 (5%% of depth)."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<int32> CVarGSNormalSmoothRadius(
	TEXT("gs.NormalSmoothRadius"),
	2,
	TEXT("Bilateral depth-smoothing kernel radius (pixels) for screen-space normal reconstruction.\n")
	TEXT("Removes the per-splat depth scatter that makes lit surfaces look like a noisy SfM mesh.\n")
	TEXT("0 = off (raw, noisy). Higher = smoother normals but softer detail and more cost. Default 2."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<float> CVarGSNormalSmoothFrac(
	TEXT("gs.NormalSmoothDepthSigma"),
	0.02f,
	TEXT("Bilateral depth-similarity sigma as a fraction of view-space depth (edge preservation).\n")
	TEXT("Smaller = preserves silhouettes/creases more (less cross-surface bleed); larger = smoother.\n")
	TEXT("Default 0.02 (2%% of depth)."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<int32> CVarGSNormalSampleStep(
	TEXT("gs.NormalSampleStep"),
	3,
	TEXT("Central-difference baseline (pixels) for screen-space normal reconstruction.\n")
	TEXT("The normal is the surface slope between samples this far apart, so a wider step averages\n")
	TEXT("out per-splat depth waviness into the overall surface slope (free — no extra taps).\n")
	TEXT("1 = sharpest/noisiest. Higher = smoother, flatter normals. Default 3."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<int32> CVarGSLightingGeometryMode(
	TEXT("gs.LightingGeometryMode"),
	0,
	TEXT("Geometry source for screen-space lighting reconstruction.\n")
	TEXT(" 0: Splat depth accumulation (default). Needs no scene setup, but per-splat depth scatter\n")
	TEXT("    makes normals noisy (mitigated by gs.NormalSampleStep/NormalSmoothRadius).\n")
	TEXT(" 1: Proxy mesh CustomDepth. Clean, noise-free normals, but requires a hidden mesh co-located\n")
	TEXT("    with the splat with \"Render CustomDepth Pass\" enabled on its component.\n")
	TEXT(" 2: Per-splat analytic normal. Each splat's own thinnest covariance axis, alpha-weighted\n")
	TEXT("    and accumulated across overlapping splats — no external mesh, no screen-space depth\n")
	TEXT("    noise. Reuses mode 0's depth accumulation for world position (only the normal source\n")
	TEXT("    changes), so it still benefits from gs.NormalSmoothRadius-free clean normals."),
	ECVF_RenderThreadSafe);

TAutoConsoleVariable<int32> CVarGSLightingDebugView(
	TEXT("gs.LightingDebugView"),
	0,
	TEXT("Diagnostic visualization for the active GeometryMode's reconstructed geometry. Bypasses\n")
	TEXT("the per-light loop and tonemap entirely so the raw data is visible undistorted; works even\n")
	TEXT("with gs.LightingBlend at 0 or no scene lights.\n")
	TEXT(" 0: Off (default).\n")
	TEXT(" 1: Show the reconstructed per-pixel world-space normal as an RGB color (normal*0.5+0.5,\n")
	TEXT("    standard normal-map encoding). Pixels with no valid geometry show the plain unlit\n")
	TEXT("    splat color, which doubles as a coverage/confidence-fade visualization."),
	ECVF_RenderThreadSafe);

// Export for other modules
int32 GGaussianSplatShowClusterBounds = 0;

static FGaussianSceneLighting GatherSceneLighting(const FSceneView* SceneView)
{
	FGaussianSceneLighting Out;
	Out.LightingBlend = CVarGSLightingBlend.GetValueOnRenderThread();
	Out.DebugView = CVarGSLightingDebugView.GetValueOnRenderThread();
	// Fast-out when lighting is disabled, UNLESS a debug view is requested — the debug view
	// bypasses the per-light loop entirely, so it still needs GeometryMode/depth gathered below
	// even with LightingBlend at 0.
	if (Out.LightingBlend <= 0.f && Out.DebugView == 0) return Out;

	const float Amb = CVarGSAmbientIntensity.GetValueOnRenderThread();
	Out.AmbientColor = FVector3f(Amb, Amb, Amb);
	Out.IntensityScale = CVarGSLightIntensityScale.GetValueOnRenderThread();
	Out.ResponseCeiling = CVarGSLightResponseCeiling.GetValueOnRenderThread();
	Out.LightWrap = FMath::Clamp(CVarGSLightWrap.GetValueOnRenderThread(), 0.f, 1.f);
	Out.RelightRatioMin = CVarGSRelightRatioMin.GetValueOnRenderThread();
	Out.RelightRatioMax = FMath::Max(CVarGSRelightRatioMax.GetValueOnRenderThread(), Out.RelightRatioMin);
	Out.bUseRelightRatio = CVarGSUseRelightRatio.GetValueOnRenderThread() != 0;
	Out.bUseNormalConfidenceFade = CVarGSNormalConfidenceFade.GetValueOnRenderThread() != 0;
	Out.bDepthProximityWeighting = CVarGSDepthProximityWeighting.GetValueOnRenderThread() != 0;
	Out.DepthProximitySigma = FMath::Max(CVarGSDepthProximitySigma.GetValueOnRenderThread(), 0.f);
	Out.NormalSmoothRadius = FMath::Clamp(CVarGSNormalSmoothRadius.GetValueOnRenderThread(), 0, 6);
	Out.NormalSmoothFrac = FMath::Max(CVarGSNormalSmoothFrac.GetValueOnRenderThread(), 0.f);
	Out.NormalSampleStep = FMath::Clamp(CVarGSNormalSampleStep.GetValueOnRenderThread(), 1, 16);
	Out.GeometryMode = FMath::Clamp(CVarGSLightingGeometryMode.GetValueOnRenderThread(), 0, 2);
	Out.ShadowMode = FMath::Clamp(CVarGSShadowMode.GetValueOnRenderThread(), 0, 2);
	if (SceneView)
	{
		Out.InvDeviceZToWorldZTransform = SceneView->InvDeviceZToWorldZTransform;
	}

	FScene* Scene = SceneView ? static_cast<FScene*>(SceneView->Family->Scene) : nullptr;
	if (!Scene) return Out;

	// Helpers: separate a light's HDR color into tint (hue, max-component = 1) and
	// intensity (the discarded brightness). GetColor() bakes intensity into the color,
	// so its max-component tracks the light's editor Intensity property.
	auto MaxComp = [](FLinearColor C) -> float {
		return FMath::Max3(C.R, C.G, C.B);
	};
	auto NormalizeTint = [&](FLinearColor C) -> FVector3f {
		float M = MaxComp(C);
		return (M > 0.f) ? FVector3f(C.R/M, C.G/M, C.B/M) : FVector3f(1,1,1);
	};

	// Slot 0: dominant directional light
	if (Scene->SimpleDirectionalLight && Scene->SimpleDirectionalLight->Proxy)
	{
		const FLightSceneProxy* P = Scene->SimpleDirectionalLight->Proxy;
		const FLinearColor C = P->GetColor();
		FGaussianLight& L = Out.Lights[Out.NumLights++];
		L.Type      = 0.f;
		// GetDirection() returns the direction the light travels (away from source).
		// Store as-is; the shader negates it to get the "toward light" vector.
		L.Direction = FVector3f(P->GetDirection());
		L.Color     = NormalizeTint(C);
		L.Intensity = MaxComp(C);
		L.InvRadius = 0.f;  // no distance falloff for directional
		L.bCastsShadow = P->CastsDynamicShadow();
	}

	// Local lights: iterate Scene->Lights (TSparseArray<FLightSceneInfoCompact>)
	// Each compact entry has LightSceneInfo->Proxy.
	// Gather candidates, score by proximity to camera, take top (MaxLights - NumLights).
	struct FLightCandidate
	{
		FGaussianLight Light;
		float Score;  // higher = more important; used for sorting
	};
	TArray<FLightCandidate, TInlineAllocator<32>> Candidates;

	const FVector CameraOrigin = SceneView->ViewMatrices.GetViewOrigin();
	const int32 SlotsLeft = FGaussianSceneLighting::MaxLights - Out.NumLights;

	for (const FLightSceneInfoCompact& Compact : Scene->Lights)
	{
		if (!Compact.LightSceneInfo || !Compact.LightSceneInfo->Proxy) continue;
		const FLightSceneProxy* P = Compact.LightSceneInfo->Proxy;

		const uint8 LT = P->GetLightType();

		// Skip directional (already handled above) and unsupported types
		if (LT == LightType_Directional) continue;
		if (LT != LightType_Point && LT != LightType_Spot) continue;

		// GetRadius() returns the attenuation radius (FLT_MAX for directional, never reached)
		const float Radius = P->GetRadius();
		if (Radius <= 0.f) continue;

		// Cull: light's sphere must overlap the camera position (conservative)
		// Use GetOrigin() for world-space light position (FVector)
		const FVector LightPos = P->GetOrigin();
		const float DistToCamera = FVector::Dist(LightPos, CameraOrigin);
		if (DistToCamera > Radius + 1000.f) continue;  // broad-phase cull (1000cm slack)

		const FLinearColor LightColor = P->GetColor();
		FGaussianLight GL;
		GL.Position  = FVector3f(LightPos);
		GL.InvRadius = (Radius > 0.f) ? (1.f / Radius) : 0.f;
		GL.Color     = NormalizeTint(LightColor);
		// UE bakes a cm²→m² unit factor (100*100) into local-light GetColor() for candela
		// intensity (see PointLightComponent.cpp ComputeLightBrightness). Divide it out so the
		// captured intensity is back in ~candela, comparable to the directional's lux magnitude,
		// letting a single gs.LightIntensityScale serve both. (Lumens/spot carry an extra
		// cone/4PI factor that the scale CVar absorbs.)
		GL.Intensity = MaxComp(LightColor) / (100.f * 100.f);
		GL.bCastsShadow = P->CastsDynamicShadow();

		if (LT == LightType_Spot)
		{
			GL.Type = 2.f;
			// GetDirection() is the forward axis of the spot cone
			GL.Direction = FVector3f(P->GetDirection());
			// GetOuterConeAngle() returns the outer half-angle in radians (from FSpotLightSceneProxy)
			// Use GetLightShaderParameters to access SpotAngles: X=CosOuter, Y=InvCosConeDifference
			FLightRenderParameters LRP;
			P->GetLightShaderParameters(LRP);
			// SpotAngles.X = CosOuterCone
			// SpotAngles.Y = 1 / (CosInner - CosOuter)  =>  CosInner = CosOuter + 1/Y
			const float CosOuter = LRP.SpotAngles.X;
			const float CosInner = (LRP.SpotAngles.Y > 0.f)
				? CosOuter + (1.f / LRP.SpotAngles.Y)
				: CosOuter;
			GL.CosOuter = CosOuter;
			GL.CosInner = FMath::Clamp(CosInner, CosOuter, 1.f);
		}
		else  // LightType_Point
		{
			GL.Type     = 1.f;
			GL.Direction = FVector3f(0,0,-1);  // unused for point
			GL.CosOuter = -1.f;                // no cone
			GL.CosInner =  1.f;
		}

		// Score: brighter (larger radius) and closer = higher priority
		const float Score = Radius / FMath::Max(DistToCamera, 1.f);
		Candidates.Add({GL, Score});
	}

	// Sort candidates descending by score, take top SlotsLeft
	Candidates.Sort([](const FLightCandidate& A, const FLightCandidate& B){ return A.Score > B.Score; });

	const int32 NumToAdd = FMath::Min(Candidates.Num(), SlotsLeft);
	for (int32 i = 0; i < NumToAdd; ++i)
	{
		Out.Lights[Out.NumLights++] = Candidates[i].Light;
	}

	// Shadow slot assignment (gs.ShadowMode): Out.Lights[] is already in priority order
	// (directional first, then locals by the same closest/brightest score used above), so the
	// first MaxShadowLights lights with bCastsShadow true get a slot — no separate ranking needed.
	if (Out.ShadowMode != 0)
	{
		const int32 MaxShadowLights = FMath::Clamp(
			CVarGSShadowMaxLights.GetValueOnRenderThread(), 0, FGaussianSceneLighting::MaxShadowLights);
		for (int32 i = 0; i < Out.NumLights && Out.NumShadowLights < MaxShadowLights; ++i)
		{
			if (Out.Lights[i].bCastsShadow)
			{
				Out.Lights[i].ShadowSlot = Out.NumShadowLights++;
			}
		}
	}

	// gs.ShadowMode == 1 (Proxy Mesh): match this frame's game-thread shadow-capture snapshot
	// against the lights that were just assigned a ShadowSlot above. There's no shared stable
	// identity between a game-thread ULightComponent and a render-thread FLightSceneProxy, so
	// matching is approximate — position for local lights (point/spot), direction for directional
	// (there's normally only one, so this is unambiguous in practice).
	int32 SnapshotCount = -1;   // -1 = not queried (ShadowMode != 1 or no slots assigned)
	int32 MatchedCount = 0;
	if (Out.ShadowMode == 1 && Out.NumShadowLights > 0)
	{
		TArray<FNanoGSShadowRenderData> Snapshot;
		if (FGaussianSplatViewExtension* VE = FGaussianSplatViewExtension::Get())
		{
			VE->GetShadowCaptureSnapshot(Snapshot);
		}
		SnapshotCount = Snapshot.Num();

		constexpr float PosEpsilon = 5.f;       // cm
		constexpr float DirCosEpsilon = 0.999f; // ~2.5 degrees

		for (int32 i = 0; i < Out.NumLights; ++i)
		{
			FGaussianLight& L = Out.Lights[i];
			if (L.ShadowSlot < 0) continue;

			for (const FNanoGSShadowRenderData& RD : Snapshot)
			{
				const bool bMatch = (L.Type < 0.5f)
					? (RD.bIsDirectional && FVector::DotProduct(FVector(L.Direction), RD.LightDirection) > DirCosEpsilon)
					: (!RD.bIsDirectional && FVector::Dist(FVector(L.Position), RD.LightWorldPos) < PosEpsilon);
				if (bMatch)
				{
					Out.ShadowCaptures[L.ShadowSlot] = RD;
					MatchedCount++;
					break;
				}
			}
		}
	}

	// Phase-A/B verification: log only when the shadow-casting set (or match outcome) changes, so
	// this is observable without spamming every frame.
	{
		static int32 LastLoggedMode = -1;
		static int32 LastLoggedCount = -1;
		static int32 LastLoggedSnapshot = -2;
		static int32 LastLoggedMatched = -1;
		if (Out.ShadowMode != LastLoggedMode || Out.NumShadowLights != LastLoggedCount
			|| SnapshotCount != LastLoggedSnapshot || MatchedCount != LastLoggedMatched)
		{
			UE_LOG(LogTemp, Log, TEXT("[NanoGS] gs.ShadowMode=%d: %d/%d lights assigned a shadow slot; capture snapshot has %d entries; %d matched"),
				Out.ShadowMode, Out.NumShadowLights, Out.NumLights, SnapshotCount, MatchedCount);
			LastLoggedMode = Out.ShadowMode;
			LastLoggedCount = Out.NumShadowLights;
			LastLoggedSnapshot = SnapshotCount;
			LastLoggedMatched = MatchedCount;
		}
	}

	return Out;
}

// Helper to get the renderer module
static IRendererModule& GetRendererModuleRef()
{
	return FModuleManager::GetModuleChecked<IRendererModule>("Renderer");
}

void FNanoGSModule::StartupModule()
{
	// Register the shader directory so we can use our custom shaders
	FString PluginShaderDir = FPaths::Combine(IPluginManager::Get().FindPlugin(TEXT("NanoGS"))->GetBaseDir(), TEXT("Shaders"));
	AddShaderSourceDirectoryMapping(TEXT("/Plugin/NanoGS"), PluginShaderDir);

	// Allocate the global accumulator (buffers are created lazily on first render)
	GlobalAccumulator = MakeUnique<FGaussianGlobalAccumulator>();

	// Register post-opaque render delegate for rendering
	PostOpaqueRenderDelegateHandle = GetRendererModuleRef().RegisterPostOpaqueRenderDelegate(
		FPostOpaqueRenderDelegate::CreateRaw(this, &FNanoGSModule::OnPostOpaqueRender_RenderThread));

	// Defer view extension creation until GEngine is valid
	// (StartupModule runs before GEngine is initialized, causing an ensure failure)
	PostEngineInitDelegateHandle = FCoreDelegates::OnPostEngineInit.AddRaw(this, &FNanoGSModule::OnPostEngineInit);

	UE_LOG(LogTemp, Log, TEXT("GaussianSplatting module started. Shader directory: %s"), *PluginShaderDir);
}

void FNanoGSModule::OnPostEngineInit()
{
	// Apply the persisted NanoGS Lighting project settings to the gs.* CVars now that the module
	// (and its CVars) are fully registered. This makes the Project Settings page authoritative at launch.
	if (const UNanoGSLightingSettings* Settings = GetDefault<UNanoGSLightingSettings>())
	{
		Settings->ApplyToCVars();
	}

	// Skip view extension creation during cooking/commandlet — GEngine is null in those contexts
	if (IsRunningCommandlet() || !GEngine)
	{
		UE_LOG(LogTemp, Log, TEXT("GaussianSplatting: Skipping ViewExtension creation (commandlet/cook mode)"));
		return;
	}

	// Now GEngine is valid — safe to create the view extension
	ViewExtension = FSceneViewExtensions::NewExtension<FGaussianSplatViewExtension>();

	if (!ViewExtension.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("GaussianSplatting: Failed to create ViewExtension!"));
	}
	else
	{
		UE_LOG(LogTemp, Log, TEXT("GaussianSplatting: ViewExtension created successfully (deferred init)"));
	}
}

void FNanoGSModule::OnPostOpaqueRender_RenderThread(FPostOpaqueRenderParameters& Parameters)
{
	FGaussianSplatViewExtension* Ext = FGaussianSplatViewExtension::Get();
	if (!Ext || !Parameters.GraphBuilder || !Parameters.View)
	{
		return;
	}

	FRDGBuilder& GraphBuilder = *Parameters.GraphBuilder;
	const FSceneView* SceneView = reinterpret_cast<const FSceneView*>(Parameters.View);
	FRDGTexture* ColorTexture = Parameters.ColorTexture;
	if (!ColorTexture)
	{
		return;
	}

	FAirSimNativeGeerView NativeGeerView;
	const FRenderTarget* FamilyRenderTarget = SceneView->Family != nullptr
		? SceneView->Family->RenderTarget
		: nullptr;
	const bool bNativeGeerRegistered = AirSimFindNativeGeerView_RenderThread(FamilyRenderTarget, NativeGeerView);
	const FIntPoint FamilyExtent = FamilyRenderTarget != nullptr
		? FamilyRenderTarget->GetSizeXY()
		: FIntPoint::ZeroValue;
	const FIntPoint RenderViewExtent = static_cast<const FViewInfo&>(*SceneView).ViewRect.Size();
	const bool bNativeDimensionsMatch = bNativeGeerRegistered &&
		FamilyExtent == NativeGeerView.output_extent &&
		SceneView->UnscaledViewRect.Size() == NativeGeerView.output_extent &&
		RenderViewExtent == NativeGeerView.output_extent;
	const bool bNativeRaymapReady = bNativeGeerRegistered && NativeGeerView.raymap.IsValid() &&
		NativeGeerView.raymap->ready && NativeGeerView.raymap->srv.IsValid();
	const bool bNativeRaymapDimensionsMatch = bNativeRaymapReady &&
		NativeGeerView.raymap->width == static_cast<uint32>(NativeGeerView.output_extent.X) &&
		NativeGeerView.raymap->height == static_cast<uint32>(NativeGeerView.output_extent.Y);
	const bool bNativeGeerView = bNativeGeerRegistered && bNativeDimensionsMatch &&
		bNativeRaymapDimensionsMatch && NativeGeerView.central && NativeGeerView.raymap->central;

	if (bNativeGeerRegistered &&
		!GNanoGSLoggedNativeGeerRegistrations.Contains(NativeGeerView.registration_serial))
	{
		GNanoGSLoggedNativeGeerRegistrations.Add(NativeGeerView.registration_serial);

		UE_LOG(LogTemp, Log,
			TEXT("[NanoGS][NativeGEER][GateA] matched camera=%s target=%p serial=%llu ")
			TEXT("registered=%dx%d family=%dx%d view_rect=%dx%d raymap=%ux%u ready=%s central=%s status=%s"),
			*NativeGeerView.camera_name, FamilyRenderTarget,
			static_cast<unsigned long long>(NativeGeerView.registration_serial),
			NativeGeerView.output_extent.X, NativeGeerView.output_extent.Y,
			FamilyExtent.X, FamilyExtent.Y,
			RenderViewExtent.X, RenderViewExtent.Y,
			NativeGeerView.raymap.IsValid() ? NativeGeerView.raymap->width : 0,
			NativeGeerView.raymap.IsValid() ? NativeGeerView.raymap->height : 0,
			bNativeRaymapReady ? TEXT("true") : TEXT("false"),
			NativeGeerView.central ? TEXT("true") : TEXT("false"),
			bNativeGeerView ? TEXT("OK") : TEXT("MISMATCH"));
		if (!bNativeGeerView)
		{
			UE_LOG(LogTemp, Error,
				TEXT("[NanoGS][NativeGEER][GateA] camera=%s matched the target identity but its ")
				TEXT("output/raymap dimensions, readiness, or centrality are invalid; native rendering will output black"),
				*NativeGeerView.camera_name);
		}
	}

	FShaderResourceViewRHIRef NativeGeerRaymap;
	FIntPoint NativeGeerRaymapSize = FIntPoint::ZeroValue;
	if (bNativeGeerRegistered)
	{
		// NativeGEER Scene is explicitly splat-only in 3c-1. Clear UE mesh/sky output before any
		// possible early return so a bad configuration can never masquerade as a plausible fallback.
		AddClearRenderTargetPass(GraphBuilder, ColorTexture, FLinearColor::Black);
		if (!bNativeGeerView)
		{
			return;
		}
		NativeGeerRaymap = NativeGeerView.raymap->srv;
		NativeGeerRaymapSize = NativeGeerView.output_extent;
	}

	TArray<FGaussianSplatSceneProxy*> Proxies;
	Ext->GetRegisteredProxies(Proxies);

	if (Proxies.Num() == 0)
	{
		return;
	}

	// Sort proxies back-to-front for correct depth ordering
	if (Proxies.Num() > 1)
	{
		FVector CameraPosition = SceneView->ViewMatrices.GetViewOrigin();
		Proxies.Sort([CameraPosition](const FGaussianSplatSceneProxy& A, const FGaussianSplatSceneProxy& B)
		{
			float DistA = FVector::DistSquared(CameraPosition, A.GetBounds().Origin);
			float DistB = FVector::DistSquared(CameraPosition, B.GetBounds().Origin);
			return DistA > DistB;
		});
	}

	FRDGTexture* DepthTexture = Parameters.DepthTexture;
	FRDGTexture* VelocityTexture = Parameters.VelocityTexture;

	int32 DebugMode = CVarShowClusterBounds.GetValueOnRenderThread();

	// Gather scene lights up front (render-thread access to Scene->Lights). Needed before Pass 1
	// so we can decide whether to allocate + write the depth-accumulation MRT this frame.
	FGaussianSceneLighting SceneLighting = GatherSceneLighting(SceneView);
	if (bNativeGeerView)
	{
		// Gate B1 validates ray evaluation in isolation. The existing compositor reconstructs
		// pinhole view-Z and is not a valid consumer until the later range/raymap gate.
		SceneLighting.LightingBlend = 0.0f;
		SceneLighting.DebugView = 0;
		DebugMode = 0;
	}

	// Output the alpha-weighted depth MRT only when lighting actually needs it. RT2 requires RT1
	// (velocity) to be bound contiguously, so we also require VelocityTexture. Debug viz disables it.
	// GeometryMode 0 uses it for normals AND position; GeometryMode 2 reuses it for position only
	// (its own normal comes from NormalAccum below). GeometryMode 1 (proxy mesh CustomDepth) doesn't
	// use this MRT at all — skip allocating it.
	const bool bWantsLighting = (SceneLighting.LightingBlend > 0.f) && (SceneLighting.NumLights > 0);
	const bool bWantsDebugView = (SceneLighting.DebugView != 0);
	const bool bLightingActive = !bNativeGeerView && (bWantsLighting || bWantsDebugView)
	                          && (VelocityTexture != nullptr)
	                          && (DebugMode == 0);
	const bool bOutputDepth = bLightingActive && (SceneLighting.GeometryMode == 0 || SceneLighting.GeometryMode == 2);
	// GeometryMode 2 only: alpha-weighted per-splat analytic normal accumulation.
	const bool bOutputNormal = bLightingActive && (SceneLighting.GeometryMode == 2);
	const bool bUseNormalConfidenceFade = SceneLighting.bUseNormalConfidenceFade;
	// gs.DepthProximityWeighting: whether Pass 1b (a second, normal-only re-rasterization after
	// Pass 1's depth accum is resolved) should run this frame. Decided properly once bAllNanite is
	// known below (Pass 1b only supports the Nanite-compacted indirect-draw path — see Pass 1b setup).
	bool bDepthProximityPass = bOutputNormal && SceneLighting.bDepthProximityWeighting;

	// GeometryMode 1: pull the proxy mesh's CustomDepth out of the scene textures the renderer
	// already produced this frame (populated only if at least one primitive has "Render CustomDepth
	// Pass" enabled). Null when unavailable — CompositeToSceneColor falls back to disabling lighting.
	FRDGTexture* CustomDepthTexture = nullptr;
	if (!bNativeGeerView && SceneLighting.GeometryMode == 1 && Parameters.SceneTexturesUniformParams)
	{
		// SceneTexturesUniformParams is a raw TRDGUniformBuffer<FSceneTextureUniformParameters>*
		// (see TRDGUniformBufferRef in RenderGraphFwd.h) — GetContents() reaches the actual
		// parameter struct; a single "->" would resolve against the wrapper class itself.
		CustomDepthTexture = Parameters.SceneTexturesUniformParams->GetContents()->CustomDepthTexture;
	}

	// Resolve the frame's depth-write policy before RDG declares depth access and before the draw
	// chooses its depth-stencil PSO. Auto mode is deliberately based on the same CPU-visible proxy
	// tests used by the global path below. GEER splats still test against opaque scene depth, but
	// they must not reject one another by writing their centre depth into the hardware depth buffer.
	const int32 CurrentGeerMode = CVarGeerEval.GetValueOnRenderThread();
	bool bAnyGeerVisibleForDepth = false;
	for (FGaussianSplatSceneProxy* Proxy : Proxies)
	{
		if (!Proxy) continue;
		if (&Proxy->GetScene() != SceneView->Family->Scene) continue;
		if (!Proxy->IsShown(SceneView)) continue;

		const FBoxSphereBounds& Bounds = Proxy->GetBounds();
		if (!bNativeGeerView && !SceneView->ViewFrustum.IntersectBox(Bounds.Origin, Bounds.BoxExtent)) continue;

		FGaussianSplatGPUResources* GPUResources = Proxy->GetGPUResources();
		if (!GPUResources || !GPUResources->IsValid()) continue;

		const bool bProxyGeer = (CurrentGeerMode == 2) ||
			(CurrentGeerMode == 1 && Proxy->IsGeerSplat());
		if (bProxyGeer)
		{
			bAnyGeerVisibleForDepth = true;
			break;
		}
	}

	const int32 GeerDepthWriteMode = CVarGeerDepthWrite.GetValueOnRenderThread();
	const bool bWriteSplatDepth = !bNativeGeerView && ((GeerDepthWriteMode > 0) ||
		(GeerDepthWriteMode < 0 && !bAnyGeerVisibleForDepth));

	// Create intermediate render target for sRGB-space alpha blending.
	// Gaussian splatting trains in sRGB space, so blending must happen in sRGB space
	// to produce correct colors. After compositing, we convert sRGB→linear for SceneColor.
	// Residual modes 1-4 add integer-like fragment counts. FP16 stops representing unit increments
	// exactly above 2048 and can saturate in a dense checkpoint, so use FP32 only while one of those
	// explicitly requested diagnostics is active. Mode 0 retains the normal format and cost.
	const int32 GeerResidualMode = CVarGeerResidualDebug.GetValueOnRenderThread();
	const EPixelFormat IntermediateFormat = (GeerResidualMode >= 1 && GeerResidualMode <= 4)
		? PF_A32B32G32R32F
		: PF_FloatRGBA;
	FRDGTextureDesc IntermediateDesc = FRDGTextureDesc::Create2D(
		ColorTexture->Desc.Extent,
		IntermediateFormat,  // Need alpha channel for accumulation tracking
		FClearValueBinding(FLinearColor::Transparent),
		TexCreate_RenderTargetable | TexCreate_ShaderResource);
	FRDGTexture* IntermediateTexture = GraphBuilder.CreateTexture(IntermediateDesc, TEXT("GaussianSplatIntermediateRT"));

	// Alpha-weighted view-space depth accumulation target (RG32F: viewZ*alpha, alpha).
	// Only allocated when lighting needs it; FP32 for precision at city-scale depths.
	FRDGTexture* DepthAccumTexture = nullptr;
	if (bOutputDepth)
	{
		FRDGTextureDesc DepthAccumDesc = FRDGTextureDesc::Create2D(
			ColorTexture->Desc.Extent,
			PF_G32R32F,
			FClearValueBinding(FLinearColor(0, 0, 0, 0)),
			TexCreate_RenderTargetable | TexCreate_ShaderResource);
		DepthAccumTexture = GraphBuilder.CreateTexture(DepthAccumDesc, TEXT("GaussianSplatDepthAccumRT"));
	}

	// Alpha-weighted per-splat analytic-normal accumulation target (RGBA16F: normal*alpha, alpha).
	// GeometryMode 2 only. Half-float is plenty of precision for a unit normal.
	FRDGTexture* NormalAccumTexture = nullptr;
	if (bOutputNormal)
	{
		FRDGTextureDesc NormalAccumDesc = FRDGTextureDesc::Create2D(
			ColorTexture->Desc.Extent,
			PF_FloatRGBA,
			FClearValueBinding(FLinearColor(0, 0, 0, 0)),
			TexCreate_RenderTargetable | TexCreate_ShaderResource);
		NormalAccumTexture = GraphBuilder.CreateTexture(NormalAccumDesc, TEXT("GaussianSplatNormalAccumRT"));
	}

	// Pass 1: Render splats to intermediate RT (sRGB blending)
	FRenderTargetParameters* Pass1Parameters = GraphBuilder.AllocParameters<FRenderTargetParameters>();
	Pass1Parameters->RenderTargets[0] = FRenderTargetBinding(IntermediateTexture, ERenderTargetLoadAction::EClear);
	// Bind velocity texture for TAA/TSR motion vector output
	if (VelocityTexture)
	{
		Pass1Parameters->RenderTargets[1] = FRenderTargetBinding(VelocityTexture, ERenderTargetLoadAction::ELoad);
	}
	// RT2: depth accumulation (requires RT1 bound, guaranteed by bOutputDepth)
	if (bOutputDepth)
	{
		Pass1Parameters->RenderTargets[2] = FRenderTargetBinding(DepthAccumTexture, ERenderTargetLoadAction::EClear);
	}
	// RT3: normal accumulation (GeometryMode 2 binds both RT2 depth accum and RT3 normal accum).
	// When gs.DepthProximityWeighting is active, Pass 1b re-clears and overwrites this with a
	// depth-proximity-weighted version once DepthAccum is resolved — Pass 1's write here is
	// then wasted but harmless (cheap relative to the second draw call Pass 1b already costs).
	if (bOutputNormal)
	{
		Pass1Parameters->RenderTargets[3] = FRenderTargetBinding(NormalAccumTexture, ERenderTargetLoadAction::EClear);
	}
	if (DepthTexture && !bNativeGeerView)
	{
		// Read-only mode still permits the depth test and stencil write; it removes only splat
		// centre-depth writes so GEER splats cannot reject one another through hardware depth.
		Pass1Parameters->RenderTargets.DepthStencil = FDepthStencilBinding(
			DepthTexture,
			ERenderTargetLoadAction::ELoad,
			ERenderTargetLoadAction::ELoad,
			bWriteSplatDepth
				? FExclusiveDepthStencil::DepthWrite_StencilWrite
				: FExclusiveDepthStencil::DepthRead_StencilWrite
		);
	}

	if (!GlobalAccumulator.IsValid())
	{
		return;
	}

	//------------------------------------------------------------------
	// GLOBAL ACCUMULATOR PATH: Phase 1 (per-proxy CalcViewData) +
	// Phase 2 (single CalcDistances + RadixSort) + single DrawSplats
	//------------------------------------------------------------------

		// Build the list of visible proxies and compute total splat count (CPU-side)
		struct FProxyRenderInfo
		{
			FGaussianSplatSceneProxy* Proxy;
			FMatrix LocalToWorld;
			uint32 GlobalBaseOffset;
			bool bUseLODRendering;
			float DistanceToCamera;  // For budget priority sorting (closer = higher priority)
		};
		TArray<FProxyRenderInfo> VisibleProxies;
		uint32 TotalSplatCount = 0;
		bool bAllNanite = true;  // True if every visible proxy supports compaction

		FVector CameraLocation = SceneView->ViewLocation;

		for (FGaussianSplatSceneProxy* Proxy : Proxies)
		{
			if (!Proxy) continue;
			if (&Proxy->GetScene() != SceneView->Family->Scene) continue;
			if (!Proxy->IsShown(SceneView)) continue;

			const FBoxSphereBounds& Bounds = Proxy->GetBounds();
			if (!bNativeGeerView && !SceneView->ViewFrustum.IntersectBox(Bounds.Origin, Bounds.BoxExtent)) continue;

			FGaussianSplatGPUResources* GPUResources = Proxy->GetGPUResources();
			if (!GPUResources || !GPUResources->IsValid()) continue;

			FProxyRenderInfo Info;
			Info.Proxy = Proxy;
			Info.LocalToWorld = Proxy->GetLocalToWorld();
			Info.GlobalBaseOffset = 0;  // Will be computed after sorting
			Info.bUseLODRendering = !bNativeGeerView && GPUResources->bEnableNanite && GPUResources->bHasLODSplats;
			Info.DistanceToCamera = FVector::Dist(Bounds.Origin, CameraLocation);
			VisibleProxies.Add(Info);

			// All proxies must support Nanite compaction for the fast global path
			if (bNativeGeerView || !GPUResources->bEnableNanite || !GPUResources->bHasClusterData || !GPUResources->bSupportsCompaction)
			{
				bAllNanite = false;
			}
		}

		// Sort by distance: closer proxies first (get budget priority when MaxRenderBudget is active)
		VisibleProxies.Sort([](const FProxyRenderInfo& A, const FProxyRenderInfo& B)
		{
			return A.DistanceToCamera < B.DistanceToCamera;
		});

		// Compute GlobalBaseOffset and TotalSplatCount after sorting
		for (FProxyRenderInfo& Info : VisibleProxies)
		{
			Info.GlobalBaseOffset = TotalSplatCount;
			TotalSplatCount += (uint32)Info.Proxy->GetSplatCount();
		}

		// Safety: global accumulator only supports up to MAX_PROXY_COUNT proxies
		if ((uint32)VisibleProxies.Num() > FGaussianGlobalAccumulator::MAX_PROXY_COUNT)
		{
			bAllNanite = false;
		}

		// Pass 1b (depth-proximity weighting) only supports the Nanite-compacted indirect-draw
		// path: it needs to re-issue the exact same draw using the GPU-resident indirect args
		// buffer, with no CPU-known splat count to fall back to for the non-compaction path.
		bDepthProximityPass = bDepthProximityPass && bAllNanite;

		if (TotalSplatCount == 0)
		{
			return;
		}

		// Gate B1 is intentionally brute-force and non-scalable. Refuse anything beyond the tiny
		// analytic scene before allocating/drawing, and refuse any proxy that would take legacy EWA.
		constexpr uint32 MaxNativeGeerGateB1Splats = 4;
		bool bAllNativeProxiesUseGeer = true;
		if (bNativeGeerView)
		{
			for (const FProxyRenderInfo& Info : VisibleProxies)
			{
				const bool bProxyGeer = (CurrentGeerMode == 2) ||
					(CurrentGeerMode == 1 && Info.Proxy->IsGeerSplat());
				bAllNativeProxiesUseGeer &= bProxyGeer;
			}
			if (TotalSplatCount > MaxNativeGeerGateB1Splats || !bAllNativeProxiesUseGeer)
			{
				if (!GNanoGSLoggedNativeGeerRefusals.Contains(NativeGeerView.registration_serial))
				{
					GNanoGSLoggedNativeGeerRefusals.Add(NativeGeerView.registration_serial);
					UE_LOG(LogTemp, Error,
						TEXT("[NanoGS][NativeGEER][GateB1] REFUSED camera=%s splats=%u cap=%u all_geer=%s; output remains black"),
						*NativeGeerView.camera_name, TotalSplatCount, MaxNativeGeerGateB1Splats,
						bAllNativeProxiesUseGeer ? TEXT("true") : TEXT("false"));
				}
				return;
			}
			if (!GNanoGSLoggedNativeGeerActive.Contains(NativeGeerView.registration_serial))
			{
				GNanoGSLoggedNativeGeerActive.Add(NativeGeerView.registration_serial);
				UE_LOG(LogTemp, Log,
					TEXT("[NanoGS][NativeGEER][GateB1] ACTIVE camera=%s splats=%u raymap=%dx%d full_output=true depth=false lighting=false velocity=zero"),
					*NativeGeerView.camera_name, TotalSplatCount,
					NativeGeerRaymapSize.X, NativeGeerRaymapSize.Y);
			}
		}

		// Check camera-static skip: if nothing has changed, skip Phase 1+2 and reuse cached sort
		// Use ProjectionNoAAMatrix to ignore TSR/TAA per-frame jitter that changes every frame
		FMatrix CurrentVP = SceneView->ViewMatrices.GetViewMatrix() * SceneView->ViewMatrices.GetProjectionNoAAMatrix();
		int32 CurrentDebugMode = DebugMode;
		int32 CurrentDebugForceLODLevel = CVarDebugForceLODLevel.GetValueOnRenderThread();
		// GEER eval mode for this frame was resolved before depth binding. It is captured below so
		// the camera-static skip can notice a change (cached W2O rows / AA opacity go stale).
		// GEER footprint mode, captured for the same reason: toggling it changes the quad axes and
		// the PBF rect offset the vertex shader adds, neither of which the skip would recompute.
		const bool bCurrentGeerPBF = CVarGeerPBF.GetValueOnRenderThread() != 0;
		const bool bCurrentGeerSort = CVarGeerSort.GetValueOnRenderThread() != 0;
		const bool bCurrentGeerAAOpacity = CVarGeerAAOpacity.GetValueOnRenderThread() != 0;

		bool bCanSkip = GlobalAccumulator->bHasCachedSortData &&
			GlobalAccumulator->CachedTotalSplatCount == TotalSplatCount &&
			GlobalAccumulator->CachedViewProjectionMatrix.Equals(CurrentVP, 0.0f) &&
			GlobalAccumulator->CachedNativeGeer == bNativeGeerView;

		if (bCanSkip)
		{
			for (const FProxyRenderInfo& Info : VisibleProxies)
			{
				FGaussianSplatGPUResources* GPUResources = Info.Proxy->GetGPUResources();
				float ProxyErrorThreshold = FMath::Max(0.1f, Info.Proxy->GetLODErrorThreshold());
				if (!GPUResources->bHasCachedSortData ||
					!GPUResources->CachedViewProjectionMatrix.Equals(CurrentVP, 0.0f) ||
					!GPUResources->CachedLocalToWorld.Equals(Info.LocalToWorld, 0.0f) ||
					GPUResources->CachedOpacityScale != Info.Proxy->GetOpacityScale() ||
					GPUResources->CachedSplatScale != Info.Proxy->GetSplatScale() ||
					GPUResources->CachedErrorThreshold != ProxyErrorThreshold ||
					GPUResources->CachedDebugMode != CurrentDebugMode ||
					GPUResources->CachedDebugForceLODLevel != CurrentDebugForceLODLevel ||
					GPUResources->CachedGeerEval != ((CurrentGeerMode == 2) ||
						(CurrentGeerMode == 1 && Info.Proxy->IsGeerSplat())) ||
					GPUResources->CachedGeerPBF != bCurrentGeerPBF ||
					GPUResources->CachedGeerSort != bCurrentGeerSort ||
					GPUResources->CachedGeerAAOpacity != bCurrentGeerAAOpacity ||
					GPUResources->CachedNativeGeer != bNativeGeerView)
				{
					bCanSkip = false;
					break;
				}
			}
		}

		// Grab index buffer from the first proxy (all proxies use identical quad geometry)
		FBufferRHIRef SharedIndexBuffer;
		if (VisibleProxies.Num() > 0)
		{
			FGaussianSplatGPUResources* FirstRes = VisibleProxies[0].Proxy->GetGPUResources();
			SharedIndexBuffer = FirstRes ? FirstRes->IndexBuffer : FBufferRHIRef();
		}

		FGaussianGlobalAccumulator* RawAccumulator = GlobalAccumulator.Get();

		// Read render budget for global accumulator buffer cap.
		// Disable budget when forcing LOD level — the debug command needs to show
		// all assets regardless of splat count (user expects to see quality vs performance).
		int32 BudgetVal = CVarMaxRenderBudget.GetValueOnRenderThread();
		uint32 MaxRenderBudget = (BudgetVal > 0) ? (uint32)BudgetVal : 0;
		if (CurrentDebugForceLODLevel >= 0)
		{
			MaxRenderBudget = 0;  // Unlimited — debug mode overrides budget
		}
		if (bNativeGeerView)
		{
			MaxRenderBudget = 0;  // Gate B1's explicit four-splat cap is the only native budget.
		}

		GraphBuilder.AddPass(
			RDG_EVENT_NAME("GaussianSplat_RenderToIntermediate"),
			Pass1Parameters,
			ERDGPassFlags::Raster,
			[SceneView, VisibleProxies, TotalSplatCount, bCanSkip, bAllNanite, RawAccumulator,
			 SharedIndexBuffer, CurrentVP, CurrentDebugMode,
			 CurrentDebugForceLODLevel, CurrentGeerMode, bCurrentGeerPBF, bCurrentGeerSort,
			 bCurrentGeerAAOpacity, DebugMode, MaxRenderBudget, bOutputDepth, bOutputNormal,
			 bUseNormalConfidenceFade, bWriteSplatDepth, bNativeGeerView, NativeGeerRaymap,
			 NativeGeerRaymapSize](FRHICommandListImmediate& RHICmdList)
			{
				if (!SceneView) return;
				SCOPED_DRAW_EVENT(RHICmdList, GaussianSplatRendering_Global);

				// SAFETY CHECK: Re-validate all proxies before rendering.
				// Proxies may have been destroyed between when we built VisibleProxies
				// and when this lambda executes (RDG deferred execution).
				// We need to rebuild the list with only valid proxies.
				TArray<FProxyRenderInfo> ValidProxies;
				ValidProxies.Reserve(VisibleProxies.Num());
				uint32 NewTotalSplatCount = 0;

				for (const auto& Info : VisibleProxies)
				{
					// Check if proxy is still valid (not destroyed or pending destruction)
					if (Info.Proxy && Info.Proxy->IsValidForRendering())
					{
						FProxyRenderInfo ValidInfo = Info;
						ValidInfo.GlobalBaseOffset = NewTotalSplatCount;
						NewTotalSplatCount += (uint32)Info.Proxy->GetSplatCount();
						ValidProxies.Add(ValidInfo);
					}
				}

				// If no valid proxies remain, skip rendering entirely
				if (ValidProxies.Num() == 0 || NewTotalSplatCount == 0)
				{
					return;
				}
				if (bNativeGeerView && NewTotalSplatCount > 4u)
				{
					// Proxy mutation after graph construction must fail closed too. SceneColor was already
					// cleared by the native pre-pass, so returning cannot expose a pinhole substitute.
					return;
				}

				// Invalidate cache skip if the proxy list changed (some proxies were destroyed)
				// This ensures we don't use stale cached data when the scene has changed
				bool bCanSkipAdjusted = bCanSkip;
				if (ValidProxies.Num() != VisibleProxies.Num() || NewTotalSplatCount != TotalSplatCount)
				{
					bCanSkipAdjusted = false;
					// Also invalidate the global accumulator cache since proxy set changed
					RawAccumulator->bHasCachedSortData = false;
				}

				// Initialize color textures (deferred init) - only for valid proxies
				for (const auto& Info : ValidProxies)
				{
					Info.Proxy->TryInitializeColorTexture(RHICmdList);
				}

				// Ensure global buffers are large enough for all splats
				RawAccumulator->ResizeIfNeeded(RHICmdList, NewTotalSplatCount);

				// Re-check if all valid proxies support Nanite compaction
				bool bAllValidNanite = !bNativeGeerView;
				for (const auto& Info : ValidProxies)
				{
					FGaussianSplatGPUResources* GPUResources = Info.Proxy->GetGPUResources();
					if (!GPUResources || !GPUResources->bEnableNanite || !GPUResources->bHasClusterData || !GPUResources->bSupportsCompaction)
					{
						bAllValidNanite = false;
						break;
					}
				}

				// Cap to MAX_PROXY_COUNT
				if ((uint32)ValidProxies.Num() > FGaussianGlobalAccumulator::MAX_PROXY_COUNT)
				{
					bAllValidNanite = false;
				}

				if (bAllValidNanite)
				{
					//==================================================
					// GLOBAL + COMPACTION PATH
					// All proxies are Nanite-enabled: GPU compaction
					// reduces working set from TotalSplatCount → TotalVisible
					// (~140x reduction at LOD5 for a 719K-splat tile).
					//==================================================

					// Ensure fixed-size prefix-sum buffers exist (allocated once)
					RawAccumulator->EnsureCompactionBuffersAllocated(RHICmdList);

					if (!bCanSkipAdjusted)
					{
						// --------------------------------------------------
						// Phase 0: Per-proxy culling + compaction + indirect args
						// Early-out: skip proxies once cumulative splat count
						// exceeds MaxRenderBudget (CPU-side estimate using total
						// splat count as conservative upper bound for visible count).
						// Proxies are sorted by distance, so closer ones get priority.
						// --------------------------------------------------
						int32 NumProcessedProxies = 0;
						uint32 CumulativeSplatCount = 0;

						for (const auto& Info : ValidProxies)
						{
							// Budget early-out: if cumulative total already exceeds budget,
							// skip culling/compaction for remaining (farther) proxies
							if (MaxRenderBudget > 0 && CumulativeSplatCount >= MaxRenderBudget)
							{
								break;
							}

							FGaussianSplatGPUResources* GPUResources = Info.Proxy->GetGPUResources();
							if (!GPUResources) continue;  // Extra safety check
							int32 SplatCount = Info.Proxy->GetSplatCount();
							int32 OriginalSplatCount = SplatCount - GPUResources->LODSplatCount;

							// Cluster culling → fills ClusterVisibilityBitmap
							FGaussianSplatRenderer::DispatchClusterCulling(
								RHICmdList, *SceneView, GPUResources,
								Info.LocalToWorld, Info.Proxy->GetLODErrorThreshold(), Info.bUseLODRendering);

							// Compact → fills CompactedSplatIndices + VisibleSplatCountBuffer
							FGaussianSplatRenderer::DispatchCompactSplats(
								RHICmdList, GPUResources,
								SplatCount, OriginalSplatCount, Info.bUseLODRendering);

							// PrepareIndirectArgs → fills IndirectDispatchArgsBuffer for CalcViewData
							FGaussianSplatRenderer::DispatchPrepareIndirectArgs(RHICmdList, GPUResources);

							CumulativeSplatCount += (uint32)SplatCount;
							NumProcessedProxies++;
						}

						// --------------------------------------------------
						// Phase 1: Gather visible counts + GPU prefix sum
						// Only gather from proxies that were actually processed
						// --------------------------------------------------
						for (int32 i = 0; i < NumProcessedProxies; i++)
						{
							FGaussianSplatGPUResources* GPUResources = ValidProxies[i].Proxy->GetGPUResources();
							if (!GPUResources) continue;  // Extra safety check
							FGaussianSplatRenderer::DispatchGatherVisibleCount(
								RHICmdList, GPUResources, RawAccumulator, i);
						}

						// Single 1-thread dispatch: computes prefix sums + writes all indirect args
						FGaussianSplatRenderer::DispatchPrefixSumVisibleCounts(
							RHICmdList, RawAccumulator, NumProcessedProxies, MaxRenderBudget);

						// --------------------------------------------------
						// Phase 2: Per-proxy CalcViewData → global buffer
						// (indirect dispatch, only visible splats per proxy)
						// Only process proxies that went through culling/compaction
						// --------------------------------------------------
						// The sort key must be uniform across the whole global sort, so it is a
						// per-FRAME decision: Euclidean iff any visible proxy takes the GEER path.
						bool bAnyGeerVisible = false;
						for (int32 i = 0; i < NumProcessedProxies; i++)
						{
							const auto& Info = ValidProxies[i];
							FGaussianSplatGPUResources* GPUResources = Info.Proxy->GetGPUResources();
							if (!GPUResources) continue;  // Extra safety check
							int32 SplatCount = Info.Proxy->GetSplatCount();
							int32 OriginalSplatCount = SplatCount - GPUResources->LODSplatCount;

							// Evaluation is per PROXY: 2 forces GEER everywhere, 1 respects the
							// per-asset declaration, 0 is the kill switch.
							const bool bProxyGeer = (CurrentGeerMode == 2) ||
								(CurrentGeerMode == 1 && Info.Proxy->IsGeerSplat());
							bAnyGeerVisible |= bProxyGeer;

							FGaussianSplatRenderer::DispatchCalcViewDataCompactedGlobal(
								RHICmdList, *SceneView, GPUResources,
								Info.LocalToWorld,
								SplatCount,
								OriginalSplatCount,
								Info.Proxy->GetSHOrder(),
								Info.Proxy->GetOpacityScale(),
								Info.Proxy->GetSplatScale(),
								i,
								RawAccumulator,
								MaxRenderBudget,
								bProxyGeer);
						}

						// --------------------------------------------------
						// Phase 3: Single global CalcDistances + RadixSort
						// (all indirect — count driven by GPU prefix sum)
						// --------------------------------------------------
						FGaussianSplatRenderer::DispatchCalcDistancesGlobalIndirect(RHICmdList, RawAccumulator, bAnyGeerVisible);
						FGaussianSplatRenderer::DispatchRadixSortGlobalIndirect(RHICmdList, RawAccumulator);

						// Update caches — only for processed proxies
						RawAccumulator->bHasCachedSortData = true;
						RawAccumulator->CachedTotalSplatCount = NewTotalSplatCount;
						RawAccumulator->CachedViewProjectionMatrix = CurrentVP;
						RawAccumulator->CachedNativeGeer = false;

						for (int32 i = 0; i < ValidProxies.Num(); i++)
						{
							FGaussianSplatGPUResources* GPUResources = ValidProxies[i].Proxy->GetGPUResources();
							if (!GPUResources) continue;

							if (i < NumProcessedProxies)
							{
								const auto& Info = ValidProxies[i];
								GPUResources->CachedViewProjectionMatrix = CurrentVP;
								GPUResources->CachedLocalToWorld = Info.LocalToWorld;
								GPUResources->CachedOpacityScale = Info.Proxy->GetOpacityScale();
								GPUResources->CachedSplatScale = Info.Proxy->GetSplatScale();
								GPUResources->CachedErrorThreshold = FMath::Max(0.1f, Info.Proxy->GetLODErrorThreshold());
								GPUResources->CachedDebugMode = CurrentDebugMode;
								GPUResources->CachedDebugForceLODLevel = CurrentDebugForceLODLevel;
								GPUResources->CachedGeerEval = (CurrentGeerMode == 2) ||
									(CurrentGeerMode == 1 && Info.Proxy->IsGeerSplat());
								GPUResources->CachedGeerPBF = bCurrentGeerPBF;
								GPUResources->CachedGeerSort = bCurrentGeerSort;
								GPUResources->CachedGeerAAOpacity = bCurrentGeerAAOpacity;
								GPUResources->CachedNativeGeer = false;
								GPUResources->bHasCachedSortData = true;
							}
							else
							{
								// Invalidate cache for budget-skipped proxies so they
								// don't block the camera-static skip check
								GPUResources->bHasCachedSortData = false;
							}
						}
					}

					// Single draw call — instance count from GlobalDrawIndirectArgsBuffer
					FGaussianSplatRenderer::DrawSplatsGlobalIndirect(
						RHICmdList, *SceneView, RawAccumulator, SharedIndexBuffer, DebugMode, bWriteSplatDepth,
						bOutputDepth, bOutputNormal,
						bUseNormalConfidenceFade);
				}
				else
				{
					//==================================================
					// NON-COMPACTION GLOBAL PATH (fallback)
					// Not all proxies are Nanite-enabled.
					// Sorts all NewTotalSplatCount splats (no compaction benefit).
					// Still provides correct cross-tile alpha blending.
					//==================================================

					// Cap splat count to render budget (CPU-side enforcement)
					uint32 CappedTotalSplatCount = NewTotalSplatCount;
					if (MaxRenderBudget > 0 && CappedTotalSplatCount > MaxRenderBudget)
					{
						CappedTotalSplatCount = MaxRenderBudget;
					}

					if (!bCanSkipAdjusted)
					{
						// --------------------------------------------------
						// Phase 1: Per-proxy ClusterCulling + CalcViewData
						// --------------------------------------------------
						// Per-FRAME sort decision, as in the compaction path above.
						bool bAnyGeerVisible = false;
						for (const auto& Info : ValidProxies)
						{
							// Skip proxies that would write beyond the render budget
							if (MaxRenderBudget > 0 && Info.GlobalBaseOffset >= MaxRenderBudget)
							{
								break;  // All subsequent proxies also exceed the budget
							}

							FGaussianSplatGPUResources* GPUResources = Info.Proxy->GetGPUResources();
							if (!GPUResources) continue;  // Extra safety check

							// Cluster culling for Nanite-enabled proxies
							if (!bNativeGeerView && GPUResources->bEnableNanite && GPUResources->bHasClusterData)
							{
								FGaussianSplatRenderer::DispatchClusterCulling(
									RHICmdList, *SceneView, GPUResources,
									Info.LocalToWorld, Info.Proxy->GetLODErrorThreshold(), Info.bUseLODRendering);
							}

							// Evaluation is per PROXY (see the compaction path for the mode table).
							const bool bProxyGeer = (CurrentGeerMode == 2) ||
								(CurrentGeerMode == 1 && Info.Proxy->IsGeerSplat());
							bAnyGeerVisible |= bProxyGeer;

							// CalcViewData → writes to GlobalViewDataBuffer at GlobalBaseOffset
							FGaussianSplatRenderer::DispatchCalcViewDataGlobal(
								RHICmdList, *SceneView, GPUResources,
								Info.LocalToWorld,
								Info.Proxy->GetSplatCount(),
								Info.Proxy->GetSHOrder(),
								Info.Proxy->GetOpacityScale(),
								Info.Proxy->GetSplatScale(),
								Info.bUseLODRendering,
								Info.GlobalBaseOffset,
								RawAccumulator,
								bProxyGeer,
								bNativeGeerView);
						}

						// --------------------------------------------------
						// Phase 2: Single global CalcDistances + RadixSort
						// --------------------------------------------------
						FGaussianSplatRenderer::DispatchCalcDistancesGlobal(
							RHICmdList, RawAccumulator, (int32)CappedTotalSplatCount,
							bAnyGeerVisible, bNativeGeerView);
						FGaussianSplatRenderer::DispatchRadixSortGlobal(RHICmdList, RawAccumulator, (int32)CappedTotalSplatCount);

						// Update caches
						RawAccumulator->bHasCachedSortData = true;
						RawAccumulator->CachedTotalSplatCount = NewTotalSplatCount;
						RawAccumulator->CachedViewProjectionMatrix = CurrentVP;
						RawAccumulator->CachedNativeGeer = bNativeGeerView;

						for (const auto& Info : ValidProxies)
						{
							FGaussianSplatGPUResources* GPUResources = Info.Proxy->GetGPUResources();
							if (!GPUResources) continue;  // Extra safety check
							GPUResources->CachedViewProjectionMatrix = CurrentVP;
							GPUResources->CachedLocalToWorld = Info.LocalToWorld;
							GPUResources->CachedOpacityScale = Info.Proxy->GetOpacityScale();
							GPUResources->CachedSplatScale = Info.Proxy->GetSplatScale();

							GPUResources->CachedErrorThreshold = FMath::Max(0.1f, Info.Proxy->GetLODErrorThreshold());
							GPUResources->CachedDebugMode = CurrentDebugMode;
							GPUResources->CachedDebugForceLODLevel = CurrentDebugForceLODLevel;
							GPUResources->CachedGeerEval = (CurrentGeerMode == 2) ||
								(CurrentGeerMode == 1 && Info.Proxy->IsGeerSplat());
							GPUResources->CachedGeerPBF = bCurrentGeerPBF;
							GPUResources->CachedGeerSort = bCurrentGeerSort;
							GPUResources->CachedGeerAAOpacity = bCurrentGeerAAOpacity;
							GPUResources->CachedNativeGeer = bNativeGeerView;
							GPUResources->bHasCachedSortData = true;
						}
					}

					// Single draw call for ALL proxies (capped to render budget)
					FGaussianSplatRenderer::DrawSplatsGlobal(
						RHICmdList, *SceneView, RawAccumulator,
						SharedIndexBuffer, (int32)CappedTotalSplatCount, DebugMode, bWriteSplatDepth,
						bOutputDepth, bOutputNormal,
						bUseNormalConfidenceFade,
						nullptr, 0.05f,
						bNativeGeerView, NativeGeerRaymap, NativeGeerRaymapSize);
				}
			}
		);

		// Pass 1b (gs.DepthProximityWeighting): re-rasterize the same already-sorted splats a
		// second time, now that Pass 1's DepthAccumTexture is fully resolved, so each splat's
		// normal contribution can be weighted by how close its own depth is to the known surface
		// depth — suppresses background/secondary splats bleeding through a semi-transparent
		// foreground splat from polluting that pixel's normal. Reuses the same sorted/indirect
		// draw data Pass 1 just built (no re-sort, no CalcViewData), so the extra cost is just the
		// second raster+blend pass. Only supports the Nanite-compacted indirect path (bAllNanite).
		if (bDepthProximityPass)
		{
			// The per-splat PS always writes Color+Velocity (SV_Target0/1) regardless of
			// permutation; this pass only cares about NormalAccum, so bind throwaway scratch
			// targets for the other two and let RDG discard them (nothing reads them afterward).
			FRDGTextureDesc ScratchColorDesc = FRDGTextureDesc::Create2D(
				ColorTexture->Desc.Extent,
				PF_FloatRGBA,
				FClearValueBinding(FLinearColor::Transparent),
				TexCreate_RenderTargetable | TexCreate_ShaderResource);
			FRDGTexture* ScratchColorTexture = GraphBuilder.CreateTexture(ScratchColorDesc, TEXT("GaussianSplatScratchColorRT"));

			FRDGTexture* ScratchVelocityTexture = GraphBuilder.CreateTexture(VelocityTexture->Desc, TEXT("GaussianSplatScratchVelocityRT"));

			FGaussianNormalProximityPassParameters* Pass1bParameters = GraphBuilder.AllocParameters<FGaussianNormalProximityPassParameters>();
			Pass1bParameters->DepthAccumTexture = DepthAccumTexture;  // read barrier: RTV(Pass 1) -> SRV(Pass 1b)
			Pass1bParameters->RenderTargets[0] = FRenderTargetBinding(ScratchColorTexture, ERenderTargetLoadAction::EClear);
			Pass1bParameters->RenderTargets[1] = FRenderTargetBinding(ScratchVelocityTexture, ERenderTargetLoadAction::EClear);
			Pass1bParameters->RenderTargets[2] = FRenderTargetBinding(NormalAccumTexture, ERenderTargetLoadAction::EClear);
			if (DepthTexture)
			{
				Pass1bParameters->RenderTargets.DepthStencil = FDepthStencilBinding(
					DepthTexture,
					ERenderTargetLoadAction::ELoad,
					ERenderTargetLoadAction::ELoad,
					bWriteSplatDepth
						? FExclusiveDepthStencil::DepthWrite_StencilWrite
						: FExclusiveDepthStencil::DepthRead_StencilWrite
				);
			}

			FGaussianGlobalAccumulator* RawAccumulatorForProximity = GlobalAccumulator.Get();
			const float DepthProximitySigma = SceneLighting.DepthProximitySigma;

			GraphBuilder.AddPass(
				RDG_EVENT_NAME("GaussianSplat_NormalDepthProximity"),
				Pass1bParameters,
				ERDGPassFlags::Raster,
				[SceneView, RawAccumulatorForProximity, SharedIndexBuffer, DebugMode,
				 bUseNormalConfidenceFade, DepthAccumTexture, DepthProximitySigma,
				 bWriteSplatDepth](FRHICommandListImmediate& RHICmdList)
				{
					if (!SceneView || !RawAccumulatorForProximity) return;
					FRHITexture* DepthAccumRHI = DepthAccumTexture ? DepthAccumTexture->GetRHI() : nullptr;
					if (!DepthAccumRHI) return;

					FGaussianSplatRenderer::DrawSplatsGlobalIndirect(
						RHICmdList, *SceneView, RawAccumulatorForProximity, SharedIndexBuffer, DebugMode,
						bWriteSplatDepth, /*bOutputDepth=*/false, /*bOutputNormal=*/true,
						bUseNormalConfidenceFade,
						DepthAccumRHI, DepthProximitySigma);
				}
			);
		}

		// Pass 2: Composite intermediate sRGB RT onto SceneColor (sRGB → linear conversion +
		// optional screen-space lighting from the alpha-weighted depth accumulation).
		// SceneLighting was gathered above (before Pass 1). FGaussianCompositePassParameters
		// declares IntermediateTexture and DepthAccumTexture as RDG-tracked inputs so RDG inserts
		// the required RTV→SRV barriers between Pass 1 and this pass.
		ERenderTargetLoadAction CompositeColorLoadAction = (DebugMode > 0) ? ERenderTargetLoadAction::EClear : ERenderTargetLoadAction::ELoad;
		FGaussianCompositePassParameters* Pass2Parameters = GraphBuilder.AllocParameters<FGaussianCompositePassParameters>();
		Pass2Parameters->IntermediateTexture = IntermediateTexture;
		// Bind the depth-accum texture for the read barrier; fall back to IntermediateTexture as a
		// harmless stand-in when lighting is off (CompositeToSceneColor forces LightingBlend=0 then).
		Pass2Parameters->DepthAccumTexture = bOutputDepth ? DepthAccumTexture : IntermediateTexture;
		// Same fallback pattern for the GeometryMode-1 proxy-mesh depth source.
		Pass2Parameters->CustomDepthTexture = CustomDepthTexture ? CustomDepthTexture : IntermediateTexture;
		// Same fallback pattern for the GeometryMode-2 per-splat normal accumulation.
		Pass2Parameters->NormalAccumTexture = bOutputNormal ? NormalAccumTexture : IntermediateTexture;
		Pass2Parameters->RenderTargets[0] = FRenderTargetBinding(ColorTexture, CompositeColorLoadAction);

		// FGaussianSceneLighting (16 lights + many scalar fields) is too large to capture by
		// value in the lambda below — RDG caps lambda capture size at 1024 bytes and this struct
		// has grown past that combined with the other captures. Heap-allocate a copy and capture
		// a TSharedRef (just a pointer) instead; CompositeToSceneColor still gets the full struct.
		TSharedRef<FGaussianSceneLighting> SceneLightingRef = MakeShared<FGaussianSceneLighting>(SceneLighting);

		GraphBuilder.AddPass(
			RDG_EVENT_NAME("GaussianSplat_CompositeToSceneColor"),
			Pass2Parameters,
			ERDGPassFlags::Raster,
			[SceneView, IntermediateTexture, DepthAccumTexture, CustomDepthTexture, NormalAccumTexture,
			 bOutputDepth, bOutputNormal, SceneLightingRef](FRHICommandListImmediate& RHICmdList)
			{
				if (!SceneView) return;

				FRHITexture* IntermediateRHI = IntermediateTexture->GetRHI();
				if (!IntermediateRHI) return;

				// Depth-accum RHI texture (null when lighting is off → CompositeToSceneColor skips lighting)
				FRHITexture* DepthAccumRHI = (bOutputDepth && DepthAccumTexture) ? DepthAccumTexture->GetRHI() : nullptr;
				// Proxy-mesh CustomDepth RHI texture (null when GeometryMode != 1 or unavailable this frame)
				FRHITexture* CustomDepthRHI = CustomDepthTexture ? CustomDepthTexture->GetRHI() : nullptr;
				// Per-splat normal accum RHI texture (null when GeometryMode != 2)
				FRHITexture* NormalAccumRHI = (bOutputNormal && NormalAccumTexture) ? NormalAccumTexture->GetRHI() : nullptr;

				FGaussianSplatRenderer::CompositeToSceneColor(
					RHICmdList, *SceneView, IntermediateRHI, DepthAccumRHI, CustomDepthRHI, NormalAccumRHI, *SceneLightingRef);
			}
		);
}

void FNanoGSModule::ShutdownModule()
{
	// Remove deferred init delegate
	if (PostEngineInitDelegateHandle.IsValid())
	{
		FCoreDelegates::OnPostEngineInit.Remove(PostEngineInitDelegateHandle);
		PostEngineInitDelegateHandle.Reset();
	}

	// Unregister post-opaque render delegate
	if (PostOpaqueRenderDelegateHandle.IsValid())
	{
		GetRendererModuleRef().RemovePostOpaqueRenderDelegate(PostOpaqueRenderDelegateHandle);
		PostOpaqueRenderDelegateHandle.Reset();
	}

	// Release global accumulator GPU buffers from the render thread
	if (GlobalAccumulator.IsValid())
	{
		FGaussianGlobalAccumulator* RawAccumulator = GlobalAccumulator.Release();
		ENQUEUE_RENDER_COMMAND(ReleaseGlobalAccumulator)(
			[RawAccumulator](FRHICommandListImmediate& RHICmdList)
			{
				RawAccumulator->Release();
				delete RawAccumulator;
			});
	}

	// Clear the view extension
	ViewExtension.Reset();
}

FNanoGSModule& FNanoGSModule::Get()
{
	return FModuleManager::LoadModuleChecked<FNanoGSModule>("NanoGS");
}

bool FNanoGSModule::IsAvailable()
{
	return FModuleManager::Get().IsModuleLoaded("NanoGS");
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FNanoGSModule, NanoGS)
