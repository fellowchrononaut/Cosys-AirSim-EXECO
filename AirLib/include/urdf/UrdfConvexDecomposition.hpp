// Approximate convex decomposition — one concave mesh in, several convex parts out.
//
// ⚠ SHARED BY BOTH BACKENDS, WHICH IS WHY IT LIVES HERE and not inside either one. The two engines
// need it in complementary places and for the same underlying reason — neither can collide a
// moving concave mesh:
//
//   Box3D   mesh shapes are STATIC-ONLY (docs/loose_ends.md #7), so a robot link's <collision>
//           <mesh> and every kinematic mirrored body are simulated as ONE convex hull today.
//           Box3DRobot.cpp:330 has said so, and named this as the fix, for a while.
//   MuJoCo  replaces ANY mesh with its convex hull for collision, so it needs decomposition for
//           static level geometry as well — a mirrored level would otherwise be one convex blob
//           per object, with doorways and interiors filled solid.
//
// Putting it in the MuJoCo backend first and generalising later would have meant writing it twice.
//
// ⚠ THE NO-COACD PATH RETURNS ONE PART, deliberately. Without the dependency
// (WITH_COACD_BINDING=0) this returns the input as a single part, which is precisely the
// single-convex-hull behaviour every call site has today. Callers therefore need no #if and no
// fallback branch of their own: they always hull whatever parts they are given, and a build
// without CoACD behaves exactly as it did before CoACD existed.
#pragma once

#include "urdf/UrdfModel.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace urdf {

/// One convex piece. Points only — every consumer hulls them with its own engine's hull builder
/// (b3CreateHull, mjs_addMesh), so handing back triangles would only be thrown away.
struct ConvexPart {
    std::vector<Vec3> points;
};

struct DecompositionOptions {
    /// ⚠ 0.10, NOT CoACD's upstream default of 0.05. Upstream is tuned for visual fidelity;
    /// this is collision geometry, where every extra part is a shape the solver tests every step
    /// forever. Measured on the Go2 hip (37,040 triangles):
    ///
    ///     threshold 0.05  ->  99 parts,  97.6 s
    ///     threshold 0.10  ->  14 parts,  30.2 s
    ///     threshold 0.20  ->   4 parts,  21.0 s
    ///
    /// 99 hulls on one leg segment is not a better simulation than 14, it is a slower one. And 14
    /// is still fourteen times better than the single fat hull that is the alternative.
    double threshold = 0.10;

    /// Hard ceiling, as a safety net rather than a tuning knob: an unlucky mesh at a low threshold
    /// can produce hundreds of parts, and no link needs hundreds. -1 means unlimited.
    int max_hulls = 32;

    /// Vertices per convex part, asked of the decomposer directly.
    ///
    /// ⚠ 64, NOT CoACD's default of 256, and this one is a CRASH FIX rather than a tuning choice.
    /// Box3D's hull builder has a hard ceiling of 255 HALF-EDGES, and a 256-vertex cloud sails past
    /// it: measured on ExoMy, parts arrived at 266-342 half-edges. Box3D usually refuses such a
    /// hull by returning null, but on some of them it segfaults inside b3NewellPlane instead — it
    /// took the editor down on 2026-08-19.
    ///
    /// Capping here rather than thinning the cloud afterwards is the right place: the decomposer
    /// chooses which vertices to keep with the shape in view, whereas a post-hoc decimation would
    /// be throwing away points it does not understand. Box3D's own budget walk still runs after
    /// this as a second line of defence.
    int max_part_vertices = 64;

    /// Off => one part, i.e. today's behaviour. Exposed so a robot can opt out per vehicle without
    /// rebuilding, and so a bad decomposition can be ruled out when diagnosing contact problems.
    bool enabled = true;

    /// Called once per mesh, BEFORE the work starts, with a short description.
    ///
    /// ⚠ The whole point is that this happens while the caller is blocked. Decomposition runs
    /// synchronously on whichever thread builds the robot — the game thread, in Unreal — so a cold
    /// cook freezes the editor with no window, no cursor and no clue. Measured on ExoMy: 312 s.
    /// A caller that can draw progress supplies this; AirLib itself cannot, and must not, know how.
    ///
    /// Not part of the cache key, obviously — it does not change the geometry.
    std::function<void(const std::string& what)> progress;

    /// ⚠ WITHOUT A CACHE THIS FEATURE IS NOT USABLE, and that is a measurement, not a caution.
    /// The Go2 hip takes 20-30 s and a mirrored Blocks level is 172 meshes; a cold cook of the
    /// level is on the order of an hour. Empty disables the disk cache, which is right for tests
    /// and wrong for everything else.
    std::string cache_dir;
};

struct DecompositionResult {
    std::vector<ConvexPart> parts;

    /// True only if CoACD actually ran or its result was loaded. False means `parts` holds the
    /// single-hull fallback, and `note` says why — no CoACD in the build, disabled by settings,
    /// too few vertices, or a failure inside CoACD.
    bool decomposed = false;
    bool from_cache = false;

    /// Wall-clock seconds spent decomposing. Zero on a cache hit — which is the number that makes
    /// a cache hit visible in a log rather than merely assumed.
    double seconds = 0;

    /// Always populated, and worth logging on the fallback paths. A link silently collapsing back
    /// to one hull is exactly the kind of quiet fidelity loss the R2 audit exists to prevent.
    std::string note;
};

/// Does this point cloud span three dimensions well enough to be hulled?
///
/// ⚠ SHARED BECAUSE BOTH ENGINES BREAK ON THE SAME INPUT, in different ways. Box3D's b3CreateHull
/// SEGFAULTS on a cloud with no volume (it took the editor down on 2026-08-19); MuJoCo's compiler
/// refuses one — "mesh has colocated vertices, cannot compute convex hull" (user_mesh.cc:1760) —
/// which fails the entire model, not just that shape. A crash and a failed load are different
/// symptoms of one cause, so the check lives here rather than in either backend.
///
/// Convex decomposition is what makes this matter: one hull per mesh meant one big well-conditioned
/// cloud, while N parts per mesh means the decomposer's thin slivers reach the hull builders too.
///
/// The test is the one a hull builder needs — can four points be found spanning three dimensions?
/// Walked as extent, then area, then thickness, taking the most extreme point at each step, so a
/// genuinely thin-but-valid slab passes while a flat sheet does not.
bool hasHullableVolume(const std::vector<Vec3>& points, double eps = 1.0e-4);

/// Decompose an INDEXED mesh. Three indices per triangle.
DecompositionResult decomposeConvex(const std::vector<Vec3>& vertices,
                                    const std::vector<int>& indices,
                                    const DecompositionOptions& opts);

/// Decompose a TRIANGLE SOUP — three vertices per face, no index buffer, which is what
/// urdf::MeshData is (see UrdfMesh.hpp). Welds by position first: without that every triangle is
/// an island, CoACD sees no connectivity at all, and the result is meaningless.
DecompositionResult decomposeConvexSoup(const std::vector<Vec3>& soup,
                                        const DecompositionOptions& opts);

/// The cache key for a mesh and a set of options: content-addressed, so a changed mesh or a
/// changed threshold is a different entry and a stale result can never be served. Exposed for
/// tests and for logs.
uint64_t decompositionCacheKey(const std::vector<Vec3>& vertices, const std::vector<int>& indices,
                               const DecompositionOptions& opts);

} // namespace urdf
