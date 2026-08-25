// Emitting a mirrored Unreal level into a MuJoCo spec — ONE implementation, two callers.
//
// ⚠ THIS EXISTS SO THERE IS NOT A SECOND COPY. The per-robot MuJoCoUrdfBackend and the shared
// MuJoCoPhysicsScene need identical level geometry: a comparison between a private-world run and a
// coordinated run is only meaningful if both robots stand on the same cooked world. Copying these
// 350 lines into the scene would make that an assumption instead of a fact, and the two would
// drift exactly the way the audit counters below already drifted from the code that fills them.
//
// ⚠ WHAT THIS DOES NOT DO. It does not convert a vast flat slab into an mjGEOM_PLANE. A long
// comment in the original code described that conversion as though it existed; it never did, and
// the counters that reported it were never assigned, so the log claimed "0 vast flat shapes
// converted to infinite planes" whatever the level contained. What actually keeps a 40 km ground
// mesh from wrecking the broadphase is the region CLIPPING in the mesh branch, which is real and
// is counted here.
#pragma once

#include "urdf/UrdfRobotBackend.hpp"
#include "urdf/UrdfStaticWorld.hpp"

#include <cstddef>
#include <functional>
#include <string>

struct mjSpec_;
struct mjsBody_;

namespace urdf {

/// World-level knobs. Deliberately NOT `BackendOptions`: a shared scene's level is a property of
/// the world, not of whichever robot happened to be built first.
struct StaticWorldEmitOptions {
    /// Preferred ground. Native, exact for slopes and steps, no decomposition, free in broadphase.
    BackendOptions::HeightField ground_height_field;

    /// Fallback floor, used only when no height field was supplied.
    bool add_ground_plane = false;
    double ground_plane_z = 0.0;

    /// Centre of the region mirrored geometry is clipped to. A level's ground is often one
    /// enormous mesh, and taking only the nearby triangles avoids both a useless approximation and
    /// a broadphase disaster.
    Vec3 clip_center;
    double static_world_radius = 30.0;
    int static_world_max_triangles = 20000;

    BackendOptions::StaticMeshMode static_mesh_mode = BackendOptions::StaticMeshMode::Split;

    /// Called as emission proceeds so a host can draw a progress bar; an enclosed map emits tens
    /// of thousands of prisms and looks like a hang without it.
    std::function<void(const std::string& stage, int done, int total)> build_progress;
};

/// What the level actually became. Every field is written by the code that does the work, so a
/// number here cannot quietly disagree with the geometry.
struct StaticWorldEmitStats {
    size_t geoms_emitted = 0;
    size_t shapes_dropped = 0;          ///< non-finite or degenerate; never silently omitted
    size_t triangles_emitted = 0;
    size_t triangles_skipped = 0;       ///< dropped at the max-triangle cap
    size_t triangles_clipped_away = 0;  ///< fell entirely outside the region radius
    size_t convex_objects = 0;
    size_t enclosures = 0;              ///< inward-facing; always split, never taken whole
    bool used_height_field = false;

    /// Largest single mirrored object, and which body it came from. A geom whose bounding radius
    /// dwarfs the scene is a broadphase hazard, and this is the number that shows it.
    double worst_span = 0;
    std::string worst_span_body;
    /// Largest absolute vertex coordinate seen, in metres from the world origin.
    double worst_vertex = 0;
};

/// Emit `static_world` (may be null) plus the ground into `spec`'s `world` body.
///
/// Game-thread-agnostic and Unreal-free: it consumes the already-mirrored solver-neutral
/// description, so it can be exercised headlessly.
void emitStaticWorld(mjSpec_* spec, mjsBody_* world, const StaticWorld* static_world,
                     const StaticWorldEmitOptions& options, StaticWorldEmitStats& stats);

} // namespace urdf
