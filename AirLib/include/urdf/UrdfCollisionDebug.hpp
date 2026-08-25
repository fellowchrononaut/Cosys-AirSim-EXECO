// What the solver ACTUALLY holds as collision geometry, as plain data, for drawing over the Unreal
// geometry it is meant to approximate.
//
// ⚠ THIS EXISTS BECAUSE "WE SENT IT" AND "THE SOLVER KEPT IT" ARE DIFFERENT CLAIMS, and every
// collision defect in this workstream so far has lived in the gap between them: 23 mesh collisions
// silently dropped by MuJoCo while every counter reported success; a height field sampled 120 m
// from where it was reported to be; a robot whose base_link has 100 % of its visual volume outside
// any collision shape. None of those is visible in a log line. All of them are obvious the instant
// the solver's own geometry is drawn on top of the level.
//
// ⚠ Frame: SOLVER frame — URDF/ROS, right-handed, Z-up, metres, origin at the **Unreal world
// origin** (UrdfStaticWorld.hpp). The drawing side applies the one Y-mirror conversion the rest of
// the URDF path uses; nothing here knows what an FVector is.
#pragma once

#include "urdf/UrdfCollisionShape.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace urdf {

/// Every collision primitive in one solver scene at one instant.
struct CollisionDebugSnapshot {
    std::string backend;  ///< "mujoco" | "box3d"
    std::vector<CollisionShape> geoms;

    /// Geoms left out because the budget ran out, or whose kind this seam does not model.
    ///
    /// ⚠ Counted, never silently dropped. An overlay missing the one geom that matters, with no
    /// sign that anything is missing, is worse than no overlay.
    size_t omitted = 0;
    /// Kinds encountered that are not represented above, for the omission to be explainable.
    std::vector<std::string> omitted_kinds;

    double solver_time = 0;

    size_t triangleCount() const
    {
        size_t n = 0;
        for (const CollisionShape& g : geoms)
            n += g.indices.size() / 3;
        return n;
    }
};

/// How much to read back. A full level mirror is tens of thousands of prisms and drawing all of
/// them at once is both useless and slow, so the caller says what it wants to look at.
struct CollisionDebugFilter {
    bool include_world = true;   ///< the mirrored level, ground plane and height field
    bool include_robots = true;  ///< the articulations' own link collision

    /// Only geometry within this distance of `center` in x/y. 0 takes everything.
    ///
    /// ⚠ A radius here bounds the DRAWING, never the physics. It exists so an operator can look at
    /// the metre around a wheel without the rest of the map on screen.
    Vec3 center;
    double radius = 0;

    /// Hard cap on geoms returned, so a mistaken filter cannot stall the game thread. Overflow is
    /// reported in `omitted`.
    size_t max_geoms = 4000;
};

} // namespace urdf
