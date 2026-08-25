// One collision primitive, solver-neutral, as plain data.
//
// ⚠ SHARED BETWEEN THE DEBUG VIEW AND THE MPM COLLIDER REGISTRY ON PURPOSE. The overlay
// (UrdfCollisionDebug.hpp) draws these; a Newton MPM collider descriptor (UrdfPhysicsDescriptor.hpp)
// is registered from them. They are the same geometry, and giving each consumer its own spelling of
// "a convex hull in a body frame" is how a view and an input quietly stop agreeing about what the
// solver contains — which is the exact failure this workstream keeps finding.
//
// ⚠ Frame: SOLVER frame — URDF/ROS, right-handed, Z-up, metres, origin at the **Unreal world
// origin** (UrdfStaticWorld.hpp). Nothing here knows what an FVector is.
#pragma once

#include "urdf/UrdfModel.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace urdf {

/// One collision primitive as the solver holds it.
struct CollisionShape {
    enum class Kind { Sphere, Capsule, Cylinder, Box, Mesh, HeightField, Plane };

    /// Where this description came from.
    ///
    /// ⚠ NOT DECORATION. `Realised` was read back out of the compiled solver model — MuJoCo's
    /// qhull output, Box3D's live shapes — and is the only kind of answer that can contradict what
    /// we believe we built. `Submitted` is what we handed the solver and have not verified it
    /// kept. An overlay that conflated the two would be the same class of diagnostic as the
    /// "2704/2704 traces hit" line that was true about the wrong place.
    enum class Provenance { Realised, Submitted };

    Kind kind = Kind::Mesh;
    Provenance provenance = Provenance::Realised;

    /// The geom's frame in the solver world, at the instant of the snapshot.
    Vec3 position;
    Quat orientation;

    double radius = 0;       ///< Sphere, Capsule, Cylinder
    double half_length = 0;  ///< Capsule, Cylinder: centre to cap centre along local +z
    Vec3 half_extents;       ///< Box; also the x/y half-extent of a HeightField or finite Plane

    /// Mesh: vertices in the geom's own frame, plus triangle indices.
    ///
    /// ⚠ For MuJoCo these are the CONVEX HULL qhull built, not the triangles we submitted — which
    /// is exactly the question being asked when someone says the collision looks bigger than the
    /// asset. For Box3D they are the real triangles, because Box3D cooks them.
    std::vector<Vec3> vertices;
    std::vector<int> indices;

    /// HeightField: row-major elevations in metres above `position.z`, spanning ±half_extents in
    /// x and y. Already denormalised — a caller must never have to know how the engine stores them.
    int rows = 0;
    int cols = 0;
    std::vector<float> heights;

    /// Contact inflation the solver applies on top of the shape, in metres. Zero for everything we
    /// author, and reported rather than assumed: a non-zero margin makes a geom collide bigger
    /// than it is drawn, which looks exactly like a modelling error and is not one.
    double margin = 0;

    /// Owning body, qualified — "Rover1/wheel_lf", or "level" for world geometry.
    std::string label;
    /// World/static geometry rather than a robot's own links. Drawn in a different colour, because
    /// "the level is fat here" and "the robot is fat here" have different fixes.
    bool is_world = false;
};

} // namespace urdf
