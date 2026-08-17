// The static world a URDF robot stands on, as plain data.
//
// This is the second seam in the design, and it exists for the same reason as UrdfRobotBackend:
// **the side that knows about Unreal must not be the side that knows about the solver.** The plugin
// walks the level and produces one of these on the game thread at BeginPlay; a backend consumes it
// without ever seeing an FVector, a UBodySetup or a UWorld.
//
// ⚠ Frame: URDF/ROS — right-handed, Z-up, metres — which is Box3D's convention exactly, and whose
// origin is the **Unreal world origin**, not any robot's spawn point. That last part is not a
// detail. Anchoring the solver frame per robot (which is what UrdfBotSimApi did before static
// geometry existed) makes one cooked level un-shareable, because the same triangle would need a
// different position for every robot. Everything downstream of this file — the shared cook, and the
// single shared world of analysis doc §6.0c stage 3 — depends on the origin being robot-independent.
//
// ⚠ Scope, per analysis doc §6.0c: **the whole level**, not a radius around the robot. A
// radius-limited mirror would make the physical world depend on which robot you are, which is the
// class of silent divergence this workstream exists to avoid. It is affordable because cooking is
// shared: measured at 17.4 ms per 80 k triangles once, against 0.006 ms to attach that cook to
// another world (tests/test_static_geometry.cpp).
#pragma once

#include "urdf/UrdfModel.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace urdf {

/// What a mirrored collision primitive is. These are the four things Unreal's cooked collision
/// actually produces (`FKAggregateGeom` convex/box/sphere/sphyl, plus cooked tri-meshes), reduced
/// to what every rigid-body engine can represent.
///
/// ⚠ `Mesh` is **static-only** in Box3D (`docs/loose_ends.md` #7) — it cannot take part in
/// dynamic-vs-dynamic contact. That is not a limitation here, because everything in this file is
/// static by construction, but it is why a robot's own `<collision>` cannot use the same path.
enum class StaticShapeKind { Hull, Mesh, Sphere, Capsule };

/// One collision primitive, in the frame of the StaticBody that owns it.
struct StaticShape {
    StaticShapeKind kind = StaticShapeKind::Hull;

    /// Hull: the point cloud to hull. Mesh: the triangle vertices.
    std::vector<Vec3> points;

    /// Mesh only. Three indices per triangle, wound so that front faces point out of the solid.
    std::vector<int> indices;

    /// Sphere: centre. Capsule: the first hemisphere centre.
    Vec3 center_a;
    /// Capsule: the second hemisphere centre. Unused otherwise.
    Vec3 center_b;
    /// Sphere and capsule.
    double radius = 0;

    size_t triangleCount() const { return indices.size() / 3; }
};

/// One static object in the level, with its shapes expressed in its own frame and its frame placed
/// in the world.
///
/// Body-local shapes rather than pre-transformed world-space ones, deliberately: it is what lets a
/// mesh cooked from a `UStaticMesh` asset be reused by every instance of that asset placed in the
/// level, which is the common case in an Unreal map and the difference between cooking a level once
/// and cooking it once per placed rock.
struct StaticBody {
    std::string name;  ///< the Unreal actor's path name, for logs and for cache keys

    Vec3 position;
    Quat orientation;

    std::vector<StaticShape> shapes;

    double friction = 0.7;
    double restitution = 0.0;
};

/// A body mirrored from outside the solver whose pose is **pushed in every step** rather than
/// frozen at load.
///
/// This is analysis doc §6.0c stage 2. It is what a level object that actually moves needs, and
/// what lets a Box3D robot feel another vehicle drive into it: the pose comes from Unreal each
/// frame and the solver treats the body as infinitely massive and externally driven.
///
/// ⚠ **One-directional by construction.** The robot is pushed by these bodies; they are never
/// pushed back, because their pose is dictated from outside. A rover shoved by a mirrored husky
/// moves; the husky does not feel the rover through Box3D. The reverse coupling comes free through
/// Unreal, since the robot's links carry Unreal collision — crude and non-momentum-conserving in
/// both directions, but no worse than drone-versus-husky is today. True two-way contact needs
/// §6.0c stage 3, a shared b3World.
///
/// ⚠ **No Mesh shapes.** Box3D's mesh and height-field shapes are static-only
/// (`docs/loose_ends.md` #7) and cannot take part in contact with a moving body. A kinematic mirror
/// therefore uses convex hulls, so a concave object is **fatter than it looks** — the same
/// limitation, and the same honesty requirement, as a robot link's `<mesh>` collision.
struct KinematicBody {
    std::string name;

    /// Pose at registration. Updated thereafter through the backend's setKinematicPose.
    Vec3 position;
    Quat orientation;

    /// Body-local, and never StaticShapeKind::Mesh — see above.
    std::vector<StaticShape> shapes;

    double friction = 0.7;
    double restitution = 0.0;
};

/// A whole mirrored level.
///
/// Held by `std::shared_ptr<const StaticWorld>` everywhere downstream, because its identity is what
/// the cooked-geometry cache is keyed on: two robots handed the same pointer share one cook, and
/// two robots handed equal-but-distinct worlds do not. Sharing the pointer is therefore not an
/// optimisation detail, it is how the saving is claimed.
struct StaticWorld {
    std::vector<StaticBody> bodies;

    /// Bodies whose pose is driven from outside each step. Deliberately kept in the same object as
    /// the static bodies: they are produced by one walk of the level and their membership is a
    /// property of that walk, so splitting them would let the two drift apart.
    ///
    /// ⚠ Unlike `bodies`, these are NOT shareable between robots' cooked geometry — each robot's
    /// world needs its own kinematic instances, because each is pushed independently. Only the
    /// static cook is shared.
    std::vector<KinematicBody> kinematic;

    /// True if the mirror ran and found nothing. Distinguished from "never ran" by the caller
    /// holding a null shared_ptr for the latter — a robot with no static world must fall, loudly,
    /// rather than quietly stand on an empty one.
    bool mirrored = false;

    /// Unreal units per metre in force when this was mirrored. Recorded so a mismatch against the
    /// robot's own scale is detectable rather than showing up as a level at 1/100 size.
    double world_to_meters = 100.0;

    size_t shapeCount() const
    {
        size_t n = 0;
        for (const StaticBody& b : bodies)
            n += b.shapes.size();
        return n;
    }

    size_t triangleCount() const
    {
        size_t n = 0;
        for (const StaticBody& b : bodies)
            for (const StaticShape& s : b.shapes)
                n += s.triangleCount();
        return n;
    }

    size_t meshCount() const
    {
        size_t n = 0;
        for (const StaticBody& b : bodies)
            for (const StaticShape& s : b.shapes)
                n += (s.kind == StaticShapeKind::Mesh) ? 1u : 0u;
        return n;
    }
};

} // namespace urdf
