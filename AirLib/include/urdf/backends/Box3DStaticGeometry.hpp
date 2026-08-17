// Cooked static world geometry, shared between Box3D worlds.
//
// The measurement that shapes this file (tests/test_static_geometry.cpp, 80 k triangles):
//
//     b3CreateMesh       cook once          17.4   ms
//     b3CreateMeshShape  attach to a world   0.006 ms      ~3000x cheaper
//
// So the whole design is: **cook once, attach many**. That is what makes analysis doc §6.0c's
// whole-level scope affordable at N robots, and what keeps `reset()` cheap — reset destroys and
// rebuilds a robot's world (§6.4), and re-attaching a cook costs microseconds where re-cooking
// would cost tens of milliseconds per reset.
//
// ⚠ Two verified facts underpin this, both non-obvious, both tested (and clean under ASan):
//
//   1. `b3CreateMeshShape` **references** the mesh; it does not copy it. So one `b3MeshData*` backs
//      shapes in as many worlds as you like — but it must outlive every shape that references it,
//      which is what the shared_ptr ownership here is for.
//   2. `b3DestroyWorld` does **not** free a mesh its shapes referenced. A surviving world keeps
//      colliding against it, and a freshly built world can attach to it afterwards. Without this,
//      one robot's reset would leave every other robot standing on freed memory.
//
// ⚠ Contrast, and it is a trap: `b3CreateHullShape` **clones** the hull, so hulls are freed
// immediately after creating the shape and are not cached. The two APIs differ. Treating them
// alike leaks in one direction and dangles in the other.
#pragma once

#include "urdf/UrdfStaticWorld.hpp"

#include <box3d/box3d.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace b3urdf {

/// One cook of one `urdf::StaticWorld`, attachable to any number of `b3World`s.
///
/// Obtained through `acquire()`, never constructed directly, so that two robots handed the same
/// `StaticWorld` provably share one cook rather than merely being likely to.
class Box3DStaticGeometry {
public:
    ~Box3DStaticGeometry();

    Box3DStaticGeometry(const Box3DStaticGeometry&) = delete;
    Box3DStaticGeometry& operator=(const Box3DStaticGeometry&) = delete;

    /// Cook `world`, or return the existing cook if one is already live for that exact pointer.
    ///
    /// Keyed on **pointer identity**, not on content: two equal-but-distinct StaticWorlds cook
    /// twice. That is deliberate — content hashing a level-sized mesh to save a cook that the
    /// caller could have saved by sharing the pointer would be paying to work around a caller bug,
    /// and it would hide the bug. The plugin mirrors the level once and hands the same pointer to
    /// every robot.
    ///
    /// The cache holds weak references, so the cook dies with the last robot using it.
    ///
    /// Returns nullptr for a null or empty `world`. Thread-safe.
    static std::shared_ptr<const Box3DStaticGeometry> acquire(std::shared_ptr<const urdf::StaticWorld> world);

    /// Create the static bodies and shapes in `world`. Called once per world build, and again on
    /// every `reset()` because reset destroys the world.
    ///
    /// ⚠ Adds to whatever is already there; it does not clear. A world must be freshly created.
    void attachTo(b3WorldId world) const;

    const urdf::StaticWorld& source() const { return *source_; }

    // --- what the cook actually cost, for logging and for the reset budget ------------------
    double cookMilliseconds() const { return cook_ms_; }
    size_t cookedMeshCount() const { return meshes_.size(); }
    size_t triangleCount() const { return source_->triangleCount(); }

    /// Shapes that were dropped because Box3D refused them (a degenerate hull, a mesh with fewer
    /// than three vertices). Non-zero means the level is mirrored *incompletely* — the robot will
    /// drive through something visible — so it is reported, never swallowed.
    size_t rejectedShapeCount() const { return rejected_; }

private:
    explicit Box3DStaticGeometry(std::shared_ptr<const urdf::StaticWorld> world);

    void cook();

    std::shared_ptr<const urdf::StaticWorld> source_;

    /// Cooked tri-meshes, in the order the mesh shapes are encountered while walking `source_`.
    /// Owned here and destroyed only when the last robot lets go, *after* every world that
    /// referenced them is gone.
    std::vector<b3MeshData*> meshes_;

    double cook_ms_ = 0;
    size_t rejected_ = 0;
};

} // namespace b3urdf
