// Reading collision geometry back out of a live Box3D world — ONE implementation, two callers.
//
// ⚠ THIS EXISTS SO THERE IS NOT A SECOND COPY, for the same reason MuJoCoCollisionReadback.hpp
// does. `Box3DPhysicsScene` (one shared world) and `Box3DUrdfBackend` (one private world per robot)
// must describe their geometry identically, or a side-by-side comparison of a coordinated run and a
// legacy run compares two readbacks rather than two solvers.
#pragma once

#include "urdf/UrdfCollisionDebug.hpp"
#include "urdf/UrdfPhysicsDescriptor.hpp"
#include "urdf/UrdfStaticWorld.hpp"

#include <string>

namespace b3urdf {
class Box3DRobot;
class Box3DStaticGeometry;
} // namespace b3urdf

namespace urdf {

/// Append one robot's link shapes, read back through Box3D's own accessors.
///
/// `prefix` is prepended to each link name — the vehicle it belongs to. "Something is fat here" is
/// only actionable once it says which robot and which link.
void readBox3DRobotCollision(const b3urdf::Box3DRobot& robot, const std::string& prefix,
                             const CollisionDebugFilter& filter, CollisionDebugSnapshot& out);

/// Append the mirrored level.
///
/// ⚠ Marked `Submitted`, not `Realised`. Box3D takes these triangles exactly for static bodies, so
/// the two should agree — but "should" is the word that hid 23 dropped meshes in the MuJoCo path,
/// and `Box3DStaticGeometry::rejectedShapeCount()` says outright that Box3D does refuse shapes.
/// Box3D exposes no enumeration of the bodies a cook created, so this is what we handed over.
void readBox3DStaticWorld(const StaticWorld& level, const CollisionDebugFilter& filter,
                          CollisionDebugSnapshot& out);

/// Describe one Box3D robot's links as registerable colliders (plan §11.1).
///
/// ⚠ Inertia is the ISOLATED RIGID-BODY tensor Box3D integrates (`b3Body_GetMassData`), not an
/// articulated effective inertia — the descriptor says so.
void describeBox3DColliders(const b3urdf::Box3DRobot& robot, const std::string& prefix,
                            PhysicsColliderSet& out);

} // namespace urdf
