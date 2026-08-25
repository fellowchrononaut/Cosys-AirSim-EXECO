// Reading collision geometry back out of a compiled MuJoCo model — ONE implementation, two callers.
//
// ⚠ THIS EXISTS SO THERE IS NOT A SECOND COPY, for the same reason MuJoCoStaticWorld.cpp does. The
// per-robot MuJoCoUrdfBackend and the shared MuJoCoPhysicsScene both need to say what their model
// actually holds, and a debug view whose two backends disagree about how to read an mjModel is a
// debug view that cannot be used to compare them. The one thing that legitimately differs is how a
// body is NAMED, so that is the callback.
#pragma once

#include "urdf/UrdfCollisionDebug.hpp"
#include "urdf/UrdfPhysicsDescriptor.hpp"

#include <functional>
#include <string>

struct mjModel_;
struct mjData_;

namespace urdf {

/// Answer "who owns body `bodyid`, and is it world geometry" for one model.
///
/// ⚠ Body 0 is the world body by MuJoCo's construction, but a caller may own bodies it still wants
/// reported as world geometry, so the flag is the callback's answer and not this file's guess.
using MuJoCoBodyLabeller = std::function<std::string(int bodyid, bool& is_world)>;

/// Walk `model`/`data` and describe every COLLIDING geom, in solver frame, at this instant.
///
/// Reads `data->geom_xpos`/`geom_xmat` for live poses, `mesh_vert`/`mesh_face` for the convex hull
/// MuJoCo built, and denormalises height-field elevations back to metres. Appends to `out`; the
/// caller sets `out.backend` and clears it.
void readMuJoCoCollisionGeometry(const mjModel_* model, const mjData_* data,
                                 const CollisionDebugFilter& filter,
                                 const MuJoCoBodyLabeller& label, CollisionDebugSnapshot& out);

/// Describe every body MuJoCo holds for one articulation as an MPM-registerable collider.
///
/// `owns(bodyid)` selects the bodies belonging to the caller (one articulation in a shared scene,
/// or every non-world body in a private one); `stable_id(bodyid)` names them.
///
/// ⚠ Inertia comes back as the ISOLATED RIGID-BODY tensor MuJoCo integrates, converted from its
/// diagonal-in-principal-frame storage. It is NOT an articulated effective inertia, and the
/// descriptor says so — see LinkInertialDescriptor::is_articulated_effective_inertia.
void describeMuJoCoColliders(const mjModel_* model, const mjData_* data,
                             const std::function<bool(int bodyid)>& owns,
                             const MuJoCoBodyLabeller& stable_id, PhysicsColliderSet& out);

} // namespace urdf
