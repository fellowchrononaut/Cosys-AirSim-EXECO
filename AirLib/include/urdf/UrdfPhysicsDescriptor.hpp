// The canonical physical description of one body, as the SOLVER holds it — plan §11.1's collider
// registration data, expressed once so every consumer reads the same numbers.
//
// ⚠ THIS IS THE PHASE-2 PIECE THE NEWTON MPM PHASES ARE BLOCKED ON. A deformable-terrain sidecar
// cannot register a collider from a pose alone: it needs mass, centre of mass, inertia, velocity,
// geometry and a coupling classification, and until now the portable backend interface exposed
// none of them per link. `totalMass()` was the whole of it.
//
// ⚠ INVARIANT 0 (plan §5), and it is the one thing here that must not be got wrong: COM and
// inertia are **body-local**; pose, twist, contact positions and impulses are **world-space**; and
// angular reduction uses `COM_world = body_pose * COM_local`. A descriptor that mixed those would
// produce reactions with the right magnitude about the wrong point, which looks like a tuning
// problem and is not one.
#pragma once

#include "urdf/UrdfCollisionShape.hpp"
#include "urdf/UrdfModel.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace urdf {

/// How a body may take part in a foreign solver's contact.
enum class CouplingRole {
    /// Never moves. The foreign solver may treat it as boundary geometry.
    Static,

    /// Pushes the foreign solver and receives nothing back.
    ///
    /// ⚠ THIS IS THE ONLY ROLE M2 MAY EMIT FOR AN ARTICULATED LINK, by the plan's own decision
    /// (§11.1). See `LinkInertialDescriptor::is_articulated_effective_inertia` for why.
    KinematicOneWay,

    /// Receives reduced linear/angular impulse back from the foreign solver.
    ///
    /// ⚠ NOT EMITTABLE FOR AN ARTICULATED LINK YET. Claiming it without a validated effective
    /// inertia would give a rover sinkage and traction numbers that are wrong in a way no log line
    /// shows — the run would look physical and be arbitrary. A free-floating single body is the
    /// only case this is honest for today.
    DynamicTwoWay
};

/// Where an inertial number came from, and therefore what it may be trusted for.
enum class InertiaProvenance {
    /// Straight out of the URDF `<inertial>` block, unverified against the solver.
    UrdfDeclared,
    /// Read back out of the compiled solver — what it will actually integrate.
    SolverRealised,
    /// Derived. `approximation` must name how.
    Approximated
};

/// Mass properties of one link.
struct LinkInertialDescriptor {
    double mass = 0;

    /// Invariant 0: BODY-LOCAL.
    Vec3 com_local;

    /// Inertia about the centre of mass, body-local, row-major 3x3.
    ///
    /// Stored dense rather than as a diagonal plus a frame because the two backends disagree —
    /// Box3D keeps a full tensor, MuJoCo keeps a diagonal in a principal frame — and converting
    /// once here is better than making every consumer handle both.
    double inertia_local[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    InertiaProvenance provenance = InertiaProvenance::UrdfDeclared;

    /// ⚠ FALSE, ALWAYS, FOR AN ARTICULATED LINK — AND THAT IS THE POINT OF THE FIELD.
    ///
    /// What is above is the ISOLATED RIGID-BODY inertia of this link. It is NOT the
    /// configuration-dependent operational-space / contact effective inertia of the link inside its
    /// constrained articulation, and the two are not close: a rover wheel rigidly geared to a
    /// chassis presents something nearer the vehicle's mass to a contact than its own.
    ///
    /// Plan §11.1 records this as a blocker rather than a mapping gap, and requires that any
    /// approximation be explicitly validated and labelled before two-way coupling is claimed. This
    /// flag is that label. A consumer that needs a true effective inertia must refuse to proceed
    /// while it is false rather than quietly using the isolated value.
    bool is_articulated_effective_inertia = false;

    /// Free text naming the approximation when `provenance == Approximated`. Empty otherwise.
    std::string approximation;

    /// Optional backend-supplied mean inverse inertia, when the backend can compute one from the
    /// ARTICULATION rather than the isolated link.
    ///
    /// ⚠ Closer to an effective inertia than the tensor above, and still not one. MuJoCo's
    /// `body_invweight0` is a mean over translation and rotation evaluated at the model's reference
    /// configuration `qpos0` — so it is articulated but configuration-INDEPENDENT, and a leg at
    /// full extension does not present what it presents when folded. Recorded because it is real
    /// data the effective-inertia spike will want, not because it closes the gap.
    bool has_mean_inverse_inertia = false;
    double mean_inverse_mass = 0;
    double mean_inverse_rotational_inertia = 0;
};

/// Contact/material parameters, as far as a backend can report them portably.
struct ContactMaterialDescriptor {
    double friction = 0;
    double restitution = 0;
    bool reported = false;  ///< false: this backend does not expose it; do not read the zeros
};

/// One body offered to a foreign solver, complete enough to register as a collider.
struct PhysicsColliderDescriptor {
    /// Stable across the run and across reset, and unique in the world. Qualified by vehicle, so
    /// two copies of one URDF are distinguishable: "Rover1/wheel_lf".
    std::string stable_id;

    /// This link's index in its own backend's ordering, for cheap per-step updates afterwards.
    size_t link_index = 0;
    std::string link_name;

    /// Invariant 0: WORLD.
    Vec3 position;
    Quat orientation;
    Vec3 linear_velocity;
    Vec3 angular_velocity;

    LinkInertialDescriptor inertial;
    ContactMaterialDescriptor material;

    /// Body-local collision primitives — the same type the debug overlay draws, deliberately.
    std::vector<CollisionShape> shapes;

    CouplingRole role = CouplingRole::KinematicOneWay;

    /// ⚠ A link the solver realised NO collision geometry for. It is still described, because
    /// silently omitting it would make "this link does not touch the sand" indistinguishable from
    /// "this link was dropped on the way" — which is exactly how 23 collision meshes went missing
    /// in the MuJoCo path while every counter reported success.
    bool has_collision_geometry() const { return !shapes.empty(); }
};

/// Every collider one robot offers, plus what could not be described.
struct PhysicsColliderSet {
    std::string backend;
    std::vector<PhysicsColliderDescriptor> colliders;

    /// Links that exist in the articulation but could not be described, and why. Counted and
    /// named; never silently absent.
    std::vector<std::string> undescribed;

    size_t withoutCollisionGeometry() const
    {
        size_t n = 0;
        for (const PhysicsColliderDescriptor& c : colliders)
            n += c.has_collision_geometry() ? 0u : 1u;
        return n;
    }

    double totalMass() const
    {
        double m = 0;
        for (const PhysicsColliderDescriptor& c : colliders)
            m += c.inertial.mass;
        return m;
    }
};

} // namespace urdf
