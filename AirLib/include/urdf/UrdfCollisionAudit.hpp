// R2 — the two collision worlds, made measurable.
//
// Box3D drives the robot on convex hulls built from <collision>. LiDAR, echo and distance sensors
// trace the *Unreal* collision world, which for a URDF link is the mesh Unreal was given — the
// <visual> geometry. Nothing forces those to agree, and where they disagree a wheel rolls on one
// surface while the sensors see another.
//
// ExoMy sharpened this from a margin effect into a body effect: **only 6 of its 23 links declare
// any <collision> at all**, and the chassis is not one of them. So the robot Box3D simulates is six
// cylinders, while the robot a LiDAR sees is a full rover. A URDF that looks complete can have
// almost no simulated collision surface, and nothing in the file says so.
//
// This audit computes that discrepancy from the URDF, before anything is simulated. It is the
// explicit consistency check Gate 2 requires, and it is deliberately a *measurement* rather than a
// pass/fail: a rover whose chassis never touches anything is a legitimate model, and the operator
// needs the number, not a veto. Callers that do want a veto apply their own threshold to the
// summary — see UrdfCollisionAudit_test and the `worst_*` fields.
#pragma once

#include "urdf/UrdfModel.hpp"

#include <string>
#include <vector>

namespace urdf {

/// Axis-aligned bounds in the **link** frame, with each geometry's <origin> already applied.
struct Bounds {
    bool valid = false;
    Vec3 min, max;

    void expand(const Vec3& p);
    Vec3 extents() const;
    double volume() const;
    /// Longest half-extent — a scalar "how big is this" that survives thin plates better than
    /// volume does.
    double radius() const;
};

struct LinkAudit {
    int link = -1;
    std::string name;

    bool has_collision = false;   ///< Box3D can push on this link
    bool has_visual = false;      ///< sensors can see this link
    bool mesh_unresolved = false; ///< a <mesh> filename that could not be found on disk

    Bounds collision;             ///< union over all <collision> geometry
    Bounds visual;                ///< union over all <visual> geometry

    double mass = 0;

    /// Fraction of the visual bounding volume not covered by the collision bounding volume, in
    /// [0,1]. 1.0 means "visible but entirely un-simulated"; 0.0 means the two agree in extent.
    ///
    /// ⚠ A bounding-box measure, not a surface one. It is a screen for gross divergence — the
    /// 17-links-with-no-collision case — and does not certify the two surfaces agree where both
    /// exist. Deliberately crude and honestly labelled rather than precise and expensive.
    double uncovered_fraction = 0;
};

struct CollisionAudit {
    std::string robot;
    std::vector<LinkAudit> links;

    int links_total = 0;
    int links_with_collision = 0;
    int links_with_visual = 0;
    int links_visual_only = 0;      ///< seen by sensors, invisible to the solver — the R2 case
    int links_unresolved_mesh = 0;

    double mass_total = 0;
    double mass_with_collision = 0; ///< mass Box3D can actually collide with

    /// Worst single-link divergence, and which link it was. This is the headline number: it says
    /// where the two worlds disagree most, not merely that they do.
    double worst_uncovered_fraction = 0;
    std::string worst_link;

    /// Fraction of the robot's mass carried by links with no <collision> at all.
    double uncollidable_mass_fraction() const
    {
        return mass_total > 0 ? (mass_total - mass_with_collision) / mass_total : 0.0;
    }

    /// Human-readable report, one line per interesting link plus a summary. Written to be pasted
    /// into a log and understood without this header.
    std::string report() const;
};

/// Audit `model`. `urdf_dir` and `search_roots` are used to resolve <mesh> filenames — a mesh that
/// cannot be found is counted and flagged rather than skipped, because "no geometry" and "geometry
/// I failed to load" must not look alike.
CollisionAudit auditCollisionConsistency(const Robot& model, const std::string& urdf_dir = ".",
                                         const std::vector<std::string>& search_roots = {});

} // namespace urdf
