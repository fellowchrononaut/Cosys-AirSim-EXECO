// Giving a link collision geometry it never declared, by copying its <visual>.
//
// Why this exists. R2 measured ExoMy, upstream and unmodified: **6 of 23 links carry <collision>,
// and 79.7 % of the robot's mass sits on links that cannot collide.** The chassis, all six steering
// links and the rocker-bogie are visible to LiDAR and absent from the dynamics. In the simulator
// that reads as a rover whose body passes through walls while its wheels bump into them.
//
// It is not an ExoMy defect. A large fraction of published URDFs omit <collision> on non-contact
// links, because in their original use — visualisation, or a solver that falls back to visual
// geometry — nothing needed it.
//
// ⚠ **Opt-in, and it must stay opt-in.** Synthesising collision changes the physics of the robot:
// links that passed through everything now collide, self-collisions become possible where the URDF
// author assumed they could not, and a visual mesh is usually *finer* than a collision proxy would
// have been. Defaulting this on would silently alter every existing model's behaviour, which is
// precisely the failure this workstream exists to avoid. The operator asks for it, and is told what
// it did.
//
// ⚠ **It composes with a real modelling error, and the two must not be confused.** Box3D can only
// take a mesh as one *convex hull* for a moving body (mesh shapes are static-only,
// docs/loose_ends.md #7). So a synthesised collision from a concave visual is the visual's convex
// hull: a C-shaped bracket becomes a solid block, and a chassis with a gap under it loses the gap.
// The robot ends up **fatter than it looks**, in a way no screenshot reveals. Convex decomposition
// is the real fix and is a separate decision.
//
// Applied to the parsed model *before* the R2 audit and before the backend build, so that
// everything downstream — the audit's numbers, the backend's shapes, the pawn's <collision>
// render fallback — sees one consistent robot rather than three different ones.
#pragma once

#include "urdf/UrdfModel.hpp"

#include <string>
#include <vector>

namespace urdf {

/// What synthesis did to one link.
struct SynthesizedLink {
    std::string link;
    size_t visuals_copied = 0;
    double mass = 0;  ///< the link's URDF mass, so the report can lead with what it recovered
};

struct SynthesisResult {
    std::vector<SynthesizedLink> links;

    /// Links that had no <collision> **and** no <visual> either, so nothing could be synthesised.
    /// These remain absent from the dynamics and are named rather than counted, because a link
    /// with neither is usually a frame marker and occasionally a mistake.
    std::vector<std::string> links_with_no_geometry;

    double mass_recovered = 0;  ///< URDF mass now able to collide that previously could not
    double mass_total = 0;

    bool empty() const { return links.empty(); }
    std::string report() const;
};

/// Copy each collision-less link's <visual> geometry into its <collision>, in place.
///
/// Only links with **zero** existing <collision> elements are touched: a link that declares even
/// one collision shape has had its contact geometry decided by its author, and second-guessing that
/// would be a different and much worse feature.
SynthesisResult synthesizeCollisionFromVisual(Robot& model);

} // namespace urdf
