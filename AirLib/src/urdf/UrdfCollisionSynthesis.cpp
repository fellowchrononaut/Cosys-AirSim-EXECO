#include "urdf/UrdfCollisionSynthesis.hpp"

#include <cstdio>

namespace urdf {

SynthesisResult synthesizeCollisionFromVisual(Robot& model)
{
    SynthesisResult out;

    for (Link& link : model.links) {
        out.mass_total += link.has_inertial ? link.inertial.mass : 0.0;

        // A link that declares any collision at all has had its contact geometry chosen by its
        // author. Adding to it would be a different feature, and a much more dangerous one.
        if (!link.collisions.empty()) continue;

        if (link.visuals.empty()) {
            out.links_with_no_geometry.push_back(link.name);
            continue;
        }

        SynthesizedLink rec;
        rec.link = link.name;
        rec.mass = link.has_inertial ? link.inertial.mass : 0.0;

        for (const Visual& v : link.visuals) {
            Collision c;
            // Named so the provenance survives into every downstream report and error message.
            // A shape that turns out to be wrong should say where it came from.
            c.name = v.name.empty() ? (link.name + "_from_visual") : (v.name + "_from_visual");
            c.origin = v.origin;
            c.geometry = v.geometry;
            c.synthesized = true;
            link.collisions.push_back(std::move(c));
            ++rec.visuals_copied;
        }

        out.mass_recovered += rec.mass;
        out.links.push_back(std::move(rec));
    }

    return out;
}

std::string SynthesisResult::report() const
{
    char buf[512];
    std::string s;

    s += "Collision synthesised from <visual> - UrdfCollisionFromVisual is ON\n";
    s += "  These links declared NO <collision>. Their visual geometry is now their contact\n";
    s += "  geometry, so the robot's physics differs from the URDF as written.\n\n";

    std::snprintf(buf, sizeof(buf), "  links given collision   : %zu\n", links.size());
    s += buf;

    if (mass_total > 0) {
        std::snprintf(buf, sizeof(buf),
                      "  mass made collidable    : %.3f kg of %.3f kg (%.1f %%)\n",
                      mass_recovered, mass_total, 100.0 * mass_recovered / mass_total);
        s += buf;
    }

    s += "\n  link                          visuals    mass(g)\n";
    s += "  ---------------------------------------------------\n";
    for (const SynthesizedLink& l : links) {
        std::snprintf(buf, sizeof(buf), "  %-28s  %5zu    %8.1f\n", l.link.c_str(),
                      l.visuals_copied, l.mass * 1000.0);
        s += buf;
    }

    if (!links_with_no_geometry.empty()) {
        s += "\n  ! these links have neither <collision> nor <visual> and remain absent from the\n";
        s += "    dynamics (usually frame markers, occasionally a mistake): ";
        for (size_t i = 0; i < links_with_no_geometry.size(); ++i)
            s += (i ? ", " : "") + links_with_no_geometry[i];
        s += "\n";
    }

    s += "\n  ! A <mesh> becomes ONE CONVEX HULL, because Box3D mesh shapes are static-only. Any\n";
    s += "    concavity is filled in, so these links are FATTER than they look. Convex\n";
    s += "    decomposition is the real fix and is not implemented.\n";

    return s;
}

} // namespace urdf
