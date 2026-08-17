#include "urdf/UrdfMimic.hpp"

#include <sstream>
#include <stdexcept>

namespace urdf {

const char* toString(MimicRole role)
{
    switch (role) {
    case MimicRole::None:        return "none";
    case MimicRole::Cosmetic:    return "cosmetic";
    case MimicRole::LoadBearing: return "load-bearing";
    }
    return "?";
}

namespace {

/// Format a mass the way an operator reads it: grams below 1 kg, kilograms above.
std::string massText(double kg)
{
    std::ostringstream s;
    s.precision(3);
    if (kg < 1.0) s << std::fixed << (kg * 1000.0) << " g";
    else          s << std::fixed << kg << " kg";
    return s.str();
}

bool isSingleDof(JointType t)
{
    return t == JointType::Revolute || t == JointType::Continuous || t == JointType::Prismatic;
}

} // namespace

std::vector<MimicClassification> classifyMimicJoints(const Robot& robot, const MimicPolicy& policy)
{
    std::vector<MimicClassification> out;
    const double total_mass = robot.totalMass();

    for (size_t ji = 0; ji < robot.joints.size(); ++ji) {
        const Joint& j = robot.joints[ji];
        if (!j.hasMimic()) continue;

        MimicClassification c;
        c.joint = static_cast<int>(ji);
        c.source_joint = robot.findJoint(j.mimic_source_joint);
        c.subtree_mass = robot.subtreeMass(j.child_index);
        // declared_only: a shape synthesised from <visual> says nothing about whether the URDF's
        // author intended this coupling to carry load. Counting it would mean that switching on
        // UrdfCollisionFromVisual silently demotes a cosmetic eye into a servo-follower
        // approximation — a physics change caused by a rendering-motivated setting.
        c.subtree_collides = robot.subtreeHasCollision(j.child_index, /*declared_only=*/true);

        // A <mimic> on a joint with no scalar coordinate has nothing to mimic *with*. Neither path
        // can be built for it, so it is load-bearing by elimination and will be refused with a
        // message that says so, rather than quietly treated as cosmetic.
        if (!isSingleDof(j.type)) {
            c.role = MimicRole::LoadBearing;
            c.reason = "joint type has no single-DoF coordinate, so a mimic relation cannot be "
                       "evaluated for it";
            out.push_back(std::move(c));
            continue;
        }

        const bool mass_frac_ok = total_mass <= 0.0 ||
                                  c.subtree_mass <= policy.cosmetic_mass_fraction * total_mass;
        const bool mass_abs_ok = c.subtree_mass <= policy.cosmetic_mass_kg;

        std::ostringstream why;
        if (c.subtree_collides) {
            c.role = MimicRole::LoadBearing;
            why << "subtree declares <collision>, so it can transmit contact force";
        }
        else if (!mass_abs_ok || !mass_frac_ok) {
            c.role = MimicRole::LoadBearing;
            why << "subtree mass " << massText(c.subtree_mass) << " exceeds the cosmetic ceiling (";
            if (!mass_abs_ok) why << "limit " << massText(policy.cosmetic_mass_kg);
            else why << policy.cosmetic_mass_fraction * 100.0 << "% of "
                     << massText(total_mass) << " total";
            why << ")";
        }
        else {
            c.role = MimicRole::Cosmetic;
            why << "no <collision> anywhere in the subtree and subtree mass "
                << massText(c.subtree_mass) << " is negligible, so the joint cannot carry load";
        }
        c.reason = why.str();
        out.push_back(std::move(c));
    }

    return out;
}

void requireMimicSupported(const Robot& robot,
                           const std::vector<MimicClassification>& classified,
                           const MimicPolicy& policy)
{
    if (policy.allow_servo_follower) return;

    for (const MimicClassification& c : classified) {
        if (c.role != MimicRole::LoadBearing) continue;

        const Joint& j = robot.joints[c.joint];
        std::ostringstream m;
        m << "joint '" << j.name << "': <mimic joint=\"" << j.mimic_source_joint
          << "\" multiplier=\"" << j.mimic_multiplier << "\" offset=\"" << j.mimic_offset
          << "\"> is load-bearing (" << c.reason << "). Box3D has no gear or coupled-joint "
             "constraint, so this coupling can only be approximated by a servo-follower, which "
             "lags and whose steady-state accuracy is bounded by the open M4 position-control gap. "
             "Loading it silently would produce a robot that does not match its URDF. Set "
             "MimicPolicy::allow_servo_follower to accept the approximation deliberately.";
        throw std::runtime_error(m.str());
    }
}

} // namespace urdf
