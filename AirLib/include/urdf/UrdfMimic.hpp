// Deciding how a URDF <mimic> joint should be honoured.
//
// <mimic> states a purely kinematic relation, q_this = multiplier * q_source + offset. Box3D has
// no gear or coupled-joint constraint, so it cannot express that directly. But — per analysis doc
// §6.5 — it does not always need to, because Gate 2 renders in Unreal while Box3D owns the poses.
// A constraint that carries no load can be resolved kinematically, exactly, on the way out.
//
// So there are two cases, and which applies is a property of the robot, not a preference:
//
//   Cosmetic    — the mimicking joint's subtree touches nothing and weighs nothing, so it cannot
//                 transmit force. Create no Box3D joint and no dynamic body; compute the pose from
//                 the source joint's angle whenever link state is read. This is EXACT, and it is
//                 strictly better than the free-joint fallback, which would visibly droop.
//                 ExoMy's `pupil_right_joint` (~0.9 g, no <collision>) is the worked example.
//
//   Load-bearing — a parallel gripper, a four-bar linkage. The joint really does carry force, and
//                 the best available answer is a servo-follower: drive the mimic joint's own motor
//                 toward multiplier * q_source + offset every step. This is an APPROXIMATION.
//
// The resulting policy is the one this workstream applies everywhere: **exact handling is
// automatic; approximate handling is opt-in.** Cosmetic mimics load without ceremony. A
// load-bearing mimic is refused by default, with a message naming the joint and why it was
// classified that way, and is honoured only when the caller asks for the servo-follower knowing
// what it costs. Silently approximating is the UrdfSim failure this workstream exists to avoid.
#pragma once

#include "urdf/UrdfModel.hpp"

#include <string>
#include <vector>

namespace urdf {

enum class MimicRole {
    None,         ///< the joint carries no <mimic>
    Cosmetic,     ///< kinematic-only; resolved exactly, outside the solver
    LoadBearing,  ///< may transmit force; needs the servo-follower approximation
};

const char* toString(MimicRole role);

/// Thresholds for "this cannot possibly carry load". Both mass tests must pass, and the subtree
/// must declare no <collision> at all.
///
/// Two mass thresholds rather than one, because either alone misjudges a plausible robot: a bare
/// fraction would make a 10 kg link on a 1-tonne machine "cosmetic", and a bare absolute would
/// misjudge anything very small or very large. Requiring both keeps the error on the safe side —
/// classifying a load-bearing joint as cosmetic is the dangerous direction, because its link would
/// then vanish from the physics entirely, so the bar to *be* cosmetic is deliberately high.
struct MimicPolicy {
    double cosmetic_mass_fraction = 0.01;   ///< of the robot's total mass
    double cosmetic_mass_kg = 0.05;         ///< absolute ceiling

    /// Honour load-bearing mimics with a servo-follower instead of refusing the model.
    ///
    /// ⚠ Off by default, and the default is the honest one. The follower is a control loop, not a
    /// constraint: it lags, and its steady-state accuracy is bounded by the same position-control
    /// gap recorded as M4 (no calibrated kp↔hertz conversion, no gravity-compensation feedforward).
    /// Turning it on is a statement that an approximate linkage is acceptable for the task.
    bool allow_servo_follower = false;

    /// Spring frequency and damping used to drive a servo-follower joint, in Box3D's units.
    /// Deliberately exposed: when M4 closes these become derived rather than guessed, and callers
    /// that already tuned a value should not have it silently changed underneath them.
    double follower_hertz = 20.0;
    double follower_damping_ratio = 1.0;
};

struct MimicClassification {
    int joint = -1;         ///< index into Robot::joints
    int source_joint = -1;  ///< index into Robot::joints
    MimicRole role = MimicRole::None;

    double subtree_mass = 0;      ///< kg below and including the mimicking joint's child link
    bool subtree_collides = false;///< any <collision> in that subtree

    /// Why this classification was reached, in a form fit to print. Populated always, including
    /// for Cosmetic, so an operator can audit the decision rather than trust it.
    std::string reason;
};

/// Classify every <mimic> joint in the model. Order matches Robot::joints, and only mimicking
/// joints appear. Never throws: a caller that wants refusal enforces it against the result, which
/// keeps "what did you decide" separable from "what will you allow".
std::vector<MimicClassification> classifyMimicJoints(const Robot& robot,
                                                     const MimicPolicy& policy = {});

/// Throw std::runtime_error if the model contains a mimic joint the policy cannot honour. The
/// message names the joint, its source, the measured mass and collision state, and the option that
/// would allow it — enough to decide without reading this header.
void requireMimicSupported(const Robot& robot,
                           const std::vector<MimicClassification>& classified,
                           const MimicPolicy& policy);

} // namespace urdf
