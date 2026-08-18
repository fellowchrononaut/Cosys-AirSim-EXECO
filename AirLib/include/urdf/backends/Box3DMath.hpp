// URDF <-> Box3D geometry. The single place those two conventions meet.
//
// The happy accident worth stating explicitly: **URDF and Box3D share a convention.** Both are
// right-handed and metric; Box3D has no intrinsic up axis (b3WorldDef::gravity is just a vector),
// so setting gravity to (0,0,-9.81) makes Box3D Z-up like URDF. There is therefore NO handedness
// flip and NO unit scale here — unlike the Unreal boundary, which needs both (see
// Box3DUnreal/Box3DConversion.h). Anything in this file that looks like a conversion is about
// *joint frames*, not about coordinate systems.
#pragma once

#include "urdf/UrdfModel.hpp"

#include <box3d/box3d.h>

#include <cmath>

namespace b3urdf {

inline b3Vec3 toB3(const urdf::Vec3& v)
{
    return b3Vec3{ static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z) };
}

/// Build a world position from doubles, in whichever precision this build of box3d uses.
///
/// ⚠ Needed because `b3Pos` is **float in default builds and double under
/// BOX3D_DOUBLE_PRECISION**, so neither a plain braced init nor a `static_cast<float>` is correct
/// in both: one narrows in large-world mode, the other narrows in default mode. Both are errors
/// under clang (`-Wc++11-narrowing`) and merely warnings under gcc — and AirLib builds with clang
/// while this workstream's headless harness builds with gcc, so getting it wrong compiles cleanly
/// in the harness and fails only in the real build.
inline b3Pos toB3Pos(double x, double y, double z)
{
    using S = decltype(b3Pos::x);
    return b3Pos{ static_cast<S>(x), static_cast<S>(y), static_cast<S>(z) };
}

inline b3Pos toB3Pos(const urdf::Vec3& v)
{
    return toB3Pos(v.x, v.y, v.z);
}

/// URDF fixed-axis roll-pitch-yaw -> quaternion, composed R = Rz(yaw) * Ry(pitch) * Rx(roll).
inline b3Quat quatFromRpy(const urdf::Vec3& rpy)
{
    const double hr = rpy.x * 0.5, hp = rpy.y * 0.5, hy = rpy.z * 0.5;
    const double sr = std::sin(hr), cr = std::cos(hr);
    const double sp = std::sin(hp), cp = std::cos(hp);
    const double sy = std::sin(hy), cy = std::cos(hy);

    b3Quat q;
    q.v.x = static_cast<float>(sr * cp * cy - cr * sp * sy);
    q.v.y = static_cast<float>(cr * sp * cy + sr * cp * sy);
    q.v.z = static_cast<float>(cr * cp * sy - sr * sp * cy);
    q.s   = static_cast<float>(cr * cp * cy + sr * sp * sy);
    return q;
}

inline b3Transform transformFromOrigin(const urdf::Origin& o)
{
    b3Transform t;
    t.p = toB3(o.xyz);
    t.q = quatFromRpy(o.rpy);
    return t;
}

inline b3Vec3 rotate(const b3Quat& q, const b3Vec3& v)
{
    // v + 2 * cross(qv, cross(qv, v) + s*v)
    const b3Vec3 u{ q.v.x, q.v.y, q.v.z };
    const b3Vec3 t{ 2.0f * (u.y * v.z - u.z * v.y),
                    2.0f * (u.z * v.x - u.x * v.z),
                    2.0f * (u.x * v.y - u.y * v.x) };
    return b3Vec3{ v.x + q.s * t.x + (u.y * t.z - u.z * t.y),
                   v.y + q.s * t.y + (u.z * t.x - u.x * t.z),
                   v.z + q.s * t.z + (u.x * t.y - u.y * t.x) };
}

inline b3Quat mulQ(const b3Quat& a, const b3Quat& b)
{
    b3Quat r;
    r.v.x = a.s * b.v.x + a.v.x * b.s + a.v.y * b.v.z - a.v.z * b.v.y;
    r.v.y = a.s * b.v.y - a.v.x * b.v.z + a.v.y * b.s + a.v.z * b.v.x;
    r.v.z = a.s * b.v.z + a.v.x * b.v.y - a.v.y * b.v.x + a.v.z * b.s;
    r.s   = a.s * b.s   - a.v.x * b.v.x - a.v.y * b.v.y - a.v.z * b.v.z;
    return r;
}

/// Compose: result applies `a` then `b`'s frame, i.e. b3Mul(a, b) in Box3D's convention.
inline b3Transform mulT(const b3Transform& a, const b3Transform& b)
{
    b3Transform r;
    const b3Vec3 rp = rotate(a.q, b.p);
    r.p = b3Vec3{ a.p.x + rp.x, a.p.y + rp.y, a.p.z + rp.z };
    r.q = mulQ(a.q, b.q);
    return r;
}

inline b3Quat normalize(b3Quat q)
{
    const float n = std::sqrt(q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z + q.s * q.s);
    const float inv = (n > 0.0f) ? 1.0f / n : 1.0f;
    q.v.x *= inv; q.v.y *= inv; q.v.z *= inv; q.s *= inv;
    return q;
}

/// A pose in **world** coordinates.
///
/// ⚠ `b3Transform` cannot be used for this. Its `p` is a `b3Vec3`, which is **float in both
/// precision modes**, whereas `b3Pos` — what `b3Body_GetPosition` returns — becomes **double**
/// under BOX3D_DOUBLE_PRECISION precisely so that coordinates stay accurate far from the origin.
/// AirLib pins that mode on (`cmake/box3d_wrapper`). So composing world poses through
/// `b3Transform` silently truncates them to float and throws away the entire point of large-world
/// mode — no error, no crash, just a robot that loses precision the further it drives.
///
/// `b3Transform` remains correct for *local* frames, where the offsets are small by construction.
struct WorldPose {
    b3Pos p{ 0, 0, 0 };
    b3Quat q = b3Quat_identity;
};

/// Compose a world pose with a local transform: `world ∘ local`.
///
/// The local offset is rotated in float on purpose — it is a link-sized quantity, and the premise
/// of large-world mode is exactly that positions are large while offsets are not. Only the
/// accumulation is done in double, which is where the precision actually has to live.
inline WorldPose mulW(const WorldPose& a, const b3Transform& b)
{
    const b3Vec3 r = rotate(a.q, b.p);
    WorldPose out;
    out.p = toB3Pos(a.p.x + r.x, a.p.y + r.y, a.p.z + r.z);
    out.q = normalize(mulQ(a.q, b.q));
    return out;
}

/// Shortest-arc rotation taking unit vector `from` to unit vector `to`.
/// Handles the antiparallel case, which is the one that silently produces NaNs if ignored.
inline b3Quat quatBetween(const b3Vec3& from, const b3Vec3& to)
{
    const float d = from.x * to.x + from.y * to.y + from.z * to.z;
    if (d >= 1.0f - 1e-6f) return b3Quat_identity;

    if (d <= -1.0f + 1e-6f) {
        // Antiparallel: any axis perpendicular to `from` works. Pick the most stable one.
        b3Vec3 axis = (std::fabs(from.x) < 0.9f) ? b3Vec3{ 1, 0, 0 } : b3Vec3{ 0, 1, 0 };
        b3Vec3 perp{ from.y * axis.z - from.z * axis.y,
                     from.z * axis.x - from.x * axis.z,
                     from.x * axis.y - from.y * axis.x };
        const float n = std::sqrt(perp.x * perp.x + perp.y * perp.y + perp.z * perp.z);
        perp = b3Vec3{ perp.x / n, perp.y / n, perp.z / n };
        return b3Quat{ perp, 0.0f };  // 180 degrees
    }

    const b3Vec3 c{ from.y * to.z - from.z * to.y,
                    from.z * to.x - from.x * to.z,
                    from.x * to.y - from.y * to.x };
    return normalize(b3Quat{ c, 1.0f + d });
}

inline b3Vec3 normalized(const urdf::Vec3& v)
{
    const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    const double inv = (n > 0) ? 1.0 / n : 0.0;
    return b3Vec3{ static_cast<float>(v.x * inv), static_cast<float>(v.y * inv),
                   static_cast<float>(v.z * inv) };
}

/// ⚠⚠ The single most error-prone mapping in this backend, and it is worse than it looks.
///
/// URDF gives a joint an arbitrary `<axis>` in the joint (== child) frame. Box3D instead bakes the
/// free axis into the joint's local frames — and **uses a different axis per joint type**:
///
///   | Box3D joint | free axis | source |
///   |-------------|-----------|--------|
///   | revolute    | **+Z** of the local frame | docs/simulation.md: "rotation about a single axis — the z-axis of the local frame" |
///   | prismatic   | **+X** of local frame A   | docs/simulation.md: "relative translation of two bodies along the x-axis of local frame A" |
///
/// Getting this wrong does not crash and does not look wrong: a prismatic joint built against +Z
/// simply never translates — it behaves as a weld with ~70 um of soft-constraint sag. That is
/// exactly what the first version of this code did, and only tests/test_joint_frames.cpp caught it.
///
/// ⚠⚠ THE DIRECTION OF THIS ROTATION WAS WRONG UNTIL 2026-08-18, and the way it was wrong is the
/// lesson. It was `quatBetween(urdf_axis, box3d_free_axis)` — the rotation that maps the URDF axis
/// ONTO the free axis. But what the joint frame needs is the opposite: the frame's rotation `q`
/// takes frame-local coordinates into body coordinates, so the hinge axis expressed in the body is
/// `q * free_axis`, and that is what must equal the URDF axis. Hence:
///
///     R_axis = quatBetween(box3d_free_axis, urdf_axis)     <- Z to axis, NOT axis to Z
///
/// The old direction is a REFLECTION of the axis through the free axis, so what it produced
/// depended on the joint:
///
///     axis parallel to the free axis  -> correct, by luck (this is why Z-hinges always worked)
///     axis perpendicular to it        -> EXACTLY INVERTED (every wheel, most steering joints)
///     axis oblique                    -> a genuinely DIFFERENT AXIS, e.g.
///                                        (0.30, -0.50, 0.81) became (-0.30, 0.50, 0.81)
///
/// Nothing crashed. Wheels span in the wrong direction, which is invisible on a symmetric wheel and
/// merely looked like the drive multipliers needing a sign; steering went the wrong way on ExoMy
/// and on the Scout, which was written off twice as a settings sign. What actually exposed it was
/// UrdfRobotBackend::getJointState CONTRADICTING ITSELF: `position` comes from Box3D
/// (b3RevoluteJoint_GetAngle, so it followed the flipped frame) while `velocity` is derived by
/// projecting onto the URDF axis. Commanding +3 rad/s gave velocity -2.97 and an angle advancing at
/// +3.00. One struct, two conventions, and only their disagreement made the bug visible.
///
/// ⚠ The unit tests DID NOT CATCH IT, and could not have: they asserted
/// `rotate(axisToZ(a), a) == Z` — that the function does what its name says — and the joint tests
/// checked only that the arm "swung in the plane it should", i.e. the axis LINE. An exactly
/// inverted axis passes every one of those. A test that encodes the same misunderstanding as the
/// code is worse than no test, because it is cited as evidence. They now assert DIRECTION.
///
/// Remembering that a URDF joint <origin> is the transform from the parent link frame to the joint
/// frame while the child link frame *coincides* with the joint frame at zero position:
///
///     localFrameA (on parent) = T_origin * R_axis
///     localFrameB (on child)  =            R_axis
/// Rotation of a REVOLUTE joint's local frame, such that the frame's +Z — Box3D's hinge axis —
/// coincides with the URDF <axis>. Named for what it returns, not for a mapping: the old name
/// `axisToZ` described the rotation that was actually wanted nowhere and was implemented literally.
inline b3Quat revoluteAxisFrame(const urdf::Vec3& axis)
{
    return quatBetween(b3Vec3_axisZ, normalized(axis));
}

/// Rotation of a PRISMATIC joint's local frame: frame A's +X is Box3D's slide axis, so +X must
/// coincide with the URDF <axis>. Separate from the revolute case because Box3D uses a different
/// free axis per joint type — see the note above.
inline b3Quat prismaticAxisFrame(const urdf::Vec3& axis)
{
    return quatBetween(b3Vec3_axisX, normalized(axis));
}

/// Rotate a symmetric 3x3 inertia tensor into another frame: I' = R * I * R^T.
/// URDF expresses <inertia> in the frame given by <inertial><origin>, which need not be the link
/// frame; Box3D wants it in the body frame.
void rotateInertia(const double in[9], const b3Quat& q, double out[9]);

} // namespace b3urdf
