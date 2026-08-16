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

inline b3Pos toB3Pos(const urdf::Vec3& v)
{
#if defined(BOX3D_DOUBLE_PRECISION)
    return b3Pos{ v.x, v.y, v.z };
#else
    return b3Pos{ static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z) };
#endif
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
/// With R_axis = quatBetween(urdf_axis, box3d_free_axis), and remembering that a URDF joint
/// <origin> is the transform from the parent link frame to the joint frame while the child link
/// frame *coincides* with the joint frame at zero position:
///
///     localFrameA (on parent) = T_origin * R_axis
///     localFrameB (on child)  =            R_axis
///
/// UrdfSim's equivalent code was never unit-tested. This one is, per joint type.
inline b3Quat axisToZ(const urdf::Vec3& axis)
{
    return quatBetween(normalized(axis), b3Vec3_axisZ);
}

/// Prismatic joints slide along local frame A's +X. See axisToZ for why this is separate.
inline b3Quat axisToX(const urdf::Vec3& axis)
{
    return quatBetween(normalized(axis), b3Vec3_axisX);
}

/// Rotate a symmetric 3x3 inertia tensor into another frame: I' = R * I * R^T.
/// URDF expresses <inertia> in the frame given by <inertial><origin>, which need not be the link
/// frame; Box3D wants it in the body frame.
void rotateInertia(const double in[9], const b3Quat& q, double out[9]);

} // namespace b3urdf
