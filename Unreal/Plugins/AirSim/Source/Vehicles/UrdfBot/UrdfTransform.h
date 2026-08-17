// The one place URDF's frame meets Unreal's.
//
// This design has three frames and confusing them produces a robot that looks almost right, which
// is the worst kind of wrong:
//
//   URDF / ROS   right-handed, Z-up, metres.  X forward, Y **left**, Z up   (FLU)
//   Unreal       left-handed,  Z-up, centimetres. X forward, Y **right**, Z up
//   AirSim NED   right-handed, Z-**down**, metres. X north, Y east, Z down
//
// A quiet advantage of the split, noted in analysis doc §6.1: URDF and Box3D share a convention
// exactly — Box3D has no intrinsic up, gravity is just a vector — so the articulation lives
// natively in the solver's frame with no NED round-trip. NED appears only at the AirSim API
// boundary, where NedTransform already handles it. Nothing in this file touches NED.
//
// URDF -> Unreal is therefore a single Y mirror plus a unit scale. Deriving it through NED gives
// the same answer and is worth stating, because it is how these two functions were checked:
//   FLU -> NED is a 180 deg rotation about X (both frames right-handed, so a proper rotation);
//   NED -> Unreal is NedTransform's (x, y, -z) with FQuat(-qx, -qy, qz, qw).
//   Composed: position (x, -y, z), quaternion (-qx, qy, -qz, qw). Which is a Y mirror.
#pragma once

#include "CoreMinimal.h"

#include "urdf/UrdfModel.hpp"
#include "urdf/UrdfRobotBackend.hpp"

namespace UrdfTransform {

/// Unreal units per metre. AirSim's world_to_meters is 100 in every environment in this repo;
/// taken as a parameter anyway so a rescaled map cannot silently halve the robot.
inline FVector toFVector(const urdf::Vec3& v, float world_to_meters)
{
    return FVector(static_cast<float>(v.x) * world_to_meters,
                   static_cast<float>(-v.y) * world_to_meters,
                   static_cast<float>(v.z) * world_to_meters);
}

inline FQuat toFQuat(const urdf::Quat& q)
{
    return FQuat(static_cast<float>(-q.x), static_cast<float>(q.y),
                 static_cast<float>(-q.z), static_cast<float>(q.w));
}

/// URDF fixed-axis rpy -> Unreal rotation. Composed as R = Rz(yaw) Ry(pitch) Rx(roll) in the URDF
/// frame first, then mirrored — rather than mapping roll/pitch/yaw onto FRotator's fields, which
/// looks equivalent and is not, because FRotator is applied in a different order and in a
/// left-handed frame.
inline FQuat toFQuat(const urdf::Vec3& rpy)
{
    const double cr = FMath::Cos(rpy.x * 0.5), sr = FMath::Sin(rpy.x * 0.5);
    const double cp = FMath::Cos(rpy.y * 0.5), sp = FMath::Sin(rpy.y * 0.5);
    const double cy = FMath::Cos(rpy.z * 0.5), sy = FMath::Sin(rpy.z * 0.5);

    urdf::Quat q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return toFQuat(q);
}

/// Unreal -> URDF. Needed since the solver frame became the **world** frame rather than each
/// robot's spawn frame (analysis doc §6.0c): a robot's spawn transform and the level's colliders
/// both start life as Unreal quantities and have to be expressed in URDF terms.
///
/// Both of these are their own inverses, because a Y mirror is an involution. That is worth
/// knowing: `toUrdfVec(toFVector(v, m), m) == v` exactly, which is how the pair is checked.
inline urdf::Vec3 toUrdfVec(const FVector& v, float world_to_meters)
{
    return urdf::Vec3{ static_cast<double>(v.X) / world_to_meters,
                       static_cast<double>(-v.Y) / world_to_meters,
                       static_cast<double>(v.Z) / world_to_meters };
}

inline urdf::Quat toUrdfQuat(const FQuat& q)
{
    return urdf::Quat{ -q.X, q.Y, -q.Z, q.W };
}

inline FTransform toFTransform(const urdf::Origin& o, float world_to_meters)
{
    return FTransform(toFQuat(o.rpy), toFVector(o.xyz, world_to_meters));
}

inline FTransform toFTransform(const urdf::LinkPose& p, float world_to_meters)
{
    return FTransform(toFQuat(p.orientation), toFVector(p.position, world_to_meters));
}

} // namespace UrdfTransform
