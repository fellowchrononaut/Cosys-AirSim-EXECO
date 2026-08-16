// URDF data model. Deliberately free of Box3D, Unreal and AirLib types so the parser can be
// tested on its own and can later move into AirLib unchanged.
//
// Conventions are URDF's, unaltered:
//   - right-handed, Z-up, metres, radians, kilograms
//   - <origin> is (xyz, rpy) with rpy applied as fixed-axis roll(X), pitch(Y), yaw(Z),
//     composed R = Rz(yaw) * Ry(pitch) * Rx(roll)
//   - a joint's <origin> is the transform from the PARENT link frame to the joint frame;
//     the child link frame coincides with the joint frame at zero joint position
//   - a joint's <axis> is expressed in the JOINT (== child) frame
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace urdf {

struct Vec3 {
    double x = 0, y = 0, z = 0;
};

/// Pose as URDF writes it: translation plus fixed-axis roll/pitch/yaw.
struct Origin {
    Vec3 xyz;
    Vec3 rpy;
};

enum class JointType { Revolute, Continuous, Prismatic, Fixed, Floating, Planar };

enum class GeometryType { Box, Cylinder, Sphere, Mesh };

struct Geometry {
    GeometryType type = GeometryType::Box;

    Vec3 box_size;                  // Box: full extents (not half)
    double radius = 0;              // Cylinder, Sphere
    double length = 0;              // Cylinder: full length along +Z, centred on the origin
    std::string mesh_filename;      // Mesh
    Vec3 mesh_scale{ 1, 1, 1 };
};

struct Collision {
    std::string name;
    Origin origin;
    Geometry geometry;
};

/// URDF <visual>. Carried because Gate 2 renders each link as an Unreal actor: the visual geometry
/// is what gets drawn, and — since an Unreal static mesh brings its own collision — it is also what
/// LiDAR, echo and distance sensors trace. That makes <visual> the *other* half of R2: Box3D drives
/// on <collision> hulls while the sensors see this. The two are compared by UrdfCollisionAudit.
struct Visual {
    std::string name;
    Origin origin;
    Geometry geometry;
};

/// URDF <inertial>. `inertia` is about the centre of mass, expressed in the frame given by
/// `origin` — so a non-zero origin.rpy means it must be rotated into the link frame before use.
/// Row-major 3x3; URDF supplies the six independent terms.
struct Inertial {
    Origin origin;
    double mass = 0;
    std::array<double, 9> inertia{ { 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

    static Inertial fromUrdfTerms(double ixx, double ixy, double ixz,
                                  double iyy, double iyz, double izz)
    {
        Inertial in;
        in.inertia = { { ixx, ixy, ixz,
                         ixy, iyy, iyz,
                         ixz, iyz, izz } };
        return in;
    }
};

struct Link {
    std::string name;
    bool has_inertial = false;
    Inertial inertial;
    std::vector<Collision> collisions;   // URDF permits several; UrdfSim supported only one
    std::vector<Visual> visuals;         // likewise; drawn and (in Unreal) traced by sensors

    // Filled in by the parser once the tree is resolved.
    int parent_joint = -1;               // index into Robot::joints, -1 for the root
    std::vector<int> child_joints;
};

struct JointLimit {
    bool present = false;
    double lower = 0;
    double upper = 0;
    double effort = 0;      // N.m for revolute, N for prismatic
    double velocity = 0;    // rad/s or m/s
};

struct JointDynamics {
    double damping = 0;
    double friction = 0;
};

struct Joint {
    std::string name;
    JointType type = JointType::Fixed;
    std::string parent_link;
    std::string child_link;
    Origin origin;
    Vec3 axis{ 1, 0, 0 };   // URDF default when <axis> is omitted
    JointLimit limit;
    JointDynamics dynamics;

    /// URDF <mimic>: q_this = multiplier * q_source + offset. Present iff `mimic_source_joint` is
    /// non-empty. Recorded in full rather than dropped — the failure mode being avoided is
    /// UrdfSim's, where <mimic> was parsed and silently never applied, with nothing to indicate it.
    ///
    /// How the constraint is honoured is a *backend* decision, not a parser one, and it depends on
    /// whether the joint carries load (analysis doc §6.5). See MimicRole and UrdfMimic.hpp.
    std::string mimic_source_joint;
    double mimic_multiplier = 1.0;   // URDF default
    double mimic_offset = 0.0;       // URDF default
    bool hasMimic() const { return !mimic_source_joint.empty(); }

    int parent_index = -1;  // into Robot::links
    int child_index = -1;
};

struct Robot {
    std::string name;
    std::vector<Link> links;
    std::vector<Joint> joints;
    int root_link = -1;

    int findLink(const std::string& n) const
    {
        for (size_t i = 0; i < links.size(); ++i)
            if (links[i].name == n) return static_cast<int>(i);
        return -1;
    }
    int findJoint(const std::string& n) const
    {
        for (size_t i = 0; i < joints.size(); ++i)
            if (joints[i].name == n) return static_cast<int>(i);
        return -1;
    }

    /// Longest root-to-leaf path measured in joints. This is the number that predicts how badly a
    /// maximal-coordinate solver will struggle — not the joint count. Four wheels on a base is
    /// depth 1; a 6-DoF serial arm is depth 6.
    int chainDepth() const;

    /// Link indices of `link` and everything below it, in a deterministic (depth-first) order.
    std::vector<int> subtreeLinks(int link) const;

    /// Sum of <inertial><mass> over the subtree rooted at `link`. Links without <inertial>
    /// contribute nothing, which matches how the backend treats them.
    double subtreeMass(int link) const;

    /// True if any link in the subtree rooted at `link` declares a <collision>. This is the test
    /// that decides whether a <mimic> joint can possibly carry load: a subtree that touches nothing
    /// cannot transmit contact force.
    bool subtreeHasCollision(int link) const;

    double totalMass() const;
};

} // namespace urdf
