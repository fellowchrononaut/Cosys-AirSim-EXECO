// msgpack wire types for the URDF robot API.
//
// These exist because the API's own structs contain Eigen types and std::string members that
// rpclib cannot serialise directly — the same reason every other vehicle type has an adaptors
// header. Nothing here adds meaning; it is a transport shape for what UrdfBotApiBase already says.
#ifndef air_UrdfBotRpcLibAdaptors_hpp
#define air_UrdfBotRpcLibAdaptors_hpp

#include "api/RpcLibAdaptorsBase.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/urdfbot/api/UrdfBotApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr
{
namespace airlib_rpclib
{

    class UrdfBotRpcLibAdaptors : public RpcLibAdaptorsBase
    {
    public:
        /// One joint as the robot declares it.
        ///
        /// ⚠ `mimic_role` is carried over the wire deliberately. A <mimic> joint that was resolved
        /// cosmetically has **no Box3D joint at all** — commanding it does nothing — and one that
        /// was refused would have stopped the robot loading. A client that cannot see the role
        /// would have no way to tell a joint it can drive from one it cannot, which is exactly the
        /// silent-wrong-answer this workstream exists to avoid.
        struct JointInfo
        {
            std::string name;
            std::string type;
            bool has_limit = false;
            double lower = 0, upper = 0, effort = 0, velocity = 0;
            std::string mimic_role = "none";
            std::string mimic_source;

            MSGPACK_DEFINE_MAP(name, type, has_limit, lower, upper, effort, velocity, mimic_role,
                               mimic_source);

            JointInfo() {}

            JointInfo(const msr::airlib::UrdfBotApiBase::JointInfo& s)
            {
                name = s.name;
                type = s.type;
                has_limit = s.has_limit;
                lower = s.lower;
                upper = s.upper;
                effort = s.effort;
                velocity = s.velocity;
                mimic_role = s.mimic_role;
                mimic_source = s.mimic_source;
            }

            msr::airlib::UrdfBotApiBase::JointInfo to() const
            {
                msr::airlib::UrdfBotApiBase::JointInfo d;
                d.name = name;
                d.type = type;
                d.has_limit = has_limit;
                d.lower = lower;
                d.upper = upper;
                d.effort = effort;
                d.velocity = velocity;
                d.mimic_role = mimic_role;
                d.mimic_source = mimic_source;
                return d;
            }
        };

        /// Measured joint coordinate. `effort` is the motor's applied effort, not the constraint
        /// reaction — the distinction is in UrdfRobotBackend and is preserved here rather than
        /// flattened into a single "force" the caller would have to guess about.
        /// One joint's state WITH its name, so a batch reply is self-describing and a client
        /// never has to assume the reply is index-aligned with a separate getJoints() call.
        struct JointStateInfo
        {
            std::string name;
            double position = 0;
            double velocity = 0;
            double effort = 0;

            MSGPACK_DEFINE_MAP(name, position, velocity, effort);

            JointStateInfo() {}

            JointStateInfo(const msr::airlib::UrdfBotApiBase::JointStateInfo& s)
            {
                name = s.name;
                position = s.position;
                velocity = s.velocity;
                effort = s.effort;
            }

            msr::airlib::UrdfBotApiBase::JointStateInfo to() const
            {
                msr::airlib::UrdfBotApiBase::JointStateInfo d;
                d.name = name;
                d.position = position;
                d.velocity = velocity;
                d.effort = effort;
                return d;
            }
        };

        struct JointState
        {
            double position = 0;
            double velocity = 0;
            double effort = 0;

            MSGPACK_DEFINE_MAP(position, velocity, effort);

            JointState() {}

            JointState(const urdf::JointState& s)
            {
                position = s.position;
                velocity = s.velocity;
                effort = s.effort;
            }

            urdf::JointState to() const
            {
                urdf::JointState d;
                d.position = position;
                d.velocity = velocity;
                d.effort = effort;
                return d;
            }
        };

        /// Linear and angular velocity. RpcLibAdaptorsBase has no Twist, so it is defined here.
        struct Twist
        {
            Vector3r linear;
            Vector3r angular;

            MSGPACK_DEFINE_MAP(linear, angular);

            Twist() {}

            Twist(const msr::airlib::Twist& s)
                : linear(s.linear), angular(s.angular)
            {
            }

            msr::airlib::Twist to() const
            {
                return msr::airlib::Twist(linear.to(), angular.to());
            }
        };

        /// ⚠ Pose and twist are in **AirSim NED**, like every other pose crossing an AirSim API —
        /// not in the URDF's own frame. The conversion happens in UrdfBotSimApi, so a client that
        /// already speaks to drones and cars needs no special case for a URDF robot.
        struct LinkPoseInfo
        {
            std::string name;
            Pose pose;
            Twist twist;

            MSGPACK_DEFINE_MAP(name, pose, twist);

            LinkPoseInfo() {}

            LinkPoseInfo(const msr::airlib::UrdfBotApiBase::LinkPoseInfo& s)
                : name(s.name), pose(s.pose), twist(s.twist)
            {
            }

            msr::airlib::UrdfBotApiBase::LinkPoseInfo to() const
            {
                msr::airlib::UrdfBotApiBase::LinkPoseInfo d;
                d.name = name;
                d.pose = pose.to();
                d.twist = twist.to();
                return d;
            }
        };

        /// Kinematics plus the simulator stamp they were sampled at.
        ///
        /// Identical in shape to the computer-vision adaptor, because the two carry the same
        /// thing: a vehicle whose state is its motion, with no control surface to report.
        struct UrdfBotState
        {
            KinematicsState kinematics_estimated;
            uint64_t timestamp = 0;

            MSGPACK_DEFINE_MAP(kinematics_estimated, timestamp);

            UrdfBotState() {}

            UrdfBotState(const msr::airlib::UrdfBotApiBase::UrdfBotState& s)
                : kinematics_estimated(s.kinematics_estimated), timestamp(s.timestamp)
            {
            }

            msr::airlib::UrdfBotApiBase::UrdfBotState to() const
            {
                return msr::airlib::UrdfBotApiBase::UrdfBotState(kinematics_estimated.to(),
                                                                 timestamp);
            }
        };
    };
}
} //namespace
#endif
