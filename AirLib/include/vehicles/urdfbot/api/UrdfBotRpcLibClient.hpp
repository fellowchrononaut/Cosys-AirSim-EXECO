// RPC client for the URDF robot API.
//
// Joint-level, mirroring UrdfBotApiBase exactly. Higher-level control — Ackermann, differential
// drive, arm IK — belongs above this, in the caller, where the robot's semantics are known. A URDF
// says nothing about which joints are wheels, so anything that assumes it would be guessing.
#ifndef air_UrdfBotRpcLibClient_hpp
#define air_UrdfBotRpcLibClient_hpp

#include "api/RpcLibClientBase.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "vehicles/urdfbot/api/UrdfBotApiBase.hpp"

#include <functional>
#include <string>
#include <vector>

namespace msr
{
namespace airlib
{

    class UrdfBotRpcLibClient : public RpcLibClientBase
    {
    public:
        /// ⚠ Default port 41454, not 41451. MultiAgent runs one server per vehicle family —
        /// 41451 drone, 41452 car, 41453 computer vision — because each exposes a different
        /// control surface. Connecting to the wrong one fails at the first call rather than at
        /// connect, which is worth knowing when a script mysteriously cannot find a method.
        UrdfBotRpcLibClient(const string& ip_address = "localhost", uint16_t port = 41454,
                            float timeout_sec = 60);

        /// What the robot is made of. Ask before commanding: a joint that a <mimic> resolved
        /// cosmetically has no solver joint behind it, and `JointInfo::mimic_role` is how that is
        /// visible rather than silently ineffective.
        std::vector<UrdfBotApiBase::JointInfo> getJoints(const std::string& vehicle_name = "");
        std::vector<std::string> getLinkNames(const std::string& vehicle_name = "");

        void setJointPosition(const std::string& joint, double radians_or_metres,
                              const std::string& vehicle_name = "");
        void setJointVelocity(const std::string& joint, double per_second,
                              const std::string& vehicle_name = "");
        void setJointEffort(const std::string& joint, double newton_metres,
                            const std::string& vehicle_name = "");
        void setJointPositionGains(const std::string& joint, double hertz, double damping_ratio,
                                   const std::string& vehicle_name = "");

        urdf::JointState getJointState(const std::string& joint,
                                       const std::string& vehicle_name = "");
        /// Pose and twist in AirSim NED, like every other pose crossing this API.
        UrdfBotApiBase::LinkPoseInfo getLinkPose(const std::string& link,
                                                 const std::string& vehicle_name = "");

        virtual ~UrdfBotRpcLibClient(); //required for pimpl
    };
}
} //namespace
#endif
