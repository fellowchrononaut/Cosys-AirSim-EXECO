#ifndef AIRLIB_HEADER_ONLY
#ifndef AIRLIB_NO_RPC

#include "vehicles/urdfbot/api/UrdfBotRpcLibClient.hpp"

#include "common/Common.hpp"
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/client.h"
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "vehicles/urdfbot/api/UrdfBotRpcLibAdaptors.hpp"

STRICT_MODE_ON

namespace msr
{
namespace airlib
{

    typedef msr::airlib_rpclib::UrdfBotRpcLibAdaptors UrdfBotRpcLibAdaptors;

    UrdfBotRpcLibClient::UrdfBotRpcLibClient(const string& ip_address, uint16_t port,
                                             float timeout_sec)
        : RpcLibClientBase(ip_address, port, timeout_sec)
    {
    }

    UrdfBotRpcLibClient::~UrdfBotRpcLibClient()
    {
    }

    std::vector<UrdfBotApiBase::JointInfo> UrdfBotRpcLibClient::getJoints(const std::string& vehicle_name)
    {
        const auto wire = static_cast<rpc::client*>(getClient())
                              ->call("getJoints", vehicle_name)
                              .as<std::vector<UrdfBotRpcLibAdaptors::JointInfo>>();
        std::vector<UrdfBotApiBase::JointInfo> out;
        out.reserve(wire.size());
        for (const auto& j : wire)
            out.push_back(j.to());
        return out;
    }

    std::vector<std::string> UrdfBotRpcLibClient::getLinkNames(const std::string& vehicle_name)
    {
        return static_cast<rpc::client*>(getClient())
            ->call("getLinkNames", vehicle_name)
            .as<std::vector<std::string>>();
    }

    void UrdfBotRpcLibClient::setJointPosition(const std::string& joint, double value,
                                               const std::string& vehicle_name)
    {
        static_cast<rpc::client*>(getClient())->call("setJointPosition", joint, value, vehicle_name);
    }

    void UrdfBotRpcLibClient::setJointVelocity(const std::string& joint, double value,
                                               const std::string& vehicle_name)
    {
        static_cast<rpc::client*>(getClient())->call("setJointVelocity", joint, value, vehicle_name);
    }

    void UrdfBotRpcLibClient::setDriveCommand(double throttle, double steering,
                                              const std::string& vehicle_name)
    {
        static_cast<rpc::client*>(getClient())->call("setDriveCommand", throttle, steering,
                                                     vehicle_name);
    }

    void UrdfBotRpcLibClient::setJointEffort(const std::string& joint, double value,
                                             const std::string& vehicle_name)
    {
        static_cast<rpc::client*>(getClient())->call("setJointEffort", joint, value, vehicle_name);
    }

    void UrdfBotRpcLibClient::setJointPositionGains(const std::string& joint, double hertz,
                                                    double damping_ratio,
                                                    const std::string& vehicle_name)
    {
        static_cast<rpc::client*>(getClient())
            ->call("setJointPositionGains", joint, hertz, damping_ratio, vehicle_name);
    }

    std::string UrdfBotRpcLibClient::getUrdfXml(const std::string& vehicle_name)
    {
        return static_cast<rpc::client*>(getClient())->call("getUrdfXml", vehicle_name).as<std::string>();
    }

    std::vector<UrdfBotApiBase::JointStateInfo>
    UrdfBotRpcLibClient::getJointStates(const std::string& vehicle_name)
    {
        const auto wire = static_cast<rpc::client*>(getClient())
                              ->call("getJointStates", vehicle_name)
                              .as<std::vector<UrdfBotRpcLibAdaptors::JointStateInfo>>();
        std::vector<UrdfBotApiBase::JointStateInfo> out;
        out.reserve(wire.size());
        for (const auto& w : wire) out.push_back(w.to());
        return out;
    }

    UrdfBotApiBase::UrdfBotState UrdfBotRpcLibClient::getUrdfBotState(const std::string& vehicle_name)
    {
        return static_cast<rpc::client*>(getClient())
            ->call("getUrdfBotState", vehicle_name)
            .as<UrdfBotRpcLibAdaptors::UrdfBotState>()
            .to();
    }

    urdf::JointState UrdfBotRpcLibClient::getJointState(const std::string& joint,
                                                        const std::string& vehicle_name)
    {
        return static_cast<rpc::client*>(getClient())
            ->call("getJointState", joint, vehicle_name)
            .as<UrdfBotRpcLibAdaptors::JointState>()
            .to();
    }

    UrdfBotApiBase::LinkPoseInfo UrdfBotRpcLibClient::getLinkPose(const std::string& link,
                                                                  const std::string& vehicle_name)
    {
        return static_cast<rpc::client*>(getClient())
            ->call("getLinkPose", link, vehicle_name)
            .as<UrdfBotRpcLibAdaptors::LinkPoseInfo>()
            .to();
    }
}
} //namespace

#endif
#endif
