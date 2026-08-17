#ifndef AIRLIB_HEADER_ONLY
#ifndef AIRLIB_NO_RPC

#include "vehicles/urdfbot/api/UrdfBotRpcLibServer.hpp"

#include "common/Common.hpp"
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/server.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
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

    UrdfBotRpcLibServer::UrdfBotRpcLibServer(ApiProvider* api_provider, string server_address,
                                             uint16_t port)
        : RpcLibServerBase(api_provider, server_address, port)
    {
        // --- structure -------------------------------------------------------------------------
        // Enumerable rather than assumed. A URDF robot has no fixed control abstraction — it has
        // whatever joints its author gave it — so a client must be able to ASK what exists rather
        // than hard-code names. This is the call that makes the API usable on an arbitrary robot.
        (static_cast<rpc::server*>(getServer()))->bind("getJoints", [&](const std::string& vehicle_name) -> std::vector<UrdfBotRpcLibAdaptors::JointInfo> {
            const auto joints = getVehicleApi(vehicle_name)->getJoints();
            std::vector<UrdfBotRpcLibAdaptors::JointInfo> out;
            out.reserve(joints.size());
            for (const auto& j : joints)
                out.push_back(UrdfBotRpcLibAdaptors::JointInfo(j));
            return out;
        });

        (static_cast<rpc::server*>(getServer()))->bind("getLinkNames", [&](const std::string& vehicle_name) -> std::vector<std::string> {
            return getVehicleApi(vehicle_name)->getLinkNames();
        });

        // --- command ---------------------------------------------------------------------------
        // Joint-level and nothing more. Ackermann steering, differential drive or an arm's IK
        // belong in the client, where the robot's semantics are known; putting them here would
        // mean guessing which joints are wheels, which is the failure this workstream avoids.
        (static_cast<rpc::server*>(getServer()))->bind("setJointPosition", [&](const std::string& joint, double value, const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->setJointPosition(joint, value);
        });
        (static_cast<rpc::server*>(getServer()))->bind("setJointVelocity", [&](const std::string& joint, double value, const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->setJointVelocity(joint, value);
        });
        (static_cast<rpc::server*>(getServer()))->bind("setJointEffort", [&](const std::string& joint, double value, const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->setJointEffort(joint, value);
        });
        (static_cast<rpc::server*>(getServer()))->bind("setJointPositionGains", [&](const std::string& joint, double hertz, double damping_ratio, const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->setJointPositionGains(joint, hertz, damping_ratio);
        });

        // --- state -----------------------------------------------------------------------------
        (static_cast<rpc::server*>(getServer()))->bind("getJointState", [&](const std::string& joint, const std::string& vehicle_name) -> UrdfBotRpcLibAdaptors::JointState {
            return UrdfBotRpcLibAdaptors::JointState(getVehicleApi(vehicle_name)->getJointState(joint));
        });
        (static_cast<rpc::server*>(getServer()))->bind("getLinkPose", [&](const std::string& link, const std::string& vehicle_name) -> UrdfBotRpcLibAdaptors::LinkPoseInfo {
            return UrdfBotRpcLibAdaptors::LinkPoseInfo(getVehicleApi(vehicle_name)->getLinkPose(link));
        });
    }

    //required for pimpl
    UrdfBotRpcLibServer::~UrdfBotRpcLibServer()
    {
    }
}
} //namespace

#endif
#endif
