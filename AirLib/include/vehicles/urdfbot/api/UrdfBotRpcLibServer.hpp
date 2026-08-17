// RPC server for the URDF robot API.
//
// MultiAgent already runs one server per vehicle family — 41451 drone, 41452 car, 41453 computer
// vision — because each exposes a different control surface. The urdfbot adds a fourth on 41454
// rather than extending an existing one: its API is joint-level and shares nothing with steer /
// throttle / brake or roll / pitch / yaw.
#ifndef air_UrdfBotRpcLibServer_hpp
#define air_UrdfBotRpcLibServer_hpp

#ifndef AIRLIB_NO_RPC

#include "api/RpcLibServerBase.hpp"
#include "common/Common.hpp"
#include "vehicles/urdfbot/api/UrdfBotApiBase.hpp"

#include <functional>

namespace msr
{
namespace airlib
{

    class UrdfBotRpcLibServer : public RpcLibServerBase
    {
    public:
        UrdfBotRpcLibServer(ApiProvider* api_provider, string server_address,
                            uint16_t port = RpcLibPort);
        virtual ~UrdfBotRpcLibServer();

    protected:
        virtual UrdfBotApiBase* getVehicleApi(const std::string& vehicle_name) override
        {
            return static_cast<UrdfBotApiBase*>(RpcLibServerBase::getVehicleApi(vehicle_name));
        }
    };

#endif
}
} //namespace
#endif
