// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef AIRLIB_HEADER_ONLY
#ifndef AIRLIB_NO_RPC

#include "api/WorldRpcLibServer.hpp"

#include "common/Common.hpp"
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER
#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/server.h"
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "api/RpcLibAdaptorsBase.hpp"
#include <map>

STRICT_MODE_ON

namespace msr
{
namespace airlib
{

typedef msr::airlib_rpclib::RpcLibAdaptorsBase RpcLibAdaptorsBase;

WorldRpcLibServer::WorldRpcLibServer(ApiProvider* api_provider, const std::string& server_address, uint16_t port)
    : RpcLibServerBase(api_provider, server_address, port)
{
    (static_cast<rpc::server*>(getServer()))->bind(
        "simGetImagesAllVehicles",
        [&](const std::map<std::string, std::vector<RpcLibAdaptorsBase::ImageRequest>>& req_adapter)
            -> std::map<std::string, std::vector<RpcLibAdaptorsBase::ImageResponse>>
        {
            // Convert adapter input to airlib types
            std::map<std::string, std::vector<msr::airlib::ImageCaptureBase::ImageRequest>> vehicle_requests;
            for (const auto& kv : req_adapter)
                vehicle_requests[kv.first] = RpcLibAdaptorsBase::ImageRequest::to(kv.second);

            const auto& responses = getWorldSimApi()->getImagesAllVehicles(vehicle_requests);

            // Convert output to adapter types
            std::map<std::string, std::vector<RpcLibAdaptorsBase::ImageResponse>> resp_adapter;
            for (const auto& kv : responses)
                resp_adapter[kv.first] = RpcLibAdaptorsBase::ImageResponse::from(kv.second);
            return resp_adapter;
        });
}

WorldRpcLibServer::~WorldRpcLibServer()
{
}

} // namespace airlib
} // namespace msr

#endif // AIRLIB_NO_RPC
#endif // AIRLIB_HEADER_ONLY
