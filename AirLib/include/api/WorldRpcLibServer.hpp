// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_WorldRpcLibServer_hpp
#define air_WorldRpcLibServer_hpp

#ifndef AIRLIB_NO_RPC

#include "common/Common.hpp"
#include "api/RpcLibServerBase.hpp"

namespace msr
{
namespace airlib
{

class WorldRpcLibServer : public RpcLibServerBase
{
public:
    WorldRpcLibServer(ApiProvider* api_provider, const std::string& server_address, uint16_t port = 41454);
    virtual ~WorldRpcLibServer();
};

} // namespace airlib
} // namespace msr

#endif // AIRLIB_NO_RPC
#endif // air_WorldRpcLibServer_hpp
