// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "SimModeWorldMultiAgent.h"
//#include "SimModeWorldMultiRotor.h"
#include "UObject/ConstructorHelpers.h"
#include "Logging/MessageLog.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"

#include "AirBlueprintLib.h"
#include "Vehicles/Car/CarPawnSimApi.h"
#include "Vehicles/SkidSteer/SkidVehiclePawnSimApi.h"
#include "Vehicles/ComputerVision/ComputerVisionPawnSimApi.h"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "MultirotorPawnSimApi.h"
#include "physics/PhysicsBody.hpp"
#include "common/ClockFactory.hpp"
#include <memory>
#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#include "vehicles/car/api/CarRpcLibServer.hpp"
#include "vehicles/computervision/api/ComputerVisionRpcLibServer.hpp"
#include "common/SteppableClock.hpp"

void ASimModeWorldMultiAgent::BeginPlay()
{
    Super::BeginPlay();

    //let base class setup physics world
    initializeForPlay();
}

void ASimModeWorldMultiAgent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //stop physics thread before we dismantle
    stopAsyncUpdator();

    Super::EndPlay(EndPlayReason);
}

void ASimModeWorldMultiAgent::setupClockSpeed()
{
    typedef msr::airlib::ClockFactory ClockFactory;

    float clock_speed = getSettings().clock_speed;

    //setup clock in ClockFactory
    std::string clock_type = getSettings().clock_type;

    if (clock_type == "ScalableClock") {
        //scalable clock returns interval same as wall clock but multiplied by a scale factor
        ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
    }
    else if (clock_type == "SteppableClock") {
        //steppable clock returns interval that is a constant number irrespective of wall clock
        //we can either multiply this fixed interval by scale factor to speed up/down the clock
        //but that would cause vehicles like quadrotors to become unstable
        //so alternative we use here is instead to scale control loop frequency. The downside is that
        //depending on compute power available, we will max out control loop frequency and therefore can no longer
        //get increase in clock speed

        //Approach 1: scale clock period, no longer used now due to quadrotor instability
        //ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
        //static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));

        //Approach 2: scale control loop frequency if clock is speeded up
        if (clock_speed >= 1) {
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9))); //no clock_speed multiplier

            setPhysicsLoopPeriod(getPhysicsLoopPeriod() / static_cast<long long>(clock_speed));
        }
        else {
            //for slowing down, this don't generate instability
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));
        }
    }
    else
        throw std::invalid_argument(common_utils::Utils::stringf(
            "clock_type %s is not recognized", clock_type.c_str()));
}

//-------------------------------- overrides -----------------------------------------------//

// std::unique_ptr<msr::airlib::ApiServerBase> ASimModeWorldMultiRotor::createApiServer() const
// {
// #ifdef AIRLIB_NO_RPC
//     return ASimModeBase::createApiServer();
// #else
//     return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::MultirotorRpcLibServer(
//         getApiProvider(), getSettings().api_server_address, getSettings().api_port));
// #endif
// }

std::vector<std::unique_ptr<msr::airlib::ApiServerBase>> ASimModeWorldMultiAgent::createApiServer() const
{
    std::vector<std::unique_ptr<msr::airlib::ApiServerBase>> api_servers;
#ifdef AIRLIB_NO_RPC
    api_servers.push_back(ASimModeBase::createApiServer());
    return api_servers;
#else
    uint16_t port_drone = 41451;

    api_servers.push_back(std::make_unique<msr::airlib::MultirotorRpcLibServer>(
        getApiProvider(), getSettings().api_server_address, port_drone));
    
    uint16_t port_ground = 41452;

    api_servers.push_back(std::make_unique<msr::airlib::CarRpcLibServer>(
        getApiProvider(), getSettings().api_server_address, port_ground));

    uint16_t port_cv = 41453;

    api_servers.push_back(std::make_unique<msr::airlib::ComputerVisionRpcLibServer>(
        getApiProvider(), getSettings().api_server_address, port_cv));
    
    return api_servers;
#endif
}

void ASimModeWorldMultiAgent::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
    TArray<AActor*> FlyingPawns;
    UAirBlueprintLib::FindAllActor<TFlyingPawn>(this, FlyingPawns);
    for (AActor* fpawn : FlyingPawns) {
        pawns.Add(fpawn);
        if (getSettings().simmode_name == "MultiAgent") {
            APawn* vehicle_pawn = static_cast<APawn*>(fpawn);
            addPawnToMap(vehicle_pawn, AirSimSettings::kVehicleTypeSimpleFlight);
        }
    }

    TArray<AActor*> CarPawns;
    UAirBlueprintLib::FindAllActor<TCarPawn>(this, CarPawns);
    for (AActor* cpawn : CarPawns) {
        pawns.Add(cpawn);
        if (getSettings().simmode_name == "MultiAgent") {
            APawn* vehicle_pawn = static_cast<APawn*>(cpawn);
            addPawnToMap(vehicle_pawn, AirSimSettings::kVehicleTypePhysXCar);
        }
    }

    TArray<AActor*> SkidPawns;
    UAirBlueprintLib::FindAllActor<TSkidPawn>(this, SkidPawns);
    for (AActor* spawn : SkidPawns) {
        pawns.Add(spawn);
        if (getSettings().simmode_name == "MultiAgent") {
            APawn* vehicle_pawn = static_cast<APawn*>(spawn);
            //addPawnToMap(vehicle_pawn, AirSimSettings::kVehicleTypeSkidSteer);
            if (vehicle_pawn->ActorHasTag(FName("Pioneer"))) {
                addPawnToMap(vehicle_pawn, AirSimSettings::kVehicleTypePioneer);
            }
            else {
                addPawnToMap(vehicle_pawn, AirSimSettings::kVehicleTypeCPHusky);
            }
        }
    }

    TArray<AActor*> CVPawns;
    UAirBlueprintLib::FindAllActor<TCVPawn>(this, CVPawns);
    for (AActor* cvpawn : CVPawns) {
        pawns.Add(cvpawn);
        if (getSettings().simmode_name == "MultiAgent") {
            APawn* vehicle_pawn = static_cast<APawn*>(cvpawn);
            addPawnToMap(vehicle_pawn, AirSimSettings::kVehicleTypeComputerVision);
        }
    }
}

bool ASimModeWorldMultiAgent::isVehicleTypeSupported(const std::string& vehicle_type) const
{
    return ((vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight) ||
            (vehicle_type == AirSimSettings::kVehicleTypePhysXCar) ||
            (vehicle_type == AirSimSettings::kVehicleTypeBoxCar) ||
            (vehicle_type == AirSimSettings::kVehicleTypePioneer) ||
            (vehicle_type == AirSimSettings::kVehicleTypeCPHusky) ||
            (vehicle_type == AirSimSettings::kVehicleTypeComputerVision) ||
            (vehicle_type == AirSimSettings::kVehicleTypePX4) ||
            (vehicle_type == AirSimSettings::kVehicleTypeArduRover) ||
            (vehicle_type == AirSimSettings::kVehicleTypeArduCopterSolo) ||
            (vehicle_type == AirSimSettings::kVehicleTypeArduCopter));
}

std::string ASimModeWorldMultiAgent::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
    //decide which derived BP to use
    std::string pawn_path = vehicle_setting.pawn_path;
    if (pawn_path == ""){
        if (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypePhysXCar) {
            pawn_path = "DefaultCar";
        }
        else if (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeBoxCar) {
            pawn_path = "BoxCar";
        }
        else if (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypePioneer) {
            pawn_path = "Pioneer";
        }
        else if (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeCPHusky) {
            pawn_path = "DefaultSkidVehicle";
        }
        else if (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeComputerVision) {
            pawn_path = "DefaultComputerVision";
        }
        else if (vehicle_setting.vehicle_type == AirSimSettings::kVehicleTypeArduRover) {
            pawn_path = "BoxCar";
        }

        else {
            pawn_path = "DefaultQuadrotor";
        }
    }

    return pawn_path;
}

PawnEvents* ASimModeWorldMultiAgent::getVehiclePawnEvents(APawn* pawn) const
{
    //return static_cast<TVehiclePawn*>(pawn)->getPawnEvents();
    std::string vehicle_type = getVehicleType(pawn);
    if (vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight ||
        vehicle_type == AirSimSettings::kVehicleTypePX4 ||
        vehicle_type == AirSimSettings::kVehicleTypeArduCopterSolo ||
        vehicle_type == AirSimSettings::kVehicleTypeArduCopter) {
        return static_cast<TFlyingPawn*>(pawn)->getPawnEvents();
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypePhysXCar ||
             vehicle_type == AirSimSettings::kVehicleTypeBoxCar ||
             vehicle_type == AirSimSettings::kVehicleTypeArduRover) {
        return static_cast<TCarPawn*>(pawn)->getPawnEvents();
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypePioneer ||
             vehicle_type == AirSimSettings::kVehicleTypeCPHusky) {
        return static_cast<TSkidPawn*>(pawn)->getPawnEvents();
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypeComputerVision) {
        return static_cast<TCVPawn*>(pawn)->getPawnEvents();
    }
    else {
        throw std::invalid_argument(common_utils::Utils::stringf(
            "vehicle_type %s is not recognized in getVehiclePawnEvents()", vehicle_type.c_str()));
    }
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeWorldMultiAgent::getVehiclePawnCameras(
    APawn* pawn) const
{
    //return (static_cast<const TVehiclePawn*>(pawn))->getCameras();
    std::string vehicle_type = getVehicleType(pawn);
    if (vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight ||
        vehicle_type == AirSimSettings::kVehicleTypePX4 ||
        vehicle_type == AirSimSettings::kVehicleTypeArduCopterSolo ||
        vehicle_type == AirSimSettings::kVehicleTypeArduCopter) {
        return (static_cast<const TFlyingPawn*>(pawn))->getCameras();
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypePhysXCar ||
             vehicle_type == AirSimSettings::kVehicleTypeBoxCar ||
             vehicle_type == AirSimSettings::kVehicleTypeArduRover) {
        return (static_cast<const TCarPawn*>(pawn))->getCameras();
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypePioneer ||
             vehicle_type == AirSimSettings::kVehicleTypeCPHusky) {
        return (static_cast<const TSkidPawn*>(pawn))->getCameras();
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypeComputerVision) {
        return (static_cast<const TCVPawn*>(pawn))->getCameras();
    }
    else {
        throw std::invalid_argument(common_utils::Utils::stringf(
            "vehicle_type %s is not recognized in getVehiclePawnCameras()", vehicle_type.c_str()));
    }

}
void ASimModeWorldMultiAgent::initializeVehiclePawn(APawn* pawn)
{
    //static_cast<TVehiclePawn*>(pawn)->initializeForBeginPlay();
    std::string vehicle_type = getVehicleType(pawn);
    if (vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight ||
        vehicle_type == AirSimSettings::kVehicleTypePX4 ||
        vehicle_type == AirSimSettings::kVehicleTypeArduCopterSolo ||
        vehicle_type == AirSimSettings::kVehicleTypeArduCopter) {
        static_cast<TFlyingPawn*>(pawn)->initializeForBeginPlay();
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypePhysXCar ||
             vehicle_type == AirSimSettings::kVehicleTypeBoxCar ||
             vehicle_type == AirSimSettings::kVehicleTypeArduRover) {
        static_cast<TCarPawn*>(pawn)->initializeForBeginPlay(getSettings().engine_sound);
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypePioneer ||
             vehicle_type == AirSimSettings::kVehicleTypeCPHusky) {
        static_cast<TSkidPawn*>(pawn)->initializeForBeginPlay(getSettings().engine_sound);
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypeComputerVision) {
        static_cast<TCVPawn*>(pawn)->initializeForBeginPlay();
    }
    else {
        throw std::invalid_argument(common_utils::Utils::stringf(
            "vehicle_type %s is not recognized in initializeVehiclePawn()", vehicle_type.c_str()));
    }
}
std::unique_ptr<PawnSimApi> ASimModeWorldMultiAgent::createVehicleSimApi(
    const PawnSimApi::Params& pawn_sim_api_params) const
{
    // auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new MultirotorPawnSimApi(pawn_sim_api_params));
    // vehicle_sim_api->initialize();
    // //For multirotors the vehicle_sim_api are in PhysicsWOrld container and then get reseted when world gets reseted
    // //vehicle_sim_api->reset();
    // return vehicle_sim_api;

    APawn* pawn = pawn_sim_api_params.pawn;
    std::string vehicle_type = getVehicleType(pawn);

    if (vehicle_type == AirSimSettings::kVehicleTypePhysXCar ||
             vehicle_type == AirSimSettings::kVehicleTypeBoxCar ||
             vehicle_type == AirSimSettings::kVehicleTypeArduRover) {
        auto vehicle_pawn = static_cast<TCarPawn*>(pawn_sim_api_params.pawn);
        auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new CarPawnSimApi(pawn_sim_api_params, vehicle_pawn->getKeyBoardControls()));
        vehicle_sim_api->initialize();
        return vehicle_sim_api;
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypePioneer ||
             vehicle_type == AirSimSettings::kVehicleTypeCPHusky) {
        auto vehicle_pawn = static_cast<TSkidPawn*>(pawn_sim_api_params.pawn);
        auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new SkidVehiclePawnSimApi(pawn_sim_api_params, vehicle_pawn->getKeyBoardControls()));
        vehicle_sim_api->initialize();
        return vehicle_sim_api;
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypeComputerVision) {
        auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new ComputerVisionPawnSimApi(pawn_sim_api_params));
        vehicle_sim_api->initialize();
        return vehicle_sim_api;
    }
    else {
        auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new MultirotorPawnSimApi(pawn_sim_api_params));
        vehicle_sim_api->initialize();
        return vehicle_sim_api;
    }
}
msr::airlib::VehicleApiBase* ASimModeWorldMultiAgent::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
                                                                    const PawnSimApi* sim_api) const
{
    // const auto multirotor_sim_api = static_cast<const MultirotorPawnSimApi*>(sim_api);
    // return multirotor_sim_api->getVehicleApi();
    APawn* pawn = pawn_sim_api_params.pawn;
    std::string vehicle_type = getVehicleType(pawn);

    if (vehicle_type == AirSimSettings::kVehicleTypePhysXCar ||
             vehicle_type == AirSimSettings::kVehicleTypeBoxCar ||
             vehicle_type == AirSimSettings::kVehicleTypeArduRover) {
        const auto car_sim_api = static_cast<const CarPawnSimApi*>(sim_api);
        return car_sim_api->getVehicleApi();
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypePioneer ||
             vehicle_type == AirSimSettings::kVehicleTypeCPHusky) {
        const auto skid_sim_api = static_cast<const SkidVehiclePawnSimApi*>(sim_api);
        return skid_sim_api->getVehicleApi();
    }
    else if (vehicle_type == AirSimSettings::kVehicleTypeComputerVision) {
        const auto cv_sim_api = static_cast<const ComputerVisionPawnSimApi*>(sim_api);
        return cv_sim_api->getVehicleApi();
    }
    else {
        const auto multirotor_sim_api = static_cast<const MultirotorPawnSimApi*>(sim_api);
        return multirotor_sim_api->getVehicleApi();
    }
}
