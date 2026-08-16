// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include "UnrealSensorFactory.h"
#include "UnrealSensors/UnrealDistanceSensor.h"
#include "UnrealSensors/UnrealLidarSensor.h"
#include "UnrealSensors/UnrealGPULidarSensor.h"
#include "UnrealSensors/UnrealEchoSensor.h"
#include "UnrealSensors/UnrealSensorTemplate.h"
#include "UnrealSensors/UnrealMarLocUwbSensor.h"
#include "UnrealSensors/UnrealWifiSensor.h"
//#include "Vehicles/AirSimVehicle.h"

UnrealSensorFactory::UnrealSensorFactory(AActor* actor, const NedTransform* ned_transform)
{
    setActor(actor, ned_transform);
}

std::shared_ptr<msr::airlib::SensorBase> UnrealSensorFactory::createSensorFromSettings(
    const AirSimSettings::SensorSetting* sensor_setting) const
{
    using SensorBase = msr::airlib::SensorBase;

    // For every vehicle but a urdfbot this is the pawn, exactly as before.
    AActor* const mount = resolveMount(sensor_setting);

    switch (sensor_setting->sensor_type) {
    case SensorBase::SensorType::Distance:
        return std::shared_ptr<UnrealDistanceSensor>(new UnrealDistanceSensor(
            *static_cast<const AirSimSettings::DistanceSetting*>(sensor_setting), mount, ned_transform_));
    case SensorBase::SensorType::Lidar:
        return std::shared_ptr<UnrealLidarSensor>(new UnrealLidarSensor(
            *static_cast<const AirSimSettings::LidarSetting*>(sensor_setting), mount, ned_transform_));
	case SensorBase::SensorType::GPULidar:
		return std::shared_ptr<UnrealGPULidarSensor>(new UnrealGPULidarSensor(
			*static_cast<const AirSimSettings::GPULidarSetting*>(sensor_setting), mount, ned_transform_));
    case SensorBase::SensorType::Echo:
        return std::shared_ptr<UnrealEchoSensor>(new UnrealEchoSensor(
            *static_cast<const AirSimSettings::EchoSetting*>(sensor_setting), mount, ned_transform_));
    case SensorBase::SensorType::SensorTemplate:
        return std::shared_ptr<UnrealSensorTemplate>(new UnrealSensorTemplate(
            *static_cast<const AirSimSettings::SensorTemplateSetting*>(sensor_setting), mount, ned_transform_));
    case SensorBase::SensorType::MarlocUwb:
        return std::shared_ptr<UnrealMarLocUwbSensor>(new UnrealMarLocUwbSensor(
            *static_cast<const AirSimSettings::MarLocUwbSetting*>(sensor_setting), mount, ned_transform_));
    case SensorBase::SensorType::Wifi:
        return std::shared_ptr<UnrealWifiSensor>(new UnrealWifiSensor(
            *static_cast<const AirSimSettings::WifiSetting*>(sensor_setting), mount, ned_transform_));
    default:
        return msr::airlib::SensorFactory::createSensorFromSettings(sensor_setting);
    }
}

void UnrealSensorFactory::setActor(AActor* actor, const NedTransform* ned_transform)
{
    actor_ = actor;
    ned_transform_ = ned_transform;
}

AActor* UnrealSensorFactory::resolveMount(const AirSimSettings::SensorSetting* sensor_setting) const
{
    if (link_mounts_.empty()) return actor_;

    // The link name is read from the sensor's raw settings json rather than added to SensorSetting,
    // because it is meaningful only for one vehicle type and every sensor subclass would otherwise
    // have to carry a field it can never use.
    const std::string link = sensor_setting->settings.getString("Link", "");
    if (link.empty()) return actor_;

    const auto it = link_mounts_.find(link);
    if (it != link_mounts_.end() && it->second != nullptr) return it->second;

    // ⚠ Fall back to the vehicle root, but record it. Silently mounting a mistyped link on the
    // root gives a sensor that works, reports confidently, and is attached to the wrong body.
    unresolved_links_.push_back(link);
    return actor_;
}
