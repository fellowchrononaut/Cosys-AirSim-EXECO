// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "CoreMinimal.h"
#include "sensors/SensorFactory.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "NedTransform.h"
#include "GameFramework/Actor.h"

class UnrealSensorFactory : public msr::airlib::SensorFactory
{
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    UnrealSensorFactory(AActor* actor, const NedTransform* ned_transform);
    virtual ~UnrealSensorFactory() {}
    void setActor(AActor* actor, const NedTransform* ned_transform);
    virtual std::shared_ptr<msr::airlib::SensorBase> createSensorFromSettings(
        const AirSimSettings::SensorSetting* sensor_setting) const override;

    /// Per-link mounting for URDF robots: a sensor whose settings carry `"Link": "<name>"` is
    /// built against the actor registered here for that link instead of the vehicle pawn.
    ///
    /// Empty for every other vehicle type, and then this class behaves exactly as before — a
    /// drone or a car has one rigid body, so "the vehicle" and "the mount" are the same actor.
    /// A URDF robot does not: a camera on a pan-tilt head and a LiDAR on the chassis move
    /// differently, and mounting both on the root would misreport both.
    void setLinkMounts(std::map<std::string, AActor*> mounts) { link_mounts_ = std::move(mounts); }

    /// Link names named by sensors but absent from the URDF. Reported by the caller rather than
    /// silently ignored — a typo'd link would otherwise mount the sensor on the robot's root and
    /// produce plausible, wrong data.
    const std::vector<std::string>& getUnresolvedLinks() const { return unresolved_links_; }

private:
    AActor* resolveMount(const AirSimSettings::SensorSetting* sensor_setting) const;

    AActor* actor_;
    const NedTransform* ned_transform_;
    std::map<std::string, AActor*> link_mounts_;
    mutable std::vector<std::string> unresolved_links_;
};
