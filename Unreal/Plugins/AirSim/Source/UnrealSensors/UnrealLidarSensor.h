// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include "common/Common.hpp"
#include "GameFramework/Actor.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "sensors/lidar/LidarSimple.hpp"
#include "NedTransform.h"

// UnrealLidarSensor implementation that uses Ray Tracing in Unreal.
// The implementation uses a model similar to CARLA Lidar implementation.
// Thanks to CARLA folks for this.
class UnrealLidarSensor : public msr::airlib::LidarSimple
{
public:
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    UnrealLidarSensor(const AirSimSettings::LidarSetting& setting,
                      AActor* actor, const NedTransform* ned_transform);

protected:
    virtual bool getPointCloud(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
        msr::airlib::TTimeDelta delta_time, msr::airlib::vector<msr::airlib::real_T>& point_cloud, msr::airlib::vector<std::string>& groundtruth, msr::airlib::vector<msr::airlib::real_T>& point_cloud_final, msr::airlib::vector<std::string>& groundtruth_final) override;

	virtual void pause(const bool is_paused);

    virtual void updatePose(const msr::airlib::Pose& sensor_pose, const msr::airlib::Pose& vehicle_pose);

    virtual void getLocalPose(msr::airlib::Pose& sensor_pose);

private:
    using Vector3r = msr::airlib::Vector3r;
    using VectorMath = msr::airlib::VectorMath;

    void createLasers();
    bool shootLaser(const msr::airlib::Pose& lidar_pose, const msr::airlib::Pose& vehicle_pose,
        const uint32 channel, const float horizontal_angle, const float vertical_angle, 
        const msr::airlib::LidarSimpleParams params, Vector3r &point, std::string &label, FVector& raw_point);
    FVector Vector3rToFVector(const Vector3r& input_vector);

private:
    AActor* actor_;
    const NedTransform* ned_transform_;
	float saved_clockspeed_ = 1;
    msr::airlib::vector<msr::airlib::real_T> laser_angles_;
    msr::airlib::vector<FVector> point_cloud_draw_;
	uint32 current_horizontal_angle_index_ = 0;
	// airsim.LogLidarSweep diagnostic. A sweep is accumulated across several ticks, each tick's
	// points expressed in the vehicle frame AT THAT TICK (shootLaser ends with
	// transformToBodyFrame(..., lidar_pose + vehicle_pose)), then handed out as one cloud with one
	// timestamp and one pose. If the vehicle moves during the sweep, earlier points carry a stale
	// frame - they appear dragged along with the robot until the sweep completes and resets.
	// These measure the span so that distortion can be quantified.
	uint64 sweep_start_time_ = 0;   // sim ns, set on the first tick after a hand-off
	int32 sweep_tick_count_ = 0;    // getPointCloud calls contributing to the current sweep
	TArray<float> horizontal_angles_;
	std::mt19937 gen_;
	std::normal_distribution<float> dist_;
    const msr::airlib::LidarSimpleParams sensor_params_;
    msr::airlib::Pose sensor_reference_frame_;
    const float draw_time_;
    const bool external_;
};