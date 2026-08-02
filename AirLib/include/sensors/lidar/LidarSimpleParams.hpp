// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_LidarSimpleParams_hpp
#define msr_airlib_LidarSimpleParams_hpp

#include "common/Common.hpp"
#include "common/AirSimSettings.hpp"

namespace msr
{
namespace airlib
{

    struct LidarSimpleParams
    {

        // Loosely a Velodyne VLP-16 Puck (https://velodynelidar.com/vlp-16.html): the channel count,
        // rotation rate and range below do match one. The azimuth resolution does NOT - see
        // measurement_per_cycle. Do not cite these defaults as VLP-16 equivalent.

        // default settings
        // TODO: enable reading of these params from AirSim settings

        uint number_of_channels = 16;
        real_T range = 10000.0f / 100; // meters
        bool generate_noise = false;			  // Toggle range based noise
        real_T min_noise_standard_deviation = 0;  // Minimum noise standard deviation
        real_T noise_distance_scale = 1;		  // Factor to scale noise based on distance

        bool limit_points = true;			      // how frequently to update the data in Hz
        bool pause_after_measurement = false;	  // Pause the simulation after each measurement. Useful for API interaction to be synced
                                                  // If true, the time passed in-engine will be used (when performance doesn't allow real-time operation)

        bool external = false;                    // define if a sensor is attached to the vehicle itself(false), or to the world and is an external sensor (true)
        bool external_ned = true;                 // define if the external sensor coordinates should be reported back by the API in local NED or Unreal coordinates
        bool draw_sensor = false;

        // Azimuth columns per revolution. 512 is a power of two chosen for convenience, NOT a sensor
        // figure: it arrived in upstream commit 830dfbab (2020-03-24), which replaced
        // `points_per_second = 100000` with this and orphaned the PointsPerSecond setting in the
        // process. At the defaults here (16 channels, 10 rev/s) the old value implied 625 columns,
        // so the "rename" also dropped 18% of the sampling density. A real VLP-16 does 300000
        // points/s = 1875 columns = 0.192 deg; 512 gives 0.703 deg, ~3.7x coarser.
        uint measurement_per_cycle = 512;
        uint horizontal_rotation_frequency = 10; // rotations/sec
        real_T horizontal_FOV_start = 0;
        real_T horizontal_FOV_end = 360;
        real_T vertical_FOV_upper = -15; // drones -15, car +10
        real_T vertical_FOV_lower = -45; // drones -45, car -10



        Pose relative_pose{
            Vector3r(0, 0, -1), // position - a little above vehicle (especially for cars) or Vector3r::Zero()
            Quaternionr::Identity() // orientation - by default Quaternionr(1, 0, 0, 0)
        };

        bool draw_debug_points = false;

        bool external_controller = true;

        real_T update_frequency = 10; // Hz
        real_T startup_delay = 1; // sec

        void initializeFromSettings(const AirSimSettings::LidarSetting& settings)
        {
            std::string simmode_name = AirSimSettings::singleton().simmode_name;

            const auto& settings_json = settings.settings;
            number_of_channels = settings_json.getInt("NumberOfChannels", number_of_channels);
            range = settings_json.getFloat("Range", range);
            // Sentinel 0 rather than the default, so that "not specified at all" is distinguishable
            // from an explicit value. PointsPerSecond below may fill it in.
            const int measurements_setting = settings_json.getInt("MeasurementsPerCycle", 0);
            horizontal_rotation_frequency = settings_json.getInt("RotationsPerSecond", horizontal_rotation_frequency);

            // PointsPerSecond was previously read nowhere at all. A settings file could state a
            // sampling rate that the sensor never honoured, and because the JSON parser does not
            // warn about unrecognised keys it failed completely silently. That matters beyond
            // tidiness: settings.json is archived alongside a dataset as its provenance record, so
            // it was asserting a sensor property that described nothing. A stated-but-wrong number
            // is worse than an absent one, because a reader has no reason to go and check it.
            //
            // Datasheets quote points/second (a VLP-16 is "300,000 points/sec"), so honour the key
            // by deriving the azimuth resolution that actually drives the sweep:
            //     measurements_per_cycle = points_per_second / (channels * rotations_per_second)
            // An explicit MeasurementsPerCycle always wins, being the more direct statement of the
            // same quantity, and a disagreement between the two is reported rather than silently
            // resolved either way.
            const int points_per_second = settings_json.getInt("PointsPerSecond", 0);
            const uint points_per_cycle_divisor = number_of_channels * horizontal_rotation_frequency;
            uint derived_measurement_per_cycle = 0;
            if (points_per_second > 0 && points_per_cycle_divisor > 0)
                derived_measurement_per_cycle = static_cast<uint>(points_per_second) / points_per_cycle_divisor;

            if (measurements_setting > 0) {
                measurement_per_cycle = static_cast<uint>(measurements_setting);
                if (derived_measurement_per_cycle > 0 && derived_measurement_per_cycle != measurement_per_cycle)
                    Utils::log(Utils::stringf(
                                   "Lidar '%s': MeasurementsPerCycle=%d and PointsPerSecond=%d disagree - "
                                   "PointsPerSecond implies %d measurements/cycle. Using MeasurementsPerCycle; "
                                   "the effective rate is %d points/second.",
                                   settings.sensor_name.c_str(), measurement_per_cycle, points_per_second,
                                   derived_measurement_per_cycle, measurement_per_cycle * points_per_cycle_divisor),
                               Utils::kLogLevelWarn);
            }
            else if (derived_measurement_per_cycle > 0) {
                measurement_per_cycle = derived_measurement_per_cycle;
                Utils::log(Utils::stringf(
                               "Lidar '%s': derived MeasurementsPerCycle=%d from PointsPerSecond=%d "
                               "(%d channels x %d rotations/second).",
                               settings.sensor_name.c_str(), measurement_per_cycle, points_per_second,
                               number_of_channels, horizontal_rotation_frequency),
                           Utils::kLogLevelInfo);
            }
            external_controller = settings_json.getBool("ExternalController", external_controller);
		    update_frequency = settings_json.getFloat("UpdateFrequency", update_frequency);
            vertical_FOV_upper = settings_json.getFloat("VerticalFOVUpper", Utils::nan<float>());
            limit_points = settings_json.getBool("LimitPoints", limit_points);
		    pause_after_measurement = settings_json.getBool("settings.pause_after_measurement", pause_after_measurement);
            draw_debug_points = settings_json.getBool("DrawDebugPoints", draw_debug_points);
            draw_sensor = settings_json.getBool("DrawSensor", draw_sensor);
            external = settings_json.getBool("External", external);
            external_ned = settings_json.getBool("ExternalLocal", external_ned);
            generate_noise = settings_json.getBool("GenerateNoise", generate_noise);
            min_noise_standard_deviation = settings_json.getFloat("MinNoiseStandardDeviation", min_noise_standard_deviation);
            noise_distance_scale = settings_json.getFloat("NoiseDistanceScale", noise_distance_scale);


            // By default, for multirotors the lidars FOV point downwards;
            // for cars, the lidars FOV is more forward facing.
            if (std::isnan(vertical_FOV_upper)) {
                if (simmode_name == AirSimSettings::kSimModeTypeMultirotor)
                    vertical_FOV_upper = -15;
                else
                    vertical_FOV_upper = +10;
            }

            vertical_FOV_lower = settings_json.getFloat("VerticalFOVLower", Utils::nan<float>());
            if (std::isnan(vertical_FOV_lower)) {
                if (simmode_name == AirSimSettings::kSimModeTypeMultirotor)
                    vertical_FOV_lower = -45;
                else
                    vertical_FOV_lower = -10;
            }

            horizontal_FOV_start = settings_json.getFloat("HorizontalFOVStart", horizontal_FOV_start);
            horizontal_FOV_end = settings_json.getFloat("HorizontalFOVEnd", horizontal_FOV_end);

            relative_pose.position = AirSimSettings::createVectorSetting(settings_json, VectorMath::nanVector());
            auto rotation = AirSimSettings::createRotationSetting(settings_json, AirSimSettings::Rotation::nanRotation());

            if (std::isnan(relative_pose.position.x()))
                relative_pose.position.x() = 0;
            if (std::isnan(relative_pose.position.y()))
                relative_pose.position.y() = 0;
            if (std::isnan(relative_pose.position.z())) {
                relative_pose.position.z() = 0;
            }

            float pitch, roll, yaw;
            pitch = !std::isnan(rotation.pitch) ? rotation.pitch : 0;
            roll = !std::isnan(rotation.roll) ? rotation.roll : 0;
            yaw = !std::isnan(rotation.yaw) ? rotation.yaw : 0;
            relative_pose.orientation = VectorMath::toQuaternion(
                Utils::degreesToRadians(pitch), // pitch - rotation around Y axis
                Utils::degreesToRadians(roll), // roll  - rotation around X axis
                Utils::degreesToRadians(yaw)); // yaw   - rotation around Z axis
        }
    };
}
} //namespace
#endif
