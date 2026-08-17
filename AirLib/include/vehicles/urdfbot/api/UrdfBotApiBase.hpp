// Control surface for a URDF robot.
//
// Deliberately joint-level and nothing more. A URDF robot has no fixed control abstraction the way
// a multirotor (roll/pitch/yaw/throttle) or a car (steer/throttle/brake) does — it has whatever
// joints its author gave it. Inventing a higher-level scheme here would mean guessing which joints
// are wheels and which are steering, and guessing wrong silently.
//
// Higher-level controllers (Ackermann, differential drive, an arm IK solver) belong above this, in
// the client, where the robot's semantics are known.
#pragma once

#include "api/VehicleApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"
#include "urdf/UrdfRobotBackend.hpp"

#include <memory>
#include <string>
#include <vector>

namespace msr
{
namespace airlib
{

    class UrdfBotApiBase : public VehicleApiBase
    {
    public:
        struct JointInfo
        {
            std::string name;
            std::string type;
            bool has_limit = false;
            double lower = 0, upper = 0, effort = 0, velocity = 0;
            /// "none" | "cosmetic" | "load-bearing" — how a <mimic> on this joint was honoured.
            std::string mimic_role = "none";
            std::string mimic_source;
        };

        struct LinkPoseInfo
        {
            std::string name;
            /// In AirSim NED, like every other pose crossing this API.
            Pose pose;
            Twist twist;
        };

        virtual ~UrdfBotApiBase() = default;

        /// What the robot is made of. Enumerable rather than assumed: a client cannot command a
        /// joint it cannot name, and the mimic role belongs here so the omission stays visible.
        virtual std::vector<JointInfo> getJoints() const = 0;
        virtual std::vector<std::string> getLinkNames() const = 0;

        virtual void setJointPosition(const std::string& joint, double radians_or_metres) = 0;
        virtual void setJointVelocity(const std::string& joint, double per_second) = 0;
        virtual void setJointEffort(const std::string& joint, double newton_metres) = 0;
        virtual void setJointPositionGains(const std::string& joint, double hertz,
                                           double damping_ratio) = 0;

        virtual urdf::JointState getJointState(const std::string& joint) const = 0;
        virtual LinkPoseInfo getLinkPose(const std::string& link) const = 0;

        // VehicleApiBase — a URDF robot has no arming or flight-control concept, so these are
        // answered honestly rather than emulated.
        virtual void enableApiControl(bool is_enabled) override { api_control_enabled_ = is_enabled; }
        virtual bool isApiControlEnabled() const override { return api_control_enabled_; }
        virtual bool armDisarm(bool arm) override
        {
            unused(arm);
            return false;  // nothing to arm; saying "yes" would imply a state that does not exist
        }

        virtual const SensorCollection& getSensors() const override { return sensors_; }
        SensorCollection& getSensors() { return sensors_; }

        /// ⚠ Ticking the sensors is NOT automatic, and forgetting it is silent.
        ///
        /// Sensors are created, mounted on their links, and answer RPC calls whether or not
        /// anything ever updates them — they simply return their initial sample forever. Measured
        /// on a live simulator before this existed: the IMU returned 40 samples across 2 s with
        /// **one distinct timestamp**, and the LiDAR returned **0 points**, while every call
        /// succeeded and every camera worked. Nothing in the API surface distinguishes a frozen
        /// sensor from a stationary robot.
        ///
        /// This mirrors CarApiBase::update exactly; the urdfbot simply never had it.
        virtual void update(float delta = 0) override
        {
            VehicleApiBase::update(delta);
            getSensors().update(delta);
        }

        /// ⚠ Sensors are reset LAST, after their ground truth has been reset — same ordering and
        /// same reason as CarApiBase. A sensor reset before the kinematics it samples would take
        /// its first sample from the previous run's state.
        virtual void resetImplementation() override
        {
            getSensors().reset();
        }

        virtual void reportState(StateReporter& reporter) override
        {
            getSensors().reportState(reporter);
        }

        /// Build the sensors declared in settings. Identical in shape to every other vehicle type;
        /// what differs is that the factory has been given a link map first, so a sensor naming a
        /// link is mounted on that link rather than on the robot's root.
        void initializeSensors(const AirSimSettings::VehicleSetting* vehicle_setting,
                               std::shared_ptr<SensorFactory> sensor_factory,
                               const Kinematics::State& state, const Environment& environment)
        {
            sensor_factory_ = sensor_factory;
            sensors_.clear();
            sensor_storage_.clear();
            sensor_factory_->createSensorsFromSettings(vehicle_setting->sensors, sensors_,
                                                       sensor_storage_);
            sensors_.initialize(&state, &environment);
        }

    protected:
        /// ⚠ FALSE by default, so the keyboard works on a freshly loaded robot and a client must
        /// take control explicitly — the same handshake every other AirSim vehicle uses.
        ///
        /// This is not cosmetic. The keyboard drive loop writes EVERY drive and steer joint on
        /// every physics step, so with both active an RPC command is overwritten 3 ms after it is
        /// issued: the call succeeds, returns, and does nothing. Two controllers on one joint need
        /// an arbiter, and enableApiControl is the one AirSim already provides.
        bool api_control_enabled_ = false;

        std::shared_ptr<SensorFactory> sensor_factory_;
        SensorCollection sensors_;
        vector<shared_ptr<SensorBase>> sensor_storage_;  // RAII for created sensors
    };
}
} //namespace
