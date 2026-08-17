// Control surface for a URDF robot.
//
// Deliberately joint-level and nothing more. A URDF robot has no fixed control abstraction the way
// a multirotor (roll/pitch/yaw/throttle) or a car (steer/throttle/brake) does — it has whatever
// joints its author gave it. Inventing a higher-level scheme here would mean guessing which joints
// are wheels and which are steering, and guessing wrong silently.
//
// Higher-level controllers (Ackermann, differential drive, an arm IK solver) belong above this, in
// the client, where the robot's semantics are known.
//
// ⚠ ONE EXCEPTION: setDriveCommand. It is not a guess about which joints are wheels — the operator
// already stated that in settings, as UrdfDrive's DriveJoints/SteerJoints with their multipliers,
// and UrdfBotSimApi::applyDriveInput already implements the mapping for the keyboard. Exposing the
// same two axes over RPC reuses that one implementation instead of copying it into every client.
// The rule above still holds for anything the URDF does NOT declare.
#pragma once

#include "api/VehicleApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"
#include "urdf/UrdfRobotBackend.hpp"

#include <atomic>
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

        /// One joint's state, carrying its own name so a batch is self-describing.
        struct JointStateInfo
        {
            std::string name;
            double position = 0;  ///< rad or m
            double velocity = 0;  ///< rad/s or m/s
            double effort = 0;    ///< N.m or N
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

        /// Every movable joint's state in ONE call, sampled together.
        ///
        /// ⚠ Not a convenience wrapper over getJointState. Polling N joints individually samples
        /// them at N different instants, so the result is a SMEAR rather than a snapshot — and a
        /// sensor_msgs/JointState built from a smear feeds robot_state_publisher a pose the robot
        /// never actually held. At ROS rates the error is small and entirely invisible, which is
        /// the property that makes it worth avoiding rather than tolerating.
        ///
        /// ⚠ FIXED joints are excluded. They have no state to report, and ROS convention is that
        /// joint_states carries only actuatable joints — robot_state_publisher takes fixed
        /// transforms from the URDF itself.
        virtual std::vector<JointStateInfo> getJointStates() const = 0;
        virtual LinkPoseInfo getLinkPose(const std::string& link) const = 0;

        /// The robot's URDF, as the simulator actually loaded it.
        ///
        /// ⚠ Served over RPC rather than left for the client to read off disk, because the client
        /// frequently CANNOT read it. The ROS 2 wrapper runs in a container where only one host
        /// directory is mounted, so the UrdfFile path from settings does not resolve there —
        /// verified 2026-08-17. A client reading the path itself would publish an empty
        /// robot_description, and the only symptom would be "the robot does not appear in RViz".
        ///
        /// It also removes a correctness trap: this is the exact text this simulator parsed, so a
        /// description and the joint names it is matched against cannot drift.
        virtual std::string getUrdfXml() const = 0;

        /// Drive the robot with the SAME two axes the keyboard uses, in the same units: throttle
        /// and steering each in [-1, 1], scaled by UrdfDrive's MaxWheelSpeed and MaxSteerAngle and
        /// distributed over DriveJoints/SteerJoints by their per-joint multipliers.
        ///
        /// ⚠ Deliberately NOT implemented by mapping Twist to joints on the client side. The
        /// wheel/steer distribution, the speed and angle limits and the steering gains all live in
        /// UrdfBotSimApi::applyDriveInput; a second copy in the ROS wrapper would drift from it,
        /// and the two would disagree in exactly the situations that are hardest to debug — a robot
        /// that steers differently depending on who is driving it. One mapping, two input sources.
        ///
        /// ⚠ Requires enableApiControl(true). Without it the keyboard remains the source and this
        /// command is ignored — the same arbiter cars and drones already use.
        virtual void setDriveCommand(double throttle, double steering)
        {
            drive_throttle_.store(clampAxis(throttle), std::memory_order_relaxed);
            drive_steering_.store(clampAxis(steering), std::memory_order_relaxed);
            // ⚠ Latches on first use, and never clears while API control is held.
            //
            // applyDriveInput must keep standing down entirely for a client that drives individual
            // joints through setJointPosition/Velocity/Effort — otherwise the drive loop would
            // overwrite those targets every physics step, which is the silent-no-op failure this
            // whole gate exists to prevent. So the drive loop runs under API control ONLY once a
            // drive command has actually been issued.
            drive_command_issued_.store(true, std::memory_order_relaxed);
        }

        double getDriveThrottle() const { return drive_throttle_.load(std::memory_order_relaxed); }
        double getDriveSteering() const { return drive_steering_.load(std::memory_order_relaxed); }
        bool hasDriveCommand() const { return drive_command_issued_.load(std::memory_order_relaxed); }

        // VehicleApiBase — a URDF robot has no arming or flight-control concept, so these are
        // answered honestly rather than emulated.
        /// ⚠ Also RELEASES drive control, in both directions.
        ///
        /// setDriveCommand latches: once issued, the drive loop writes every drive and steer joint
        /// each physics step, so setJointPosition on those joints stops holding. Without a way to
        /// clear that latch it persisted for the life of the process — measured 2026-08-17, a
        /// steer joint commanded to +0.20, +0.10, +0.05 and -0.20 sat at -0.007 for all four,
        /// because a drive command issued minutes earlier still owned it. Every call succeeded.
        ///
        /// Tying the release to enableApiControl keeps one arbiter rather than adding a second:
        /// toggling control off and on returns the robot to joint-level command, and a client that
        /// never calls setDriveCommand is unaffected either way.
        virtual void enableApiControl(bool is_enabled) override
        {
            api_control_enabled_ = is_enabled;
            drive_command_issued_.store(false, std::memory_order_relaxed);
            drive_throttle_.store(0.0, std::memory_order_relaxed);
            drive_steering_.store(0.0, std::memory_order_relaxed);
        }
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

        /// Drive axes last commanded over RPC. Atomic because they are written by the RPC thread
        /// and read by the physics thread; relaxed ordering throughout, because a one-step-old
        /// throttle is indistinguishable from a command issued one step later and there is nothing
        /// to order it against.
        std::atomic<double> drive_throttle_{ 0.0 };
        std::atomic<double> drive_steering_{ 0.0 };
        std::atomic<bool> drive_command_issued_{ false };

        static double clampAxis(double v) { return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v); }

        std::shared_ptr<SensorFactory> sensor_factory_;
        SensorCollection sensors_;
        vector<shared_ptr<SensorBase>> sensor_storage_;  // RAII for created sensors
    };
}
} //namespace
