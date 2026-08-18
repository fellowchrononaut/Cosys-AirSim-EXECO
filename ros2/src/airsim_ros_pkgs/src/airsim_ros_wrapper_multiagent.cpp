#include <airsim_ros_wrapper_multiagent.h>
#include <fstream>
#include <sstream>
#include "common/AirSimSettings.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using namespace std::placeholders;

// ============================================================================
// Static member definitions
// ============================================================================

constexpr char AirsimROSWrapperMultiAgent::CAM_YML_NAME[];
constexpr char AirsimROSWrapperMultiAgent::WIDTH_YML_NAME[];
constexpr char AirsimROSWrapperMultiAgent::HEIGHT_YML_NAME[];
constexpr char AirsimROSWrapperMultiAgent::K_YML_NAME[];
constexpr char AirsimROSWrapperMultiAgent::D_YML_NAME[];
constexpr char AirsimROSWrapperMultiAgent::R_YML_NAME[];
constexpr char AirsimROSWrapperMultiAgent::P_YML_NAME[];
constexpr char AirsimROSWrapperMultiAgent::DMODEL_YML_NAME[];

const std::unordered_map<int, std::string> AirsimROSWrapperMultiAgent::image_type_int_to_string_map_ = {
    { 0, "Scene" },
    { 1, "DepthPlanar" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" },
    { 8, "OpticalFlow" },
    { 9, "OpticalFlowVis" },
    { 10, "Lighting" },
    { 11, "Annotation" },
};

// ============================================================================
// Constructor
// ============================================================================

AirsimROSWrapperMultiAgent::AirsimROSWrapperMultiAgent(
    const std::shared_ptr<rclcpp::Node> nh,
    const std::shared_ptr<rclcpp::Node> nh_img,
    const std::shared_ptr<rclcpp::Node> nh_lidar,
    const std::shared_ptr<rclcpp::Node> nh_gpulidar,
    const std::shared_ptr<rclcpp::Node> nh_echo,
    const std::string& host_ip,
    const std::shared_ptr<rclcpp::CallbackGroup> callbackGroup,
    bool enable_api_control,
    bool enable_object_transforms_list)
    : is_used_lidar_timer_cb_queue_(false)
    , is_used_img_timer_cb_queue_(false)
    , is_used_gpulidar_timer_cb_queue_(false)
    , is_used_echo_timer_cb_queue_(false)
    , airsim_settings_parser_(host_ip, DRONE_PORT)  // settings are fetched from the drone server
    , host_ip_(host_ip)
    , enable_api_control_(enable_api_control)
    , enable_object_transforms_list_(enable_object_transforms_list)
    , multirotor_client_(nullptr)
    , car_client_(nullptr)
    , cv_client_(nullptr)
    , urdfbot_client_(nullptr)
    // Dedicated image connections — one per server
    , airsim_client_images_drone_(host_ip, DRONE_PORT)
    , airsim_client_images_car_(host_ip, CAR_PORT)
    , airsim_client_images_cv_(host_ip, CV_PORT)
    , airsim_client_images_urdfbot_(host_ip, URDFBOT_PORT)
    // Dedicated lidar connections
    , airsim_client_lidar_drone_(host_ip, DRONE_PORT)
    , airsim_client_lidar_car_(host_ip, CAR_PORT)
    , airsim_client_lidar_cv_(host_ip, CV_PORT)
    , airsim_client_lidar_urdfbot_(host_ip, URDFBOT_PORT)
    // Dedicated GPU-lidar connections
    , airsim_client_gpulidar_drone_(host_ip, DRONE_PORT)
    , airsim_client_gpulidar_car_(host_ip, CAR_PORT)
    , airsim_client_gpulidar_cv_(host_ip, CV_PORT)
    , airsim_client_gpulidar_urdfbot_(host_ip, URDFBOT_PORT)
    // Dedicated echo connections
    , airsim_client_echo_drone_(host_ip, DRONE_PORT)
    , airsim_client_echo_car_(host_ip, CAR_PORT)
    , airsim_client_echo_cv_(host_ip, CV_PORT)
    , airsim_client_echo_urdfbot_(host_ip, URDFBOT_PORT)
    , nh_(nh)
    , nh_img_(nh_img)
    , nh_lidar_(nh_lidar)
    , nh_gpulidar_(nh_gpulidar)
    , nh_echo_(nh_echo)
    , cb_(callbackGroup)
    , publish_clock_(false)
    , has_gimbal_cmd_(false)
{
    ros_clock_.clock = rclcpp::Time(0);

    RCLCPP_INFO(nh_->get_logger(), "AirsimROSWrapperMultiAgent: SimMode = MultiAgent");
    RCLCPP_INFO(nh_->get_logger(), "  Drone   server → %s:%u", host_ip_.c_str(), DRONE_PORT);
    RCLCPP_INFO(nh_->get_logger(), "  Car     server → %s:%u", host_ip_.c_str(), CAR_PORT);
    RCLCPP_INFO(nh_->get_logger(), "  CV      server → %s:%u", host_ip_.c_str(), CV_PORT);
    RCLCPP_INFO(nh_->get_logger(), "  URDF    server → %s:%u", host_ip_.c_str(), URDFBOT_PORT);

    tf_broadcaster_  = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
    static_tf_pub_   = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nh_);

    initialize_ros();

    RCLCPP_INFO(nh_->get_logger(), "AirsimROSWrapperMultiAgent initialized!");
}

// ============================================================================
// initialize_airsim — connect all three typed clients + dedicated connections
// ============================================================================

void AirsimROSWrapperMultiAgent::initialize_airsim()
{
    try {
        multirotor_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, DRONE_PORT);
        car_client_        = std::make_unique<msr::airlib::CarRpcLibClient>(host_ip_, CAR_PORT);
        cv_client_         = std::make_unique<msr::airlib::ComputerVisionRpcLibClient>(host_ip_, CV_PORT);
        urdfbot_client_    = std::make_unique<msr::airlib::UrdfBotRpcLibClient>(host_ip_, URDFBOT_PORT);

        multirotor_client_->confirmConnection();
        car_client_->confirmConnection();
        cv_client_->confirmConnection();
        urdfbot_client_->confirmConnection();
        publish_urdf_descriptions();

        airsim_client_images_drone_.confirmConnection();
        airsim_client_images_car_.confirmConnection();
        airsim_client_images_cv_.confirmConnection();
        airsim_client_images_urdfbot_.confirmConnection();

        airsim_client_lidar_drone_.confirmConnection();
        airsim_client_lidar_car_.confirmConnection();
        airsim_client_lidar_cv_.confirmConnection();
        airsim_client_lidar_urdfbot_.confirmConnection();

        airsim_client_gpulidar_drone_.confirmConnection();
        airsim_client_gpulidar_car_.confirmConnection();
        airsim_client_gpulidar_cv_.confirmConnection();
        airsim_client_gpulidar_urdfbot_.confirmConnection();

        airsim_client_echo_drone_.confirmConnection();
        airsim_client_echo_car_.confirmConnection();
        airsim_client_echo_cv_.confirmConnection();
        airsim_client_echo_urdfbot_.confirmConnection();

        if (enable_api_control_) {
            // ⚠ PER VEHICLE, and non-fatal. This loop used to be unguarded, so ONE vehicle
            // refusing API control killed the whole node — and a PX4 multirotor with no flight
            // controller attached refuses every time ("Cannot perform operation when no vehicle is
            // connected or vehicle is not responding"). The result was that enable_api_control:=true
            // could not be used at all in a mixed scene, which is exactly the scene this wrapper
            // exists for. A vehicle that will not take API control is simply not commandable; that
            // is not a reason to drop the other four.
            for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
                const std::string& vname = vehicle_name_ptr_pair.first;
                auto& client = get_state_client(vehicle_name_ptr_pair.second->vehicle_mode_);
                try {
                    client.enableApiControl(true, vname);
                    client.armDisarm(true, vname);
                }
                catch (const std::exception& e) {
                    RCLCPP_WARN(nh_->get_logger(),
                                "Vehicle '%s': could not take API control at startup (%s). It will "
                                "not accept commands until whatever it is waiting on is available; "
                                "other vehicles are unaffected.",
                                vname.c_str(), e.what());
                }
            }
        }

        origin_geo_point_ = get_origin_geo_point();
        origin_geo_point_msg_ = get_gps_msg_from_airsim_geo_point(origin_geo_point_);

        // Publish initial instance segmentation using any available vehicle
        auto vehicle_name_ptr_pair = vehicle_name_ptr_map_.begin();
        auto& vehicle_ros = vehicle_name_ptr_pair->second;
        airsim_interfaces::msg::InstanceSegmentationList seg_msg = get_instance_segmentation_list_msg_from_airsim();
        seg_msg.header.stamp = vehicle_ros->stamp_;
        seg_msg.header.frame_id = world_frame_id_;
        vehicle_ros->instance_segmentation_pub_->publish(seg_msg);

        if (enable_object_transforms_list_) {
            airsim_interfaces::msg::ObjectTransformsList obj_msg = get_object_transforms_list_msg_from_airsim(vehicle_ros->stamp_);
            vehicle_ros->object_transforms_pub_->publish(obj_msg);
        }
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception during AirSim connection:\n%s", msg.c_str());
        rclcpp::shutdown();
    }
}

// ============================================================================
// initialize_ros
// ============================================================================

/// Publish each URDF robot's description, once, on its latched topic.
///
/// ⚠ Called from initialize_airsim(), NOT from create_ros_pubs_from_settings_json(). The publishers
/// are created there, but urdfbot_client_ does not exist until initialize_airsim() runs at the end
/// of initialize_ros() — fetching the description at creation time dereferenced a null client and
/// killed the node outright.
///
/// ⚠ Fetched over RPC rather than read from UrdfFile. The simulator resolves that path on the HOST;
/// this node normally runs in a container that cannot see it — verified on sad_vio_dense, which
/// mounts one host directory and cannot open the ExoMy model at all. Reading the path here would
/// publish an empty description, whose only symptom is "the robot does not appear in RViz". The RPC
/// also returns the exact text the simulator parsed, so description and joint names cannot drift.
void AirsimROSWrapperMultiAgent::publish_urdf_descriptions()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;
        if (vehicle_ros->vehicle_mode_ != VehicleMode::URDFBOT) continue;

        auto urdf_ros = static_cast<UrdfBotROS*>(vehicle_ros.get());
        if (!urdf_ros->robot_description_pub_) continue;

        const std::string& vname = vehicle_ros->vehicle_name_;
        std_msgs::msg::String desc;
        try {
            desc.data = urdfbot_client_->getUrdfXml(vname);
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(nh_->get_logger(), "Vehicle '%s': getUrdfXml failed (%s)", vname.c_str(), e.what());
        }

        if (desc.data.empty()) {
            // Fallback for a node running on the host, where UrdfFile does resolve.
            std::ifstream in(urdf_ros->urdf_file_);
            if (in) { std::stringstream ss; ss << in.rdbuf(); desc.data = ss.str(); }
        }

        if (!desc.data.empty()) {
            urdf_ros->robot_description_pub_->publish(desc);
            RCLCPP_INFO(nh_->get_logger(), "Vehicle '%s': published robot_description (%zu bytes)",
                        vname.c_str(), desc.data.size());
        }
        else {
            RCLCPP_ERROR(nh_->get_logger(),
                         "Vehicle '%s': robot_description is EMPTY - neither getUrdfXml nor UrdfFile "
                         "yielded anything. robot_state_publisher cannot build link TF and the robot "
                         "will not appear in RViz.", vname.c_str());
        }
    }
}

void AirsimROSWrapperMultiAgent::initialize_ros()
{
    double update_airsim_control_every_n_sec;
    nh_->get_parameter("is_vulkan", is_vulkan_);
    nh_->get_parameter("update_airsim_control_every_n_sec", update_airsim_control_every_n_sec);
    nh_->get_parameter("publish_clock", publish_clock_);
    nh_->get_parameter_or("world_frame_id", world_frame_id_, world_frame_id_);
    nh_->get_parameter_or("odom_frame_id", odom_frame_id_, odom_frame_id_);
    vel_cmd_duration_ = 0.05;

    nh_->declare_parameter("vehicle_name", rclcpp::ParameterValue(""));
    nh_->declare_parameter("urdf_drive_cmd_timeout", rclcpp::ParameterValue(0.5));
    nh_->get_parameter("urdf_drive_cmd_timeout", urdf_drive_cmd_timeout_);
    create_ros_pubs_from_settings_json();
    airsim_control_update_timer_ = nh_->create_wall_timer(
        std::chrono::duration<double>(update_airsim_control_every_n_sec),
        std::bind(&AirsimROSWrapperMultiAgent::drone_state_timer_cb, this),
        cb_);
}

// ============================================================================
// Client routing helpers
// ============================================================================

msr::airlib::RpcLibClientBase& AirsimROSWrapperMultiAgent::get_state_client(VehicleMode mode)
{
    switch (mode) {
        case VehicleMode::DRONE: return *multirotor_client_;
        case VehicleMode::CAR:   return *car_client_;
        case VehicleMode::CV:    return *cv_client_;
        case VehicleMode::URDFBOT: return *urdfbot_client_;
    }
    return *multirotor_client_;
}

msr::airlib::RpcLibClientBase& AirsimROSWrapperMultiAgent::get_images_client(VehicleMode mode)
{
    switch (mode) {
        case VehicleMode::DRONE: return airsim_client_images_drone_;
        case VehicleMode::CAR:   return airsim_client_images_car_;
        case VehicleMode::CV:    return airsim_client_images_cv_;
        case VehicleMode::URDFBOT: return airsim_client_images_urdfbot_;
    }
    return airsim_client_images_drone_;
}

msr::airlib::RpcLibClientBase& AirsimROSWrapperMultiAgent::get_lidar_client(VehicleMode mode)
{
    switch (mode) {
        case VehicleMode::DRONE: return airsim_client_lidar_drone_;
        case VehicleMode::CAR:   return airsim_client_lidar_car_;
        case VehicleMode::CV:    return airsim_client_lidar_cv_;
        case VehicleMode::URDFBOT: return airsim_client_lidar_urdfbot_;
    }
    return airsim_client_lidar_drone_;
}

msr::airlib::RpcLibClientBase& AirsimROSWrapperMultiAgent::get_gpulidar_client(VehicleMode mode)
{
    switch (mode) {
        case VehicleMode::DRONE: return airsim_client_gpulidar_drone_;
        case VehicleMode::CAR:   return airsim_client_gpulidar_car_;
        case VehicleMode::CV:    return airsim_client_gpulidar_cv_;
        case VehicleMode::URDFBOT: return airsim_client_gpulidar_urdfbot_;
    }
    return airsim_client_gpulidar_drone_;
}

msr::airlib::RpcLibClientBase& AirsimROSWrapperMultiAgent::get_echo_client(VehicleMode mode)
{
    switch (mode) {
        case VehicleMode::DRONE: return airsim_client_echo_drone_;
        case VehicleMode::CAR:   return airsim_client_echo_car_;
        case VehicleMode::CV:    return airsim_client_echo_cv_;
        case VehicleMode::URDFBOT: return airsim_client_echo_urdfbot_;
    }
    return airsim_client_echo_drone_;
}

// ============================================================================
// Vehicle type → VehicleMode mapping
// ============================================================================

AirsimROSWrapperMultiAgent::VehicleMode AirsimROSWrapperMultiAgent::get_vehicle_mode_from_type(const std::string& vehicle_type) const
{
    using S = msr::airlib::AirSimSettings;

    // Aerial vehicles → drone server (41451)
    if (vehicle_type == S::kVehicleTypeSimpleFlight ||
        vehicle_type == S::kVehicleTypePX4 ||
        vehicle_type == S::kVehicleTypeArduCopter ||
        vehicle_type == S::kVehicleTypeArduCopterSolo) {
        return VehicleMode::DRONE;
    }

    // Ground vehicles (wheeled + skid-steer) → car server (41452)
    // Note: BoxCar, Pioneer, CPHusky, ArduRover may be project-specific extensions
    if (vehicle_type == S::kVehicleTypePhysXCar ||
        vehicle_type == "boxcar" ||
        vehicle_type == "pioneer" ||
        vehicle_type == "cphusky" ||
        vehicle_type == "ardurover") {
        return VehicleMode::CAR;
    }

    // Computer vision → CV server (41453)
    if (vehicle_type == S::kVehicleTypeComputerVision) {
        return VehicleMode::CV;
    }

    // URDF robots → urdfbot server (41454). Not folded into CAR: the car server casts its vehicle
    // APIs to CarApiBase, so routing a urdfbot there is a bad cast rather than a graceful failure.
    if (vehicle_type == S::kVehicleTypeUrdfBot) {
        return VehicleMode::URDFBOT;
    }

    RCLCPP_WARN(rclcpp::get_logger("AirsimROSWrapperMultiAgent"),
                "Unknown vehicle type '%s', defaulting to DRONE mode.", vehicle_type.c_str());
    return VehicleMode::DRONE;
}

// ============================================================================
// create_ros_pubs_from_settings_json
// ============================================================================

void AirsimROSWrapperMultiAgent::create_ros_pubs_from_settings_json()
{
    gimbal_angle_quat_cmd_sub_   = nh_->create_subscription<airsim_interfaces::msg::GimbalAngleQuatCmd>("~/gimbal_angle_quat_cmd", 50, std::bind(&AirsimROSWrapperMultiAgent::gimbal_angle_quat_cmd_cb, this, _1));
    gimbal_angle_euler_cmd_sub_  = nh_->create_subscription<airsim_interfaces::msg::GimbalAngleEulerCmd>("~/gimbal_angle_euler_cmd", 50, std::bind(&AirsimROSWrapperMultiAgent::gimbal_angle_euler_cmd_cb, this, _1));
    origin_geo_point_pub_ = nh_->create_publisher<airsim_interfaces::msg::GPSYaw>("~/origin_geo_point", 10);

    airsim_img_request_vehicle_name_pair_vec_.clear();
    image_pub_vec_.clear();
    cam_info_pub_vec_.clear();
    camera_info_msg_vec_.clear();
    camera_model_pub_vec_.clear();
    vehicle_name_ptr_map_.clear();

    size_t lidar_cnt    = 0;
    size_t gpulidar_cnt = 0;
    size_t echo_cnt     = 0;

    image_transport::ImageTransport image_transporter(nh_);

    for (const auto& curr_vehicle_elem : AirSimSettings::singleton().vehicles) {
        auto& vehicle_setting    = curr_vehicle_elem.second;
        auto curr_vehicle_name   = curr_vehicle_elem.first;

        nh_->set_parameter(rclcpp::Parameter("vehicle_name", curr_vehicle_name));
        set_nans_to_zeros_in_pose(*vehicle_setting);

        // Determine this vehicle's server type from its vehicle_type string
        const VehicleMode vmode = get_vehicle_mode_from_type(vehicle_setting->vehicle_type);

        std::unique_ptr<VehicleROS> vehicle_ros = nullptr;
        switch (vmode) {
            case VehicleMode::DRONE:
                vehicle_ros = std::make_unique<MultiRotorROS>();
                RCLCPP_INFO(nh_->get_logger(), "Vehicle '%s' → DRONE (port %u)", curr_vehicle_name.c_str(), DRONE_PORT);
                break;
            case VehicleMode::CAR:
                vehicle_ros = std::make_unique<CarROS>();
                RCLCPP_INFO(nh_->get_logger(), "Vehicle '%s' → CAR   (port %u)", curr_vehicle_name.c_str(), CAR_PORT);
                break;
            case VehicleMode::CV:
                vehicle_ros = std::make_unique<ComputerVisionROS>();
                RCLCPP_INFO(nh_->get_logger(), "Vehicle '%s' → CV    (port %u)", curr_vehicle_name.c_str(), CV_PORT);
                break;
            case VehicleMode::URDFBOT:
                vehicle_ros = std::make_unique<UrdfBotROS>();
                RCLCPP_INFO(nh_->get_logger(), "Vehicle '%s' → URDF  (port %u)", curr_vehicle_name.c_str(), URDFBOT_PORT);
                break;
        }

        vehicle_ros->vehicle_name_    = curr_vehicle_name;
        vehicle_ros->vehicle_mode_    = vmode;
        vehicle_ros->odom_frame_id_   = curr_vehicle_name + "/" + odom_frame_id_;

        append_static_vehicle_tf(vehicle_ros.get(), *vehicle_setting);

        const std::string topic_prefix = "~/" + curr_vehicle_name;

        vehicle_ros->odom_local_pub_  = nh_->create_publisher<nav_msgs::msg::Odometry>(topic_prefix + "/" + odom_frame_id_, 10);
        // MAVLink vehicles only: the flight controller's own estimate, published alongside the
        // ground truth on odom_local rather than instead of it. See I-V.
        if (vehicle_ros->vehicle_mode_ == VehicleMode::DRONE)
            vehicle_ros->odom_estimated_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>(topic_prefix + "/" + odom_frame_id_ + "_estimated", 10);
        vehicle_ros->env_pub_         = nh_->create_publisher<airsim_interfaces::msg::Environment>(topic_prefix + "/environment", 10);
        vehicle_ros->global_gps_pub_  = nh_->create_publisher<sensor_msgs::msg::NavSatFix>(topic_prefix + "/global_gps", 10);

        auto qos_latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        vehicle_ros->instance_segmentation_pub_ = nh_->create_publisher<airsim_interfaces::msg::InstanceSegmentationList>("~/instance_segmentation_labels", qos_latched);

        std::function<bool(std::shared_ptr<airsim_interfaces::srv::RefreshInstanceSegmentation::Request>,
                           std::shared_ptr<airsim_interfaces::srv::RefreshInstanceSegmentation::Response>)>
            fcn_ins_seg = std::bind(&AirsimROSWrapperMultiAgent::instance_segmentation_refresh_cb, this, _1, _2);
        vehicle_ros->instance_segmentation_refresh_srvr_ = nh_->create_service<airsim_interfaces::srv::RefreshInstanceSegmentation>(topic_prefix + "/instance_segmentation_refresh", fcn_ins_seg);

        if (enable_object_transforms_list_) {
            vehicle_ros->object_transforms_pub_ = nh_->create_publisher<airsim_interfaces::msg::ObjectTransformsList>("~/object_transforms", qos_latched);

            std::function<bool(std::shared_ptr<airsim_interfaces::srv::RefreshObjectTransforms::Request>,
                               std::shared_ptr<airsim_interfaces::srv::RefreshObjectTransforms::Response>)>
                fcn_obj_trans = std::bind(&AirsimROSWrapperMultiAgent::object_transforms_refresh_cb, this, _1, _2);
            vehicle_ros->object_transforms_refresh_srvr_ = nh_->create_service<airsim_interfaces::srv::RefreshObjectTransforms>(topic_prefix + "/object_transforms_refresh", fcn_obj_trans);
        }

        // Vehicle-type-specific publishers / subscribers
        if (vmode == VehicleMode::DRONE) {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
            drone->has_vel_cmd_ = false;

            std::function<void(const airsim_interfaces::msg::VelCmd::SharedPtr)> fcn_body =
                std::bind(&AirsimROSWrapperMultiAgent::vel_cmd_body_frame_cb, this, _1, curr_vehicle_name);
            drone->vel_cmd_body_frame_sub_ = nh_->create_subscription<airsim_interfaces::msg::VelCmd>(topic_prefix + "/vel_cmd_body_frame", 1, fcn_body);

            std::function<void(const airsim_interfaces::msg::VelCmd::SharedPtr)> fcn_world =
                std::bind(&AirsimROSWrapperMultiAgent::vel_cmd_world_frame_cb, this, _1, curr_vehicle_name);
            drone->vel_cmd_world_frame_sub_ = nh_->create_subscription<airsim_interfaces::msg::VelCmd>(topic_prefix + "/vel_cmd_world_frame", 1, fcn_world);

            std::function<bool(std::shared_ptr<airsim_interfaces::srv::Takeoff::Request>,
                               std::shared_ptr<airsim_interfaces::srv::Takeoff::Response>)>
                fcn_takeoff = std::bind(&AirsimROSWrapperMultiAgent::takeoff_srv_cb, this, _1, _2, curr_vehicle_name);
            drone->takeoff_srvr_ = nh_->create_service<airsim_interfaces::srv::Takeoff>(topic_prefix + "/takeoff", fcn_takeoff);

            std::function<bool(std::shared_ptr<airsim_interfaces::srv::Land::Request>,
                               std::shared_ptr<airsim_interfaces::srv::Land::Response>)>
                fcn_land = std::bind(&AirsimROSWrapperMultiAgent::land_srv_cb, this, _1, _2, curr_vehicle_name);
            drone->land_srvr_ = nh_->create_service<airsim_interfaces::srv::Land>(topic_prefix + "/land", fcn_land);

        } else if (vmode == VehicleMode::CAR) {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            car->has_car_cmd_ = false;
            car->car_state_pub_ = nh_->create_publisher<airsim_interfaces::msg::CarState>(topic_prefix + "/car_state", 10);

            if (enable_api_control_) {
                std::function<void(const airsim_interfaces::msg::CarControls::SharedPtr)> fcn_car =
                    std::bind(&AirsimROSWrapperMultiAgent::car_cmd_cb, this, _1, curr_vehicle_name);
                car->car_cmd_sub_ = nh_->create_subscription<airsim_interfaces::msg::CarControls>(topic_prefix + "/car_cmd", 1, fcn_car);
            }

        } else if (vmode == VehicleMode::URDFBOT) {
            auto urdf_ros = static_cast<UrdfBotROS*>(vehicle_ros.get());
            urdf_ros->joint_state_pub_ =
                nh_->create_publisher<sensor_msgs::msg::JointState>(topic_prefix + "/joint_states", 10);

            // ⚠ LATCHED (transient_local + reliable). robot_state_publisher and RViz almost never
            // start before this node does, and a volatile description would simply be missed —
            // leaving a robot that publishes joint angles nobody can turn into transforms.
            urdf_ros->robot_description_pub_ = nh_->create_publisher<std_msgs::msg::String>(
                topic_prefix + "/robot_description", qos_latched);
            urdf_ros->urdf_file_ = vehicle_setting->urdf_file;

            if (enable_api_control_) {
                std::function<void(const geometry_msgs::msg::Twist::SharedPtr)> fcn_vel =
                    std::bind(&AirsimROSWrapperMultiAgent::urdf_cmd_vel_cb, this, _1, curr_vehicle_name);
                urdf_ros->cmd_vel_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(
                    topic_prefix + "/cmd_vel", 1, fcn_vel);

                std::function<void(const sensor_msgs::msg::JointState::SharedPtr)> fcn_joint =
                    std::bind(&AirsimROSWrapperMultiAgent::urdf_joint_cmd_cb, this, _1, curr_vehicle_name);
                urdf_ros->joint_cmd_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
                    topic_prefix + "/joint_cmd", 1, fcn_joint);

                // ⚠ Say the units out loud, once, per robot. linear.x and angular.z are treated as
                // NORMALISED axes in [-1, 1], NOT m/s and rad/s. Converting a metric Twist would
                // need the wheel radius, which the URDF's <collision> does not reliably give and
                // which UrdfDrive does not state — so a metric-looking mapping would be a guess
                // dressed as a measurement. A teleop publishing 0.5 gets half throttle.
                RCLCPP_INFO(nh_->get_logger(),
                            "Vehicle '%s': cmd_vel accepts NORMALISED axes - linear.x -> throttle "
                            "[-1,1], angular.z -> steering [-1,1]. These are NOT m/s and rad/s.",
                            curr_vehicle_name.c_str());
            }

            // ⚠ The DESCRIPTION IS NOT FETCHED HERE. initialize_ros() creates these publishers
            // and only calls initialize_airsim() at its very end, so urdfbot_client_ is still
            // nullptr at this point — dereferencing it segfaulted the whole node immediately after
            // the "Vehicle 'Rover' -> URDF" line, with no other symptom. Published from
            // publish_urdf_descriptions(), after the clients exist.
        } else { // CV
            auto cv = static_cast<ComputerVisionROS*>(vehicle_ros.get());
            cv->computer_vision_state_pub_ = nh_->create_publisher<airsim_interfaces::msg::ComputerVisionState>(topic_prefix + "/computervision_state", 10);
        }

        //Per Vehicle Image request vector
        std::vector<ImageRequest> current_image_request_vec;
        // Cameras
        for (auto& curr_camera_elem : vehicle_setting->cameras) {
            auto& camera_setting   = curr_camera_elem.second;
            auto& curr_camera_name = curr_camera_elem.first;

            set_nans_to_zeros_in_pose(*vehicle_setting, camera_setting);
            append_static_camera_tf(vehicle_ros.get(), curr_camera_name, camera_setting);

            // Phase 3b step 6: a generic camera's real calibration, once per CAMERA rather than
            // once per image type, and latched so a node that subscribes later still gets it.
            // Absent a CameraModel block this topic does not exist at all - the non-invasiveness
            // contract reaches the topic list too, not just message contents.
            if (camera_setting.camera_model.enabled) {
                auto model_pub = nh_->create_publisher<std_msgs::msg::String>(
                    topic_prefix + "/" + curr_camera_name + "/camera_model",
                    rclcpp::QoS(1).transient_local());
                std_msgs::msg::String model_msg;
                model_msg.data = generate_camera_model_json(camera_setting);
                model_pub->publish(model_msg);
                camera_model_pub_vec_.push_back(model_pub);
                RCLCPP_INFO(nh_->get_logger(),
                            "Camera %s is a generic %s camera: CameraInfo is published as "
                            "UNCALIBRATED unless the model is expressible; the calibration is on "
                            "%s/%s/camera_model. Depth on this camera is RANGE ALONG THE RAY in "
                            "metres, and out-of-domain pixels are NaN.",
                            curr_camera_name.c_str(),
                            msr::airlib::cameras::toString(camera_setting.camera_model.model.type),
                            topic_prefix.c_str(), curr_camera_name.c_str());
            }

            for (const auto& curr_capture_elem : camera_setting.capture_settings) {
                auto& capture_setting = curr_capture_elem.second;
                if (!std::isnan(capture_setting.fov_degrees)) {
                    ImageType curr_image_type = msr::airlib::Utils::toEnum<ImageType>(capture_setting.image_type);

                    if (curr_image_type == ImageType::Annotation) {
                        for (const auto& ann : AirSimSettings::singleton().annotator_settings) {
                            current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false, ann.name));
                            const std::string cam_topic = topic_prefix + "/" + curr_camera_name + "_" + image_type_int_to_string_map_.at(capture_setting.image_type) + "_" + ann.name;
                            image_pub_vec_.push_back(image_transporter.advertise(cam_topic + "/image", 1));
                            cam_info_pub_vec_.push_back(nh_->create_publisher<sensor_msgs::msg::CameraInfo>(cam_topic + "/camera_info", 10));
                            camera_info_msg_vec_.push_back(generate_cam_info(curr_camera_name, camera_setting, capture_setting));
                        }
                    } else {
                        if (curr_image_type == ImageType::DepthPlanar ||
                            curr_image_type == ImageType::DepthPerspective ||
                            curr_image_type == ImageType::DepthVis ||
                            curr_image_type == ImageType::DisparityNormalized) {
                            current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, true));
                        } else {
                            current_image_request_vec.push_back(ImageRequest(curr_camera_name, curr_image_type, false, false));
                        }
                        const std::string cam_topic = topic_prefix + "/" + curr_camera_name + "_" + image_type_int_to_string_map_.at(capture_setting.image_type);
                        image_pub_vec_.push_back(image_transporter.advertise(cam_topic + "/image", 1));
                        cam_info_pub_vec_.push_back(nh_->create_publisher<sensor_msgs::msg::CameraInfo>(cam_topic + "/camera_info", 10));
                        camera_info_msg_vec_.push_back(generate_cam_info(curr_camera_name, camera_setting, capture_setting));
                    }
                }
            }
        }

        // Full list of image requests for the vehicle pushed ONCE
        airsim_img_request_vehicle_name_pair_vec_.push_back({ current_image_request_vec, curr_vehicle_name, vmode });

        // Sensors
        for (auto& curr_sensor_map : vehicle_setting->sensors) {
            auto& sensor_name    = curr_sensor_map.first;
            auto& sensor_setting = curr_sensor_map.second;

            if (!sensor_setting->enabled)
                continue;

            switch (sensor_setting->sensor_type) {
            case SensorBase::SensorType::Barometer: {
                vehicle_ros->barometer_pubs_.emplace_back(
                    create_sensor_publisher<airsim_interfaces::msg::Altimeter>("Barometer sensor", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/altimeter/" + sensor_name, 10));
                break;
            }
            case SensorBase::SensorType::Imu: {
                vehicle_ros->imu_pubs_.emplace_back(
                    create_sensor_publisher<sensor_msgs::msg::Imu>("Imu sensor", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/imu/" + sensor_name, 10));
                break;
            }
            case SensorBase::SensorType::Gps: {
                vehicle_ros->gps_pubs_.emplace_back(
                    create_sensor_publisher<sensor_msgs::msg::NavSatFix>("Gps sensor", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/gps/" + sensor_name, 10));
                break;
            }
            case SensorBase::SensorType::Magnetometer: {
                vehicle_ros->magnetometer_pubs_.emplace_back(
                    create_sensor_publisher<sensor_msgs::msg::MagneticField>("Magnetometer sensor", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/magnetometer/" + sensor_name, 10));
                break;
            }
            case SensorBase::SensorType::Distance: {
                vehicle_ros->distance_pubs_.emplace_back(
                    create_sensor_publisher<sensor_msgs::msg::Range>("Distance sensor", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/distance/" + sensor_name, 10));
                break;
            }
            case SensorBase::SensorType::Lidar: {
                auto lidar_setting = *static_cast<LidarSetting*>(sensor_setting.get());
                msr::airlib::LidarSimpleParams params;
                params.initializeFromSettings(lidar_setting);
                append_static_lidar_tf(vehicle_ros.get(), sensor_name, params);
                vehicle_ros->lidar_pubs_.emplace_back(
                    create_sensor_publisher<sensor_msgs::msg::PointCloud2>("Lidar sensor", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/lidar/points/" + sensor_name, 10));
                vehicle_ros->lidar_labels_pubs_.emplace_back(
                    create_sensor_publisher<airsim_interfaces::msg::StringArray>("", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/lidar/labels/" + sensor_name, 10));
                ++lidar_cnt;
                break;
            }
            case SensorBase::SensorType::GPULidar: {
                auto gpulidar_setting = *static_cast<GPULidarSetting*>(sensor_setting.get());
                msr::airlib::GPULidarSimpleParams params;
                params.initializeFromSettings(gpulidar_setting);
                append_static_gpulidar_tf(vehicle_ros.get(), sensor_name, params);
                vehicle_ros->gpulidar_pubs_.emplace_back(
                    create_sensor_publisher<sensor_msgs::msg::PointCloud2>("GPULidar sensor", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/gpulidar/points/" + sensor_name, 10));
                ++gpulidar_cnt;
                break;
            }
            case SensorBase::SensorType::Echo: {
                auto echo_setting = *static_cast<EchoSetting*>(sensor_setting.get());
                msr::airlib::EchoSimpleParams params;
                params.initializeFromSettings(echo_setting);
                append_static_echo_tf(vehicle_ros.get(), sensor_name, params);
                if (params.active) {
                    vehicle_ros->echo_active_pubs_.emplace_back(
                        create_sensor_publisher<sensor_msgs::msg::PointCloud2>("Echo (active) sensor", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/echo/active/points/" + sensor_name, 10));
                    vehicle_ros->echo_active_labels_pubs_.emplace_back(
                        create_sensor_publisher<airsim_interfaces::msg::StringArray>("", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/echo/active/labels/" + sensor_name, 10));
                }
                if (params.passive) {
                    vehicle_ros->echo_passive_pubs_.emplace_back(
                        create_sensor_publisher<sensor_msgs::msg::PointCloud2>("Echo (passive) sensor", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/echo/passive/points/" + sensor_name, 10));
                    vehicle_ros->echo_passive_labels_pubs_.emplace_back(
                        create_sensor_publisher<airsim_interfaces::msg::StringArray>("", sensor_setting->sensor_name, sensor_setting->sensor_type, curr_vehicle_name + "/echo/passive/labels/" + sensor_name, 10));
                }
                ++echo_cnt;
                break;
            }
            default:
                throw std::invalid_argument("Unexpected sensor type");
            }
        }

        vehicle_name_ptr_map_.emplace(curr_vehicle_name, std::move(vehicle_ros));
    }

    // Count how many drone vehicles we have to decide whether to add fleet-wide services
    size_t drone_count = 0;
    for (const auto& p : vehicle_name_ptr_map_)
        if (p.second->vehicle_mode_ == VehicleMode::DRONE)
            ++drone_count;

    if (drone_count > 1) {
        takeoff_all_srvr_ = nh_->create_service<airsim_interfaces::srv::Takeoff>("~/all_robots/takeoff", std::bind(&AirsimROSWrapperMultiAgent::takeoff_all_srv_cb, this, _1, _2));
        land_all_srvr_    = nh_->create_service<airsim_interfaces::srv::Land>("~/all_robots/land", std::bind(&AirsimROSWrapperMultiAgent::land_all_srv_cb, this, _1, _2));

        vel_cmd_all_body_frame_sub_  = nh_->create_subscription<airsim_interfaces::msg::VelCmd>("~/all_robots/vel_cmd_body_frame", 1, std::bind(&AirsimROSWrapperMultiAgent::vel_cmd_all_body_frame_cb, this, _1));
        vel_cmd_all_world_frame_sub_ = nh_->create_subscription<airsim_interfaces::msg::VelCmd>("~/all_robots/vel_cmd_world_frame", 1, std::bind(&AirsimROSWrapperMultiAgent::vel_cmd_all_world_frame_cb, this, _1));

        vel_cmd_group_body_frame_sub_  = nh_->create_subscription<airsim_interfaces::msg::VelCmdGroup>("~/group_of_robots/vel_cmd_body_frame", 1, std::bind(&AirsimROSWrapperMultiAgent::vel_cmd_group_body_frame_cb, this, _1));
        vel_cmd_group_world_frame_sub_ = nh_->create_subscription<airsim_interfaces::msg::VelCmdGroup>("~/group_of_robots/vel_cmd_world_frame", 1, std::bind(&AirsimROSWrapperMultiAgent::vel_cmd_group_world_frame_cb, this, _1));

        takeoff_group_srvr_ = nh_->create_service<airsim_interfaces::srv::TakeoffGroup>("~/group_of_robots/takeoff", std::bind(&AirsimROSWrapperMultiAgent::takeoff_group_srv_cb, this, _1, _2));
        land_group_srvr_    = nh_->create_service<airsim_interfaces::srv::LandGroup>("~/group_of_robots/land", std::bind(&AirsimROSWrapperMultiAgent::land_group_srv_cb, this, _1, _2));
    }

    reset_srvr_                = nh_->create_service<airsim_interfaces::srv::Reset>("~/reset", std::bind(&AirsimROSWrapperMultiAgent::reset_srv_cb, this, _1, _2));
    list_scene_object_tags_srvr_ = nh_->create_service<airsim_interfaces::srv::ListSceneObjectTags>("~/list_scene_object_tags", std::bind(&AirsimROSWrapperMultiAgent::list_scene_object_tags_srv_cb, this, _1, _2));

    // Must be the ABSOLUTE topic "/clock". "~/clock" resolves to "/airsim_node/clock", which no
    // ROS node ever reads: rclcpp's use_sim_time implementation subscribes to /clock and nothing
    // else. Published privately, the sim clock was invisible to the graph, so every consumer fell
    // back to wall time while every message carried a sim-time stamp - the two drift apart by the
    // accumulated sim/wall deficit (measured: 274 s after a ~90 minute session). See I-J.
    if (publish_clock_)
        clock_pub_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

    if (!airsim_img_request_vehicle_name_pair_vec_.empty()) {
        double update_airsim_img_response_every_n_sec;
        nh_->get_parameter("update_airsim_img_response_every_n_sec", update_airsim_img_response_every_n_sec);
        airsim_img_response_timer_ = nh_img_->create_wall_timer(
            std::chrono::duration<double>(update_airsim_img_response_every_n_sec),
            std::bind(&AirsimROSWrapperMultiAgent::img_response_timer_cb, this), cb_);
        is_used_img_timer_cb_queue_ = true;
    }

    if (lidar_cnt > 0) {
        double update_lidar_every_n_sec;
        nh_->get_parameter("update_lidar_every_n_sec", update_lidar_every_n_sec);
        airsim_lidar_update_timer_ = nh_lidar_->create_wall_timer(
            std::chrono::duration<double>(update_lidar_every_n_sec),
            std::bind(&AirsimROSWrapperMultiAgent::lidar_timer_cb, this), cb_);
        is_used_lidar_timer_cb_queue_ = true;
    }

    if (gpulidar_cnt > 0) {
        double update_gpulidar_every_n_sec;
        nh_->get_parameter("update_gpulidar_every_n_sec", update_gpulidar_every_n_sec);
        airsim_gpulidar_update_timer_ = nh_gpulidar_->create_wall_timer(
            std::chrono::duration<double>(update_gpulidar_every_n_sec),
            std::bind(&AirsimROSWrapperMultiAgent::gpulidar_timer_cb, this), cb_);
        is_used_gpulidar_timer_cb_queue_ = true;
    }

    if (echo_cnt > 0) {
        double update_echo_every_n_sec;
        nh_->get_parameter("update_echo_every_n_sec", update_echo_every_n_sec);
        airsim_echo_update_timer_ = nh_echo_->create_wall_timer(
            std::chrono::duration<double>(update_echo_every_n_sec),
            std::bind(&AirsimROSWrapperMultiAgent::echo_timer_cb, this), cb_);
        is_used_echo_timer_cb_queue_ = true;
    }

    initialize_airsim();
}

// ============================================================================
// Sensor publisher factory
// ============================================================================

template <typename T>
const SensorPublisherMA<T> AirsimROSWrapperMultiAgent::create_sensor_publisher(
    const std::string& sensor_type_name,
    const std::string& sensor_name,
    SensorBase::SensorType sensor_type,
    const std::string& topic_name,
    int QoS)
{
    if (!sensor_type_name.empty())
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Publishing " << sensor_type_name << " '" << sensor_name << "'");

    SensorPublisherMA<T> sensor_publisher;
    sensor_publisher.sensor_name = sensor_name;
    sensor_publisher.sensor_type = sensor_type;
    sensor_publisher.publisher   = nh_->create_publisher<T>("~/" + topic_name, QoS);
    return sensor_publisher;
}

// ============================================================================
// Main state timer callback
// ============================================================================

void AirsimROSWrapperMultiAgent::drone_state_timer_cb()
{
    try {
        origin_geo_point_pub_->publish(origin_geo_point_msg_);

        const auto now = update_state();

        if (!multirotor_client_->simIsPaused())
            ros_clock_.clock = now;

        if (publish_clock_)
            clock_pub_->publish(ros_clock_);

        publish_vehicle_state();
        update_commands();
    }
    catch (rpc::rpc_error& e) {
        std::string msg = e.get_error().as<std::string>();
        RCLCPP_ERROR(nh_->get_logger(), "Exception raised by the API:\n%s", msg.c_str());
    }
}

// ============================================================================
// update_state — fetch state from the correct server per vehicle
// ============================================================================

rclcpp::Time AirsimROSWrapperMultiAgent::update_state()
{
    bool got_sim_time = false;
    rclcpp::Time curr_ros_time = nh_->now();

    // ⚠ Single pass, and it is single deliberately — this loop used to run TWICE.
    //
    // A URDF robot had no simulator timestamp of its own, so it inherited curr_ros_time from
    // whichever other vehicle set it first. vehicle_name_ptr_map_ is an std::unordered_map, so
    // "first" is hash order rather than insertion order, and it happened to put the rovers ahead
    // of the cars: every URDF robot was stamped with WALL CLOCK while a perfectly good simulator
    // clock existed two entries later. The two-pass ordering fixed that symptom.
    //
    // getUrdfBotState now carries a simulator stamp (added 2026-08-18), so every vehicle type
    // establishes sim time from its own state and no vehicle depends on another's ordering. The
    // ordering scaffolding is gone with the dependency it existed to sequence.
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;
        const std::string& vname = vehicle_ros->vehicle_name_;
        const VehicleMode vmode  = vehicle_ros->vehicle_mode_;

        rclcpp::Time vehicle_time;

        // Environment truth is available on all servers
        auto env_data = get_state_client(vmode).simGetGroundTruthEnvironment(vname);

        if (vmode == VehicleMode::DRONE) {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
            drone->curr_drone_state_ = multirotor_client_->getMultirotorState(vname);

            vehicle_time = rclcpp::Time(drone->curr_drone_state_.timestamp);
            if (!got_sim_time) { curr_ros_time = vehicle_time; got_sim_time = true; }

            vehicle_ros->gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(drone->curr_drone_state_.gps_location);
            vehicle_ros->gps_sensor_msg_.header.stamp = vehicle_time;

            // I-V: for MAVLink vehicles, MultirotorState::kinematics_estimated is NOT simulator
            // truth. MavLinkMultirotorApi::getKinematicsEstimated() returns
            // current_state_.local_est - PX4's LOCAL_POSITION_NED - and updateState() only writes
            // current_state_ when mav_vehicle_ != nullptr, so with PX4 not connected it returns a
            // default-constructed struct: all zeros, silently, with no validity signal. The
            // vehicle's TF (and therefore where its LiDAR cloud lands in the world) was being
            // built from that.
            //
            // Cars and SimpleFlight are unaffected: PhysXCarApi reports physics directly, and
            // AirSimSimpleFlightEstimator is a documented ground-truth pass-through. PX4Multirotor
            // is the only type where the field means something else.
            //
            // Publish simulator ground truth on odom_local, consistent with every other vehicle
            // type, and keep the flight-controller estimate on its own topic rather than discarding
            // it - it is the useful signal for evaluating the estimator, just not for registering
            // sensor data.
            vehicle_ros->curr_odom_ = get_odom_msg_from_kinematic_state(
                multirotor_client_->simGetGroundTruthKinematics(vname));
            vehicle_ros->curr_odom_estimated_ = get_odom_msg_from_multirotor_state(drone->curr_drone_state_);
            vehicle_ros->curr_odom_estimated_.header.frame_id = vname;
            vehicle_ros->curr_odom_estimated_.child_frame_id  = vehicle_ros->odom_frame_id_;
            vehicle_ros->curr_odom_estimated_.header.stamp    = vehicle_time;

        } else if (vmode == VehicleMode::CAR) {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            car->curr_car_state_ = car_client_->getCarState(vname);

            vehicle_time = rclcpp::Time(car->curr_car_state_.timestamp);
            if (!got_sim_time) { curr_ros_time = vehicle_time; got_sim_time = true; }

            vehicle_ros->gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(env_data.geo_point);
            vehicle_ros->gps_sensor_msg_.header.stamp = vehicle_time;
            vehicle_ros->curr_odom_ = get_odom_msg_from_car_state(car->curr_car_state_);

            airsim_interfaces::msg::CarState state_msg = get_roscarstate_msg_from_car_state(car->curr_car_state_);
            state_msg.header.frame_id = vname;
            car->car_state_msg_ = state_msg;

        } else if (vmode == VehicleMode::URDFBOT) {
            // ⚠ getUrdfBotState, NOT simGetGroundTruthKinematics — for the timestamp, not the pose.
            //
            // The kinematics the two return are the same. What simGetGroundTruthKinematics cannot
            // return is a time: Kinematics::State carries none and neither does Environment::State,
            // so a URDF robot used to have no simulator stamp at all. It inherited one from another
            // vehicle in a mixed scene, and in a urdfbot-ONLY scene fell back to WALL CLOCK — which
            // for a dataset generator is wrong in the worst way available, since the poses are
            // correct and only their times are not. Nothing downstream complains; the data simply
            // fails to register against every other stream.
            //
            // ⚠ Not fixed by reading a clock separately. That is two RPCs at two instants, so the
            // stamp describes neither sample, and under a paused or scaled clock the gap is
            // unbounded rather than one round-trip. The simulator publishes the pair together at
            // the moment it computes the kinematics (UrdfBotSimApi::updateRenderedState).
            //
            // This does NOT reintroduce a type-specific control abstraction — UrdfBotState is
            // shaped like ComputerVisionState: kinematics and a timestamp, nothing else. A URDF
            // robot still has no fixed control surface to report, and this does not invent one.
            const auto urdf_state = urdfbot_client_->getUrdfBotState(vname);

            vehicle_time = rclcpp::Time(urdf_state.timestamp);
            if (!got_sim_time) { curr_ros_time = vehicle_time; got_sim_time = true; }

            vehicle_ros->gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(env_data.geo_point);
            vehicle_ros->gps_sensor_msg_.header.stamp = vehicle_time;
            vehicle_ros->curr_odom_ = get_odom_msg_from_kinematic_state(urdf_state.kinematics_estimated);

            // ⚠ ONE batched call, not one call per joint. Polling joints individually samples them
            // at different instants, and a JointState assembled from that describes a pose the
            // robot never held — which robot_state_publisher would then turn into TF.
            auto urdf_ros = static_cast<UrdfBotROS*>(vehicle_ros.get());
            const auto js = urdfbot_client_->getJointStates(vname);
            auto& jm = urdf_ros->joint_state_msg_;
            jm.header.stamp = vehicle_time;
            jm.header.frame_id = vname;
            jm.name.clear(); jm.position.clear(); jm.velocity.clear(); jm.effort.clear();
            jm.name.reserve(js.size()); jm.position.reserve(js.size());
            jm.velocity.reserve(js.size()); jm.effort.reserve(js.size());
            for (const auto& j : js) {
                jm.name.push_back(j.name);
                jm.position.push_back(j.position);
                jm.velocity.push_back(j.velocity);
                jm.effort.push_back(j.effort);
            }

        } else { // CV
            auto cv = static_cast<ComputerVisionROS*>(vehicle_ros.get());
            cv->curr_computer_vision_state_ = cv_client_->getComputerVisionState(vname);

            vehicle_time = rclcpp::Time(cv->curr_computer_vision_state_.timestamp);
            if (!got_sim_time) { curr_ros_time = vehicle_time; got_sim_time = true; }

            vehicle_ros->gps_sensor_msg_ = get_gps_sensor_msg_from_airsim_geo_point(env_data.geo_point);
            vehicle_ros->gps_sensor_msg_.header.stamp = vehicle_time;
            vehicle_ros->curr_odom_ = get_odom_msg_from_computer_vision_state(cv->curr_computer_vision_state_);

            airsim_interfaces::msg::ComputerVisionState state_msg = get_roscomputervisionstate_msg_from_computer_vision_state(cv->curr_computer_vision_state_);
            state_msg.header.frame_id = vname;
            cv->computer_vision_state_msg_ = state_msg;
        }

        vehicle_ros->stamp_ = vehicle_time;

        airsim_interfaces::msg::Environment env_msg = get_environment_msg_from_airsim(env_data);
        env_msg.header.frame_id = vname;
        env_msg.header.stamp    = vehicle_time;
        vehicle_ros->env_msg_   = env_msg;

        vehicle_ros->curr_odom_.header.frame_id  = vname;
        vehicle_ros->curr_odom_.child_frame_id   = vehicle_ros->odom_frame_id_;
        vehicle_ros->curr_odom_.header.stamp     = vehicle_time;
    }

    return curr_ros_time;
}

// ============================================================================
// publish_vehicle_state
// ============================================================================

void AirsimROSWrapperMultiAgent::publish_vehicle_state()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;
        const VehicleMode vmode = vehicle_ros->vehicle_mode_;
        auto& base_client = get_state_client(vmode);

        vehicle_ros->env_pub_->publish(vehicle_ros->env_msg_);

        if (vmode == VehicleMode::CAR) {
            static_cast<CarROS*>(vehicle_ros.get())->car_state_pub_->publish(
                static_cast<CarROS*>(vehicle_ros.get())->car_state_msg_);
        } else if (vmode == VehicleMode::CV) {
            static_cast<ComputerVisionROS*>(vehicle_ros.get())->computer_vision_state_pub_->publish(
                static_cast<ComputerVisionROS*>(vehicle_ros.get())->computer_vision_state_msg_);
        } else if (vmode == VehicleMode::URDFBOT) {
            auto urdf_ros = static_cast<UrdfBotROS*>(vehicle_ros.get());
            if (urdf_ros->joint_state_pub_ && !urdf_ros->joint_state_msg_.name.empty())
                urdf_ros->joint_state_pub_->publish(urdf_ros->joint_state_msg_);
        }

        vehicle_ros->odom_local_pub_->publish(vehicle_ros->curr_odom_);
        // TF is built from ground truth, so sensor data registers against the true pose (I-V).
        publish_odom_tf(vehicle_ros->curr_odom_);
        if (vehicle_ros->odom_estimated_pub_)
            vehicle_ros->odom_estimated_pub_->publish(vehicle_ros->curr_odom_estimated_);
        vehicle_ros->global_gps_pub_->publish(vehicle_ros->gps_sensor_msg_);

        const std::string& vname = vehicle_ros->vehicle_name_;

        for (auto& pub : vehicle_ros->barometer_pubs_) {
            auto data = base_client.getBarometerData(pub.sensor_name, vname);
            auto msg  = get_altimeter_msg_from_airsim(data);
            msg.header.frame_id = vname;
            pub.publisher->publish(msg);
        }

        for (auto& pub : vehicle_ros->imu_pubs_) {
            auto data = base_client.getImuData(pub.sensor_name, vname);
            auto msg  = get_imu_msg_from_airsim(data);
            msg.header.frame_id = vname;
            pub.publisher->publish(msg);
        }

        for (auto& pub : vehicle_ros->distance_pubs_) {
            auto data = base_client.getDistanceSensorData(pub.sensor_name, vname);
            auto msg  = get_range_from_airsim(data);
            msg.header.frame_id = vname;
            pub.publisher->publish(msg);
        }

        for (auto& pub : vehicle_ros->gps_pubs_) {
            auto data = base_client.getGpsData(pub.sensor_name, vname);
            auto msg  = get_gps_msg_from_airsim(data);
            msg.header.frame_id = vname;
            pub.publisher->publish(msg);
        }

        for (auto& pub : vehicle_ros->magnetometer_pubs_) {
            auto data = base_client.getMagnetometerData(pub.sensor_name, vname);
            auto msg  = get_mag_msg_from_airsim(data);
            msg.header.frame_id = vname;
            pub.publisher->publish(msg);
        }

        update_and_publish_static_transforms(vehicle_ros.get());
    }
}

// ============================================================================
// update_commands
// ============================================================================

void AirsimROSWrapperMultiAgent::update_commands()
{
    for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
        auto& vehicle_ros = vehicle_name_ptr_pair.second;
        const VehicleMode vmode = vehicle_ros->vehicle_mode_;

        if (vmode == VehicleMode::DRONE) {
            auto drone = static_cast<MultiRotorROS*>(vehicle_ros.get());
            if (drone->has_vel_cmd_) {
                std::lock_guard<std::mutex> guard(control_mutex_);
                multirotor_client_->moveByVelocityAsync(
                    drone->vel_cmd_.x, drone->vel_cmd_.y, drone->vel_cmd_.z,
                    vel_cmd_duration_,
                    msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
                    drone->vel_cmd_.yaw_mode,
                    drone->vehicle_name_);
            }
            drone->has_vel_cmd_ = false;

        } else if (vmode == VehicleMode::CAR) {
            auto car = static_cast<CarROS*>(vehicle_ros.get());
            if (enable_api_control_ && car->has_car_cmd_) {
                std::lock_guard<std::mutex> guard(control_mutex_);
                car_client_->setCarControls(car->car_cmd_, vehicle_ros->vehicle_name_);
            }
            car->has_car_cmd_ = false;

        } else if (vmode == VehicleMode::URDFBOT) {
            auto urdf_ros = static_cast<UrdfBotROS*>(vehicle_ros.get());
            const std::string& vname = vehicle_ros->vehicle_name_;

            if (enable_api_control_ && (urdf_ros->has_drive_cmd_ || urdf_ros->has_joint_cmd_)) {
                std::lock_guard<std::mutex> guard(control_mutex_);

                // ⚠ Take API control on the FIRST command, not at startup. Until something
                // actually commands the robot the keyboard should keep working; the moment ROS
                // does, the keyboard must stand down, because the drive loop writes every drive
                // and steer joint on every physics step and would overwrite an RPC command 3 ms
                // after it was issued — a call that succeeds and does nothing.
                if (!urdf_ros->api_control_taken_) {
                    urdfbot_client_->enableApiControl(true, vname);
                    urdf_ros->api_control_taken_ = true;
                    RCLCPP_INFO(nh_->get_logger(),
                                "Vehicle '%s': ROS command received - taking API control; the "
                                "keyboard no longer drives this robot.", vname.c_str());
                }

                if (urdf_ros->has_drive_cmd_) {
                    urdfbot_client_->setDriveCommand(urdf_ros->drive_throttle_,
                                                     urdf_ros->drive_steering_, vname);
                    urdf_ros->last_drive_cmd_time_ = nh_->now();
                    urdf_ros->drive_stopped_by_watchdog_ = false;
                }

                if (urdf_ros->has_joint_cmd_) {
                    // ⚠ Whichever array is populated selects the mode, checked in the order
                    // position, velocity, effort. A message carrying several is not an error but
                    // only the first is honoured — the alternative is issuing three conflicting
                    // targets to one joint and letting the last write win invisibly.
                    const auto& jc = urdf_ros->joint_cmd_;
                    for (size_t i = 0; i < jc.name.size(); ++i) {
                        try {
                            if (i < jc.position.size())
                                urdfbot_client_->setJointPosition(jc.name[i], jc.position[i], vname);
                            else if (i < jc.velocity.size())
                                urdfbot_client_->setJointVelocity(jc.name[i], jc.velocity[i], vname);
                            else if (i < jc.effort.size())
                                urdfbot_client_->setJointEffort(jc.name[i], jc.effort[i], vname);
                        }
                        catch (const std::exception& e) {
                            // A joint name the robot does not have is client error, not fatal.
                            RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000,
                                                 "Vehicle '%s': joint command '%s' rejected: %s",
                                                 vname.c_str(), jc.name[i].c_str(), e.what());
                        }
                    }
                }
            }
            urdf_ros->has_drive_cmd_ = false;
            urdf_ros->has_joint_cmd_ = false;

            // ⚠ WATCHDOG. setDriveCommand is a latched setpoint in the simulator, so a publisher
            // that simply stops leaves the robot driving forever. Zero the axes once after
            // urdf_drive_cmd_timeout seconds of silence, then stay quiet until a new command
            // arrives — re-sending every cycle would fight a deliberate joint-level command.
            //
            // Only armed after this wrapper has actually taken control: a robot being driven by
            // the keyboard must not be stopped by a ROS topic nobody is publishing to.
            if (enable_api_control_ && urdf_ros->api_control_taken_ &&
                !urdf_ros->drive_stopped_by_watchdog_) {
                const double idle = (nh_->now() - urdf_ros->last_drive_cmd_time_).seconds();
                if (idle > urdf_drive_cmd_timeout_) {
                    std::lock_guard<std::mutex> guard(control_mutex_);
                    urdfbot_client_->setDriveCommand(0.0, 0.0, vname);
                    urdf_ros->drive_stopped_by_watchdog_ = true;
                    RCLCPP_WARN(nh_->get_logger(),
                                "Vehicle '%s': no cmd_vel for %.2f s - drive axes zeroed. The "
                                "simulator latches the last command, so a stopped publisher would "
                                "otherwise leave the robot driving.", vname.c_str(), idle);
                }
            }
        }
    }

    if (has_gimbal_cmd_) {
        std::lock_guard<std::mutex> guard(control_mutex_);
        // Gimbal calls go to whichever server owns that vehicle; use drone client as default
        multirotor_client_->simSetCameraPose(gimbal_cmd_.camera_name,
                                             get_airlib_pose(0, 0, 0, gimbal_cmd_.target_quat),
                                             gimbal_cmd_.vehicle_name);
    }
    has_gimbal_cmd_ = false;
}

// ============================================================================
// Lidar / GPULidar / Echo timer callbacks
// ============================================================================

void AirsimROSWrapperMultiAgent::lidar_timer_cb()
{
    try {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            auto& vehicle_ros = vehicle_name_ptr_pair.second;
            if (vehicle_ros->lidar_pubs_.empty()) continue;

            auto& lidar_client = get_lidar_client(vehicle_ros->vehicle_mode_);
            const std::string& vname = vehicle_name_ptr_pair.first;

            std::unordered_map<std::string, msr::airlib::LidarData> sensor_cache;

            // I-F: this timer fires at 100 Hz (update_lidar_every_n_sec: 0.01) while the sensor
            // completes a revolution at 5-20 Hz, so most ticks re-sent an identical cloud under an
            // identical header stamp: ~39 MB/s per LiDAR of pure duplication, against ~2-8 MB/s of
            // real data. Publish only when the source timestamp advances.
            //
            // NOTE this does NOT reduce the RPC read rate - the data must be fetched to learn its
            // timestamp - so it is a bandwidth fix, not a concurrency one. The concurrency problem
            // (I-S) is fixed in LidarBase by handing out an immutable snapshot. To cut RPC load as
            // well, raise update_lidar_every_n_sec toward the sensor's actual rate.
            for (auto& pub : vehicle_ros->lidar_pubs_) {
                auto data  = lidar_client.getLidarData(pub.sensor_name, vname);
                sensor_cache[pub.sensor_name] = data;
                if (data.time_stamp == pub.last_published_stamp)
                    continue;
                pub.last_published_stamp = data.time_stamp;
                pub.publisher->publish(get_lidar_msg_from_airsim(data, vname, pub.sensor_name));
            }
            for (auto& pub : vehicle_ros->lidar_labels_pubs_) {
                auto it = sensor_cache.find(pub.sensor_name);
                msr::airlib::LidarData data = (it != sensor_cache.end()) ? it->second : lidar_client.getLidarData(pub.sensor_name, vname);
                if (data.time_stamp == pub.last_published_stamp)
                    continue;
                pub.last_published_stamp = data.time_stamp;
                pub.publisher->publish(get_lidar_labels_msg_from_airsim(data, vname, pub.sensor_name));
            }
        }
    }
    catch (rpc::rpc_error& e) {
        RCLCPP_ERROR(nh_->get_logger(), "Lidar error: %s", e.get_error().as<std::string>().c_str());
    }
}

void AirsimROSWrapperMultiAgent::gpulidar_timer_cb()
{
    try {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            auto& vehicle_ros = vehicle_name_ptr_pair.second;
            if (vehicle_ros->gpulidar_pubs_.empty()) continue;

            auto& gpulidar_client = get_gpulidar_client(vehicle_ros->vehicle_mode_);
            const std::string& vname = vehicle_name_ptr_pair.first;

            for (auto& pub : vehicle_ros->gpulidar_pubs_) {
                auto data = gpulidar_client.getGPULidarData(pub.sensor_name, vname);
                pub.publisher->publish(get_gpulidar_msg_from_airsim(data, vname, pub.sensor_name));
            }
        }
    }
    catch (rpc::rpc_error& e) {
        RCLCPP_ERROR(nh_->get_logger(), "GPULidar error: %s", e.get_error().as<std::string>().c_str());
    }
}

void AirsimROSWrapperMultiAgent::echo_timer_cb()
{
    try {
        for (auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
            auto& vehicle_ros = vehicle_name_ptr_pair.second;
            if (vehicle_ros->echo_active_pubs_.empty() && vehicle_ros->echo_passive_pubs_.empty()) continue;

            auto& echo_client = get_echo_client(vehicle_ros->vehicle_mode_);
            const std::string& vname = vehicle_name_ptr_pair.first;

            std::unordered_map<std::string, msr::airlib::EchoData> sensor_cache;

            for (auto& pub : vehicle_ros->echo_active_pubs_) {
                auto data = echo_client.getEchoData(pub.sensor_name, vname);
                sensor_cache[pub.sensor_name] = data;
                pub.publisher->publish(get_active_echo_msg_from_airsim(data, vname, pub.sensor_name));
            }
            for (auto& pub : vehicle_ros->echo_passive_pubs_) {
                auto it = sensor_cache.find(pub.sensor_name);
                msr::airlib::EchoData data = (it != sensor_cache.end()) ? it->second : echo_client.getEchoData(pub.sensor_name, vname);
                sensor_cache[pub.sensor_name] = data;
                pub.publisher->publish(get_passive_echo_msg_from_airsim(data, vname, pub.sensor_name));
            }
            for (auto& pub : vehicle_ros->echo_active_labels_pubs_) {
                auto it = sensor_cache.find(pub.sensor_name);
                msr::airlib::EchoData data = (it != sensor_cache.end()) ? it->second : echo_client.getEchoData(pub.sensor_name, vname);
                sensor_cache[pub.sensor_name] = data;
                pub.publisher->publish(get_active_echo_labels_msg_from_airsim(data, vname, pub.sensor_name));
            }
            for (auto& pub : vehicle_ros->echo_passive_labels_pubs_) {
                auto it = sensor_cache.find(pub.sensor_name);
                msr::airlib::EchoData data = (it != sensor_cache.end()) ? it->second : echo_client.getEchoData(pub.sensor_name, vname);
                pub.publisher->publish(get_passive_echo_labels_msg_from_airsim(data, vname, pub.sensor_name));
            }
        }
    }
    catch (rpc::rpc_error& e) {
        RCLCPP_ERROR(nh_->get_logger(), "Echo error: %s", e.get_error().as<std::string>().c_str());
    }
}

// ============================================================================
// Image timer callback
// ============================================================================

void AirsimROSWrapperMultiAgent::img_response_timer_cb()
{
    try {
        int image_response_idx = 0;
        for (const auto& pair : airsim_img_request_vehicle_name_pair_vec_) {
            auto& img_client = get_images_client(pair.mode);
            const std::vector<ImageResponse>& img_response = img_client.simGetImages(pair.requests, pair.vehicle_name);

            if (img_response.size() == pair.requests.size()) {
                process_and_publish_img_response(img_response, image_response_idx, pair.vehicle_name);
                image_response_idx += img_response.size();
            }
        }
    }
    catch (rpc::rpc_error& e) {
        RCLCPP_ERROR(nh_->get_logger(), "Image error: %s", e.get_error().as<std::string>().c_str());
    }
}

// ============================================================================
// Service callbacks
// ============================================================================

bool AirsimROSWrapperMultiAgent::takeoff_srv_cb(
    const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response,
    const std::string& vehicle_name)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);
    if (request->wait_on_last_task)
        multirotor_client_->takeoffAsync(20, vehicle_name)->waitOnLastTask();
    else
        multirotor_client_->takeoffAsync(20, vehicle_name);
    return true;
}

bool AirsimROSWrapperMultiAgent::takeoff_group_srv_cb(
    const std::shared_ptr<airsim_interfaces::srv::TakeoffGroup::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::TakeoffGroup::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);
    for (const auto& vname : request->vehicle_names) {
        if (request->wait_on_last_task)
            multirotor_client_->takeoffAsync(20, vname)->waitOnLastTask();
        else
            multirotor_client_->takeoffAsync(20, vname);
    }
    return true;
}

bool AirsimROSWrapperMultiAgent::takeoff_all_srv_cb(
    const std::shared_ptr<airsim_interfaces::srv::Takeoff::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::Takeoff::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);
    for (const auto& p : vehicle_name_ptr_map_) {
        if (p.second->vehicle_mode_ != VehicleMode::DRONE) continue;
        if (request->wait_on_last_task)
            multirotor_client_->takeoffAsync(20, p.first)->waitOnLastTask();
        else
            multirotor_client_->takeoffAsync(20, p.first);
    }
    return true;
}

bool AirsimROSWrapperMultiAgent::land_srv_cb(
    const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::Land::Response> response,
    const std::string& vehicle_name)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);
    if (request->wait_on_last_task)
        multirotor_client_->landAsync(60, vehicle_name)->waitOnLastTask();
    else
        multirotor_client_->landAsync(60, vehicle_name);
    return true;
}

bool AirsimROSWrapperMultiAgent::land_group_srv_cb(
    const std::shared_ptr<airsim_interfaces::srv::LandGroup::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::LandGroup::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);
    for (const auto& vname : request->vehicle_names) {
        if (request->wait_on_last_task)
            multirotor_client_->landAsync(60, vname)->waitOnLastTask();
        else
            multirotor_client_->landAsync(60, vname);
    }
    return true;
}

bool AirsimROSWrapperMultiAgent::land_all_srv_cb(
    const std::shared_ptr<airsim_interfaces::srv::Land::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::Land::Response> response)
{
    unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);
    for (const auto& p : vehicle_name_ptr_map_) {
        if (p.second->vehicle_mode_ != VehicleMode::DRONE) continue;
        if (request->wait_on_last_task)
            multirotor_client_->landAsync(60, p.first)->waitOnLastTask();
        else
            multirotor_client_->landAsync(60, p.first);
    }
    return true;
}

bool AirsimROSWrapperMultiAgent::reset_srv_cb(
    const std::shared_ptr<airsim_interfaces::srv::Reset::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::Reset::Response> response)
{
    unused(request); unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);
    multirotor_client_->reset();
    car_client_->reset();
    cv_client_->reset();
    urdfbot_client_->reset();
    return true;
}

bool AirsimROSWrapperMultiAgent::instance_segmentation_refresh_cb(
    const std::shared_ptr<airsim_interfaces::srv::RefreshInstanceSegmentation::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::RefreshInstanceSegmentation::Response> response)
{
    unused(request); unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Starting instance segmentation refresh...");
    auto& vehicle_ros = vehicle_name_ptr_map_.begin()->second;
    auto msg = get_instance_segmentation_list_msg_from_airsim();
    msg.header.stamp     = vehicle_ros->stamp_;
    msg.header.frame_id  = world_frame_id_;
    vehicle_ros->instance_segmentation_pub_->publish(msg);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Completed instance segmentation refresh!");
    return true;
}

bool AirsimROSWrapperMultiAgent::object_transforms_refresh_cb(
    const std::shared_ptr<airsim_interfaces::srv::RefreshObjectTransforms::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::RefreshObjectTransforms::Response> response)
{
    unused(request); unused(response);
    std::lock_guard<std::mutex> guard(control_mutex_);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Starting object transforms refresh...");
    auto& vehicle_ros = vehicle_name_ptr_map_.begin()->second;
    vehicle_ros->object_transforms_pub_->publish(get_object_transforms_list_msg_from_airsim(vehicle_ros->stamp_));
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Completed object transforms refresh!");
    return true;
}

bool AirsimROSWrapperMultiAgent::list_scene_object_tags_srv_cb(
    const std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags::Request> request,
    const std::shared_ptr<airsim_interfaces::srv::ListSceneObjectTags::Response> response)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    std::string regex_name = request->regex_name.empty() ? ".*" : request->regex_name;
    auto tags = multirotor_client_->simListSceneObjectsTags(regex_name);
    for (const auto& pair : tags) {
        response->objects.push_back(pair.first);
        response->tags.push_back(pair.second);
    }
    return true;
}

// ============================================================================
// Subscriber callbacks
// ============================================================================

void AirsimROSWrapperMultiAgent::vel_cmd_body_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    drone->vel_cmd_      = get_airlib_body_vel_cmd(*msg, drone->curr_drone_state_.kinematics_estimated.pose.orientation);
    drone->has_vel_cmd_  = true;
}

void AirsimROSWrapperMultiAgent::vel_cmd_world_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    drone->vel_cmd_     = get_airlib_world_vel_cmd(*msg);
    drone->has_vel_cmd_ = true;
}

void AirsimROSWrapperMultiAgent::vel_cmd_group_body_frame_cb(const airsim_interfaces::msg::VelCmdGroup::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    for (const auto& vname : msg->vehicle_names) {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vname].get());
        drone->vel_cmd_     = get_airlib_body_vel_cmd(msg->vel_cmd, drone->curr_drone_state_.kinematics_estimated.pose.orientation);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapperMultiAgent::vel_cmd_group_world_frame_cb(const airsim_interfaces::msg::VelCmdGroup::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    for (const auto& vname : msg->vehicle_names) {
        auto drone = static_cast<MultiRotorROS*>(vehicle_name_ptr_map_[vname].get());
        drone->vel_cmd_     = get_airlib_world_vel_cmd(msg->vel_cmd);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapperMultiAgent::vel_cmd_all_body_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    for (auto& p : vehicle_name_ptr_map_) {
        if (p.second->vehicle_mode_ != VehicleMode::DRONE) continue;
        auto drone = static_cast<MultiRotorROS*>(p.second.get());
        drone->vel_cmd_     = get_airlib_body_vel_cmd(*msg, drone->curr_drone_state_.kinematics_estimated.pose.orientation);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapperMultiAgent::vel_cmd_all_world_frame_cb(const airsim_interfaces::msg::VelCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    for (auto& p : vehicle_name_ptr_map_) {
        if (p.second->vehicle_mode_ != VehicleMode::DRONE) continue;
        auto drone = static_cast<MultiRotorROS*>(p.second.get());
        drone->vel_cmd_     = get_airlib_world_vel_cmd(*msg);
        drone->has_vel_cmd_ = true;
    }
}

void AirsimROSWrapperMultiAgent::urdf_cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg,
                                                 const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    auto urdf_ros = static_cast<UrdfBotROS*>(vehicle_name_ptr_map_[vehicle_name].get());

    // ⚠ NORMALISED, not metric — see the startup log line. Clamped here so a teleop sending 2.0
    // saturates rather than being silently rescaled somewhere downstream.
    auto clamp1 = [](double v) { return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v); };
    urdf_ros->drive_throttle_ = clamp1(msg->linear.x);
    urdf_ros->drive_steering_ = clamp1(msg->angular.z);
    urdf_ros->has_drive_cmd_ = true;
}

void AirsimROSWrapperMultiAgent::urdf_joint_cmd_cb(const sensor_msgs::msg::JointState::SharedPtr msg,
                                                   const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    auto urdf_ros = static_cast<UrdfBotROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    urdf_ros->joint_cmd_ = *msg;
    urdf_ros->has_joint_cmd_ = true;
}

void AirsimROSWrapperMultiAgent::car_cmd_cb(const airsim_interfaces::msg::CarControls::SharedPtr msg, const std::string& vehicle_name)
{
    std::lock_guard<std::mutex> guard(control_mutex_);
    auto car = static_cast<CarROS*>(vehicle_name_ptr_map_[vehicle_name].get());
    car->car_cmd_.throttle      = msg->throttle;
    car->car_cmd_.steering      = msg->steering;
    car->car_cmd_.brake         = msg->brake;
    car->car_cmd_.handbrake     = msg->handbrake;
    car->car_cmd_.is_manual_gear = msg->manual;
    car->car_cmd_.manual_gear   = msg->manual_gear;
    car->car_cmd_.gear_immediate = msg->gear_immediate;
    car->has_car_cmd_ = true;
}

void AirsimROSWrapperMultiAgent::gimbal_angle_quat_cmd_cb(const airsim_interfaces::msg::GimbalAngleQuatCmd::SharedPtr msg)
{
    tf2::Quaternion q;
    try {
        tf2::convert(msg->orientation, q);
        q.normalize();
        gimbal_cmd_.target_quat  = get_airlib_quat(q);
        gimbal_cmd_.camera_name  = msg->camera_name;
        gimbal_cmd_.vehicle_name = msg->vehicle_name;
        has_gimbal_cmd_ = true;
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
    }
}

void AirsimROSWrapperMultiAgent::gimbal_angle_euler_cmd_cb(const airsim_interfaces::msg::GimbalAngleEulerCmd::SharedPtr msg)
{
    try {
        tf2::Quaternion q;
        q.setRPY(math_common::deg2rad(msg->roll), math_common::deg2rad(msg->pitch), math_common::deg2rad(msg->yaw));
        q.normalize();
        gimbal_cmd_.target_quat  = get_airlib_quat(q);
        gimbal_cmd_.camera_name  = msg->camera_name;
        gimbal_cmd_.vehicle_name = msg->vehicle_name;
        has_gimbal_cmd_ = true;
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
    }
}

// ============================================================================
// TF helpers
// ============================================================================

void AirsimROSWrapperMultiAgent::publish_odom_tf(const nav_msgs::msg::Odometry& odom_msg)
{
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header              = odom_msg.header;
    odom_tf.child_frame_id      = odom_msg.child_frame_id;
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation  = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(odom_tf);
}

void AirsimROSWrapperMultiAgent::update_and_publish_static_transforms(VehicleROS* vehicle_ros)
{
    if (vehicle_ros && !vehicle_ros->static_tf_msg_vec_.empty()) {
        for (auto& tf_msg : vehicle_ros->static_tf_msg_vec_) {
            tf_msg.header.stamp = vehicle_ros->stamp_;
            static_tf_pub_->sendTransform(tf_msg);
        }
    }
}

void AirsimROSWrapperMultiAgent::convert_tf_msg_to_ros(geometry_msgs::msg::TransformStamped& tf_msg)
{
    tf_msg.transform.translation.z = -tf_msg.transform.translation.z;
    tf_msg.transform.translation.y = -tf_msg.transform.translation.y;
    tf_msg.transform.rotation.z    = -tf_msg.transform.rotation.z;
    tf_msg.transform.rotation.y    = -tf_msg.transform.rotation.y;
}

void AirsimROSWrapperMultiAgent::append_static_vehicle_tf(VehicleROS* vehicle_ros, const VehicleSetting& vehicle_setting)
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id  = world_frame_id_;
    tf_msg.header.stamp     = nh_->now();
    tf_msg.child_frame_id   = vehicle_ros->vehicle_name_;

    // ⚠ TRANSLATION ONLY. The spawn ROTATION must NOT go in here, and leaving it in rotated every
    // vehicle's sensor data by its own spawn yaw.
    //
    // simGetVehiclePose reports in a HYBRID frame: position relative to the vehicle's spawn,
    // orientation ABSOLUTE. That is upstream AirSim behaviour, not a fork quirk — NedTransform's
    // per-pawn pivot is a POSITION offset only (`local_ned_offset_ = pivot->GetActorLocation()`),
    // and toNed(FQuat) is an axis flip that never sees the pivot. Measured 2026-08-17 on five
    // vehicles: every one reported position (0.00, 0.00) at spawn while reporting its true spawn
    // yaw (60 -> 60.0, 300 -> 300.0, 180 -> 181.8).
    //
    // The odom transform published under this one therefore ALREADY carries the absolute
    // orientation. Putting the spawn rotation here too composed spawn_yaw with absolute_yaw, so at
    // spawn each body frame sat at 2x its yaw — and every sensor frame hangs off that body frame.
    // Invisible when all vehicles spawn at yaw 0 (execo_test.json), which is why it survived: the
    // clouds only visibly separate once vehicles face different directions.
    //
    // With translation only the chain is consistent: this frame is world-axis-aligned at the spawn
    // point, the odom delta is expressed in world axes (both operands of the subtraction above are
    // world Unreal coordinates), and the odom orientation supplies the true heading.
    msr::airlib::AirSimSettings::Rotation no_rotation;
    no_rotation.yaw = 0.0f; no_rotation.pitch = 0.0f; no_rotation.roll = 0.0f;
    tf_msg.transform        = get_transform_msg_from_airsim(vehicle_setting.position, no_rotation);
    convert_tf_msg_to_ros(tf_msg);
    vehicle_ros->static_tf_msg_vec_.emplace_back(tf_msg);
}

void AirsimROSWrapperMultiAgent::append_static_lidar_tf(VehicleROS* vehicle_ros, const std::string& lidar_name, const msr::airlib::LidarSimpleParams& lidar_setting)
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = lidar_setting.external ? world_frame_id_ : vehicle_ros->vehicle_name_ + "/" + odom_frame_id_;
    tf_msg.child_frame_id  = vehicle_ros->vehicle_name_ + "/" + lidar_name;
    auto data = get_lidar_client(vehicle_ros->vehicle_mode_).getLidarData(lidar_name, vehicle_ros->vehicle_name_);
    tf_msg.transform = get_transform_msg_from_airsim(data.pose.position, data.pose.orientation);
    convert_tf_msg_to_ros(tf_msg);
    vehicle_ros->static_tf_msg_vec_.emplace_back(tf_msg);
}

void AirsimROSWrapperMultiAgent::append_static_gpulidar_tf(VehicleROS* vehicle_ros, const std::string& gpulidar_name, const msr::airlib::GPULidarSimpleParams& gpulidar_setting)
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = gpulidar_setting.external ? world_frame_id_ : vehicle_ros->vehicle_name_ + "/" + odom_frame_id_;
    tf_msg.child_frame_id  = vehicle_ros->vehicle_name_ + "/" + gpulidar_name;
    auto data = get_gpulidar_client(vehicle_ros->vehicle_mode_).getGPULidarData(gpulidar_name, vehicle_ros->vehicle_name_);
    tf_msg.transform = get_transform_msg_from_airsim(data.pose.position, data.pose.orientation);
    convert_tf_msg_to_ros(tf_msg);
    vehicle_ros->static_tf_msg_vec_.emplace_back(tf_msg);
}

void AirsimROSWrapperMultiAgent::append_static_echo_tf(VehicleROS* vehicle_ros, const std::string& echo_name, const msr::airlib::EchoSimpleParams& echo_setting)
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = echo_setting.external ? world_frame_id_ : vehicle_ros->vehicle_name_ + "/" + odom_frame_id_;
    tf_msg.child_frame_id  = vehicle_ros->vehicle_name_ + "/" + echo_name;
    auto data = get_echo_client(vehicle_ros->vehicle_mode_).getEchoData(echo_name, vehicle_ros->vehicle_name_);
    tf_msg.transform = get_transform_msg_from_airsim(data.pose.position, data.pose.orientation);
    convert_tf_msg_to_ros(tf_msg);
    vehicle_ros->static_tf_msg_vec_.emplace_back(tf_msg);
}

void AirsimROSWrapperMultiAgent::append_static_camera_tf(VehicleROS* vehicle_ros, const std::string& camera_name, const CameraSetting& camera_setting)
{
    geometry_msgs::msg::TransformStamped body_tf;
    body_tf.header.frame_id = camera_setting.external ? world_frame_id_ : vehicle_ros->vehicle_name_ + "/" + odom_frame_id_;
    body_tf.child_frame_id  = vehicle_ros->vehicle_name_ + "/" + camera_name + "_body";
    auto info = get_images_client(vehicle_ros->vehicle_mode_).simGetCameraInfo(camera_name, vehicle_ros->vehicle_name_);
    body_tf.transform = get_transform_msg_from_airsim(info.pose.position, info.pose.orientation);
    convert_tf_msg_to_ros(body_tf);

    geometry_msgs::msg::TransformStamped optical_tf = body_tf;
    optical_tf.child_frame_id = vehicle_ros->vehicle_name_ + "/" + camera_name + "_optical";
    optical_tf.transform      = get_camera_optical_tf_from_body_tf(body_tf.transform);

    vehicle_ros->static_tf_msg_vec_.emplace_back(body_tf);
    vehicle_ros->static_tf_msg_vec_.emplace_back(optical_tf);
}

// ============================================================================
// AirLib ↔ ROS conversion utilities (identical to the original wrapper)
// ============================================================================

tf2::Quaternion AirsimROSWrapperMultiAgent::get_tf2_quat(const msr::airlib::Quaternionr& q) const
{
    return tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
}

msr::airlib::Quaternionr AirsimROSWrapperMultiAgent::get_airlib_quat(const geometry_msgs::msg::Quaternion& q) const
{
    return msr::airlib::Quaternionr(q.w, q.x, q.y, q.z);
}

msr::airlib::Quaternionr AirsimROSWrapperMultiAgent::get_airlib_quat(const tf2::Quaternion& q) const
{
    return msr::airlib::Quaternionr(q.w(), q.x(), q.y(), q.z());
}

nav_msgs::msg::Odometry AirsimROSWrapperMultiAgent::get_odom_msg_from_kinematic_state(const msr::airlib::Kinematics::State& k) const
{
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x    = k.pose.position.x();
    odom.pose.pose.position.y    = k.pose.position.y();
    odom.pose.pose.position.z    = k.pose.position.z();
    odom.pose.pose.orientation.x = k.pose.orientation.x();
    odom.pose.pose.orientation.y = k.pose.orientation.y();
    odom.pose.pose.orientation.z = k.pose.orientation.z();
    odom.pose.pose.orientation.w = k.pose.orientation.w();
    odom.twist.twist.linear.x    = k.twist.linear.x();
    odom.twist.twist.linear.y    = k.twist.linear.y();
    odom.twist.twist.linear.z    = k.twist.linear.z();
    odom.twist.twist.angular.x   = k.twist.angular.x();
    odom.twist.twist.angular.y   = k.twist.angular.y();
    odom.twist.twist.angular.z   = k.twist.angular.z();
    // NED → ROS coordinate flip
    odom.pose.pose.position.y    = -odom.pose.pose.position.y;
    odom.pose.pose.position.z    = -odom.pose.pose.position.z;
    odom.pose.pose.orientation.y = -odom.pose.pose.orientation.y;
    odom.pose.pose.orientation.z = -odom.pose.pose.orientation.z;
    odom.twist.twist.linear.y    = -odom.twist.twist.linear.y;
    odom.twist.twist.linear.z    = -odom.twist.twist.linear.z;
    odom.twist.twist.angular.y   = -odom.twist.twist.angular.y;
    odom.twist.twist.angular.z   = -odom.twist.twist.angular.z;
    return odom;
}

nav_msgs::msg::Odometry AirsimROSWrapperMultiAgent::get_odom_msg_from_multirotor_state(const msr::airlib::MultirotorState& s) const
{
    return get_odom_msg_from_kinematic_state(s.kinematics_estimated);
}

nav_msgs::msg::Odometry AirsimROSWrapperMultiAgent::get_odom_msg_from_car_state(const msr::airlib::CarApiBase::CarState& s) const
{
    return get_odom_msg_from_kinematic_state(s.kinematics_estimated);
}

nav_msgs::msg::Odometry AirsimROSWrapperMultiAgent::get_odom_msg_from_computer_vision_state(const msr::airlib::ComputerVisionApiBase::ComputerVisionState& s) const
{
    return get_odom_msg_from_kinematic_state(s.kinematics_estimated);
}

airsim_interfaces::msg::CarState AirsimROSWrapperMultiAgent::get_roscarstate_msg_from_car_state(const msr::airlib::CarApiBase::CarState& car_state) const
{
    airsim_interfaces::msg::CarState msg;
    const auto odo = get_odom_msg_from_car_state(car_state);
    msg.pose      = odo.pose;
    msg.twist     = odo.twist;
    msg.speed     = car_state.speed;
    msg.gear      = car_state.gear;
    msg.rpm       = car_state.rpm;
    msg.maxrpm    = car_state.maxrpm;
    msg.handbrake = car_state.handbrake;
    msg.header.stamp = rclcpp::Time(car_state.timestamp);
    return msg;
}

airsim_interfaces::msg::ComputerVisionState AirsimROSWrapperMultiAgent::get_roscomputervisionstate_msg_from_computer_vision_state(const msr::airlib::ComputerVisionApiBase::ComputerVisionState& cv_state) const
{
    airsim_interfaces::msg::ComputerVisionState msg;
    const auto odo = get_odom_msg_from_computer_vision_state(cv_state);
    msg.pose  = odo.pose;
    msg.twist = odo.twist;
    msg.header.stamp = rclcpp::Time(cv_state.timestamp);
    return msg;
}

msr::airlib::Pose AirsimROSWrapperMultiAgent::get_airlib_pose(const float& x, const float& y, const float& z, const msr::airlib::Quaternionr& q) const
{
    return msr::airlib::Pose(msr::airlib::Vector3r(x, y, z), q);
}

airsim_interfaces::msg::GPSYaw AirsimROSWrapperMultiAgent::get_gps_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    airsim_interfaces::msg::GPSYaw msg;
    msg.latitude  = geo_point.latitude;
    msg.longitude = geo_point.longitude;
    msg.altitude  = geo_point.altitude;
    return msg;
}

sensor_msgs::msg::NavSatFix AirsimROSWrapperMultiAgent::get_gps_sensor_msg_from_airsim_geo_point(const msr::airlib::GeoPoint& geo_point) const
{
    sensor_msgs::msg::NavSatFix msg;
    msg.latitude  = geo_point.latitude;
    msg.longitude = geo_point.longitude;
    msg.altitude  = geo_point.altitude;
    return msg;
}

sensor_msgs::msg::Imu AirsimROSWrapperMultiAgent::get_imu_msg_from_airsim(const msr::airlib::ImuBase::Output& imu_data) const
{
    sensor_msgs::msg::Imu msg;
    msg.header.stamp     = rclcpp::Time(imu_data.time_stamp);
    msg.orientation.x    = imu_data.orientation.inverse().x();
    msg.orientation.y    = imu_data.orientation.inverse().y();
    msg.orientation.z    = imu_data.orientation.inverse().z();
    msg.orientation.w    = imu_data.orientation.inverse().w();
    msg.angular_velocity.x  =  imu_data.angular_velocity.x();
    msg.angular_velocity.y  = -imu_data.angular_velocity.y();
    msg.angular_velocity.z  = -imu_data.angular_velocity.z();
    msg.linear_acceleration.x  =  imu_data.linear_acceleration.x();
    msg.linear_acceleration.y  = -imu_data.linear_acceleration.y();
    msg.linear_acceleration.z  = -imu_data.linear_acceleration.z();
    return msg;
}

airsim_interfaces::msg::Altimeter AirsimROSWrapperMultiAgent::get_altimeter_msg_from_airsim(const msr::airlib::BarometerBase::Output& alt_data) const
{
    airsim_interfaces::msg::Altimeter msg;
    msg.header.stamp = rclcpp::Time(alt_data.time_stamp);
    msg.altitude     = alt_data.altitude;
    msg.pressure     = alt_data.pressure;
    msg.qnh          = alt_data.qnh;
    return msg;
}

sensor_msgs::msg::Range AirsimROSWrapperMultiAgent::get_range_from_airsim(const msr::airlib::DistanceSensorData& dist_data) const
{
    sensor_msgs::msg::Range msg;
    msg.header.stamp = rclcpp::Time(dist_data.time_stamp);
    msg.range        = dist_data.distance;
    msg.min_range    = dist_data.min_distance;
    msg.max_range    = dist_data.max_distance;
    return msg;
}

sensor_msgs::msg::NavSatFix AirsimROSWrapperMultiAgent::get_gps_msg_from_airsim(const msr::airlib::GpsBase::Output& gps_data) const
{
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp  = rclcpp::Time(gps_data.time_stamp);
    msg.latitude      = gps_data.gnss.geo_point.latitude;
    msg.longitude     = gps_data.gnss.geo_point.longitude;
    msg.altitude      = gps_data.gnss.geo_point.altitude;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
    msg.status.status  = gps_data.gnss.fix_type;
    return msg;
}

sensor_msgs::msg::MagneticField AirsimROSWrapperMultiAgent::get_mag_msg_from_airsim(const msr::airlib::MagnetometerBase::Output& mag_data) const
{
    sensor_msgs::msg::MagneticField msg;
    msg.magnetic_field.x = mag_data.magnetic_field_body.x();
    msg.magnetic_field.y = mag_data.magnetic_field_body.y();
    msg.magnetic_field.z = mag_data.magnetic_field_body.z();
    std::copy(std::begin(mag_data.magnetic_field_covariance), std::end(mag_data.magnetic_field_covariance), std::begin(msg.magnetic_field_covariance));
    msg.header.stamp = rclcpp::Time(mag_data.time_stamp);
    return msg;
}

airsim_interfaces::msg::Environment AirsimROSWrapperMultiAgent::get_environment_msg_from_airsim(const msr::airlib::Environment::State& env_data) const
{
    airsim_interfaces::msg::Environment msg;
    msg.position.x        = env_data.position.x();
    msg.position.y        = env_data.position.y();
    msg.position.z        = env_data.position.z();
    msg.geo_point.latitude  = env_data.geo_point.latitude;
    msg.geo_point.longitude = env_data.geo_point.longitude;
    msg.geo_point.altitude  = env_data.geo_point.altitude;
    msg.gravity.x         = env_data.gravity.x();
    msg.gravity.y         = env_data.gravity.y();
    msg.gravity.z         = env_data.gravity.z();
    msg.air_pressure      = env_data.air_pressure;
    msg.temperature       = env_data.temperature;
    msg.air_density       = env_data.temperature;
    return msg;
}

msr::airlib::GeoPoint AirsimROSWrapperMultiAgent::get_origin_geo_point() const
{
    return AirSimSettings::singleton().origin_geopoint.home_geo_point;
}

VelCmdMA AirsimROSWrapperMultiAgent::get_airlib_world_vel_cmd(const airsim_interfaces::msg::VelCmd& msg) const
{
    VelCmdMA vel_cmd;
    vel_cmd.x = msg.twist.linear.x;
    vel_cmd.y = msg.twist.linear.y;
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.drivetrain           = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd.yaw_mode.is_rate     = true;
    vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);
    return vel_cmd;
}

VelCmdMA AirsimROSWrapperMultiAgent::get_airlib_body_vel_cmd(const airsim_interfaces::msg::VelCmd& msg, const msr::airlib::Quaternionr& airlib_quat) const
{
    VelCmdMA vel_cmd;
    double roll, pitch, yaw;
    tf2::Matrix3x3(get_tf2_quat(airlib_quat)).getRPY(roll, pitch, yaw);
    vel_cmd.x = (msg.twist.linear.x * cos(yaw)) - (msg.twist.linear.y * sin(yaw));
    vel_cmd.y = (msg.twist.linear.x * sin(yaw)) + (msg.twist.linear.y * cos(yaw));
    vel_cmd.z = msg.twist.linear.z;
    vel_cmd.drivetrain           = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;
    vel_cmd.yaw_mode.is_rate     = true;
    vel_cmd.yaw_mode.yaw_or_rate = math_common::rad2deg(msg.twist.angular.z);
    return vel_cmd;
}

geometry_msgs::msg::Transform AirsimROSWrapperMultiAgent::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::AirSimSettings::Rotation& rotation)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    tf2::Quaternion q;
    q.setRPY(rotation.roll * (M_PI / 180.0), rotation.pitch * (M_PI / 180.0), rotation.yaw * (M_PI / 180.0));
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.rotation.w = q.w();
    return transform;
}

geometry_msgs::msg::Transform AirsimROSWrapperMultiAgent::get_transform_msg_from_airsim(const msr::airlib::Vector3r& position, const msr::airlib::Quaternionr& quaternion)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    transform.rotation.x    = quaternion.x();
    transform.rotation.y    = quaternion.y();
    transform.rotation.z    = quaternion.z();
    transform.rotation.w    = quaternion.w();
    return transform;
}

geometry_msgs::msg::Transform AirsimROSWrapperMultiAgent::get_camera_optical_tf_from_body_tf(const geometry_msgs::msg::Transform& body_tf) const
{
    geometry_msgs::msg::Transform optical_tf = body_tf;
    auto q = msr::airlib::Quaternionr(optical_tf.rotation.w, optical_tf.rotation.x, optical_tf.rotation.y, optical_tf.rotation.z);
    q *= msr::airlib::Quaternionr(0.5, -0.5, 0.5, -0.5);
    optical_tf.rotation.w = q.w();
    optical_tf.rotation.x = q.x();
    optical_tf.rotation.y = q.y();
    optical_tf.rotation.z = q.z();
    return optical_tf;
}

// ============================================================================
// PointCloud helpers
// ============================================================================

static void fixPointCloudMA(std::vector<float>& data, int offset, std::vector<int> flip_indexes)
{
    for (size_t i = 1; i < data.size(); i += offset) {
        data[i] = -data[i];
        for (int fi : flip_indexes)
            if (i + fi < data.size())
                data[i + fi] = -data[i + fi];
    }
}

sensor_msgs::msg::PointCloud2 AirsimROSWrapperMultiAgent::get_lidar_msg_from_airsim(const msr::airlib::LidarData& lidar_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp    = rclcpp::Time(lidar_data.time_stamp);
    msg.header.frame_id = vehicle_name + "/" + sensor_name;

    if (lidar_data.point_cloud.size() > 3) {
        msg.height = 1;
        msg.width  = lidar_data.point_cloud.size() / 3;
        msg.fields.resize(3);
        msg.fields[0].name = "x"; msg.fields[1].name = "y"; msg.fields[2].name = "z";
        int offset = 0;
        for (size_t d = 0; d < msg.fields.size(); ++d, offset += 4) {
            msg.fields[d].offset   = offset;
            msg.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg.fields[d].count    = 1;
        }
        msg.is_bigendian = false;
        msg.point_step   = offset;
        msg.row_step     = msg.point_step * msg.width;
        msg.is_dense     = true;
        std::vector<float> data = lidar_data.point_cloud;
        fixPointCloudMA(data, 3, {1});
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data.data());
        msg.data = std::vector<unsigned char>(bytes, bytes + sizeof(float) * data.size());
    }
    return msg;
}

airsim_interfaces::msg::StringArray AirsimROSWrapperMultiAgent::get_lidar_labels_msg_from_airsim(const msr::airlib::LidarData& lidar_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    airsim_interfaces::msg::StringArray msg;
    msg.header.stamp    = rclcpp::Time(lidar_data.time_stamp);
    msg.header.frame_id = vehicle_name + "/" + sensor_name;
    if (lidar_data.point_cloud.size() > 3)
        msg.data = std::move(lidar_data.groundtruth);
    return msg;
}

sensor_msgs::msg::PointCloud2 AirsimROSWrapperMultiAgent::get_gpulidar_msg_from_airsim(const msr::airlib::GPULidarData& gpulidar_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp    = rclcpp::Time(gpulidar_data.time_stamp);
    msg.header.frame_id = vehicle_name + "/" + sensor_name;

    if (gpulidar_data.point_cloud.size() > 5) {
        std::vector<float> data = gpulidar_data.point_cloud;
        fixPointCloudMA(data, 5, {1});
        size_t num_points = data.size() / 5;
        pcl::PointCloud<PointXYZRGBI_MA> cloud;
        cloud.points.resize(num_points);
        cloud.width   = static_cast<uint32_t>(num_points);
        cloud.height  = 1;
        cloud.is_dense = true;
        for (size_t i = 0; i < num_points; ++i) {
            auto& pt = cloud.points[i];
            pt.x = data[i * 5 + 0]; pt.y = data[i * 5 + 1]; pt.z = data[i * 5 + 2];
            auto rgb_packed = static_cast<std::uint32_t>(data[i * 5 + 3]);
            std::uint32_t r = (rgb_packed >> 16) & 0xFF;
            std::uint32_t g = (rgb_packed >> 8)  & 0xFF;
            std::uint32_t b =  rgb_packed         & 0xFF;
            std::uint32_t rgb = (r << 16) | (g << 8) | b;
            pt.rgb       = *reinterpret_cast<float*>(&rgb);
            pt.intensity = data[i * 5 + 4];
        }
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp    = rclcpp::Time(gpulidar_data.time_stamp);
        msg.header.frame_id = vehicle_name + "/" + sensor_name;
    }
    return msg;
}

sensor_msgs::msg::PointCloud2 AirsimROSWrapperMultiAgent::get_active_echo_msg_from_airsim(const msr::airlib::EchoData& echo_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp    = rclcpp::Time(echo_data.time_stamp);
    msg.header.frame_id = vehicle_name + "/" + sensor_name;
    if (echo_data.point_cloud.size() > 6) {
        msg.height = 1;
        msg.width  = echo_data.point_cloud.size() / 6;
        msg.fields.resize(6);
        const char* names[] = {"x","y","z","a","d","r"};
        int offset = 0;
        for (size_t d = 0; d < 6; ++d, offset += 4) {
            msg.fields[d].name = names[d]; msg.fields[d].offset = offset;
            msg.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32; msg.fields[d].count = 1;
        }
        msg.is_bigendian = false; msg.point_step = offset; msg.row_step = offset * msg.width; msg.is_dense = true;
        std::vector<float> data = echo_data.point_cloud;
        fixPointCloudMA(data, 6, {1});
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data.data());
        msg.data = std::vector<unsigned char>(bytes, bytes + sizeof(float) * data.size());
    }
    return msg;
}

airsim_interfaces::msg::StringArray AirsimROSWrapperMultiAgent::get_active_echo_labels_msg_from_airsim(const msr::airlib::EchoData& echo_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    airsim_interfaces::msg::StringArray msg;
    msg.header.stamp    = rclcpp::Time(echo_data.time_stamp);
    msg.header.frame_id = vehicle_name + "/" + sensor_name;
    if (echo_data.point_cloud.size() > 6)
        msg.data = std::move(echo_data.groundtruth);
    return msg;
}

sensor_msgs::msg::PointCloud2 AirsimROSWrapperMultiAgent::get_passive_echo_msg_from_airsim(const msr::airlib::EchoData& echo_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp    = rclcpp::Time(echo_data.time_stamp);
    msg.header.frame_id = vehicle_name + "/" + sensor_name;
    if (echo_data.passive_beacons_point_cloud.size() > 9) {
        msg.height = 1;
        msg.width  = echo_data.passive_beacons_point_cloud.size() / 9;
        msg.fields.resize(9);
        const char* names[] = {"x","y","z","a","d","r","xd","yd","zd"};
        int offset = 0;
        for (size_t d = 0; d < 9; ++d, offset += 4) {
            msg.fields[d].name = names[d]; msg.fields[d].offset = offset;
            msg.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32; msg.fields[d].count = 1;
        }
        msg.is_bigendian = false; msg.point_step = offset; msg.row_step = offset * msg.width; msg.is_dense = true;
        std::vector<float> data = echo_data.passive_beacons_point_cloud;
        fixPointCloudMA(data, 9, {1, 6, 7});
        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data.data());
        msg.data = std::vector<unsigned char>(bytes, bytes + sizeof(float) * data.size());
    }
    return msg;
}

airsim_interfaces::msg::StringArray AirsimROSWrapperMultiAgent::get_passive_echo_labels_msg_from_airsim(const msr::airlib::EchoData& echo_data, const std::string& vehicle_name, const std::string& sensor_name) const
{
    airsim_interfaces::msg::StringArray msg;
    msg.header.stamp    = rclcpp::Time(echo_data.time_stamp);
    msg.header.frame_id = vehicle_name + "/" + sensor_name;
    if (echo_data.passive_beacons_point_cloud.size() > 9)
        msg.data = std::move(echo_data.passive_beacons_groundtruth);
    return msg;
}

// ============================================================================
// Instance segmentation / object transforms
// ============================================================================

airsim_interfaces::msg::InstanceSegmentationList AirsimROSWrapperMultiAgent::get_instance_segmentation_list_msg_from_airsim() const
{
    airsim_interfaces::msg::InstanceSegmentationList msg;
    auto names     = multirotor_client_->simListInstanceSegmentationObjects();
    auto color_map = multirotor_client_->simGetInstanceSegmentationColorMap();
    int idx = 0;
    for (auto it = names.begin(); it != names.end(); ++it, ++idx) {
        airsim_interfaces::msg::InstanceSegmentationLabel label;
        label.name  = *it;
        label.r     = color_map[idx].x();
        label.g     = color_map[idx].y();
        label.b     = color_map[idx].z();
        label.index = idx;
        msg.labels.push_back(label);
    }
    return msg;
}

airsim_interfaces::msg::ObjectTransformsList AirsimROSWrapperMultiAgent::get_object_transforms_list_msg_from_airsim(rclcpp::Time timestamp) const
{
    airsim_interfaces::msg::ObjectTransformsList msg;
    auto names = multirotor_client_->simListInstanceSegmentationObjects();
    auto poses = multirotor_client_->simListInstanceSegmentationPoses();
    int idx = 0;
    for (auto it = names.begin(); it != names.end(); ++it, ++idx) {
        const auto& pose = poses[idx];
        if (std::isnan(pose.position.x())) continue;
        geometry_msgs::msg::TransformStamped tf;
        tf.child_frame_id = *it;
        tf.transform.translation.x = pose.position.x();
        tf.transform.translation.y = -pose.position.y();
        tf.transform.translation.z = -pose.position.z();
        tf.transform.rotation.x    = pose.orientation.inverse().x();
        tf.transform.rotation.y    = pose.orientation.inverse().y();
        tf.transform.rotation.z    = pose.orientation.inverse().z();
        tf.transform.rotation.w    = pose.orientation.inverse().w();
        tf.header.stamp = timestamp;
        msg.objects.push_back(tf);
    }
    msg.header.stamp    = timestamp;
    msg.header.frame_id = world_frame_id_;
    return msg;
}

// ============================================================================
// Camera info / image processing
// ============================================================================

// CameraInfo is published PER CAMERA MODEL - Phase 3b step 6, design doc section 7.3.
//
// The message's geometry fields are K (a 3x3 pinhole matrix), D (coefficients) and a
// distortion_model string naming which model D is in. Every model in that vocabulary shares one
// structure: divide by z, distort, multiply by K - "a pinhole camera plus a correction". That
// structure fails outright at wide field of view, and DOUBLE SPHERE IS NOT OF THAT FORM at all:
// it projects onto a unit sphere, then onto a second sphere displaced by xi, then through a
// pinhole-like step blended by alpha. There is no K + D decomposition to extract and no string
// for it.
//
// The danger is that the failure is SILENT. Publish "equidistant" with coefficients fitted to
// approximate a Double Sphere lens and every downstream node accepts it, computes subtly wrong
// rays, triangulation and point clouds, and nothing errors - it surfaces weeks later as an
// unexplained bias in a SLAM result. So for a model we cannot express we publish the one thing
// in the vocabulary that means "this camera is not calibrated": all matrices ZEROED. A consumer
// that needs calibration then fails immediately and visibly. depth_image_proc refusing to build
// a point cloud out of a fisheye depth image is the CORRECT outcome; it genuinely cannot.
//
// The authoritative calibration goes out on the <camera>/camera_model topic instead - see
// generate_camera_model_json. That is the real interface, and it is what the tools which do
// support these models (SaDVIO, Basalt, OpenVINS, Kalibr) read anyway.
sensor_msgs::msg::CameraInfo AirsimROSWrapperMultiAgent::generate_cam_info(const std::string& camera_name,
                                                                           const CameraSetting& camera_setting,
                                                                           const CaptureSetting& capture_setting) const
{
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = camera_name + "_optical";
    info.height = capture_setting.height;
    info.width  = capture_setting.width;

    // No CameraModel block: this is every camera that existed before Phase 3b, and the message
    // below must stay byte-identical to what it published then. Do not "improve" it here.
    if (!camera_setting.camera_model.enabled) {
        float f_x = (capture_setting.width / 2.0f) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0f));
        info.k = { f_x, 0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 1.0 };
        info.p = { f_x, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
        info.d = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        info.r = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
        return info;
    }

    const auto& model = camera_setting.camera_model.model;

    // The cube path is REFUSED when the model's size and the capture size disagree
    // (APIPCamera::buildRaymapResource), and the camera then renders as an ordinary pinhole. We
    // cannot tell from here which happened, so a mismatch means we do not know what is on this
    // topic - which is exactly the uncalibrated case. Saying nothing beats saying something
    // plausible.
    const bool size_matches = model.width == capture_setting.width &&
                              model.height == capture_setting.height;
    const bool intrinsics_known = size_matches && std::isfinite(model.fx) && std::isfinite(model.fy) &&
                                  std::isfinite(model.cx) && std::isfinite(model.cy);

    if (intrinsics_known && model.type == msr::airlib::cameras::CameraModelType::Pinhole) {
        // Exact. Note this is NOT identical to the branch above: D12 fixes the principal point at
        // (W-1)/2 because integer pixel coordinates are pixel CENTRES, which is the convention
        // every calibration we hold is written in. The old form uses W/2 - half a pixel out.
        info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        info.k = { model.fx, 0.0, model.cx, 0.0, model.fy, model.cy, 0.0, 0.0, 1.0 };
        info.p = { model.fx, 0.0, model.cx, 0.0, 0.0, model.fy, model.cy, 0.0, 0.0, 0.0, 1.0, 0.0 };
        info.d = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        info.r = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
    }
    else if (intrinsics_known && model.type == msr::airlib::cameras::CameraModelType::KannalaBrandt) {
        // Also exact, not an approximation: "equidistant" IS Kannala-Brandt, k1..k4 in this order.
        // The caveat is the consumer's, not ours - image_pipeline still divides by z, so a >=180
        // degree lens breaks it however right the coefficients are.
        info.distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
        info.k = { model.fx, 0.0, model.cx, 0.0, model.fy, model.cy, 0.0, 0.0, 1.0 };
        info.p = { model.fx, 0.0, model.cx, 0.0, 0.0, model.fy, model.cy, 0.0, 0.0, 0.0, 1.0, 0.0 };
        info.d = { model.k1, model.k2, model.k3, model.k4 };
        info.r = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
    }
    else {
        // DoubleSphere, Raymap, or a misconfigured camera. Zeroed on purpose - see the header
        // comment. Width and height stay correct: the image size is not in dispute.
        info.distortion_model = "";
        info.k.fill(0.0);
        info.p.fill(0.0);
        info.r.fill(0.0);
        info.d.clear();
    }

    return info;
}

// The authoritative calibration of a generic camera, as the settings JSON that produced it.
//
// Design section 7.3 resolution point 1: ship the real calibration in the format the tools which
// support these models already read, and treat CameraInfo as the approximate one. On a ROS graph
// the equivalent of "ships with the dataset" is a LATCHED topic, because it lands in the bag.
//
// The contract is that a user can paste this straight back under a camera's "CameraModel" key and
// get the same camera - which is a testable claim, not a hope: feed it to tools/raymap_dump and
// the raymap must come back bit-identical. Hence full double precision, and hence no NaN fields:
// a value the settings parser would reject must never appear here.
std::string AirsimROSWrapperMultiAgent::generate_camera_model_json(const CameraSetting& camera_setting) const
{
    const auto& setting = camera_setting.camera_model;
    const auto& model   = setting.model;

    std::ostringstream json;
    json << std::setprecision(17);
    json << "{\"Type\": \"" << msr::airlib::cameras::toString(model.type) << "\"";
    json << ", \"Width\": " << model.width << ", \"Height\": " << model.height;

    // resolveParams has already filled these in, including for a Pinhole written as FOV_Degrees,
    // so the payload is self-contained rather than a copy of whatever shorthand the user typed.
    if (std::isfinite(model.fx)) json << ", \"fx\": " << model.fx;
    if (std::isfinite(model.fy)) json << ", \"fy\": " << model.fy;
    if (std::isfinite(model.cx)) json << ", \"cx\": " << model.cx;
    if (std::isfinite(model.cy)) json << ", \"cy\": " << model.cy;

    if (model.type == msr::airlib::cameras::CameraModelType::KannalaBrandt) {
        json << ", \"k1\": " << model.k1 << ", \"k2\": " << model.k2
             << ", \"k3\": " << model.k3 << ", \"k4\": " << model.k4;
    }
    if (model.type == msr::airlib::cameras::CameraModelType::DoubleSphere) {
        json << ", \"xi\": " << model.xi << ", \"alpha\": " << model.alpha;
    }
    if (model.type == msr::airlib::cameras::CameraModelType::Raymap) {
        // escaped, because a path is the one field here that can carry a character which would
        // otherwise end the string and hand a consumer invalid JSON
        std::string path;
        for (char c : model.raymap_path) {
            if (c == '\\' || c == '"')
                path += '\\';
            path += c;
        }
        json << ", \"Path\": \"" << path << "\"";
    }

    json << ", \"CubeFaceResolution\": " << setting.cube_face_resolution;
    if (setting.faces == 0)
        json << ", \"Faces\": \"Auto\"";
    else
        json << ", \"Faces\": " << setting.faces;
    json << ", \"RenderBackend\": \""
         << (setting.render_backend == AirSimSettings::CameraModelSetting::RenderBackend::NativeGEER
                 ? "NativeGEER"
                 : "Cube")
         << "\"";
    //Retained for consumers of the schema reserved before RenderBackend was introduced.
    json << ", \"SplatOnly\": " << (setting.splat_only ? "true" : "false");
    json << "}";
    return json.str();
}

std::shared_ptr<sensor_msgs::msg::Image> AirsimROSWrapperMultiAgent::get_img_msg_from_response(const ImageResponse& img_response, const rclcpp::Time curr_ros_time, const std::string frame_id)
{
    unused(curr_ros_time);
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->data     = img_response.image_data_uint8;
    msg->step     = img_response.image_data_uint8.size() / img_response.height;
    msg->header.stamp    = rclcpp::Time(img_response.time_stamp);
    msg->header.frame_id = frame_id;
    msg->height   = img_response.height;
    msg->width    = img_response.width;
    msg->encoding = is_vulkan_ ? "rgb8" : "bgr8";
    msg->is_bigendian = 0;
    return msg;
}

// DEPTH SEMANTICS, and they are NOT the same for every camera - Phase 3b steps 5 and 6.
//
// On a camera with a CameraModel block:
//   * the value is RANGE ALONG THE RAY, in METRES, not planar depth. DepthPlanar has no meaning
//     without an image plane, so both DepthPlanar and DepthPerspective publish the same image -
//     bit-identical, deliberately. A consumer that assumes planar depth is wrong.
//   * a pixel outside the lens' valid domain publishes NaN (REP 117: unknown). On a wide Double
//     Sphere that is ~8% of the frame, in the corners. It used to publish 0.0, which reads as a
//     surface touching the lens.
//
// On EVERY camera, generic or not, "no geometry" still publishes a large finite value (~16400)
// where REP 117 asks for +Inf. That is pre-existing base-AirSim behaviour, not something the
// generic camera introduced, and changing it touches every existing user - it is open decision
// O6 in the project plan.
//
// The image is not mis-scaled and does not need converting. If it looks entirely white, that is
// the viewer: most tools map 32FC1 to [0, 1] without normalising, so everything past 1 m
// saturates.
std::shared_ptr<sensor_msgs::msg::Image> AirsimROSWrapperMultiAgent::get_depth_img_msg_from_response(const ImageResponse& img_response, const rclcpp::Time curr_ros_time, const std::string frame_id)
{
    unused(curr_ros_time);
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->width  = img_response.width;
    msg->height = img_response.height;
    msg->data.resize(img_response.image_data_float.size() * sizeof(float));
    memcpy(msg->data.data(), img_response.image_data_float.data(), msg->data.size());
    msg->encoding     = "32FC1";
    msg->step         = msg->data.size() / img_response.height;
    msg->is_bigendian = 0;
    msg->header.stamp    = rclcpp::Time(img_response.time_stamp);
    msg->header.frame_id = frame_id;
    return msg;
}

void AirsimROSWrapperMultiAgent::process_and_publish_img_response(const std::vector<ImageResponse>& img_response_vec, const int img_response_idx, const std::string& vehicle_name)
{
    rclcpp::Time curr_ros_time = nh_->now();
    int idx = img_response_idx;
    for (const auto& curr_img_response : img_response_vec) {
        camera_info_msg_vec_[idx].header.stamp = rclcpp::Time(curr_img_response.time_stamp);
        cam_info_pub_vec_[idx]->publish(camera_info_msg_vec_[idx]);
        if (curr_img_response.pixels_as_float) {
            image_pub_vec_[idx].publish(get_depth_img_msg_from_response(curr_img_response, curr_ros_time, vehicle_name + "/" + curr_img_response.camera_name + "_optical"));
        } else {
            image_pub_vec_[idx].publish(get_img_msg_from_response(curr_img_response, curr_ros_time, vehicle_name + "/" + curr_img_response.camera_name + "_optical"));
        }
        ++idx;
    }
}

// ============================================================================
// NaN-zeroing helpers
// ============================================================================

void AirsimROSWrapperMultiAgent::set_nans_to_zeros_in_pose(VehicleSetting& vs) const
{
    if (std::isnan(vs.position.x())) vs.position.x() = 0.0;
    if (std::isnan(vs.position.y())) vs.position.y() = 0.0;
    if (std::isnan(vs.position.z())) vs.position.z() = 0.0;
    if (std::isnan(vs.rotation.yaw))   vs.rotation.yaw   = 0.0;
    if (std::isnan(vs.rotation.pitch)) vs.rotation.pitch = 0.0;
    if (std::isnan(vs.rotation.roll))  vs.rotation.roll  = 0.0;
}

void AirsimROSWrapperMultiAgent::set_nans_to_zeros_in_pose(const VehicleSetting& vs, CameraSetting& cs) const
{
    if (std::isnan(cs.position.x())) cs.position.x() = vs.position.x();
    if (std::isnan(cs.position.y())) cs.position.y() = vs.position.y();
    if (std::isnan(cs.position.z())) cs.position.z() = vs.position.z();
    if (std::isnan(cs.rotation.yaw))   cs.rotation.yaw   = vs.rotation.yaw;
    if (std::isnan(cs.rotation.pitch)) cs.rotation.pitch = vs.rotation.pitch;
    if (std::isnan(cs.rotation.roll))  cs.rotation.roll  = vs.rotation.roll;
}


// ============================================================================
// YAML camera calibration (unused in practice but kept for parity)
// ============================================================================

void AirsimROSWrapperMultiAgent::convert_yaml_to_simple_mat(const YAML::Node& node, SimpleMatrix_MA& m) const
{
    int rows = node["rows"].as<int>();
    int cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows * cols; ++i)
        m.data[i] = data[i].as<double>();
}

void AirsimROSWrapperMultiAgent::read_params_from_yaml_and_fill_cam_info_msg(const std::string& file_name, sensor_msgs::msg::CameraInfo& cam_info) const
{
    std::ifstream fin(file_name.c_str());
    YAML::Node doc = YAML::Load(fin);
    cam_info.width  = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();
    SimpleMatrix_MA K_(3, 3, &cam_info.k[0]); convert_yaml_to_simple_mat(doc[K_YML_NAME], K_);
    SimpleMatrix_MA R_(3, 3, &cam_info.r[0]); convert_yaml_to_simple_mat(doc[R_YML_NAME], R_);
    SimpleMatrix_MA P_(3, 4, &cam_info.p[0]); convert_yaml_to_simple_mat(doc[P_YML_NAME], P_);
    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();
    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows = D_node["rows"].as<int>(), D_cols = D_node["cols"].as<int>();
    const YAML::Node& D_data = D_node["data"];
    cam_info.d.resize(D_rows * D_cols);
    for (int i = 0; i < D_rows * D_cols; ++i)
        cam_info.d[i] = D_data[i].as<float>();
}
