#include <airsim_ros_wrapper_multiagent.h>
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
    // Dedicated image connections — one per server
    , airsim_client_images_drone_(host_ip, DRONE_PORT)
    , airsim_client_images_car_(host_ip, CAR_PORT)
    , airsim_client_images_cv_(host_ip, CV_PORT)
    // Dedicated lidar connections
    , airsim_client_lidar_drone_(host_ip, DRONE_PORT)
    , airsim_client_lidar_car_(host_ip, CAR_PORT)
    , airsim_client_lidar_cv_(host_ip, CV_PORT)
    // Dedicated GPU-lidar connections
    , airsim_client_gpulidar_drone_(host_ip, DRONE_PORT)
    , airsim_client_gpulidar_car_(host_ip, CAR_PORT)
    , airsim_client_gpulidar_cv_(host_ip, CV_PORT)
    // Dedicated echo connections
    , airsim_client_echo_drone_(host_ip, DRONE_PORT)
    , airsim_client_echo_car_(host_ip, CAR_PORT)
    , airsim_client_echo_cv_(host_ip, CV_PORT)
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

        multirotor_client_->confirmConnection();
        car_client_->confirmConnection();
        cv_client_->confirmConnection();

        airsim_client_images_drone_.confirmConnection();
        airsim_client_images_car_.confirmConnection();
        airsim_client_images_cv_.confirmConnection();

        airsim_client_lidar_drone_.confirmConnection();
        airsim_client_lidar_car_.confirmConnection();
        airsim_client_lidar_cv_.confirmConnection();

        airsim_client_gpulidar_drone_.confirmConnection();
        airsim_client_gpulidar_car_.confirmConnection();
        airsim_client_gpulidar_cv_.confirmConnection();

        airsim_client_echo_drone_.confirmConnection();
        airsim_client_echo_car_.confirmConnection();
        airsim_client_echo_cv_.confirmConnection();

        if (enable_api_control_) {
            for (const auto& vehicle_name_ptr_pair : vehicle_name_ptr_map_) {
                const std::string& vname = vehicle_name_ptr_pair.first;
                get_state_client(vehicle_name_ptr_pair.second->vehicle_mode_).enableApiControl(true, vname);
                get_state_client(vehicle_name_ptr_pair.second->vehicle_mode_).armDisarm(true, vname);
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
    }
    return *multirotor_client_;
}

msr::airlib::RpcLibClientBase& AirsimROSWrapperMultiAgent::get_images_client(VehicleMode mode)
{
    switch (mode) {
        case VehicleMode::DRONE: return airsim_client_images_drone_;
        case VehicleMode::CAR:   return airsim_client_images_car_;
        case VehicleMode::CV:    return airsim_client_images_cv_;
    }
    return airsim_client_images_drone_;
}

msr::airlib::RpcLibClientBase& AirsimROSWrapperMultiAgent::get_lidar_client(VehicleMode mode)
{
    switch (mode) {
        case VehicleMode::DRONE: return airsim_client_lidar_drone_;
        case VehicleMode::CAR:   return airsim_client_lidar_car_;
        case VehicleMode::CV:    return airsim_client_lidar_cv_;
    }
    return airsim_client_lidar_drone_;
}

msr::airlib::RpcLibClientBase& AirsimROSWrapperMultiAgent::get_gpulidar_client(VehicleMode mode)
{
    switch (mode) {
        case VehicleMode::DRONE: return airsim_client_gpulidar_drone_;
        case VehicleMode::CAR:   return airsim_client_gpulidar_car_;
        case VehicleMode::CV:    return airsim_client_gpulidar_cv_;
    }
    return airsim_client_gpulidar_drone_;
}

msr::airlib::RpcLibClientBase& AirsimROSWrapperMultiAgent::get_echo_client(VehicleMode mode)
{
    switch (mode) {
        case VehicleMode::DRONE: return airsim_client_echo_drone_;
        case VehicleMode::CAR:   return airsim_client_echo_car_;
        case VehicleMode::CV:    return airsim_client_echo_cv_;
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
        }

        vehicle_ros->vehicle_name_    = curr_vehicle_name;
        vehicle_ros->vehicle_mode_    = vmode;
        vehicle_ros->odom_frame_id_   = curr_vehicle_name + "/" + odom_frame_id_;

        append_static_vehicle_tf(vehicle_ros.get(), *vehicle_setting);

        const std::string topic_prefix = "~/" + curr_vehicle_name;

        vehicle_ros->odom_local_pub_  = nh_->create_publisher<nav_msgs::msg::Odometry>(topic_prefix + "/" + odom_frame_id_, 10);
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

    if (publish_clock_)
        clock_pub_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("~/clock", 1);

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
            vehicle_ros->curr_odom_ = get_odom_msg_from_multirotor_state(drone->curr_drone_state_);

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
        }

        vehicle_ros->odom_local_pub_->publish(vehicle_ros->curr_odom_);
        publish_odom_tf(vehicle_ros->curr_odom_);
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

            for (auto& pub : vehicle_ros->lidar_pubs_) {
                auto data  = lidar_client.getLidarData(pub.sensor_name, vname);
                sensor_cache[pub.sensor_name] = data;
                pub.publisher->publish(get_lidar_msg_from_airsim(data, vname, pub.sensor_name));
            }
            for (auto& pub : vehicle_ros->lidar_labels_pubs_) {
                auto it = sensor_cache.find(pub.sensor_name);
                msr::airlib::LidarData data = (it != sensor_cache.end()) ? it->second : lidar_client.getLidarData(pub.sensor_name, vname);
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
    tf_msg.transform        = get_transform_msg_from_airsim(vehicle_setting.position, vehicle_setting.rotation);
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

sensor_msgs::msg::CameraInfo AirsimROSWrapperMultiAgent::generate_cam_info(const std::string& camera_name,
                                                                           const CameraSetting& camera_setting,
                                                                           const CaptureSetting& capture_setting) const
{
    unused(camera_setting);
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = camera_name + "_optical";
    info.height = capture_setting.height;
    info.width  = capture_setting.width;
    float f_x = (capture_setting.width / 2.0f) / tan(math_common::deg2rad(capture_setting.fov_degrees / 2.0f));
    info.k = { f_x, 0.0, capture_setting.width / 2.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 1.0 };
    info.p = { f_x, 0.0, capture_setting.width / 2.0, 0.0, 0.0, f_x, capture_setting.height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
    info.d = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    info.r = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
    return info;
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
