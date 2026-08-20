import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    output = DeclareLaunchArgument(
        "output",
        default_value='screen')

    # Default ON. Every message this node publishes is stamped from AirSim's SteppableClock, so
    # without /clock the graph runs on wall time while the data is in sim time, and the two drift
    # apart by the sim/wall deficit. Consumers still need use_sim_time:=true to honour it. See I-J.
    publish_clock = DeclareLaunchArgument(
        "publish_clock",
        default_value='True')

    # D9 fleet-synchronous capture. Default ON: per-vehicle capture leaves a multi-robot recording
    # with no common instant (measured 189 ms of fleet spread vs 0.000 ms batched). ⚠ It costs a
    # larger worst-case frame stall (render p95 66 ms -> 234 ms), so set false when frame pacing
    # matters more than a shared capture instant.
    batch_image_capture = DeclareLaunchArgument(
        "batch_image_capture",
        default_value='True')

    is_vulkan = DeclareLaunchArgument(
        "is_vulkan",
        default_value='True')

    host_ip = DeclareLaunchArgument(
        "host_ip",
        default_value='localhost')

    enable_api_control = DeclareLaunchArgument(
        "enable_api_control",
        default_value='False')

    enable_object_transforms_list = DeclareLaunchArgument(
        "enable_object_transforms_list",
        default_value='True')

    airsim_node_multiagent = Node(
            package='airsim_ros_pkgs',
            executable='airsim_node_multiagent',
            name='airsim_node',
            output=LaunchConfiguration('output'),
            parameters=[{
                'is_vulkan': LaunchConfiguration('is_vulkan'),
                'update_airsim_img_response_every_n_sec': 0.2,
                'update_airsim_control_every_n_sec': 0.01,
                'update_lidar_every_n_sec': 0.01,
                'update_gpulidar_every_n_sec': 0.01,
                'update_echo_every_n_sec': 0.01,
                'publish_clock': LaunchConfiguration('publish_clock'),
                'batch_image_capture': LaunchConfiguration('batch_image_capture'),
                'host_ip': LaunchConfiguration('host_ip'),
                'enable_api_control': LaunchConfiguration('enable_api_control'),
                'enable_object_transforms_list': LaunchConfiguration('enable_object_transforms_list')
            }])

    ld = LaunchDescription()

    ld.add_action(output)
    ld.add_action(publish_clock)
    ld.add_action(batch_image_capture)
    ld.add_action(is_vulkan)
    ld.add_action(host_ip)
    ld.add_action(enable_api_control)
    ld.add_action(enable_object_transforms_list)
    ld.add_action(airsim_node_multiagent)

    return ld
