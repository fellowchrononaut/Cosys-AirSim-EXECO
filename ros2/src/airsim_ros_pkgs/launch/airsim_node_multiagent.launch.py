import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    output = DeclareLaunchArgument(
        "output",
        default_value='screen')

    publish_clock = DeclareLaunchArgument(
        "publish_clock",
        default_value='False')

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
                'host_ip': LaunchConfiguration('host_ip'),
                'enable_api_control': LaunchConfiguration('enable_api_control'),
                'enable_object_transforms_list': LaunchConfiguration('enable_object_transforms_list')
            }])

    ld = LaunchDescription()

    ld.add_action(output)
    ld.add_action(publish_clock)
    ld.add_action(is_vulkan)
    ld.add_action(host_ip)
    ld.add_action(enable_api_control)
    ld.add_action(enable_object_transforms_list)
    ld.add_action(airsim_node_multiagent)

    return ld
