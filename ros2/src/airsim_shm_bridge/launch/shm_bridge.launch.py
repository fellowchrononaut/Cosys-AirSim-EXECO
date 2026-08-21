"""Launch the shared-memory image bridge.

  everything found:
    ros2 launch airsim_shm_bridge shm_bridge.launch.py

  a subset - the allowlist is what makes this selectable per topic or per robot:
    ros2 launch airsim_shm_bridge shm_bridge.launch.py \
        topics:="Go2_1/fisheye/0,Go2_1/fisheye/5,Go2_1/fisheye/1"

  The allowlist is matched exactly and there is no wildcard: a wildcard that matched nothing would
  look identical to a sim that is not publishing, and that is the failure this node exists to make
  visible rather than silent.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("stream_dir", default_value="/dev/shm"),
        DeclareLaunchArgument(
            "topics", default_value="",
            description="Comma-separated allowlist of vehicle/camera/imagetype. "
                        "Empty = publish everything found."),
        DeclareLaunchArgument("topic_prefix", default_value="/airsim_shm"),
        DeclareLaunchArgument("poll_sec", default_value="0.002"),
        DeclareLaunchArgument("rescan_sec", default_value="2.0"),
        DeclareLaunchArgument(
            "reliable", default_value="false",
            description="RELIABLE QoS instead of BEST_EFFORT. Images default to BEST_EFFORT "
                        "because RELIABLE delivery of ~1 MB messages collapses the rate "
                        "(8 Hz -> 0.4 Hz, measured). In rviz2 set the Image display's Reliability "
                        "Policy to Best Effort to see them."),
        DeclareLaunchArgument("report_sec", default_value="5.0"),
        DeclareLaunchArgument("frame_id_prefix", default_value=""),
    ]

    node = Node(
        package="airsim_shm_bridge",
        executable="shm_bridge",
        name="airsim_shm_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "stream_dir": ParameterValue(LaunchConfiguration("stream_dir"), value_type=str),
            "topics": ParameterValue(LaunchConfiguration("topics"), value_type=str),
            "topic_prefix": ParameterValue(LaunchConfiguration("topic_prefix"), value_type=str),
            "poll_sec": ParameterValue(LaunchConfiguration("poll_sec"), value_type=float),
            "rescan_sec": ParameterValue(LaunchConfiguration("rescan_sec"), value_type=float),
            "reliable": ParameterValue(LaunchConfiguration("reliable"), value_type=bool),
            "report_sec": ParameterValue(LaunchConfiguration("report_sec"), value_type=float),
            "frame_id_prefix": ParameterValue(LaunchConfiguration("frame_id_prefix"),
                                              value_type=str),
        }],
    )

    return LaunchDescription(args + [node])
