from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="handcarry.yaml",
    )

    recorder_node = Node(
        package="kaist_rosbag",
        executable="rosbag_recorder",
        name="rosbag_recorder",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
            }
        ],
    )

    return LaunchDescription(
        [
            config_file_arg,
            recorder_node,
        ]
    )
