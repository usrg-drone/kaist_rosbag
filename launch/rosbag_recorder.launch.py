import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    data_dir = os.environ.get('DATA_DIR', os.path.expanduser('~/bags'))

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='test.yaml',
        description=(
            'Config file where topics and command line arguments are '
            'specified')
    )

    experiment_name_arg = DeclareLaunchArgument(
        'experiment_name',
        default_value='experiment',
        description=(
            'Experiment name; bag file will be saved in '
            'DATA_DIR/experiment_name')
    )
    save_dir = PathJoinSubstitution([
        data_dir,
        LaunchConfiguration('experiment_name'),
    ])

    recorder_node = Node(
        package='kaist_rosbag',
        executable='rosbag_recorder',
        name='rosbag_recorder',
        output='screen',
        parameters=[{
            'save_dir': save_dir,
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    return LaunchDescription([
        config_file_arg,
        experiment_name_arg,
        recorder_node
    ])
