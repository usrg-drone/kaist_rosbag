from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    topic = LaunchConfiguration('topic')
    record_script = LaunchConfiguration('record_script')
    record_folder = LaunchConfiguration('record_folder')
    verbose = LaunchConfiguration('verbose')

    return LaunchDescription([

        DeclareLaunchArgument(
            'topic',
            default_value='/start_topic'
        ),

        DeclareLaunchArgument(
            'record_script',
            default_value=PathJoinSubstitution([
                FindPackageShare('kros'),
                'config',
                'kros2.sh'
            ])
        ),

        DeclareLaunchArgument(
            'record_folder',
            default_value='~/bags'
        ),

        DeclareLaunchArgument(
            'verbose',
            default_value='false'
        ),

        Node(
            package='kros',
            executable='kros2',
            name='kros',
            output='screen',

            parameters=[{
                'record_script': record_script,
                'record_folder': record_folder,
                'trigger_topic_name': topic,
                'verbose': verbose,
                'rc_channel': 9,
                'rc_threshold': 1500
            }]
        )

    ])
