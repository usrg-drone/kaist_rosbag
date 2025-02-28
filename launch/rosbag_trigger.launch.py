import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('record_script', default_value=os.path.expanduser('~/glim_ws/src/kaist_rosbag/config/record_topics.sh'),
                              description='Shell script to start rosbag recording'),
        DeclareLaunchArgument('trigger_topic_name', default_value='/mavros/state',
                              description='Topic to trigger recording'),

        launch_ros.actions.Node(
            package='kaist_rosbag',
            executable='rosbag_trigger',
            name='rosbag_trigger',
            output='screen',
            parameters=[{
                'record_script': LaunchConfiguration('record_script'),
                'trigger_topic_name': LaunchConfiguration('trigger_topic_name')
            }]
        ),
    ])
