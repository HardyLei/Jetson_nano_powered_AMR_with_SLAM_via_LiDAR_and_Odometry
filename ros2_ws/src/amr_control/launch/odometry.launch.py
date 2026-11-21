from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

import os


def generate_launch_description():
    parameters_file = os.path.join(
        get_package_share_directory('amr_control'),
        'config',
        'odometry.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('publish_tf', default_value='true'),
        Node(
            package='amr_control',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[parameters_file, {
                'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
            }],
        ),
    ])
