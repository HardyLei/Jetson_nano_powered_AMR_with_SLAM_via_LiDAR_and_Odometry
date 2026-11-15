from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    parameters_file = os.path.join(
        get_package_share_directory('amr_control'),
        'config',
        'mecanum_controller.yaml',
    )

    return LaunchDescription([
        Node(
            package='amr_control',
            executable='mecanum_controller',
            name='mecanum_controller',
            output='screen',
            parameters=[parameters_file],
        ),
    ])
