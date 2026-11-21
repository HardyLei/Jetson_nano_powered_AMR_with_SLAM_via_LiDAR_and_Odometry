"""Run saved-map navigation against an already running robot."""

import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Keep the standalone navigation arguments as aliases for autonomy."""
    share = get_package_share_path('amr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/amr_maps/amr.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config' / 'nav2.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_ekf', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'autonomy.launch.py')),
            launch_arguments={
                'mode': 'navigation',
                'start_robot': 'false',
                'explore': 'false',
                'map': LaunchConfiguration('map'),
                'nav2_params_file': LaunchConfiguration('params_file'),
                'rviz': LaunchConfiguration('use_rviz'),
                'use_ekf': LaunchConfiguration('use_ekf'),
            }.items(),
        ),
    ])
