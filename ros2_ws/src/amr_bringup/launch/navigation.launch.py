import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_share = Path(
        get_package_share_directory('nav2_bringup')
    )
    default_map = os.path.expanduser('~/amr_maps/amr.yaml')
    default_params = str(
        nav2_share / 'params' / 'nav2_params.yaml'
    )

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the saved map YAML file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with the Nav2 display configuration.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    nav2_share
                    / 'launch'
                    / 'bringup_launch.py'
                )
            ),
            launch_arguments={
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': 'false',
                'slam': 'false',
                'use_localization': 'true',
                'serve_static_map': 'true',
                'autostart': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    nav2_share
                    / 'launch'
                    / 'rviz_launch.py'
                )
            ),
            launch_arguments={
                'namespace': '',
                'use_sim_time': 'false',
                'rviz_config': str(
                    nav2_share / 'rviz' / 'nav2_default_view.rviz'
                ),
            }.items(),
            condition=IfCondition(use_rviz),
        ),
    ])
