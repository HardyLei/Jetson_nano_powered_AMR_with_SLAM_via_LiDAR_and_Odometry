from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(
        get_package_share_directory('amr_bringup')
    )
    nav2_share = Path(
        get_package_share_directory('nav2_bringup')
    )
    slam_share = Path(
        get_package_share_directory('slam_toolbox')
    )

    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(
                bringup_share / 'config' / 'slam_toolbox.yaml'
            ),
            description='Full path to the SLAM Toolbox parameter file.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 for mapping.',
        ),
        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            description='Start keyboard teleoperation.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    slam_share
                    / 'launch'
                    / 'online_async_launch.py'
                )
            ),
            launch_arguments={
                'autostart': 'true',
                'use_sim_time': 'false',
                'slam_params_file':
                    LaunchConfiguration('slam_params_file'),
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
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            condition=IfCondition(use_teleop),
        ),
    ])
