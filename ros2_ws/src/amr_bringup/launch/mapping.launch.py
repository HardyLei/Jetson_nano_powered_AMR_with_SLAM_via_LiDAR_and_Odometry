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
    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(
                bringup_share / 'config' / 'gmapping.yaml'
            ),
            description='Full path to the GMapping parameter file.',
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
                str(bringup_share / 'launch' / 'gmapping.launch.py')
            ),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(bringup_share / 'config' / 'mapping.rviz')],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            condition=IfCondition(use_teleop),
        ),
    ])
