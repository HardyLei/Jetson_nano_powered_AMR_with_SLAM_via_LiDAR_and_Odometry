"""Start mapping with the AMR sensors and optional base."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Combine bring-up and GMapping."""
    share = get_package_share_path('amr_bringup')
    sensors = {'lidar_port': '/dev/ydlidar', 'imu_port': '/dev/myimu',
               'imu_rate_hz': '50.0', 'accel_scale': '9.80665', 'gyro_scale': '1.0'}
    base = ('arduino_port', 'use_ekf', 'enable_motor_commands')
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=value) for key, value in sensors.items()),
        DeclareLaunchArgument(
            'slam_params_file', default_value=str(share / 'config' / 'gmapping.yaml'),
            description='Full path to the GMapping parameter file.',
        ),
        DeclareLaunchArgument('enable_motor_commands', default_value='false'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/amr'),
        DeclareLaunchArgument('include_base', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_ekf', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'sensors.launch.py')),
            launch_arguments={key: LaunchConfiguration(key) for key in sensors}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'base.launch.py')),
            condition=IfCondition(LaunchConfiguration('include_base')),
            launch_arguments={key: LaunchConfiguration(key) for key in base}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'gmapping.launch.py')),
            launch_arguments={'slam_params_file': LaunchConfiguration('slam_params_file')}.items(),
        ),
    ])
