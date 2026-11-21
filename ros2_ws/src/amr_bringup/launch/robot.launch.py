"""Start the Jetson's hardware, odometry and optional frontier explorer."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Reuse the sensor and drive-base launches without starting Nav2 or SLAM."""
    launch_dir = get_package_share_path('amr_bringup') / 'launch'
    sensors = {'lidar_port': '/dev/ydlidar', 'imu_port': '/dev/myimu',
               'imu_rate_hz': '50.0', 'accel_scale': '9.80665', 'gyro_scale': '1.0'}
    base = {'arduino_port': '/dev/amr', 'use_ekf': 'false', 'enable_motor_commands': 'false'}
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=value) for key, value in sensors.items()),
        DeclareLaunchArgument('arduino_port', default_value=base['arduino_port']),
        *(DeclareLaunchArgument(key, default_value=base[key], choices=['true', 'false'])
          for key in ('use_ekf', 'enable_motor_commands')),
        DeclareLaunchArgument('explore', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / 'sensors.launch.py')),
            launch_arguments={key: LaunchConfiguration(key) for key in sensors}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / 'base.launch.py')),
            launch_arguments={key: LaunchConfiguration(key) for key in base}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / 'explorer.launch.py')),
            condition=IfCondition(LaunchConfiguration('explore')),
        ),
    ])
