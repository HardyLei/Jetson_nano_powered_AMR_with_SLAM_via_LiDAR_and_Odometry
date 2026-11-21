"""Display the running IMU in RViz, with optional driver startup."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Use the running IMU by default so only one driver owns the port."""
    share = get_package_share_path('imu_ros2_device')
    args = {'port': '/dev/myimu', 'rate_hz': '50.0', 'accel_scale': '9.80665', 'gyro_scale': '1.0'}
    return LaunchDescription([
        DeclareLaunchArgument('rvizconfig', default_value=str(share / 'rviz' / 'ybimu.rviz')),
        DeclareLaunchArgument('start_driver', default_value='false',
                              description='Set true only if the serial IMU is stopped'),
        *(DeclareLaunchArgument(key, default_value=value) for key, value in args.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'ybimu.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_driver')),
            launch_arguments={key: LaunchConfiguration(key) for key in args}.items(),
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
    ])
