"""Start the Yahboom serial driver and Madgwick filter."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Configure one IMU driver and filter."""
    config = get_package_share_path('imu_ros2_device') / 'config' / 'imu_filter_param.yaml'
    args = {'port': ('/dev/myimu', str), 'rate_hz': ('50.0', float),
            'accel_scale': ('9.80665', float), 'gyro_scale': ('1.0', float)}
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=default)
          for key, (default, _) in args.items()),
        Node(
            package='imu_ros2_device',
            executable='ybimu_driver',
            output='screen',
            parameters=[{
                key: ParameterValue(LaunchConfiguration(key), value_type=kind)
                for key, (_, kind) in args.items()
            }],
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[str(config)],
        ),
    ])
