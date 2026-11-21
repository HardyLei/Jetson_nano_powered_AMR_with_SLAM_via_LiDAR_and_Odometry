"""Start the lidar, IMU and URDF transform publishers."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the sensors without creating motor commands."""
    lidar = get_package_share_path('ydlidar_ros2_driver') / 'launch' / 'x3_ydlidar_launch.py'
    imu = get_package_share_path('imu_ros2_device') / 'launch' / 'ybimu.launch.py'
    urdf = get_package_share_path('amr_description') / 'urdf' / 'amr.urdf'
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ydlidar'),
        DeclareLaunchArgument('imu_port', default_value='/dev/myimu'),
        DeclareLaunchArgument('imu_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('accel_scale', default_value='9.80665'),
        DeclareLaunchArgument('gyro_scale', default_value='1.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lidar)),
            launch_arguments={'lidar_port': LaunchConfiguration('lidar_port')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(imu)),
            launch_arguments={
                'port': LaunchConfiguration('imu_port'),
                'rate_hz': LaunchConfiguration('imu_rate_hz'),
                'accel_scale': LaunchConfiguration('accel_scale'),
                'gyro_scale': LaunchConfiguration('gyro_scale'),
                'rotate_yaw_180': 'true',
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf.read_text()}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),
    ])
