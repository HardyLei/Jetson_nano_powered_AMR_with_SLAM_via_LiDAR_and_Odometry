from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    """Select a lidar model, defaulting to X3."""
    launches = get_package_share_path('ydlidar_ros2_driver') / 'launch'
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_type', default_value=EnvironmentVariable('LIDAR_TYPE', default_value='x3'),
            choices=['x3', '4ros'],
        ),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ydlidar'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launches / 'x3_ydlidar_launch.py')),
            condition=LaunchConfigurationEquals('lidar_type', 'x3'),
            launch_arguments={'lidar_port': LaunchConfiguration('lidar_port')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launches / '4ros_ydlidar_launch.py')),
            condition=LaunchConfigurationEquals('lidar_type', '4ros'),
            launch_arguments={'lidar_port': LaunchConfiguration('lidar_port')}.items(),
        ),
    ])
