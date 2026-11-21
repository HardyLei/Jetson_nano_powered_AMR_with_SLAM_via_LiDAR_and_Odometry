"""Start the Arduino bridge, mecanum controller and odometry."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Select the single odom-to-base TF publisher."""
    control = get_package_share_path('amr_control') / 'launch'
    bridge = get_package_share_path('amr_hardware') / 'launch' / 'hardware.launch.py'
    ekf_config = get_package_share_path('amr_bringup') / 'config' / 'ekf.yaml'
    ekf = LaunchConfiguration('use_ekf')
    return LaunchDescription([
        DeclareLaunchArgument('enable_motor_commands', default_value='false'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/amr'),
        DeclareLaunchArgument('use_ekf', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bridge)),
            launch_arguments={key: LaunchConfiguration(key)
                              for key in ('arduino_port', 'enable_motor_commands')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(control / 'mecanum_controller.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(control / 'odometry.launch.py')),
            launch_arguments={
                'publish_tf': PythonExpression(["'false' if '", ekf, "' == 'true' else 'true'"]),
            }.items(),
        ),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            condition=IfCondition(ekf), parameters=[str(ekf_config)], output='screen',
        ),
    ])
