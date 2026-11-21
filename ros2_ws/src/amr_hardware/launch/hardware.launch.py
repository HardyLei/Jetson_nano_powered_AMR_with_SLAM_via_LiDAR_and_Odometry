"""Launch the Arduino bridge with the installed serial parameters."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Select an Arduino device path without editing a YAML file."""
    config = get_package_share_path('amr_hardware') / 'config' / 'hardware.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('enable_motor_commands', default_value='false'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/amr'),
        Node(
            package='amr_hardware',
            executable='serial_interface_node',
            name='serial_interface_node',
            output='screen',
            parameters=[str(config), {
                'port': ParameterValue(LaunchConfiguration('arduino_port'), value_type=str),
                'enable_motor_commands': ParameterValue(
                    LaunchConfiguration('enable_motor_commands'), value_type=bool,
                ),
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
