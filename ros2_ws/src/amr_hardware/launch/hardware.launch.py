from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_hardware',
            executable='serial_interface_node',
            name='serial_interface_node',
            output='screen',
            parameters=["config/hardware.yaml"]
        )
    ])