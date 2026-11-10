from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("amr_description"))
    urdf_path = package_share / "urdf" / "amr.urdf"
    robot_description = urdf_path.read_text()

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(LaunchConfiguration("use_gui")),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
