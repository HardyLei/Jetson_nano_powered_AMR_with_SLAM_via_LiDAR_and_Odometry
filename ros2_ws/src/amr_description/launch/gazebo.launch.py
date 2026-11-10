from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("amr_description"))
    urdf_path = package_share / "urdf" / "amr.urdf"
    robot_description = urdf_path.read_text()
    gazebo_launch = Path(
        get_package_share_directory("gazebo_ros")
    ) / "launch" / "gazebo.launch.py"

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gazebo_launch))
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", "amr", "-file", str(urdf_path)],
                output="screen",
            ),
        ]
    )
