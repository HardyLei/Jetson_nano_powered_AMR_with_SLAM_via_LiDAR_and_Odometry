"""Start the live mapper against the scan and odometry TF in this ROS domain."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Use a single mapper configuration across all live mapping launches."""
    config = get_package_share_path('amr_bringup') / 'config' / 'gmapping.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file', default_value=str(config)),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        Node(
            package='slam_gmapping',
            executable='slam_gmapping_node',
            name='slam_gmapping',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
            remappings=[('scan', '/scan'), ('map', '/map')],
        ),
    ])
