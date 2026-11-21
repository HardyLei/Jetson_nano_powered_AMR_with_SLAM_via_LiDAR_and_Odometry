"""Start the shared frontier explorer against the available Nav2 stack."""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Load the project's exploration configuration."""
    config = get_package_share_path('amr_bringup') / 'config' / 'explore.yaml'
    return LaunchDescription([
        Node(package='explore_lite', executable='explore', name='explore',
             output='screen', parameters=[str(config)]),
    ])
