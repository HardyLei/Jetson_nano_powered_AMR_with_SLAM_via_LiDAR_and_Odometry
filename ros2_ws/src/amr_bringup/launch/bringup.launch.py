from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    description_share = Path(get_package_share_directory('amr_description'))
    control_share = Path(get_package_share_directory('amr_control'))
    hardware_share = Path(get_package_share_directory('amr_hardware'))
    imu_share = Path(get_package_share_directory('imu_ros2_device'))
    lidar_share = Path(get_package_share_directory('ydlidar_ros2_driver'))
    bringup_share = Path(get_package_share_directory('amr_bringup'))

    robot_description = (
        description_share / 'urdf' / 'amr.urdf'
    ).read_text()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_description',
            default_value='true',
            description='Start robot_state_publisher.',
        ),
        DeclareLaunchArgument(
            'use_control',
            default_value='true',
            description='Start the mecanum velocity controller.',
        ),
        DeclareLaunchArgument(
            'use_odometry',
            default_value='true',
            description='Start encoder-based odometry.',
        ),
        DeclareLaunchArgument(
            'use_hardware',
            default_value='true',
            description='Start the serial motor and encoder interface.',
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Start the YBIMU driver and Madgwick filter.',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Start the YDLIDAR X3 lifecycle driver.',
        ),
        DeclareLaunchArgument(
            'use_localization',
            default_value='true',
            description='Start the Robot Localization EKF.',
        ),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='false',
            description='Publish odom -> base_link from encoder odometry. '
                        'Set false when Robot Localization is enabled.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(LaunchConfiguration('use_description')),
            output='screen',
        ),
        Node(
            package='amr_control',
            executable='mecanum_controller',
            name='mecanum_controller',
            parameters=[str(control_share / 'config' / 'mecanum_controller.yaml')],
            condition=IfCondition(LaunchConfiguration('use_control')),
            output='screen',
        ),
        Node(
            package='amr_control',
            executable='odometry_node',
            name='odometry_node',
            parameters=[
                str(control_share / 'config' / 'odometry.yaml'),
                {'publish_tf': LaunchConfiguration('publish_odom_tf')},
            ],
            condition=IfCondition(LaunchConfiguration('use_odometry')),
            output='screen',
        ),
        Node(
            package='amr_hardware',
            executable='serial_interface_node',
            name='serial_interface_node',
            parameters=[
                str(hardware_share / 'config' / 'hardware.yaml')
            ],
            condition=IfCondition(LaunchConfiguration('use_hardware')),
            output='screen',
        ),
        Node(
            package='imu_ros2_device',
            executable='ybimu_driver',
            name='ybimu_driver',
            condition=IfCondition(LaunchConfiguration('use_imu')),
            output='screen',
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            parameters=[str(imu_share / 'config' / 'imu_filter_param.yaml')],
            condition=IfCondition(LaunchConfiguration('use_imu')),
            output='screen',
        ),
        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            parameters=[
                str(lidar_share / 'params' / 'ydlidar_x3.yaml'),
                {'frame_id': 'LiDAR_Link'},
            ],
            condition=IfCondition(LaunchConfiguration('use_lidar')),
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                str(bringup_share / 'config' / 'ekf.yaml')
            ],
            condition=IfCondition(LaunchConfiguration('use_localization')),
            output='screen',
        ),
    ])
