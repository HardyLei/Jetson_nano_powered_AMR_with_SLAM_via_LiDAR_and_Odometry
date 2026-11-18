"""Run Nav2 with live SLAM or localization on a saved map."""

from pathlib import Path

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def include(path, **params):
    """Forward explicit arguments to an installed launch file."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)), launch_arguments=params.items(),
    )


def make_stack(context, share, nav2):
    """Resolve operating mode and select the owner of map-to-odom TF."""
    mode = LaunchConfiguration('mode').perform(context)
    mapping = mode == 'mapping'
    explore = LaunchConfiguration('explore').perform(context) == 'true'
    if explore and not mapping:
        raise RuntimeError('explore:=true requires mode:=mapping')

    use_ekf = LaunchConfiguration('use_ekf')
    odom_topic = '/odometry/filtered' if use_ekf.perform(context) == 'true' else '/odom'
    params = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params_file'), root_key='',
        param_rewrites={
            'odom_topic': odom_topic,
            'allow_unknown': 'true' if mapping else 'false',
        },
        convert_types=True,
    )
    nav_args = {'params_file': params, 'use_sim_time': 'false',
                'autostart': 'true', 'use_composition': 'False'}
    launch_dir = share / 'launch'
    sensors = ('lidar_port', 'imu_port', 'imu_rate_hz', 'accel_scale', 'gyro_scale')
    base = ('arduino_port', 'use_ekf', 'enable_motor_commands')
    actions = [
        GroupAction(
            condition=IfCondition(LaunchConfiguration('start_robot')),
            actions=[
                include(launch_dir / 'sensors.launch.py',
                        **{key: LaunchConfiguration(key) for key in sensors}),
                include(launch_dir / 'base.launch.py',
                        **{key: LaunchConfiguration(key) for key in base}),
            ],
        ),
    ]
    if mapping:
        slam = get_package_share_path('slam_toolbox') / 'launch' / 'online_async_launch.py'
        actions.append(include(
            slam, slam_params_file=str(share / 'config' / 'slam_toolbox.yaml'),
            use_sim_time='false',
        ))
    else:
        map_file = LaunchConfiguration('map').perform(context)
        if not map_file or not Path(map_file).is_file():
            raise RuntimeError('mode:=navigation requires map:=/path/to/saved-map.yaml')
        actions.append(GroupAction(actions=[
            include(nav2 / 'localization_launch.py',
                    map=str(Path(map_file).resolve()), **nav_args),
        ]))

    actions.extend([
        GroupAction(actions=[include(nav2 / 'navigation_launch.py', **nav_args)]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', str(share / 'config' / 'mapping.rviz')],
        ),
    ])
    if explore:
        actions.append(Node(
            package='explore_lite', executable='explore', name='explore',
            output='screen', parameters=[str(share / 'config' / 'explore.yaml')],
        ))
    return actions


def generate_launch_description():
    """Declare the mapping and navigation entry point."""
    share = get_package_share_path('amr_bringup')
    nav2 = get_package_share_path('nav2_bringup') / 'launch'
    flags = {'explore': 'false', 'rviz': 'false', 'use_ekf': 'false',
             'start_robot': 'true', 'enable_motor_commands': 'false'}
    defaults = {'map': '', 'lidar_port': '/dev/ydlidar', 'imu_port': '/dev/myimu',
                'arduino_port': '/dev/amr', 'imu_rate_hz': '50.0',
                'accel_scale': '9.80665', 'gyro_scale': '1.0'}
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'navigation']),
        DeclareLaunchArgument(
            'nav2_params_file', default_value=str(share / 'config' / 'nav2.yaml'),
        ),
        *(DeclareLaunchArgument(key, default_value=value, choices=['true', 'false'])
          for key, value in flags.items()),
        *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
        OpaqueFunction(function=make_stack, args=[share, nav2]),
    ])
