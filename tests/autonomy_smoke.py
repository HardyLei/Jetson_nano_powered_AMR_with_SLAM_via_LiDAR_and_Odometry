"""Exercise SLAM, Nav2 and map reload with a simulated planar robot and lidar."""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class TestRobot(Node):
    """Integrate Nav2 twist and raycast a rectangular room at wall-clock rate."""

    def __init__(self):
        super().__init__('amr_test_robot')
        self.pose = [0.0, 0.0, 0.0]
        self.barrier = False
        self.cmd = Twist()
        self.cmd_time = 0.0
        self.map = None
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.explore_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        fixed = TransformStamped()
        fixed.header.frame_id = 'base_link'
        fixed.child_frame_id = 'LiDAR_Link'
        fixed.transform.rotation.w = 1.0
        self.static_tf.sendTransform(fixed)
        self.create_timer(0.05, self.tick)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def on_command(self, cmd):
        """Accept only recent velocity commands."""
        self.cmd = cmd
        self.cmd_time = time.monotonic()

    def on_map(self, msg):
        """Track map publication from SLAM or map_server."""
        self.map = msg

    def tick(self):
        """Publish simulated odometry, TF and lidar data."""
        x, y, yaw = self.pose
        cmd = self.cmd if time.monotonic() - self.cmd_time < 0.3 else Twist()
        x += 0.05 * (cmd.linear.x * math.cos(yaw) - cmd.linear.y * math.sin(yaw))
        y += 0.05 * (cmd.linear.x * math.sin(yaw) + cmd.linear.y * math.cos(yaw))
        yaw += 0.05 * cmd.angular.z
        self.pose = [x, y, yaw]
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(yaw / 2)
        tf.transform.rotation.w = math.cos(yaw / 2)
        self.tf.sendTransform(tf)
        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = tf.child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist = cmd
        self.odom_pub.publish(odom)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'LiDAR_Link'
        scan.angle_min = -math.pi
        scan.angle_increment = math.tau / 360
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min = 0.12
        scan.range_max = 8.0
        for i in range(360):
            angle = scan.angle_min + i * scan.angle_increment + yaw
            dx, dy = math.cos(angle), math.sin(angle)
            sx = ((2.5 if dx > 0 else -1.8) - x) / dx if abs(dx) > 1e-8 else math.inf
            sy = ((1.6 if dy > 0 else -1.4) - y) / dy if abs(dy) > 1e-8 else math.inf
            distance = min(sx, sy)
            if self.barrier:
                # A panel occludes part of the room, leaving frontiers around its edges.
                for wall_x in (1.0, 1.1):
                    t = (wall_x - x) / dx if abs(dx) > 1e-8 else -1.0
                    if t > 0.12 and -0.55 < y + t * dy < 0.55:
                        distance = min(distance, t)
                for wall_y in (-0.55, 0.55):
                    t = (wall_y - y) / dy if abs(dy) > 1e-8 else -1.0
                    if t > 0.12 and 1.0 < x + t * dx < 1.1:
                        distance = min(distance, t)
            scan.ranges.append(distance)
        self.scan_pub.publish(scan)

    def wait_for(self, condition, seconds, label):
        """Spin until a ROS condition becomes true."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return
        raise RuntimeError(f'Timed out waiting for {label}')

    def drive_to(self, x, y):
        """Send a goal through the same action server used by RViz."""
        self.wait_for_nav2()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        accepted = self.nav.send_goal_async(goal)
        self.wait_for(accepted.done, 10, 'goal response')
        if not accepted.result().accepted:
            raise RuntimeError('Nav2 rejected the goal')
        result = accepted.result().get_result_async()
        self.wait_for(result.done, 25, 'goal result')
        if result.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Nav2 goal status {result.result().status}')
        if math.hypot(x - self.pose[0], y - self.pose[1]) > 0.25:
            raise RuntimeError(f'Simulated pose {self.pose} did not reach ({x}, {y})')

    def wait_for_nav2(self):
        """Wait for all navigation lifecycle transitions before sending a goal."""
        client = self.create_client(GetState, '/velocity_smoother/get_state')
        try:
            self.wait_for(client.service_is_ready, 15, 'Nav2 lifecycle service')
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                state = client.call_async(GetState.Request())
                # DDS can still discover the previous mode's endpoints during
                # a rapid restart. Retry instead of waiting on a stale server.
                reply_by = time.monotonic() + 1.0
                while not state.done() and time.monotonic() < reply_by:
                    rclpy.spin_once(self, timeout_sec=0.05)
                if not state.done():
                    client.remove_pending_request(state)
                elif state.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                    self.wait_for(self.nav.server_is_ready, 5, 'Nav2 goal server')
                    return
                pause_until = time.monotonic() + 0.2
                self.wait_for(lambda: time.monotonic() >= pause_until, 1, 'lifecycle polling')
            raise RuntimeError('Nav2 did not become active')
        finally:
            self.destroy_client(client)

    def set_initial_pose(self):
        """Seed saved-map localization with the current simulated pose."""
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x, pose.pose.pose.position.y, yaw = self.pose
        pose.pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.pose.orientation.w = math.cos(yaw / 2)
        pose.pose.covariance[0] = 0.01
        pose.pose.covariance[7] = 0.01
        pose.pose.covariance[35] = 0.01
        self.initial_pub.publish(pose)


def start_launch(mode, log, *args):
    """Start one real autonomy launch in its own process group."""
    return subprocess.Popen(
        ['ros2', 'launch', 'amr_bringup', 'autonomy.launch.py',
         'start_robot:=false', f'mode:={mode}', *args],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )


def stop_launch(process):
    """Stop the launch and its child nodes before switching modes."""
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def check_exploration(robot):
    """Verify automatic frontier motion and pause without sending an RViz goal."""
    robot.pose = [0.0, 0.0, 0.0]
    robot.barrier = True
    robot.map = None
    robot.cmd = Twist()
    with tempfile.TemporaryFile(mode='w+') as log:
        launch = start_launch('mapping', log, 'explore:=true')
        try:
            robot.wait_for(lambda: robot.map is not None, 25, 'exploration map')
            robot.wait_for_nav2()
            robot.wait_for(
                lambda: robot.explore_pub.get_subscription_count() > 0,
                10, 'exploration control subscription',
            )
            deadline = time.monotonic() + 3
            robot.wait_for(lambda: time.monotonic() >= deadline, 4, 'idle startup')
            if math.dist((0.0, 0.0, 0.0), robot.pose) > 0.01:
                raise RuntimeError('The explorer moved before being started')
            robot.explore_pub.publish(Bool(data=True))
            robot.wait_for(
                lambda: math.hypot(*robot.pose[:2]) > 0.12, 35, 'frontier-directed motion',
            )
            robot.explore_pub.publish(Bool(data=False))
            deadline = time.monotonic() + 3
            robot.wait_for(lambda: time.monotonic() >= deadline, 4, 'explorer cancellation')
            stopped_pose = tuple(robot.pose)
            deadline = time.monotonic() + 1.5
            robot.wait_for(lambda: time.monotonic() >= deadline, 2, 'stopped robot')
            if math.dist(stopped_pose, robot.pose) > 0.02:
                raise RuntimeError('The robot continued moving after pausing exploration')
            print('PASS frontier-selected motion and pause', flush=True)
        except Exception:
            stop_launch(launch)
            log.seek(0)
            print(log.read()[-24_000:])
            raise
        finally:
            stop_launch(launch)


def main():
    """Build a SLAM map, drive on it, and repeat with saved-map localization."""
    rclpy.init()
    robot = TestRobot()
    try:
        with tempfile.TemporaryDirectory(prefix='amr-smoke-') as folder:
            room = str(Path(folder) / 'room')
            for mode, goal in (('mapping', (0.55, 0.22)), ('navigation', (0.1, -0.2))):
                args = () if mode == 'mapping' else (f'map:={room}.yaml',)
                robot.map = None
                with tempfile.TemporaryFile(mode='w+') as log:
                    launch = start_launch(mode, log, *args)
                    try:
                        robot.wait_for(lambda: robot.map is not None, 25, f'{mode} /map')
                        if mode == 'navigation':
                            # AMCL must see the map and receive an initial pose before activating Nav2.
                            robot.wait_for(
                                lambda: robot.initial_pub.get_subscription_count() > 0,
                                15, 'AMCL initial pose subscriber',
                            )
                            timer = robot.create_timer(0.5, robot.set_initial_pose)
                            try:
                                robot.wait_for_nav2()
                            finally:
                                robot.destroy_timer(timer)
                            robot.set_initial_pose()
                            deadline = time.monotonic() + 1.5
                            robot.wait_for(lambda: time.monotonic() >= deadline, 2, 'AMCL settling')
                        robot.drive_to(*goal)
                        if mode == 'mapping':
                            subprocess.run(
                                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', room],
                                timeout=12, check=True,
                            )
                            if not Path(f'{room}.yaml').is_file():
                                raise RuntimeError('Map saver did not write its YAML')
                        print(f'PASS {mode} with live lidar and Nav2 goal {goal}', flush=True)
                    except Exception:
                        stop_launch(launch)
                        log.seek(0)
                        print(log.read()[-16_000:])
                        raise
                    finally:
                        stop_launch(launch)
    finally:
        robot.destroy_node()
        rclpy.shutdown()

    # Use a fresh DDS domain so queued simulator commands from either earlier
    # mode cannot be mistaken for motion from the paused explorer.
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    os.environ['ROS_DOMAIN_ID'] = str((domain + 1) % 102)
    rclpy.init()
    robot = TestRobot()
    try:
        check_exploration(robot)
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
