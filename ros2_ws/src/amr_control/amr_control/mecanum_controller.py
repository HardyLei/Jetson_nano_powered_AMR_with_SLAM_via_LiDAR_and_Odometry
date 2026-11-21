"""Limit body twist commands and convert them to mecanum wheel speeds."""

from math import hypot, isfinite
from time import monotonic

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from amr_interfaces.msg import DriveMode, ManualDrive, WheelSpeed
from amr_interfaces.srv import SetManualDrive
from geometry_msgs.msg import Twist
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool

from .kinematics import inverse_kinematics


class MecanumController(Node):
    """Publish rad/s wheel commands and stop when cmd_vel goes stale."""

    def __init__(self):
        super().__init__('mecanum_controller')
        self.radius = self.declare_parameter('wheel_radius', 0.04).value
        self.k = (self.declare_parameter('wheel_base_x', 0.0825).value +
                  self.declare_parameter('wheel_base_y', 0.1031).value)
        self.max_linear = self.declare_parameter('max_linear_speed', 0.15).value
        self.max_angular = self.declare_parameter('max_angular_speed', 0.5).value
        self.timeout = self.declare_parameter('cmd_vel_timeout_sec', 0.25).value
        if not all(isfinite(v) and v > 0.0 for v in (
                self.radius, self.k, self.max_linear, self.max_angular, self.timeout)):
            raise ValueError('Wheel geometry, velocity limits and command timeout must be positive')

        input_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        output_topic = self.declare_parameter('wheel_speed_topic', '/wheel_speed_cmd').value
        self.wheel_pub = self.create_publisher(WheelSpeed, output_topic, 1)
        self.last_command = None
        self.navigation_enabled = True
        self.manual_session = ''
        self.last_manual_command = 0.0
        self.active_nav_goals = set()
        self.nav_status = {}
        self.navigation_release = None

        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        manual_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.mode_pub = self.create_publisher(DriveMode, '/amr/drive_mode', retained)
        self.explore_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.cancel_nav = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self.on_nav_status, retained,
        )
        self.mode_group = ReentrantCallbackGroup()
        self.mode_service = self.create_service(
            SetManualDrive, '/amr/set_manual_drive', self.set_manual_drive,
            callback_group=self.mode_group,
        )
        self.cmd_sub = self.create_subscription(Twist, input_topic, self.on_nav_command, 1)
        self.manual_sub = self.create_subscription(
            ManualDrive, '/amr/manual_drive', self.on_manual_command, manual_qos,
        )
        self.timer = self.create_timer(min(self.timeout / 2.0, 0.05), self.check_timeout)
        self.publish_mode()

    def publish_mode(self):
        """Report the drive mode and which RViz session owns manual control."""
        self.mode_pub.publish(DriveMode(
            navigation_enabled=self.navigation_enabled, manual_session=self.manual_session,
        ))

    def on_nav_status(self, msg):
        """Track when Nav2 has actually finished canceling its goals."""
        self.nav_status = {
            bytes(goal.goal_info.goal_id.uuid): goal.status for goal in msg.status_list
        }
        running = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                   GoalStatus.STATUS_CANCELING}
        self.active_nav_goals = {key for key, state in self.nav_status.items() if state in running}

    async def set_manual_drive(self, request, response):
        """Grant exclusive manual control or hand it back after Nav2 stops."""
        self.expire_manual_session(monotonic())
        if not request.session_id:
            response.message = 'A drive session ID is required.'
        elif self.navigation_release is not None:
            response.message = 'Waiting for Nav2 to stop.'
        elif self.manual_session and self.manual_session != request.session_id:
            response.message = 'Another RViz window has manual control.'
        elif request.enable:
            self.navigation_enabled = False
            self.manual_session = request.session_id
            self.last_manual_command = monotonic()
            self.stop()
            self.explore_pub.publish(Bool(data=False))
            if self.cancel_nav.service_is_ready():
                # An empty goal ID and timestamp request cancellation of all goals.
                self.cancel_nav.call_async(CancelGoal.Request())
            self.publish_mode()
            response.success = True
        elif self.navigation_enabled:
            response.success = True
        elif not self.cancel_nav.service_is_ready():
            response.message = 'Nav2 is unavailable. Drive remains paused.'
        else:
            self.stop()
            self.explore_pub.publish(Bool(data=False))
            completion = Future(executor=self.executor)
            self.navigation_release = (
                completion, self.cancel_nav.call_async(CancelGoal.Request()), monotonic() + 2.0,
            )
            response.success = await completion
            if not response.success:
                response.message = 'Nav2 did not confirm that its goals stopped. Drive remains paused.'
        return response

    def finish_navigation_release(self, now):
        """Enable Nav2 only after the cancel request and goal status agree."""
        completion, cancellation, deadline = self.navigation_release
        success = False
        if not cancellation.done():
            if now < deadline:
                return
            self.cancel_nav.remove_pending_request(cancellation)
        elif not cancellation.cancelled() and cancellation.exception() is None:
            reply = cancellation.result()
            if reply.return_code == CancelGoal.Response.ERROR_NONE:
                finished = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                            GoalStatus.STATUS_ABORTED}
                awaiting_stop = self.active_nav_goals or any(
                    self.nav_status.get(bytes(goal.goal_id.uuid)) not in finished
                    for goal in reply.goals_canceling
                )
                if awaiting_stop and now < deadline:
                    return
                success = not awaiting_stop

        self.navigation_release = None
        if success:
            self.manual_session = ''
            self.navigation_enabled = True
            self.publish_mode()
        completion.set_result(success)

    def on_nav_command(self, msg):
        """Accept the original command topic whenever navigation owns the base."""
        if self.navigation_enabled:
            self.on_command(msg)

    def on_manual_command(self, msg):
        """Accept commands only from the current RViz session."""
        now = monotonic()
        self.expire_manual_session(now)
        if (self.manual_session and msg.session_id == self.manual_session
                and self.navigation_release is None):
            self.last_manual_command = now
            self.on_command(msg.velocity)

    def expire_manual_session(self, now):
        """A silent manual station stops the base and leaves Nav2 locked out."""
        if self.manual_session and now - self.last_manual_command >= self.timeout:
            self.manual_session = ''
            self.stop()
            self.publish_mode()

    def on_command(self, msg):
        """Limit velocity magnitude while preserving the translation direction."""
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        if not all(map(isfinite, (vx, vy, wz))):
            self.stop()
            return

        linear = hypot(vx, vy)
        if linear > self.max_linear:
            vx *= self.max_linear / linear
            vy *= self.max_linear / linear
        wz = max(-self.max_angular, min(self.max_angular, wz))
        fl, fr, rl, rr = inverse_kinematics(vx, vy, wz, self.radius, self.k)
        self.wheel_pub.publish(WheelSpeed(
            front_left=fl, front_right=fr, rear_left=rl, rear_right=rr,
        ))
        self.last_command = monotonic()

    def stop(self):
        """Publish one zero command after receiving motion commands."""
        if self.last_command is not None:
            self.wheel_pub.publish(WheelSpeed())
            self.last_command = None

    def check_timeout(self):
        """Stop sending stale motion requests."""
        now = monotonic()
        self.expire_manual_session(now)
        if self.navigation_release is not None:
            self.finish_navigation_release(now)
        if self.last_command is not None and now - self.last_command >= self.timeout:
            self.stop()


def main(args=None):
    """Start the mecanum controller."""
    rclpy.init(args=args)
    node = None
    try:
        node = MecanumController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
