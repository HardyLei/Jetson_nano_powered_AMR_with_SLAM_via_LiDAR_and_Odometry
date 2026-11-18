"""Limit body twist commands and convert them to mecanum wheel speeds."""

from math import hypot, isfinite
from time import monotonic

from amr_interfaces.msg import WheelSpeed
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

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
        self.cmd_sub = self.create_subscription(Twist, input_topic, self.on_command, 1)
        self.last_command = None
        self.timer = self.create_timer(min(self.timeout / 2.0, 0.05), self.check_timeout)

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
        if self.last_command is not None and monotonic() - self.last_command >= self.timeout:
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
