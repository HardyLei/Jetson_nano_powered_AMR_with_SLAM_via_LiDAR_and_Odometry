"""Publish planar mecanum odometry from cumulative encoder ticks."""

from math import isfinite

from amr_interfaces.msg import EncoderTicks
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .kinematics import encoder_twist, integrate_pose, yaw_quaternion


class OdometryNode(Node):
    """Estimate the body twist, pose and optional odom-to-base transform."""

    def __init__(self):
        super().__init__('odometry_node')
        self.radius = self.declare_parameter('wheel_radius', 0.04).value
        self.k = (self.declare_parameter('wheel_base_x', 0.0825).value +
                  self.declare_parameter('wheel_base_y', 0.1031).value)
        self.ticks_per_rev = self.declare_parameter('ticks_per_revolution', 660).value
        self.max_dt = self.declare_parameter('max_dt_sec', 0.5).value
        if not all(isfinite(v) and v > 0 for v in (
                self.radius, self.k, self.ticks_per_rev, self.max_dt)):
            raise ValueError('Wheel geometry, encoder resolution and max_dt_sec must be positive')

        encoder_topic = self.declare_parameter('encoder_topic', '/encoder_ticks').value
        odom_topic = self.declare_parameter('odometry_topic', '/odom').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        publish_tf = self.declare_parameter('publish_tf', True).value
        self.pose_covariance = self.read_covariance(
            'pose_covariance_diagonal', [0.04, 0.04, 1e3, 1e3, 1e3, 0.09],
        )
        self.twist_covariance = self.read_covariance(
            'twist_covariance_diagonal', [0.02, 0.04, 1e3, 1e3, 1e3, 0.04],
        )
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.encoder_sub = self.create_subscription(
            EncoderTicks, encoder_topic, self.on_encoder, 10,
        )
        self.broadcaster = TransformBroadcaster(self) if publish_tf else None
        self.previous = None
        self.pose = (0.0, 0.0, 0.0)

    def read_covariance(self, name, default):
        """Expand the parameter's six variances into a 6-by-6 covariance."""
        diagonal = self.declare_parameter(name, default).value
        if len(diagonal) != 6 or any(not isfinite(v) or v <= 0 for v in diagonal):
            raise ValueError(f'{name} must contain six positive finite variances')
        matrix = [0.0] * 36
        matrix[::7] = diagonal
        return matrix

    def on_encoder(self, msg):
        """Integrate a pair of samples if their timestamps have a usable gap."""
        now = self.get_clock().now()
        ticks = (msg.front_left, msg.front_right, msg.rear_left, msg.rear_right)
        previous = self.previous
        self.previous = (now, ticks)
        if previous is None:
            return

        dt = (now - previous[0]).nanoseconds / 1e9
        if not 0.0 < dt <= self.max_dt:
            self.get_logger().warning(
                f'Skipping odometry update after encoder time gap ({dt:.3f} s)', once=True,
            )
            return

        vx, vy, wz = encoder_twist(previous[1], ticks, dt, self.ticks_per_rev,
                                   self.radius, self.k)
        self.pose = integrate_pose(*self.pose, vx, vy, wz, dt)
        self.publish_odometry(now, vx, vy, wz)

    def publish_odometry(self, now, vx, vy, wz):
        """Publish the odometry estimate and its matching transform."""
        x, y, yaw = self.pose
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        pose = odom.pose.pose
        pose.position.x = x
        pose.position.y = y
        orientation = pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = yaw_quaternion(yaw)
        odom.pose.covariance = self.pose_covariance
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = self.twist_covariance
        self.odom_pub.publish(odom)

        if self.broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation = orientation
            self.broadcaster.sendTransform(transform)


def main(args=None):
    """Start the encoder odometry node."""
    rclpy.init(args=args)
    node = None
    try:
        node = OdometryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
