import math

import rclpy

from rclpy.node import Node

from amr_interfaces.msg import EncoderTicks
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from tf2_ros import quaternion_from_euler

class OdometryNode(Node):

    def __init__(self):

        super().__init__('odometry_node')

        # --------------------
        # Robot parameters
        # --------------------

        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('wheel_base_x', 0.0825)
        self.declare_parameter('wheel_base_y', 0.1031)
        self.declare_parameter('ticks_per_revolution', 660)
        self.declare_parameter('encoder_topic', '/encoder_ticks')
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base_x').value
        self.W = self.get_parameter('wheel_base_y').value
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        self.encoder_topic = self.get_parameter('encoder_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_odom_tf = self.get_parameter('publish_tf').value

        if self.wheel_radius <= 0 or self.L + self.W <= 0:
            raise ValueError('wheel_radius and wheel_base_x + wheel_base_y must be positive')
        if self.ticks_per_rev <= 0:
            raise ValueError('ticks_per_revolution must be positive')

        # --------------------
        # Robot pose
        # --------------------

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # --------------------
        # Encoder bookkeeping
        # --------------------

        self.prev_ticks = None
        self.prev_time = self.get_clock().now()

        # --------------------
        # ROS interfaces
        # --------------------

        self.encoder_sub = self.create_subscription(
            EncoderTicks,
            self.encoder_topic,
            self.encoder_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odometry_topic,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Odometry node started")

    def encoder_callback(self, msg: EncoderTicks):
        # Get current time
        current_time = self.get_clock().now()

        # Calculate time difference
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # seconds

        if self.prev_ticks is not None and dt > 0:
            # Calculate wheel displacements
            delta_ticks_fl = msg.front_left - self.prev_ticks.front_left
            delta_ticks_fr = msg.front_right - self.prev_ticks.front_right
            delta_ticks_rl = msg.rear_left - self.prev_ticks.rear_left
            delta_ticks_rr = msg.rear_right - self.prev_ticks.rear_right


            dtheta_fl = (delta_ticks_fl / self.ticks_per_rev) * 2 * math.pi
            dtheta_fr = (delta_ticks_fr / self.ticks_per_rev) * 2 * math.pi
            dtheta_rl = (delta_ticks_rl / self.ticks_per_rev) * 2 * math.pi
            dtheta_rr = (delta_ticks_rr / self.ticks_per_rev) * 2 * math.pi

            omega_fl = dtheta_fl / dt
            omega_fr = dtheta_fr / dt
            omega_rl = dtheta_rl / dt
            omega_rr = dtheta_rr / dt

            k = self.L + self.W
            r = self.wheel_radius

            vx = (r / 4) * (omega_fl + omega_fr + omega_rl + omega_rr)
            vy = (r / 4) * (-omega_fl + omega_fr + omega_rl - omega_rr)
            wz = (r / (4 * k)) * (-omega_fl + omega_fr - omega_rl + omega_rr)


            # Update robot pose
            self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
            self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
            self.yaw += wz * dt

            # Normalize yaw to [-pi, pi]
            self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

            # Publish odometry message
            self.publish_odometry(current_time, vx, vy, wz)

        # Update previous ticks and time
        self.prev_ticks = msg
        self.prev_time = current_time

    def publish_odometry(self, current_time, vx, vy, wz):
                odom_msg = Odometry()
                odom_msg.header.stamp = current_time.to_msg()
                odom_msg.header.frame_id = self.odom_frame
                odom_msg.child_frame_id = self.base_frame
                odom_msg.pose.pose.position.x = self.x
                odom_msg.pose.pose.position.y = self.y
                odom_msg.pose.pose.position.z = 0.0
                q = quaternion_from_euler(0, 0, self.yaw)
                odom_msg.pose.pose.orientation.x = q[0]
                odom_msg.pose.pose.orientation.y = q[1]
                odom_msg.pose.pose.orientation.z = q[2]
                odom_msg.pose.pose.orientation.w = q[3]
                odom_msg.twist.twist.linear.x = vx
                odom_msg.twist.twist.linear.y = vy
                odom_msg.twist.twist.angular.z = wz 
                self.odom_pub.publish(odom_msg)
                if self.publish_odom_tf:
                    self.publish_tf(current_time, q)

    def publish_tf(self, current_time, q):
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()