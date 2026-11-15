import rclpy

from rclpy.node import Node

from amr_interfaces.msg import WheelSpeed

from geometry_msgs.msg import Twist

class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base_x', 0.0825)
        self.declare_parameter('wheel_base_y', 0.1031)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_speed_topic', '/wheel_speed_cmd')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base_x = self.get_parameter('wheel_base_x').value
        self.wheel_base_y = self.get_parameter('wheel_base_y').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.wheel_speed_topic = self.get_parameter('wheel_speed_topic').value

        if self.wheel_radius <= 0 or self.wheel_base_x + self.wheel_base_y <= 0:
            raise ValueError(
                'wheel_radius and wheel_base_x + wheel_base_y must be positive'
            )

        self.wheel_speed_publisher = self.create_publisher(
            WheelSpeed,
            self.wheel_speed_topic,
            10
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        # Convert Twist message to wheel speeds for mecanum wheels
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        k = self.wheel_base_x + self.wheel_base_y

        # inverse kinematics for mecanum wheels
        fl = (vx - vy - k*wz) / self.wheel_radius
        fr = (vx + vy + k*wz) / self.wheel_radius
        rl = (vx + vy - k*wz) / self.wheel_radius
        rr = (vx - vy + k*wz) / self.wheel_radius

        wheel_speed_msg = WheelSpeed(
            front_left=fl,
            front_right=fr,
            rear_left=rl,
            rear_right=rr
        )
        
        self.wheel_speed_publisher.publish(wheel_speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()