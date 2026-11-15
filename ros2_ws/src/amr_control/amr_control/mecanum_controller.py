import rclpy

from rclpy.node import Node

from amr_interfaces.msg import WheelSpeed

from geometry_msgs.msg import Twist

class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')
        self.wheel_speed_publisher = self.create_publisher(WheelSpeed, '/wheel_speed_cmd', 10)

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        # Convert Twist message to wheel speeds for mecanum wheels
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        R = 0.05  # Wheel radius in meters
        L = 0.0825  # Distance from the center of the robot to the wheel along the x-axis
        W = 0.1031  # Distance from the center of the robot to the wheel along the y-axis

        k = L + W

        # inverse kinematics for mecanum wheels
        fl = (vx - vy - k*wz) / R
        fr = (vx + vy + k*wz) / R
        rl = (vx + vy - k*wz) / R
        rr = (vx - vy + k*wz) / R

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