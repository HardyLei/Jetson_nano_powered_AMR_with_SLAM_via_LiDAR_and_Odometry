import rclpy

from rclpy.node import Node

from amr_interfaces.msg import EncoderTicks, WheelSpeed

from .serial_driver import SerialDriver
from .packet_builder import build_wheel_command
from .packet_parser import parse_encoder_packet
from .config import SERIAL_PORT, BAUDRATE

class SerialInterfaceNode(Node):
    def __init__(self):
        super().__init__('serial_interface_node')
        self.serial_driver = SerialDriver(SERIAL_PORT, BAUDRATE)
        self.encoder_publisher = self.create_publisher(EncoderTicks, '/encoder_ticks', 10)
        self.wheel_command_subscription = self.create_subscription(
            WheelSpeed,
            '/wheel_speed_cmd',
            self.wheel_command_callback,
            10
        )

        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        packet = self.serial_driver.read()
        if packet:
            try:
                encoder_values = parse_encoder_packet(packet)
                msg = EncoderTicks(
                    front_left=encoder_values["front_left"],
                    front_right=encoder_values["front_right"],
                    rear_left=encoder_values["rear_left"],
                    rear_right=encoder_values["rear_right"]
                )
                self.encoder_publisher.publish(msg)
            except ValueError as e:
                self.get_logger().error(f"Failed to parse encoder packet: {e}")

    def wheel_command_callback(self, msg):
        fl, fr, rl, rr = map(
            int,
            (msg.front_left, msg.front_right, msg.rear_left, msg.rear_right)
        )
        packet = build_wheel_command(fl, fr, rl, rr)
        self.serial_driver.write(packet)

def main(args=None):
    rclpy.init(args=args)
    node = SerialInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()