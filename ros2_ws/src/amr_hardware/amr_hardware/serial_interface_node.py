"""Bridge ROS wheel commands and encoder feedback to the Arduino Mega."""

from math import isfinite

from amr_interfaces.msg import EncoderTicks, WheelSpeed
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from serial import SerialException

from .config import BAUDRATE, SERIAL_PORT
from .packet_builder import build_wheel_command
from .packet_parser import parse_encoder_packet
from .pwm import wheel_speed_to_pwm
from .serial_driver import SerialDriver


class SerialInterfaceNode(Node):
    """Publish encoder ticks and send motor PWM only after a command arrives."""

    def __init__(self):
        super().__init__('serial_interface_node')
        self.port = self.declare_parameter('port', SERIAL_PORT).value
        baudrate = self.declare_parameter('baudrate', BAUDRATE).value
        self.full_scale_speed = self.declare_parameter('max_wheel_speed_rad_s', 20.0).value
        self.max_pwm = self.declare_parameter('max_pwm', 255).value
        if not isfinite(self.full_scale_speed) or self.full_scale_speed <= 0:
            raise ValueError('max_wheel_speed_rad_s must be finite and positive')
        if not 1 <= self.max_pwm <= 255:
            raise ValueError('max_pwm must be between 1 and 255')

        try:
            self.serial_driver = SerialDriver(self.port, baudrate)
        except (OSError, SerialException) as exc:
            self.get_logger().error(f'Cannot open Arduino on {self.port} at {baudrate}: {exc}')
            raise
        self.encoder_pub = self.create_publisher(EncoderTicks, '/encoder_ticks', 10)
        self.sent_command = False
        if self.declare_parameter('enable_motor_commands', False).value:
            self.wheel_sub = self.create_subscription(
                WheelSpeed, '/wheel_speed_cmd', self.wheel_command_callback, 1,
            )
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        """Drain complete encoder frames with no waiting for data."""
        for packet in self.serial_driver.read_lines():
            try:
                counts = parse_encoder_packet(packet)
            except ValueError as exc:
                self.get_logger().warning(
                    f'Skipping invalid Arduino line: {exc}', throttle_duration_sec=5.0,
                )
                continue
            self.encoder_pub.publish(EncoderTicks(**counts))

    def wheel_command_callback(self, msg):
        """Translate a wheel-speed message into one serial PWM frame."""
        speeds = (msg.front_left, msg.front_right, msg.rear_left, msg.rear_right)
        if not all(map(isfinite, speeds)):
            speeds = (0.0,) * 4
        pwms = [wheel_speed_to_pwm(speed, self.full_scale_speed, self.max_pwm)
                for speed in speeds]
        self.serial_driver.write(build_wheel_command(*pwms))
        self.sent_command = True

    def close(self):
        """Stop commanded motion when shutting down normally."""
        try:
            if self.sent_command:
                self.serial_driver.write(build_wheel_command(0, 0, 0, 0))
        except (OSError, SerialException) as exc:
            self.get_logger().warning(f'Arduino disconnected, relying on firmware timeout: {exc}')
        finally:
            self.serial_driver.close()


def main(args=None):
    """Run the bridge and release the serial device on exit."""
    rclpy.init(args=args)
    node = None
    try:
        node = SerialInterfaceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (OSError, SerialException) as exc:
        if node is not None:
            node.get_logger().error(f'Arduino communication failed on {node.port}: {exc}')
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
