"""Publish serial Yahboom IMU measurements."""

from math import isfinite
from time import monotonic

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from serial import SerialException
from std_msgs.msg import Float32MultiArray
from YbImuLib import YbImuSerial


class YbImuDriver(Node):
    """Read the latest measurements cached by the Yahboom library."""

    def __init__(self):
        super().__init__('ybimu_node')
        port = self.declare_parameter('port', '/dev/myimu').value
        rate = self.declare_parameter('rate_hz', 50.0).value
        self.frame_id = self.declare_parameter('frame_id', 'IMU_Link').value
        # YbImuLib returns acceleration in g and gyro in rad/s.
        self.accel_scale = self.declare_parameter('accel_scale', 9.80665).value
        self.max_sample_age = self.declare_parameter('max_sample_age_sec', 0.5).value
        self.gyro_scale = self.declare_parameter('gyro_scale', 1.0).value
        if not isfinite(rate) or rate <= 0.0:
            raise ValueError('rate_hz must be finite and positive')
        self.covariances = {}
        for field, default in (
            ('orientation', 0.04), ('angular_velocity', 0.0025),
            ('linear_acceleration', 0.04),
        ):
            value = self.declare_parameter(f'{field}_variance', default).value
            self.covariances[field] = [value, 0., 0., 0., value, 0., 0., 0., value]

        if not isfinite(self.max_sample_age) or self.max_sample_age <= 0.0:
            raise ValueError('max_sample_age_sec must be finite and positive')
        self.port = port
        try:
            self.robot = YbImuSerial(port, exclusive=True)
        except (OSError, SerialException) as exc:
            self.get_logger().error(f'Cannot open IMU on {port}: {exc}')
            raise
        self.robot.create_receive_threading()
        version = self.robot.get_version() or 'unknown'
        self.declare_parameter('firmware_version', version, ParameterDescriptor(read_only=True))
        self.get_logger().info(f'IMU opened on {port}. Firmware: {version}')
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.baro_pub = self.create_publisher(Float32MultiArray, '/baro', 10)
        self.euler_pub = self.create_publisher(Float32MultiArray, '/euler', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_data)

    def publish_data(self):
        """Publish the core data even when an optional sensor fails."""
        last_sample = self.robot.last_raw_data_time
        if last_sample is None or monotonic() - last_sample > self.max_sample_age:
            self.get_logger().warning(
                f'No recent raw IMU frame on {self.port}. Check USB and module power.',
                throttle_duration_sec=5.0,
            )
            return

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id
        ax, ay, az = self.robot.get_accelerometer_data()
        gx, gy, gz = self.robot.get_gyroscope_data()
        qw, qx, qy, qz = self.robot.get_imu_quaternion_data()
        imu.linear_acceleration.x = ax * self.accel_scale
        imu.linear_acceleration.y = ay * self.accel_scale
        imu.linear_acceleration.z = az * self.accel_scale
        imu.angular_velocity.x = gx * self.gyro_scale
        imu.angular_velocity.y = gy * self.gyro_scale
        imu.angular_velocity.z = gz * self.gyro_scale
        imu.orientation.w = qw
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        for field, covariance in self.covariances.items():
            setattr(imu, f'{field}_covariance', covariance)
        if not any((qw, qx, qy, qz)):
            # A zero quaternion means the module has yet to report orientation.
            imu.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu)

        try:
            mx, my, mz = self.robot.get_magnetometer_data()
            mag = MagneticField()
            mag.header = imu.header
            mag.magnetic_field.x = mx
            mag.magnetic_field.y = -my
            mag.magnetic_field.z = mz
            self.mag_pub.publish(mag)
        except Exception as exc:
            self.get_logger().warning(f'Magnetometer unavailable: {exc}', once=True)

        try:
            self.baro_pub.publish(Float32MultiArray(data=self.robot.get_baro_data()))
        except Exception as exc:
            self.get_logger().warning(f'Barometer unavailable: {exc}', once=True)

        self.euler_pub.publish(Float32MultiArray(data=self.robot.get_imu_attitude_data(True)))


def main(args=None):
    """Start the serial IMU node."""
    rclpy.init(args=args)
    node = None
    try:
        node = YbImuDriver()
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
