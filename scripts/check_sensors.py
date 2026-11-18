#!/usr/bin/env python3
"""Check sustained ROS 2 delivery from the lidar and IMU."""

import argparse
import math
from statistics import mean
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def main():
    """Measure three live sensor topics and return a status for each."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20.0)
    parser.add_argument('--imu-rate-hz', type=float, default=50.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 5.0:
        parser.error('--seconds must be at least 5')
    if not math.isfinite(args.imu_rate_hz) or args.imu_rate_hz <= 0:
        parser.error('--imu-rate-hz must be positive')

    rclpy.init()
    node = rclpy.create_node('amr_sensor_check')
    records = {topic: [] for topic in ('/scan', '/imu/data_raw', '/imu/data')}

    def collect(topic):
        def receive(msg):
            stamp = msg.header.stamp
            records[topic].append((time.monotonic(), (stamp.sec, stamp.nanosec), msg))

        return receive

    try:
        for topic in records:
            kind = LaserScan if topic == '/scan' else Imu
            node.create_subscription(kind, topic, collect(topic), qos_profile_sensor_data)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    passed = True
    for topic, samples in records.items():
        times = [t for t, _, _ in samples]
        stamps = {stamp for _, stamp, _ in samples}
        messages = [msg for _, _, msg in samples]
        frames = {msg.header.frame_id for msg in messages}
        gaps = [right - left for left, right in zip(times, times[1:])]
        rate = len(gaps) / (times[-1] - times[0]) if gaps else 0.0
        max_gap = max(gaps, default=math.inf)
        if topic == '/scan':
            good_ratios = [
                sum(msg.range_min <= d <= msg.range_max for d in msg.ranges)
                / max(len(msg.ranges), 1)
                for msg in messages
            ]
            good = mean(good_ratios) if good_ratios else 0.0
            beams = sorted({len(msg.ranges) for msg in messages})
            details = f'beams={beams}, usable ranges={100 * good:.1f}%'
            frame, min_rate, gap_limit = 'LiDAR_Link', 3.0, 0.8
            valid = good >= 0.10
        else:
            gravity = [math.hypot(*xyz(msg.linear_acceleration)) for msg in messages]
            gyro = [math.hypot(*xyz(msg.angular_velocity)) for msg in messages]
            norms = [math.hypot(*xyz(msg.orientation), msg.orientation.w) for msg in messages]
            g = mean(gravity) if gravity else 0.0
            az = mean(msg.linear_acceleration.z for msg in messages) if messages else 0.0
            omega = mean(gyro) if gyro else 0.0
            details = f'avg accel magnitude={g:.3f}, avg z={az:.3f}, avg gyro magnitude={omega:.4f}'
            frame, min_rate, gap_limit = 'IMU_Link', 0.7 * args.imu_rate_hz, 0.5
            valid = all(math.isfinite(v) for v in (*gravity, *gyro)) and g > 0.1
            if topic == '/imu/data':
                valid = valid and bool(norms) and all(0.95 <= norm <= 1.05 for norm in norms)
                if messages:
                    q = messages[-1].orientation
                    details += f', quaternion xyzw=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})'
        ok = (valid and frames == {frame} and len(stamps) >= min_rate * (args.seconds - 3.0)
              and rate >= min_rate and max_gap <= gap_limit)
        passed &= ok
        print(f'CONTAINER {"PASS" if ok else "FAIL"} {topic}: {len(times)} messages, '
              f'{rate:.2f} Hz, max gap {max_gap:.3f} s, frames={sorted(frames)}, {details}')
        if topic == '/imu/data_raw' and messages:
            if 0.7 <= g <= 1.3:
                print('CONTAINER Hint: acceleration resembles g. Verify accel_scale=9.80665.')
            elif 8.0 <= g <= 11.0:
                print('CONTAINER Acceleration magnitude is consistent with m/s² at rest.')
            print('CONTAINER Verify gyro scale with a measured turn. At rest, both units read near zero.')
    return 0 if passed else 1


def xyz(vector):
    """Return the XYZ components of a ROS vector or quaternion."""
    return vector.x, vector.y, vector.z


if __name__ == '__main__':
    raise SystemExit(main())
