#!/usr/bin/env python3
"""Calibrate the Yahboom accelerometer and gyroscope with the sensor flat."""

import argparse

from serial import SerialException
from YbImuLib import YbImuSerial


def calibrate(magnetometer=False):
    """Run exactly one calibration with exclusive access to the selected port."""
    parser = argparse.ArgumentParser(description=(
        'Optional magnetometer calibration' if magnetometer else __doc__
    ))
    parser.add_argument('port', nargs='?', default='/dev/myimu')
    args = parser.parse_args()
    try:
        imu = YbImuSerial(args.port, exclusive=True)
        imu.create_receive_threading()
        print(f'CONTAINER IMU firmware: {imu.get_version() or "unknown"}')
        if magnetometer:
            print('CONTAINER Rotate the IMU through many orientations. Timeout is about 90 s.')
            imu.calibration_mag(timeout_ms=90_000)
            expected = imu.FUNC_CALIB_MAG
        else:
            print('CONTAINER Keep the IMU flat and still during calibration.')
            imu.calibration_imu()
            expected = imu.FUNC_CALIB_IMU
    except (OSError, SerialException) as exc:
        print(f'CONTAINER Cannot use {args.port}: {exc}. Stop the IMU driver and check the port.')
        return 1

    state = imu._rx_state if imu._rx_func == expected else None
    result = 'success' if state == 1 else 'incomplete' if not state else f'code {state}'
    print(f'CONTAINER Calibration result: {result}')
    return 0 if state == 1 else 1


if __name__ == '__main__':
    raise SystemExit(calibrate())
