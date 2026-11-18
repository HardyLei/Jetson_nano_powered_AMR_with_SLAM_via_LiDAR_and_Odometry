#!/usr/bin/env python3
"""Optional magnetometer calibration for modules with that sensor."""

from YbImu_Calibrate_IMU import calibrate


if __name__ == '__main__':
    raise SystemExit(calibrate(magnetometer=True))
