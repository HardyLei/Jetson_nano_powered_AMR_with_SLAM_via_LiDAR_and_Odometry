# Jetson Nano AMR with lidar SLAM and Nav2

This ROS 2 Humble workspace connects a Jetson Nano to a YDLIDAR X3, a Yahboom
IMU, and a four-wheel mecanum base with an Arduino Mega. The AMR has two
operating modes.

- **Mapping** runs SLAM Toolbox and Nav2 together. SLAM Toolbox builds a 2D map
  from lidar scans and odometry. Nav2 drives to RViz goals as the map grows.
  The optional frontier explorer selects goals automatically.
- **Navigation** loads a map saved during mapping, localizes with AMCL, and
  drives to a goal selected in RViz.

Start with the [quickstart](docs/QUICKSTART.md). Use the
[hardware test plan](docs/HARDWARE_TEST_PLAN.md) before enabling the motors and
the [troubleshooting guide](docs/TROUBLESHOOTING.md) if a check fails.

Wheel counts feed `/odom`. The optional EKF combines wheel velocity with IMU
yaw rate from `/imu/data` and publishes `/odometry/filtered`. In mapping mode,
SLAM Toolbox owns `map → odom`. In saved-map mode, AMCL owns that transform.
The selected odometry source owns `odom → base_link`. The URDF provides
`base_link → LiDAR_Link` and `base_link → IMU_Link`.

The lidar, IMU, encoder streams, and hand-turned wheel directions were
verified on the Jetson during bring-up. Physical motion, mapping, and
navigation still need the tests in the hardware test plan.

## License

Project source is Apache-2.0 unless a component has its own license.
The YDLIDAR SDK, YDLIDAR driver, Yahboom library, and the frontier explorer
retain their upstream licenses.
