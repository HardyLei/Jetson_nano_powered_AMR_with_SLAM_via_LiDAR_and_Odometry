# Jetson Nano Powered AMR with SLAM via LiDAR and Odometry

This project is a ROS 2 Humble software stack for an autonomous mobile robot
(AMR) powered by a Jetson Nano. The robot uses a four-wheel mecanum drive,
wheel encoders, an Arduino Mega motor controller, a YBIMU inertial sensor, and
a YDLIDAR laser scanner to support omnidirectional movement, odometry
estimation, visualization, SLAM, and navigation.

## Features

- Mecanum-drive inverse kinematics for forward, lateral, and rotational motion.
- ROS 2 velocity control through the `/cmd_vel` topic.
- Serial communication between the Jetson Nano and Arduino Mega.
- Custom ROS 2 messages for wheel commands and encoder feedback.
- Encoder-based odometry published on `/odom`.
- Automatic `odom -> base_link` TF broadcasting.
- YBIMU integration for IMU, orientation, magnetic-field, Euler-angle, and
  barometric data.
- YDLIDAR integration for laser scans and mapping.
- URDF robot model with visual and collision meshes for the chassis, wheels,
  LiDAR, and IMU.
- Support for RViz visualization, SLAM, and Navigation2 workflows.

## Architecture

```text
/cmd_vel
    |
    v
Mecanum Controller
    |
    v
/wheel_speed_cmd
    |
    v
Jetson Nano <--> Arduino Mega Serial Link
    |
    v
Motor Drivers and Wheel Encoders
    |
    v
/encoder_ticks
    |
    v
Odometry Node
    |
    +--> /odom
    +--> odom -> base_link TF
```

The LiDAR supplies laser-scan data for perception and mapping. The IMU
publishes inertial and orientation measurements that can be used with wheel
odometry by localization and navigation components.

## ROS 2 Packages

- [`amr_control`](ros2_ws/src/amr_control)
  Mecanum controller and encoder-based odometry nodes.

- [`amr_hardware`](ros2_ws/src/amr_hardware)
  Serial communication between the Jetson Nano and Arduino Mega, including
  wheel command transmission and encoder packet parsing.

- [`amr_interfaces`](ros2_ws/src/amr_interfaces)
  Custom `WheelSpeed` and `EncoderTicks` message definitions.

- [`amr_description`](ros2_ws/src/amr_description)
  URDF, meshes, robot transforms, and visualization resources.

- [`imu_ros2_device`](ros2_ws/src/imu_ros2_device)
  ROS 2 driver for the YBIMU device.

- [`ydlidar_ros2_driver-master`](ros2_ws/src/ydlidar_ros2_driver-master)
  YDLIDAR ROS 2 driver, configuration, launch files, and RViz integration.

- [`YDLidar-SDK`](ros2_ws/src/YDLidar-SDK)
  YDLIDAR device SDK included as a Git submodule.

Arduino motor and encoder test firmware is available in [`scripts`](scripts).

## Containerized Development

The project provides a reproducible ROS 2 Humble development environment
through [`.devcontainer/Dockerfile`](.devcontainer/Dockerfile) and
[`.devcontainer/devcontainer.json`](.devcontainer/devcontainer.json).

The container installs:

- ROS 2 Humble, Colcon, CMake, and build tools
- YDLidar SDK build dependencies
- SLAM Toolbox
- Navigation2 and Nav2 Bringup
- Robot Localization
- `pyserial` and `smbus2`
- The local YBIMU Python library

The ROS 2 workspace is mounted at `/workspace`. Host networking and privileged
container access are enabled for serial devices, LiDAR communication, and
hardware testing.

## Building

Open the project in the provided development container, then build the ROS 2
workspace:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Useful Launch Commands

Launch the mecanum controller:

```bash
ros2 launch amr_control mecanum_controller.launch.py
```

Launch encoder odometry:

```bash
ros2 launch amr_control odometry.launch.py
```

Launch the hardware serial interface:

```bash
ros2 launch amr_hardware hardware.launch.py
```

Launch the YDLIDAR driver or its RViz visualization configuration:

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
ros2 launch ydlidar_ros2_driver view_ydlidar_launch.py
```

## Hardware Defaults

The AMR hardware interface uses the following default serial configuration:

```text
Port:     /dev/ttyUSB0
Baudrate: 115200
```

The YDLIDAR configuration is stored in
[`ros2_ws/src/ydlidar_ros2_driver-master/params`](ros2_ws/src/ydlidar_ros2_driver-master/params).
Robot dimensions, wheel parameters, and ROS topic names are configurable
through the YAML files in the individual packages.

## License

The project source code is released under the Apache License 2.0 unless a
component contains its own license. Third-party components, including the
YDLIDAR SDK and ROS 2 driver, retain their respective licenses.