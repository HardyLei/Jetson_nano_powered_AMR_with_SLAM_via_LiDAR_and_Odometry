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

## SLAM and Navigation Data Flow

The robot has separate motion, localization, mapping, and navigation data
paths. The hardware and sensor nodes publish ROS 2 topics, while TF describes
the fixed and moving relationships between frames.

```text
Wheel encoders
    -> /encoder_ticks
    -> Odometry Node
    -> /odom
    -> odom -> base_link

YBIMU
    -> /imu/data_raw
    -> IMU Madgwick filter
    -> /imu/data

/odom + /imu/data
    -> Robot Localization EKF
    -> /odometry/filtered
    -> odom -> base_link

YDLIDAR
    -> /scan
    -> SLAM Toolbox

SLAM Toolbox
    -> /map
    -> map -> odom

Nav2
    -> /cmd_vel
    -> Mecanum Controller
    -> /wheel_speed_cmd
    -> Serial Interface
    -> Arduino Mega / motors
```

### Motion command flow

1. Nav2 publishes a velocity command on `/cmd_vel` as a
   `geometry_msgs/Twist`.
2. `mecanum_controller` applies the robot dimensions and mecanum inverse
   kinematics.
3. It publishes four wheel angular velocities on `/wheel_speed_cmd` using the
   custom `WheelSpeed` message.
4. `serial_interface_node` converts those values into serial packets and sends
   them to the Arduino Mega.
5. The Arduino controls the motors and reads wheel encoders.
6. Encoder counts are published on `/encoder_ticks` as custom `EncoderTicks`
   messages.

### Odometry and sensor fusion flow

The existing `odometry_node` integrates `/encoder_ticks` and publishes:

- `/odom` as `nav_msgs/Odometry`
- `odom -> base_link` as a dynamic TF transform

The YBIMU driver publishes raw inertial data on `/imu/data_raw`. When fused
with wheel odometry, Robot Localization can publish:

- `/odometry/filtered` as the filtered pose and velocity
- `odom -> base_link` as the filtered dynamic TF transform

Only one node should publish `odom -> base_link`. When using Robot Localization,
disable TF publication in `odometry_node` and let the EKF be the sole publisher
of this transform. Encoder-only operation is also possible, but its drift and
sideways-motion error will normally be greater.

### SLAM flow

SLAM Toolbox uses the following inputs:

- YDLIDAR `sensor_msgs/LaserScan` messages on `/scan`
- The static LiDAR transform from `base_link`
- Filtered or encoder odometry
- `/tf` and `/tf_static`

During mapping, SLAM Toolbox estimates the robot pose against the generated
occupancy map and publishes:

- `/map` as the occupancy grid
- `map -> odom` as the SLAM correction transform

Robot Localization or the encoder odometry node provides `odom -> base_link`.
SLAM Toolbox provides `map -> odom`. Together they complete the robot pose:

```text
map pose = map -> odom -> base_link
```

### Navigation flow

Nav2 consumes the map, laser scans, TF tree, and filtered odometry. It uses a
global planner to generate a path to a goal and a local controller to follow
that path while avoiding obstacles.

The navigation loop is:

```text
Goal in RViz
    -> Nav2
    -> global and local path planning
    -> /cmd_vel
    -> mecanum_controller
    -> Arduino / motors
```

Nav2 continuously depends on:

- `/map` for the known or live occupancy grid
- `/scan` for obstacle detection
- `/odom` or `/odometry/filtered` for the robot velocity and pose
- `/tf` and `/tf_static` for sensor and robot transforms
- `/goal_pose` or the `NavigateToPose` action for its destination

## TF Tree

The required TF tree for SLAM and navigation is:

```text
map
└── odom
    └── base_link
        ├── LiDAR_Link
        ├── IMU_Link
        └── wheel links
```

A `base_footprint` frame can be inserted between `odom` and `base_link` when
required by Nav2 configuration:

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── LiDAR_Link
            └── IMU_Link
```

`base_footprint` should normally share the XY position and yaw of `base_link`
and have zero Z.

### TF ownership

| Transform | Primary publisher |
|---|---|
| `odom -> base_link` | Encoder odometry, or Robot Localization EKF |
| `map -> odom` | SLAM Toolbox, or AMCL for an existing map |
| `base_link -> LiDAR_Link` | `robot_state_publisher` from `amr.urdf` |
| `base_link -> IMU_Link` | `robot_state_publisher` from `amr.urdf` |
| `base_link -> wheel links` | `robot_state_publisher` from `amr.urdf` |

Do not run SLAM Toolbox mapping and AMCL at the same time. Both provide
`map -> odom`, so they will compete for that transform.

## Frame Naming

The current repository has several different frame names that must be
reconciled before launch:

| Component | Current frame |
|---|---|
| URDF LiDAR link | `LiDAR_Link` |
| URDF IMU link | `IMU_Link` |
| YDLIDAR parameters | `laser` |
| YBIMU driver | `imu_link` |
| Encoder odometry | `odom -> base_link` |

Use one consistent pair in each setup:

- YDLIDAR: `frame_id: LiDAR_Link`
- YBIMU: `frame_id: IMU_Link`
- Robot description: maintain `LiDAR_Link` and `IMU_Link`

Alternatively, standardize the URDF to lowercase `laser` and `imu_link`, and
update the YDLIDAR and YBIMU publishers accordingly. Do not leave the physical
frame names inconsistent.

The URDF places the LiDAR relative to `base_link` at approximately:

```text
x = 0.091609 m
y = 0.00013794 m
z = 0.192 m
```

Do not add a YDLIDAR static-transform publisher for `base_link -> laser` if
`robot_state_publisher` is already publishing the URDF transform
`base_link -> LiDAR_Link`. An extra transform with a different parent-child
name or pose will create TF conflicts.

## Recommended Bringup Sequence

Launch and start the components in this order:

1. `robot_state_publisher` with `amr.urdf`
2. `serial_interface_node`
3. `odometry_node`
4. YBIMU driver and Madgwick IMU filter
5. Robot Localization EKF
6. YDLIDAR driver
7. SLAM Toolbox
8. Nav2
9. RViz2

For mapping, run SLAM Toolbox in mapping mode and use Nav2 navigation goals
from RViz. For navigation using an existing saved map, run:

- `map_server` to load the map
- Nav2
- AMCL for localization, or SLAM Toolbox in localization mode
- No competing mapping/localization TF publisher

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