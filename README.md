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
    -> GMapping

GMapping
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

GMapping consumes best-effort `sensor_msgs/msg/LaserScan` messages on `/scan`.
It looks up the LiDAR pose at each scan timestamp through
`odom → base_link → LiDAR_Link`. The X3 configuration keeps the 180° bearing
correction and an 8 m range limit. The EKF fuses encoder planar velocity and yaw
rate with IMU yaw rate. GMapping receives the resulting motion through TF.

GMapping publishes `nav_msgs/msg/OccupancyGrid` on `/map` at 0.05 m resolution
and broadcasts `map → odom`. The map is reliable and transient local so Nav2
and RViz can receive the latest grid when they join after mapping has started.

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
| `map -> odom` | GMapping, or AMCL for an existing map |
| `base_link -> LiDAR_Link` | `robot_state_publisher` from `amr.urdf` |
| `base_link -> IMU_Link` | `robot_state_publisher` from `amr.urdf` |
| `base_link -> wheel links` | `robot_state_publisher` from `amr.urdf` |

Do not run GMapping mapping and AMCL at the same time. Both provide
`map -> odom`, so they will compete for that transform.

## Frame Naming

The repository uses these consistent physical sensor frames:

| Component | Current frame |
|---|---|
| URDF LiDAR link | `LiDAR_Link` |
| URDF IMU link | `IMU_Link` |
| YDLIDAR parameters and scan messages | `LiDAR_Link` |
| YBIMU and filtered IMU messages | `IMU_Link` |
| Encoder odometry | `odom -> base_link` |

Do not add a YDLIDAR static-transform publisher: `robot_state_publisher`
already publishes the URDF transform from `base_link` to `LiDAR_Link`.

The URDF places the LiDAR relative to `base_link` at approximately:

```text
x = 0.091609 m
y = 0.00013794 m
z = 0.192 m
```

An extra transform with a different parent-child name or pose will create TF
conflicts.

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

- [`amr_bringup`](ros2_ws/src/amr_bringup)
  Configurable launch file for starting the robot description, control,
  odometry, hardware, IMU, LiDAR, and optional Robot Localization EKF.

- [`YDLidar-SDK`](ros2_ws/src/YDLidar-SDK)
  YDLIDAR device SDK included as a Git submodule.

Arduino motor and encoder test firmware is available in [`scripts`](scripts).

## Humble Docker image

Run the project inside Docker on both machines. The VM can keep ROS 2 Lyrical
on its host. The project scripts use ROS 2 Humble in the container with
`ROS_DOMAIN_ID=67` by default on both machines. Use host networking and
synchronize the Jetson and VM clocks so scan timestamps match the TF history.

Run `./scripts/build_image.sh` from the repository root on each machine after
switching to this branch and after changes to the Dockerfile or its source
patches. The startup scripts build the image if it is missing, replace this
checkout's container after an image rebuild, and build the mounted workspace
before launching.

The image builds [GMHadou/slam_gmapping_Humble](https://github.com/GMHadou/slam_gmapping_Humble/tree/7ffe2f768bcd75b5158a41937fcc2abd33173143)
at revision `7ffe2f768bcd75b5158a41937fcc2abd33173143`, including its
`openslam_gmapping` package. This port is based on
[Project-MANAS/slam_gmapping](https://github.com/Project-MANAS/slam_gmapping)
and already publishes a transient-local map. The project patch uses
`SensorDataQoS` for the X3 scan, preserves subsecond timing, and applies the
configured map update interval across scans.
GMapping is absent from the official Humble distribution. Humble's
`nav2_bringup` can still install SLAM Toolbox transitively. The AMR live
launches use GMapping.

## Split startup and stationary verification

On the Jetson, from the repository root:

```bash
./scripts/start_robot.sh use_ekf:=true explore:=true enable_motor_commands:=false lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 arduino_port:=/dev/ttyUSB2
```

On the Ubuntu VM, from the repository root in a graphical session:

```bash
./scripts/start_vm.sh
```

The Jetson runs the sensors, Arduino bridge, EKF, and frontier explorer. The VM
runs GMapping, Nav2, and RViz. Use this pairing for split operation. Running
`start.sh` on the Jetson alongside `start_vm.sh` can duplicate the mapper and
Nav2 action servers on domain 67.

Use a second VM terminal and run `./scripts/shell.sh --vm`. In the container,
check the scan and map QoS, the single live map publisher, and the TF chain:

```bash
ros2 topic info -v /scan
ros2 topic info -v /map
ros2 topic echo /map --once --field info --qos-durability transient_local
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map LiDAR_Link
ros2 topic echo /odometry/filtered --once
ros2 topic echo /global_costmap/costmap --once --field info --qos-durability transient_local
ros2 action info /navigate_to_pose
```

In RViz, use `map` as the fixed frame and confirm the scan, map, and robot
overlay. The explorer starts paused. With motor commands disabled, select
**Explore** to check for a frontier goal when the map has reachable unknown
space, then select **RViz Goals** to pause exploration and try a **Nav2 Goal**.
A goal that requires movement cannot finish during this stationary check.

For an authorized, supervised motion test after the stationary checks pass,
stop the Jetson launch and repeat its command with
`enable_motor_commands:=true`. Keep the VM launch running. Start with a short
RViz goal in a clear area before selecting **Explore**. The RViz panel also
provides **Manual Drive** and hold-to-rotate controls.

## All-in-one Jetson startup

Use `start.sh` by itself when the Jetson will run the whole stack. Use a local
graphical session for RViz, or pass `rviz:=false` for headless use.

```bash
./scripts/start.sh use_ekf:=true explore:=true rviz:=true enable_motor_commands:=false lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 arduino_port:=/dev/ttyUSB2
```

## Save a map and navigate with AMCL

While mapping is running, open a second container shell with
`./scripts/shell.sh --vm` on the split VM or `./scripts/shell.sh` on the
all-in-one Jetson. Save the occupancy grid into the mounted workspace:

```bash
mkdir -p /workspace/maps
ros2 run nav2_map_server map_saver_cli -f /workspace/maps/amr --ros-args -p map_subscribe_transient_local:=true
```

Keep `ros2_ws/maps/amr.yaml` and its image together. For split operation, stop
the VM mapping launch and restart the Jetson command with `explore:=false`.
Then run from the VM repository root:

```bash
./scripts/start_vm.sh mode:=navigation map:=/workspace/maps/amr.yaml
```

For all-in-one operation, stop the live launch and repeat the Jetson
`start.sh` command with `mode:=navigation explore:=false
map:=/workspace/maps/amr.yaml`. The map server publishes `/map` and AMCL owns
`map → odom`. Use **2D Pose Estimate** in RViz to seed AMCL before sending
a **Nav2 Goal**.

Inside a sourced container, `autonomy.launch.py` selects the mode and
`start_robot:=false` selects an external robot. The standalone
`slam.launch.py` starts GMapping with local sensors and an optional base.
`mapping.launch.py` starts GMapping against external scans and TF, with
optional RViz and keyboard teleoperation. `navigation.launch.py` launches
saved-map navigation against an external robot.

## Hardware Defaults

The AMR uses the following fixed serial device assignments:

```text
LiDAR:    /dev/ttyUSB0
IMU:      /dev/ttyUSB1
Arduino:  /dev/ttyUSB2
Baudrate: 115200 (Arduino and LiDAR)
```

The YDLIDAR configuration is stored in
[`ros2_ws/src/ydlidar_ros2_driver-master/params`](ros2_ws/src/ydlidar_ros2_driver-master/params).
Robot dimensions, wheel parameters, and ROS topic names are configurable
through the YAML files in the individual packages.

## License

The project source code is released under the Apache License 2.0 unless a
component contains its own license. Third-party components, including the
YDLIDAR SDK and ROS 2 driver, retain their respective licenses.