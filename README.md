# Jetson Nano Powered AMR with LiDAR and Odometry

This project controls a four-wheel mecanum autonomous mobile robot (AMR)
using ROS 2 Humble. The **Jetson Nano** reads the sensors and drives the
robot. The **Arduino Mega** handles wheel PWM, encoder counting, and the door
servo. An **Ubuntu VM** can run live mapping, navigation, and RViz while the
Jetson stays connected to the hardware.

- [Run the robot and VM](#run-the-robot-and-vm)
- [How the startup scripts work](#how-the-startup-scripts-work)
- [System architecture](#system-architecture)
- [Repository layout and packages](#repository-layout-and-packages)
- [Libraries and dependencies](#libraries-and-dependencies)
- [Save and reuse a map](#save-and-reuse-a-map)
- [Troubleshooting](#troubleshooting)

## Run the robot and VM

### 1. Prepare both machines

Install Git and Docker Engine with Buildx on the Jetson and the Ubuntu VM, and
make sure your user can run Docker. Run the VM commands from a terminal in its
graphical desktop. On Ubuntu, `sudo apt install x11-xserver-utils` provides
`xhost`, which the startup script uses to authorize the RViz window when X11
is available. [Docker installation on Ubuntu](https://docs.docker.com/engine/install/ubuntu/) · [Docker access for your user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

On each machine, clone the branch and build the image. If you already have the
checkout, run the build command from its root.

```bash
git clone --branch gmapping https://github.com/HardyLei/Jetson_nano_powered_AMR_with_SLAM_via_LiDAR_and_Odometry.git
cd Jetson_nano_powered_AMR_with_SLAM_via_LiDAR_and_Odometry
./scripts/build_image.sh
```

Build on each machine so its binaries match its CPU. Rebuild the image when
the [Dockerfile](.devcontainer/Dockerfile) or its patches change. The startup
scripts rebuild the mounted ROS workspace on launch, so source changes inside
`ros2_ws/src/` can be picked up by restarting.

Put the Jetson and VM on a network where ROS 2 discovery and data can pass.
Both scripts default to **ROS domain 67**. If necessary, set
`AMR_ROS_DOMAIN_ID` to the same value on both. Use bridged networking on the
Jetson's LAN when the VM's shared network prevents ROS discovery. Synchronize
both clocks so the mapper can match each scan to the corresponding transform.

### 2. Start the Jetson

Flash [`motor_encoder_deployment.ino`](scripts/motor_encoder_deployment.ino)
to the Arduino Mega. Install the Arduino **Servo** library if the IDE
requests it. The deployment sketch provides both the wheel protocol and the
A3 door servo. If you previously uploaded
[`servo_script.ino`](scripts/servo_script.ino) for a bench test, flash the
deployment sketch again for normal operation.

Identify the LiDAR, IMU, and Arduino serial devices on the Jetson:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-path/ 2>/dev/null
```

Device numbers can change after reconnecting USB hardware. Replace the three
example paths below with the ones on your Jetson. The split robot script
requires an explicit `arduino_port` argument.

```bash
./scripts/start_robot.sh \
  use_ekf:=true explore:=true enable_motor_commands:=false \
  lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 arduino_port:=/dev/ttyUSB2
```

`enable_motor_commands:=false` keeps the wheel bridge from sending motor
commands, while sensors, encoders, odometry, and the separately controlled
door servo can still run.

Face recognition starts automatically on the Jetson **host**. It needs the
JetPack build of OpenCV with GStreamer, Python's `face_recognition` package,
and a CSI camera. If the host Python packages are missing, install them with
`python3 -m pip install face_recognition pyserial`. Keep JetPack's OpenCV build
for the CSI camera pipeline. If the script reports a missing Haar detector XML, install
`opencv-data` on the Jetson or supply the XML with `--cascade` when running
the script manually. To bring up the robot without opening the camera, prefix
the Jetson command with `AMR_FACE_RECOGNITION=0`.

### 3. Start the VM

From the repository root in the Ubuntu desktop:

```bash
./scripts/start_vm.sh
```

Leave both terminals running. In RViz, confirm that the **Fixed Frame** is
`map` and that the laser points line up with the map walls. The explorer
starts paused. Use **Explore** to send frontier goals or **RViz Goals** to
send your own goal. **Manual Drive** provides keyboard movement and
hold-to-rotate buttons.

Use a second VM terminal to check data coming from the robot:

```bash
./scripts/shell.sh --vm
```

Inside the container shell:

```bash
timeout 15s ros2 topic echo /scan --once --field header --qos-reliability best_effort
timeout 15s ros2 topic echo /map --once --field info --qos-durability transient_local
ros2 run tf2_ros tf2_echo map LiDAR_Link
```

A scan with `frame_id: LiDAR_Link`, map metadata, and repeated transforms
confirm that the inputs for the stationary mapping check are reaching the VM.
Stop `tf2_echo` with Ctrl+C. For a supervised driving test after those checks
pass, restart the Jetson launch with `enable_motor_commands:=true` and leave
the VM running.

### Stop or restart

Stop each foreground launch with Ctrl+C. If you closed the launch terminal and
its processes are still running, use `docker stop amr-vm` on the VM or
`docker stop amr` on the Jetson. Then rerun that machine's start command.

## How the startup scripts work

| Entry point | What it launches |
| --- | --- |
| [`start_robot.sh`](scripts/start_robot.sh) | Calls `start.sh --robot`. The `amr` container runs `robot.launch.py`, which starts LiDAR, IMU, robot transforms, the Arduino bridge, wheel control, odometry, and the optional EKF and frontier explorer. The wrapper defaults to `use_ekf:=true` and `explore:=true`. |
| [`start_vm.sh`](scripts/start_vm.sh) | Calls `start.sh --vm`. The `amr-vm` container runs `autonomy.launch.py` with `start_robot:=false` and `explore:=false`, so the VM starts the mapper or saved-map localization, Nav2, and RViz. It defaults to filtered odometry and enables RViz. |
| [`start.sh`](scripts/start.sh) | Shared startup code. With no profile flag it runs the combined hardware and autonomy stack on one Jetson. The shell-driven face recognition starts in the `--robot` profile. |
| [`build_image.sh`](scripts/build_image.sh) | Builds `amr:latest` from the repository Dockerfile. |
| [`run_container.sh`](scripts/run_container.sh) | Starts or reuses a container for this checkout. It uses host networking, mounts `ros2_ws/` at `/workspace`, and shares the X11 socket when available. The robot profile also mounts the host's `/dev` for serial access. |
| [`shell.sh`](scripts/shell.sh) | Opens a shell with ROS Humble and the project overlays sourced. Use `--vm` to open `amr-vm`. |

The shared launcher validates serial paths for a hardware launch, builds with
`colcon --symlink-install` inside the selected container, and starts the
launch file with that container's launch lock. [`common.sh`](scripts/common.sh)
holds the container names, ROS domain default, and environment setup. Face
recognition opens the Jetson CSI camera outside the container and is stopped
when the robot startup script exits.

For an all-in-one setup, run this on the Jetson by itself, with your actual
USB paths:

```bash
./scripts/start.sh \
  use_ekf:=true explore:=true rviz:=true enable_motor_commands:=false \
  lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 arduino_port:=/dev/ttyUSB2
```

Pass `rviz:=false` for a headless combined launch. Use either the combined
launch or the split Jetson-plus-VM pair. Starting both can duplicate the
mapper and Nav2 action servers.

## System architecture

```text
JETSON                                            VM
X3 LiDAR ────────── /scan ────────────────────► GMapping and Nav2
Yahboom IMU ─┐
             ├─► EKF ── /odometry/filtered ───► Nav2
Wheel odom ──┘      └── odom → base_link ─────► GMapping, Nav2, RViz

Arduino ◄── Serial bridge ◄── Mecanum controller ◄── /cmd_vel ◄── Nav2
Arduino ───► Serial bridge ──► /encoder_ticks ───► Wheel odom

Frontier explorer ◄── global costmap ──────────── Nav2
Frontier explorer ── NavigateToPose goal ───────► Nav2
RViz ── NavigateToPose goal ────────────────────► Nav2
GMapping ── /map and map → odom ────────────────► Nav2 and RViz
```

### Drive, odometry, and navigation

- **Drive.** The mecanum controller converts forward, sideways, and turn
  velocity requests into four wheel speeds. It arbitrates between Nav2 and
  RViz manual control, applies velocity limits, and sends zero wheel speeds
  when commands expire. The serial bridge converts `/wheel_speed_cmd` into
  newline-terminated `CMD,fl,fr,rl,rr` packets at **115200 baud**. The Mega
  uses the requested sign for direction and the magnitude to set initial PWM.
  Its feedback loop then regulates active wheels toward a fixed encoder-count
  target. The Mega cuts wheel output after **500 ms** without a valid wheel
  command.
- **Odometry.** The Mega sends cumulative encoder counts in
  `ENC,fl,fr,rl,rr` packets, normally every 50 ms. The bridge publishes
  `/encoder_ticks`. The odometry node uses the configured wheel radius,
  geometry, and counts per revolution to estimate pose and velocity on
  `/odom`. With `use_ekf:=true`, Robot Localization fuses the encoder-based
  planar velocity and yaw rate with IMU yaw rate and publishes
  `/odometry/filtered`. The Madgwick filter produces `/imu/data` from
  `/imu/data_raw`.
- **Mapping.** The X3 driver publishes best-effort `/scan` in `LiDAR_Link`,
  with the configured 180° bearing correction and an 8 m scan limit.
  GMapping looks up the LiDAR pose in TF at each scan's timestamp, builds a
  5 cm occupancy grid, and publishes the latest `/map` with reliable,
  transient-local QoS so late subscribers can receive it.
- **Navigation.** Nav2 uses the map, scans, transforms, and odometry to plan
  and follow paths. Its global costmap includes the occupancy grid and
  obstacles. Its rolling local costmap tracks nearby obstacles. RViz goals
  and the frontier explorer both use Nav2's `NavigateToPose` action. The
  explorer chooses boundaries between known free space and unexplored space.

### Coordinate frames and ownership

TF is the ROS representation of how coordinate frames move relative to one
another. `base_link` is attached to the chassis, `odom` is the local
odometry frame, and `map` is the frame of the occupancy grid.

```text
map → odom → base_link
                  ├── LiDAR_Link
                  ├── IMU_Link
                  └── wheel links
```

| Transform | Publisher |
| --- | --- |
| `map → odom` | GMapping in live mapping mode. AMCL in saved-map mode. |
| `odom → base_link` | Robot Localization when `use_ekf:=true`. Wheel odometry when `use_ekf:=false`. |
| `base_link → sensor and wheel links` | `robot_state_publisher` using the URDF and joint states. |

### Face recognition and the door

[`Face_Recognition.py`](scripts/Face_Recognition.py) uses OpenCV's Haar
detector to find faces and `face_recognition` to compare them with reference
photos in [`scripts/known_faces/`](scripts/known_faces). The filename stem is
the label. A newly recognized face sends one `DOOR,OPEN` request over a
local socket shared with the Arduino bridge in the container. The bridge owns
the USB serial device for both the wheels and door.

The deployment sketch drives the door servo on **A3** to **126° for five
seconds**, then returns it to **0°** without blocking the wheel loop. One
continuous appearance triggers one cycle. Unknown faces send no request. The
wheel watchdog tracks valid wheel commands independently of door commands.

For a standalone camera preview, stop any other face recognition process and
run `python3 -u scripts/Face_Recognition.py --no-door` on the Jetson host.
For a standalone servo test with the robot stack stopped, pass
`--arduino-port` with the Mega's actual device path. The script waits for
encoder data before sending a door request.

## Repository layout and packages

```text
.devcontainer/                 Dockerfile and patches for source dependencies
scripts/                       Startup, container tools, camera code and firmware
├── known_faces/               Reference images for the Jetson camera
├── Face_Recognition.py        Face enrollment, recognition and door requests
├── motor_encoder_deployment.ino
├── motor_encoder_test.ino     Standalone wheel and encoder test
├── servo_script.ino           Standalone A3 servo sweep test
└── YbImuLib/                  Bundled Yahboom Python device library
ros2_ws/
├── src/                       ROS packages and vendor driver sources
├── build/, install/, log/     Generated on each machine by Colcon
└── maps/                      Saved maps, when created
```

Each ROS package has a `package.xml` describing its dependencies. Python
packages use `setup.py` for installation. C++ packages and the interface
generator use `CMakeLists.txt`.

| Package | What it does and where to start reading |
| --- | --- |
| [`amr_bringup`](ros2_ws/src/amr_bringup) | Assembles the system. `launch/robot.launch.py` combines the sensors, drive base and optional explorer. `launch/autonomy.launch.py` selects live mapping or saved-map localization and starts Nav2. `config/` holds GMapping, EKF, Nav2, explorer and RViz settings. |
| [`amr_control`](ros2_ws/src/amr_control) | `mecanum_controller.py` handles navigation and manual commands. `kinematics.py` converts between chassis and wheel motion. `odometry_node.py` integrates encoder counts and publishes `/odom`. Wheel dimensions and controller limits are in `config/`. |
| [`amr_hardware`](ros2_ws/src/amr_hardware) | `serial_interface_node.py` bridges ROS and the Mega, `serial_driver.py` frames the serial data, and the packet helpers and `pwm.py` parse reports and convert wheel requests into PWM commands. |
| [`amr_interfaces`](ros2_ws/src/amr_interfaces) | Defines `WheelSpeed`, `EncoderTicks`, `ManualDrive`, and `DriveMode` messages, plus the `SetManualDrive` service used by the RViz panel. |
| [`amr_description`](ros2_ws/src/amr_description) | `urdf/amr.urdf` describes the robot links, joints, and sensor placements. `meshes/` contains the visual and collision models. |
| [`amr_rviz`](ros2_ws/src/amr_rviz) | `src/exploration_panel.cpp` implements the RViz controls for **Explore**, **RViz Goals**, **Manual Drive**, and hold-to-rotate. |
| [`imu_ros2_device`](ros2_ws/src/imu_ros2_device) | Uses `YbImuLib` to publish raw IMU, magnetic field, Euler angles and barometer data. Its launch file also starts the Madgwick filter. |
| [`ydlidar_ros2_driver`](ros2_ws/src/ydlidar_ros2_driver-master) | Vendor driver using the YDLidar SDK. `params/ydlidar_x3.yaml` holds the X3 model, serial settings, range, and orientation. |

## Libraries and dependencies

| Dependency | Purpose and where it runs |
| --- | --- |
| ROS 2 Humble, Colcon, Ament and CMake | ROS middleware and build tooling inside the Docker image. Python nodes use `rclpy`. The C++ driver and RViz plugin use `rclcpp`. |
| GMapping and OpenSLAM | Live occupancy grid and map correction. The image builds the [Humble port at `7ffe2f7`](https://github.com/GMHadou/slam_gmapping_Humble/tree/7ffe2f768bcd75b5158a41937fcc2abd33173143). The [project patch](.devcontainer/slam-gmapping-humble.patch) adapts scan QoS and scan/map timing. |
| Nav2, AMCL and Map Server | Planning, control, costmaps, map saving, and localization against a saved map. The default local planner is DWB. |
| Robot Localization, Madgwick and TF2 | Wheel/IMU fusion, IMU orientation filtering, and coordinate transforms. |
| `m-explore-ros2` / `explore_lite` | The Docker image builds a [pinned source revision](https://github.com/robo-friends/m-explore-ros2/tree/326cf8a0b487c34246bb8f3326afbcd69576dc60) with repository patches for paused startup and continued exploration behavior. |
| RViz2, Qt5 and pluginlib | Display and plugin tools used by the custom RViz panel. |
| YDLidar SDK, YbImuLib, pySerial and smbus2 | The SDK backs the X3 driver. The vendored Yahboom library and the Python serial/bus libraries are installed in the container for hardware communication. The Dockerfile can fetch the pinned SDK if the submodule directory is empty. |
| JetPack OpenCV, GStreamer and `face_recognition` | Camera capture, face detection and recognition on the Jetson host. The optional standalone Arduino connection also uses host pySerial. |
| Arduino AVR core and Servo library | Compile and run the Mega firmware, including the A3 servo signal. |

The exact packages, source revisions, and build commands are in
[`.devcontainer/Dockerfile`](.devcontainer/Dockerfile). The VM needs Docker
and a graphical session for RViz. Its ROS packages come from the Humble image.

## Save and reuse a map

While mapping is running, open a second container shell on the VM with
`./scripts/shell.sh --vm` or on an all-in-one Jetson with
`./scripts/shell.sh`. Inside that shell:

```bash
mkdir -p /workspace/maps
ros2 run nav2_map_server map_saver_cli -f /workspace/maps/amr --ros-args -p map_subscribe_transient_local:=true
```

Keep `ros2_ws/maps/amr.yaml` and the generated map image together. For the
split setup, stop the VM mapping launch and restart the Jetson command with
`explore:=false`. Then run on the VM from the repo root:

```bash
./scripts/start_vm.sh mode:=navigation map:=/workspace/maps/amr.yaml
```

For an all-in-one launch, add `mode:=navigation explore:=false
map:=/workspace/maps/amr.yaml` to the Jetson command. The map server publishes
the saved grid and AMCL publishes `map → odom`. In RViz, set **2D Pose
Estimate** before sending a **Nav2 Goal**.

## Troubleshooting

| Symptom | First check |
| --- | --- |
| Docker build says `Could not resolve host: github.com` | Check `getent hosts github.com` on the Ubuntu host. The build needs DNS and internet access to fetch its pinned dependencies. |
| Colcon fails with `JSONDecodeError` in `_read_codemodel` | Check `df -h ros2_ws` and `df -i ros2_ws`. With the launch stopped, move aside the failing package's generated directory under `ros2_ws/build/` and rerun the start script. |
| RViz opens without a scan or map | Verify the Jetson publishes `/scan` in its own container. Then check both ROS domain IDs and whether the VM sees the same scan. |
| Nav2 keeps waiting for `odom → base_link` | Check the Arduino bridge and encoder data on the Jetson, then check the transform on both machines. Match the VM's `use_ekf` setting to the Jetson's. |
| RViz shows motion while the robot is still | Pause autonomous driving. Compare `/encoder_ticks` and `/odometry/filtered` with `tf2_echo odom base_link` and `tf2_echo map odom`. A moving map correction with stable local odometry points toward scan matching. |
| Door request gives `Errno 5` | Check for USB disconnects and re-enumeration on the Jetson. Restart with the Arduino's current port. |
| Recognition works but the door stays closed | Verify the deployed Mega sketch, servo signal on A3, servo power and common ground. The standalone sweep sketch can test the servo path with wheel motor power switched off. |

## License

Project source is licensed under Apache 2.0 unless a component includes its own
license. Third-party components retain their respective licenses.
