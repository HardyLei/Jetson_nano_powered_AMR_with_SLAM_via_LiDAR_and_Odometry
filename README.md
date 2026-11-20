# Jetson Nano AMR with LiDAR SLAM and Nav2

## Run automatic mapping

Run these commands from the repository root on the Jetson after completing the
[first-time setup](#first-time-setup). Use a clear, bounded area, reliable
battery power, and the [hardware checks](docs/HARDWARE_TEST_PLAN.md) before
enabling motion. Stay within reach of the motor cutoff during the test.

**Terminal 1 (Jetson desktop)** — Start mapping with the frontier explorer.
This example uses the USB assignments observed during testing. Verify them
with `ls -l /dev/serial/by-path/` and change the arguments if needed.

```bash
./scripts/start.sh mode:=mapping explore:=true rviz:=true enable_motor_commands:=true lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 arduino_port:=/dev/ttyUSB2
```

Wait for the map to appear and for Nav2 to report that its managed nodes are
active. Leave this terminal running. The explorer starts paused.

In RViz's **Exploration** panel, click **Explore** to start autonomous mapping.
To drive to a destination you choose, click **RViz Goals**. Wait for the panel
to show **Explorer paused**, then use **Nav2 Goal** to pick a clear point.
Click **Explore** again to return to autonomous mapping. SLAM Toolbox keeps
updating the same map as you switch modes.

When running over SSH without a display, omit `rviz:=true` and use the second
terminal below to control exploration.

**Terminal 2 (Jetson host, then container)** — Open the ROS shell, then send
the start signal at the container prompt when the area is clear.

```bash
./scripts/shell.sh
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: true}'
```

The AMR chooses where to go while SLAM Toolbox builds the map. To pause
exploration, run this in Terminal 2:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'
```

To resume, send `{data: true}` again. If no eligible frontiers remain, the
explorer lets the active Nav2 goal finish, then selects a nearby clear viewpoint
in the mapped area. It switches back to frontier goals as soon as one becomes
eligible. If the costmap has no reachable viewpoint, the robot waits while the
explorer keeps checking. Use the pause command when you're done surveying.

If the publish command waits for a matching
subscription, press **Ctrl+C** and check Terminal 1 for explorer startup
errors and confirm that you launched with `explore:=true`.

## First-time setup

The workspace runs ROS 2 Humble in Docker on the Jetson Nano. It uses a
YDLIDAR X3, Yahboom IMU, Arduino Mega, and four-wheel mecanum base. On the
Jetson host, clone the project branch and build its image:

```bash
git clone --branch rafay_modify_1 --recurse-submodules https://github.com/HardyLei/Jetson_nano_powered_AMR_with_SLAM_via_LiDAR_and_Odometry.git ~/amr_ws
cd ~/amr_ws
./scripts/setup_host.sh
./scripts/build_image.sh
```

The [quickstart](docs/QUICKSTART.md) explains Docker access, RViz, and stable
serial names. The defaults are `/dev/ydlidar`, `/dev/myimu`, and `/dev/amr`
when configured with udev. Numbered USB ports can change after reconnecting.

For a stationary check, launch with `enable_motor_commands:=false`. From a
second Jetson terminal, run `./scripts/doctor.sh` with the same port
overrides to check the device streams, odometry, map, and Nav2. Once powered
motion passes the hardware checks, a short **Nav2 Goal** in known free space
can verify the drive before you start hands-off exploration.

## Save a map and navigate on it

Pause exploration and wait for the robot to stop. From the container shell in
Terminal 2, save the map:

```bash
mkdir -p /workspace/maps
ros2 run nav2_map_server map_saver_cli -f /workspace/maps/auto-demo
```

The map image and YAML are saved in `ros2_ws/maps/` on the Jetson host. Stop
mapping with **Ctrl+C** in Terminal 1, then launch saved-map navigation:

```bash
./scripts/start.sh mode:=navigation map:=/workspace/maps/auto-demo.yaml rviz:=true enable_motor_commands:=true lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 arduino_port:=/dev/ttyUSB2
```

In RViz, use **2D Pose Estimate** to set the robot's starting pose. Once the
scan lines up with the saved map, use **Nav2 Goal** to select a destination.

## How the code fits together

| Component | Role |
| --- | --- |
| `amr_bringup` | Launches the sensors, robot transforms, drive base, and autonomy stack. |
| YDLIDAR and Yahboom drivers | Publish `/scan` and `/imu/data_raw`. The Madgwick filter publishes `/imu/data`. |
| `amr_hardware` | Sends signed PWM commands to the Arduino and publishes encoder counts on `/encoder_ticks`. Per-wheel PWM floors help overcome starting friction, and zero commands still send zero PWM. |
| `amr_control` | Converts `/cmd_vel` to mecanum wheel commands and publishes wheel odometry on `/odom`. An optional EKF fuses IMU yaw rate with wheel odometry. |
| `amr_description` | Provides the URDF and meshes for RViz, including the lidar mounting transform checked on the physical AMR. |
| SLAM Toolbox, Nav2, and the frontier explorer | During mapping, SLAM Toolbox builds `/map`, the explorer proposes goals at the edge of the explored area, and Nav2 plans and drives. Saved-map mode uses AMCL to localize on the map. |

The odometry source publishes `odom → base_link`. SLAM Toolbox supplies
`map → odom` during mapping, and AMCL supplies it during saved-map navigation.

## Test status

- On the Jetson, live lidar, IMU, and encoder data were verified. The lidar
  orientation was checked against the robot in RViz. All four encoder
  channels responded, and a 1-meter test reported about 0.96 m forward.
- Physical exploration started and moved the AMR. The observed run stopped
  after Nav2 could not plan to its remaining frontier. The per-wheel PWM
  floors are based on lifted-wheel tests and may need tuning under load.
- Live SLAM with Nav2, explorer start and pause, and saving and navigating on
  a map passed in simulation. Saved-map navigation still needs a complete
  physical run.

See the [hardware test plan](docs/HARDWARE_TEST_PLAN.md) and
[troubleshooting guide](docs/TROUBLESHOOTING.md) for diagnostics.

## License

Project source is Apache-2.0 unless a component has its own license.
The YDLIDAR SDK, YDLIDAR driver, Yahboom library, and frontier explorer
retain their upstream licenses.
