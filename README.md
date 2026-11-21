# Jetson Nano AMR with LiDAR SLAM and Nav2

## Run on a Jetson and an Ubuntu VM

Use the same checkout on both machines. Run Docker Engine inside the Ubuntu
VM and bridge the VM's network adapter to the Jetson's LAN. Both start scripts
run ROS 2 **Humble** in Docker on domain **67**. On the VM, run from its
desktop terminal so Docker can open RViz on the local display. The VM's
installed ROS 2 Lyrical environment can remain on the host.

| Run on | Script | What it starts |
| --- | --- | --- |
| Jetson Nano | `./scripts/start_robot.sh` | LiDAR, IMU, robot transforms, Arduino bridge, mecanum control, wheel odometry, EKF, and the paused frontier explorer |
| Ubuntu VM | `./scripts/start_vm.sh` | SLAM Toolbox, Nav2 and RViz in the `amr-vm` container |

Stop the all-in-one launch before starting the split setup. Check the USB
assignments on the Jetson and start there with motor output disabled for the
first test.

```bash
./scripts/start_robot.sh enable_motor_commands:=false \
  lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 arduino_port:=/dev/ttyUSB2
```

In the **Ubuntu VM desktop terminal**, start the other half from the repo
root. The first run builds the Humble image if needed.

```bash
./scripts/start_vm.sh
```

Wait for Nav2 to activate and for the map and scan to appear in RViz. The
Jetson publishes `/scan`, `/odometry/filtered`, and `odom → base_link`.
SLAM Toolbox on the VM publishes `map → odom`, and Nav2 sends `/cmd_vel` back
to the Jetson. Keep both machines' clocks synchronized for timestamped scans
and transforms.

From a second terminal on the VM, enter its Humble container to check ROS
discovery and the odometry stream.

```bash
./scripts/shell.sh --vm
printenv ROS_DISTRO ROS_DOMAIN_ID
ros2 topic list
ros2 topic hz /odometry/filtered
```

Use `./scripts/shell.sh` for diagnostics on the Jetson. After the stationary
test, restart the Jetson command with `enable_motor_commands:=true` when ready
to drive. Use **Explore** or **RViz Goals** on the VM just as in the single
Jetson workflow. In this setup the live map is published on the VM, so save it
from the VM shell to store it in that checkout's `ros2_ws/maps/` directory.

## Run automatic mapping

From the repository root in a Jetson desktop terminal, run this command after
completing [first-time setup](#first-time-setup).

```bash
./scripts/start.sh mode:=mapping explore:=true rviz:=true \
  enable_motor_commands:=true use_ekf:=true \
  lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 arduino_port:=/dev/ttyUSB2
```

Check the USB assignments with `ls -l /dev/serial/by-path/` and adjust the
arguments if the devices were renumbered. `use_ekf:=true` combines wheel
odometry with the IMU's yaw rate. Nav2 and SLAM Toolbox use the resulting
motion estimate. For a stationary test, set `enable_motor_commands:=false`.
Use a clear area, reliable battery power, and the
[hardware checks](docs/HARDWARE_TEST_PLAN.md) before driving.
Stay within reach of the motor cutoff during the test.

Leave the launch terminal running. Wait for the live map and scan to appear
in RViz and for Nav2 to report `Managed nodes are active`. The explorer starts
paused.

### Switch modes in RViz

- **Explore** in the **Exploration** panel lets the robot choose destinations
  automatically. It sends Nav2 to frontiers at the edge of the known map.
  When no eligible frontier remains, it finishes its current goal and starts
  visiting clear viewpoints in the mapped area. It gives new frontiers priority
  as they appear. If it finds no reachable viewpoint, it waits and checks again.
- **RViz Goals** pauses the explorer and cancels its active goal. Wait for
  **Explorer paused** and for the robot to stop. Use **Nav2 Goal** in the top
  toolbar and click-drag on a clear spot in the map to set the destination
  and final heading. The robot waits for another instruction after arriving.
  Click **Explore** to return to autonomous exploration.

SLAM Toolbox updates the same live map in both modes. If you close a panel,
reopen it from the **Panels** menu by checking **Exploration** or **Navigation 2**.

### Drive with the keyboard or rotate buttons

Click **Manual Drive** in the Exploration panel to pause exploration, cancel
the current Nav2 goal, and take control of the base.

| Control | Motion |
| --- | --- |
| Hold **Space + ↑ / ↓** | Drive forward or backward |
| Hold **Space + ← / →** | Turn left or right |
| Hold **Space + Shift + ← / →** | Strafe left or right |
| Hold **Rotate Left** or **Rotate Right** | Turn in place for as long as the button is held |

Keep the panel focused for keyboard input. Releasing the controls or leaving
the panel stops manual motion. One RViz window can own manual control at a
time. If that window disconnects, the Jetson stops and waits for a new mode
selection. Click **RViz Goals** or **Explore** to hand control back to Nav2.
SLAM Toolbox continues updating the map while you drive manually.

For a split setup, update the Jetson and every RViz workstation together so
they have the same manual-drive interfaces. A separate RViz-only checkout
also needs to build `amr_interfaces` alongside `amr_description` and
`amr_rviz`. The existing startup scripts build the workspace automatically.

### Control exploration from a terminal

Set `rviz:=false` when launching from SSH without a display. Open a second
terminal on the Jetson, go to the repository root, and run `./scripts/shell.sh`.
From that shell, resume exploration with this command.

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: true}'
```

Pause exploration with this command.

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'
```

If a command waits for a matching subscription, confirm that you launched with
`explore:=true` and check the launch terminal for explorer errors. Press
**Ctrl+C** in the launch terminal to stop the whole stack.

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

### Update an existing Git checkout

After stopping the launch, run these commands from your `rafay_modify_1`
checkout.

```bash
git pull --ff-only
git submodule update --init --recursive
```

Run `./scripts/build_image.sh` when the Dockerfile or the explorer patches in
`.devcontainer/` change. `start.sh` builds the ROS workspace at launch.
For an older copy without a `.git` directory, clone the branch as shown above
and copy any saved maps from the old `ros2_ws/maps/` folder. If an existing
`amr` container points to the old folder, stop and rename that container before
starting from the new checkout.

## Save a map and navigate on it

Pause exploration and wait for the robot to stop. Open a second terminal in the
repository root, enter the container, and save the map.

```bash
./scripts/shell.sh
mkdir -p /workspace/maps
ros2 run nav2_map_server map_saver_cli -f /workspace/maps/auto-demo
```

The map image and YAML are saved in `ros2_ws/maps/` on the Jetson host. Stop
mapping with **Ctrl+C** in the launch terminal, then run saved-map navigation
from the Jetson host.

```bash
./scripts/start.sh mode:=navigation map:=/workspace/maps/auto-demo.yaml rviz:=true \
  enable_motor_commands:=true use_ekf:=true \
  lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 arduino_port:=/dev/ttyUSB2
```

In RViz, use **2D Pose Estimate** to set the robot's starting pose. Once the
scan lines up with the saved map, use **Nav2 Goal** to select a destination.

For the split setup, save the map from `./scripts/shell.sh --vm`. Restart
the Jetson side with `explore:=false` and start the VM side with
`./scripts/start_vm.sh mode:=navigation map:=/workspace/maps/auto-demo.yaml`.

## Check live sensor and navigation data

With mapping running, open separate Jetson terminals and run
`./scripts/shell.sh` in each. Run one of these commands per terminal.

| Command | What it shows |
| --- | --- |
| `ros2 topic echo /imu/data` | Filtered IMU orientation, angular velocity, and linear acceleration |
| `ros2 topic echo /imu/data_raw` | IMU messages directly from the driver |
| `ros2 topic echo /scan --qos-reliability best_effort --field ranges` | Live LiDAR ranges |
| `ros2 topic hz /odometry/filtered` | Fused odometry publishing rate when `use_ekf:=true` |

If `ros2 topic echo` cannot determine the type, run `ros2 topic list -t` to
see which topics are available. `shell.sh` opens the ROS environment. The
sensor drivers start from a launch file. If `/imu/data_raw` exists but
`/imu/data` is missing, check the Madgwick filter in the launch log.

To check only the sensors after the workspace has been built, stop the mapping
launch and run the following inside `./scripts/shell.sh`.

```bash
ros2 launch amr_bringup sensors.launch.py lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1
```

For a full-stack check, run `./scripts/doctor.sh` on the Jetson host with the
same port overrides you used to launch.

If Nav2 misses its 15 Hz control-loop rate, one update took longer than about
67 ms. If a goal fails, look for the planner or controller message before
`Goal failed`. `Failed to create plan` points to the route or costmap,
`Failed to make progress` means the robot's estimated position did not move
8 cm in 10 seconds, and `Controller patience exceeded` means the controller
went more than 0.3 seconds without a valid velocity command. `Goal canceled`
means a client requested cancellation.
See the [troubleshooting guide](docs/TROUBLESHOOTING.md) for more checks.

## How the code fits together

| Component | Role |
| --- | --- |
| `amr_bringup` | Launches the sensors, robot transforms, drive base, and autonomy stack. |
| YDLIDAR and Yahboom drivers | Publish `/scan` and `/imu/data_raw`. The Madgwick filter publishes `/imu/data`. |
| `amr_hardware` | Sends signed PWM commands to the Arduino and publishes encoder counts on `/encoder_ticks`. Per-wheel PWM floors help overcome starting friction, and zero commands still send zero PWM. |
| `amr_control` | Selects Nav2 or manual drive commands on the Jetson, converts them to mecanum wheel commands, and publishes wheel odometry on `/odom`. |
| `robot_localization` | With `use_ekf:=true`, fuses wheel motion from `/odom` with yaw rate from `/imu/data` and publishes `/odometry/filtered`. |
| `amr_description` | Provides the URDF and meshes for RViz, including the lidar mounting transform checked on the physical AMR. |
| `amr_rviz` | Provides the Exploration panel. Its buttons send commands on `/explore/resume`, and its label follows `/explore/status`. |
| Frontier explorer | Sends Nav2 goals at frontiers or clear survey waypoints when frontiers run out. |
| SLAM Toolbox and Nav2 | SLAM Toolbox builds `/map` during mapping. Nav2 plans and drives to goals from RViz or the explorer. Saved-map mode uses AMCL to localize on the map. |

The odometry source publishes `odom → base_link`. SLAM Toolbox supplies
`map → odom` during mapping, and AMCL supplies it during saved-map navigation.

## Test status

- On the Jetson, live lidar, IMU, and encoder data were verified. The lidar
  orientation was checked against the robot in RViz. All four encoder
  channels responded, and a 1-meter test reported about 0.96 m forward.
- Physical exploration started and moved the AMR. The per-wheel PWM floors
  are based on lifted-wheel tests and may need tuning under load.
- An isolated test on the Jetson verified startup pause, fallback survey
  goals, return to frontier goals, and cancellation on pause. The Exploration
  panel was also opened in RViz on the Jetson.
- Live SLAM with Nav2, explorer start and pause, and saving and navigating on
  a map passed in simulation. Saved-map navigation still needs a complete
  physical run.

See the [hardware test plan](docs/HARDWARE_TEST_PLAN.md) and
[troubleshooting guide](docs/TROUBLESHOOTING.md) for diagnostics.

## License

Project source is Apache-2.0 unless a component has its own license.
The YDLIDAR SDK, YDLIDAR driver, Yahboom library, and frontier explorer
retain their upstream licenses.
