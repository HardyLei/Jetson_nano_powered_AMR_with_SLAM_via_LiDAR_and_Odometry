# Quickstart on the Jetson

Run commands labeled **HOST** in a Jetson terminal. A terminal connected to
the Jetson by SSH also counts as HOST. Run commands labeled **CONTAINER**
after entering the AMR container with the shell script.

## Install and start

1. **HOST** — Clone the repo and select the project branch.

   ```bash
   git clone https://github.com/HardyLei/Jetson_nano_powered_AMR_with_SLAM_via_LiDAR_and_Odometry.git ~/amr_ws
   cd ~/amr_ws
   git switch rafay_modify_1
   git submodule update --init
   ```

   Use a checkout containing the bring-up changes. The Docker build uses the
   checked-out SDK if present. It fetches a pinned upstream SDK revision when
   the submodule is empty.

2. **HOST** — Set up Docker and serial permissions, then build the image.

   ```bash
   ./scripts/setup_host.sh
   ./scripts/build_image.sh
   ```

   Log out and back in if setup adds you to a group. Setup skips the unfilled
   udev template. Follow [Stable USB names](#stable-usb-names) when you have
   identified the devices. The Dockerfile installs the pinned frontier
   explorer and all ROS dependencies into the image. The explorer starts
   paused until you tell it to begin.

3. **HOST** — Start mapping with motor commands disabled. Use a Jetson
   desktop terminal for RViz.

   ```bash
   ./scripts/start.sh mode:=mapping rviz:=true
   ```

   If you use explicit devices, pass the paths identified on your machine.
   This example uses the USB assignments observed on the Jetson during
   bring-up.

   ```bash
   ./scripts/start.sh mode:=mapping lidar_port:=/dev/ttyUSB0 imu_port:=/dev/ttyUSB1 arduino_port:=/dev/ttyUSB2
   ```

   The container is named `amr`. Keep the launch terminal open and use
   another terminal for diagnostics. On a headless SSH session, omit
   `rviz:=true`.

4. **HOST** — Check both sensors and the ROS graph from a second terminal.
   Supply the same port arguments if you used explicit devices.

   ```bash
   ./scripts/doctor.sh
   ./scripts/shell.sh
   ```

   Expect lidar, IMU, encoder, odometry, map, and Nav2 lifecycle checks.
   The IMU firmware and resting measurements appear in the output. Docker
   runs with your user ID, so you can edit or delete generated workspace
   files from the HOST. Edit source files on the HOST.

## Map and navigate

Complete the powered tests in the [hardware test plan](HARDWARE_TEST_PLAN.md)
before using the following commands. Stop the current launch with Ctrl+C
before switching modes. Use only one source of drive goals at a time.

**HOST** — Start live SLAM and Nav2 after the base passes. Pick one nearby
goal in visible free space with **Nav2 Goal** in RViz.

```bash
./scripts/start.sh mode:=mapping enable_motor_commands:=true rviz:=true
```

**HOST** — After that works, start hands-off exploration in a bounded test
area. Wait for the map and Nav2 to become ready.

```bash
./scripts/start.sh mode:=mapping explore:=true enable_motor_commands:=true rviz:=true
```

**CONTAINER** — Start autonomous exploration from a second shell when the
area is clear.

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: true}'
```

**CONTAINER** — Pause exploration and save the map from a second shell.
Wait for the robot to stop before saving.

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'
mkdir -p /workspace/maps
ros2 run nav2_map_server map_saver_cli -f /workspace/maps/demo
```

Expect `demo.yaml` and a map image in `ros2_ws/maps` on the HOST. Stop
the mapping launch with Ctrl+C.

**HOST** — Start saved-map navigation. In RViz, use **2D Pose Estimate** to
place the AMR on the map. Check that the lidar lines up with the walls.
Then use **Nav2 Goal** to select a nearby destination.

```bash
./scripts/start.sh mode:=navigation map:=/workspace/maps/demo.yaml enable_motor_commands:=true rviz:=true
```

Repeat the port arguments in each launch if using explicit ports. Add
`use_ekf:=true` after the IMU and EKF pass the measured turn test. Select the
same odometry source for both modes.

## Stable USB names

**HOST** — List the devices and read the attributes for the device you are
identifying. Unplug one device at a time to see which entry disappears.

```bash
ls -l /dev/serial/by-path /dev/serial/by-id
udevadm info -q property -n /dev/ttyUSB0
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial|KERNELS'
cp scripts/udev/99-amr.rules /tmp/amr-local.rules
```

Fill the three placeholders in `/tmp/amr-local.rules` with the corresponding
`ID_PATH` values. Keep devices in the same sockets when matching by path.
If serial numbers are unique, the template describes how to match those
instead. Two adapters of the same model can share USB identifiers.

**HOST** — Install and verify your completed rules.

```bash
./scripts/install_udev.sh /tmp/amr-local.rules
ls -l /dev/ydlidar /dev/myimu /dev/amr
```

After a replug, restart the ROS launch so the drivers open the new device
nodes. The container already sees the live `/dev` tree. Use
[troubleshooting](TROUBLESHOOTING.md) if a name is missing.

## Test without the robot

**HOST** — Run the simulated test after building the image. It uses the
real SLAM Toolbox, Nav2 and explorer with synthetic lidar and odometry.
It checks live mapping, saving and reloading a map, an RViz-equivalent
goal in each mode, and explorer start and pause.

```bash
docker run --rm --env ROS_LOCALHOST_ONLY=1 \
  --mount "type=bind,src=$PWD,dst=/project,readonly" amr:latest bash -ec '
    cp -r /project/ros2_ws/src /workspace/
    source /opt/amr-explore/local_setup.bash
    MAKEFLAGS=-j2 colcon build --executor sequential --symlink-install --packages-ignore ydlidar_sdk
    source /workspace/install/setup.bash
    python3 /project/tests/autonomy_smoke.py
  '
```
