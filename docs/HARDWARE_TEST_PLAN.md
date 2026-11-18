# Hardware acceptance on the Jetson

Run the [quickstart](QUICKSTART.md) setup first. Use the lab-approved connection
for the wired robot. Keep the motor supply off for steps 1–3. Enter a sourced
CONTAINER shell using the HOST shell script.

## 1. Lidar and IMU

**HOST** — Start the complete stack with motion disabled.

```bash
./scripts/start.sh mode:=mapping
./scripts/doctor.sh
```

Run the second command in another HOST terminal. Add the same port arguments
to both commands when using numbered ports.

**CONTAINER** — Inspect the live sensor streams.

```bash
ros2 topic hz /scan --window 20
ros2 topic echo /scan --once --qos-reliability best_effort --field header
ros2 topic hz /imu/data_raw --window 30
ros2 topic echo /imu/data --once --field orientation
python3 /amr-scripts/check_sensors.py --seconds 20
```

Expect frame `LiDAR_Link`, valid ranges, and sustained scans. A rate near
4–6 Hz can be normal for X3 hardware. This Jetson also delivered about
11 Hz with 360–430 beams in prior tests. Treat beam count as an observation
and verify the scan shape against a wall. The IMU should publish about
50 Hz at the default rate. Flat and still, acceleration magnitude should
be near 9.8 m/s² and gyro near zero. The installed Yahboom library reports
g and rad/s, so the default scales are 9.80665 and 1.0.

## 2. Arduino and wheel odometry

**CONTAINER** — Keep motor power off and check the encoder stream.

```bash
ros2 topic hz /encoder_ticks --window 30
ros2 topic echo /encoder_ticks
ros2 topic echo /odom --once
```

Expect about 20 Hz. In a separate test for each wheel, mark the tire, turn
it forward once, and check that its channel gains about 660 ticks. Mark a
1 m straight path before starting the push. Record pose before and after.
A reasonable first-pass threshold is 0.90–1.10 m for distance and small
sideways drift on a straight push. For a measured 360° turn, check that yaw
advances through a full revolution and finishes near the initial angle.
Adjust wheel radius or ticks per turn for systematic distance errors. Check
chassis geometry and slip before adjusting wheelbase for yaw errors.

## 3. IMU yaw rate, EKF, and transforms

**CONTAINER** — Record wheel odometry and raw/filtered IMU during a measured
turn. Stop the bag after the turn.

```bash
ros2 bag record /odom /imu/data /imu/data_raw /tf
```

Use the IMU angular velocity around the robot's vertical axis. Verify the
sign, near-zero output at rest, and roughly 6.28 rad integrated over a full
turn. Inspect IMU mounting if signs or axes disagree. Keep wheel odometry as
the baseline.

**HOST** — After stopping the launch, repeat the turn with the EKF enabled.

```bash
./scripts/start.sh mode:=mapping use_ekf:=true
```

**CONTAINER** — Verify the output and the TF chain with the EKF enabled.

```bash
ros2 topic echo /odometry/filtered --once
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
```

The EKF should be the sole `odom → base_link` publisher when enabled.
Verify `map → odom → base_link → LiDAR_Link` and `base_link → IMU_Link`.
With RViz fixed frame set to `odom`, a stationary wall should stay in place
as you push the robot slowly.

**HOST** — To test calibration persistence, stop the launch, put the module
flat and still, and run calibration.

```bash
./scripts/calibrate_imu.sh /dev/myimu
```

Repeat the flat/still check after restarting. Power-cycle the IMU, check
again, and record whether the calibration persisted.

**CONTAINER** — Optional for a module with a magnetometer. Keep the IMU
driver stopped and rotate the module through several orientations.

```bash
python3 /amr-scripts/YbImu_Calibrate_Mag.py /dev/myimu
```

This affects the module's onboard orientation. The Madgwick filter runs
with magnetometer input disabled.

## 4. Powered base

Lift the four wheels clear of the floor. Keep an operator at the power
cutoff. Verify each axis separately before any floor test.

**HOST** — Explicitly enable wheel commands. Keep exploration off.

```bash
./scripts/start.sh mode:=mapping enable_motor_commands:=true
```

**CONTAINER** — Run the following one at a time.

```bash
timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.08}}'
timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.08}}'
timeout 2 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}'
```

Check direction and confirm wheel motion stops after publishing ceases.
The controller times out stale commands and the firmware has a 500 ms
watchdog. Verify stopping on hardware. Tune `max_wheel_speed_rad_s` from
encoder speed and choose limits that still overcome motor deadband at low
speed. Then repeat small moves on the floor.

## 5. SLAM and Nav2 together

**HOST** — Run mapping mode with RViz. Check the footprint against actual
dimensions including anything protruding beyond the chassis.

```bash
./scripts/start.sh mode:=mapping enable_motor_commands:=true rviz:=true
```

Use one nearby RViz goal in visible free space. The robot should move,
stop at the goal, and expand the lidar map while moving. Check the path
and costmaps against the actual walls. If it fails, keep frontier
exploration off while diagnosing.

## 6. Hands-off exploration and saved map

**HOST** — Restart with exploration enabled in a bounded area.

```bash
./scripts/start.sh mode:=mapping explore:=true enable_motor_commands:=true rviz:=true
```

Start the explorer from the CONTAINER shell after the map and Nav2 are
ready. Observe at least two selected goals and growth of the map. Test pause while the AMR is moving, then resume and verify that it can
continue. Verify behavior when every frontier is unreachable.

**CONTAINER** — Use these in a second shell.

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: true}'
```

Pause again before saving the map as shown in the quickstart. Switch to
saved-map navigation, set the initial pose, and test a clear goal, a
detour around an obstacle, and a blocked route. Confirm that it reaches
and stops at reachable goals and stops on failure or cancellation.
Reboot the Jetson and repeat the saved-map demo.
