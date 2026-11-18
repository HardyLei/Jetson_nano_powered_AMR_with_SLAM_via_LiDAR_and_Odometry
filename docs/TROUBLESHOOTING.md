# Troubleshooting

| Symptom | Cause to check | Action |
|---|---|---|
| Buildx reports permission denied under the Docker config | A previous root command created user config files | **HOST** Run the setup script and log back in if Docker group membership changed. |
| Docker build says it needs an argument | Build context missing | **HOST** Use the image build script. It supplies the repo root. |
| Container exits after a shell | The shell was the main process | **HOST** Start the container with the run script and open shells with the shell script. |
| Devices disappear after replug | Numbered ports changed or a serial handle still refers to the old device | **HOST** Check by-path devices, use stable names or explicit ports, and restart the launch. |
| A lidar port belongs to the IMU | Devices exchanged numbered ports | **HOST** Identify each device by unplugging it briefly or by its USB path. Pass distinct port arguments. |
| No tty for the Mega | USB failed to enumerate | **HOST** Inspect the cable, port and lab-approved isolator. Confirm the device-connection events before launching ROS. |
| Lidar build fails on parameter declarations | Source predates the Humble parameter API | **CONTAINER** Build the driver shipped in this workspace. |
| Scan ranges or frame look wrong | Incorrect model YAML or mounting settings | **HOST** Check the X3 YAML and physical mounting. Expected points are in frame `LiDAR_Link`. |
| Lidar reports real points exceed fixed points | Actual beam count slightly exceeds the scan buffer | **CONTAINER** Check recent scan rate and range validity. The small mismatch warning alone is benign. |
| A lidar topic exists but RViz shows nothing | Reliable subscriber with a best-effort publisher, or missing TF | **CONTAINER** Set LaserScan Reliability to Best Effort and inspect the lidar frame transform. |
| Scan rate is 4–6 Hz | Normal X3 speed for some units | **CONTAINER** Measure with a finite topic-rate window. Keep the scanner clear and avoid touching the rotor. |
| Yahboom import fails | Missing YbImuLib, pyserial or smbus2 | **HOST** Rebuild the image. All three are installed there. |
| IMU filter or RViz display missing | Image lacks the Madgwick filter or RViz IMU plugin | **HOST** Rebuild the image and run the packaged launch. |
| /imu/data_raw shows no recent frames | Wrong port, unplugged module or no serial data | **HOST** Check the USB connection and selected port. **CONTAINER** Look for firmware and recent-frame messages. |
| IMU orientation twitches or units look wrong | Raw onboard orientation or a unit/mounting issue | **CONTAINER** Use filtered `/imu/data` for orientation. Check gravity magnitude and perform a measured turn. Zero mag/baro can mean the sensor variant lacks those readings. |
| Calibration cannot open the serial port | Another IMU process owns it | **HOST** Stop the launch before calibration. The display launch can attach to an existing IMU stream by default. |
| ROS packages are missing in a shell | Workspace was not sourced | **HOST** Open the shell with the shell script. **CONTAINER** Confirm the workspace build succeeded. |
| Need to edit files | Workspace files live on the host | **HOST** Edit the repo directly. **CONTAINER** Nano is installed for quick inspection. |
| RViz cannot open a display | Headless session or missing X11 authorization | **HOST** Launch from the Jetson desktop or use headless mode. Check the doctor's X11 result. |
| SLAM or Nav2 reports missing transforms | Wrong base frame or missing odometry | **CONTAINER** Check `map → odom → base_link → LiDAR_Link`. Use `base_link` in both stacks. |
| Nav2 plans but the base stays still | Motor commands are disabled or speed is below deadband | **HOST** Check the hardware test plan before explicitly enabling motion. Then calibrate PWM from measured wheel speeds. |
| Explorer stops or won't find a goal | Paused at startup, stale map, no reachable frontier, or poor footprint clearance | **CONTAINER** Resume when ready, then inspect the global costmap and frontier markers. Try a nearby RViz goal with exploration paused. |
| RViz goal in saved-map mode does nothing | AMCL has no initial pose | **CONTAINER** Use RViz **2D Pose Estimate**, align the lidar with the map, and then send the goal. |

Use the scripts as written from the repo root. **CONTAINER** commands go in
a sourced container shell. Dockerfile instructions run during image build.
