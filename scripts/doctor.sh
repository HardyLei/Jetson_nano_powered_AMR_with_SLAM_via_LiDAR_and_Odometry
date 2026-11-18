#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"

lidar=/dev/ydlidar
imu=/dev/myimu
arduino=/dev/amr
imu_rate=50.0
for arg in "$@"; do
    case "$arg" in
        lidar_port:=*) lidar=${arg#*:=} ;;
        imu_port:=*) imu=${arg#*:=} ;;
        arduino_port:=*) arduino=${arg#*:=} ;;
        imu_rate_hz:=*) imu_rate=${arg#*:=} ;;
        *) echo "HOST Unknown option $arg" >&2; exit 2 ;;
    esac
done
fail=0
pass() { echo "HOST PASS $*"; }
failure() { echo "HOST FAIL $*" >&2; fail=1; }

config=${DOCKER_CONFIG:-$HOME/.docker}
if [[ -d $config ]]; then
    probe=$(find "$config" ! -user "$(id -u)" -print -quit 2>&1) || true
    if [[ -n $probe ]]; then
        failure "Docker config ownership under $config. Run ./scripts/setup_host.sh."
    else
        pass "Docker config ownership under $config."
    fi
fi
if id -nG | tr ' ' '\n' | grep -Fxq docker; then
    pass 'Docker group.'
else
    failure 'Docker group. Run ./scripts/setup_host.sh and log in again.'
fi
if docker info >/dev/null 2>&1; then
    pass 'Docker daemon.'
else
    failure 'Docker daemon. Check its service and login groups.'
fi
echo 'HOST Numbered serial devices currently present:'
find /dev -maxdepth 1 \( -name 'ttyUSB*' -o -name 'ttyACM*' \) -print | sort
for port in "$lidar" "$imu" "$arduino"; do
    if [[ -c $port && -r $port && -w $port ]]; then
        pass "$port resolves to $(readlink -f -- "$port") and is accessible."
    else
        failure "$port. Check the cable, udev rule, or explicit launch argument."
    fi
done
display_number=${DISPLAY:-}
display_number=${display_number##*:}
if [[ -n ${DISPLAY:-} && -S /tmp/.X11-unix/X${display_number%%.*} ]]; then
    pass "X11 socket for DISPLAY=$DISPLAY."
else
    echo 'HOST WARN X11 display unavailable. Run RViz from a Jetson desktop session.'
fi

if [[ $(docker inspect -f '{{.State.Running}}' "$AMR_CONTAINER" 2>/dev/null || true) != true ]]; then
    failure 'AMR container. Run ./scripts/start.sh before checking live ROS topics.'
    exit "$fail"
fi
pass 'AMR container running.'
if amr_exec python3 -c 'import YbImuLib, serial, smbus2'; then
    pass 'YbImuLib and serial dependencies.'
else
    failure 'YbImuLib import. Rebuild the image.'
fi
if ! amr_exec python3 /amr-scripts/check_sensors.py --seconds 8 --imu-rate-hz "$imu_rate"; then
    failure 'Sensor streams. Check the selected ports and the launch output.'
fi
if ! amr_exec timeout 8 ros2 param get /ybimu_node firmware_version; then
    failure 'IMU firmware query. Check the IMU launch.'
fi
for topic in /encoder_ticks /odom /map; do
    if amr_exec timeout 8 ros2 topic echo "$topic" --once --field \
            "$(case "$topic" in /encoder_ticks) echo front_left;; *) echo header;; esac)" >/dev/null; then
        pass "Received $topic."
    else
        failure "No $topic. Check the Arduino for encoder/odom topics and SLAM or map_server for /map."
    fi
done
if ! amr_exec bash -ec 'for node in controller_server planner_server bt_navigator; do
        timeout 5 ros2 lifecycle get "/$node" | grep -q "^active " ||
            { echo "CONTAINER FAIL /$node is not active."; exit 1; }
        echo "CONTAINER PASS /$node active."
    done'; then
    failure 'Nav2 lifecycle. Check the launch log for map, TF, and plugin errors.'
fi
exit "$fail"
