#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"

if [[ ${1:-} == --help ]]; then
    echo 'HOST Usage: ./scripts/start.sh [mode:=mapping|navigation] [key:=value ...]'
    exit
fi
lidar=/dev/ydlidar
imu=/dev/myimu
arduino=/dev/amr
start_robot=true
for arg in "$@"; do
    case "$arg" in
        lidar_port:=*) lidar=${arg#*:=} ;;
        imu_port:=*) imu=${arg#*:=} ;;
        arduino_port:=*) arduino=${arg#*:=} ;;
        start_robot:=*) start_robot=${arg#*:=} ;;
    esac
done
if [[ $start_robot == true ]]; then
    for port in "$lidar" "$imu" "$arduino"; do
        if [[ ! -c $port || ! -r $port || ! -w $port ]]; then
            echo "HOST Cannot open $port. Check USB, run setup_host.sh, or pass the matching port argument." >&2
            exit 1
        fi
    done
    if [[ $lidar -ef $imu || $lidar -ef $arduino || $imu -ef $arduino ]]; then
        echo 'HOST Lidar, IMU, and Arduino must use three distinct devices.' >&2
        exit 1
    fi
fi

"$AMR_ROOT/scripts/run_container.sh"
echo 'CONTAINER Building the workspace, then launching SLAM/Navigation. Press Ctrl+C to stop.'
docker exec --workdir /workspace "$AMR_CONTAINER" flock -n /tmp/amr-launch.lock bash -ec '
    source /opt/ros/humble/setup.bash
    source /opt/amr-explore/local_setup.bash
    export MAKEFLAGS=-j2
    colcon build --executor sequential --symlink-install --packages-ignore ydlidar_sdk
'
amr_exec flock -n --no-fork /tmp/amr-launch.lock ros2 launch amr_bringup autonomy.launch.py "$@"
