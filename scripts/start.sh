#!/usr/bin/env bash
set -euo pipefail

if [[ ${1:-} == --help ]]; then
    echo 'HOST Usage: ./scripts/start.sh [mode:=mapping|navigation] [key:=value ...]'
    echo 'HOST Split setup: ./scripts/start_robot.sh on the Jetson and ./scripts/start_vm.sh on the VM.'
    exit
fi
launch_file=autonomy.launch.py
case ${1:-} in
    --robot)
        shift
        launch_file=robot.launch.py
        export AMR_PROFILE=robot
        set -- use_ekf:=true explore:=true "$@"
        ;;
    --vm)
        shift
        export AMR_PROFILE=vm
        set -- use_ekf:=true rviz:=true "$@" start_robot:=false explore:=false
        ;;
esac
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"

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
if [[ $launch_file == robot.launch.py || $start_robot == true ]]; then
    for port in "$lidar" "$imu" "$arduino"; do
        if [[ ! -c $port || ! -r $port || ! -w $port ]]; then
            echo "HOST Cannot open $port. Check USB and device permissions, or pass the matching port argument." >&2
            exit 1
        fi
    done
    if [[ $lidar -ef $imu || $lidar -ef $arduino || $imu -ef $arduino ]]; then
        echo 'HOST Lidar, IMU, and Arduino must use three distinct devices.' >&2
        exit 1
    fi
fi

"$AMR_ROOT/scripts/run_container.sh"
echo "CONTAINER Building the workspace, then running $launch_file on ROS domain $AMR_ROS_DOMAIN_ID. Press Ctrl+C to stop."
docker exec --workdir /workspace "$AMR_CONTAINER" flock -n /tmp/amr-launch.lock bash -ec '
    source /opt/ros/humble/setup.bash
    source /opt/amr-explore/local_setup.bash
    source /opt/amr-gmapping/local_setup.bash
    export MAKEFLAGS=-j2
    colcon build --executor sequential --symlink-install --packages-ignore ydlidar_sdk
'

if [[ $launch_file == robot.launch.py && ${AMR_FACE_RECOGNITION:-1} == 1 ]]; then
    echo 'HOST Starting face recognition on the Jetson.'
    python3 -u "$AMR_ROOT/scripts/Face_Recognition.py" \
        --known-dir "$AMR_ROOT/known_faces" --no-display &
    face_pid=$!
    stop_face_recognition() {
        if jobs -pr | grep -Fxq "$face_pid"; then
            kill "$face_pid" 2>/dev/null || true
        fi
        wait "$face_pid" 2>/dev/null || true
    }
    trap stop_face_recognition EXIT
fi

amr_exec flock -n --no-fork /tmp/amr-launch.lock \
    env AMR_DOOR_SOCKET=/workspace/build/amr-door.sock \
    ros2 launch amr_bringup "$launch_file" "$@"
