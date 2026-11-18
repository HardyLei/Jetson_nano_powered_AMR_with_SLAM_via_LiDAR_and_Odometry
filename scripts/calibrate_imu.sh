#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"

port=${1:-/dev/myimu}
"$AMR_ROOT/scripts/run_container.sh"
echo 'HOST Keep the IMU flat and still. Stop the running IMU launch before calibrating.'
status=0
amr_exec flock -n -E 42 --no-fork /tmp/amr-launch.lock \
    python3 /amr-scripts/YbImu_Calibrate_IMU.py "$port" || status=$?
if (( status == 42 )); then
    echo 'HOST Stop the running stack with Ctrl+C, then retry calibration.'
fi
echo "HOST Restart the stack with ./scripts/start.sh imu_port:=$port and your other launch arguments."
exit "$status"
