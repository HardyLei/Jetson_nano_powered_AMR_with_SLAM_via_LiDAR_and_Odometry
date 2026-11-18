#!/usr/bin/env bash
# Shared paths and Docker/ROS helpers for the host scripts.
AMR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
AMR_IMAGE=amr:latest
AMR_CONTAINER=amr

amr_exec() {
    local -a tty=()
    if [[ -t 0 && -t 1 ]]; then tty=(-it); fi
    docker exec "${tty[@]}" --workdir /workspace -e "DISPLAY=${DISPLAY:-}" "$AMR_CONTAINER" \
        bash -ec 'source /opt/ros/humble/setup.bash
                  source /opt/amr-explore/local_setup.bash
                  if [[ -f /workspace/install/setup.bash ]]; then
                      source /workspace/install/setup.bash
                  fi
                  exec "$@"' amr "$@"
}
