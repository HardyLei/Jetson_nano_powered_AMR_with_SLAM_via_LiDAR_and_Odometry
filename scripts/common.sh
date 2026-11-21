#!/usr/bin/env bash
# Shared paths and Docker/ROS helpers for the host scripts.
AMR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
AMR_IMAGE=amr:latest
AMR_CONTAINER=amr
AMR_ROS_DOMAIN_ID=${AMR_ROS_DOMAIN_ID:-67}
if [[ ${AMR_PROFILE:-} == vm ]]; then
    AMR_CONTAINER=amr-vm
fi

amr_exec() {
    local -a options=(--workdir /workspace -e "DISPLAY=${DISPLAY:-}"
                     -e "ROS_DOMAIN_ID=$AMR_ROS_DOMAIN_ID" -e ROS_LOCALHOST_ONLY=0)
    if [[ -t 0 && -t 1 ]]; then options+=(-it); fi
    docker exec "${options[@]}" "$AMR_CONTAINER" \
        bash -ec 'source /opt/ros/humble/setup.bash
                  source /opt/amr-explore/local_setup.bash
                  source /opt/amr-gmapping/local_setup.bash
                  if [[ -f /workspace/install/setup.bash ]]; then
                      source /workspace/install/setup.bash
                  fi
                  exec "$@"' amr "$@"
}
