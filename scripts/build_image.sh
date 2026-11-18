#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
echo "HOST Building $AMR_IMAGE with ROS 2 Humble, SLAM Toolbox, Nav2, and the explorer."
docker build -f "$AMR_ROOT/.devcontainer/Dockerfile" -t "$AMR_IMAGE" "$AMR_ROOT"
