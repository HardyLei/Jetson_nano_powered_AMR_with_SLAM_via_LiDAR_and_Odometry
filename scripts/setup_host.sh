#!/usr/bin/env bash
set -euo pipefail

if [[ $(uname -s) != Linux || $EUID == 0 ]]; then
    echo 'HOST Run this on the Jetson as the regular user.' >&2
    exit 1
fi
if ! command -v docker >/dev/null; then
    echo 'HOST Install Docker on the Jetson before running setup.' >&2
    exit 1
fi
config=${DOCKER_CONFIG:-$HOME/.docker}
if [[ -d $config ]]; then
    probe=$(find "$config" ! -user "$(id -u)" -print -quit 2>&1) || true
    if [[ -n $probe ]]; then
        echo "HOST Repairing ownership under $config so Docker and Buildx can write there."
        sudo chown -R -P "$(id -u):$(id -g)" "$config"
    fi
fi
for group in docker dialout; do
    if ! getent group "$group" >/dev/null; then sudo groupadd "$group"; fi
    if ! id -nG "$(id -un)" | tr ' ' '\n' | grep -Fxq "$group"; then
        echo "HOST Adding $(id -un) to $group."
        sudo usermod -aG "$group" "$(id -un)"
    fi
done
"$(dirname -- "${BASH_SOURCE[0]}")/install_udev.sh" "$@"
if docker info >/dev/null 2>&1; then
    echo 'HOST Docker is reachable.'
else
    echo 'HOST Docker unavailable. Start its service if stopped and log out and back in after group changes.' >&2
    exit 1
fi
if ! id -nG | tr ' ' '\n' | grep -Fxq dialout; then
    echo 'HOST Log out and back in to activate dialout membership before accessing serial devices.'
fi
