#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"

if ! docker image inspect "$AMR_IMAGE" >/dev/null 2>&1; then
    "$AMR_ROOT/scripts/build_image.sh"
fi
display_number=${DISPLAY:-}
display_number=${display_number##*:}
if [[ -n ${DISPLAY:-} && -S /tmp/.X11-unix/X${display_number%%.*} ]] && command -v xhost >/dev/null; then
    echo 'HOST Allowing the current user to open X11 windows.'
    xhost "+SI:localuser:$(id -un)" >/dev/null || echo 'HOST X11 authorization failed. Headless launch is still available.'
fi

image_id=$(docker image inspect --format '{{.Id}}' "$AMR_IMAGE")
if docker container inspect "$AMR_CONTAINER" >/dev/null 2>&1; then
    owner=$(docker inspect --format '{{index .Config.Labels "amr.workspace"}}' "$AMR_CONTAINER")
    if [[ $owner != "$AMR_ROOT" ]]; then
        echo "HOST Container $AMR_CONTAINER belongs to another workspace. Choose a free name before starting." >&2
        exit 1
    fi
    if [[ $(docker inspect --format '{{.Image}}' "$AMR_CONTAINER") != "$image_id" ]]; then
        echo 'HOST Replacing this checkout’s container to use the rebuilt image.'
        docker rm -f "$AMR_CONTAINER" >/dev/null
    else
        if [[ $(docker inspect --format '{{.State.Running}}' "$AMR_CONTAINER") != true ]]; then
            echo 'HOST Starting the existing AMR container.'
            docker start "$AMR_CONTAINER" >/dev/null
        else
            echo 'HOST Reusing the running AMR container.'
        fi
        exit 0
    fi
fi

args=()
if dialout=$(getent group dialout | cut -d: -f3) && [[ -n $dialout ]]; then
    args+=(--group-add "$dialout")
fi
if [[ -d /tmp/.X11-unix ]]; then
    args+=(--volume /tmp/.X11-unix:/tmp/.X11-unix:ro)
fi
echo 'HOST Starting the AMR container as your user. Live /dev access lets device names follow USB replugging.'
docker run -d --init --name "$AMR_CONTAINER" --network host --privileged \
    --label "amr.workspace=$AMR_ROOT" --user "$(id -u):$(id -g)" \
    --env HOME=/tmp/amr-home --env QT_X11_NO_MITSHM=1 \
    --volume /dev:/dev --volume "$AMR_ROOT/ros2_ws:/workspace" \
    --volume "$AMR_ROOT/scripts:/amr-scripts:ro" \
    "${args[@]}" "$AMR_IMAGE" bash -ec 'mkdir -p "$HOME"; exec sleep infinity' >/dev/null
