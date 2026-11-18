#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"

rule=${1:-$AMR_ROOT/scripts/udev/99-amr.rules}
target=/etc/udev/rules.d/99-amr.rules
if [[ ! -r $rule ]]; then
    echo "HOST Cannot read $rule." >&2
    exit 1
fi
if grep -q '___' "$rule"; then
    echo "HOST $rule is an untested template. Fill in the hardware attributes and rerun, or pass explicit port arguments."
    exit 0
fi
if ! cmp -s "$rule" "$target"; then
    echo "HOST Installing stable serial names from $rule."
    sudo install -m 0644 "$rule" "$target"
    sudo udevadm control --reload-rules
    sudo udevadm trigger --action=add --subsystem-match=tty
    sudo udevadm settle --timeout=10
else
    echo 'HOST The AMR udev rules are already installed.'
fi
