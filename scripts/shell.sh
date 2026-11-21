#!/usr/bin/env bash
set -euo pipefail
if [[ ${1:-} == --vm ]]; then
    export AMR_PROFILE=vm
fi
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
"$AMR_ROOT/scripts/run_container.sh"
echo 'HOST Opening a sourced shell in the AMR container.'
amr_exec bash
