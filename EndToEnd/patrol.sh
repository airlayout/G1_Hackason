#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/runtime.sh"
e2e_launch sim/collect.py --patrol --controller unitree --scene-config "$E2E_ROOT/configs/scenes/office.yaml" \
    --env-config "$E2E_ROOT/configs/task/office_patrol.yaml" --repeat-lidar-pattern \
    --reset-on-done --steps 3000 "$@"
