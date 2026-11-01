#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/arm_env.sh" "$@"
ARM_ENV="$ARM_ROOT/.venv-arm"
if [[ "${1:-}" == --vla ]]; then
    ARM_ENV="$ARM_MODEL_ENV"
    shift
fi
if [[ ! -x "$ARM_ENV/bin/python" ]]; then
    echo '[NG] 先に bash EndToEnd/arm-setup.sh --ui または --model を実行してください' >&2
    exit 1
fi
export PYTHONPATH="$ARM_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$ARM_ENV/bin/python" -m g1_e2e.vla.app "$@" --config "$ARM_CONFIG"
