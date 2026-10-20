#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/scripts/runtime.sh"
mkdir -p "$E2E_ROOT/datasets" "$E2E_ROOT/checkpoints" "$E2E_ROOT/runs"
if [[ ! -x "$E2E_ROOT/.venv/bin/python" ]]; then
    "$ISAAC_SIM_PATH/kit/python/bin/python3" -m venv "$E2E_ROOT/.venv"
fi
if command -v uv >/dev/null 2>&1; then
    E2E_EXTRAS=(--extra sim --extra dev)
    for arg in "$@"; do
        [[ "$arg" == --with-demo ]] && E2E_EXTRAS+=(--extra demo)
    done
    uv sync --project "$E2E_ROOT" --frozen "${E2E_EXTRAS[@]}"
else
    echo '[NG] uvをインストールしてください。uv.lockに固定した依存関係を使います' >&2
    exit 1
fi
"$ISAAC_SIM_PATH/python.sh" "$E2E_ROOT/scripts/fetch_assets.py" "$@"
echo '[READY] bash EndToEnd/run.sh --controller hold --fixed-base --steps 100 で指令経路を確認できます'
