#!/usr/bin/env bash
set -euo pipefail
E2E_EXTRAS=(--extra sim --extra dev)
E2E_ASSET_ARGS=()
E2E_WITH_DDS=false
for arg in "$@"; do
    case "$arg" in
        --with-demo) E2E_EXTRAS+=(--extra demo); E2E_ASSET_ARGS+=(--with-demo) ;;
        --with-dds) E2E_EXTRAS+=(--extra dds); E2E_WITH_DDS=true ;;
        --with-video) E2E_EXTRAS+=(--extra video) ;;
        --help|-h) echo 'Usage: bash EndToEnd/setup.sh [--with-demo] [--with-dds] [--with-video]'; exit 0 ;;
        *) echo "[NG] unknown option: $arg" >&2; exit 2 ;;
    esac
done
if ! command -v uv >/dev/null 2>&1; then
    echo '[NG] uvをインストールしてください。uv.lockに固定した依存関係を使います' >&2
    exit 1
fi
source "$(dirname "${BASH_SOURCE[0]}")/scripts/runtime.sh"
mkdir -p "$E2E_ROOT/datasets" "$E2E_ROOT/checkpoints" "$E2E_ROOT/runs"
if [[ ! -x "$E2E_ROOT/.venv/bin/python" ]]; then
    "$ISAAC_SIM_PATH/kit/python/bin/python3" -m venv "$E2E_ROOT/.venv"
fi
if "$E2E_WITH_DDS"; then
    bash "$E2E_ROOT/scripts/setup_dds.sh"
fi
uv sync --project "$E2E_ROOT" --python "$ISAAC_SIM_PATH/kit/python/bin/python3" --frozen "${E2E_EXTRAS[@]}"
"$ISAAC_SIM_PATH/python.sh" "$E2E_ROOT/scripts/fetch_assets.py" "${E2E_ASSET_ARGS[@]}"
echo '[READY] bash EndToEnd/run.sh --controller hold --fixed-base --steps 100 で指令経路を確認できます'
