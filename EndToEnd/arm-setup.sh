#!/usr/bin/env bash
# Isaac環境とは分けて、UIまたはVLA推論だけを準備する。
set -euo pipefail
ARM_MODE=--ui
ARM_SETUP_ARGS=()
while (( $# )); do
    case "$1" in
        --ui|--model) ARM_MODE="$1"; shift ;;
        --config)
            if (( $# < 2 )); then echo '[NG] --configにはファイルを指定してください' >&2; exit 2; fi
            ARM_SETUP_ARGS+=(--config "$2"); shift 2 ;;
        --help|-h) echo 'Usage: bash EndToEnd/arm-setup.sh [--ui|--model] [--config PATH]'; exit 0 ;;
        *) echo "[NG] 未知の引数: $1" >&2; exit 2 ;;
    esac
done
source "$(dirname "${BASH_SOURCE[0]}")/scripts/arm_env.sh" "${ARM_SETUP_ARGS[@]}"
command -v uv >/dev/null || { echo '[NG] uvをインストールしてください' >&2; exit 1; }
if [[ "$ARM_MODE" == --model ]]; then
    ARM_FREE_KB="$(df -Pk "$ARM_ROOT" | awk 'NR==2 {print $4}')"
    ARM_INSTALLED=false
    if [[ -x "$ARM_MODEL_ENV/bin/python" ]]; then
        ARM_INSTALLED="$(python3 - "$ARM_CONFIG" "$ARM_ROOT" <<'PY'
import json, sys
from pathlib import Path
config = json.load(open(sys.argv[1]))
path = Path(config['model_directory'])
if not path.is_absolute():
    path = Path(sys.argv[2]) / path
ready = ((path / 'model.safetensors').is_file() if config.get('backend') == 'pi05'
         else (path / 'processor_config.json').is_file())
print('true' if ready else 'false')
PY
)"
    fi
    if ! "$ARM_INSTALLED" && (( ARM_FREE_KB < 30 * 1024 * 1024 )); then
        echo '[NG] モデルとCUDA環境の初回導入には空き30GBを確保してください。既存データは削除しません' >&2
        exit 1
    fi
    ARM_ENV="$ARM_MODEL_ENV"
else
    ARM_ENV="$ARM_ROOT/.venv-arm"
fi
if [[ ! -x "$ARM_ENV/bin/python" ]]; then
    uv venv --python 3.12 "$ARM_ENV"
fi
if [[ "$ARM_MODE" == --ui ]]; then
    uv pip install --python "$ARM_ENV/bin/python" -r "$ARM_ROOT/configs/vla/requirements-ui.txt"
    if [[ "$ARM_BACKEND" == groot ]]; then
        "$ARM_ENV/bin/python" "$ARM_ROOT/scripts/fetch_assets.py" --urdf-only
    fi
    echo '[READY] bash EndToEnd/arm-ui.sh --demo'
    exit 0
fi
uv pip install --python "$ARM_ENV/bin/python" --extra-index-url https://download.pytorch.org/whl/cu128 \
    --index-strategy unsafe-best-match -r "$ARM_REQUIREMENTS"
if [[ "$ARM_BACKEND" == groot ]]; then
    "$ARM_ENV/bin/python" "$ARM_ROOT/scripts/fetch_assets.py" --urdf-only
    ARM_SOURCE="$ARM_ROOT/assets/cache/vla/Isaac-GR00T"
    ARM_COMMIT="$("$ARM_ENV/bin/python" -c 'import json,sys; print(json.load(open(sys.argv[1]))["source_commit"])' "$ARM_CONFIG")"
    if [[ ! -d "$ARM_SOURCE/.git" ]]; then
        git clone --no-checkout --filter=blob:none https://github.com/NVIDIA/Isaac-GR00T.git "$ARM_SOURCE"
    elif [[ -n "$(git -C "$ARM_SOURCE" status --porcelain)" ]]; then
        echo '[NG] GR00Tの取得先に変更があります。上書きせず停止します' >&2
        exit 1
    fi
    git -C "$ARM_SOURCE" fetch --depth 1 origin "$ARM_COMMIT"
    GIT_LFS_SKIP_SMUDGE=1 git -C "$ARM_SOURCE" checkout --detach "$ARM_COMMIT"
    uv pip install --python "$ARM_ENV/bin/python" --no-deps -e "$ARM_SOURCE"
fi
export PYTHONPATH="$ARM_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
"$ARM_ENV/bin/python" -m g1_e2e.vla.tools download --config "$ARM_CONFIG"
echo "[READY] backend=$ARM_BACKEND / bash EndToEnd/arm-ui.sh --vla --config $ARM_CONFIG"
