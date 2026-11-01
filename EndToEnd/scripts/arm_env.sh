#!/usr/bin/env bash
# モデルの設定・仮想環境の選択を、セットアップとUIで共通化する。
set -euo pipefail
ARM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARM_CONFIG="$ARM_ROOT/configs/vla/pi05.json"
ARM_CONFIG_NEXT=false
for ARM_ARG in "$@"; do
    if "$ARM_CONFIG_NEXT"; then
        ARM_CONFIG="$ARM_ARG"
        ARM_CONFIG_NEXT=false
    elif [[ "$ARM_ARG" == --config ]]; then
        ARM_CONFIG_NEXT=true
    elif [[ "$ARM_ARG" == --config=* ]]; then
        ARM_CONFIG="${ARM_ARG#--config=}"
    fi
done
if "$ARM_CONFIG_NEXT"; then
    echo '[NG] --configには設定ファイルを指定してください' >&2
    exit 2
fi
ARM_BACKEND="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("backend","groot"))' "$ARM_CONFIG")"
case "$ARM_BACKEND" in
    pi05) ARM_MODEL_ENV="$ARM_ROOT/.venv-pi05"; ARM_REQUIREMENTS="$ARM_ROOT/configs/vla/requirements-pi05.txt" ;;
    groot) ARM_MODEL_ENV="$ARM_ROOT/.venv-vla"; ARM_REQUIREMENTS="$ARM_ROOT/configs/vla/requirements-vla.txt" ;;
    *) echo "[NG] 不明なbackend: $ARM_BACKEND" >&2; exit 2 ;;
esac
