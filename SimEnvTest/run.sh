#!/usr/bin/env bash
# G1 デジタルツイン操作環境の起動スクリプト。
#
# この環境では isaacsim と isaaclab が別々の Python に入っているため、
# Isaac Sim の python.sh に PYTHONPATH を通して両方を使えるようにする。
# （./isaaclab.sh は使えない。詳細は G1/CLAUDE.md 参照）
#
# 使い方:
#   bash run.sh                # Warehouse シーンで起動（GUI）
#   bash run.sh --flat         # 平地で起動（動作確認用）
#   bash run.sh --viz none     # ヘッドレス
set -euo pipefail

ISAAC_SIM=/home/spacedata/isaacSim6.0dev2/_build/linux-x86_64/release
ISAACLAB=/home/spacedata/IsaacLab
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# editable install の実ソースと、依存パッケージ（warp, rsl_rl 等）の両方を通す
LAB_SOURCES=$(ls -d "$ISAACLAB"/source/*/ | tr '\n' ':')
LAB_SITE_PACKAGES="$ISAACLAB/env_isaaclab/lib/python3.12/site-packages"

export PYTHONPATH="${SCRIPT_DIR}/src:${LAB_SOURCES}${LAB_SITE_PACKAGES}"
export DISPLAY="${DISPLAY:-:1}"

mkdir -p "$SCRIPT_DIR/logs"
LOG="$SCRIPT_DIR/logs/g1_twin.log"

# --viz が指定されていなければ GUI (kit) を既定にする
# （このバージョンの IsaacLab はヘッドレスが既定のため明示が必要）
ARGS=("$@")
if ! printf '%s\n' "$@" | grep -q -- "--viz"; then
    ARGS+=(--viz kit)
fi

# print() をバッファリングさせない（tee 越しでも進捗が即座に見えるように）
export PYTHONUNBUFFERED=1

# 起動時に blas_thread_shutdown / __libc_fork 内で segfault することがあるため
# BLAS のスレッド数を 1 に抑える（Isaac Sim の fork と OpenBLAS の相性問題）
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

echo "[INFO] G1 デジタルツインを起動します (log: $LOG)"
exec "$ISAAC_SIM/python.sh" "$SCRIPT_DIR/src/run_g1_twin.py" "${ARGS[@]}" 2>&1 \
    | stdbuf -oL -eL tee "$LOG"
