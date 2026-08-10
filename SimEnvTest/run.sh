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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# パスの定義は env.sh に一本化してある。2 箇所に書くと片方の修正漏れが起きる。
source "$SCRIPT_DIR/env.sh"

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
