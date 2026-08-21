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

# G1 の頭部カメラ（camera.py）を使うには起動時の --enable_cameras が必要。
# 無いとカメラ拡張が読み込まれずセンサ構築時にエラーになる。
if ! printf '%s\n' "$@" | grep -q -- "--enable_cameras"; then
    ARGS+=(--enable_cameras)
fi

# --enable_cameras 指定時、環境によっては拡張レジストリ（オンライン）への
# 到達性が無く「AppLauncher initialization complete」より前で無言のまま
# 数分〜無限に止まることがある（extscache に無いカメラ関連拡張をオンライン
# 解決しようとして固まる）。ローカルの extscache だけで完結させ、
# レジストリへは問い合わせないようにする。
# ユーザーが自分で --kit_args を指定している場合は上書きしない。
if ! printf '%s\n' "$@" | grep -q -- "--kit_args"; then
    ARGS+=(--kit_args="--/app/extensions/registryEnabled=false")
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
