#!/usr/bin/env bash
# G1 デジタルツイン操作環境の起動スクリプト。
#
# この環境では isaacsim と isaaclab が別々の Python に入っているため、
# Isaac Sim の python.sh に PYTHONPATH を通して両方を使えるようにする。
# （./isaaclab.sh は使えない。詳細は G1/CLAUDE.md 参照）
#
# 使い方:
#   bash run.sh                # 実測した部屋（UiS_room_v3）で起動（GUI）
#   bash run.sh --flat         # 平地で起動（動作確認用）
#   bash run.sh --warehouse    # NVIDIA の Warehouse で起動（⚠️ インターネットが要る）
#   bash run.sh --viz none     # ヘッドレス
#
# 操作: W/S 前後  A/D 左右  Q/E 旋回  SPACE 停止  SHIFT 低速
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# パスの定義は env.sh に一本化してある。2 箇所に書くと片方の修正漏れが起きる。
source "$SCRIPT_DIR/env.sh"

mkdir -p "$SCRIPT_DIR/logs"
LOG="$SCRIPT_DIR/logs/g1_twin.log"

# ---------------------------------------------------------------------------
# 既定のシーン
# ---------------------------------------------------------------------------
# ⚠️ **既定を Warehouse に戻さないこと。**
# Warehouse の USD は NVIDIA の S3 から取ってくる（NUCLEUS_ASSET_ROOT_DIR +
# /Isaac/Environments/Simple_Warehouse/full_warehouse.usd）。デモ機
# （192.168.123.200）はインターネットに出られないので、引数なしで起動すると
# 16 秒待たされた末に FileNotFoundError で落ちる（2026-09-17 実測）。
# ローカルにある実測部屋の USD を既定にして、オフラインで完結させる。
# Warehouse が要るときだけ --warehouse を明示する。
ROOM_USD="$SCRIPT_DIR/assets/uis_room_v3_sim_obstacles.usd"

# この USD は床を含まない（runner.py が別途平地を敷く）。
# スポーンは Demo/03_isaac_nav2.sh と同じ自由空間の値を使う。
ROOM_SPAWN_X=6.55
ROOM_SPAWN_Y=-0.78

# ---------------------------------------------------------------------------
# 引数
# ---------------------------------------------------------------------------
# ⚠️ ここで `printf '%s\n' "$@" | grep -q` を使わないこと。`set -o pipefail` の下では
#    grep が先に抜けたときパイプ元が SIGPIPE で死ぬため、見つかった側（＝合格の経路）
#    だけが非ゼロを返しうる。1 回のループで数えるほうが安全で速い。
ARGS=()
WANT_WAREHOUSE=0
HAS_SCENE=0
HAS_SPAWN=0
HAS_VIZ=0
HAS_CAMERAS=0
HAS_KIT_ARGS=0
for a in "$@"; do
    case "$a" in
        # run_g1_twin.py は --warehouse を知らない（「何も足さない」のが Warehouse
        # を意味する）。ここで食べて、python 側へは渡さない。
        --warehouse)                      WANT_WAREHOUSE=1; continue ;;
        --flat|--scene-usd|--scene-usd=*) HAS_SCENE=1 ;;
        --x|--x=*|--y|--y=*)              HAS_SPAWN=1 ;;
        --viz|--viz=*)                    HAS_VIZ=1 ;;
        --enable_cameras)                 HAS_CAMERAS=1 ;;
        --kit_args|--kit_args=*)          HAS_KIT_ARGS=1 ;;
    esac
    ARGS+=("$a")
done

if [[ "$WANT_WAREHOUSE" -eq 1 ]]; then
    echo "[WARN] Warehouse は NVIDIA の S3 から読む。ネットに出られない機では落ちる。"
elif [[ "$HAS_SCENE" -eq 0 ]]; then
    if [[ -f "$ROOM_USD" ]]; then
        ARGS+=(--scene-usd "$ROOM_USD")
        echo "[INFO] シーン: 実測した部屋 (assets/$(basename "$ROOM_USD"))"
        if [[ "$HAS_SPAWN" -eq 0 ]]; then
            ARGS+=(--x "$ROOM_SPAWN_X" --y "$ROOM_SPAWN_Y")
            echo "[INFO] スポーン: ($ROOM_SPAWN_X, $ROOM_SPAWN_Y)"
        fi
    else
        ARGS+=(--flat)
        echo "[WARN] 部屋の USD がありません: $ROOM_USD"
        echo "       平地で起動します。部屋で動かすには assets/ を用意すること。"
    fi
fi

# --viz が指定されていなければ GUI (kit) を既定にする
# （このバージョンの IsaacLab はヘッドレスが既定のため明示が必要）
if [[ "$HAS_VIZ" -eq 0 ]]; then
    ARGS+=(--viz kit)
fi

# G1 の頭部カメラ（camera.py）を使うには起動時の --enable_cameras が必要。
# 無いとカメラ拡張が読み込まれずセンサ構築時にエラーになる。
if [[ "$HAS_CAMERAS" -eq 0 ]]; then
    ARGS+=(--enable_cameras)
fi

# --enable_cameras 指定時、環境によっては拡張レジストリ（オンライン）への
# 到達性が無く「AppLauncher initialization complete」より前で無言のまま
# 数分〜無限に止まることがある（extscache に無いカメラ関連拡張をオンライン
# 解決しようとして固まる）。ローカルの extscache だけで完結させ、
# レジストリへは問い合わせないようにする。
# ユーザーが自分で --kit_args を指定している場合は上書きしない。
if [[ "$HAS_KIT_ARGS" -eq 0 ]]; then
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
