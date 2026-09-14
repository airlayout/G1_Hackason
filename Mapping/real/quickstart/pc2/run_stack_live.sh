#!/usr/bin/env bash
# **PC2（機体の Orin NX）で測位 ＋ Nav2 を一度に起こす。**
#
#   bash run_stack_live.sh                          # 既定の地図と初期姿勢
#   bash run_stack_live.sh --init "[x, y, 0.0, yaw, 0.0, 0.0]"
#   G1_PC2_SECONDS=120 bash run_stack_live.sh       # 秒数を切る（既定 0 = 無限）
#   bash run_stack_live.sh --stop                   # 全部落とす
#
# ⚠️⚠️ **初期姿勢を疑わずに歩かせないこと。**
# 既定は地図の datum を焼いている。機体がそこに居なければ MOLA は**偽の極大に落ちる**。
# 09-13 の実機では真値から 2.26m・16.4° 外したまま 1 時間以上走り、
# `preflight.sh` は全項目 OK と答えた（`pose_quality` は誤りの方が高い）。
# ⇒ Mac 側の `bootstrap_localization.sh` が出した姿勢を `--init` で渡すこと。
#    （PC2 への移植は未了。段 4 で確認する）
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="${G1_PC2_CFG:-$HOME/g1_cfg}"
MAP_MM="${G1_MOLA_MAP:-$CFG/map/map.mm}"
INIT_POSE="${G1_MOLA_INIT_POSE:-[-0.0168, 0.0264, 0.0, 0.24, 0.0, 0.0]}"
SECONDS_LIMIT="${G1_PC2_SECONDS:-0}"
MOLA_WARMUP="${G1_PC2_MOLA_WARMUP:-18}"

say() { echo "[stack] $*"; }

stop_all() {
    say "止める（SIGINT → 猶予 → SIGKILL）"
    # ⚠️ **SIGINT で落とす。** SIGTERM だと内側の ros2 launch が子を孤児にする
    pkill -INT -f "ros2 launch" 2>/dev/null
    pkill -INT -f "run_mola_live.sh" 2>/dev/null
    pkill -INT -f "run_nav2_live.sh" 2>/dev/null
    sleep 3
    pkill -9 -f "ld-linux-aarch64" 2>/dev/null
    pkill -9 -f static_transform_publisher 2>/dev/null
    sleep 1
    say "残存プロセス: $(pgrep -cf ld-linux-aarch64)"
}

if [ "${1:-}" = "--stop" ]; then stop_all; exit 0; fi
while [ $# -gt 0 ]; do
    case "$1" in
        --init) INIT_POSE="$2"; shift 2 ;;
        --map)  MAP_MM="$2"; shift 2 ;;
        -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "[stack] 知らない引数: $1" >&2; exit 2 ;;
    esac
done

[ -f "$MAP_MM" ] || { echo "[stack] 地図が無い: $MAP_MM" >&2; exit 2; }
[ -x "$HERE/run_mola_live.sh" ] || { echo "[stack] run_mola_live.sh が無い" >&2; exit 2; }
[ -x "$HERE/run_nav2_live.sh" ] || { echo "[stack] run_nav2_live.sh が無い" >&2; exit 2; }

stop_all
say "初期姿勢: $INIT_POSE"

# ── 1. 測位（MOLA-LO）─────────────────────────────────────────────
say "[1] MOLA-LO を起こす（map -> base_link を出させる）"
G1_PC2_SECONDS="$SECONDS_LIMIT" nohup bash "$HERE/run_mola_live.sh" \
    --map "$MAP_MM" --init "$INIT_POSE" > /tmp/pc2_mola.log 2>&1 &
sleep "$MOLA_WARMUP"
if ! grep -q "Map loaded successfully" /tmp/pc2_mola.log 2>/dev/null; then
    echo "[stack] ⛔ MOLA が地図を読めていない。/tmp/pc2_mola.log を見ること" >&2
    tail -5 /tmp/pc2_mola.log >&2 2>/dev/null
    exit 1
fi
say "    地図を読み込んだ"

# ── 2. Nav2 ────────────────────────────────────────────────────────
# ⚠️ MOLA より**後**に起こす。map -> base_link が無いと costmap が
# `Timed out waiting for transform from base_link` を延々と出す
say "[2] Nav2 を起こす"
G1_PC2_SECONDS="$SECONDS_LIMIT" nohup bash "$HERE/run_nav2_live.sh" \
    > /tmp/pc2_nav2_run.log 2>&1 &
sleep 25

say "起動したもの:"
pgrep -af "ld-linux-aarch64" 2>/dev/null | sed -E 's|.*/(rootfs/opt/ros/humble/)?||; s/ --ros-args.*//' \
    | sed 's/^/  /' | sort -u | head -14
say "ログ: /tmp/pc2_mola.log /tmp/pc2_nav2.log /tmp/pc2_mapserver.log"
say "止める: bash $HERE/run_stack_live.sh --stop"
