#!/usr/bin/env bash
# **静止のまま 1 本録って、測位候補を採点する。** 段 A/B/C を同じ物差しで比べるための入口。
#
#   bash quickstart/pc2/measure_still.sh mola_odom 8
#   bash quickstart/pc2/measure_still.sh amcl 8
#
# ⚠️ 候補は 1 つずつ動かすこと。2 つが同時に map->base_link を出すと /tf が壊れ、
#    比較にならないどころか、CPU 競合という調査対象そのものを混ぜてしまう。
set -uo pipefail

LABEL="${1:?使い方: measure_still.sh <ラベル> [秒]}"
SECS="${2:-8}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"   # …/G1_Hackason
REPO="$(dirname "$ROOT")"                                          # …/physical_ai
PC2="unitree@10.42.0.76"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 -i "$HOME/.ssh/id_ed25519_g1" -o IdentitiesOnly=yes)
OUT="$ROOT/Mapping/real/runs/_meas/$LABEL"
REF="$ROOT/Mapping/real/runs/20260906T135940_UiS_room_v3/map/nav_map_ref.yaml"
VENV="$ROOT/Navigation/.venv/bin/python"
INIT="${G1_INIT_POSE:-0.703 12.966 -57.5}"

say() { printf '[meas] %s\n' "$*" >&2; }

# ── 1. 何が map->base_link を出しているかを先に見る（2 つ動いていたら止める）──
say "PC2 で動いている測位を確認"
RUNNING="$("${SSH[@]}" "$PC2" 'pgrep -af "mola-cli|amcl|fastlio|fast_lio" | grep -v "pgrep" | wc -l' 2>/dev/null)"
say "  測位らしきプロセス: ${RUNNING} 本"
[ "${RUNNING:-0}" -eq 0 ] && { echo "[meas] ⛔ 測位が 1 つも動いていない。先に候補を起動すること" >&2; exit 2; }

# ── 1.5 録るトピックを正典から読む ────────────────────────────────────
#
# ⚠️ **直書きしない。** 2026-09-15 に 4 度目の「録っておけばよかった」を踏んだ——
# ここが点群と /tf 系の 3 件しか録っていなかったため、静止記録に **IMU が無く**、
# GLIM の評価が指定の 3 本では**成立しなかった**（IMU 無しでは CT-ICP に落ち、
# 事前地図とサブマップ原点の規約が食い違って yaw が 113° 外れる）。
# 過去 3 回（09-10 IMU / 09-12 脚 odom / 09-13 /cmd_vel）と同じ型である。
#
# ⚠️ **存在しないトピックを並べても記録は壊れない**（2026-09-14 実測）。
# rosbag2 は在るものだけ購読して録り続ける。だから全部並べてよい。
TOPICS_FILE="${G1_TOPICS_FILE:-$ROOT/Mapping/real/quickstart/record_topics.txt}"
REC_TOPICS="$(awk '/^[[:space:]]*#/{next} $2=="sensor"||$2=="ros"{printf "%s ", $1}' "$TOPICS_FILE" 2>/dev/null | sed 's/ *$//')"
[ -n "$REC_TOPICS" ] || { echo "⛔ $TOPICS_FILE から記録トピックを読めない" >&2; exit 2; }
say "録るトピック $(echo "$REC_TOPICS" | wc -w | tr -d ' ') 件"

# ── 2. 録る ────────────────────────────────────────────────────────────
say "${SECS} 秒録る（機体は静止のまま。何も動かさない）"
"${SSH[@]}" "$PC2" "
source ~/jammy_ros/env.sh
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"eth0\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>'
rm -rf /tmp/meas_$LABEL
( jros2 bag record -o /tmp/meas_$LABEL $REC_TOPICS > /tmp/meas_$LABEL.log 2>&1 ) &
sleep $((SECS + 1))
pkill -INT -f 'bin/ros2 bag recor[d]'
sleep 2
" >/dev/null 2>&1

# ── 3. 持ってくる ──────────────────────────────────────────────────────
mkdir -p "$(dirname "$OUT")"; rm -rf "$OUT"
scp -q -r -o BatchMode=yes -o ConnectTimeout=10 -i "$HOME/.ssh/id_ed25519_g1" -o IdentitiesOnly=yes \
    "$PC2:/tmp/meas_$LABEL" "$OUT" || { echo "[meas] ⛔ 記録を取り出せない" >&2; exit 2; }
say "取得: $OUT ($(du -sh "$OUT" | cut -f1))"

# ── 4. 採点 ────────────────────────────────────────────────────────────
# shellcheck disable=SC2086
"$VENV" "$(dirname "${BASH_SOURCE[0]}")/still_report.py" "$OUT" --ref "$REF" --init $INIT --label "$LABEL"
