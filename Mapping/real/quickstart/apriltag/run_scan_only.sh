#!/usr/bin/env bash
# `/scan` を出すノードだけを起こす（測位は起こさない）。
#
# ## なぜこれだけで足りるのか
#
# タグの登録に測位は要らない。姿勢は `global_localize.py` がスキャン 1 枚から
# 地図全体を探索して求める（2026-09-16 の実測で AMCL より正確だった）。
# だから必要なのは **`/scan` が出ていること**だけである。
#
# ⚠️ `/tmp` は再起動で消える。2026-09-16 に PC2 が再起動して画像 86 枚を失った。
# **記録は `~/g1_cfg/runs/` に置くこと。**
#
#   bash run_scan_only.sh start   /   stop   /   status
set -euo pipefail

JAMMY="${G1_JAMMY:-$HOME/jammy_ros}"
NODES="${G1_PC2_NODES:-$HOME/g1_cfg}"
PIDFILE="$HOME/g1_cfg/scan_only.pid"
LOGFILE="$HOME/g1_cfg/scan_only.log"
BAND_LO="${G1_SCAN_MIN_HEIGHT:-1.30}"
BAND_HI="${G1_SCAN_MAX_HEIGHT:-1.80}"
RANGE_MAX="${G1_SCAN_RANGE_MAX:-20.0}"

running() { [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; }

case "${1:-status}" in
start)
    if running; then echo "すでに動いています (PID $(cat "$PIDFILE"))"; exit 0; fi
    # shellcheck source=/dev/null
    . "$JAMMY/env.sh"
    export ROS_DOMAIN_ID="${G1_PC2_DOMAIN:-0}"
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${G1_PC2_DDS_NIC:-eth0}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
    # ⚠️ `jrun` はシェル関数。`&` で起こすと `$!` が中間のサブシェルを指し、孤児が出る
    # shellcheck disable=SC2086
    env $JAMMY_ENV "$LOADER" --library-path "$JAMMY_LIBS" \
        "$PREFIX/usr/bin/python3.10" "$NODES/cloud_to_scan.py" \
        --min-height "$BAND_LO" --max-height "$BAND_HI" --range-max "$RANGE_MAX" \
        > "$LOGFILE" 2>&1 &
    echo $! > "$PIDFILE"
    sleep 6
    if ! running; then echo "⛔ 起動に失敗:"; tail -15 "$LOGFILE"; exit 1; fi
    echo "起動しました (PID $(cat "$PIDFILE")) 帯 ${BAND_LO}〜${BAND_HI} m"
    tail -3 "$LOGFILE"
    ;;
stop)
    if running; then
        kill "$(cat "$PIDFILE")" 2>/dev/null || true
        for _ in $(seq 20); do running || break; sleep 0.2; done
        running && kill -9 "$(cat "$PIDFILE")" 2>/dev/null || true
        echo "止めました"
    else echo "動いていません"; fi
    rm -f "$PIDFILE"
    ;;
status)
    if running; then echo "動いています (PID $(cat "$PIDFILE"))"; tail -2 "$LOGFILE"
    else echo "動いていません"; fi
    ;;
*) echo "使い方: bash run_scan_only.sh {start|stop|status}" >&2; exit 2 ;;
esac
