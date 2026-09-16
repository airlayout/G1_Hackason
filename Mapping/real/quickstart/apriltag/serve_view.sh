#!/usr/bin/env bash
# 頭カメラのライブ配信を PC2 で起こす・止める。
#
# ⚠️ **`pkill -f serve_view` は使わない。** ssh 越しに撃つと、自分のコマンド行に
# `serve_view` が含まれるせいで**自分の shell を殺す**（2026-09-15 に 2 回踏んだ。
# ssh は終了コード 255 を返すだけで理由を言わない）。だから PID ファイルで扱う。
#
#   bash serve_view.sh start [追加の引数...]
#   bash serve_view.sh stop
#   bash serve_view.sh status
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ⚠️ **PID ファイルを /tmp に置かない。** 再起動で消えると `stop` が効かなくなり、
# プロセスだけ生き残ってカメラを握り続ける（2026-09-16 に踏んだ。
# 記録スクリプトが「画像 0 枚」で止まって初めて気づいた）。
PIDFILE="${G1_PC2_CFG:-$HOME/g1_cfg}/serve_view.pid"
LOGFILE="${G1_PC2_CFG:-$HOME/g1_cfg}/serve_view.log"

running() {
    [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null
}

case "${1:-status}" in
start)
    shift || true
    if running; then
        echo "すでに動いています (PID $(cat "$PIDFILE"))"
    else
        cd "$HERE"
        nohup python3 ./serve_view.py "$@" > "$LOGFILE" 2>&1 &
        echo $! > "$PIDFILE"
        sleep 6
        if ! running; then
            echo "⛔ 起動に失敗しました。ログ:"; tail -20 "$LOGFILE"; exit 1
        fi
        echo "起動しました (PID $(cat "$PIDFILE"))"
    fi
    grep -E '^  http://' "$LOGFILE" || true
    echo "止めるとき: bash serve_view.sh stop"
    ;;
stop)
    if running; then
        kill "$(cat "$PIDFILE")"
        for _ in $(seq 20); do running || break; sleep 0.2; done
        running && kill -9 "$(cat "$PIDFILE")" 2>/dev/null || true
        echo "止めました"
    else
        echo "動いていません"
    fi
    rm -f "$PIDFILE"
    ;;
status)
    if running; then
        echo "動いています (PID $(cat "$PIDFILE"))"
        grep -E '^  http://' "$LOGFILE" || true
    else
        echo "動いていません"
    fi
    ;;
*)
    echo "使い方: bash serve_view.sh {start|stop|status}" >&2; exit 2 ;;
esac
