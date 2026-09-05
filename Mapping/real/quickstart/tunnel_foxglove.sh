#!/usr/bin/env bash
# Mac 側。PC2 の Foxglove Bridge を localhost に引き込む。
# app.foxglove.dev は HTTPS なので ws:// のリモート直接接続は混在コンテンツとして
# 遮断される。localhost 宛だけは例外なので、トンネル経由なら繋がる。
#
#   bash tunnel_foxglove.sh          # 開始
#   bash tunnel_foxglove.sh stop     # 停止
set -uo pipefail
PORT="${FOXGLOVE_PORT:-8765}"
HOST="${G1_SSH_HOST:-g1}"
LOG="${TMPDIR:-/tmp}/g1_foxglove_tunnel.log"
PATTERN="ssh -N .*-L ${PORT}:localhost:${PORT}"

stop_existing() {
    pkill -f -- "$PATTERN" >/dev/null 2>&1 && { echo "[tunnel] 既存を停止"; sleep 1; }
}

if [ "${1:-}" = "stop" ]; then
    stop_existing
    echo "[tunnel] 停止しました"
    exit 0
fi

stop_existing
# nohup + 全 FD を切り離す。付けたままだと呼び出し元が終了を待ち続ける。
nohup ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 \
      -L "${PORT}:localhost:${PORT}" "$HOST" >"$LOG" 2>&1 </dev/null &
disown
sleep 3

if lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then
    echo "[tunnel] localhost:${PORT} で待受中"
    echo "[tunnel] ブラウザ → https://app.foxglove.dev"
    echo "[tunnel]   Open connection → Foxglove WebSocket → ws://localhost:${PORT}"
else
    echo "[tunnel] 待受に失敗しました。ログ: $LOG" >&2
    tail -5 "$LOG" >&2
    exit 1
fi
