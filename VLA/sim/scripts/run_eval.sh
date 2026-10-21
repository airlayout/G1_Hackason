#!/usr/bin/env bash
# サーバーを起動 → ポートが開くまで待つ → rollout → サーバーを止める、を 1 回で行う。
# サーバーは PID で止める（`pkill -f` は ssh 越しだと自分のシェルに一致して落ちるため使わない）。
#
# 使い方: CKPT=cloudwalk-research/GR00T-N1.6-G1-PnPAppleToPlate N_EP=3 bash run_eval.sh
#   WAIT_SEC   サーバー起動の待ち時間（既定 900。初回は checkpoint の取得で数分かかる）
#   LOG_DIR    サーバーログの置き場（既定 ./logs。.gitignore 済み）
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"
require_groot_dir || exit 1

WAIT_SEC="${WAIT_SEC:-900}"
LOG_DIR="${LOG_DIR:-$PWD/logs}"
mkdir -p "$LOG_DIR"
SERVER_LOG="$LOG_DIR/server_$(date +%Y%m%d_%H%M%S).log"

port_open() { (exec 3<>"/dev/tcp/$POLICY_HOST/$POLICY_PORT") 2>/dev/null; }
if port_open; then
  echo "[eval] $POLICY_HOST:$POLICY_PORT は既に使われています。別のサーバーが動いていないか確認してください。" >&2
  exit 1
fi

bash "$HERE/run_server.sh" > "$SERVER_LOG" 2>&1 &
SERVER_PID=$!
# 正常終了でも失敗でも、起動したサーバーは必ず止める。
# shellcheck disable=SC2329  # trap から呼ばれる
cleanup() {
  pkill -P "$SERVER_PID" 2>/dev/null
  kill "$SERVER_PID" 2>/dev/null
}
trap cleanup EXIT
echo "[eval] server pid=$SERVER_PID ckpt=$CKPT log=$SERVER_LOG"

waited=0
until port_open; do
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "[eval] サーバーが落ちました。ログ末尾:" >&2
    tail -n 20 "$SERVER_LOG" >&2
    exit 1
  fi
  if [ "$waited" -ge "$WAIT_SEC" ]; then
    echo "[eval] ${WAIT_SEC}s 待ってもポートが開きません。ログ: $SERVER_LOG" >&2
    exit 1
  fi
  sleep 5
  waited=$((waited + 5))
done
echo "[eval] server up after ~${waited}s"

bash "$HERE/run_client.sh"
STATUS=$?
echo "[eval] client exit=$STATUS"
exit "$STATUS"
