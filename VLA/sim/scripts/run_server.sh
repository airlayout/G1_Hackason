#!/usr/bin/env bash
# G1 用 GR00T N1.6 の方策サーバーを起動する（ZMQ、既定 127.0.0.1:5555。前面で動き続ける）。
# checkpoint は環境変数 CKPT で切り替える（既定は env.sh）。
#
# 使い方: CKPT=cloudwalk-research/GR00T-N1.6-G1-PnPAppleToPlate bash run_server.sh
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"
require_groot_dir || exit 1

cd "$GROOT_DIR" || exit 1
echo "[server] ckpt=$CKPT port=$POLICY_PORT"
"$POLICY_PY" gr00t/eval/run_gr00t_server.py \
  --model-path "$CKPT" \
  --embodiment-tag UNITREE_G1 \
  --use-sim-policy-wrapper \
  --host "$POLICY_HOST" --port "$POLICY_PORT"
STATUS=$?
echo "[server] exit=$STATUS"
exit "$STATUS"
