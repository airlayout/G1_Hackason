#!/usr/bin/env bash
# F3 の頭打ち（反復 50,000 以降 70%）への直し 2・3 を順に試す（10-27 ユーザー「1〜3 を実施して」。
# 作業ログ docs/作業ログ/2026-10-27_ドア課題_学習し直しF3の経過と頭打ちの見立て.html の 8 章・10 章）。
#
#   段 A（f3b）: いちばん良い重み（反復 53,750・77.0%）から、行動の雑音に上限 NOISE_MAX（既定 0.26＝45,150 の値）を付けて
#               ROUNDS_A 回（既定 6 回＝約 3.4 時間）。雑音が 0.26→0.33 と増え続けていた
#   段 B（f3c）: 同じ重みから、雑音の上限＋「押し開けの区間のまま扉の先 1.2 m より先へ歩き去る」罰（パッチ 0079、重み WALK_W 既定 −12）。
#               ROUNDS_B 回（既定 9999＝止めるまで）。段 B が終わったら f3_keepalive_1026.sh が段 B の続きを始め直す
# どちらも同じ重みから始めるので、段 A と段 B の差が罰の効き目になる（どちらも学習の条件は F3 と同じ・評価も同じ）。
#
# 止め方: ~/work_inoue/STOP_DOOR_F3 を置く（今の回の評価まで済ませて止まり、次の段も見張りも始めない）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_f3bc_1027.sh >> logs_f3bc_1027.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
SRC="${BEST_SRC:-$HOME/work_inoue/doorman6/VIRAL6_fix/logs_rl/door_f3/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261027_133521/last.pt}"
BEST_DIR="${BEST_DIR:-$HOME/work_inoue/doorman/ckpt/f3_best_it53750}"
CAP="++algo.config.max_noise_std=${NOISE_MAX:-0.26}"
WALK="++rewards.reward_scales.walk_away_in_swing=${WALK_W:--12.0}"
STOP_FILE="$HOME/work_inoue/STOP_DOOR_F3"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

# いちばん良い重みを、上書きされない場所へ控える（学習の設定と一緒に）
if [ ! -f "$BEST_DIR/last.pt" ]; then
  [ -f "$SRC" ] || { log "いちばん良い重みが無い: $SRC"; exit 1; }
  mkdir -p "$BEST_DIR" && cp "$SRC" "$BEST_DIR/" && cp "$(dirname "$SRC")"/config.yaml "$(dirname "$SRC")"/meta.yaml "$BEST_DIR/" 2>/dev/null
  log "控えた $BEST_DIR <- $SRC"
fi
BEST="$BEST_DIR/last.pt"

log "段 A（f3b・雑音の上限 ${CAP#*=}）を始める ck=$BEST rounds=${ROUNDS_A:-6}"
NAME=f3b TRAIN_MORE="$CAP" CK="$BEST" ROUNDS="${ROUNDS_A:-6}" PASS_RATE=1.1 ./exp_f3_retrain_1026.sh < /dev/null
if [ -e "$STOP_FILE" ]; then log "止め札（${STOP_FILE}）があるので段 B を始めずに終わる"; exit 0; fi

log "段 B（f3c・雑音の上限＋歩き去りの罰 ${WALK#*=}）を始める ck=$BEST rounds=${ROUNDS_B:-9999}"
export NAME=f3c TRAIN_MORE="$CAP,$WALK"
CK="$BEST" ROUNDS="${ROUNDS_B:-9999}" PASS_RATE=1.1 ./exp_f3_retrain_1026.sh < /dev/null
if [ -e "$STOP_FILE" ]; then log "止め札（${STOP_FILE}）があるので見張りを始めずに終わる"; exit 0; fi
[ "${NO_KEEPALIVE:-0}" = 1 ] && { log "NO_KEEPALIVE=1 なので見張りを始めずに終わる"; exit 0; }
log "段 B の続きは見張り（f3_keepalive_1026.sh）に任せる（name=${NAME} more=${TRAIN_MORE}）"
exec ./f3_keepalive_1026.sh
