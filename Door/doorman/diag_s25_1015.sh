#!/usr/bin/env bash
# 段 2.5（2026-10-15）の診断: 状態入力の生徒は学習を続けるほど落ちた（23 → 52 → 23 → 15%、52% の 950 から学習率 1e-4 で続けても 16%）。
# 950 の重みで 2 本だけ評価して、原因を分ける（どちらも 256 回・立った姿勢・時間 2 倍・扉 _weight）:
#   1. 乱数の種を替える（別の扉の組）: 52% がまぐれでないか
#   2. LiDAR と測位の誤差・遅れ・欠けを全部 0: 計測器の崩しが原因か、見えない情報（力・重さ・レバーの角度）が原因か
# 使い方（OMEN の ~/work_inoue/doorman で）: IDLE_GATE=0 setsid nohup ./diag_s25_1015.sh > diag_s25_1015.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="$HOME/work_inoue/doorman/ckpt/start_1015_1428/last.pt"
TASK=door_en1154_g13_shaped_left_weight
EVAL_EXTRA="++env.config.task.name=$TASK ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
S="++env.config.door_sensor"
PERFECT="$S.rate_hz=50.0 $S.latency_s=0.0 $S.angle_noise_deg=0.0 $S.angle_bias_deg=0.0 $S.dropout=0.0 $S.dropout_near_hand=0.0 $S.max_incidence_deg=90.0 $S.loc_pos_bias_m=0.0 $S.loc_pos_noise_m=0.0 $S.loc_yaw_bias_deg=0.0 $S.loc_yaw_noise_deg=0.0 $S.geometry_noise_m=0.0"
export COUNT_ARGS="--criterion through" MINUTES=20 IDLE_GATE="${IDLE_GATE:-1}"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

log "diag_s25 start ck=$CK"
log "1. 種を替える（++seed=1）"
EXTRA_ARGS="$EVAL_EXTRA ++seed=1" ./eval_from_start.sh "$CK" 256 > diag_s25_seed1.log 2>&1
grep -a "eval_from_start out=" diag_s25_seed1.log
log "2. 計測器の崩しを 0（10 Hz・遅れ・揺れ・片寄り・欠けなし）"
EXTRA_ARGS="$EVAL_EXTRA $PERFECT" ./eval_from_start.sh "$CK" 256 > diag_s25_perfect.log 2>&1
grep -a "eval_from_start out=" diag_s25_perfect.log
log "diag_s25_done"
