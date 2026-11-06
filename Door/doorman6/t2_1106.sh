#!/usr/bin/env bash
# 11-06 夜（T2・計画 docs/plan/2026-11-06-g1-door-depth-rl.md の 7.5）: 実機の純正の歩行は方策の指令に
# 前後 0.52・左右 0.36・旋回 0.61 倍しか応えず、遅れは 0.26・0.56・0.10 秒（11-06 の 8 回の記録・Door/real/ana_stock_gain_1106.py）。
# シミュレータの純正に似せた脚（倍率 0.8〜1.2・遅れ 0〜15 歩）より大幅に鈍い。D6 の 2 回は扉の 0.5 m 手前で留まり右へ回った。
# 脚の設定だけを 倍率 0.3〜0.8・遅れ 5〜28 歩（0.1〜0.56 秒）に替えて（パッチ 0085 の既存の設定。コードは変えない）:
#   1. 教師 f3t1 をその脚で 256 回評価（学習なし。f3t1 は元の脚で 93.8%）
#   2. 同じ重みから学び直す（強化学習・PPO。段の名前 f3t2。T1 の肩 roll の切り詰めと罰・ほかの引数は f3t1 と同じ）
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup ./t2_1106.sh >> logs_t2_1106.log 2>&1 < /dev/null &
#   止め札: ~/work_inoue/STOP_DOOR_F3（区切りの境目で止まる）。評価を飛ばす: SKIP_EVAL=1。倍率・遅れ: GAIN=0.3\|0.8 DELAY=5\|28
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
log() { echo "=== [$(date "+%m-%d %T")] t2: $*"; }
CK="${CK:-$HOME/work_inoue/doorman6/VIRAL6_fix/logs_rl/door_f3t1/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261106_101525/last.pt}"
TASK=door_en1154_g13_shaped_left_weight
T1="++env.config.shoulder_roll_press_max_nm=6.0"
PEN="++rewards.reward_scales.penalty_shoulder_roll_press=-0.01"
GAIN="${GAIN:-0.3|0.8}"
DELAY="${DELAY:-5|28}"
LEGS="++algo.config.homie_switch_threshold=0.1,++env.config.walk_in_hold_threshold=0.1,++env.config.loco_cmd_delay_steps_range=[$DELAY],++env.config.loco_cmd_gain_range=[$GAIN]"
EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
[ -f "$CK" ] || { log "出発点の重みが無い: $CK"; exit 1; }
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
if [ "${SKIP_EVAL:-0}" != 1 ]; then
  log "評価 f3t1＋鈍い脚 ck=$CK legs=$LEGS"
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion through" MAX_WAIT_MIN=60 MEM_MAX=9G \
    EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA $T1 $(echo "$LEGS" | tr ',' ' ' | tr '|' ',')" \
    ./eval_from_start.sh "$CK" 256 > logs_t2_1106_eval_base.log 2>&1
  log "評価 終わり: $(tail -n 2 logs_t2_1106_eval_base.log | tr "\n" " ")"
fi
log "学び直し f3t2 を始める"
SKIP_A=1 MASK=0 COMPLETE_THROUGH=1 TERM_UP=0.9 B_NAME=f3t2 AUTO_ENVS=1 IDLE_GATE="${IDLE_GATE:-1}" CK="$CK" \
  STOCK_MORE="$LEGS" EXTRA_MORE="$T1,$PEN" ./exp_stocklegs_1030.sh
log "終わり"
