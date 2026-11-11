#!/usr/bin/env bash
# 11-07（案 A・計画 docs/plan/2026-11-06-g1-door-depth-rl.md の 7.6）: 実機の記録から作った純正の応え方（パッチ 0095 の
# env.config.loco_resp。値は run_loco_resp_1107.sh の fit_loco_resp_1107.txt）と、評価と同じ段階の持ち時間で、教師を学び直す。
#   1. 教師 f3t1 をその脚で 256 回評価（学習なし。元の脚 93.8%・昨夜の一様に鈍い脚 51.6%）
#   2. f3t1 から学び直す（強化学習 PPO。段の名前 f3t3。T1 の肩 roll の切り詰めと罰・ほかの引数は f3t1・f3t2 と同じ）
#      持ち時間: 段階ごと [250,100,100,100,600,200] → 評価と同じ [500,200,200,200,600,400] 歩、回の上限 30 → 45 秒。
#      昨夜の f3t2 は 3 区切り目から崩れた（鈍い脚で時間内に終えられない回が多いと罰を減らす方へ学ぶ見込み）
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup ./t3_1107.sh >> logs_t3_1107.log 2>&1 < /dev/null &
#   止め札: ~/work_inoue/STOP_DOOR_F3（区切りの境目で止まる）。評価を飛ばす: SKIP_EVAL=1
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
log() { echo "=== [$(date "+%m-%d %T")] t3: $*"; }
CK="${CK:-$HOME/work_inoue/doorman6/VIRAL6_fix/logs_rl/door_f3t1/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261106_101525/last.pt}"
FIT="${FIT:-$HOME/work_inoue/doorman/fit_loco_resp_1107.txt}"
TASK=door_en1154_g13_shaped_left_weight
T1="++env.config.shoulder_roll_press_max_nm=6.0"
PEN="++rewards.reward_scales.penalty_shoulder_roll_press=-0.01"
RESP_EVAL=$(grep -A1 '評価の引数' "$FIT" | tail -1)
RESP_CHAIN=$(grep -A1 '学習の連鎖の引数' "$FIT" | tail -1)
case "$RESP_EVAL" in ++env.config.loco_resp.enabled=true*) ;; *) log "応え方の層の値が無い: $FIT"; exit 1 ;; esac
SWITCH="++algo.config.homie_switch_threshold=0.1,++env.config.walk_in_hold_threshold=0.1"
STAGE="++env.config.max_stage_time=[500|200|200|200|600|400],++env.config.max_episode_length_s=45"
EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
[ -f "$CK" ] || { log "出発点の重みが無い: $CK"; exit 1; }
grep -qF "_init_loco_resp" "$HOME/work_inoue/doorman6/VIRAL6_fix/gr00t/rl/envs/base_task/homie_base.py" ||
  { log "パッチ 0095 が VIRAL6_fix に当たっていない"; exit 1; }
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
if [ "${SKIP_EVAL:-0}" != 1 ]; then
  log "評価 f3t1＋実機に合わせた脚 ck=$CK"
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion through" MAX_WAIT_MIN=120 MEM_MAX=9G \
    EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA $T1 $(echo "$SWITCH" | tr ',' ' ') $RESP_EVAL" \
    ./eval_from_start.sh "$CK" 256 > logs_t3_1107_eval_base.log 2>&1
  log "評価 終わり: $(tail -n 2 logs_t3_1107_eval_base.log | tr "\n" " ")"
fi
log "学び直し f3t3 を始める"
SKIP_A=1 MASK=0 COMPLETE_THROUGH=1 TERM_UP=0.9 B_NAME=f3t3 AUTO_ENVS=1 IDLE_GATE="${IDLE_GATE:-1}" CK="$CK" \
  STOCK_MORE="$SWITCH,$RESP_CHAIN" LONG_MORE="$STAGE" EXTRA_MORE="$T1,$PEN" ./exp_stocklegs_1030.sh
log "終わり"
