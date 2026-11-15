#!/usr/bin/env bash
# 11-12（計画 docs/plan/2026-11-12-g1-door-script-shaped-rl.md の S0）: 実機の台本をシムの腕だけの段でそのまま回す（学習なし）。
#   方策の腕の出力は台本が書き換える（パッチ 0108 script_teacher）。脚は掛け金が外れるまで立つ HOMIE。完了は扉 3°
# 使い方: N=64 TAGNAME=s0a START_RANGE="[-0.44,-0.36,-0.365,-0.285,-6.0,6.0]" PARAMS=script_params.json ./s0_script_1112.sh
#   MORE に足す hydra の引数（空白区切り）
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
N="${N:-16}"; NAME="${TAGNAME:-s0}"
CK="${CK:-$HOME/work_inoue/doorman/ckpt/start_1111_1654/last.pt}"
RS="$HOME/work_inoue/doorman/real_script"
PARAMS="${PARAMS:-script_params.json}"
START_RANGE="${START_RANGE:-[-0.44,-0.36,-0.365,-0.285,-6.0,6.0]}"
OUTJ="$HOME/work_inoue/doorman/s0/${NAME}_$(date +%m%d_%H%M%S).jsonl"
mkdir -p "$HOME/work_inoue/doorman/s0"
FIT="$HOME/work_inoue/doorman/fit_loco_resp_1107.txt"
RESP_EVAL=$(grep -A1 '評価の引数' "$FIT" | tail -1)
TASK="${TASK:-door_en1154_g13_shaped_left_weight_realwall}"
ARGS="++env.config.task.name=$TASK ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,3000,3000,3000,600,400] ++env.config.max_episode_length_s=${EP_S:-30} ++seed=${SEED:-999}"
ARGS="$ARGS ++env.config.arm_stage.enabled=True ++env.config.arm_stage.start_range=$START_RANGE ++env.config.staged_reset_ratios=[0.7,0.0,0.15,0.15,0.0,0.0] ++rewards.reward_scales.penalty_standing_still=0.0"
ARGS="$ARGS ++env.config.arm_stage.complete_door_deg=${DOOR_DEG:-3.0} ++env.config.reset_on_complete_delay=10 ++env.config.arm_stage.free_legs_when_unlatched=True ++env.config.arm_stage.release=palm_press"
ARGS="$ARGS ++env.config.shoulder_roll_press_max_nm=6.0 ++algo.config.homie_switch_threshold=0.1 ++env.config.walk_in_hold_threshold=0.1 $RESP_EVAL"
ARGS="$ARGS ++env.config.script_teacher.enabled=True ++env.config.script_teacher.dir=$RS ++env.config.script_teacher.params=$RS/$PARAMS ++env.config.script_teacher.out=$OUTJ ${MORE:-}"
echo "=== [$(date "+%m-%d %T")] s0: n=$N params=$PARAMS start=$START_RANGE out=$OUTJ"
EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=30 MEM_MAX=9G MINUTES="${MINUTES:-15}" IDLE_GATE=1 \
  EXTRA_ARGS="$ARGS" ./eval_from_start.sh "$CK" "$N" > "s0/${NAME}.log" 2>&1
echo "=== [$(date "+%m-%d %T")] s0: 終わり $(tail -n 1 "s0/${NAME}.log") jsonl=$OUTJ lines=$(wc -l < "$OUTJ" 2>/dev/null)"
