#!/usr/bin/env bash
# 11-13（計画 docs/plan/2026-11-12-g1-door-scripted-stage123.md の 16 節）: 11-13 の実機の台本をシムで段 3c まで回す（学習なし）。
#   台本の置き場は ~/work_inoue/doorman/real_script_1113（11-12 の real_script とは別。11-12 の S0 を回し直せるように）
#   s0_script_1112.sh との違い: 台本の脚の指令を通す（walk）・関節の力を台本へ渡して手の力の見積もりを答え合わせ（tau・パッチ 0111）・
#   完了は扉 30°（3° でなく 3c の押し開けまで）・回の長さ 70 秒・TWIST=1 で台本の腰 yaw のひねり（半身）を入れる（パッチ 0111）
# 使い方:
#   N=32 TAGNAME=s13a PARAMS=script_params_run1_sim.json ./s0_script_1113.sh                       1 回目の再現
#   N=32 TAGNAME=s13b PARAMS=script_params_twist_sim.json TWIST=1 START_RANGE="[...]" ./s0_script_1113.sh   半身
#   START_RANGE は [x 下, x 上, y 下, y 上, 向き 下, 向き 上]（y は付け根から・シムの機体は置いた所から約 6 cm 後ろへ下がる）
#   MORE に足す hydra の引数（空白区切り）
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
N="${N:-32}"; NAME="${TAGNAME:-s13}"
CK="${CK:-$HOME/work_inoue/doorman/ckpt/start_1111_1654/last.pt}"
RS="${RS:-$HOME/work_inoue/doorman/real_script_1113}"
PARAMS="${PARAMS:-script_params_run1_sim.json}"
START_RANGE="${START_RANGE:-[-0.40,-0.34,-0.365,-0.285,-6.0,6.0]}"
OUTJ="$HOME/work_inoue/doorman/s0/${NAME}_$(date +%m%d_%H%M%S).jsonl"
mkdir -p "$HOME/work_inoue/doorman/s0"
FIT="$HOME/work_inoue/doorman/fit_loco_resp_1107.txt"
RESP_EVAL=$(grep -A1 '評価の引数' "$FIT" | tail -1)
TASK="${TASK:-door_en1154_g13_shaped_left_weight_realwall}"
ARGS="++env.config.task.name=$TASK ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,4000,4000,4000,600,400] ++env.config.max_episode_length_s=${EP_S:-70} ++seed=${SEED:-999}"
ARGS="$ARGS ++env.config.arm_stage.enabled=True ++env.config.arm_stage.start_range=$START_RANGE ++env.config.staged_reset_ratios=[0.7,0.0,0.15,0.15,0.0,0.0] ++rewards.reward_scales.penalty_standing_still=0.0"
ARGS="$ARGS ++env.config.arm_stage.complete_door_deg=${DOOR_DEG:-30.0} ++env.config.reset_on_complete_delay=10 ++env.config.arm_stage.free_legs_when_unlatched=True ++env.config.arm_stage.release=palm_press"
ARGS="$ARGS ++env.config.shoulder_roll_press_max_nm=6.0 ++algo.config.homie_switch_threshold=0.1 ++env.config.walk_in_hold_threshold=0.1 $RESP_EVAL"
ARGS="$ARGS ++env.config.script_teacher.enabled=True ++env.config.script_teacher.dir=$RS ++env.config.script_teacher.params=$RS/$PARAMS ++env.config.script_teacher.out=$OUTJ"
ARGS="$ARGS ++env.config.script_teacher.walk=True ++env.config.script_teacher.tau=True ++env.config.script_teacher.trace=${TRACE:-4}"
if [ "${TWIST:-0}" = 1 ]; then
  ARGS="$ARGS ++env.config.script_teacher.twist=True ++env.config.waist_yaw_comp.enabled=True ++env.config.waist_yaw_comp.gain=0.0 ++env.config.waist_yaw_comp.max_deg=40.0"
  ARGS="$ARGS ++env.config.waist_yaw_comp.hide_obs=True ++env.config.waist_yaw_comp.door_obs=pelvis"
fi
ARGS="$ARGS ${MORE:-}"
echo "=== [$(date "+%m-%d %T")] s0 1113: n=$N params=$PARAMS twist=${TWIST:-0} start=$START_RANGE out=$OUTJ"
EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=30 MEM_MAX=9G MINUTES="${MINUTES:-25}" IDLE_GATE=1 \
  EXTRA_ARGS="$ARGS" ./eval_from_start.sh "$CK" "$N" > "s0/${NAME}.log" 2>&1
echo "=== [$(date "+%m-%d %T")] s0 1113: 終わり $(tail -n 1 "s0/${NAME}.log") jsonl=$OUTJ lines=$(wc -l < "$OUTJ" 2>/dev/null)"
