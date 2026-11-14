#!/usr/bin/env bash
# 11-14（計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 D・E）: 合わせた扉（door_room4_fit_1114）で、
#   実機の台本を回しながら、実機の記録の pelvis の動きを再生する（パッチ 0113 の pelvis_replay）。台 i は回 LABELS[i % 回の数]。
#   全台を毎周期記録する（TRACE=N）。数えるのは Mac の s0_bench_cmp_1114.py（door_max_deg では数えない・11-13 夜の決まり）
# 使い方（OMEN の ~/work_inoue/doorman で）:
#   N=24 TAGNAME=rep7v PARAMS=script_params_rep7_sim.json LABELS=1,3,7 METHOD=vel ./s0_replay_1114.sh
#   TWIST=1 START_RANGE="[-0.44,-0.38,-0.205,-0.125,-6.0,6.0]" …  半身の組
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
N="${N:-24}"; NAME="${TAGNAME:-rep}"
CK="${CK:-$HOME/work_inoue/doorman/ckpt/start_1111_1654/last.pt}"
RS="${RS:-$HOME/work_inoue/doorman/real_script_1113}"
PARAMS="${PARAMS:-script_params_rep7_sim.json}"
REPLAY="${REPLAY:-$HOME/work_inoue/doorman/real_script_1113/pelvis_replay_1114.npz}"
START_RANGE="${START_RANGE:-[-0.40,-0.34,-0.365,-0.285,-6.0,6.0]}"
OUTJ="$HOME/work_inoue/doorman/s0/${NAME}_$(date +%m%d_%H%M%S).jsonl"
mkdir -p "$HOME/work_inoue/doorman/s0"
TASK="${TASK:-door_room4_fit_1114}"
ARGS="++env.config.task.name=$TASK ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,4000,4000,4000,600,400] ++env.config.max_episode_length_s=${EP_S:-30} ++seed=${SEED:-999}"
ARGS="$ARGS ++env.config.arm_stage.enabled=True ++env.config.arm_stage.start_range=$START_RANGE ++env.config.staged_reset_ratios=[0.7,0.0,0.15,0.15,0.0,0.0] ++rewards.reward_scales.penalty_standing_still=0.0"
ARGS="$ARGS ++env.config.arm_stage.complete_door_deg=80.0 ++env.config.reset_on_complete_delay=500 ++env.config.shoulder_roll_press_max_nm=6.0 ++algo.config.homie_switch_threshold=0.1 ++env.config.walk_in_hold_threshold=0.1"
ARGS="$ARGS ++env.config.script_teacher.enabled=True ++env.config.script_teacher.dir=$RS ++env.config.script_teacher.params=$RS/$PARAMS ++env.config.script_teacher.out=$OUTJ"
ARGS="$ARGS ++env.config.script_teacher.walk=True ++env.config.script_teacher.tau=True ++env.config.script_teacher.tau_src=applied ++env.config.script_teacher.trace=$N"
ARGS="$ARGS ++env.config.virtual_latch.enabled=True ++domain_rand.randomize_friction=False"
# 0113b: 回の始めの腕を実機の純正の腕を下ろした形へ（11-13 朝の記録の 2a の始めの中央・SDK の並びの左腕 7・右腕 7）
INIT_ARM="${INIT_ARM:-[0.28,0.20,-0.02,0.99,0.19,0.04,-0.02,0.28,-0.20,0.03,1.00,-0.23,0.03,0.02]}"
[ "$INIT_ARM" != off ] && ARGS="$ARGS ++env.config.script_teacher.init_arm=$INIT_ARM"
if [ "${METHOD:-vel}" != off ]; then
  ARGS="$ARGS ++env.config.pelvis_replay.enabled=True ++env.config.pelvis_replay.file=$REPLAY ++env.config.pelvis_replay.method=${METHOD:-vel} ++env.config.pelvis_replay.labels=[${LABELS:-1,3,7}]"
  ARGS="$ARGS ++env.config.pelvis_replay.hold_from=${HOLD_FROM:-2b} ++env.config.pelvis_replay.start_sub=${START_SUB:-3a}"
fi
if [ "${TWIST:-0}" = 1 ]; then
  ARGS="$ARGS ++env.config.script_teacher.twist=True ++env.config.waist_yaw_comp.enabled=True ++env.config.waist_yaw_comp.gain=0.0 ++env.config.waist_yaw_comp.max_deg=40.0"
  ARGS="$ARGS ++env.config.waist_yaw_comp.hide_obs=True ++env.config.waist_yaw_comp.door_obs=pelvis"
fi
ARGS="$ARGS ${MORE:-}"
echo "=== [$(date "+%m-%d %T")] replay 1114: n=$N task=$TASK params=$PARAMS method=${METHOD:-vel} labels=${LABELS:-1,3,7} twist=${TWIST:-0} out=$OUTJ"
EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=30 MEM_MAX=9G MINUTES="${MINUTES:-20}" IDLE_GATE=1 \
  EXTRA_ARGS="$ARGS" ./eval_from_start.sh "$CK" "$N" > "s0/${NAME}.log" 2>&1
echo "=== [$(date "+%m-%d %T")] replay 1114: 終わり $(tail -n 1 "s0/${NAME}.log") jsonl=$OUTJ lines=$(wc -l < "$OUTJ" 2>/dev/null)"
