#!/usr/bin/env bash
# 11-14（計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 C）: 扉だけを動かす試し（パッチ 0112 の lever_probe）。
#   機体は扉の 1.8〜1.9 m 手前に立たせ（腕は届かない）、台 i のレバーに 0〜HMAX N·m の下げる回りを 1 秒から、蝶番に開ける向きの
#   HINGE N·m を 4 秒から掛ける。レバーが落ち着いた角度でばねを、蝶番が開いたかで掛け金が外れる角度を確かめる（probe_lever_1114.py）。
# 使い方（OMEN の ~/work_inoue/doorman で）: N=32 TASK=door_room4_fit_1114 ./s0_probe_1114.sh
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
N="${N:-32}"; NAME="${TAGNAME:-probe1114}"
CK="${CK:-$HOME/work_inoue/doorman/ckpt/start_1111_1654/last.pt}"
TASK="${TASK:-door_room4_fit_1114}"
ARGS="++env.config.task.name=$TASK ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,4000,4000,4000,600,400] ++env.config.max_episode_length_s=${EP_S:-9} ++seed=${SEED:-999}"
ARGS="$ARGS ++env.config.arm_stage.enabled=True ++env.config.arm_stage.start_range=[-1.9,-1.8,-0.35,-0.30,0.0,0.0] ++env.config.staged_reset_ratios=[0.7,0.0,0.15,0.15,0.0,0.0] ++rewards.reward_scales.penalty_standing_still=0.0"
ARGS="$ARGS ++env.config.lever_probe.enabled=True ++env.config.lever_probe.handle_torque_max=${HMAX:-2.5} ++env.config.lever_probe.handle_start_s=1.0"
ARGS="$ARGS ++env.config.lever_probe.hinge_torque=${HINGE:-30.0} ++env.config.lever_probe.hinge_start_s=4.0 ++domain_rand.randomize_friction=False"
ARGS="$ARGS ++env.config.virtual_latch.enabled=${VLATCH:-True} ++env.config.arm_stage.complete_door_deg=80.0 ++env.config.reset_on_complete_delay=500 ${MORE:-}"
echo "=== [$(date "+%m-%d %T")] probe 1114: n=$N task=$TASK hmax=${HMAX:-2.5} hinge=${HINGE:-30.0}"
EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=30 MEM_MAX=9G MINUTES="${MINUTES:-15}" IDLE_GATE=1 \
  EXTRA_ARGS="$ARGS" ./eval_from_start.sh "$CK" "$N" > "s0/${NAME}.log" 2>&1
echo "=== [$(date "+%m-%d %T")] probe 1114: 終わり $(tail -n 1 "s0/${NAME}.log")"
