#!/usr/bin/env bash
# 11-04（画像だけの生徒 V3）: 生徒（vision_door_student.VisionDoorStudent）が f3sc 27 回目を真似る DAgger を、区切り（ROUND_MIN 分）
# ごとに扉の種（seed）を替えて回す。扉は台を作るときに種から引くので、区切りごとに NUM_ENVS 枚の新しい扉になる。
# 重みは区切りの最後の保存（last.pt）から続ける。機体・扉・報酬・乱択の引数は f3sc 27 回目と同じ（f3sc の .hydra/overrides.yaml）。
#
# 使い方（OMEN の ~/work_inoue/doorman6）:
#   ENC=<V2 の出力.pt> NUM_ENVS=256 ROUNDS=48 ROUND_MIN=30 setsid nohup bash exp_vis_1104.sh <名前> >> vis_1104/<名前>.log 2>&1 < /dev/null &
#   CK=<続きの last.pt>: 途中から。SEED0: 最初の区切りの種（既定 1000。区切り r は SEED0 + r）
#   止め札: ~/work_inoue/STOP_DOOR_VIS（区切りの境目で止まる）
set -uo pipefail
NAME="$1"
B="$HOME/work_inoue/doorman6"
BASE="$HOME/work_inoue/doorman"
REPO="${REPO:-VIRAL6_vis}"
TEACHER="${TEACHER:-$BASE/ckpt/start_1103_0807/last.pt}"
ENC="${ENC:?V2 の出力（画像の網と扉の頭）を ENC に渡す}"
N="${NUM_ENVS:-256}"
ROUNDS="${ROUNDS:-48}"
ROUND_MIN="${ROUND_MIN:-30}"
SEED0="${SEED0:-1000}"
# V4（PPO で磨く）: EXP=wbmanip/door_open_homie_29dof_vision_f3_ppo-lstm CK=<make_vis_ppo_ckpt.py の出力>
EXP="${EXP:-wbmanip/door_open_homie_29dof_vision_f3_dagger-lstm}"
MEM_MAX="${MEM_MAX:-12G}"
NEED_MB=$(( ${MEM_MAX%G} * 1024 + ${NEED_MARGIN_MB:-256} ))
STOP_FILE="$HOME/work_inoue/STOP_DOOR_VIS"
export PATH="$HOME/miniforge3/envs/doorman6/bin:$PATH" OMNI_KIT_ACCEPT_EULA=YES HYDRA_FULL_ERROR=1 \
  XDG_RUNTIME_DIR="/run/user/$(id -u)" PYTHONUNBUFFERED=1 TMPDIR="$B/tmp" PYTHONPATH="$B/$REPO" \
  PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
source "$BASE/gate.sh"
export IGNORE_PAUSE=1
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
mkdir -p "$B/vis_1104/$NAME"
cd "$B/$REPO" || exit 1
[ -f "$ENC" ] || { log "ENC が無い: $ENC"; exit 1; }
# f3sc 27 回目の起動の引数のうち、機体・扉・報酬・乱択（PPO の学習の設定と exp・台数・重み・名前は除く）
F3_ARGS=(
  ++env.config.delta_action_scale=0.3 ++env.config.task.name=door_en1154_g13_shaped_left_weight
  ++env.config.push_door_force_max=50 ++env.config.push_unlatched_force_max=50 ++rewards.reward_scales.push_door_force=0.12
  ++env.config.door_through_angle_deg=70 ++env.config.door_hold_open_angle_deg=90 ++env.config.mirror_left_doors=True
  ++env.config.push_door_hinge_max_deg=80 ++env.config.door_overopen_start_deg=85 ++rewards.reward_scales.door_overopen=-6.0
  "++env.config.arm_accum_guard_delta=[0.2,0.2,0.3,0.3,0.3,0.2,0.2,0.2,0.2,0.3,0.3,0.3,0.2,0.2]"
  ++env.config.arm_accum_guard_margin=0.05 ++env.config.clip_targets_to_limits_margin=0.05 ++env.config.target_step_limit=0.7
  "++robot.dof_armature_list=[0.01017752004,0.025101925,0.01017752004,0.025101925,0.00721945,0.00721945,0.01017752004,0.025101925,0.01017752004,0.025101925,0.00721945,0.00721945,0.01017752004,0.00721945,0.00721945,0.003609725,0.003609725,0.003609725,0.003609725,0.003609725,0.00425,0.00425,0.003609725,0.003609725,0.003609725,0.003609725,0.003609725,0.00425,0.00425]"
  ++simulator.config.armature_scale=1.0 ++domain_rand.randomize_ctrl_delay=True "++domain_rand.ctrl_delay_step_range=[0,1]"
  "++robot.dof_joint_friction_list=[0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.1,0.1,0.2,0.2,0.2,0.2,0.2,0.1,0.1]"
  ++simulator.config.joint_friction_scale=1.0 ++domain_rand.randomize_joint_friction=True "++domain_rand.joint_friction_scale_range=[0.5,1.5]"
  "++robot.dof_effort_limit_list=[88.0,139.0,88.0,120.0,35.0,35.0,88.0,139.0,88.0,120.0,35.0,35.0,88.0,35.0,35.0,25.0,25.0,25.0,25.0,25.0,5.0,5.0,25.0,25.0,25.0,25.0,25.0,5.0,5.0]"
  ++domain_rand.randomize_friction=True "++domain_rand.friction_range=[0.3,1.5]"
  ++domain_rand.randomize_torso_mass=True "++domain_rand.torso_mass_range=[-2.0,2.0]"
  ++domain_rand.randomize_base_com=True "++domain_rand.base_com_range.x=[-0.03,0.03]"
  "++domain_rand.base_com_range.y=[-0.03,0.03]" "++domain_rand.base_com_range.z=[-0.03,0.03]"
  ++domain_rand.randomize_actuator_gains=True "++domain_rand.actuator_kp_range=[0.9,1.1]" "++domain_rand.actuator_kd_range=[0.9,1.1]"
  ++domain_rand.push_robots=True ++domain_rand.push_isaacsim=True "++domain_rand.push_interval_s=[8,11]" ++domain_rand.max_push_vel_xy=0.5
  ++env.config.substep_monitor=True ++rewards.reward_scales.penalty_arm_torque_saturation=-0.2
  ++simulator.config.arm_body_contact_sensor=True ++rewards.reward_scales.penalty_arm_body_contact=-0.2
  ++robot.asset.usd_file=g1/g1_29dof_rev_1_0_col/g1_29dof_rev_1_0_col.usd
  ++env.config.walk_away_start_m=1.0 ++rewards.reward_scales.walk_away_in_swing=-40.0
  "++env.config.max_stage_time=[${STAGE_TIME:-250,100,100,100,600,200}]" ++env.config.target_root_distance_swing_scale=0.0  # 11-07: STAGE_TIME で替えられる
  ++rewards.reward_scales.penalty_arm_action_change=-0.2 ++rewards.reward_scales.penalty_walk_in_hold=-0.3
  ++algo.config.homie_switch_threshold=0.1 ++env.config.walk_in_hold_threshold=0.1
  "++env.config.loco_cmd_delay_steps_range=[${LEGS_DELAY:-0,15}]" "++env.config.loco_cmd_gain_range=[${LEGS_GAIN:-0.8,1.2}]"  # 11-06: LEGS_DELAY・LEGS_GAIN で替えられる（t2 の鈍い脚）
  ++env.config.complete_requires_through_stage=True ++env.config.termination_curriculum.level_up_ave_goal_reached_rate=0.9
  ++env.config.enable_staged_reset=True
)
CK="${CK:-}"
rm -f "$STOP_FILE"
log "start name=$NAME exp=$EXP n=$N rounds=$ROUNDS round_min=$ROUND_MIN enc=$ENC teacher=$TEACHER ck=${CK:-なし}"
for r in $(seq 0 $((ROUNDS - 1))); do
  [ -e "$STOP_FILE" ] && { log "止め札があるので止める"; break; }
  SEED=$((SEED0 + r))
  OUT="$B/vis_1104/$NAME/r$(printf %03d "$r")_s$SEED"
  mkdir -p "$OUT"
  log "区切り $r seed=$SEED ck=${CK:-なし} 空き待ち"
  # 2 区切り目からは、2 分の空き待ちの代わりにその場で 1 回だけ確かめる（自分の前の区切りが空けたばかり）
  if [ "$r" -gt 0 ] && [ "$(others_gpu_mb)" -le "$OTHERS_GPU_MB" ] && [ "$(mem_avail_mb)" -gt "$NEED_MB" ]; then
    :
  else
    wait_machine_free || { log "空かない"; break; }
  fi
  CK_ARG=()
  [ -n "$CK" ] && CK_ARG=(++checkpoint="$CK")
  nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 2000 > "$OUT/gpu.csv" &
  MP=$!
  setsid systemd-run --user --scope --quiet -p MemoryMax="$MEM_MAX" -p MemorySwapMax=0 -p CPUWeight=20 \
    "${SIGINT_DEFAULT_EXEC[@]}" python gr00t/rl/train_agent_trl.py +exp="$EXP" \
    ++num_envs="$N" ++seed="$SEED" ++teacher_actor_path="$TEACHER" ++door_head_path="$ENC" ++vision_encoder_path="$ENC" \
    ++use_wandb=False ++headless=True ++project_name="vis_$NAME" \
    ++env.config.reset_from_dataset.motion_file_dir="$BASE/LAFAN-G1" \
    "${F3_ARGS[@]}" "${CK_ARG[@]}" ${EXTRA_ARGS:-} \
    > "$OUT/train.log" 2>&1 < /dev/null &
  PID=$!
  START=$(date +%s)
  END=$(( START + ROUND_MIN * 60 ))
  REASON=round_end
  while kill -0 "$PID" 2>/dev/null; do
    [ "$(date +%s)" -ge "$END" ] && break
    grep -aq "Traceback" "$OUT/train.log" && { REASON=traceback; sleep 5; break; }
    if [ $(( $(date +%s) - START )) -ge 600 ] && ! grep -aq "Learning iteration" "$OUT/train.log"; then REASON=startup_hang; break; fi
    [ -e "$STOP_FILE" ] && { REASON=stop_file; break; }
    sleep 10
  done
  # 区切りの時間の前に学習の口が自分で終わった（11-04: PPO の反復の上限 num_total_batches で 1 反復ずつ黙って終わった）
  if [ "$REASON" = round_end ] && ! kill -0 "$PID" 2>/dev/null && [ "$(date +%s)" -lt $(( END - 60 )) ]; then
    REASON=exited_early
  fi
  kill -INT -- -"$PID" 2>/dev/null
  for _ in $(seq 30); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
  kill -KILL -- -"$PID" 2>/dev/null
  kill "$MP" 2>/dev/null
  NEW=$(ls -t "$B/$REPO"/logs_rl/"vis_$NAME"/*/*/last.pt 2>/dev/null | head -1)
  ITERS=$(grep -ac "Learning iteration" "$OUT/train.log")
  log "区切り $r 終わり reason=$REASON 反復 $ITERS gpu_peak=$(awk -F', ' 'BEGIN{m=0}{if($2>m)m=$2}END{print m}' "$OUT/gpu.csv")MiB last=$NEW"
  grep -a "dagger_bc_loss\|obj_pred_loss" "$OUT/train.log" | tail -2
  if [ "$REASON" = exited_early ]; then
    EARLY=$(( ${EARLY:-0} + 1 ))
    [ "$EARLY" -ge 2 ] && { log "区切りが 2 回続けて早く終わったので止める（学習の口のログを見る）"; break; }
  else
    EARLY=0
  fi
  if [ "$REASON" = traceback ] || [ "$REASON" = startup_hang ]; then
    [ "$REASON" = traceback ] && grep -a -A3 "Error" "$OUT/train.log" | tail -8
    [ "$r" = 0 ] && { log "最初の区切りで落ちたので止める"; break; }
  fi
  [ -n "$NEW" ] && [ "$NEW" != "$CK" ] && CK="$NEW"
done
log "終わり ck=$CK"
