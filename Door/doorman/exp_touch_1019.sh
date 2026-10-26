#!/usr/bin/env bash
# 段 5（10-19）: レバーにゆっくり近づける（勢いよくぶつけて傷めないため。ユーザー判断「触れた瞬間 15 N 以下」）。パッチ 0054。
# 目安: 初めて触れてから 0.1 秒の、手がレバーから受ける力の最大が 15 N 以下を 9 割の回で（touch_metrics.py）。成功率は大きく下げない。
#
#   smoke       試運転の評価（8 台）
#   smoketrain  試運転の学習（6 分。空きの確かめに 2 分かかる）
#   a      A＝実機の制御で腕の目標の変化に上限をかける。学習し直さず、同じ上限をシミュレータに入れて 256 回ずつ測る
#   a2     A の改良＝上限はレバーの近くかつ初めて触れるまでだけ（パッチ 0055）
#   b      B＝ゆっくり近づくことを学習させる。手の速さ・触れた瞬間の力・レバーを下げる速さに減点を足し、45,150 から 30 分 × ROUNDS
#
# どれも学習の扉・立った姿勢・時間 2 倍・関節の目標を可動域の 0.05 rad 内側で切る（実機と同じ）・手のひらの位置を記録（pregrasp_diag）。
# 使い方（OMEN の ~/work_inoue/doorman で）: ./exp_touch_1019.sh smoke a b
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
CK="$(readlink -f "${CK:-ckpt/s25rl_best_it45150_ovr6/last.pt}")"
TASK=door_en1154_g13_shaped_left_weight
EVAL_BASE="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 ++env.config.record_states.pregrasp_diag=True"
B_REWARDS="${B_REWARDS:-++rewards.reward_scales.palm_speed_near_lever=-10.0,++rewards.reward_scales.lever_impact=-0.5,++rewards.reward_scales.lever_press_speed=-2.0}"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

# 評価して当たり方を数える（引数: 名前・台数・足す引数...）
measure() {
  local name="$1" n="$2"; shift 2
  local evlog="touch_logs/eval_${name}.log"
  mkdir -p touch_logs
  COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_BASE $*" ./eval_from_start.sh "$CK" "$n" > "$evlog" 2>&1
  local out
  out=$(grep -ao 'eval_from_start out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  if [ -z "$out" ]; then log "[$name] 評価が終わらなかった（$evlog）"; return 1; fi
  log "[$name] $(grep -ao 'success=[^ ]* = [0-9.]*%' "$evlog" | tail -1) out=$out"
  "$PY" touch_metrics.py "$out" --json "$out/touch.json" 2>&1 | sed "s/^/  [$name] /"
}

for MODE in "$@"; do
  case "$MODE" in
    smoke)
      log "smoke: 評価（近くだけ上限）"
      measure smoke_near 8 ++env.config.arm_target_rate_limit=0.5 ++env.config.arm_target_rate_limit_near_m=0.10
      ;;
    smoketrain)
      log "smoke: 6 分の学習（B の報酬。空きの確かめに 2 分かかる）"
      EXP=wbmanip/door_open_homie_29dof_state_ppo-lstm EVAL_EXTRA="$EVAL_BASE" MEM_MAX=9G NEED_MARGIN_MB=1024 PASS_RATE=1.1 EVAL_N=16 \
        PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE25" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_25" \
        ./stage_chain.sh "$CK" "s5touchsmoke:$TASK:1:6:through:++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12,++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90,++env.config.mirror_left_doors=True,++env.config.push_door_hinge_max_deg=80,++env.config.door_overopen_start_deg=85,++rewards.reward_scales.door_overopen=-6.0,++env.config.clip_targets_to_limits_margin=0.05,$B_REWARDS" \
        2>&1 | grep -aE "===|Traceback|Error" | tail -8
      ;;
    a)
      measure base 256
      for R in 3.0 1.5 0.75; do measure "all_${R}" 256 ++env.config.arm_target_rate_limit="$R"; done
      measure near10_0.5 256 ++env.config.arm_target_rate_limit=0.5 ++env.config.arm_target_rate_limit_near_m=0.10
      measure near15_0.3 256 ++env.config.arm_target_rate_limit=0.3 ++env.config.arm_target_rate_limit_near_m=0.15
      ;;
    a2)
      # A の改良（10-19）: 上限は「レバーの近く」かつ「初めて触れるまで」だけ（パッチ 0055）。距離だけだと押している間も遅くなり成功 20% に落ちた
      measure until15_0.3 256 ++env.config.arm_target_rate_limit=0.3 ++env.config.arm_target_rate_limit_near_m=0.15 ++env.config.arm_target_rate_limit_until_touch=True
      measure until15_0.5 256 ++env.config.arm_target_rate_limit=0.5 ++env.config.arm_target_rate_limit_near_m=0.15 ++env.config.arm_target_rate_limit_until_touch=True
      measure until25_0.3 256 ++env.config.arm_target_rate_limit=0.3 ++env.config.arm_target_rate_limit_near_m=0.25 ++env.config.arm_target_rate_limit_until_touch=True
      ;;
    b)
      TRAIN="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12,++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90,++env.config.mirror_left_doors=True,++env.config.push_door_hinge_max_deg=80,++env.config.door_overopen_start_deg=85,++rewards.reward_scales.door_overopen=-6.0,++env.config.clip_targets_to_limits_margin=0.05,$B_REWARDS${B_ADD:+,$B_ADD}"
      log "B: 学習 ${ROUNDS:-3} 回（30 分ずつ）rewards=$B_REWARDS add=${B_ADD:-なし}"
      EXP=wbmanip/door_open_homie_29dof_state_ppo-lstm EVAL_EXTRA="$EVAL_BASE" MEM_MAX=9G NEED_MARGIN_MB=1024 PASS_RATE=1.1 EVAL_N=256 \
        PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE25" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_25" \
        ./stage_chain.sh "$CK" "${B_NAME:-s5touch}:$TASK:${ROUNDS:-3}:30:through:$TRAIN" > "touch_logs/chain_${B_NAME:-s5touch}.log" 2>&1
      L=$(ls -td chain/*/ | head -1)
      log "B: chain=$L"
      while IFS=$'\t' read -r stage round iter succ rate ck rec; do
        [ -d "$rec" ] || continue
        log "[B 回 $round・反復 $iter] 成功 $succ（$rate）"
        "$PY" touch_metrics.py "$rec" --json "$rec/touch.json" 2>&1 | sed "s/^/  [B 回 $round] /"
      done < "$L/results.tsv"
      ;;
  esac
done
log "exp_touch_1019 done"
