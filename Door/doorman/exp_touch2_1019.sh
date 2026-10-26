#!/usr/bin/env bash
# 段 5（10-19 夜）: レバーへの当たり方を、計器を直して測り直す（パッチ 0057: 物理 1 回ごとに力を読み、制御の 1 ステップの最大と平均を記録）。
# 10-19 昼の「触れた瞬間の力」は、物理 4 回のうち 4 回目の後の写しだった。近づく速さとの相関が 0.04 で、ぶつかる強さを表していない疑いがある。
#
#   smoke  8 台の評価（新しい記録の項目が出るか）
#   base   45,150 を 256 回（元の方策の当たり方を、直した計器で）
#
# どれも学習の扉・立った姿勢・時間 2 倍・関節の目標を可動域の 0.05 rad 内側で切る（実機と同じ）・手のひらの位置を記録（pregrasp_diag）。
# 使い方（OMEN の ~/work_inoue/doorman で）: ./exp_touch2_1019.sh smoke base
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
CK="$(readlink -f "${CK:-ckpt/s25rl_best_it45150_ovr6/last.pt}")"
TASK=door_en1154_g13_shaped_left_weight
EVAL_BASE="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 ++env.config.record_states.pregrasp_diag=True ++env.config.lever_force_substeps=True"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

# 評価して当たり方を数える（引数: 名前・台数・足す引数...）
measure() {
  local name="$1" n="$2"; shift 2
  local evlog="touch_logs/eval_${name}.log"
  mkdir -p touch_logs
  COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_BASE $*" ./eval_from_start.sh "$CK" "$n" > "$evlog" 2>&1
  local out
  out=$(grep -ao 'eval_from_start out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  if [ -z "$out" ]; then log "[$name] 評価が終わらなかった（$evlog）"; grep -aE "Traceback|Error" "$evlog" | grep -v JSON | head -5; return 1; fi
  log "[$name] $(grep -ao 'success=[^ ]* = [0-9.]*%' "$evlog" | tail -1) out=$out"
  for how in peak snapshot; do
    "$PY" touch_metrics.py "$out" --force "$how" --json "$out/touch_${how}.json" 2>&1 | sed "s/^/  [$name $how] /"
  done
}

for MODE in "$@"; do
  case "$MODE" in
    smoke) measure substep_smoke 8 ;;
    base) measure substep_base 256 ;;
    *) log "知らないモード: $MODE" ;;
  esac
done
log "exp_touch2_1019 done"
