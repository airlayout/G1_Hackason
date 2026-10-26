#!/usr/bin/env bash
# 段 5（10-19 夜）: レバーにやさしく触れる — 文献の ①（引っ張る力に上限）と ②（罰の重みを自動で上げる）を、直した計器で学習する。
# 計器（0057）: 力は物理 1 回ごとの最大。元の方策（45,150）は当たりの山が中央 40 N・15 N 以下 2.4%。
# 当たりの山 ≒ 27 N ×（直前の速さ m/s）＋ 23 N。速さが 0 でも約 23 N で当たる＝勢いと押し付けの 2 つの成分がある（見込み）
#
#   smoke  6 分の学習（①＋② を両方入れて、式が学習の中で動くか・λ の記録が書かれるか）
#   cap    ①: 掛け金が外れる前は前へ押す報酬を 15 N で頭打ち・下へ押す報酬は 15 N で最大で 25 N 超で減らす・掛け金が外れる前の 25 N 超に固定の罰
#   lag    ②: 当たりの山が 15 N を超えた回の割合を 1 割以下に。超えている間は罰の重み λ が上がる（罰は近づく速さの超過＋当たりの山の超過）
#   both   ①＋②
#
# どれも 45,150 から 30 分 × ROUNDS（既定 3）。回ごとに 256 回評価し、当たり方を数える（touch_metrics.py。力は物理 1 回ごとの最大）。
# 評価は学習の扉・立った姿勢・時間 2 倍・関節の目標を可動域の 0.05 rad 内側で切る（実機と同じ）。
# 使い方（OMEN の ~/work_inoue/doorman で）: ./exp_touch3_1019.sh smoke cap lag both
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
CK="$(readlink -f "${CK:-ckpt/s25rl_best_it45150_ovr6/last.pt}")"
TASK=door_en1154_g13_shaped_left_weight
EXP=wbmanip/door_open_homie_29dof_state_ppo-lstm
EVAL_BASE="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 ++env.config.record_states.pregrasp_diag=True ++env.config.lever_force_substeps=True ++env.config.lever_lagrange.update=False"
# 45,150 の学習と同じ設定（exp_touch_1019.sh の b と同じ）＋ 物理 1 回ごとの力
TRAIN_BASE="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12,++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90,++env.config.mirror_left_doors=True,++env.config.push_door_hinge_max_deg=80,++env.config.door_overopen_start_deg=85,++rewards.reward_scales.door_overopen=-6.0,++env.config.clip_targets_to_limits_margin=0.05,++env.config.lever_force_substeps=True"
# ①: 罰の重み −75 は「1 N 超えるごとに −5」（B の 10 倍の lever_impact と同じ強さ）
CAP_ARGS="++env.config.push_door_force_latched_max=15,++env.config.press_handle_force_peak_n=15,++env.config.press_handle_force_over_n=25,++env.config.lever_force_cap_n=15,++env.config.lever_force_cap_margin_n=10,++rewards.reward_scales.lever_force_over_cap=${CAP_W:--75.0}"
# ②: λ の初め 100 は、速さの罰で B の 10 倍（−100）・当たりの罰で 1 N ごとに −6.7（B の 10 倍は −5）と同じ強さ
lag_args() {
  echo "++rewards.reward_scales.lever_touch_lagrange=1.0,++env.config.lever_lagrange.lambda_init=${LAG_INIT:-100.0},++env.config.lever_lagrange.rate_init=1.0,++env.config.lever_lagrange.lr=${LAG_LR:-0.02},++env.config.lever_lagrange.target_rate=0.1,++env.config.lever_lagrange.rate_ema=0.002,++env.config.lever_lagrange.lambda_max=2000.0,++env.config.lever_lagrange.impact_max_n=15.0,++env.config.lever_lagrange.impact_steps=2,++env.config.lever_lagrange.v_touch=0.2,++env.config.lever_lagrange.slope=2.0,++env.config.lever_lagrange.contact_dist_m=0.07,++env.config.lever_lagrange.zone_m=0.30,++env.config.lever_lagrange.update=True,++env.config.lever_lagrange.state_path=$HOME/work_inoue/doorman/touch_logs/lambda_$1.json"
}
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

# 学習の連鎖（引数: 名前・回数・分・足す学習の引数）。回ごとに当たり方を数える
chain() {
  local name="$1" rounds="$2" minutes="$3" extra="$4"
  mkdir -p touch_logs
  log "[$name] 学習 $rounds 回（${minutes} 分ずつ）extra=$extra"
  EXP=$EXP EVAL_EXTRA="$EVAL_BASE" MEM_MAX=9G NEED_MARGIN_MB=1024 PASS_RATE=1.1 EVAL_N="${EVAL_N:-256}" \
    PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE25" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_25" \
    ./stage_chain.sh "$CK" "$name:$TASK:$rounds:$minutes:through:$TRAIN_BASE,$extra" > "touch_logs/chain_${name}.log" 2>&1
  local L
  L=$(grep -ao 'chain/[0-9_]*' "touch_logs/chain_${name}.log" | head -1)
  [ -n "$L" ] || L=$(ls -td chain/*/ | head -1)
  log "[$name] chain=$L"
  grep -aE "Traceback|Error" "touch_logs/chain_${name}.log" | grep -v JSON | head -5
  [ -f "$L/results.tsv" ] || { log "[$name] results.tsv が無い"; return 1; }
  tail -n +2 "$L/results.tsv" | while IFS=$'\t' read -r stage task round iter succ rate ck rec; do
    log "[$name 回 $round・反復 $iter] 成功 $succ（$rate）"
    [ -d "$rec" ] && "$PY" touch_metrics.py "$rec" --force peak --json "$rec/touch_peak.json" 2>&1 | sed "s/^/  [$name 回 $round] /"
  done
  [ -f "touch_logs/lambda_${name}.json" ] && log "[$name] λ の最後: $(cat "touch_logs/lambda_${name}.json")"
}

for MODE in "$@"; do
  case "$MODE" in
    smoke)
      rm -f touch_logs/lambda_s5smoke3.json touch_logs/lambda_s5smoke3.json.log
      EVAL_N=16 chain s5smoke3 1 6 "$CAP_ARGS,$(lag_args s5smoke3)"
      tail -3 touch_logs/lambda_s5smoke3.json.log 2>/dev/null | sed 's/^/  [λ] /'
      ;;
    cap) chain s5cap 3 30 "$CAP_ARGS" ;;
    lag)
      rm -f touch_logs/lambda_s5lag.json touch_logs/lambda_s5lag.json.log
      chain s5lag 3 30 "$(lag_args s5lag)"
      ;;
    both)
      rm -f touch_logs/lambda_s5both.json touch_logs/lambda_s5both.json.log
      chain s5both 3 30 "$CAP_ARGS,$(lag_args s5both)"
      ;;
    # 10-19 深夜: ② の続き。② の 3 回目（46,500・λ 760）から 30 分 × 3 回。失敗は区間 1（構える）の時間切れが 3→31 回に増えていた。
    # 構える点は把持目標から約 18 cm で、速さの罰の範囲（30 cm）の中にある。A はそのまま続ける、B は速さの罰を構えた後（区間 2 以降）だけにする
    lagA|lagB)
      cp touch_logs/lambda_s5lag_final_1019.json "touch_logs/lambda_s5${MODE}.json"
      rm -f "touch_logs/lambda_s5${MODE}.json.log"
      add=""
      [ "$MODE" = lagB ] && add=",++env.config.lever_lagrange.speed_from_stage=2"
      CK="$(readlink -f ckpt/s5lag_r3_it46500/last.pt)" chain "s5$MODE" 3 30 "$(lag_args "s5$MODE")$add"
      ;;
    *) log "知らないモード: $MODE" ;;
  esac
done
log "exp_touch3_1019 done"
