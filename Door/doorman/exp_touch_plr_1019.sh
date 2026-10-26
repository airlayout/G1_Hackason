#!/usr/bin/env bash
# 段 5（10-19）: ② ＋ B。生徒に「手のひらからレバーまでの位置の差」（palm_lever_rel、頭のカメラで測る前提。パッチ 0056）を見せ、
# レバーにゆっくり近づく減点（B、パッチ 0054）で学習する。② なしの B（減点 10 倍）は 3 回とも当たり方が変わらなかった
# （レバーまで あと何 cm かが分からない＝枠の位置の片寄り σ3 cm・巻き尺 σ1 cm の推し量りでは、触れる直前だけ優しくできない）。
# 出発は 45,150 に新しい 3 列を重み 0 で差し込んだ重み（add_obs_term_ckpt.py。動きは 45,150 と同じ）。
# 誤差は仮の値（片寄り 0.5 cm・揺れ 0.5 cm・10 回毎秒・0.1 秒遅れ）。PLR_ARGS で替える
#   smoke  8 台の評価（新しい観測が動くか。重み 0 なので 45,150 と同じ振る舞いのはず）
#   train  30 分 × ROUNDS（既定 3）。回ごとに当たり方を数える
# 使い方（OMEN の ~/work_inoue/doorman で）: ./exp_touch_plr_1019.sh smoke train
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
CK="$(readlink -f "${CK:-ckpt/s5plr_start/last.pt}")"
TASK=door_en1154_g13_shaped_left_weight
EXP_PLR=wbmanip/door_open_homie_29dof_state_ppo_plr-lstm
PLR_ARGS="${PLR_ARGS:-}"
EVAL_BASE="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 ++env.config.record_states.pregrasp_diag=True"
B_REWARDS="${B_REWARDS:-++rewards.reward_scales.palm_speed_near_lever=-100.0,++rewards.reward_scales.lever_impact=-5.0,++rewards.reward_scales.lever_press_speed=-20.0}"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

for MODE in "$@"; do
  case "$MODE" in
    smoke)
      # 出発の重みの隣に評価用の config.yaml（_plr の exp）が要る。学習を 1 回回すまでは無いので、45,150 の config に観測を足したものを作る
      D=$(dirname "$CK")
      if [ ! -f "$D/config.yaml" ]; then
        "$PY" - "$D" <<'PYEOF'
import sys, yaml
src = yaml.safe_load(open("ckpt/s25rl_best_it45150_ovr6/config.yaml"))
obs = yaml.safe_load(open("../doorman6/VIRAL6/gr00t/rl/config/obs/wbmanip/door_open_homie_29dof_state_ppo_plr.yaml"))["obs"]
src["obs"] = obs
yaml.safe_dump(src, open(sys.argv[1] + "/config.yaml", "w"), allow_unicode=True, sort_keys=False)
print("config.yaml を作った（観測だけ _plr）")
PYEOF
      fi
      mkdir -p touch_logs
      COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_BASE $PLR_ARGS" ./eval_from_start.sh "$CK" 8 > touch_logs/eval_plr_smoke.log 2>&1
      out=$(grep -ao 'eval_from_start out=[^ ]*' touch_logs/eval_plr_smoke.log | tail -1 | cut -d= -f2)
      log "[plr smoke] $(grep -ao 'success=[^ ]* = [0-9.]*%' touch_logs/eval_plr_smoke.log | tail -1) out=$out"
      grep -aE "Traceback|Error" touch_logs/eval_plr_smoke.log | grep -v JSON | head -5
      [ -n "$out" ] && "$PY" touch_metrics.py "$out" 2>&1 | sed "s/^/  [plr smoke] /"
      ;;
    train)
      TRAIN="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12,++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90,++env.config.mirror_left_doors=True,++env.config.push_door_hinge_max_deg=80,++env.config.door_overopen_start_deg=85,++rewards.reward_scales.door_overopen=-6.0,++env.config.clip_targets_to_limits_margin=0.05,$B_REWARDS${PLR_ARGS:+,${PLR_ARGS// /,}}"
      log "② ＋ B: 学習 ${ROUNDS:-3} 回（30 分ずつ）rewards=$B_REWARDS plr=${PLR_ARGS:-既定}"
      EXP=$EXP_PLR EVAL_EXTRA="$EVAL_BASE $PLR_ARGS" MEM_MAX=9G NEED_MARGIN_MB=1024 PASS_RATE=1.1 EVAL_N=256 \
        PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE25" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_25" \
        ./stage_chain.sh "$CK" "${NAME:-s5plr10}:$TASK:${ROUNDS:-3}:30:through:$TRAIN" > "touch_logs/chain_${NAME:-s5plr10}.log" 2>&1
      L=$(ls -td chain/*/ | head -1)
      log "② ＋ B: chain=$L"
      tail -n +2 "$L/results.tsv" | while IFS=$'\t' read -r stage task round iter succ rate ck rec; do
        log "[② ＋ B 回 $round・反復 $iter] 成功 $succ（$rate）"
        [ -d "$rec" ] && "$PY" touch_metrics.py "$rec" --json "$rec/touch.json" 2>&1 | sed "s/^/  [② ＋ B 回 $round] /"
      done
      ;;
  esac
done
log "exp_touch_plr_1019 done"
