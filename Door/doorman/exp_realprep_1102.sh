#!/usr/bin/env bash
# 11-02 計画 docs/plan/2026-11-02-g1-door-real-stock-legs.md の手順 1〜3 のうち OMEN で測るもの（機体なし）。
#   手順 1: 線を 20 rad/s に固定した候補 3 本（ckpt/lvl1.0_r4・r8・r10。seed=0 の 256 回は exp_level_1102 で測り済み）を
#           別の扉の組（seed=1）の 256 回でも測り、合わせた 512 回の成功率で 1 本を選ぶ
#   手順 2: 選んだ重みを、生徒の脚と腰の観測を隠して測る（パッチ 0086 の prob=1.0、seed=0。学習はしない）
#   手順 3: 選んだ重みで正解の記録を 8 台取る（観測の雑音 0・脚の遅れ 0・倍率 1）。Mac の check_golden.py で door_core と突き合わせる
# 測る間は学習を札で止める（KEEP_PAUSE=1 で置き、全部済むか落ちたら外す）。連鎖の学習は札が外れると続きから再開する
# 済んだ行は realprep_1102.tsv を見て飛ばす（停電の後に始め直せる）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid -f nohup ./exp_realprep_1102.sh >> logs_realprep_1102.log 2>&1 < /dev/null
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
# 札は自分の PAUSE_DOOR_F3（連鎖の学習もこれを見る）。gate.sh の既定 PAUSE_DOORMAN_TRAINING はユーザーの札と同じ名前なので使わない
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
PY="$HOME/miniforge3/envs/doorman6/bin/python"
TASK=door_en1154_g13_shaped_left_weight
EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
NOISE0="++obs.noise_scales.dof_pos=0.0 ++obs.noise_scales.dof_vel=0.0 ++obs.noise_scales.arm_dof_torque=0.0 ++obs.noise_scales.e_homie_dof_pos=0.0 ++obs.noise_scales.f_homie_dof_vel=0.0"
# 脚の遅れと倍率を切る（入れたままだと、シミュレータの HOMIE が遅れた指令を見て door_core と一致しない）
LEGS0="++env.config.loco_cmd_delay_steps_range=[0,0] ++env.config.loco_cmd_gain_range=[1.0,1.0]"
OUT_TSV="$HOME/work_inoue/doorman/realprep_1102.tsv"
GOLDEN="$HOME/work_inoue/doorman/records/golden_realprep_1102.npz"
CANDS="r4 r8 r10"
SEED0_REC="r4:start_1102_1510 r8:start_1102_1514 r10:start_1102_1519"   # exp_level_1102 の線 1.0・seed=0 の記録
[ -f "$OUT_TSV" ] || printf 'step\tname\tseed\tsuccess\trate\trecord\n' > "$OUT_TSV"
trap './pause_training.sh off >/dev/null; log "札を外した"' EXIT

summary_cols() {  # summary.json → "成功/回数<TAB>割合"
  "$PY" -c "import json,sys; d=json.load(open(sys.argv[1])); print(f\"{d['success']}/{d['episodes_from_stage0']}\t{d['success_rate']}\")" "$1/summary.json" 2>/dev/null \
    || printf 'none\tnone'
}

is_done() {  # 引数: 手順・名前・seed
  awk -F'\t' -v s="$1" -v n="$2" -v d="$3" '$1 == s && $2 == n && $3 == d && $4 != "none" { f = 1 } END { exit !f }' "$OUT_TSV"
}

# 引数: 手順・名前・重み・seed・台数・足す引数
run() {
  local step="$1" name="$2" ck="$3" seed="$4" n="$5" extra="$6" lg out
  is_done "$step" "$name" "$seed" && { log "済み: $step $name seed=$seed"; return 0; }
  lg="logs_realprep_1102_${step}_${name}_s${seed}.log"
  log "測る: $step $name seed=$seed n=$n"
  KEEP_PAUSE=1 IDLE_GATE=0 MAX_WAIT_MIN=60 EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion through" \
    EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA ++seed=$seed $extra" ./eval_from_start.sh "$ck" "$n" > "$lg" 2>&1
  out=$(grep -o 'out=[^ ]*' "$lg" | tail -1 | cut -d= -f2)
  printf '%s\t%s\t%s\t%s\t%s\n' "$step" "$name" "$seed" "$(summary_cols "$out")" "$out" >> "$OUT_TSV"
  log "$step $name seed=$seed: $(summary_cols "$out") out=$out"
  [ -n "$out" ] && [ -f "$out/static.npz" ] && nice "$PY" bygrade.py "$out"
}

# 手順 1: seed=0 は測り済みの記録を表に写し、seed=1 を測る
for pair in $SEED0_REC; do
  name="${pair%%:*}"; rec="records/${pair#*:}"
  is_done step1 "$name" 0 || printf 'step1\t%s\t0\t%s\t%s\n' "$name" "$(summary_cols "$rec")" "$HOME/work_inoue/doorman/$rec" >> "$OUT_TSV"
done
for name in $CANDS; do
  run step1 "$name" "$HOME/work_inoue/doorman/ckpt/lvl1.0_$name/last.pt" 1 256 ""
done
best=$(awk -F'\t' '$1 == "step1" && $4 != "none" { split($4, a, "/"); s[$2] += a[1]; n[$2] += a[2] }
  END { for (k in s) if (n[k] >= 512) printf "%s %.4f\n", k, s[k] / n[k] }' "$OUT_TSV" | sort -k2 -nr | head -1 | cut -d' ' -f1)
[ -n "$best" ] || { log "手順 1 の 512 回がそろわない（表を確かめる）"; exit 1; }
log "手順 1 で選んだ重み: $best（ckpt/lvl1.0_$best）"
BEST_CK="$HOME/work_inoue/doorman/ckpt/lvl1.0_$best/last.pt"

# 手順 2: 脚と腰の観測を隠す（学習はしない）
run step2 "mask_$best" "$BEST_CK" 0 256 "++env.config.actor_obs_mask_lower_body_prob=1.0"

# 手順 3: 正解の記録（8 台）
run step3 "golden_$best" "$BEST_CK" 0 8 "$NOISE0 $LEGS0 ++algo.config.eval.golden_trace_path=$GOLDEN"
log "realprep_1102 が終わった（選んだ重み $best・正解の記録 $GOLDEN）"
