#!/usr/bin/env bash
# 段 2-4b-3（2026-10-14、2-4b の続き）: 扉の重さを 15〜120 kg に広げる（door_en1154_g13_shaped_left_weight＝_lever ＋重さ）。
# 計画は 15〜100 kg だが上限 120 を残す（b1 でバックセットを狭めたら、外した範囲の扉で成功が落ちた）。教師は重さを数値で見ている。
# 出発は b1・b2 の合格（反復 42,050）。学習の引数・評価の条件・段の回し方は exp_s24b_1014.sh と同じ
# （学習なしの評価 → 80% 未満なら 30 分 × 最大 4 回。評価は 256 回・立った姿勢・時間 2 倍）。
# 最後に、合格した重みを b2 の扉（重さは既定の 80〜120 kg）でも評価する（重さを広げたせいで元の扉が落ちていないか）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_s24b3_1014.sh > exp_s24b3_1014.log 2>&1 < /dev/null &
#   在席の判定を切るとき（OMEN の前の人がユーザー本人のときだけ）: IDLE_GATE=0 CK=<重み> setsid nohup ./exp_s24b3_1014.sh > ... &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
# 出発の重みは CK で替えられる（10-15: 10-14 19:56 に在席の判定で学習が 10 分で止まったので、反復 42,200 から IDLE_GATE=0 で回し直した。ユーザー本人が OMEN の前にいた）
CK="${CK:-$HOME/work_inoue/doorman/ckpt/s24b2_pass_it42050/last.pt}"
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
RULES="++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90"
MIRROR="++env.config.mirror_left_doors=True"
WINDOW="++env.config.push_door_hinge_max_deg=80,++env.config.door_overopen_start_deg=85,++rewards.reward_scales.door_overopen=-3.0"
TRAIN="$CAP3,$RULES,$MIRROR,$WINDOW"
export EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
export MINUTES=30 MEM_MAX=9G NEED_MARGIN_MB=1024 PASS_RATE=0.8 EVAL_N=256 EVAL_FIRST=1
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE2B" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_B"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

log "exp_s24b3 start ckpt=$CK"
./stage_chain.sh "$CK" "s24b3:door_en1154_g13_shaped_left_weight:4:30:through:$TRAIN"
L=$(ls -td chain/*/ | head -1)
log "chain done chain=$L"
cut -f1-6 "$L/results.tsv" | sed "s/^/  [chain] /"

KEPT=$(grep -o 'kept=[^ ]*' "$L/chain.log" | tail -1 | cut -d= -f2)
if [ -n "$KEPT" ] && [ -f "$KEPT/last.pt" ]; then
  log "b2 の扉で評価し直す ckpt=$KEPT/last.pt"
  COUNT_ARGS="--criterion through" EXTRA_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_lever $EVAL_EXTRA" \
    ./eval_from_start.sh "$KEPT/last.pt" 256 > "$L/eval_b2_door_regression.log" 2>&1
  grep -a "eval_from_start out=" "$L/eval_b2_door_regression.log" | sed "s/^/  [b2 の扉] /"
else
  log "合格した重みが無いので b2 の扉での評価は飛ばす"
fi
log "exp_s24b3_done"
