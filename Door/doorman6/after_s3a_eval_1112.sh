#!/usr/bin/env bash
# 11-12 17:4x（計画 docs/plan/2026-11-12-g1-door-script-shaped-rl.md の S1 の判定と S3）: s3a は 8〜10 区切り目で 96% で頭打ち。
#   区切りの境目（止め札）で止め、最良の重みで (1) 外乱なし G0 (2) 回り 20° の外乱 Y20（扉 3°・arm3）
#   (3) 3°・6°・10° で教師 f3t3 に引き継いで通り抜け（armthrough・記憶は 0）を、学習に無い扉（種 999）256 回で測る。
#   終わったら s3a を止まった所から起こし直す（すき間の罰の上限 40・扉の値の片寄り 0.03 m＝P0 の結果による見直し）。
# 使い方: setsid nohup bash after_s3a_eval_1112.sh >> logs_s3a_1112.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
log() { echo "=== [$(date "+%m-%d %T")] after_s3a: $*"; }
STOP="$HOME/work_inoue/STOP_DOOR_F3"
F3T3="$HOME/work_inoue/doorman6/VIRAL6_fix/logs_rl/door_f3t3/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261107_131030/last.pt"
OUT="$HOME/work_inoue/doorman/s3_eval_1112.tsv"
BEST_CK="$HOME/work_inoue/doorman/ckpt/start_1112_1603/last.pt"; BEST_N=247
touch "$STOP"
log "止め札を置いた。s2b_1109.sh が区切りの境目で止まるのを待つ"
while pgrep -f "[s]2b_1109.sh" > /dev/null; do sleep 30; done
CK_LAST=$(grep -a -o "chain_done result=stopped_at_s3a last_ckpt=[^ ]*" logs_s3a_1112.log | tail -1 | sed "s/.*last_ckpt=//")
rm -f "$STOP"
line=$(grep -a -E "stage=s3a round=[0-9]+ iter=[0-9]+ success=" logs_s3a_1112.log | tail -1)
n=$(echo "$line" | sed -E "s/.*success=([0-9]+)\/.*/\1/")
rec=$(echo "$line" | sed -E "s/.*record=[^ ]*\/records\/(start_[0-9_]+).*/\1/")
ARM_CK="$BEST_CK"
if [ -n "$n" ] && [ "$n" -gt "$BEST_N" ] && [ -f "ckpt/$rec/last.pt" ]; then ARM_CK="$HOME/work_inoue/doorman/ckpt/$rec/last.pt"; fi
log "止まった（続きの重み ${CK_LAST:-無し}・最後の区切り ${n:-?}/256）。評価は $ARM_CK で"
BASE="++env.config.task.name=door_en1154_g13_shaped_left_weight_realwall ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++seed=999"
NO="++env.config.yaw_disturb.enabled=True ++env.config.yaw_disturb.prob=0.0"
Y20="++env.config.yaw_disturb.enabled=True ++env.config.yaw_disturb.amp_deg=[20,20] ++env.config.yaw_disturb.start_s=[1.4,1.6] ++env.config.yaw_disturb.dur_s=[0.5,0.5] ++env.config.yaw_disturb.sign=lever_side ++env.config.yaw_disturb.prob=1.0"
THROUGH="++env.config.arm_stage.complete_door_deg=null ++env.config.arm_stage.complete_through=True"
[ -f "$OUT" ] || printf "name\trate\tsuccess\trecord\tarm_ck\targs\n" > "$OUT"
source ./gate.sh
ev() {
  local name="$1" crit="$2"; shift 2
  local evlog="logs_s3_eval_1112_${name}.log" rec rate succ
  while ! desk_idle || [ "$(others_gpu_mb)" -gt 4511 ]; do sleep 30; done
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion $crit" MAX_WAIT_MIN=60 MEM_MAX=9G MINUTES=30 EXTRA_ARGS="$BASE $*" ./eval_from_start.sh "$ARM_CK" 256 > "$evlog" 2>&1
  rec=$(grep -o "out=[^ ]*" "$evlog" | tail -1 | cut -d= -f2)
  rate=$("$PY" -c "import json,sys; print(json.load(open(sys.argv[1]))[\"success_rate\"])" "$rec/summary.json" 2>/dev/null || echo NA)
  succ=$("$PY" -c "import json,sys; d=json.load(open(sys.argv[1])); print(str(d[\"success\"]) + \"/\" + str(d[\"episodes_from_stage0\"]))" "$rec/summary.json" 2>/dev/null || echo NA)
  printf "%s\t%s\t%s\t%s\t%s\t%s\n" "$name" "$rate" "$succ" "$rec" "$ARM_CK" "$*" >> "$OUT"
  log "$name success=$succ rate=$rate record=$rec"
}
ev G0 arm3 $NO
ev Y20 arm3 $Y20
for d in 3 6 10; do
  ev "H${d}" armthrough $THROUGH "++env.config.arm_stage.handover_door_deg=$d.0" "++algo.config.eval.handover_policy_path=$F3T3" ++algo.config.eval.handover_hidden=zero
done
log "評価が終わった（s3_eval_1112.tsv）"
if [ -e "$STOP" ]; then log "止め札が置き直されたので s3a は起こさない"; exit 0; fi
if pgrep -f "[s]2b_1109.sh" > /dev/null; then log "s2b_1109.sh が既に動いているので起こさない"; exit 0; fi
[ -n "$CK_LAST" ] && [ -f "$CK_LAST" ] || { log "続きの重みが無い（$CK_LAST）ので s3a は起こさない"; exit 1; }
log "s3a を続きから起こす（$CK_LAST・すき間の罰の上限 40・扉の値の片寄り 0.03 m）"
CK="$CK_LAST" WG_LMAX=40.0 DOOR_NOISE_S3="0.03|1.0" exec ./s3a_1112.sh
