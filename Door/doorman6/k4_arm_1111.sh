#!/usr/bin/env bash
# 11-11（計画 docs/plan/2026-11-11-g1-door-stage2-fix.md の K4）: 腕の先行と速さを抑えると成功がどれだけ落ちるかを、今の s2f 最良
#   （start_1110_1703）で見る（学習なし・外乱なし）。実機で体の回りが減るかは機体の K4r で見る。
#   L10・L15・L20: 腕の目標の先行の守り arm_accum_guard_delta を全関節 0.10・0.15・0.20 rad に（s2f は関節ごとに 0.2〜0.3。実機の段 2 は
#     ほぼずっと 0.3 に張り付いていた）  R10・R15・R20: 腕の目標の速さの上限 arm_target_rate_limit を 1.0・1.5・2.0 rad/s に（s2f は無し）
#   どれも学習に無い扉（種 999）・256 回・成功は扉 3°。基準は K2 の Y0（217/256）。解析は ~/work_inoue/ana_1111/k4_<名前>.json
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup bash k4_arm_1111.sh >> logs_k4_arm_1111.log 2>&1 < /dev/null &
#   K2・K6a の台本が走っている間は待つ
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
log() { echo "=== [$(date "+%m-%d %T")] k4: $*"; }
CK="${CK:-$HOME/work_inoue/doorman/ckpt/start_1110_1703/last.pt}"
OUT="$HOME/work_inoue/doorman/k2_yaw_1111.tsv"  # K2 と同じ表に足す
ANA="$HOME/work_inoue/ana_1111"
NAMES="${NAMES:-L10 L15 L20 R10 R15 R20}"
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++seed=999"
[ -f "$CK" ] || { log "重みが無い: $CK"; exit 1; }
source ./gate.sh

while pgrep -f "[k]2_yaw_1111.sh|[k]6a_wall_1111.sh" > /dev/null; do sleep 30; done
ev() {
  local name="$1"; shift
  local evlog="logs_k4_arm_1111_${name}.log" rec rate succ
  while ! desk_idle || [ "$(others_gpu_mb)" -gt 4511 ]; do sleep 30; done  # 机で使われている間・他人が GPU を使っている間は始めない
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=60 MEM_MAX=9G \
    EXTRA_ARGS="$BASE_ARGS $*" ./eval_from_start.sh "$CK" 256 > "$evlog" 2>&1
  rec=$(grep -o 'out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  rate=$("$PY" -c 'import json,sys; print(json.load(open(sys.argv[1]))["success_rate"])' "$rec/summary.json" 2>/dev/null || echo NA)
  succ=$("$PY" -c 'import json,sys; d=json.load(open(sys.argv[1])); print(str(d["success"]) + "/" + str(d["episodes_from_stage0"]))' "$rec/summary.json" 2>/dev/null || echo NA)
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$name" "$CK" "$rate" "$succ" "$rec" "$*" >> "$OUT"
  log "$name success=$succ rate=$rate record=$rec"
  (cd "$ANA" && "$PY" ana_stage2_sim_1111.py "$rec" --urdf g1_29dof_rev_1_0.urdf --json "k4_${name}.json" > "k4_${name}.txt" 2>&1) \
    || log "$name: 解析が落ちた（$ANA/k4_${name}.txt）"
}

log "始め ck=$CK names=$NAMES"
for n in $NAMES; do
  case "$n" in
    L*) v="0.${n#L}"; ev "$n" "++env.config.arm_accum_guard_delta=$v" ;;
    R*) v=$(awk -v x="${n#R}" 'BEGIN{printf "%.1f", x/10}'); ev "$n" "++env.config.arm_target_rate_limit=$v" ;;
    *) log "知らない名前 $n" ;;
  esac
done
log "終わり"
