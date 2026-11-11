#!/usr/bin/env bash
# 11-10（案 A・計画書 docs/plan/2026-11-09-g1-door-staged-rgbd.md の手順 5。ユーザー「引き継ぎは f3t3（2 つの方策）」）:
#   腕だけの段の方策で扉を 3° 開け、そこから教師 f3t3 に引き継いで通り抜けられるかをシムで測る（パッチ 0101）。学習はしない。
#   H0: 引き継がない（腕の方策のまま通り抜けまで）  H1: 3° で f3t3 に・記憶は回の始めから影で回す  H2: 3° で f3t3 に・記憶は替える歩で 0
#   どれも学習に無い扉（種 999）・256 回・成功は段 1 から始めて扉の先 1.5 m（count_success.py --criterion armthrough）。
#   腕の方策の条件（始めの範囲・引き金・扉の値の雑音）は重みの隣の config.yaml のまま（引き継ぎだけを替える）
#   DEGS="3 6 10"（11-10 ユーザー「引き継ぐ角度をずらす確かめ（6°・10°）を s2f の測り直しと一緒に」）: 引き継ぐ角度を並べて回す。
#     このときの記憶は HIDDEN（既定 zero。1 回目で shadow と差が無かった）。DEGS が空なら 1 回目と同じ H0・H1（3°・shadow）・H2（3°・zero）
# 使い方（OMEN の ~/work_inoue/doorman）: ARM_CK=<腕の方策の last.pt> setsid nohup bash handover_eval_1110.sh >> logs_handover_1110.log 2>&1 < /dev/null &
#   GPU を使うので、学習を止めてから回す（止め札 ~/work_inoue/STOP_DOOR_F3 で区切りの境目に止まる）
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
log() { echo "=== [$(date "+%m-%d %T")] handover: $*"; }
ARM_CK="${ARM_CK:-$HOME/work_inoue/doorman/ckpt/start_1110_0148/last.pt}"
F3T3="${F3T3:-$HOME/work_inoue/doorman6/VIRAL6_fix/logs_rl/door_f3t3/wbmanip/door_open_homie_29dof_state_ppo-lstm_lstm-20261107_131030/last.pt}"
TAG="${TAG:-$(date +%m%d_%H%M)}"
OUT="$HOME/work_inoue/doorman/handover_eval_1110.tsv"
BASE="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++seed=999"
THROUGH="++env.config.arm_stage.complete_door_deg=null ++env.config.arm_stage.complete_through=True"
DEGS="${DEGS:-}"
HIDDEN="${HIDDEN:-zero}"
[ -f "$ARM_CK" ] && [ -f "$F3T3" ] || { log "重みが無い: $ARM_CK / $F3T3"; exit 1; }
grep -qF "_load_handover_policy" "$HOME/work_inoue/doorman6/VIRAL6_fix/gr00t/rl/trl/trainer/ppo_trainer_homie_api.py" || { log "パッチ 0101 が当たっていない"; exit 1; }
grep -qF '"armthrough"' count_success.py || { log "count_success.py に armthrough が無い"; exit 1; }
[ -f "$OUT" ] || printf 'tag\tname\trate\tsuccess\trecord\tarm_ck\n' > "$OUT"
ev() {
  local name="$1"; shift
  local evlog="logs_handover_1110_${TAG}_${name}.log" rec rate succ
  local deg="${HDEG:-3.0}"
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion armthrough" MAX_WAIT_MIN=60 MEM_MAX=9G MINUTES=30 \
    EXTRA_ARGS="$BASE $THROUGH ++env.config.arm_stage.handover_door_deg=$deg $*" ./eval_from_start.sh "$ARM_CK" 256 > "$evlog" 2>&1
  rec=$(grep -o 'out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  rate=$("$PY" -c 'import json,sys; print(json.load(open(sys.argv[1]))["success_rate"])' "$rec/summary.json" 2>/dev/null || echo NA)
  succ=$("$PY" -c 'import json,sys; d=json.load(open(sys.argv[1])); print(str(d["success"]) + "/" + str(d["episodes_from_stage0"]))' "$rec/summary.json" 2>/dev/null || echo NA)
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$TAG" "$name" "$rate" "$succ" "$rec" "$ARM_CK" >> "$OUT"
  log "$name success=$succ rate=$rate record=$rec"
}
log "始め arm=$ARM_CK f3t3=$F3T3 degs=${DEGS:-（1 回目の 3 本）} hidden=$HIDDEN"
ev H0_none
if [ -z "$DEGS" ]; then
  ev H1_shadow "++algo.config.eval.handover_policy_path=$F3T3" ++algo.config.eval.handover_hidden=shadow
  ev H2_zero "++algo.config.eval.handover_policy_path=$F3T3" ++algo.config.eval.handover_hidden=zero
else
  for d in $DEGS; do
    HDEG="$d" ev "H_${d}deg_${HIDDEN}" "++algo.config.eval.handover_policy_path=$F3T3" "++algo.config.eval.handover_hidden=$HIDDEN"
  done
fi
log "終わり"
