#!/usr/bin/env bash
# 11-11（計画 docs/plan/2026-11-11-g1-door-stage2-fix.md の K6a）: 壁と枠の出を実物（壁 1 cm・枠 3 cm。シムの元は 5・6 cm）にした課題
#   door_en1154_g13_shaped_left_weight_realwall（パッチ 0104 の door.py が要る）で、今の s2f 最良（start_1110_1703）を評価する。学習はしない。
#   W0: 外乱なし  W20: K2 の Y20 と同じ外乱（1.4〜1.6 s に始めて 0.5 s で把手の側へ +20°）
#   どれも学習に無い扉（種 999）・256 回・成功は扉 3°。同じ種なら扉は K2 と同じで、壁と枠だけが違う（door.py は乱数を引かない）
#   解析は ana_stage2_sim_1111.py --wall-x -0.03（~/work_inoue/ana_1111/k6a_<名前>.json）
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup bash k6a_wall_1111.sh >> logs_k6a_wall_1111.log 2>&1 < /dev/null &
#   K2 の台本（k2_yaw_1111.sh）が走っている間は待つ
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
log() { echo "=== [$(date "+%m-%d %T")] k6a: $*"; }
CK="${CK:-$HOME/work_inoue/doorman/ckpt/start_1110_1703/last.pt}"
OUT="$HOME/work_inoue/doorman/k2_yaw_1111.tsv"  # K2 と同じ表に足す
ANA="$HOME/work_inoue/ana_1111"
NAMES="${NAMES:-W0 W20}"
TASK=door_en1154_g13_shaped_left_weight_realwall
REPO="$HOME/work_inoue/doorman6/VIRAL6_fix"
BASE_ARGS="++env.config.task.name=$TASK ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++seed=999"
YD="++env.config.yaw_disturb.enabled=True ++env.config.yaw_disturb.start_s=[1.4,1.6] ++env.config.yaw_disturb.dur_s=[0.5,0.5] ++env.config.yaw_disturb.sign=lever_side"
[ -f "$CK" ] || { log "重みが無い: $CK"; exit 1; }
[ -f "$REPO/gr00t/rl/data/tasks/$TASK/scenario_cfg/isaacsim.py" ] || { log "課題 $TASK が無い"; exit 1; }
grep -qF "wall_front_out" "$REPO/gr00t/rl/isaac_utils/playground/env_rand/door.py" || { log "パッチ 0104 の door.py が当たっていない"; exit 1; }
[ -f "$OUT" ] || printf 'name\tck\trate\tsuccess\trecord\tmore\n' > "$OUT"
source ./gate.sh

while pgrep -f "[k]2_yaw_1111.sh" > /dev/null; do sleep 30; done
ev() {
  local name="$1"; shift
  local evlog="logs_k6a_wall_1111_${name}.log" rec rate succ
  while ! desk_idle || [ "$(others_gpu_mb)" -gt 4511 ]; do sleep 30; done  # 机で使われている間・他人が GPU を使っている間は始めない
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=60 MEM_MAX=9G \
    EXTRA_ARGS="$BASE_ARGS $*" ./eval_from_start.sh "$CK" 256 > "$evlog" 2>&1
  rec=$(grep -o 'out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  # 評価本体の表示は記録の置き場の eval.log に出る（台本のログではない）
  if [ -n "$*" ] && ! grep -q "\[yaw_disturb\]" "$rec/eval.log" 2>/dev/null; then log "$name: 外乱の口が起きていない（$rec/eval.log）"; fi
  rate=$("$PY" -c 'import json,sys; print(json.load(open(sys.argv[1]))["success_rate"])' "$rec/summary.json" 2>/dev/null || echo NA)
  succ=$("$PY" -c 'import json,sys; d=json.load(open(sys.argv[1])); print(str(d["success"]) + "/" + str(d["episodes_from_stage0"]))' "$rec/summary.json" 2>/dev/null || echo NA)
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$name" "$CK" "$rate" "$succ" "$rec" "task=$TASK $*" >> "$OUT"
  log "$name success=$succ rate=$rate record=$rec"
  (cd "$ANA" && "$PY" ana_stage2_sim_1111.py "$rec" --urdf g1_29dof_rev_1_0.urdf --wall-x -0.03 --json "k6a_${name}.json" > "k6a_${name}.txt" 2>&1) \
    || log "$name: 解析が落ちた（$ANA/k6a_${name}.txt）"
}

log "始め ck=$CK task=$TASK names=$NAMES"
for n in $NAMES; do
  case "$n" in
    W0) ev W0 ;;
    W*) a="${n#W}"; ev "$n" "$YD ++env.config.yaw_disturb.amp_deg=[$a,$a]" ;;
    *) log "知らない名前 $n" ;;
  esac
done
log "終わり"
