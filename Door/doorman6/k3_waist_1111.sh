#!/usr/bin/env bash
# 11-11（計画 docs/plan/2026-11-11-g1-door-stage2-fix.md の K3）: 腰 yaw で体の回りを打ち消す（パッチ 0105 の env.config.waist_yaw_comp）と、
#   K2 の外乱の下で成功が戻るかを、今の s2f 最良（start_1110_1703）で見る（学習なし）。
#   C0: 打ち消しだけ（外乱なし。基準 Y0 217/256 から −3 pt 以内が線）  C20・C30: 打ち消し＋K2 の +20°・+30°（基準 Y20 61.3%・Y30 47.3%。+20 pt が線）
#   C20p: C20 と同じで、扉の値を pelvis から見たまま（door_obs=pelvis。二重に直すかを見る）
#   打ち消しは倍率 1・上限 35°・観測から打ち消しの分を引く（hide_obs）。どれも学習に無い扉（種 999）・256 回・成功は扉 3°
#   ⚠️ シムの外乱は回りを外から与えるので、腰を回す反作用で pelvis がさらに回る実機の効き方は出ない（上限の見積もり）
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup bash k3_waist_1111.sh >> logs_k3_waist_1111.log 2>&1 < /dev/null &
#   K2・K6a・K4 の台本が走っている間は待つ
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
log() { echo "=== [$(date "+%m-%d %T")] k3: $*"; }
CK="${CK:-$HOME/work_inoue/doorman/ckpt/start_1110_1703/last.pt}"
OUT="$HOME/work_inoue/doorman/k2_yaw_1111.tsv"  # K2 と同じ表に足す
ANA="$HOME/work_inoue/ana_1111"
NAMES="${NAMES:-C20 C0 C20p C30}"
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++seed=999"
YD="++env.config.yaw_disturb.enabled=True ++env.config.yaw_disturb.start_s=[1.4,1.6] ++env.config.yaw_disturb.dur_s=[0.5,0.5] ++env.config.yaw_disturb.sign=lever_side"
WYC="++env.config.waist_yaw_comp.enabled=True ++env.config.waist_yaw_comp.gain=1.0 ++env.config.waist_yaw_comp.max_deg=35.0 ++env.config.waist_yaw_comp.hide_obs=True"
[ -f "$CK" ] || { log "重みが無い: $CK"; exit 1; }
grep -qF "_wyc_update" "$HOME/work_inoue/doorman6/VIRAL6_fix/gr00t/rl/envs/door/door_open_homie.py" || { log "パッチ 0105 が VIRAL6_fix に当たっていない"; exit 1; }
source ./gate.sh

while pgrep -f "[k]2_yaw_1111.sh|[k]6a_wall_1111.sh|[k]4_arm_1111.sh" > /dev/null; do sleep 30; done
ev() {
  local name="$1"; shift
  local evlog="logs_k3_waist_1111_${name}.log" rec rate succ
  while ! desk_idle || [ "$(others_gpu_mb)" -gt 4511 ]; do sleep 30; done  # 机で使われている間・他人が GPU を使っている間は始めない
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=60 MEM_MAX=9G \
    EXTRA_ARGS="$BASE_ARGS $*" ./eval_from_start.sh "$CK" 256 > "$evlog" 2>&1
  rec=$(grep -o 'out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  grep -q "\[waist_yaw_comp\]" "$rec/eval.log" 2>/dev/null || log "$name: 打ち消しの口が起きていない（$rec/eval.log）"
  rate=$("$PY" -c 'import json,sys; print(json.load(open(sys.argv[1]))["success_rate"])' "$rec/summary.json" 2>/dev/null || echo NA)
  succ=$("$PY" -c 'import json,sys; d=json.load(open(sys.argv[1])); print(str(d["success"]) + "/" + str(d["episodes_from_stage0"]))' "$rec/summary.json" 2>/dev/null || echo NA)
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$name" "$CK" "$rate" "$succ" "$rec" "$*" >> "$OUT"
  log "$name success=$succ rate=$rate record=$rec"
  (cd "$ANA" && "$PY" ana_stage2_sim_1111.py "$rec" --urdf g1_29dof_rev_1_0.urdf --json "k3_${name}.json" > "k3_${name}.txt" 2>&1) \
    || log "$name: 解析が落ちた（$ANA/k3_${name}.txt）"
}

log "始め ck=$CK names=$NAMES"
for n in $NAMES; do
  case "$n" in
    C0) ev C0 "$WYC ++env.config.waist_yaw_comp.door_obs=torso" ;;
    C20) ev C20 "$WYC ++env.config.waist_yaw_comp.door_obs=torso $YD ++env.config.yaw_disturb.amp_deg=[20,20]" ;;
    C20p) ev C20p "$WYC ++env.config.waist_yaw_comp.door_obs=pelvis $YD ++env.config.yaw_disturb.amp_deg=[20,20]" ;;
    C30) ev C30 "$WYC ++env.config.waist_yaw_comp.door_obs=torso $YD ++env.config.yaw_disturb.amp_deg=[30,30]" ;;
    *) log "知らない名前 $n" ;;
  esac
done
log "終わり"
