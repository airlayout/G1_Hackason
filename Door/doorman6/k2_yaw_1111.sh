#!/usr/bin/env bash
# 11-11（計画 docs/plan/2026-11-11-g1-door-stage2-fix.md の K2・ユーザー「引き継ぎに従い実施して」）:
#   段 2 の途中で胴を把手の側へ回す外乱（パッチ 0104 の env.config.yaw_disturb）で、11-11 の実機の空振り（触れた所が把手の側へ
#   9〜14 cm・成功が大きく落ちる）がシムで再現するかを見る。学習はしない。重みは s2f 最良（start_1110_1703）。
#   Y0: 外乱なし（同じ扉の組の基準）  Y10・Y20・Y30: 1.4〜1.6 s に始めて 0.5 s で +10・+20・+30°（実機は 2.0 s で +10.6〜+24.7°）
#   どれも学習に無い扉（種 999）・256 回・成功は扉 3°（count_success.py --criterion arm3）。終わったら ana_stage2_sim_1111.py で
#   触れた所・触れる前の回り・壁への寄りを出す（~/work_inoue/ana_1111/k2_<名前>.json）
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup bash k2_yaw_1111.sh >> logs_k2_yaw_1111.log 2>&1 < /dev/null &
#   NAMES="Y20" で一部だけ回す
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
log() { echo "=== [$(date "+%m-%d %T")] k2: $*"; }
CK="${CK:-$HOME/work_inoue/doorman/ckpt/start_1110_1703/last.pt}"
OUT="$HOME/work_inoue/doorman/k2_yaw_1111.tsv"
ANA="$HOME/work_inoue/ana_1111"
NAMES="${NAMES:-Y0 Y10 Y20 Y30}"
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++seed=999"
YD="++env.config.yaw_disturb.enabled=True ++env.config.yaw_disturb.start_s=[1.4,1.6] ++env.config.yaw_disturb.dur_s=[0.5,0.5] ++env.config.yaw_disturb.sign=lever_side"
[ -f "$CK" ] || { log "重みが無い: $CK"; exit 1; }
grep -qF "_apply_yaw_disturb" "$HOME/work_inoue/doorman6/VIRAL6_fix/gr00t/rl/envs/door/door_open_homie.py" || { log "パッチ 0104 が VIRAL6_fix に当たっていない"; exit 1; }
[ -f "$OUT" ] || printf 'name\tck\trate\tsuccess\trecord\tmore\n' > "$OUT"
source ./gate.sh

ev() {
  local name="$1"; shift
  local evlog="logs_k2_yaw_1111_${name}.log" rec rate succ
  while ! desk_idle || [ "$(others_gpu_mb)" -gt 4511 ]; do sleep 30; done  # 机で使われている間・他人が GPU を使っている間は始めない
  EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion arm3" MAX_WAIT_MIN=60 MEM_MAX=9G \
    EXTRA_ARGS="$BASE_ARGS $*" ./eval_from_start.sh "$CK" 256 > "$evlog" 2>&1
  rec=$(grep -o 'out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  # 評価本体の表示は記録の置き場の eval.log に出る（11-11 の初版は台本のログを見て「起きていない」と誤って出した）
  if [ -n "$*" ] && ! grep -q "\[yaw_disturb\]" "$rec/eval.log" 2>/dev/null; then log "$name: 外乱の口が起きていない（$rec/eval.log）"; fi
  rate=$("$PY" -c 'import json,sys; print(json.load(open(sys.argv[1]))["success_rate"])' "$rec/summary.json" 2>/dev/null || echo NA)
  succ=$("$PY" -c 'import json,sys; d=json.load(open(sys.argv[1])); print(str(d["success"]) + "/" + str(d["episodes_from_stage0"]))' "$rec/summary.json" 2>/dev/null || echo NA)
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$name" "$CK" "$rate" "$succ" "$rec" "$*" >> "$OUT"
  log "$name success=$succ rate=$rate record=$rec"
  (cd "$ANA" && "$PY" ana_stage2_sim_1111.py "$rec" --urdf g1_29dof_rev_1_0.urdf --json "k2_${name}.json" > "k2_${name}.txt" 2>&1) \
    || log "$name: 解析が落ちた（$ANA/k2_${name}.txt）"
}

log "始め ck=$CK names=$NAMES"
for n in $NAMES; do
  case "$n" in
    Y0) ev Y0 ;;
    Y*) a="${n#Y}"; ev "$n" "$YD ++env.config.yaw_disturb.amp_deg=[$a,$a]" ;;
    *) log "知らない名前 $n" ;;
  esac
done
log "終わり"
