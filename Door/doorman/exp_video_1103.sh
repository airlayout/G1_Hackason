#!/usr/bin/env bash
# 11-03: 締め始め 0.9 の連鎖（chain/1102_1704）の今の重みと、11-02 午前の 4 回目の重みを同じ扉で並べて動画にする（作業ログ 12 章）。
# 撮り方は exp_video64b_1102.sh と同じ 64 台の 2 回に分ける方法:
#   1 回目: 動画なしで評価し、64 台での失敗の台を出す（同じ重み・同じ台数なら評価は決まった結果になる）
#   2 回目: 2 つの重みの失敗の台を合わせた台（16 台まで）だけ動画に書いて評価し直す
# 重みはどちらも打ち切りの線 20 rad/s: r4l は 4 回目（65,350）の線を 1.0 に書き換えた写し、now は 25 回目（69,500。線は学習で 1.0 に戻った）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid -f nohup ./exp_video_1103.sh >> logs_video_1103.log 2>&1 < /dev/null
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
# 札は自分の PAUSE_DOOR_F3（連鎖の学習もこれを見る）。gate.sh の既定 PAUSE_DOORMAN_TRAINING はユーザーの札と同じ名前なので使わない
# （11-02 14:25〜16:09 の評価は渡し忘れて、ユーザーの札を置いては外していた）
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
PY="$HOME/miniforge3/envs/doorman6/bin/python"
TOOLS="${TOOLS:-$HOME/work_inoue/doorman}"
TASK=door_en1154_g13_shaped_left_weight
EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
MAX_VIDEOS="${MAX_VIDEOS:-16}"
declare -A CK=(
  [r4l]="ckpt/lvl1.0_r4" [now]="${NOW_CK:-ckpt/start_1103_0703}"
)
declare -A OUT

# 引数: 名前・VIDEO（0/1）・足す hydra の引数
run() {
  local name="$1" video="$2" more="$3" lg out
  lg="logs_video_1103_${name}_v${video}.log"
  log "評価 ${name} video=${video} ${more}"
  VIDEO="$video" RENDER_W=480 RENDER_H=360 KEEP_PAUSE=1 IDLE_GATE=0 MAX_WAIT_MIN=60 EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion through" \
    EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA $more" ./eval_from_start.sh "$HOME/work_inoue/doorman/${CK[$name]}/last.pt" 64 > "$lg" 2>&1
  out=$(grep -o 'out=[^ ]*' "$lg" | tail -1 | cut -d= -f2)
  log "評価 ${name} video=${video} out=${out} $(grep -o 'success=[^ ]*' "$lg" | tail -1)"
  OUT[$name]="$out"
  [ -n "$out" ] && [ -f "$out/summary.json" ] && nice "$PY" "$TOOLS/term_analysis.py" "$out"
}

# 2 つの記録の失敗の台を合わせ、小さい番号から MAX_VIDEOS 台まで（hydra の並びの形で出す）
fail_ids() {
  "$PY" - "$1" "$2" "$MAX_VIDEOS" <<'PYEOF'
import json, sys
ids = set()
for rec in sys.argv[1:3]:
    ids |= {e["env"] for e in json.load(open(f"{rec}/summary.json"))["per_env"] if not e["success"]}
print("[" + ",".join(str(i) for i in sorted(ids)[: int(sys.argv[3])]) + "]")
PYEOF
}

for name in r4l now; do run "$name" 0 ""; done
for pair in "r4l now"; do
  set -- $pair
  [ -n "${OUT[$1]:-}" ] && [ -n "${OUT[$2]:-}" ] || { log "1 回目の記録が無いので飛ばす: $pair"; continue; }
  ids=$(fail_ids "${OUT[$1]}" "${OUT[$2]}")
  log "動画に書く台 ($pair): $ids"
  for name in "$1" "$2"; do
    run "$name" 1 "++simulator.config.render_env_ids=$ids"
    out="${OUT[$name]}"
    [ -n "$out" ] && [ -f "$out/per_env.json" ] || continue
    nice -n 15 "$PY" "$TOOLS/annotate_0089.py" "$out" "$HOME/work_inoue/doorman/video_1103/$name" "$name" > "logs_video_1103_${name}_ann.log" 2>&1
    log "書き込み ${name}: $(grep -c annotate "logs_video_1103_${name}_ann.log") 本"
  done
done
./pause_training.sh off
log "video_1103 が終わった"
