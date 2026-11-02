#!/usr/bin/env bash
# 11-02: 64 台で 2 回に分けて動画を撮る（4 回目と 8 回目を同じ扉で並べる）。
#   1 回目: 動画なしで評価し、64 台での失敗の台を出す（同じ重み・同じ台数なら評価は決まった結果になる）
#   2 回目: 4 回目と 8 回目の失敗の台を合わせた台（16 台まで）だけ動画に書いて評価し直す（64 本を同時に書くと RAM の上限で止められた）
# 重みは 2 通り: 保存された線のまま（s: 4 回目 5.9・8 回目 5.0 rad/s）と、線を 20 rad/s に戻した写し（l: ckpt/lvl1.0_*）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid -f nohup ./exp_video64b_1102.sh >> logs_video64b_1102.log 2>&1 < /dev/null
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
  [r4s]="ckpt/start_1102_1058" [r8s]="ckpt/start_1102_1314"
  [r4l]="ckpt/lvl1.0_r4" [r8l]="ckpt/lvl1.0_r8"
)
declare -A OUT

# 引数: 名前・VIDEO（0/1）・足す hydra の引数
run() {
  local name="$1" video="$2" more="$3" lg out
  lg="logs_video64b_1102_${name}_v${video}.log"
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

for name in r4s r8s r4l r8l; do run "$name" 0 ""; done
for pair in "r4s r8s" "r4l r8l"; do
  set -- $pair
  [ -n "${OUT[$1]:-}" ] && [ -n "${OUT[$2]:-}" ] || { log "1 回目の記録が無いので飛ばす: $pair"; continue; }
  ids=$(fail_ids "${OUT[$1]}" "${OUT[$2]}")
  log "動画に書く台 ($pair): $ids"
  for name in "$1" "$2"; do
    run "$name" 1 "++simulator.config.render_env_ids=$ids"
    out="${OUT[$name]}"
    [ -n "$out" ] && [ -f "$out/per_env.json" ] || continue
    nice -n 15 "$PY" "$TOOLS/annotate_0089.py" "$out" "$HOME/work_inoue/doorman/video_1102/$name" "$name" > "logs_video64b_1102_${name}_ann.log" 2>&1
    log "書き込み ${name}: $(grep -c annotate "logs_video64b_1102_${name}_ann.log") 本"
  done
done
./pause_training.sh off
log "video64b_1102 が終わった"
