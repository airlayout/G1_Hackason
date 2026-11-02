#!/usr/bin/env bash
# 11-02 ユーザー「失敗の原因を動画を含めてしっかり分析し、4 回目と今で併記して」。パッチ 0089（終わった理由の記録・選んだ台だけの動画）。
# 連鎖（chain/1102_0846）が止め札で止まるのを待ってから、4 本を順に回す（どれも 256 回・評価の設定は連鎖と同じ）:
#   1. 4 回目の重み（反復 65,350・93.4%）で、4 回目と 8 回目の失敗の台 45 台を Isaac Sim の画で写す
#   2. 8 回目の重み（反復 65,950・87.1%）で、同じ 45 台を写す（同じ扉で並べる）
#   3. 4 回目の重みで、上半身の関節の速さの打ち切りを外す（解析のためだけ。打ち切られなければ通れたかを見る）
#   4. 8 回目の重みで、同じく外す
#   ⚠️ 3・4 は効かなかった: 線（termination_level）は重みの env_state_dict に保存され、設定より後に読まれる（失敗の台は元と 1 台も
#   変わらなかった）。線を変えて測るのは exp_level_1102.sh（重みの写しを書き換える）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid -f nohup ./exp_term_1102.sh >> logs_term_1102.log 2>&1 < /dev/null
#   打ち切りを外した 2 本を飛ばす: SKIP_NOSPEED=1（14:45 に済んだ。失敗の台は元の評価と 1 台も変わらなかった）
# 動画は 256 台を全部描いて 45 台だけ書く（Isaac Lab 3.0 のカメラは台の数だけ要る）。1 台 640×480 だと描画の画が 10,240×7,680 になり
# DLSS の準備が失敗した（NGX CreateFeature failed）ので 480×360（7,680×5,760）にする
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
# 札は自分の PAUSE_DOOR_F3（連鎖の学習もこれを見る）。gate.sh の既定 PAUSE_DOORMAN_TRAINING はユーザーの札と同じ名前なので使わない
# （11-02 14:25〜16:09 の評価は渡し忘れて、ユーザーの札を置いては外していた）
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
TASK=door_en1154_g13_shaped_left_weight
EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
R4="$HOME/work_inoue/doorman/ckpt/start_1102_1058/last.pt"
R8="$HOME/work_inoue/doorman/ckpt/start_1102_1314/last.pt"
IDS="[4,8,11,12,26,34,48,50,52,58,63,71,72,80,81,100,101,106,107,110,116,125,135,137,142,143,152,165,172,177,178,185,189,195,197,198,200,201,218,227,230,239,246,250,254]"
NOSPEED="++env.config.termination_curriculum.initial_level=1000 ++env.config.termination_curriculum.max_level=1000"
RESULTS="$HOME/work_inoue/doorman/term_1102.tsv"
PY="$HOME/miniforge3/envs/doorman6/bin/python"

grep -qF "パッチ 0089" "$HOME/work_inoue/doorman6/VIRAL6_fix/gr00t/rl/envs/door/door_open_homie.py" ||
  { log "パッチ 0089 が VIRAL6_fix に当たっていない"; exit 1; }
while pgrep -f "[s]tage_chain.sh" > /dev/null; do sleep 30; done
log "連鎖が止まった。始める"

# 引数: 名前・重み・VIDEO（0/1）・足す hydra の引数
run() {
  local name="$1" ck="$2" video="$3" more="$4" evlog out
  evlog="$HOME/work_inoue/doorman/logs_term_1102_${name}.log"
  log "評価 ${name} ck=${ck} video=${video} more=${more}"
  VIDEO="$video" RENDER_W="${RENDER_W:-480}" RENDER_H="${RENDER_H:-360}" KEEP_PAUSE=1 IDLE_GATE=0 MAX_WAIT_MIN=60 MINUTES=45 EVAL_REPO=VIRAL6_fix \
    COUNT_ARGS="--criterion through" EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA $more" \
    ./eval_from_start.sh "$ck" 256 > "$evlog" 2>&1
  out=$(grep -o 'out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  printf '%s\t%s\t%s\n' "$name" "$out" "$(grep -o 'success=[^ ]*' "$evlog" | tail -1)" >> "$RESULTS"
  log "評価 ${name} out=${out} $(grep -o 'success=[^ ]*' "$evlog" | tail -1)"
  [ -n "$out" ] && [ -f "$out/summary.json" ] && nice "$PY" "$HOME/work_inoue/doorman/term_analysis.py" "$out"
}

run r4_video "$R4" 1 "++simulator.config.render_env_ids=$IDS"
run r8_video "$R8" 1 "++simulator.config.render_env_ids=$IDS"
if [ "${SKIP_NOSPEED:-0}" != 1 ]; then
  run r4_nospeed "$R4" 0 "$NOSPEED"
  run r8_nospeed "$R8" 0 "$NOSPEED"
fi
./pause_training.sh off
log "term_1102 が終わった"
