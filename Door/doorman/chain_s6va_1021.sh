#!/usr/bin/env bash
# 段 6 (b)（10-21）: 映像の生徒の DAgger を、合格するまで「評価 → 続きから学習」で回す鎖（OMEN、~/work_inoue/doorman6）。
#   1. 今の学習（exp_campitch_1021.sh dagger か、鎖が起こした train_yield_s6va_1021.sh）が動いていれば終わるのを待つ
#      （10-21 は train_yield_s6va_1021.sh の後に && でつないで起こした。10-22 に評価の区切り方を直して START_ROUND=2 で起こし直した）
#   2. 最新の last.pt を ckpt/s6va_r<回>/ に写し、俯角 PITCHES（既定 40°・65°）で評価する
#      （16 台・1 回の起動で台ごとに何回も・count_success_cutoff.py で偏らずに数える。数えられる回が EVAL_EPISODES（既定 100）に
#       届いたら止める。上限 EVAL_MAX_MIN 分（既定 60）。⚠️ 10-22 の 1 回目は 14 分で区切ったので、機械が遅い時間に当たった俯角 65° が 16 回しか数えられなかった）
#   3. 俯角の平均が合格の線（PASS_RATE、既定 0.659＝教師 82.4% の 8 割。状態入力の生徒と同じ線）以上なら
#      ckpt/s6va_pass_r<回>/ に残して止まる
#   4. 届かなければ、その重みから TRAIN_MIN 分（既定 360）学習を続けて 2 へ（他の人に譲りながら）。ROUNDS 回（既定 8）まで
# 評価の条件は状態入力の生徒の合否と同じ（立った姿勢から・区間の時間 2 倍・回の上限 60 秒・教師は運転しない）。
# カメラは学習の設定（赤外線の画角・灰色）のまま、俯角だけ乱択を切って決まった角度にする。
# 止め札: ~/work_inoue/STOP_S6VA を置くと、次の学習の前で止まる（評価は済ませる）。始めるときに残っていたら消す
# 出力: chain_s6va/<日時>/chain.log・results.tsv（回・俯角・成功・回数・率・ckpt）・eval_r<回>_p<俯角>/
#
# 使い方: setsid nohup bash chain_s6va_1021.sh > logs_chain_s6va_1021.log 2>&1 < /dev/null &
#   START_ROUND=<回> … その回から始める（回の番号は ckpt/s6va_r<回>/ の名前になる）
# うまく進んでいないときの切り替え（10-22、ユーザー承認）: 評価のたびに、全部の俯角の回を合わせて区間の通り抜けを数え
#   （stage_funnel.py）、成功 < SWITCH_SUCCESS（既定 0.10）かつ 3→4 < SWITCH_C34（既定 0.30）なら、次の学習から
#   STAGED_ARGS（既定 ++env.config.enable_staged_reset=True＝途中の区間から回を始める。状態入力の生徒と同じ配分）を入れる。
#   一度入れたら外さない。入れた学習が起動しなければ（train_yield が失敗で終わる）、外して同じ回の学習をやり直す
set -uo pipefail
W6="$HOME/work_inoue/doorman6"
BASE="$HOME/work_inoue/doorman"
cd "$W6" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
RUNS="$W6/VIRAL6/logs_rl/port6/wbmanip"
PASS_RATE="${PASS_RATE:-0.659}"
ROUNDS="${ROUNDS:-8}"
TRAIN_MIN="${TRAIN_MIN:-360}"
PITCHES="${PITCHES:-40 65}"
EVAL_EPISODES="${EVAL_EPISODES:-100}"
EVAL_MAX_MIN="${EVAL_MAX_MIN:-60}"
START_ROUND="${START_ROUND:-1}"
SWITCH_SUCCESS="${SWITCH_SUCCESS:-0.10}"
SWITCH_C34="${SWITCH_C34:-0.30}"
STAGED_ARGS="${STAGED_ARGS:-++env.config.enable_staged_reset=True}"
STAGED="${STAGED:-0}"
STAGED_FAILED=0
STOP_FILE="$HOME/work_inoue/STOP_S6VA"
OUTDIR="$W6/chain_s6va/$(date +%m%d_%H%M)"
mkdir -p "$OUTDIR"
rm -f "$STOP_FILE"
log() { echo "=== [$(date '+%m-%d %T')] $*" | tee -a "$OUTDIR/chain.log"; }
EVAL_ARGS=(++env.config.reset_from_dataset.enabled=False "++env.config.max_stage_time=[500,200,200,200,600,400]"
           ++env.config.max_episode_length_s=60 ++algo.config.enforce_teacher_rollout=False
           ++env.config.enable_staged_reset=False
           ++algo.config.eval.eval_num_envs_episodes=False ++algo.config.eval.reset_policy_on_done=True
           ++domain_rand.randomize_camera_pitch=False)

latest_ckpt() { ls -t "$RUNS"/door_open_homie_29dof_dagger-lstm_lstm-*/last.pt 2>/dev/null | head -1; }

decide_staged() {  # $1=回。評価の後に、うまく進んでいなければ次の学習から STAGED_ARGS を入れる
  local r="$1" dirs=() p sr c34
  [ "$STAGED" = 0 ] && [ "$STAGED_FAILED" = 0 ] || return 0
  for p in $PITCHES; do [ -d "$OUTDIR/eval_r${r}_p$p" ] && dirs+=("$OUTDIR/eval_r${r}_p$p"); done
  if [ "${#dirs[@]}" -eq 0 ] || ! "$PY" "$W6/stage_funnel.py" "${dirs[@]}" --json "$OUTDIR/funnel_r$r.json" > "$OUTDIR/funnel_r$r.txt" 2>&1; then
    log "round=$r 区間の集計ができない → 学習の設定はそのまま"; return 0
  fi
  read -r sr c34 < <(python3 -c "import json,sys; j=json.load(open(sys.argv[1])); c=j['conv']['3to4']; print(j['success_rate'] or 0, 0 if c is None else c)" "$OUTDIR/funnel_r$r.json")
  log "round=$r 区間の通り抜け（全部の俯角）: 成功 ${sr}・3→4 ${c34}（切り替えの線: 成功 < ${SWITCH_SUCCESS} かつ 3→4 < ${SWITCH_C34}）"
  if python3 -c "import sys; sys.exit(0 if $sr < $SWITCH_SUCCESS and $c34 < $SWITCH_C34 else 1)"; then
    STAGED=1
    log "round=$r うまく進んでいない → 次の学習から途中の区間から始める仕組みを入れる（${STAGED_ARGS}）"
  else
    log "round=$r 線を越えた → 学習の設定はそのまま"
  fi
}

eval_pitch() {  # $1=ckpt $2=俯角 $3=出力先 → 率（0〜1）を出す。数えられなければ空
  local q
  q=$(python3 -c "import math; t=math.radians($2)/2; print(f'[{math.cos(t):.6f},0.0,{math.sin(t):.6f},0.0]')")
  (cd "$BASE" && WORK="$W6" REPO=VIRAL6 ENV_NAME=doorman6 IGNORE_PAUSE=1 IDLE_GATE="${IDLE_GATE:-1}" \
    MAX_WAIT_MIN="${MAX_WAIT_MIN:-120}" MEM_MAX="${EVAL_MEM:-11G}" NEED_MARGIN_MB=512 STOP_WHEN_EPISODES="$EVAL_EPISODES" \
    ./run_eval_record.sh "$1" "$3" 16 100000 "$EVAL_MAX_MIN" "${EVAL_ARGS[@]}" \
    ++simulator.config.cameras.camera_rot_offset="$q" ++seed=0) > "$3.log" 2>&1
  "$PY" "$BASE/count_success_cutoff.py" "$3" --json "$3/summary_cutoff.json" > "$3.summary.txt" 2>&1 || return 0
  python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['rate'])" "$3/summary_cutoff.json" 2>/dev/null
}

log "chain start pass_rate=$PASS_RATE rounds=${START_ROUND}..$ROUNDS train_min=$TRAIN_MIN pitches=$PITCHES eval=${EVAL_EPISODES} 回（上限 ${EVAL_MAX_MIN} 分）staged=$STAGED switch=成功<${SWITCH_SUCCESS}かつ3→4<${SWITCH_C34}"
printf 'round\tpitch\tsuccess\tepisodes\trate\tckpt\n' > "$OUTDIR/results.tsv"
# ⚠️ 自分のコマンド行に当たらない型にする（起こした bash -c の包みは train_yield の名前を含むが、先頭が bash -c なので当たらない）
while pgrep -f "^bash .*exp_campitch_1021\.sh dagger" > /dev/null \
   || pgrep -f "^bash /home/ubuntu/work_inoue/doorman6/train_yield_s6va_1021\.sh" > /dev/null; do sleep 60; done
log "今の学習が終わった"

for R in $(seq "$START_ROUND" "$ROUNDS"); do
  SRC="$(latest_ckpt)"
  [ -n "$SRC" ] || { log "last.pt が見つからない → 止める"; exit 1; }
  KEEP="$BASE/ckpt/s6va_r$R"
  mkdir -p "$KEEP"
  cp "$SRC" "$KEEP/last.pt" && cp "$(dirname "$SRC")"/{config,meta}.yaml "$KEEP/" || { log "ckpt を写せない: $SRC"; exit 1; }
  log "round=$R ckpt=$SRC → ${KEEP}（評価する）"
  sum=0; cnt=0
  for P in $PITCHES; do
    E="$OUTDIR/eval_r${R}_p$P"
    rate="$(eval_pitch "$KEEP/last.pt" "$P" "$E")"
    if [ -z "$rate" ]; then
      log "round=$R pitch=$P 評価が数えられない（$E.log を見る）"
      printf '%s\t%s\t\t\t\t%s\n' "$R" "$P" "$KEEP" >> "$OUTDIR/results.tsv"
      continue
    fi
    read -r succ eps < <(python3 -c "import json,sys; j=json.load(open(sys.argv[1])); print(j['success'], j['episodes'])" "$E/summary_cutoff.json")
    log "round=$R pitch=$P success=$succ/$eps rate=$rate"
    [ "$eps" -ge "$EVAL_EPISODES" ] || log "round=$R pitch=$P ⚠️ 数えられた回が ${eps} で、${EVAL_EPISODES} に届かなかった（上限 ${EVAL_MAX_MIN} 分。機械が遅かった？）"
    printf '%s\t%s\t%s\t%s\t%s\t%s\n' "$R" "$P" "$succ" "$eps" "$rate" "$KEEP" >> "$OUTDIR/results.tsv"
    sum=$(python3 -c "print($sum + $rate)"); cnt=$((cnt + 1))
  done
  if [ "$cnt" -gt 0 ]; then
    mean=$(python3 -c "print($sum / $cnt)")
    log "round=$R 俯角の平均 rate=${mean}（合格の線 ${PASS_RATE}）"
    if python3 -c "import sys; sys.exit(0 if $mean >= $PASS_RATE else 1)"; then
      PASS="$BASE/ckpt/s6va_pass_r$R"
      cp -r "$KEEP" "$PASS"
      log "PASS round=$R rate=$mean kept=$PASS"
      log "chain_done result=pass"
      exit 0
    fi
  fi
  decide_staged "$R"
  if [ -e "$STOP_FILE" ]; then log "止め札（${STOP_FILE}）があるので止める"; log "chain_done result=stopped"; exit 0; fi
  [ "$R" -lt "$ROUNDS" ] || break
  log "round=$R 届かない → $KEEP/last.pt から ${TRAIN_MIN} 分 学習を続ける"
  # 共用機の他の人に譲りながら学習する（train_yield_s6va_1021.sh。譲った分は空いてから続きを回して合計 TRAIN_MIN 分）
  extra=""; [ "$STAGED" = 1 ] && extra="$STAGED_ARGS"
  log "round=$R 学習の追加の引数: ${extra:-なし}"
  if ! TRAIN_EXTRA="$extra" bash "$W6/train_yield_s6va_1021.sh" "$TRAIN_MIN" "$KEEP/last.pt" > "$OUTDIR/train_r$R.log" 2>&1; then
    log "round=$R 学習が失敗で終わった（$OUTDIR/train_r$R.log）"
    if [ "$STAGED" = 1 ]; then
      log "round=$R 途中の区間から始める仕組みで学習が続かなかった → 外して、この回の学習をやり直す"
      STAGED=0; STAGED_FAILED=1
      TRAIN_EXTRA="" bash "$W6/train_yield_s6va_1021.sh" "$TRAIN_MIN" "$(latest_ckpt)" > "$OUTDIR/train_r${R}_retry.log" 2>&1 \
        || log "round=$R やり直しの学習も失敗で終わった（$OUTDIR/train_r${R}_retry.log）"
    fi
  fi
  if [ "$(latest_ckpt)" = "$SRC" ]; then
    log "round=$R 新しい last.pt ができていない（学習が始まらなかった）→ 同じ重みを評価し直さずに止める"; exit 1
  fi
done
log "chain_done result=rounds_exhausted"
