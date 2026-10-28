#!/usr/bin/env bash
# 段 2-2・2-3 を順に学習して合否を測る（学習 → 最初の区間からの評価 → 合格なら次の段）。
# 段ごとに「学習を MIN 分 → eval_from_start.sh の 128 回」を繰り返し、成功率が合格の線（既定 0.8）を越えたら合格。
# 合格したチェックポイントを ckpt/<段>_pass_<日時>/ に残し、8 台の動画を撮ってから、次の段をそこから始める。
# 上限の回数まで回して越えなければ、その段で止める（段 2-3 なら、その等級が「開けられない」）。
#
# 使い方（OMEN の ~/work_inoue/doorman で）:
#   ./stage_chain.sh <最初の ckpt> <段>...
#   段は「名前:タスク名:回数の上限:1 回の学習の分[:成功の定義[:学習に足す hydra の引数（, 区切り）[:合否を見るタスク名]]]」。成功の定義は count_success.py の --criterion
#   （through＝通り抜け〔既定〕・door30＝レバーを下げて扉を 30° 以上〔段 2-2、2026-10-08 のユーザー判断〕）。例:
#     ./stage_chain.sh ckpt/stage21_pass_it4950/last.pt s22:door:6:60 g1:door_en1154_g1:3:60 g2:door_en1154_g2:3:60
#     g3:door_en1154_g3:3:60:door30:++env.config.push_door_force_max=50,++rewards.reward_scales.push_door_force=0.12
#   6 つ目の欄は学習だけに足す（評価は ckpt の config.yaml を読み、成功の数え方は報酬に依らない）
#   7 つ目の欄は合否を見る扉（省くと学習と同じ扉）。段 2-3 は、易しい扉を混ぜた扉で学習し、純粋な等級の扉で合否を見る:
#     m1:door_en1154_upto_g1:3:60:door30::door_en1154_g1
# 札は PAUSE_FILE（既定 ~/work_inoue/PAUSE_DOOR_STAGE2）。段 2-1 のキューの札（PAUSE_DOORMAN_TRAINING）とは別にしてある。
# STALL_MAX（既定 3）: 学習は始まったのに重みが増えない回がこれだけ続いたら止める（result=stalled_at_<段>）。
#   10-26 21 時〜10-27 07:30 に、反復の上限（algo.trl.num_total_batches）に届いた学習が始まるたび 40 秒で終わり、
#   「増えていない」と書いて次の回へ進むだけの空回りを 10 時間続けた。空くのを待っただけの回（学習が始まらない）は数えない
# 止め札 STOP_FILE（既定 ~/work_inoue/STOP_DOOR_CHAIN）を置くと、次の回の前で止まる（評価は最後まで済ませる）。始めるときに残っていたら消す
# EVAL_N で評価の回数（既定 128。128 回の評価の揺れは ±3.5 ポイント）。PASS_RATE=1.1 にすると合格で止まらず決めた回数を回す（比べる用）
# IDLE_GATE=0 で、学習も評価も画面の前の人の在席を見ない（既定 1＝離席中だけ回す。共用機なので 0 は人に確かめてから）
# EVAL_FIRST=1 で、段ごとに学習の前に評価する（回 0。合格ならその段の学習を飛ばす。2026-10-08 に段 2-3 を短くするため足した）
# EVAL_EXTRA で評価だけに足す hydra の引数（空白区切り）。10-14 から合否は立った姿勢から・時間 2 倍で見る（段 2-4a の成功値の条件）:
#   EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60" MINUTES=30
# 最初の ckpt に none を渡すと、1 回目の学習は重みなしで始める（段 2.5 の生徒。EXP で exp を替え、教師は学習の引数で渡す）:
#   EXP=wbmanip/door_open_homie_29dof_state_dagger-lstm ./stage_chain.sh none "s25:<扉>:8:30:through:++teacher_actor_path=<教師>,..."
# 出力: chain/<日時>/chain.log・results.tsv（段・回・反復・成功・率・ckpt・記録）
set -uo pipefail
BASE="$HOME/work_inoue/doorman"
WORK6="$HOME/work_inoue/doorman6"
NUM_ENVS="${NUM_ENVS:-512}"
PASS_RATE="${PASS_RATE:-0.8}"
STALL_MAX="${STALL_MAX:-3}"
# 1: 段ごとに、学習の前にまず評価する（回 0）。合格ならその段は学習しない（段 2-3 の等級で、前の等級の方策がそのまま通るときの 60 分を省く）
EVAL_FIRST="${EVAL_FIRST:-0}"
export PAUSE_FILE="${PAUSE_FILE:-$HOME/work_inoue/PAUSE_DOOR_STAGE2}"
export STOP_FILE="${STOP_FILE:-$HOME/work_inoue/STOP_DOOR_CHAIN}"
rm -f "$STOP_FILE"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
# CHAIN_REPO: 学習と評価の作業ツリー（~/work_inoue/doorman6 の下。既定 VIRAL6。10-26 の学習し直しは VIRAL6_fix）
CHAIN_REPO="${CHAIN_REPO:-VIRAL6}"
cd "$BASE" || exit 1

CK="$1"; shift
if [ "$CK" = none ]; then
  CK=""  # 1 回目は重みなしで学習を始める
else
  [ -f "$CK" ] || { echo "最初の ckpt が無い: $CK"; exit 2; }
  CK="$(readlink -f "$CK")"
fi
OUTDIR="$BASE/chain/$(date +%m%d_%H%M)"
# 同じ分に始めた連鎖が前の連鎖の results.tsv を上書きしないように、あれば _01・_02… を付ける（10-28。並べると元の名前の後に来る）
k=0; while [ -e "$OUTDIR" ]; do k=$((k + 1)); OUTDIR="$BASE/chain/$(date +%m%d_%H%M)_$(printf %02d "$k")"; done
mkdir -p "$OUTDIR"
RESULTS="$OUTDIR/results.tsv"
printf 'stage\ttask\tround\titer\tsuccess\trate\tckpt\trecord\n' > "$RESULTS"
log() { echo "=== [$(date '+%m-%d %T')] $*" | tee -a "$OUTDIR/chain.log"; }

# チェックポイントの反復数（DoorMan の last.pt の state.global_step）
ckpt_iter() { "$PY" -c "import torch,sys; print(torch.load(sys.argv[1], map_location='cpu', weights_only=False)['state'].global_step)" "$1" 2>/dev/null || echo -1; }

# いまの CK を TASK で 128 回評価し、results.tsv と chain.log に書く（引数: 段の名前・タスク・回）。
# 合格なら ckpt/<段>_pass_it<反復>/ に写し、8 台の動画を撮って 0 を返す。不合格なら 1
eval_and_keep() {
  local name="$1" task="$2" round="$3" it evlog rec rate succ keep
  it=$(ckpt_iter "$CK")
  evlog="$OUTDIR/eval_${name}_r${round}.log"
  EVAL_REPO="$CHAIN_REPO" COUNT_ARGS="$COUNT_ARGS" EXTRA_ARGS="++env.config.task.name=$task ${EVAL_EXTRA:-}" ./eval_from_start.sh "$CK" "${EVAL_N:-128}" > "$evlog" 2>&1
  rec=$(grep -o 'out=[^ ]*' "$evlog" | tail -1 | cut -d= -f2)
  rate=$("$PY" -c "import json,sys; print(json.load(open(sys.argv[1]))['success_rate'])" "$rec/summary.json" 2>/dev/null || echo 0)
  succ=$("$PY" -c "import json,sys; d=json.load(open(sys.argv[1])); print(f\"{d['success']}/{d['episodes_from_stage0']}\")" "$rec/summary.json" 2>/dev/null || echo none)
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$name" "$task" "$round" "$it" "$succ" "$rate" "$CK" "$rec" >> "$RESULTS"
  log "stage=$name round=$round iter=$it success=$succ rate=$rate record=$rec"
  "$PY" -c "import sys; sys.exit(0 if float(sys.argv[1]) >= float(sys.argv[2]) else 1)" "$rate" "$PASS_RATE" || return 1
  keep="$BASE/ckpt/${name}_pass_it${it}"
  mkdir -p "$keep" && cp "$CK" "$keep/last.pt" && cp "$(dirname "$CK")"/{config,meta}.yaml "$keep/" && cp "$rec/summary.json" "$keep/summary_128.json"
  log "stage=$name PASS iter=$it rate=$rate kept=$keep"
  EVAL_REPO="$CHAIN_REPO" COUNT_ARGS="$COUNT_ARGS" VIDEO=1 EXTRA_ARGS="++env.config.task.name=$task ${EVAL_EXTRA:-}" ./eval_from_start.sh "$CK" 8 > "$OUTDIR/video_${name}.log" 2>&1
  log "stage=$name video: $(tail -1 "$OUTDIR/video_${name}.log")"
  return 0
}

log "chain start repo=$CHAIN_REPO ckpt=${CK:-none} exp=${EXP:-default} stages=$* pass_rate=$PASS_RATE pause_file=$PAUSE_FILE stop_file=$STOP_FILE eval_extra=${EVAL_EXTRA:-}"
for SPEC in "$@"; do
  IFS=: read -r NAME TASK ROUNDS MIN CRIT EXTRA EVAL_TASK <<< "$SPEC"
  EVAL_TASK="${EVAL_TASK:-$TASK}"
  TRAIN_EXTRA="${EXTRA//,/ }"
  TRAIN_EXTRA="${TRAIN_EXTRA//|/,}"  # リストの中のカンマは | で書く（例: ++env.config.staged_reset_ratios=[0.3|0.3|0.1|0.1|0.1|0.1]）
  COUNT_ARGS="--criterion ${CRIT:-through}"
  PROJECT="door_${NAME}"
  LOGROOT="$WORK6/$CHAIN_REPO/logs_rl/$PROJECT"
  PASSED=0
  STALL=0
  if [ "$EVAL_FIRST" = 1 ] && [ -n "$CK" ]; then
    if [ -e "$STOP_FILE" ]; then log "止め札（${STOP_FILE}）があるので止める"; log "chain_done result=stopped_at_$NAME last_ckpt=$CK"; exit 1; fi
    log "stage=$NAME task=$TASK eval_task=$EVAL_TASK round=0（学習の前に評価） ckpt=$CK"
    eval_and_keep "$NAME" "$EVAL_TASK" 0 && PASSED=1
  fi
  for R in $(seq 1 "$ROUNDS"); do
    [ "$PASSED" -eq 1 ] && break
    if [ -e "$STOP_FILE" ]; then log "止め札（${STOP_FILE}）があるので止める"; log "chain_done result=stopped_at_$NAME last_ckpt=$CK"; exit 1; fi
    UNTIL=$(date -d "+${MIN} min" +%H:%M)
    log "stage=$NAME task=$TASK round=$R/$ROUNDS train until=$UNTIL resume=$CK${TRAIN_EXTRA:+ extra=$TRAIN_EXTRA}"
    TRAIN_LINES=$(cat "$OUTDIR/train_${NAME}.log" 2>/dev/null | wc -l | tr -d ' ')
    WORK="$WORK6" REPO="$CHAIN_REPO" ENV_NAME=doorman6 IDLE_GATE="${IDLE_GATE:-1}" PROJECT="$PROJECT" KIND="${NAME}_601" MEM_MAX="${MEM_MAX:-9G}" \
      RESUME_CKPT="$CK" ./run_train_segments.sh "$NUM_ENVS" "$UNTIL" ++env.config.task.name="$TASK" $TRAIN_EXTRA \
      >> "$OUTDIR/train_${NAME}.log" 2>&1
    NEW=$(find "$LOGROOT" -name last.pt -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
    if [ -z "$NEW" ] || [ "$NEW" = "$CK" ] || { [ -n "$CK" ] && [ "$(stat -c %Y "$NEW")" -le "$(stat -c %Y "$CK")" ]; }; then
      if tail -n +"$((TRAIN_LINES + 1))" "$OUTDIR/train_${NAME}.log" | grep -q "start seg="; then
        STALL=$((STALL + 1))
        log "stage=$NAME round=$R 学習は始まったのに重みが増えていない（${STALL}/${STALL_MAX} 回目）。反復の上限（algo.trl.num_total_batches）に届いたか、起動で落ちている。train_${NAME}.log を見る"
        tail -3 "$OUTDIR/train_${NAME}.log" | tee -a "$OUTDIR/chain.log"
        if [ "$STALL" -ge "$STALL_MAX" ]; then
          log "chain_done result=stalled_at_$NAME last_ckpt=$CK"
          exit 1
        fi
      else
        log "stage=$NAME round=$R 空くのを待っていて学習は始まらなかった（画面の前の人・他の人の GPU・札）。数えずに次の回へ"
      fi
      continue
    fi
    STALL=0
    CK="$NEW"
    eval_and_keep "$NAME" "$EVAL_TASK" "$R" && { PASSED=1; break; }
  done
  if [ "$PASSED" -ne 1 ]; then
    log "stage=$NAME FAIL（${ROUNDS} 回で合格の線に届かなかった）。ここで止める"
    log "chain_done result=fail_at_$NAME last_ckpt=$CK"
    exit 1
  fi
done
log "chain_done result=all_pass last_ckpt=$CK"
