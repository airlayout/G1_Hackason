#!/usr/bin/env bash
# 1 つのチェックポイントを、学習せずに複数のタスク（扉）で続けて評価する（段 2-3 の見通し: 等級ごとの成功率）。
# 走っている学習（stage_chain のチェーン）は札 HOLD_FILE で止めたまま通しで評価し、終わったら（失敗しても）札を外す。
# 評価ごとに札を外すと、評価の合間に学習が起動しかけて無駄になるので、通しで止める。
#
# 使い方（OMEN の ~/work_inoue/doorman で）:
#   HOLD_FILE=$HOME/work_inoue/PAUSE_DOOR_STAGE2B COUNT_ARGS="--criterion door30" \
#     ./sweep_tasks.sh <ckpt> door_en1154_g1 door_en1154_g2 ...
# 出力: sweep/<日時>/sweep.log・results.tsv（タスク・成功・率・扉 10° 以上・レバー 40° 以上・届いた区間・記録）
set -uo pipefail
BASE="$HOME/work_inoue/doorman"
PY="$HOME/miniforge3/envs/doorman6/bin/python"
HOLD_FILE="${HOLD_FILE:?走っている学習の札（例 \$HOME/work_inoue/PAUSE_DOOR_STAGE2B）}"
cd "$BASE" || exit 1

CK="${1:?ckpt}"; shift
[ -f "$CK" ] || { echo "ckpt が無い: $CK"; exit 2; }
CK="$(readlink -f "$CK")"
OUTDIR="$BASE/sweep/$(date +%m%d_%H%M)"; mkdir -p "$OUTDIR"
RESULTS="$OUTDIR/results.tsv"
printf 'task\tsuccess\trate\tdoor10\thandle40\treach_stage_ge\trecord\n' > "$RESULTS"
log() { echo "=== [$(date '+%m-%d %T')] $*" | tee -a "$OUTDIR/sweep.log"; }

trap 'rm -f "$HOLD_FILE"; log "札を外した（$HOLD_FILE）。学習は空きを確かめて続きから再開する"' EXIT
log "sweep start ckpt=$CK tasks=$* count_args=${COUNT_ARGS:-} hold=$HOLD_FILE"
# 走り始めて 5 分以内の区切りは止めるのが遅れるので、eval_from_start.sh と同じく 3 回まで待つ（2026-10-09）
PAUSED=1
for _ in 1 2 3; do
  PAUSE_FILE="$HOLD_FILE" ./pause_training.sh on | tee -a "$OUTDIR/sweep.log"
  [ "${PIPESTATUS[0]}" -eq 0 ] && { PAUSED=0; break; }
done
[ "$PAUSED" -eq 0 ] || { log "学習が止まらないので評価しない"; exit 1; }

for TASK in "$@"; do
  EVLOG="$OUTDIR/eval_${TASK}.log"
  # 評価側の札は使い捨て（評価の終わりに外す札を、学習を止めている札と別にする）
  PAUSE_FILE="$OUTDIR/pause_eval" EXTRA_ARGS="++env.config.task.name=$TASK" \
    ./eval_from_start.sh "$CK" 128 > "$EVLOG" 2>&1
  REC=$(grep -o 'out=[^ ]*' "$EVLOG" | tail -1 | cut -d= -f2)
  ROW=$("$PY" - "$REC/summary.json" <<'PYEOF' 2>/dev/null || printf 'none\t0\t-\t-\t-'
import json, sys
d = json.load(open(sys.argv[1]))
m = d["episodes_from_stage0"]
print(f"{d['success']}/{m}\t{d['success_rate']:.3f}\t{d['door_opened_10deg']}/{m}\t{d['handle_down']}/{m}\t{d['reach_stage_ge']}")
PYEOF
)
  printf '%s\t%s\t%s\n' "$TASK" "$ROW" "$REC" >> "$RESULTS"
  log "task=$TASK $(echo "$ROW" | tr '\t' ' ') record=$REC"
  sleep 61  # eval_from_start の記録のディレクトリは分単位の名前なので、同じ分に 2 つ作らない
done
log "sweep_done results=$RESULTS"
