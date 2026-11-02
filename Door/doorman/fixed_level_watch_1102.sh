#!/usr/bin/env bash
# 11-02 ユーザー「打ち切りの線を締める仕組みは残し、この方策で成功率が上がるように学習を継続して」。
# 締める仕組み（termination_curriculum）は、学習中の成功の割合が 0.7 を超えるたびに線（関節の速さの上限）を締めるので、
# 重みに保存された線で測った成功率は 88〜93% のあたりに保たれる。伸びを見るため、連鎖の EVERY 回ごとに、その回の重みを
# 決まった線（LEVELS。0.25 = 5.0 rad/s・1.0 = 20 rad/s）でも測る（重みの写しの env_state_dict.termination_level だけを書き換える。学習は変えない）。
# 測る間は学習を札で止める（KEEP_PAUSE=1 で置き、2 本とも済んだら外す）。連鎖が終わるか止め札があれば終わる
# 使い方（OMEN の ~/work_inoue/doorman で）: CHAIN=chain/<日時> setsid -f nohup ./fixed_level_watch_1102.sh >> logs_fixed_level_1102.log 2>&1 < /dev/null
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
# 札は自分の PAUSE_DOOR_F3（連鎖の学習もこれを見る）。gate.sh の既定 PAUSE_DOORMAN_TRAINING はユーザーの札と同じ名前なので使わない
# （11-02 14:25〜16:09 の評価は渡し忘れて、ユーザーの札を置いては外していた）
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
PY="$HOME/miniforge3/envs/doorman6/bin/python"
CHAIN="${CHAIN:?CHAIN=chain/<日時> を渡す}"
EVERY="${EVERY:-2}"
LEVELS="${LEVELS:-0.25 1.0}"
OUT_TSV="$HOME/work_inoue/doorman/fixed_level_1102.tsv"
STOP_FILE="$HOME/work_inoue/STOP_DOOR_F3"
TASK=door_en1154_g13_shaped_left_weight
EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
[ -f "$OUT_TSV" ] || printf 'chain\tround\titer\tstored_level\tlevel\tsuccess\trate\trecord\n' > "$OUT_TSV"
done_round=0

# 引数: 評価の記録の名前（start_<日時>。eval_from_start が ckpt/<同じ名前>/ に重みを写している）・回・反復
measure() {
  local tag="$1" round="$2" iter="$3" level dst lg out stored
  for level in $LEVELS; do
    dst="ckpt/lvl${level}_${tag}"
    mkdir -p "$dst" && cp "ckpt/$tag/config.yaml" "ckpt/$tag/meta.yaml" "$dst/" || { log "写せない: ckpt/$tag"; continue; }
    stored=$("$PY" - "ckpt/$tag/last.pt" "$dst/last.pt" "$level" <<'PYEOF'
import sys, torch
c = torch.load(sys.argv[1], map_location="cpu", weights_only=False)
print(f"{float(c['env_state_dict']['termination_level']):.4f}")
c["env_state_dict"]["termination_level"] = torch.tensor(float(sys.argv[3]))
torch.save(c, sys.argv[2])
PYEOF
) || { log "書き換えに失敗: ckpt/$tag"; continue; }
    lg="logs_fixed_level_1102_r${round}_l${level}.log"
    log "回 ${round}（${iter}）を線 ${level} で測る（保存された線 ${stored}）"
    KEEP_PAUSE=1 IDLE_GATE=0 MAX_WAIT_MIN=60 EVAL_REPO=VIRAL6_fix COUNT_ARGS="--criterion through" \
      EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA" ./eval_from_start.sh "$HOME/work_inoue/doorman/$dst/last.pt" 256 > "$lg" 2>&1
    out=$(grep -o 'out=[^ ]*' "$lg" | tail -1 | cut -d= -f2)
    succ=$("$PY" -c "import json,sys; d=json.load(open(sys.argv[1])); print(f\"{d['success']}/{d['episodes_from_stage0']}\t{d['success_rate']}\")" "$out/summary.json" 2>/dev/null || printf 'none\tnone')
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$CHAIN" "$round" "$iter" "$stored" "$level" "$succ" "$out" >> "$OUT_TSV"
    log "回 ${round} 線 ${level}: ${succ} out=${out}"
    [ -n "$out" ] && [ -f "$out/summary.json" ] && nice "$PY" "$HOME/work_inoue/doorman/term_analysis.py" "$out"
  done
  ./pause_training.sh off
}

while :; do
  [ -e "$STOP_FILE" ] && { log "止め札があるので終わる"; exit 0; }
  if [ -f "$CHAIN/results.tsv" ]; then
    # 列: stage task round iter success rate ckpt record
    last=$(awk -F'\t' 'NR > 1 { r = $3; i = $4; rec = $8 } END { if (r != "") print r, i, rec }' "$CHAIN/results.tsv")
    if [ -n "$last" ]; then
      set -- $last
      round="$1" iter="$2" tag="$(basename "$3")"
      if [ "$round" -gt "$done_round" ] && [ $((round % EVERY)) -eq 0 ]; then
        measure "$tag" "$round" "$iter"
        done_round="$round"
      fi
    fi
  fi
  grep -q "chain_done" "$CHAIN/chain.log" 2>/dev/null && { log "連鎖が終わったので終わる"; exit 0; }
  sleep 60
done
