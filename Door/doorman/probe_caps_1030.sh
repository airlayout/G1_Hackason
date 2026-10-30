#!/usr/bin/env bash
# 10-30: CAPS の時間の項（パッチ 0084、algo.config.caps_temporal_coef）の重みを決めるため、短く学習して記録の数字を比べる
# （ユーザー 10-30「b かな」）。F3'（f3kn4096）のいちばん新しい回の重みから、F3' と同じ引数に CAPS の重みだけを足して
# PROBE_MIN 分（既定 10）ずつ回し、重みごとに 1 行にまとめる（最初の 5 反復と最後の 5 反復の中央値）:
#   caps＝腕 14 の出力の平均の隣り合う差の 2 乗和（重みを掛ける前）・kl＝方策の変わり幅・lr＝KL で変わった実際の学習率・
#   pg と vf＝方策と価値の損失・rew＝1 回の報酬・arm＝腕の罰（雑音を含む出力の変化、1 回の合計÷30 秒）・
#   term＝打ち切りの段階の値 termination_level（倒れの率ではない。上半身の関節の速さの上限＝この値 × 20 rad/s。10-30 に読み違えた）
# 重み 0 は、入れる前の今の方策の値を測るため（0083 の罰が鳴らなかった教訓: 入れる前に、その量が今どれだけあるかを測る）
# 重みは logs_rl/door_capsprobe1030 に分けて置く（段の重みと混ぜない）。止め札は STOP_DOOR_PROBE。
# 譲る札は PAUSE_DOOR_F3（gate.sh の既定は映像の生徒の札 PAUSE_DOORMAN_TRAINING で、渡し忘れると始まらずに待ち続ける）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./probe_caps_1030.sh 0 0.1 1.0 >> logs_probe_caps_1030.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
N="${NUM_ENVS:-4096}"
REF="${REF:-f3kn4096}"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
# F3' のいちばん新しい回の重み（results.tsv の 1 列目が段の名前、4 列目が反復、7 列目が重み）
CK="${CK:-$(cat chain/*/results.tsv 2>/dev/null | awk -F'\t' -v s="$REF" '$1 == s { print $4 "\t" $7 }' | sort -n | tail -1 | cut -f2)}"
# F3' の扉と学習の引数（stage_chain.sh が chain.log に書いた「task=」「extra=」。その段の最後の行）
LINE=$(grep -h "stage=$REF task=" chain/*/chain.log | tail -1)
TASK=$(echo "$LINE" | grep -o 'task=[^ ]*' | cut -d= -f2)
EXTRA=$(echo "$LINE" | grep -o ' extra=.*' | cut -d= -f2-)
[ -n "$TASK" ] && [ -n "$EXTRA" ] || { log "${REF} の扉か学習の引数が chain.log から取れない"; exit 1; }
[ -f "$CK" ] || { log "出発点の重みが無い: $CK"; exit 1; }
CAPS_RANGE="++algo.config.caps_temporal_start=3 ++algo.config.caps_temporal_end=17"
log "probe start caps_w=$* ck=$CK task=$TASK min=${PROBE_MIN:-10} n=$N mem_max=${MEM_MAX:-12G}"

summarize() {
  python3 - "$1" <<'EOF'
import re, statistics, sys
d = sys.argv[1]
log = open(f"{d}/train.log", errors="replace").read()
keys = {
    "caps": r"loss/caps_temporal_avg:", "kl": r"policy/approxkl_avg:", "lr": r"policy/learning_rate:",
    "pg": r"loss/policy_avg:", "vf": r"loss/value_avg:", "rew": r"Mean rewards:",
    "arm": r"Mean episode rew_penalty_arm_action_change:", "term": r"Env/termination_level:",
}
out = []
for name, pat in keys.items():
    vals = [float(v) for v in re.findall(pat + r"\s+([-+0-9.eE]+)", log)]
    if not vals:
        out.append(f"{name}=none")
        continue
    head, tail = statistics.median(vals[:5]), statistics.median(vals[-5:])
    out.append(f"{name}={head:.4g}->{tail:.4g}")
iters = len(re.findall(r"Learning iteration", log))
print(f"iters={iters} " + " ".join(out) + f" dir={d}")
EOF
}

for W in "$@"; do
  TAG=$(echo "$W" | tr -d '.')
  UNTIL=$(date -d "+${PROBE_MIN:-10} min" +%H:%M)
  log "probe caps_w=$W until=$UNTIL"
  # shellcheck disable=SC2086  # EXTRA は stage_chain.sh と同じく空白で分けて渡す
  WORK="$HOME/work_inoue/doorman6" REPO=VIRAL6_fix ENV_NAME=doorman6 IDLE_GATE=1 PROJECT=door_capsprobe1030 KIND="capsprobe${TAG}_601" \
    MEM_MAX="${MEM_MAX:-12G}" NEED_MARGIN_MB=0 EXP=wbmanip/door_open_homie_29dof_state_ppo-lstm \
    PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_F3" STOP_FILE="$HOME/work_inoue/STOP_DOOR_PROBE" RESUME_CKPT="$CK" \
    ./run_train_segments.sh "$N" "$UNTIL" ++env.config.task.name="$TASK" $EXTRA ++algo.config.caps_temporal_coef="$W" $CAPS_RANGE
  D=$(ls -td bench/capsprobe"${TAG}"_601_n"${N}"_* 2>/dev/null | head -1)
  if [ -z "$D" ]; then log "caps_w=$W の出力が無い（学習が始まらなかった）"; continue; fi
  log "result caps_w=$W $(summarize "$D")"
done
log "probe_done"
