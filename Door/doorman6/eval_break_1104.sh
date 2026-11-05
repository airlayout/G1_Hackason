#!/usr/bin/env bash
# 11-04（画像だけの生徒 V3）: DAgger の連鎖（exp_vis_1104.sh）を区切りの境目で止め、最新の重みを 2 通りに評価してから、
# 同じ重みと次の種で連鎖を始め直す（11-04 ユーザー了承「止めて評価する」）。
#   ① 生徒だけで運転（学習に無い扉 seed 999・最初の区間から・256 台）
#   ② 教師に運転させる（同じ扉。配管の検算＝f3sc 単独の率に戻るか）
# 使い方（OMEN の ~/work_inoue/doorman6）: NAME=d1 ENC=<V2 の出力> setsid nohup bash eval_break_1104.sh >> vis_1104/pipeline.log 2>&1 < /dev/null &
set -uo pipefail
B="$HOME/work_inoue/doorman6"
V="$B/vis_1104"
NAME="${NAME:-d1}"
LOG="$V/pipeline.log"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }
touch "$HOME/work_inoue/STOP_DOOR_VIS"
log "評価のため連鎖 $NAME を止める（止め札）"
while pgrep -f "[e]xp_vis_1104.sh $NAME" > /dev/null; do sleep 5; done
CK=$(ls -t "$B"/VIRAL6_vis/logs_rl/"vis_$NAME"/*/*/last.pt | head -1)
LAST_SEED=$(grep -ao "区切り [0-9]* seed=[0-9]*" "$LOG" | tail -1 | grep -o "seed=[0-9]*" | cut -d= -f2)
TAG="$(date +%m%d_%H%M)"
log "評価 ck=$CK（次の種 $LAST_SEED から再開）"
bash "$B/eval_vis_1104.sh" "$CK" "$V/eval_${TAG}_student" 256 > "$V/eval_${TAG}_student.log" 2>&1
log "① 生徒: $(tail -3 "$V/eval_${TAG}_student.log" | tr '\n' ' ' | cut -c1-300)"
TEACHER_DRIVE=1 bash "$B/eval_vis_1104.sh" "$CK" "$V/eval_${TAG}_teacher" 256 > "$V/eval_${TAG}_teacher.log" 2>&1
log "② 教師: $(tail -3 "$V/eval_${TAG}_teacher.log" | tr '\n' ' ' | cut -c1-300)"
rm -f "$HOME/work_inoue/STOP_DOOR_VIS"
log "連鎖 $NAME を再開 ck=$CK seed0=$LAST_SEED"
CK="$CK" SEED0="$LAST_SEED" ENC="${ENC:?}" NUM_ENVS="${NUM_ENVS:-256}" ROUNDS="${ROUNDS:-46}" ROUND_MIN="${ROUND_MIN:-30}" \
  bash "$B/exp_vis_1104.sh" "$NAME"
log "連鎖 $NAME 終わり"
