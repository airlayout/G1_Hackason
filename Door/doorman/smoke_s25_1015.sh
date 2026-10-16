#!/usr/bin/env bash
# 段 2.5（2026-10-15）の試運転: 状態入力の生徒（DAgger・映像なし、パッチ 0045）が回るかを確かめる。本番の前に 1 回だけ。
#   1. 生徒の学習が落ちずに回り、last.pt が残るか（64 台・数分）
#   2. 蒸留の環境の中で教師に運転させたとき（enforce_teacher_rollout）、教師だけの評価（反復 42,200 で 82.4%）と同じくらい
#      通れるか。教師の観測（teacher_obs）の鏡写しが効いていなければ、把手が左の扉で大きく落ちる
#   3. 生徒の評価が回るか（まだ下手なので成功は 0 に近いはず）
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./smoke_s25_1015.sh > smoke_s25_1015.log 2>&1 < /dev/null &
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
TEACHER="$HOME/work_inoue/doorman/ckpt/s24b3_pass_it42200/last.pt"
TASK=door_en1154_g13_shaped_left_weight
CAP3="++env.config.push_door_force_max=50 ++env.config.push_unlatched_force_max=50 ++rewards.reward_scales.push_door_force=0.12"
RULES="++env.config.door_through_angle_deg=70 ++env.config.door_hold_open_angle_deg=90"
MIRROR="++env.config.mirror_left_doors=True"
WINDOW="++env.config.push_door_hinge_max_deg=80 ++env.config.door_overopen_start_deg=85 ++rewards.reward_scales.door_overopen=-3.0"
EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
export EXP=wbmanip/door_open_homie_29dof_state_dagger-lstm
PROJECT=door_s25smoke
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

log "smoke_s25 start teacher=$TEACHER"
UNTIL=$(date -d "+9 min" +%H:%M)
WORK="$HOME/work_inoue/doorman6" REPO=VIRAL6 ENV_NAME=doorman6 IDLE_GATE="${IDLE_GATE:-1}" PROJECT="$PROJECT" KIND=s25smoke_601 MEM_MAX=9G \
  RESUME_CKPT="" ./run_train_segments.sh 64 "$UNTIL" ++env.config.task.name=$TASK $CAP3 $RULES $MIRROR $WINDOW \
  ++teacher_actor_path="$TEACHER"
SEG=$(ls -td bench/s25smoke_601_n64_* | head -1)
log "train log: $SEG/train.log"
grep -aE "Traceback|Error" "$SEG/train.log" | head -5
python3 "$HOME/work_inoue/doorman/train_progress.py" "$SEG/train.log" 2>/dev/null | tail -3
grep -aoE "dagger_bc_loss[^,}]*" "$SEG/train.log" | sed -n '1p;$p'
CK=$(find "$HOME/work_inoue/doorman6/VIRAL6/logs_rl/$PROJECT" -name last.pt -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
[ -n "$CK" ] || { log "smoke_s25 FAIL last.pt が無い"; exit 1; }
log "student ckpt=$CK"

log "教師に運転させた評価（64 回）"
COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA ++algo.config.enforce_teacher_rollout=True" \
  ./eval_from_start.sh "$CK" 64 > smoke_s25_teacher_eval.log 2>&1
grep -a "eval_from_start out=" smoke_s25_teacher_eval.log
grep -aE "Traceback|Error" smoke_s25_teacher_eval.log | head -3

log "生徒の評価（16 回）"
COUNT_ARGS="--criterion through" MINUTES=10 EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA" \
  ./eval_from_start.sh "$CK" 16 > smoke_s25_student_eval.log 2>&1
grep -a "eval_from_start out=" smoke_s25_student_eval.log
grep -aE "Traceback|Error" smoke_s25_student_eval.log | head -3
log "smoke_s25_done"
