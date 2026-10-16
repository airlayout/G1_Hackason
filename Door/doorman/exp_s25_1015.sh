#!/usr/bin/env bash
# 段 2.5（2026-10-15）: 状態入力の生徒を DAgger で作る（パッチ 0045・exp door_open_homie_29dof_state_dagger-lstm）。
# 教師は段 2-4b の合格（反復 42,200）。教師は _weight の扉で 211/256（82.4%）。
# 生徒は一から作る: 30 分の学習 → 256 回の評価（立った姿勢・時間 2 倍・同じ扉。教師と同じ条件）を最大 ROUNDS 回（既定 8）。
# 合格の線は「教師の 8 割」＝ 0.824 × 0.8 ＝ 0.659（計画書の段 2.5。169/256 以上）。合格したら 8 台の動画を撮って止まる。
# 最後に、合格した重み（無ければ最後の重み）を b2 の扉（重さは既定の 80〜120 kg）でも評価する（教師は 235/256）。
# 学習の引数は教師の学習（exp_s24b3_1014.sh）と同じ＋教師の重み。報酬は生徒の学習に効かないが、env の動きと教師の観測をそろえる。
# 使い方（OMEN の ~/work_inoue/doorman で）: setsid nohup ./exp_s25_1015.sh > exp_s25_1015.log 2>&1 < /dev/null &
#   続きから回すとき: CK=<生徒の last.pt> ROUNDS=<回数> setsid nohup ./exp_s25_1015.sh > ... &
#   在席の判定を切るとき（OMEN の前の人がユーザー本人のときだけ）: IDLE_GATE=0 を足す
#   学習率を替えるとき: LR=1e-4（続きから回すときは重みのファイルの値に戻るので resume_learning_rate も渡す。パッチ 0047）
#   続きの重みを先に評価するとき: EVAL_FIRST=1
# 10-15 の経過: 30 分ごとの評価が 23.0 → 51.6 → 23.0% と振れた（学習率 3e-4）。3 回目の評価は Isaac Sim が起動で固まった。
#   3 回目の重みから EVAL_FIRST=1 LR=1e-4 で回し直したが、最良の 950（52%）から 1e-4 で続けても 16%。計測器の崩しを 0 にすると 59%
#   （差の大半は見えない情報）。16:50 に 0048（腕のトルク・3 割は教師が運転）で一から回し直した
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
TEACHER="$HOME/work_inoue/doorman/ckpt/s24b3_pass_it42200/last.pt"
TASK=door_en1154_g13_shaped_left_weight
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
RULES="++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90"
MIRROR="++env.config.mirror_left_doors=True"
WINDOW="++env.config.push_door_hinge_max_deg=80,++env.config.door_overopen_start_deg=85,++rewards.reward_scales.door_overopen=-3.0"
TRAIN="$CAP3,$RULES,$MIRROR,$WINDOW,++teacher_actor_path=$TEACHER"
[ -n "${LR:-}" ] && TRAIN="$TRAIN,++algo.config.actor_learning_rate=$LR,++algo.config.resume_learning_rate=$LR"
export EXP=wbmanip/door_open_homie_29dof_state_dagger-lstm
# 0048 から学習では 3 割の台を教師が運転する（enforce_teacher_rollout）。評価では必ず切る
export EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++algo.config.enforce_teacher_rollout=False"
export MINUTES=30 MEM_MAX=9G NEED_MARGIN_MB=1024 PASS_RATE="${PASS_RATE:-0.659}" EVAL_N=256 EVAL_FIRST="${EVAL_FIRST:-0}"
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE25" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_25"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

log "exp_s25 start ck=${CK:-none} teacher=$TEACHER rounds=${ROUNDS:-8} pass_rate=$PASS_RATE lr=${LR:-default} eval_first=$EVAL_FIRST"
./stage_chain.sh "${CK:-none}" "s25:$TASK:${ROUNDS:-8}:30:through:$TRAIN"
L=$(ls -td chain/*/ | head -1)
log "chain done chain=$L"
cut -f1-6 "$L/results.tsv" | sed "s/^/  [chain] /"

KEPT=$(grep -o 'kept=[^ ]*' "$L/chain.log" | tail -1 | cut -d= -f2)
LAST=$(grep -o 'last_ckpt=[^ ]*' "$L/chain.log" | tail -1 | cut -d= -f2)
FINAL="${KEPT:+$KEPT/last.pt}"; FINAL="${FINAL:-$LAST}"
if [ -n "$FINAL" ] && [ -f "$FINAL" ]; then
  log "b2 の扉で評価し直す ckpt=$FINAL"
  COUNT_ARGS="--criterion through" EXTRA_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_lever $EVAL_EXTRA" \
    ./eval_from_start.sh "$FINAL" 256 > "$L/eval_b2_door.log" 2>&1
  grep -a "eval_from_start out=" "$L/eval_b2_door.log" | sed "s/^/  [b2 の扉] /"
fi
log "exp_s25_done"
