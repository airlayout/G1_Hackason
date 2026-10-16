#!/usr/bin/env bash
# 段 2.5（2026-10-15 夕）: 状態入力の生徒を強化学習（PPO）で磨く（パッチ 0049・exp door_open_homie_29dof_state_ppo-lstm）。
# 真似（DAgger）だけだと、950 反復の 52%（別の扉の組で 50%）が天井で、続けると 15〜23% に崩れた。計測器の崩しを 0 にしても 59%。
# 差の大半は生徒に見えない情報（扉の重さ・手の力・レバーの角度）で、教師の押し方を平均でしか真似できない。
# 強化学習なら、生徒が見えている値だけで成功するように磨ける。出発は 950 の生徒（actor）＋教師の評価係（critic）、
# 腕の関節のトルクの入力を重み 0 で足す（make_state_ppo_ckpt.py）。報酬・env・学習の引数は教師の学習と同じ。
# 30 分の学習 → 256 回の評価を最大 ROUNDS 回。合格は教師の 8 割＝0.659。合格したら 8 台の動画を撮って止まる。
# 最後に、合格した重み（無ければ最後の重み）を別の扉の組（seed=1）と b2 の扉でも評価する（選んだ回のまぐれでないか）。
# 使い方（OMEN の ~/work_inoue/doorman で）: IDLE_GATE=0 setsid nohup ./exp_s25rl_1015.sh > exp_s25rl_1015.log 2>&1 < /dev/null &
#   続きから: CK=<last.pt> ROUNDS=<回数> を足す
# 10-15 19:39 に 3 回目（反復 43,650）で 189/256（73.8%）＝合格。seed=1 で 76.6%、b2 の扉で 68.0%（教師 91.8%）。
#   その後は「段階を踏む」（ユーザーの提案）の比べ: CK=ckpt/s25rl_pass_it43650/last.pt ROUNDS=2 PASS_RATE=1.1 で
#   NAME=s25cur TRAIN_ADD=<区間の割合> と NAME=s25base（割合は既定）を同じ回数ずつ
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
PY="$HOME/miniforge3/envs/doorman6/bin/python"
TEACHER="$HOME/work_inoue/doorman/ckpt/s24b3_pass_it42200/last.pt"
STUDENT="$HOME/work_inoue/doorman/ckpt/start_1015_1428/last.pt"  # DAgger の生徒・反復 950（52%）
START="$HOME/work_inoue/doorman/ckpt/s25rl_start/last.pt"
TASK=door_en1154_g13_shaped_left_weight
CAP3="++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12"
RULES="++env.config.door_through_angle_deg=70,++env.config.door_hold_open_angle_deg=90"
MIRROR="++env.config.mirror_left_doors=True"
# 押し切りの減点は OVR で替える（10-15 夜: 強化学習を続けると押し切りが 44〜45 回に増えたので −6.0 を試す）
WINDOW="++env.config.push_door_hinge_max_deg=80,++env.config.door_overopen_start_deg=85,++rewards.reward_scales.door_overopen=${OVR:--3.0}"
TRAIN="$CAP3,$RULES,$MIRROR,$WINDOW"
# 段階を踏む（ユーザーの提案 10-15）: 弱い区間から始める回を増やす。TRAIN_ADD に足す引数（, 区切り・リストの中は |）
#   例: TRAIN_ADD="++env.config.staged_reset_ratios=[0.3|0.3|0.1|0.1|0.1|0.1]"（区間 0〜5 から始める割合。既定は 0.5・0.1 × 5）
[ -n "${TRAIN_ADD:-}" ] && TRAIN="$TRAIN,$TRAIN_ADD"
export EXP=wbmanip/door_open_homie_29dof_state_ppo-lstm
export EVAL_EXTRA="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60"
export MINUTES=30 MEM_MAX=9G NEED_MARGIN_MB=1024 PASS_RATE="${PASS_RATE:-0.659}" EVAL_N=256 EVAL_FIRST=0
export PAUSE_FILE="$HOME/work_inoue/PAUSE_DOOR_STAGE25" STOP_FILE="$HOME/work_inoue/STOP_DOOR_CHAIN_25"
log() { echo "=== [$(date '+%m-%d %T')] $*"; }

if [ -z "${CK:-}" ]; then
  mkdir -p "$(dirname "$START")"
  "$PY" make_state_ppo_ckpt.py "$STUDENT" "$TEACHER" "$START" --std-scale "${STD_SCALE:-0.5}" || { log "出発の重みを作れなかった"; exit 1; }
  CK="$START"
fi
log "exp_s25rl start ck=$CK rounds=${ROUNDS:-8} pass_rate=$PASS_RATE train_add=${TRAIN_ADD:-none} overopen=${OVR:--3.0}"
# 段の名前（学習の置き場所 logs_rl/door_<名前>）。段階を踏む比べでは NAME=s25cur・s25base のように分ける
./stage_chain.sh "$CK" "${NAME:-s25rl}:$TASK:${ROUNDS:-8}:30:through:$TRAIN"
L=$(ls -td chain/*/ | head -1)
log "chain done chain=$L"
cut -f1-6 "$L/results.tsv" | sed "s/^/  [chain] /"

KEPT=$(grep -o 'kept=[^ ]*' "$L/chain.log" | tail -1 | cut -d= -f2)
LAST=$(grep -o 'last_ckpt=[^ ]*' "$L/chain.log" | tail -1 | cut -d= -f2)
FINAL="${KEPT:+$KEPT/last.pt}"; FINAL="${FINAL:-$LAST}"
if [ -n "$FINAL" ] && [ -f "$FINAL" ]; then
  log "別の扉の組（seed=1）で評価し直す ckpt=$FINAL"
  COUNT_ARGS="--criterion through" EXTRA_ARGS="++env.config.task.name=$TASK $EVAL_EXTRA ++seed=1" \
    ./eval_from_start.sh "$FINAL" 256 > "$L/eval_seed1.log" 2>&1
  grep -a "eval_from_start out=" "$L/eval_seed1.log" | sed "s/^/  [seed=1] /"
  log "b2 の扉で評価し直す ckpt=$FINAL"
  COUNT_ARGS="--criterion through" EXTRA_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_lever $EVAL_EXTRA" \
    ./eval_from_start.sh "$FINAL" 256 > "$L/eval_b2_door.log" 2>&1
  grep -a "eval_from_start out=" "$L/eval_b2_door.log" | sed "s/^/  [b2 の扉] /"
fi
log "exp_s25rl_done"
