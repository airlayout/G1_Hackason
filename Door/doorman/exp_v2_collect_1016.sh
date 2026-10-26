#!/usr/bin/env bash
# V2（10-16）: 頭のカメラの画像と扉の状態の正解を集める（推定のモデルの学習用、パッチ 0052 の vision_dataset）。
# 機体を動かすのは最良の状態入力の生徒（45,150）。評価は段 2.5 と同じ条件（立った姿勢・時間 2 倍・学習の扉）で、
# 各台の 1 回目で止めず EPISODES 回まで回す（記録の多さのため）。
#
# カメラ: 頭の D435i の IR に合わせる（視野 87°×58° を 240×135 で）。取付の位置は DoorMan の d435_link（torso_link から）。
# 俯角は実機でセッションごとに 32〜45° 変わった（2026-09-16）ので、俯角を変えて何回かに分けて集める。
# ドームライト（照明）は乱択する（Kit の BehaviorScript、パッチ 0011）。
#
# ⚠️ 俯角ごとに種（SEEDS）を変える。同じ種だと機体の動きが全く同じになり、写る姿勢が増えない（10-16 に 3 つの俯角で
#    15,424 枚ずつ・同じ経路になった）。
#
# 使い方（OMEN の ~/work_inoue/doorman で）: PITCHES="32 38 44 50" SEEDS="0 1 2 3" EPISODES=96 ./exp_v2_collect_1016.sh
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="${CK:-ckpt/s25rl_best_it45150_ovr6/last.pt}"
CK=$(readlink -f "$CK")  # run_eval_record.sh は作業ツリーへ cd するので絶対パスに
OUTROOT="${OUTROOT:-$HOME/work_inoue/doorman/v2data}"
PITCHES="${PITCHES:-32 38 44 50}"
SEEDS="${SEEDS:-0 1 2 3}"
read -r -a SEED_ARR <<< "$SEEDS"
NUM_ENVS="${NUM_ENVS:-16}"
EPISODES="${EPISODES:-96}"
EVERY="${EVERY:-5}"
MINUTES="${MINUTES:-40}"
EVAL_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.enable_staged_reset=False"

cam_args() {  # $1 = 俯角 [deg]
  local q
  q=$(python3 -c "import math; t=math.radians($1)/2; print(f'[{math.cos(t):.6f},0.0,{math.sin(t):.6f},0.0]')")
  echo "++simulator.config.cameras.enable_cameras=True ++simulator.config.cameras.camera_attached_link=torso_link \
++simulator.config.cameras.camera_pos_offset=[0.0576,0.0525,0.4299] ++simulator.config.cameras.camera_rot_offset=$q \
++simulator.config.cameras.camera_resolutions=[135,240] ++simulator.config.cameras.camera_horizontal_aperture=3.567 \
++simulator.config.cameras.camera_vertical_aperture=2.006 ++simulator.config.randomize_dome_light=${DOME:-True} \
++simulator.config.scene.env_spacing=100.0"
}

i=0
for P in $PITCHES; do
  S="${SEED_ARR[$i]:-$i}"; i=$((i + 1))
  OUT="$OUTROOT/pitch$P"
  mkdir -p "$OUT"
  echo "=== [$(date +%T)] collect pitch=$P seed=$S out=$OUT"
  WORK="$HOME/work_inoue/doorman6" REPO=VIRAL6 ENV_NAME=doorman6 IGNORE_PAUSE=1 IDLE_GATE="${IDLE_GATE:-1}" MAX_WAIT_MIN="${MAX_WAIT_MIN:-30}" \
    MEM_MAX="${MEM_MAX:-11G}" NEED_MARGIN_MB="${NEED_MARGIN_MB:-512}" \
    ./run_eval_record.sh "$CK" "$OUT/records" "$NUM_ENVS" "$EPISODES" "$MINUTES" $EVAL_ARGS $(cam_args "$P") \
    ++algo.config.eval.vision_dataset_dir="$OUT" ++algo.config.eval.vision_dataset_every="$EVERY" \
    ++algo.config.eval.vision_dataset_envs="$NUM_ENVS" ++seed="$S" ${EXTRA_ARGS:-} 2>&1 | grep -aE "===|Traceback|Error" | sed "s/^/[pitch=$P] /"
  N=$(ls "$OUT"/shard_*.npz 2>/dev/null | wc -l)
  echo "=== [$(date +%T)] pitch=$P shards=$N $(grep -a 'vision dataset' "$OUT/records/eval.log" | tail -1)"
done
echo "=== [$(date +%T)] exp_v2_collect_1016 done"
