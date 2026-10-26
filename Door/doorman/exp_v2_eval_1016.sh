#!/usr/bin/env bash
# V2（10-16）: 状態入力の生徒（45,150）に、LiDAR と測位の代わりに「画像からの推定」を渡して、閉ループで成功率を測る（パッチ 0052）。
# 条件（COND）:
#   v2          角度も姿勢も画像から推定（本番の形）
#   v2_pose     姿勢だけ画像から（角度は本当の値）＝姿勢の推定の効き目を切り分ける
#   v2_angle    角度だけ画像から（姿勢は本当の値）
#   truth       計測器の誤差を全部 0（本当の値）＝上限
#   lidar       今までの計測器の模型（LiDAR・測位の仮の誤差）＝比べる元
#   （10-20 追加。押して開ける間（区間 4 以降）だけ値の出どころを替える。パッチ 0060、door_vision_push.py）
#   e1          区間 4 以降は本当の値＝押す間の見え方を完全に直した上限
#   e2a         区間 4 に入った時の枠の位置（区間 1〜3 の推定の平均）を、本当の体の動きで運ぶ（odometry の誤差 0）
#   e2b         e2a に odometry の誤差（回ごとの距離の倍率 σ5%・旋回の片寄り σ0.3°/s・位置の酔歩 2 cm/√s）
#   e3          e2b に加え、区間 4 以降の扉の角度をライダーの計器の模型（片寄り ±2°・揺れ 1.5°・欠け 5%／手の近く 30%・斜め 75° まで）
# どの条件も、関節の目標を可動域の端から 0.05 rad 内側で切る（実機と同じ、パッチ 0051）。カメラは推定と同じ設定で描く（画像を使わない条件も同じ見た目にする）。
# カメラを描くと 16 台しか並べられないので、1 回の起動で台ごとに何回も評価する（終わった台の方策の記憶を消す、パッチ 0053）。
# 数えるのは「記録の終わりから 3000 ステップ（1 回の上限 60 秒）より前に始まった回」全部（count_success_cutoff.py。偏らない）。
#
# 使い方: EST=v2model/run1/estimator.pt PITCH=41 MINUTES=14 COND="v2 v2_pose" ./exp_v2_eval_1016.sh
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK="${CK:-ckpt/s25rl_best_it45150_ovr6/last.pt}"
CK=$(readlink -f "$CK")  # run_eval_record.sh は作業ツリーへ cd するので絶対パスに
EST="${EST:?推定のモデル（estimator.pt）を EST に}"
EST=$(readlink -f "$EST")
PITCH="${PITCH:-44}"
SEED="${SEED:-0}"
CONDS="${COND:-v2 truth}"
NUM_ENVS="${NUM_ENVS:-16}"
TAG="${TAG:-v2eval_$(date +%m%d_%H%M)}"
OUTROOT="$HOME/work_inoue/doorman/records/$TAG"
mkdir -p "$OUTROOT"
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05"
QUIET="++env.config.door_sensor.angle_noise_deg=0.0 ++env.config.door_sensor.angle_bias_deg=0.0 ++env.config.door_sensor.dropout=0.0 ++env.config.door_sensor.dropout_near_hand=0.0 ++env.config.door_sensor.loc_pos_bias_m=0.0 ++env.config.door_sensor.loc_pos_noise_m=0.0 ++env.config.door_sensor.loc_yaw_bias_deg=0.0 ++env.config.door_sensor.loc_yaw_noise_deg=0.0"
q=$(python3 -c "import math; t=math.radians($PITCH)/2; print(f'[{math.cos(t):.6f},0.0,{math.sin(t):.6f},0.0]')")
CAM="++simulator.config.cameras.enable_cameras=True ++simulator.config.cameras.camera_attached_link=torso_link \
++simulator.config.cameras.camera_pos_offset=[0.0576,0.0525,0.4299] ++simulator.config.cameras.camera_rot_offset=$q \
++simulator.config.cameras.camera_resolutions=[135,240] ++simulator.config.cameras.camera_horizontal_aperture=3.567 \
++simulator.config.cameras.camera_vertical_aperture=2.006 ++simulator.config.randomize_dome_light=${DOME:-True} \
++simulator.config.scene.env_spacing=100.0"

ODOM="++env.config.door_vision.odom.scale_sigma=0.05 ++env.config.door_vision.odom.yaw_rate_bias_deg=0.3 ++env.config.door_vision.odom.pos_rw_m=0.02"

cond_args() {
  case "$1" in
    v2) echo "$QUIET ++env.config.door_vision.estimator_path=$EST" ;;
    v2_pose) echo "$QUIET ++env.config.door_vision.estimator_path=$EST ++env.config.door_vision.truth_angle=True" ;;
    v2_angle) echo "$QUIET ++env.config.door_vision.estimator_path=$EST ++env.config.door_vision.truth_pose=True" ;;
    e1) echo "$QUIET ++env.config.door_vision.estimator_path=$EST ++env.config.door_vision.truth_from_stage=4" ;;
    e2a) echo "$QUIET ++env.config.door_vision.estimator_path=$EST ++env.config.door_vision.hold_pose_from_stage=4" ;;
    e2b) echo "$QUIET ++env.config.door_vision.estimator_path=$EST ++env.config.door_vision.hold_pose_from_stage=4 $ODOM" ;;
    e3) echo "$QUIET ++env.config.door_vision.estimator_path=$EST ++env.config.door_vision.hold_pose_from_stage=4 $ODOM ++env.config.door_vision.lidar_angle_from_stage=4" ;;
    truth) echo "$QUIET" ;;
    lidar) echo "" ;;
    *) echo "未知の条件 $1" >&2; exit 2 ;;
  esac
}

for C in $CONDS; do
  OUT="$OUTROOT/$C"
  echo "=== [$(date +%T)] cond=$C out=$OUT"
  WORK="$HOME/work_inoue/doorman6" REPO=VIRAL6 ENV_NAME=doorman6 IGNORE_PAUSE=1 IDLE_GATE="${IDLE_GATE:-1}" MAX_WAIT_MIN="${MAX_WAIT_MIN:-30}" \
    MEM_MAX="${MEM_MAX:-11G}" NEED_MARGIN_MB="${NEED_MARGIN_MB:-512}" \
    ./run_eval_record.sh "$CK" "$OUT" "$NUM_ENVS" 100000 "${MINUTES:-14}" $BASE_ARGS $CAM $(cond_args "$C") \
    ++env.config.enable_staged_reset=False ++algo.config.eval.eval_num_envs_episodes=False \
    ++algo.config.eval.reset_policy_on_done=True ++seed="$SEED" \
    ++algo.config.eval.vision_dataset_dir="$OUT/vision" ++algo.config.eval.vision_dataset_every=5 \
    ++algo.config.eval.vision_dataset_envs="$NUM_ENVS" 2>&1 | grep -aE "===|Traceback" | sed "s/^/[$C] /"
  ~/miniforge3/envs/doorman6/bin/python count_success_cutoff.py "$OUT" --json "$OUT/summary_cutoff.json" 2>&1 | sed "s/^/[$C] /"
done
echo "=== [$(date +%T)] exp_v2_eval_1016 done tag=$TAG"
