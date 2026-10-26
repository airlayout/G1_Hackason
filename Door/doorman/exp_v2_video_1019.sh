#!/usr/bin/env bash
# V2（10-19）: 2 回目の推定（v2model/run2）で扉を開けさせ、外から見た動画（固定のカメラ、台ごとの mp4）と、
# G1 の頭のカメラの画像（毎ステップ、推定と正解つき）を記録する。成功と失敗の両方が出るように種を変えて 2 回。
# 俯角は学習に無い 41°。数え方は eval_from_start.sh と同じ（各台の最初の 1 回）。
# 使い方（OMEN の ~/work_inoue/doorman で）: SEEDS="0 1" ./exp_v2_video_1019.sh
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
EST=$(readlink -f "${EST:-v2model/run2/estimator.pt}")
PITCH="${PITCH:-41}"
q=$(python3 -c "import math; t=math.radians($PITCH)/2; print(f'[{math.cos(t):.6f},0.0,{math.sin(t):.6f},0.0]')")
BASE="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05"
QUIET="++env.config.door_sensor.angle_noise_deg=0.0 ++env.config.door_sensor.angle_bias_deg=0.0 ++env.config.door_sensor.dropout=0.0 ++env.config.door_sensor.dropout_near_hand=0.0 ++env.config.door_sensor.loc_pos_bias_m=0.0 ++env.config.door_sensor.loc_pos_noise_m=0.0 ++env.config.door_sensor.loc_yaw_bias_deg=0.0 ++env.config.door_sensor.loc_yaw_noise_deg=0.0"
CAM="++simulator.config.cameras.enable_cameras=True ++simulator.config.cameras.camera_attached_link=torso_link ++simulator.config.cameras.camera_pos_offset=[0.0576,0.0525,0.4299] ++simulator.config.cameras.camera_rot_offset=$q ++simulator.config.cameras.camera_resolutions=[135,240] ++simulator.config.cameras.camera_horizontal_aperture=3.567 ++simulator.config.cameras.camera_vertical_aperture=2.006 ++simulator.config.randomize_dome_light=True ++simulator.config.scene.env_spacing=100.0"
for S in ${SEEDS:-0 1}; do
  echo "=== [$(date +%T)] video seed=$S"; rm -rf "$HOME/work_inoue/doorman/v2video/seed$S"
  VIDEO=1 IDLE_GATE="${IDLE_GATE:-1}" COUNT_ARGS="--criterion through" MINUTES=25 \
    EXTRA_ARGS="$BASE $QUIET $CAM ++env.config.door_vision.estimator_path=$EST ++seed=$S ++algo.config.eval.vision_dataset_dir=$HOME/work_inoue/doorman/v2video/seed$S/vision ++algo.config.eval.vision_dataset_every=1 ++algo.config.eval.vision_dataset_envs=${NUM_ENVS:-8} ++algo.config.eval.vision_dataset_shard=4000" \
    ./eval_from_start.sh "$(readlink -f ckpt/s25rl_best_it45150_ovr6/last.pt)" "${NUM_ENVS:-8}" 2>&1 | grep -aE "eval_from_start out|成功|videos:|Traceback" | sed "s/^/[seed=$S] /"
done
echo "=== [$(date +%T)] exp_v2_video_1019 done"
