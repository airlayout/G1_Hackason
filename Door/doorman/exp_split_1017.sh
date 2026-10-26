#!/usr/bin/env bash
# 段 0（10-17）: 実測の角度の値（exp_realsensor_1017.sh）で 79.7%、誤差を全部 0 で 88.3%（学習の扉）。
# 残りの差が「枠の位置と向き（測位）の誤差」と「手が板の近くにあるときの欠け」のどちらから来るかを、1 つずつ 0 にして分ける。
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK=ckpt/s25rl_best_it45150_ovr6/last.pt
SENSOR="++env.config.door_sensor.angle_noise_deg=0.2 ++env.config.door_sensor.angle_bias_deg=1.5 ++env.config.door_sensor.dropout=0.0"
LOC0="++env.config.door_sensor.loc_pos_bias_m=0.0 ++env.config.door_sensor.loc_pos_noise_m=0.0 ++env.config.door_sensor.loc_yaw_bias_deg=0.0 ++env.config.door_sensor.loc_yaw_noise_deg=0.0"
HAND0="++env.config.door_sensor.dropout_near_hand=0.0"
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 $SENSOR"
run() {
  echo "=== [$(date +%T)] $1"
  IDLE_GATE="${IDLE_GATE:-1}" COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="$BASE_ARGS $2" ./eval_from_start.sh "$CK" 256 2>&1 | grep -E "eval_from_start out|成功|失敗の内訳|Traceback|Error" | sed "s/^/[$1] /"
}
run loc0 "$LOC0"
run hand0 "$HAND0"
echo "=== [$(date +%T)] exp_split_1017 done"
