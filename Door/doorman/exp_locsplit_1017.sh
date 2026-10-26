#!/usr/bin/env bash
# 段 0（10-17）: 測位の誤差（枠の位置と向き）のうち、どれが成功率を下げているか。exp_split_1017.sh で
# 測位の誤差を全部 0 にすると 79.7 → 87.1%（学習の扉）。ここでは 1 つだけ残して、ほかを 0 にする。
#   posb3・posb1.5  位置の片寄り（回ごと・σ）だけ 3 cm・1.5 cm
#   yawb1・yawb0.5  向きの片寄り（回ごと・σ）だけ 1°・0.5°
#   noise           揺れ（ステップごと・σ）だけ 1 cm・0.3°（仮の値のまま）
# 扉の角度は実測の値（exp_realsensor_1017.sh と同じ）。条件は学習の扉・256 回・目標を 0.05 rad 内側で切る
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK=ckpt/s25rl_best_it45150_ovr6/last.pt
SENSOR="++env.config.door_sensor.angle_noise_deg=0.2 ++env.config.door_sensor.angle_bias_deg=1.5 ++env.config.door_sensor.dropout=0.0"
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 $SENSOR"
loc() {  # 位置の片寄り, 位置の揺れ, 向きの片寄り, 向きの揺れ
  echo "++env.config.door_sensor.loc_pos_bias_m=$1 ++env.config.door_sensor.loc_pos_noise_m=$2 ++env.config.door_sensor.loc_yaw_bias_deg=$3 ++env.config.door_sensor.loc_yaw_noise_deg=$4"
}
run() {
  echo "=== [$(date +%T)] $1"
  IDLE_GATE="${IDLE_GATE:-1}" COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="$BASE_ARGS $2" ./eval_from_start.sh "$CK" 256 2>&1 | grep -E "eval_from_start out|成功|失敗の内訳|Traceback|Error" | sed "s/^/[$1] /"
}
run posb3 "$(loc 0.03 0.0 0.0 0.0)"
run yawb1 "$(loc 0.0 0.0 1.0 0.0)"
run noise "$(loc 0.0 0.01 0.0 0.3)"
run posb1.5 "$(loc 0.015 0.0 0.0 0.0)"
run yawb0.5 "$(loc 0.0 0.0 0.5 0.0)"
echo "=== [$(date +%T)] exp_locsplit_1017 done"
