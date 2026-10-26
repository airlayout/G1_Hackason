#!/usr/bin/env bash
# 段 0（10-17）: 扉の角度の計器（door_state_sensor.py）の仮の値を、10-16 の実機の頭のライダーで測った値に置き換えて、
# 最良の生徒 45,150 を評価し直す。条件は exp_clip_1016.sh・exp_clip2_1016.sh と同じ（256 台・立った姿勢・時間 2 倍・
# 関節の目標を可動域の端から 0.05 rad 内側で切る＝実機と同じ）。比べる元（仮の値）: 学習の扉 82.0%・seed=1 84.0%・b2 79.7%
#
# 実測（Door/real/door_lidar_angle.py、扉の 1 m 手前・吊って静止・3 回の開け閉め＋閉じたまま 150 秒）:
#   1 回ごとの揺れ σ 0.07°（閉）・0.18°（開）→ 0.2° ／ 閉じたときの読み +1.07〜1.14° → 片寄り ±1.5°
#   測れなかった回 0/2598 → 欠け 0 ／ 開け切り 86.5° で板の中心への入射角 69.5°（点 97）→ 上限 75° は変えない
#   手の近くの欠け・遅れ・測位の誤差は測っていないので変えない
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK=ckpt/s25rl_best_it45150_ovr6/last.pt
SENSOR="++env.config.door_sensor.angle_noise_deg=0.2 ++env.config.door_sensor.angle_bias_deg=1.5 ++env.config.door_sensor.dropout=0.0"
BASE_ARGS="++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 $SENSOR"
run() {
  echo "=== [$(date +%T)] $1"
  IDLE_GATE="${IDLE_GATE:-1}" COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="$BASE_ARGS $2" ./eval_from_start.sh "$CK" 256 2>&1 | grep -E "eval_from_start out|成功|失敗の内訳|Traceback|Error" | sed "s/^/[$1] /"
}
run train "++env.config.task.name=door_en1154_g13_shaped_left_weight"
run seed1 "++env.config.task.name=door_en1154_g13_shaped_left_weight ++seed=1"
run b2 "++env.config.task.name=door_en1154_g13_shaped_left_lever"
echo "=== [$(date +%T)] exp_realsensor_1017 done"
