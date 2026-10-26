#!/usr/bin/env bash
# 指令の遅れの効き目（10-24）: 学習は遅れ 0。実機は状態を読んでから目標を出すまで平均 15 ms・最大 32 ms（10-23 の記録）。
# 今の方策（45,150）に遅れを入れて評価だけする。条件は exp_lidar_carry_1023b の loc0（運ばない・測位の誤差 0、3 種の平均 87.4%）と同じ
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK=ckpt/s25rl_best_it45150_ovr6/last.pt
BASE="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05 ++env.config.door_sensor.angle_noise_deg=0.2 ++env.config.door_sensor.angle_bias_deg=1.5 ++env.config.door_sensor.dropout=0.0 ++env.config.door_sensor.loc_pos_bias_m=0.0 ++env.config.door_sensor.loc_pos_noise_m=0.0 ++env.config.door_sensor.loc_yaw_bias_deg=0.0 ++env.config.door_sensor.loc_yaw_noise_deg=0.0"
run() {
  echo "=== [$(date +%T)] $1"
  KEEP_PAUSE=1 IDLE_GATE=1 COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="$BASE $2" ./eval_from_start.sh "$CK" 256 2>&1 | grep -E "eval_from_start out|成功|Traceback|Error" | sed "s/^/[$1] /"
}
run delay1 "++env.config.domain_rand.randomize_ctrl_delay=True ++env.config.domain_rand.ctrl_delay_step_range=[1,1]"
run delay02 "++env.config.domain_rand.randomize_ctrl_delay=True ++env.config.domain_rand.ctrl_delay_step_range=[0,2]"
echo "=== [$(date +%T)] exp_delay_1024 done"
