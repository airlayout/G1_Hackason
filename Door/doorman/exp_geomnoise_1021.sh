#!/usr/bin/env bash
# 実験 B（10-21）: 生徒（反復 45,150）は扉の寸法の誤差にどこまで耐えるか。評価だけ。
#
# 生徒は扉の寸法 4 つ（幅・高さ・レバーの高さ・バックセット）を観測で受け取り、学習では回ごとに σ 1 cm の誤差を混ぜていた
# （door_state_sensor.py の geometry_noise_m=0.01）。寸法も画像（V2）で推定するなら、どこまでの誤差なら成功率が落ちないかを先に知りたい。
# 誤差は回ごと・寸法ごとに独立な片寄り（σ）。条件のほかは exp_clip2_1016.sh と同じ（256 台・立った姿勢・時間 2 倍・可動域の端から 0.05 rad）
#
# 読み方: 学習と同じ 1 cm を基準に、成功率が 3 ポイント以上落ち始める σ が「V2 に求める寸法の精度」
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
CK=ckpt/s25rl_best_it45150_ovr6/last.pt
BASE_ARGS="++env.config.task.name=door_en1154_g13_shaped_left_weight ++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60 ++env.config.clip_targets_to_limits_margin=0.05"
run() {
  echo "=== [$(date +%T)] $1"
  IDLE_GATE="${IDLE_GATE:-1}" COUNT_ARGS="--criterion through" MINUTES=20 EXTRA_ARGS="$BASE_ARGS $2" ./eval_from_start.sh "$CK" 256 2>&1 | grep -E "eval_from_start out|成功|失敗の内訳|Traceback|Error" | sed "s/^/[$1] /"
}
for g in ${GEOM_SIGMAS:-0.0 0.01 0.02 0.04}; do
  run "geom_${g}" "++env.config.door_sensor.geometry_noise_m=${g} ++seed=${SEED:-0}"
done
echo "=== [$(date +%T)] exp_geomnoise_1021 done"
