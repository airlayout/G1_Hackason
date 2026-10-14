#!/usr/bin/env bash
# 段 1 の計測と段 2-1 の起動確認を順番に流す（同時に 2 本は RAM 15 GiB に入らない）。
# 各計測は run_bench.sh が「機械が空くまで」待ってから始める。
#
# 計測済み（2026-10-06）:
#   - 教師 256 台: 2,689 歩/秒・VRAM 4.5 GB・RSS 5.4 GB（起動時の山 6.5 GB）
#   - 教師 512 台: 19:00 に別に流した（bench/teacher_n512_1006_1900）
set -uo pipefail
BASE="$HOME/work_inoue/doorman"
cd "$BASE"

# 段 2-1: 指の無い 29 関節版が起動して学習が回るか（VIRAL29）
REPO=VIRAL29 EXP=wbmanip/door_open_homie_29dof_lstm KIND=t29 MEM_MAX=8G ./run_bench.sh 256 12

# 段 1: 映像の生徒（カメラあり）。教師は 256 台の計測の last.pt（描画と VRAM を測るだけなので質は問わない）
for N in 16 32 64; do
  MEM_MAX=8G EXP=wbmanip/door_open_homie_dagger-lstm KIND=student \
  BASE_ARGS="++algo.config.num_steps_per_env=32 ++algo.config.actor.backbone.vision_module.module_config_dict.layer_config.trainable=True ++algo.config.obj_pred_loss_coef=1.0 ++algo.config.actor.running_mean_std=True ++algo.config.num_learning_epochs=1 ++algo.config.num_mini_batches=$N ++algo.config.teacher_rollout_ratio=0.3 ++teacher_actor_path=$BASE/bench/teacher256_last.pt" \
    ./run_bench.sh "$N" 9
done
echo QUEUE_DONE
