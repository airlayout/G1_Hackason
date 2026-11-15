#!/usr/bin/env bash
# 11-12（計画 docs/plan/2026-11-12-g1-door-script-shaped-rl.md の S1・ユーザー「台本の形で学習し直す」「学習を明日の 8 時まで進めて」）:
#   段 2 の腕だけの方策を、11-12 に実機で通った台本の形で学び直す（強化学習 PPO・段の名前 s3a）。
#   出発: s2g の 6 区切り目（ckpt/start_1111_1654・学習に無い扉 87.9%）
#   構え: 正面（向き ±12°）・扉から −0.42〜−0.33 m に置く（立ち上がりで約 6 cm 後ろへ下がる＝−0.48〜−0.39 m。台本は −0.40）・
#         横はレバーの付け根から −0.40〜−0.25 m（台本は −0.325。手が肩の 8 cm 外に来る）
#   手のひらの当てる点: 今までの点（棒の中心の上）から 手前 10 cm・付け根の側へ 3 cm・下へ 2 cm（台本が触れた時の手のひら＝手の先で棒を上から押す。
#         S0 の記録: 触れた時 (−0.10, +0.025, 0.00)・下げ切った時 (−0.10, +0.01, −0.05〜−0.07)）。構える点はその 12 cm 上（台本は 10 cm 上）
#   扉の値の片寄り: 0.02 m・1.0°（ライダーでレバーを直接測る前提。s2g は網の 0.06 m・2.0°）
#   体の回りの外乱: 0〜15°・5 割の回（11-12 の R1: 回りは腕の反動でなく当たり。当たりはシムの物理が出すので弱める）
#   すき間の罰・実物の壁・脚を放す引き金 palm_press・純正の応え方の層・肩 roll の切り詰めは s2g と同じ
# 使い方（OMEN の ~/work_inoue/doorman）: setsid nohup bash s3a_1112.sh >> logs_s3a_1112.log 2>&1 < /dev/null &
#   続きから: CK=<last.pt> で起こし直す。止め札 ~/work_inoue/STOP_DOOR_F3 で区切りの境目に止まる
set -uo pipefail
cd "$HOME/work_inoue/doorman" || exit 1
log() { echo "=== [$(date "+%m-%d %T")] s3a: $*"; }
ENVF="$HOME/work_inoue/doorman6/VIRAL6_fix/gr00t/rl/envs/door/door_open_homie.py"
for k in _apply_yaw_disturb _reward_wall_gap_lagrange; do grep -qF "$k" "$ENVF" || { log "パッチが VIRAL6_fix に当たっていない（$k）"; exit 1; }; done
CK0="${CK:-$HOME/work_inoue/doorman/ckpt/start_1111_1654/last.pt}"
[ -f "$CK0" ] || { log "出発の重みが無い: $CK0"; exit 1; }
STOP="$HOME/work_inoue/STOP_DOOR_F3"
if [ -e "$STOP" ]; then rm -f "$STOP"; log "前の止め札を外した（$STOP）"; fi
WG_STATE="$HOME/work_inoue/doorman/wall_gap_s3a_1112.json"
YD="++env.config.yaw_disturb.enabled=True,++env.config.yaw_disturb.amp_deg=[0.0|15.0],++env.config.yaw_disturb.start_s=[1.0|2.5],++env.config.yaw_disturb.dur_s=[0.3|0.7],++env.config.yaw_disturb.sign=lever_side,++env.config.yaw_disturb.prob=0.5"
WG="++env.config.wall_gap.enabled=True,++env.config.wall_gap.x_m=0.06,++env.config.wall_gap.lambda_init=1.0,++env.config.wall_gap.lambda_max=${WG_LMAX:-20.0},++env.config.wall_gap.lr=0.001,++env.config.wall_gap.target_rate=0.1,++env.config.wall_gap.state_path=$WG_STATE,++rewards.reward_scales.wall_gap_lagrange=1.0"
SHAPE="++env.config.grasp_target_offset=[-0.04|0.0|-0.01],++env.config.grasp_target_offset_along_lever=0.01,++env.config.pregrasp_offset=[0.0|0.0|0.12]"
log "始め ck=$CK0 構え=${START_RANGE_S3:-[-0.42|-0.33|-0.40|-0.25|-12.0|12.0]} 形=$SHAPE"
TASK=door_en1154_g13_shaped_left_weight_realwall SKIP_EVAL=1 CK="$CK0" RELEASE=palm_press B_NAME=s3a \
  START_RANGE="${START_RANGE_S3:-[-0.42|-0.33|-0.40|-0.25|-12.0|12.0]}" DOOR_NOISE="${DOOR_NOISE_S3:-0.02|1.0}" S2_MORE="$YD,$WG,$SHAPE" \
  exec ./s2b_1109.sh
