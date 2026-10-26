#!/usr/bin/env bash
# 段 5 の最初の試し（2026-10-16）。door_start.sh で、接続から切り離して起こす。機体はリモコンの start まで脱力（WAIT）で待つ
# 置き場所: 扉の面から 1.00 m 手前・開口の真ん中・向き −1.5°（頭のライダーの 0° の記録から）
# 扉: 幅 0.94・高さ 2.05・レバーの高さ 0.99（ライダーの記録から）・バックセット 0.064 [m]
cd ~/door_real || exit 1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1  # 10-16: BLAS の糸が多いと周期が乱れる（run_stand.sh の注を参照）
ODOM="${ODOM:-dog}"
exec python3 -u door_run_real.py --policy door_policy_it45150.npz --iface eth0 \
  --start-x=-1.00 --start-y=0.00 --start-yaw-deg=-1.5 --door 0.94,2.05,0.99,0.064 \
  --run-seconds 5 --odom "$ODOM"
