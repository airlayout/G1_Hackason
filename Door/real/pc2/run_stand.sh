#!/usr/bin/env bash
# 段 5 の「立つだけ」の試し（2026-10-16）。door_start.sh run_stand.sh で、接続から切り離して起こす。
# HOLD で A を押すと HOMIE だけで立つ。2 回目の A では扉の方策へ進まない（--stand-only）。
#
# 10-16 の 1 回目（扉の前・run_first.sh）で分かったことを入れてある:
#   - 吊られて 15.6° 傾いたまま A を押し、HOMIE が暴れた → 傾き 8° を越えている間は A を受け付けない（door_run_real.py の既定）
#   - 1 周期が 91 ms 空いて止まった → odom の受け取り（約 1000 回/秒）を切り、BLAS の糸を 1 本に
#     （--no-send で 20 秒ずつ比べて、最長の周期 48.4 ms → 23.1 ms）。立つだけなら odom は要らない
#   - 暴れたら早めに脱力する: 傾き 20° で脱力（シミュレータの最大 12.4°。既定の 30° は扉の試し用）
cd ~/door_real || exit 1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec python3 -u door_run_real.py --policy door_policy_it45150.npz --iface eth0 \
  --start-x=-1.00 --start-y=0.00 --start-yaw-deg=0 --door 0.94,2.05,0.99,0.064 \
  --stand-only --odom none --max-tilt-deg 20
