#!/usr/bin/env bash
# 腕だけの空振り（2026-10-23）: 強化学習の方策（反復 45,150）の腕の出力だけを使う。脚と腰は HOLD の姿勢のまま PD で固く保ち、
# HOMIE は使わない（STAND を通らない）。見立ての扉の位置を数字で渡す（機体は歩かないので動かない）。door_start.sh で起こす
#
#   ./door_start.sh run_arms_dry.sh
#   見立ての位置: シミュレータで手がレバーに当たる所の中央値（x −0.38・y +0.33 m・向き +2°。256 台、10-23）
#   扉の寸法: 幅 0.95・高さ 2.18（実測 3.70 を学習の範囲の上限に切った）・レバーの高さ 1.00・バックセット 0.05 [m]、把手は左（鏡写し）
#
# 流れ（リモコン）: start → HOLD（固い姿勢）→ 両足を床に着ける → A で扉の方策（腕だけ）を SECS 秒 → POST（脚は固いまま・腕はその場）
# 止め方: B（脚は固いまま止まる）／L2+B（脱力）。機体の前 0.8 m 以内に人と物を置かない（腕が前へ伸びる）
cd ~/door_real || exit 1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec python3 -u door_run_real.py --policy door_policy_it45150.npz --iface eth0 \
  --door-source tape --start-x="${START_X:--0.38}" --start-y="${START_Y:-0.33}" --start-yaw-deg="${START_YAW:-2}" --odom none \
  --door "${DOOR:-0.95,2.18,1.00,0.05}" --run-seconds "${SECS:-5}" --legs hold
