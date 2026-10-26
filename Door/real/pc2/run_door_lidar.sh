#!/usr/bin/env bash
# L5・L6（2026-10-23）: 枠の位置と扉の角度をライダーで作って、今の方策（反復 45,150）を動かす。door_start.sh で起こす。
# 先に ./lidar_start.sh で推定器を起こしておく。機体はリモコンの start まで脱力（WAIT）で待つ
#
#   DOOR=<幅,高さ,レバーの高さ,バックセット> ./door_start.sh run_door_lidar.sh
#     例: DOOR=0.95,2.04,1.00,0.064 SECS=5 ./door_start.sh run_door_lidar.sh
#   SECS     扉の方策を動かす秒（既定 5。L6 で延ばす）
#   MODE     track（毎回当て直す・既定）／still（止まって測った姿勢から odometry だけで運ぶ。比べる元）
#   CENTER   立っているときの pelvis から見たライダーの水平の位置 [m]「前 左」（STAND で巻き尺と比べて lidar_offset_from_tape.py で出す）。
#            無ければ 10-21 の旋回の値を使い、door_run_real.py が警告を出す
#
# STAND に入ると推定器へ REF が送られ、2.5 秒＋地図を作る時間のあいだ「準備中」と出る。「準備よし」が出るまで A は受け付けない
cd ~/door_real || exit 1
: "${DOOR:?扉の寸法を DOOR=幅,高さ,レバーの高さ,バックセット [m] で渡す（高さ・バックセットは巻き尺）}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1  # 10-16: BLAS の糸が多いと周期が乱れる
ARGS=(--policy door_policy_it45150.npz --iface eth0 --door-source lidar --lidar-mode "${MODE:-track}"
      --door "$DOOR" --run-seconds "${SECS:-5}" --odom dog)
if [ -n "${CENTER:-}" ]; then
  read -r CX CY <<< "$CENTER"
  ARGS+=(--lidar-center-xy "$CX" "$CY")
else
  echo "[run] ⚠️ CENTER が無い: door_run_real.py は 10-21 の旋回の値を使って警告を出す（巻き尺で合わせる 1 回目はこれでよい）"
fi
exec python3 -u door_run_real.py "${ARGS[@]}"
