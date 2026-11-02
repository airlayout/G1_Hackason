#!/usr/bin/env bash
# 11-02: 扉の方策を純正の脚で動かす（door_run_stock.py。腕は rt/arm_sdk・歩きは SetVelocity）。door_start.sh で起こす。
# 先に ./lidar_start.sh で推定器を起こしておく。機体は吊って綱を緩め、リモコンで通常運転（500/501）に入れておく。
# 起こした後の指示は ./stock_cmd.sh a（腕を構える）→ d（扉の方策）→ stop（立ったまま止まる）→ r（腕を戻す）→ q（終わる）
#
#   DOOR=<幅,高さ,レバーの高さ,バックセット> ./door_start.sh run_stock.sh
#     例: DOOR=0.94,2.05,0.99,0.064 SECS=15 STOP_DEG=30 ./door_start.sh run_stock.sh
#   SECS      扉の方策を動かす秒（既定 5）
#   STOP_DEG  扉がこの角度 [°] まで開いたら立ったまま止まる（無ければ止めない。通り抜けた・時間でも止まる）
#   CENTER    立っているときの pelvis から見たライダーの水平の位置 [m]「前 左」（巻き尺と比べて lidar_offset_from_tape.py で出す）
#   LEGACT    生徒の観測の脚と腰の行動: shadow（影の HOMIE・既定）／from_q（関節の角度から逆算）／zero
#   POLICY    重みの npz（既定 door_policy_f3sc_r4_it65350.npz＝f3sc の 4 回目・線 20 rad/s で 512 回 94.3%）
#   MODE      track（毎回当て直す・既定）／still
#   MAX_VX    前への上限 [m/s]（既定 0.5＝学習の頭打ち）
cd ~/door_real || exit 1
: "${DOOR:?扉の寸法を DOOR=幅,高さ,レバーの高さ,バックセット [m] で渡す}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1  # 10-16: BLAS の糸が多いと周期が乱れる
ARGS=(--policy "${POLICY:-door_policy_f3sc_r4_it65350.npz}" --iface eth0 --door-source lidar --lidar-mode "${MODE:-track}"
      --door "$DOOR" --run-seconds "${SECS:-5}" --odom dog --leg-actions "${LEGACT:-shadow}" --max-vx "${MAX_VX:-0.5}"
      --cmd-fifo logs/stock_cmd.fifo)
[ -n "${STOP_DEG:-}" ] && ARGS+=(--stop-door-deg "$STOP_DEG")
if [ -n "${CENTER:-}" ]; then
  read -r CX CY <<< "$CENTER"
  ARGS+=(--lidar-center-xy "$CX" "$CY")
else
  echo "[run] ⚠️ CENTER が無い: door_run_stock.py は 10-21 の旋回の値を使って警告を出す"
fi
exec python3 -u door_run_stock.py "${ARGS[@]}"
