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
#   ARM_TEMP  腕のモーターがこの温度 [°C] 以上なら立ったまま止まる（既定 75。11-03 はデータを取るため 82）
#   ODOM      relay（既定。rt/dog_odom を別のプロセスで受ける。11-03: 同じプロセスだと SetVelocity が 76 ms に延びた）／dog
#   ARM_TEMP_HOLD  腕の温度が ARM_TEMP をこれだけ [s] 続けて越えたら止める（既定 1.0。11-04: 読みは腕を押し当てた山で跳ねる。札は待たない）
#   RPC_TIMEOUT  純正の RPC の返事を待つ上限 [s]（既定 1.0）。11-04: 1.0 だと返事が 1 回落ちるたびに次の SetVelocity を 1 秒送れず、
#             効き目 0.5 秒が切れて純正が減速した（手順 7.5 で 0.25 を測る）。起動の時の FSM の読み取りにも効く
#   LOCO      relay（既定。SetVelocity を別のプロセス loco_relay.py で送る）／inproc（11-04 までの作り）。11-04: 同じプロセスでは
#             返事が 1/4〜1/2 落ちた（3104）。同じ失敗を子のプロセスへ移して直した例がある（4paradigm/phanthymotus-driver 24274c5）
#   START     「前 左 [向き°]」を渡すと扉の値をライダーでなく巻き尺＋脚の odom で出す（--door-source tape）。
#             前は扉の板の面から pelvis まで（手前は負＝−(足先の巻き尺 + 0.123)）、左は開口の中心から pelvis まで（扉を向いて左が正）。
#             11-09: 画像の網は扉が画面から外れると壁を扉と取り違えた → 教師を巻き尺の値で動かす（例 START="-0.883 -0.12"）
cd ~/door_real || exit 1
: "${DOOR:?扉の寸法を DOOR=幅,高さ,レバーの高さ,バックセット [m] で渡す}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1  # 10-16: BLAS の糸が多いと周期が乱れる
SRC=(--door-source lidar --lidar-mode "${MODE:-track}")
if [ -n "${START:-}" ]; then
  read -r SX SY SYAW <<< "$START"
  SRC=(--door-source tape --start-x "$SX" --start-y "$SY" --start-yaw-deg "${SYAW:-0}")
fi
ARGS=(--policy "${POLICY:-door_policy_f3sc_r4_it65350.npz}" --iface eth0 "${SRC[@]}"
      --door "$DOOR" --run-seconds "${SECS:-5}" --odom "${ODOM:-relay}" --leg-actions "${LEGACT:-shadow}" --max-vx "${MAX_VX:-0.5}" --arm-temp-stop "${ARM_TEMP:-75}"
      --rpc-timeout "${RPC_TIMEOUT:-1.0}" --arm-temp-hold "${ARM_TEMP_HOLD:-1.0}" --loco "${LOCO:-relay}"
      --cmd-fifo logs/stock_cmd.fifo)
[ -n "${STOP_DEG:-}" ] && ARGS+=(--stop-door-deg "$STOP_DEG")
if [ -n "${CENTER:-}" ]; then
  read -r CX CY <<< "$CENTER"
  ARGS+=(--lidar-center-xy "$CX" "$CY")
elif [ -z "${START:-}" ]; then
  echo "[run] CENTER が無い: ライダーの取付は URDF の順運動学だけ（11-10 L3）"
fi
exec python3 -u door_run_stock.py "${ARGS[@]}"
