#!/usr/bin/env bash
# 11-11（計画 docs/plan/2026-11-11-g1-door-stage2-decision.md の R1・R2・R4）: 扉から離れた何も無い所で、11-11 の段 2 の腕の送り値を
# そのまま流し、純正の脚で立ったまま体が回るか（反動が主因か）を見る（door_run_stock.py --replay-arm）。door_start.sh で起こす。
# FAST-LIO・目の網は要らない（扉の値は使わない。巻き尺の口に置き場所だけ渡す）。
# 機体は吊って綱を緩め、リモコンで通常運転（500/501）に入れ、周り 1 m に何も無い所に立たせておく。
# 指示: ./stock_cmd.sh a（腕を借りて再生の最初の姿勢へ）→ d（再生）→ 終わると立ったまま止まる → r（腕を返す）→ q
#
#   ./door_start.sh run_replay.sh
#     例: REPLAY=logs/door_stock_20261111_075139.npz ./door_start.sh run_replay.sh                 （R1: 4 回目を元の速さで）
#         REPLAY=logs/door_stock_20261111_075139.npz SPEED=0.5 ./door_start.sh run_replay.sh       （R2: 半分の速さ）
#         REPLAY=logs/door_stock_20261111_075139.npz WAIST_GAIN=1 ./door_start.sh run_replay.sh    （R4: 腰 yaw の打ち消しを足す）
#   REPLAY      流す記録（door_stock_*.npz。既定 logs/door_stock_20261111_075139.npz＝4 回目）
#   SEGMENT     policy（既定・段 2 の方策の区間）か stage2（腕の構え 2.5 秒を含む）
#   SPEED       速さの倍率（既定 1.0）
#   MAX_YAW     再生を始めてからの体の回りがこの角度 [°] を越えたら立ったまま止まる（既定 45。R1 の判定 10° が見えるように大きめ）
#   WAIST_GAIN  腰 yaw の打ち消しの倍率（既定 0＝切）。WAIST_MAX [°]（既定 35）・WAIST_RATE [rad/s]（既定 1.0）
#   POLICY      腕の強さ・初めの姿勢・積み上げの守りを読む npz（既定 door_policy_s2g_it80700.npz。方策の出力は使わない）
#   NOSEND=1    腕も歩きも送らない（記録と表示だけ）
#   ARM_TEMP・ARM_TEMP_HOLD・RPC_TIMEOUT・LOCO は run_staged.sh と同じ
cd ~/door_real || exit 1
REPLAY="${REPLAY:-logs/door_stock_20261111_075139.npz}"
POLICY="${POLICY:-door_policy_s2g_it80700.npz}"
for f in "$REPLAY" "$POLICY"; do
  [ -f "$f" ] || { echo "[run] 無い: $f（Mac から deploy_to_pc2.sh で配る）"; exit 1; }
done
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1  # 10-16: BLAS の糸が多いと周期が乱れる
ARGS=(--policy "$POLICY" --iface eth0 --door-source tape --start-x=-1.0 --start-y=0.0 --door "${DOOR:-0.94,2.05,0.99,0.064}"
      --odom none --replay-arm "$REPLAY" --replay-segment "${SEGMENT:-policy}" --replay-speed "${SPEED:-1.0}"
      --replay-max-yaw-deg "${MAX_YAW:-45}" --waist-comp-gain "${WAIST_GAIN:-0}" --waist-comp-max-deg "${WAIST_MAX:-35}"
      --waist-comp-rate "${WAIST_RATE:-1.0}" --arm-temp-stop "${ARM_TEMP:-75}" --arm-temp-hold "${ARM_TEMP_HOLD:-1.0}"
      --rpc-timeout "${RPC_TIMEOUT:-1.0}" --loco "${LOCO:-relay}" --cmd-fifo logs/stock_cmd.fifo)
# 足の確かめ（--load-check）は既定のまま入れる: 綱が張って体を支えていると回りが抑えられ、R1 の判定が狂う
[ "${NOSEND:-0}" = 1 ] && ARGS+=(--no-send)
exec python3 -u door_run_stock.py "${ARGS[@]}"
