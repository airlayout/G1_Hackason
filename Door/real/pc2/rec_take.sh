#!/usr/bin/env bash
# 段 1（2026-10-21）: 脚の odometry を測る記録を 1 本録る。接続が切れても止まらないよう切り離して起こし、終わるまで待って頻度を出す。
#
#   ./rec_take.sh <名前> <秒> [ir]      例: ./rec_take.sh odom_1021_01 60 / ./rec_take.sh ir_1021_01 240 ir
#   3 つ目に ir を付けると、頭の赤外線カメラも録る（rec_ir.py。<名前>_ir/ に画像と関節・IMU）
#
# ⚠️ 1 つのプロセスで 3 つを録ると、/dog_odom の 1000 回/秒の受け取りに追われてライダーが 3.6 Hz に落ちる（10-21 実測。
#    ライダーだけなら 9.97 Hz）。だから 3 つのプロセスに分ける。どれも PC2 の time.time_ns() で時刻を付けるので突き合わせられる
#      <名前>_li     ライダー・IMU（FAST_LIO を掛け直す元）
#      <名前>_odom   /dog_odom
#      <名前>_remote.log  リモコンが動いた瞬間（watch_remote.py）
# 記録するだけで、機体には何も送らない
set -u
NAME="${1:?名前を渡す（例 odom_1021_01）}"
SECS="${2:?秒を渡す}"
REC=~/mapping_tools/record_dds_to_bag.py
LOG=~/mapping_runs/logs_1021
mkdir -p "$LOG"
cd ~/mapping_tools || exit 1

nohup setsid -w python3 "$REC" --duration "$SECS" --name "${NAME}_li" \
  --topic /utlidar/cloud_livox_mid360 --topic /utlidar/imu_livox_mid360 > "$LOG/${NAME}_li.log" 2>&1 < /dev/null &
p1=$!
nohup setsid -w python3 "$REC" --duration "$SECS" --name "${NAME}_odom" --topic /dog_odom > "$LOG/${NAME}_odom.log" 2>&1 < /dev/null &
p2=$!
nohup setsid -w python3 ~/door_real/watch_remote.py "$SECS" > "$LOG/${NAME}_remote.log" 2>&1 < /dev/null &
p3=$!
pids=("$p1" "$p2" "$p3")
if [ "${3:-}" = ir ]; then
  nohup setsid -w python3 ~/door_real/rec_ir.py ~/mapping_runs/"$(date +%Y%m%dT%H%M%S)_${NAME}_ir" "$SECS" > "$LOG/${NAME}_ir.log" 2>&1 < /dev/null &
  pids+=("$!")
fi
echo "[rec] $(date +%H:%M:%S) 録り始めた: ${NAME}（${SECS} 秒）"
wait "${pids[@]}"
echo "[rec] $(date +%H:%M:%S) 終わり"
grep -h -E "^session|Hz$" "$LOG/${NAME}_li.log" "$LOG/${NAME}_odom.log"
echo "[rec] リモコンが動いた行: $(grep -c "keys=" "$LOG/${NAME}_remote.log")"
[ -f "$LOG/${NAME}_ir.log" ] && tail -1 "$LOG/${NAME}_ir.log"
