#!/usr/bin/env bash
# L5（2026-10-23）: ライダーの推定器（door_lidar_node.py）を、接続から切り離して起こす。door_run_real.py より先に。
#   THICK=<板の厚み m> ./lidar_start.sh            例: THICK=0.040 ./lidar_start.sh
#   LEVER_RIGHT=1 で把手が右の扉（今回の扉は左なので付けない）
# 推定器は REF（door_run_real.py が STAND に入った時に送る）を待ち、来たら 2.5 秒で扉と地図を作ってから 10 Hz で送る。
# 制御と CPU を取り合わないよう、コア LIDAR_CPUS（既定 6,7）に寄せる（10-16: 記録が CPU を取り合って制御の周期を乱した）
# 出力は logs/lidar_console_<日時>.log、推定の全部は logs/lidar_node_<日時>.npz（止めた時に書く）。止めるのは ./lidar_stop.sh
cd ~/door_real || exit 1
: "${THICK:?板の厚み [m] を THICK= で渡す（巻き尺。枠の原点は板の厚みの真ん中）}"
if [ -f logs/lidar.pid ] && kill -0 "$(cat logs/lidar.pid)" 2>/dev/null; then echo "[lidar] もう動いている（pid $(cat logs/lidar.pid)）"; exit 1; fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
CPUS="${LIDAR_CPUS:-6,7}"
PIN=(nice -n 5)  # 制御（door_run_real.py）より先に CPU を譲る
command -v taskset > /dev/null && PIN+=(taskset -c "$CPUS")
ARGS=(--thickness "$THICK" --iface eth0 --out "logs/lidar_node_$(date +%Y%m%d_%H%M%S).npz")
[ "${LEVER_RIGHT:-0}" = 1 ] && ARGS+=(--lever-right)
ts=$(date +%Y%m%d_%H%M%S)
setsid nohup "${PIN[@]}" python3 -u door_lidar_node.py "${ARGS[@]}" > "logs/lidar_console_$ts.log" 2>&1 < /dev/null &
echo $! > logs/lidar.pid
ln -sf "lidar_console_$ts.log" logs/lidar_console_latest.log
sleep 3
echo "[lidar] 起こした（pid $(cat logs/lidar.pid)・${PIN[*]}）。出力:"; cat "logs/lidar_console_$ts.log"
