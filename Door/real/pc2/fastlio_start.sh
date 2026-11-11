#!/usr/bin/env bash
# 11-10 手順 L2（計画書 9.4）: FAST-LIO（逐次にマッピング・事前地図なし）と、その姿勢を扉の座標にして走らせ役へ送る口
# （door_fastlio_node.py）を起こす。走らせ役（door_run_*.py）より先に。止めるのは ./fastlio_stop.sh
#   ./fastlio_start.sh
# 走らせ役は段 0 で止まって網（c2）で扉を測り、扉の座標でのライダーの床の上の姿勢を REF2 で送る
# （door_lidar_source.request_ref(lidar_pose)・値は door_lidar_msg.pelvis_to_lidar で作る）。口は 1 秒待って 2.5 秒で床と段 0 を作り、
# 以後 FAST-LIO の出力ごと（10 Hz）に今と同じ電文（door_lidar_msg）を送る。受け口の track の形で /dog_odom とつなぐ
# FAST-LIO の設定: fastlio_g1_vox025.yaml（Door/real/fastlio_cfg_1110/ の写し。L1 でボクセル 0.25 に決めた。Mapping 班の元は 0.5 のまま）
# CPU: FAST-LIO はコア FASTLIO_CPUS（既定 4,5）、口は NODE_CPUS（既定 6,7）。制御と取り合わないよう nice 5
# 排他: Mapping 班の台本と同じ /tmp/pc2_lock（mkdir）を取る。取れなければ起こさない（C5 などが動いている）
# 出力: logs/fastlio_<日時>.log・logs/fastlio_node_console_<日時>.log・logs/fastlio_node_<日時>.npz（止めた時に書く）
# ⚠️ 機体ではまだ試していない（11-10 は記録の再生だけ）。初回は CPU（top）と電文の遅れを測る（計画書 L2 の確かめ）
# ⚠️ bash で実行する（source しない）。setsid の pid の取り方が非対話のシェルの前提（lidar_start.sh と同じ）
cd ~/door_real || exit 1
JAMMY="${G1_JAMMY_ROOT:-$HOME/jammy_ros}"
WS="${G1_C5_ROOT:-$JAMMY/ws_c5}"
CFG="${FASTLIO_CFG:-$PWD/fastlio_g1_vox025.yaml}"  # deploy_to_pc2.sh が Door/real/fastlio_cfg_1110/ から写す
# pid のファイルの pid が、名前に $2 を含むプロセスとして生きているか（pid の使い回しで別のプロセスを見ない）
alive_as() { local pid; pid=$(cat "$1" 2>/dev/null) || return 1; [ -n "$pid" ] && tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q "$2"; }
[ -f "$JAMMY/env.sh" ] || { echo "[fastlio] env.sh が無い: $JAMMY/env.sh"; exit 2; }
[ -x "$WS/bin/fastlio_mapping" ] || { echo "[fastlio] fastlio_mapping が無い: $WS/bin"; exit 2; }
[ -f "$CFG" ] || { echo "[fastlio] 設定が無い: $CFG"; exit 2; }
alive_as logs/fastlio.pid fastlio_mapping && { echo "[fastlio] FAST-LIO がもう動いている（pid $(cat logs/fastlio.pid)）"; exit 1; }
alive_as logs/fastlio_node.pid door_fastlio_node && { echo "[fastlio] 口がもう動いている（pid $(cat logs/fastlio_node.pid)）"; exit 1; }
# 自前の推定器と REF の口（127.0.0.1:47322）・コア 6,7 を取り合う。同時には動かさない
alive_as logs/lidar.pid door_lidar_node && { echo "[fastlio] 自前の推定器 door_lidar_node が動いている（./lidar_stop.sh で止めてから）"; exit 1; }
mkdir /tmp/pc2_lock 2>/dev/null || { echo "[fastlio] /tmp/pc2_lock が取れない（他の測位が動いている: $(cat /tmp/pc2_lock/owner 2>/dev/null)）"; exit 1; }
STARTED=0
# 起こし終わる前に落ちた・止められた時は、起こした分を止めてロックを返す
on_exit() {
  [ "$STARTED" = 1 ] && return 0
  alive_as logs/fastlio_node.pid door_fastlio_node && kill -TERM "$(cat logs/fastlio_node.pid)"
  alive_as logs/fastlio.pid fastlio_mapping && kill -INT "$(cat logs/fastlio.pid)"
  rm -f logs/fastlio.pid logs/fastlio_node.pid
  rm -rf /tmp/pc2_lock
  echo "[fastlio] 起こし終わる前に止まった: 起こした分を止め、/tmp/pc2_lock を返した"
}
trap on_exit EXIT
echo "door_fastlio $(date +%F_%T)" > /tmp/pc2_lock/owner
mkdir -p logs
ts=$(date +%Y%m%d_%H%M%S)

(
  # shellcheck source=/dev/null
  . "$JAMMY/env.sh"
  export ROS_DOMAIN_ID=0
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"eth0\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
  PIN=(nice -n 5)
  command -v taskset > /dev/null && PIN+=(taskset -c "${FASTLIO_CPUS:-4,5}")
  # ⚠️ LD_LIBRARY_PATH は export しない（Mapping の write_jammy_env.sh の罠 5）。ローダに --library-path で渡す
  # shellcheck disable=SC2086
  setsid nohup "${PIN[@]}" env $JAMMY_ENV "$LOADER" --library-path "$JAMMY_LIBS:$WS/lib" "$WS/bin/fastlio_mapping" \
    --ros-args --params-file "$CFG" > "logs/fastlio_$ts.log" 2>&1 < /dev/null &
  echo $! > logs/fastlio.pid
)
sleep 3
if ! alive_as logs/fastlio.pid fastlio_mapping; then
  echo "[fastlio] FAST-LIO が起きない（logs/fastlio_$ts.log）"; tail -5 "logs/fastlio_$ts.log"; exit 1
fi

export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
PIN=(nice -n 5)
command -v taskset > /dev/null && PIN+=(taskset -c "${NODE_CPUS:-6,7}")
setsid nohup "${PIN[@]}" python3 -u door_fastlio_node.py --iface eth0 --out "logs/fastlio_node_$ts.npz" \
  > "logs/fastlio_node_console_$ts.log" 2>&1 < /dev/null &
echo $! > logs/fastlio_node.pid
ln -sf "fastlio_node_console_$ts.log" logs/fastlio_node_console_latest.log
sleep 3
if ! alive_as logs/fastlio_node.pid door_fastlio_node; then
  echo "[fastlio] 口が起きない（logs/fastlio_node_console_$ts.log）"; tail -5 "logs/fastlio_node_console_$ts.log"; exit 1
fi
STARTED=1
echo "[fastlio] 起こした（FAST-LIO pid $(cat logs/fastlio.pid)・口 pid $(cat logs/fastlio_node.pid)）。口の出力:"
cat "logs/fastlio_node_console_$ts.log"
