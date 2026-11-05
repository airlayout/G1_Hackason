#!/usr/bin/env bash
# 11-03: 方策の回の生の点群を、方策のプロセスが動いている間ずっと録る（door_start.sh が run_stock.sh と一緒に切り離して起こす）。
# 11-03 の方策の 3 回は生の点群が無く、ライダーの追跡が崩れた原因を記録の上で決められなかった。録っておけば
# Mac で door_lidar_node.py --replay <…_li> --ref-from <lidar_node_*.npz> として、実機と同じ回から地図を作り直して再現できる。
#
#   ./rec_follow.sh <名前> <見張る pid> [上限秒]    例: ./rec_follow.sh run_stock_20261104_101500 12345 900
#   ./rec_follow.sh --check                          空き容量と記録の道具だけ確かめる（足りなければ終了コード 1）
#
# 録るもの（rec_take.sh と同じく別々のプロセス。1 つのプロセスだと /dog_odom の 1000 回/秒に追われてライダーが 3.6 Hz に落ちる）:
#   ~/mapping_runs/<日時>_<名前>_li     ライダー・頭の IMU
#   ~/mapping_runs/<日時>_<名前>_odom   /dog_odom（REC_ODOM=0 で録らない）
# 見張る pid が終わったら SIGINT で db3 と metadata を書き切らせて止める。上限秒（既定 900）で記録の道具は自分でも止まる
# （孤児の記録がディスクを埋めないように。09 月の run_stage.sh の事故）。機体には何も送らない
#
# 環境変数: REC_CPUS（既定 4,5。推定器は 6,7）・REC_MIN_FREE_GB（既定 8。上限 900 秒で約 5 GB）・REC_ODOM（既定 1）
#           REC_PY（記録の道具）・REC_RUNS（記録の置き場）
set -u
REC_PY="${REC_PY:-$HOME/mapping_tools/record_dds_to_bag.py}"
RUNS="${REC_RUNS:-$HOME/mapping_runs}"
MIN_FREE_GB="${REC_MIN_FREE_GB:-8}"

check() {
  [ -f "${REC_PY}" ] || { echo "[rec] 記録の道具が無い: ${REC_PY}"; return 1; }
  mkdir -p "${RUNS}" || return 1
  local free_gb
  free_gb=$(df -Pk "${RUNS}" | awk 'NR == 2 { printf "%d", $4 / 1048576 }')
  if [ "${free_gb}" -lt "${MIN_FREE_GB}" ]; then
    echo "[rec] ⚠️ 空きが ${free_gb} GB しかない（${MIN_FREE_GB} GB 要る）。古い記録を消すか REC=0 で録らずに起こす"
    du -sh "${RUNS}" 2>/dev/null | sed 's/^/[rec]   記録の置き場: /'
    return 1
  fi
  echo "[rec] 空き ${free_gb} GB（${RUNS}）"
}

if [ "${1:-}" = --check ]; then
  check
  exit $?
fi
NAME="${1:?名前を渡す（例 run_stock_20261104_101500）}"
PID="${2:?見張る pid を渡す}"
MAX_S="${3:-900}"
check || exit 1

PIN=(nice -n 10)  # 制御の輪（door_run_stock.py）と推定器より後に回す
command -v taskset > /dev/null && PIN+=(taskset -c "${REC_CPUS:-4,5}")
cd "$(dirname "${REC_PY}")" || exit 1
"${PIN[@]}" python3 "${REC_PY}" --duration "${MAX_S}" --name "${NAME}_li" --runs-dir "${RUNS}" \
  --topic /utlidar/cloud_livox_mid360 --topic /utlidar/imu_livox_mid360 > "${RUNS}/rec_${NAME}_li.log" 2>&1 < /dev/null &
pids=("$!")
if [ "${REC_ODOM:-1}" = 1 ]; then
  "${PIN[@]}" python3 "${REC_PY}" --duration "${MAX_S}" --name "${NAME}_odom" --runs-dir "${RUNS}" \
    --topic /dog_odom > "${RUNS}/rec_${NAME}_odom.log" 2>&1 < /dev/null &
  pids+=("$!")
fi

stop_all() {
  local p
  for p in "${pids[@]}"; do kill -INT "${p}" 2>/dev/null; done
  for _ in $(seq 1 10); do  # 5 秒待って残っていれば SIGTERM（記録の道具はどちらでも書き切って止まる）
    alive=0
    for p in "${pids[@]}"; do kill -0 "${p}" 2>/dev/null && alive=1; done
    [ "${alive}" = 0 ] && return 0
    sleep 0.5
  done
  for p in "${pids[@]}"; do kill -TERM "${p}" 2>/dev/null; done
}
trap 'stop_all; exit 0' INT TERM HUP

echo "[rec] $(date +%H:%M:%S) 録り始めた: ${NAME}（pid ${PID} が終わるまで・上限 ${MAX_S} 秒・${PIN[*]}）"
t0=$(date +%s)
while kill -0 "${PID}" 2>/dev/null; do
  alive=0
  for p in "${pids[@]}"; do kill -0 "${p}" 2>/dev/null && alive=1; done
  [ "${alive}" = 1 ] || { echo "[rec] ⚠️ 記録の道具が先に終わった（上限 ${MAX_S} 秒か、落ちた）"; break; }
  sleep 1
done
if kill -0 "${PID}" 2>/dev/null; then
  echo "[rec] $(date +%H:%M:%S) 記録だけ止まった（$(( $(date +%s) - t0 )) 秒）。pid ${PID} はまだ動いている（これより後は録っていない）"
else
  echo "[rec] $(date +%H:%M:%S) pid ${PID} が終わった（$(( $(date +%s) - t0 )) 秒）。書き切らせて止める"
fi
stop_all
wait "${pids[@]}" 2>/dev/null
grep -h -E "^session|Hz$" "${RUNS}/rec_${NAME}_li.log" "${RUNS}/rec_${NAME}_odom.log" 2>/dev/null
ls -d "${RUNS}"/*_"${NAME}"_li "${RUNS}"/*_"${NAME}"_odom 2>/dev/null | sed 's/^/[rec] 記録: /'
echo "[rec] 終わり"
