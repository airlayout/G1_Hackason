#!/usr/bin/env bash
# 案 A（2026-10-26）: PC2 の上で xr_teleoperate を切り離して起こす。Mac の scripts/pc2_teleop.sh start から呼ばれる。
#
#   ./teleop_start.sh <課題名>            例: ./teleop_start.sh door_room4_1027
#   TELEOP_DRY=1 ./teleop_start.sh <課題名>   起こさずに、確かめと起動のコマンドだけを出す
#
# PC2 に tmux も screen も無いので、ssh が切れても遠隔操作が止まらないよう setsid nohup で切り離す。
# 操作（r・s・q）は同じフォルダの teleop_ipc.py で IPC に送る（--ipc。キーボードは使わない）。
# 機体の制御（DDS）は eth0 だけに縛る。縛らないと無線の側にも DDS が出て、機体の中の通信が壊れうる（2026-09）。
set -u
TASK="${1:?課題名を渡す（例 door_room4_1027）}"
PY="${PC2_TV_PY:-$HOME/miniforge3/envs/tv/bin/python}"
XR="${PC2_XR_DIR:-$HOME/xr_teleoperate}"
DATA="${PC2_TELEOP_DATA:-$HOME/teleop_data}"
LOGDIR="${PC2_TELEOP_LOGS:-$HOME/teleop_logs}"
IFACE="${PC2_DDS_IFACE:-eth0}"
FREQ="${TELEOP_FREQ:-30}"
WAIT_S="${TELEOP_WAIT_S:-120}"   # 29DoF の初回は逆運動学のモデルづくりで 1 分ほどかかる。ログは画面の幅で折り返されるので短い文字列で待つ（10-27）

die() { echo "[start] ✗ $*"; exit 1; }

[[ "$TASK" =~ ^[A-Za-z0-9_-]+$ ]] || die "課題名は英数字と _ - だけ: ${TASK}"
if pgrep -f '^[^ ]*python[^ ]* (-u )?teleop_hand_and_arm.py' > /dev/null; then
  die "もう動いている: $(pgrep -af '^[^ ]*python[^ ]* (-u )?teleop_hand_and_arm.py' | head -1)"
fi
[ -x "$PY" ] || die "Python が無い: ${PY}（PC2 に遠隔操作の環境を作るのが先）"
MAIN="$XR/teleop/teleop_hand_and_arm.py"
[ -f "$MAIN" ] || die "xr_teleoperate が無い: ${XR}"
# 修正が当たっていないと、記録に歩きが残らない・Quest が切れても歩き続ける・押し込みで落ちる
for m in "PATCH(record-loco-cmd)" "PATCH(stick-pause)" "PATCH(xr-stale-stop)" "PATCH(xr-stale-log)" "PATCH(safe-exit)" \
         "PATCH(record-arm-effort)" "PATCH(state-watchdog)" "PATCH(go-home-release)" "PATCH(arm-tau-getter)"; do
  f="$MAIN"
  case "$m" in "PATCH(state-watchdog)"|"PATCH(go-home-release)"|"PATCH(arm-tau-getter)") f="$XR/teleop/robot_control/robot_arm.py" ;; esac
  grep -qF "$m" "$f" || die "${m} が当たっていない（apply_patches.py を先に流す）"
done
grep -qF "PATCH(record-timestamp)" "$XR/teleop/utils/episode_writer.py" || die "PATCH(record-timestamp) が当たっていない"
# 2026-10-27: PC2 は起動の直後、時刻合わせ（chrony）が効くまでに時計が数秒単位で飛ぶ（起動からの補正のばらつき 8 秒）。
# 記録の時刻（time.time_ns()）が途中で飛ばないよう、合ってから始める。タイムゾーンは中国時間のままだが時刻には関係ない
if [ "$(timedatectl show -p NTPSynchronized --value 2> /dev/null)" != "yes" ] && [ -z "${TELEOP_SKIP_NTP:-}" ]; then
  die "PC2 の時計がまだ時刻合わせされていない（起動の直後に多い）。1〜2 分待ってから。急ぐなら TELEOP_SKIP_NTP=1"
fi

mkdir -p "$DATA" "$LOGDIR"
LOG="$LOGDIR/teleop_$(date +%Y%m%dT%H%M%S)_${TASK}.log"
ARGS=(--input-mode=controller --arm=G1_29 --display-mode=pass-through
      --network-interface="$IFACE" --motion --record --headless --ipc
      --frequency="$FREQ" --task-name="$TASK" --task-dir="$DATA")

if [ -n "${TELEOP_DRY:-}" ]; then
  echo "[start] （試し）修正は全部当たっている。起こすなら:"
  echo "  cd $XR/teleop && OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 $PY -u teleop_hand_and_arm.py ${ARGS[*]}"
  exit 0
fi

cd "$XR/teleop" || die "cd できない: ${XR}/teleop"
# BLAS の糸を 1 本に（2026-10-16: PC2 の Python は糸が多いと周期が乱れた）
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 nohup setsid "$PY" -u teleop_hand_and_arm.py "${ARGS[@]}" \
  > "$LOG" 2>&1 < /dev/null &
PID=$!
echo "$PID" > "$LOGDIR/teleop.pid"
echo "[start] 起こした: PID ${PID}・記録 ${DATA}/${TASK}/・ログ ${LOG}"

for _ in $(seq 1 "$WAIT_S"); do
  if grep -qF "Press [r] to start" "$LOG" 2>/dev/null; then
    WIFI_IP=$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1)
    echo "[start] ✓ 準備ができた。Quest で https://${WIFI_IP:-<PC2 の無線の IP>}:8012/?ws=wss://${WIFI_IP:-<IP>}:8012 を開き"
    echo "        「Pass-through」（周りが見える方。「Virtual Reality」は全面グレーになる）を押してから、Mac で go"
    exit 0
  fi
  kill -0 "$PID" 2> /dev/null || { echo "[start] ✗ 途中で終わった。ログの最後:"; tail -n 20 "$LOG"; exit 1; }
  sleep 1
done
echo "[start] ⚠️ ${WAIT_S} 秒待っても準備ができない（機体の DDS が来ていない？）。遠隔操作は動いたまま。ログの最後:"
tail -n 20 "$LOG"
exit 1
