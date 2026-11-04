#!/usr/bin/env bash
# 案 A（2026-10-26）: PC2 の上で xr_teleoperate を切り離して起こす。Mac の scripts/pc2_teleop.sh start から呼ばれる。
#
#   ./teleop_start.sh <課題名>            例: ./teleop_start.sh door_room4_1027
#   TELEOP_DRY=1 ./teleop_start.sh <課題名>   起こさずに、確かめと起動のコマンドだけを出す
#   TELEOP_LEGS=none ./teleop_start.sh <課題名>  足なし: 純正が立たせたまま腕だけ（歩きの指令を送らない。修正 no-loco）
#   TELEOP_LEGS=seated ./teleop_start.sh <課題名> 座位: 座らせてダンピング（L2+B）のまま腕だけ。純正を止めて rt/lowcmd
#                                             （--motion なし。脚と腰は起動時の角度で固める。修正 seated-controls）。
#                                             合間は保持係（seat_hold.py）が最後の姿勢を保つ
#
# 2026-11-04: 起こす前に機体の状態を形ごとに確かめる（form_check.py）。立った形（walk・none）は純正が通常運転
# 500/501、座位は純正がダンピング（FSM 1）か止まっていてモータが有効。11-04 朝は FSM 0 のまま立った形で起こし、
# go しても腕が動かなかった。急ぐなら TELEOP_SKIP_FORM=1（確かめを飛ばす）
#
# 2026-11-03: 起こすのは Mac でも OMEN でもよい（どちらも scripts/pc2_teleop.sh から ssh で呼ぶ）。
# 機体への有線（PC2_DDS_IFACE、既定 eth0）に 192.168.123.x が無ければ起こさない。
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
LEGS="${TELEOP_LEGS:-walk}"                # walk＝足あり（スティックで歩く）／none＝足なし（歩きの指令を送らない）／seated＝座位
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # form_check.py・seat_hold.py（PC2 の ~/teleop_tools/）
WIFI_IFACE="${TELEOP_WIFI_IFACE:-wlan0}"   # Quest に見せる URL の IP を取る無線（PC2 は wlan0・OMEN は wlp128s20f3）
WAIT_S="${TELEOP_WAIT_S:-120}"   # 29DoF の初回は逆運動学のモデルづくりで 1 分ほどかかる。ログは画面の幅で折り返されるので短い文字列で待つ（10-27）

die() { echo "[start] ✗ $*"; exit 1; }

[[ "$TASK" =~ ^[A-Za-z0-9_-]+$ ]] || die "課題名は英数字と _ - だけ: ${TASK}"
[[ "$LEGS" = walk || "$LEGS" = none || "$LEGS" = seated ]] || die "TELEOP_LEGS は walk・none・seated のどれか: ${LEGS}"
if pgrep -f '^[^ ]*python[^ ]* (-u )?teleop_hand_and_arm.py' > /dev/null; then
  die "もう動いている: $(pgrep -af '^[^ ]*python[^ ]* (-u )?teleop_hand_and_arm.py' | head -1)"
fi
[ -x "$PY" ] || die "Python が無い: ${PY}（PC2 に遠隔操作の環境を作るのが先）"
# 2026-11-03: 機体への有線に 192.168.123.x が付いていなければ起こさない（OMEN はケーブルを抜くと消える）
ip -4 -o addr show "$IFACE" 2> /dev/null | grep -q " 192\.168\.123\." ||
  die "${IFACE} に 192.168.123.x が無い（機体への有線が抜けている・機体の電源が入っていない？）"
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

FORM=standing; [ "$LEGS" = seated ] && FORM=seated
if [ -z "${TELEOP_SKIP_FORM:-}" ]; then
  "$PY" "$TOOLS_DIR/form_check.py" --form "$FORM" --iface "$IFACE" || die "機体の状態がこの形に合わない（上の ✗）。急ぐなら TELEOP_SKIP_FORM=1"
fi

mkdir -p "$DATA" "$LOGDIR"
LOG="$LOGDIR/teleop_$(date +%Y%m%dT%H%M%S)_${TASK}.log"
ARGS=(--input-mode=controller --arm=G1_29 --display-mode=pass-through
      --network-interface="$IFACE" --record --headless --ipc
      --frequency="$FREQ" --task-name="$TASK" --task-dir="$DATA" --task-desc="legs=${LEGS}")
# 2026-11-04: 座位は --motion なし（遠隔操作が起動のときに純正を止め、rt/lowcmd で全身を指令する）
if [ "$LEGS" = seated ]; then
  grep -qF "PATCH(seated-controls)" "$MAIN" || die "座位には PATCH(seated-controls) が要る（pc2_teleop.sh patch を先に流す）"
  [ -z "${TELEOP_FWD_MAX:-}" ] || die "座位では歩かないので TELEOP_FWD_MAX は使えない"
  if [ -n "${TELEOP_DRY:-}" ]; then
    echo "[start] （試し）起こすときは先に保持係を起こす: $PY $TOOLS_DIR/seat_hold.py ensure --iface $IFACE"
  else
    "$PY" "$TOOLS_DIR/seat_hold.py" ensure --iface "$IFACE" || die "保持係を起こせない（合間の姿勢が保たれない）"
  fi
  echo "[start] 座位: 純正を止めて腕だけ（脚と腰は起動時の角度で固める・歩かない）。一時停止・A の終了・Quest の途切れの一時停止は効く"
  echo "        終わった後は保持係が最後の姿勢を保つ（pc2_teleop.sh hold で状態・release で力を抜いて止める）"
else
  ARGS+=(--motion)
fi
# 2026-11-03: 足なし。--motion のまま歩きの指令を送らない（記録の歩きは 0）。課題の説明（data.json の text.desc）にも残す
if [ "$LEGS" = none ]; then
  grep -qF "PATCH(no-loco)" "$MAIN" || die "足なしには PATCH(no-loco) が要る（apply_patches.py を先に流す）"
  ARGS+=(--no-loco)
  echo "[start] 足なし: 歩きの指令を送らない（スティックを倒しても歩かない。一時停止・A の終了・Quest の途切れの一時停止は効く）"
fi
# 10-30: 前（左スティックを前へ）の歩きの上限を上げて試すとき（修正 loco-fwd-max。後ろ・横・旋回は 0.3 のまま）
if [ -n "${TELEOP_FWD_MAX:-}" ]; then
  grep -qF "PATCH(loco-fwd-max)" "$MAIN" || die "TELEOP_FWD_MAX を使うには PATCH(loco-fwd-max) が要る（pc2_teleop.sh setup を先に流す）"
  awk -v v="$TELEOP_FWD_MAX" 'BEGIN { exit !(v + 0 > 0 && v + 0 <= 1.0 && v ~ /^[0-9.]+$/) }' ||
    die "TELEOP_FWD_MAX は 0 より大きく 1.0 以下の数: ${TELEOP_FWD_MAX}"
  ARGS+=(--loco-fwd-max="$TELEOP_FWD_MAX")
  echo "[start] 前への歩きの上限 ${TELEOP_FWD_MAX} m/s（後ろ・横・旋回は 0.3 のまま）"
fi

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
    WIFI_IP=$(ip -4 -o addr show "$WIFI_IFACE" 2>/dev/null | awk '{print $4}' | cut -d/ -f1)
    echo "[start] ✓ 準備ができた。Quest で https://${WIFI_IP:-<無線の IP>}:8012/?ws=wss://${WIFI_IP:-<IP>}:8012 を開き"
    echo "        「Pass-through」（周りが見える方。「Virtual Reality」は全面グレーになる）を押してから、Mac で go"
    exit 0
  fi
  kill -0 "$PID" 2> /dev/null || { echo "[start] ✗ 途中で終わった。ログの最後:"; tail -n 20 "$LOG"; exit 1; }
  sleep 1
done
echo "[start] ⚠️ ${WAIT_S} 秒待っても準備ができない（機体の DDS が来ていない？）。遠隔操作は動いたまま。ログの最後:"
tail -n 20 "$LOG"
exit 1
