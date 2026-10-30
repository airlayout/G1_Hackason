#!/usr/bin/env bash
# 案 A（2026-10-26）: 遠隔操作（xr_teleoperate）を PC2 で動かし、Mac から操作する。
# 無線は Quest ↔ PC2 と Mac ↔ PC2（ssh）だけ。機体の制御は PC2 の中の有線（eth0）。
# 計画書: docs/plan/2026-10-26-g1-door-teleop-demos.md の 9 章
#
#   ./scripts/pc2_teleop.sh deploy            PC2 へ道具を配る（~/teleop_tools/）
#   ./scripts/pc2_teleop.sh setup             配ってから、PC2 に遠隔操作の環境を作る（切り離して流す。初回は 10〜20 分）
#   ./scripts/pc2_teleop.sh setup-log         環境づくりのログを流し見る
#   ./scripts/pc2_teleop.sh check             機体の状態（モード・腕を借りられる状態か）を見る。動かさない
#   ./scripts/pc2_teleop.sh probe             腕を 3° だけ動かして、腕を借りられるかを確かめる（確認あり）
#   ./scripts/pc2_teleop.sh sensors <名前> [秒] ライダー・IMU・/dog_odom・リモコン・赤外線を録る（切り離す。既定 1800 秒）
#                                             ⚠️ 遠隔操作より先に起こす。中身は Door/real/pc2/rec_take.sh（PC2 の ~/door_real/）
#   ./scripts/pc2_teleop.sh sensors-stop <名前> センサーの記録を早めに止める（書き切ってから止まる）
#   ./scripts/pc2_teleop.sh start <課題名>     遠隔操作を切り離して起こす（--motion・記録あり・素通し）
#   ./scripts/pc2_teleop.sh go                追従の開始（キーボードの r）
#   ./scripts/pc2_teleop.sh rec               記録の開始／保存（s）。1 回ごとに 2 度押す
#   ./scripts/pc2_teleop.sh check-ep <課題名> [番号]  録った 1 本を点検する（歩きの値・時刻・センサー・扉の角度）。既定は最新
#   ./scripts/pc2_teleop.sh state             状態（追従中・記録中）
#   ./scripts/pc2_teleop.sh log               ログを流し見る（Ctrl+C で抜けても遠隔操作は止まらない）
#   ./scripts/pc2_teleop.sh quit              終える（q）。⚠️ 後始末で腕を下ろす。扉から手を離してから
#   ./scripts/pc2_teleop.sh kill              quit が効かないときだけ。Ctrl+C と同じ（やはり腕を下ろす）
#
# 接続先は G1_PC2_SSH（既定 g1wifi＝~/.ssh/config の 192.168.0.82）。Mac も Physical_AI_5G につなぐこと。
# Mac が Physical_AI_5G に居ないときは G1_PC2_JUMP=ubuntu@100.99.102.70（OMEN の Tailscale）で中継できる。
# 止めるときの第一は Quest の両スティック 0.5 秒（一時停止）。この道具の quit は終わらせるときに使う。
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIT="$HERE/../vendor/g1-starter-kit"
PC2="${G1_PC2_SSH:-g1wifi}"
TOOLS="teleop_tools"                          # PC2 の $HOME 相対
PY='$HOME/miniforge3/envs/tv/bin/python'      # PC2 の上で展開させる
SSH=(ssh -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
SCP=(scp -q -o ConnectTimeout=8)
if [ -n "${G1_PC2_JUMP:-}" ]; then
  SSH+=(-J "$G1_PC2_JUMP"); SCP+=(-J "$G1_PC2_JUMP")
fi

say() { echo "[pc2_teleop] $*"; }
pc2() { "${SSH[@]}" "$PC2" "$@"; }

cmd="${1:-}"
[ $# -gt 0 ] && shift
case "$cmd" in
  deploy|setup)
    pc2 "mkdir -p ~/$TOOLS ~/teleop_logs" || exit 1
    "${SCP[@]}" \
      "$HERE/pc2/teleop_start.sh" "$HERE/pc2/teleop_ipc.py" \
      "$HERE/pc2/setup_pc2_tv.sh" "$HERE/pc2/tv_constraints.txt" \
      "$KIT/tools/mode_check.py" "$KIT/tools/armsdk_probe.py" \
      "$KIT/setup/apply_patches.py" "$KIT/tools/stick_pause_selftest.py" \
      "$KIT/tools/record_timestamp_selftest.py" "$KIT/tools/state_watchdog_selftest.py" \
      "$KIT/tools/go_home_selftest.py" "$KIT/tools/arm_effort_selftest.py" "$HERE/pc2/ik_bench.py" "$HERE/pc2/teleop_episode_check.py" \
      "$PC2:$TOOLS/" || exit 1
    pc2 "chmod +x ~/$TOOLS/*.sh ~/$TOOLS/*.py" && say "配った: PC2 の ~/${TOOLS}/"
    if [ "$cmd" = setup ]; then
      pc2 'L=~/teleop_logs/setup_$(date +%Y%m%dT%H%M%S).log; nohup setsid bash ~/teleop_tools/setup_pc2_tv.sh > "$L" 2>&1 < /dev/null & echo "[pc2_teleop] 環境づくりを始めた（PID $!）。ログ: $L"'
      say "進み具合は ./scripts/pc2_teleop.sh setup-log"
    fi
    ;;
  setup-log) pc2 'f=$(ls -t ~/teleop_logs/setup_*.log 2>/dev/null | head -1); [ -n "$f" ] && tail -n 40 -F "$f" || echo "ログが無い"' ;;
  check) pc2 "$PY ~/$TOOLS/mode_check.py --iface eth0" ;;
  probe) "${SSH[@]}" -t "$PC2" "$PY ~/$TOOLS/armsdk_probe.py --iface eth0" ;;
  start)
    task="${1:?課題名を渡す（例 door_room4_1027）}"
    [[ "$task" =~ ^[A-Za-z0-9_-]+$ ]] || { say "課題名は英数字と _ - だけ"; exit 1; }
    pc2 "~/$TOOLS/teleop_start.sh $task"
    ;;
  sensors)
    name="${1:?名前を渡す（例 t0_1027_sensors）}"; secs="${2:-1800}"
    [[ "$name" =~ ^[A-Za-z0-9_-]+$ && "$secs" =~ ^[0-9]+$ ]] || { say "名前は英数字と _ -、秒は整数"; exit 1; }
    # 10-29: 「nohup setsid … &」だと ssh が閉じずに待ち続けた（記録は動いていた）。setsid -f で完全に切り離し、ssh の入力も閉じる
    "${SSH[@]}" -n "$PC2" "mkdir -p ~/mapping_runs/logs_1021 && setsid -f nohup ~/door_real/rec_take.sh $name $secs ir > ~/mapping_runs/logs_1021/${name}_take.log 2>&1 < /dev/null; sleep 3; tail -n 3 ~/mapping_runs/logs_1021/${name}_take.log"
    ;;
  sensors-stop)
    name="${1:?名前を渡す}"
    [[ "$name" =~ ^[A-Za-z0-9_-]+$ ]] || { say "名前は英数字と _ - だけ"; exit 1; }
    pc2 "~/door_real/rec_stop.sh $name"
    ;;
  check-ep)
    task="${1:?課題名を渡す}"; num="${2:-}"
    [[ "$task" =~ ^[A-Za-z0-9_-]+$ && ( -z "$num" || "$num" =~ ^[0-9]+$ ) ]] || { say "課題名は英数字と _ -、番号は整数"; exit 1; }
    pc2 "$PY ~/$TOOLS/teleop_episode_check.py --task $task ${num:+--episode $num}"
    ;;
  go)    pc2 "$PY ~/$TOOLS/teleop_ipc.py start" ;;
  rec)   pc2 "$PY ~/$TOOLS/teleop_ipc.py record" ;;
  quit)  pc2 "$PY ~/$TOOLS/teleop_ipc.py stop" ;;
  state) pc2 "$PY ~/$TOOLS/teleop_ipc.py state; pgrep -af '^[^ ]*python[^ ]* (-u )?teleop_hand_and_arm.py' | head -1 | grep . || echo '（遠隔操作は動いていない）'" ;;
  log)   pc2 'f=$(ls -t ~/teleop_logs/teleop_*.log 2>/dev/null | head -1); [ -n "$f" ] && tail -n 30 -F "$f" || echo "ログが無い"' ;;
  kill)  pc2 'p=$(cat ~/teleop_logs/teleop.pid 2>/dev/null); [ -n "$p" ] && kill -INT "$p" && echo "SIGINT を送った（PID $p）" || echo "PID が分からない"' ;;
  *)     awk 'NR > 1 && /^#/ { print; next } NR > 1 { exit }' "$0"; exit 1 ;;
esac
