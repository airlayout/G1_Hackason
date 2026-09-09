#!/usr/bin/env bash
# **PC2 の上で**動かす。`/cmd_vel` を購読して localhost の UDP へ中継する側を起こす。
#
#   ssh g1 'bash ~/mapping_tools/start_cmd_vel_bridge.sh'
#   ssh g1 'bash ~/mapping_tools/start_cmd_vel_bridge.sh stop'
#
# ⚠️ **これだけでは足は動かない。** これは ROS 側（rclpy）のプロセスで、
# 足に渡すのは SDK 側の `loco_driver.py`（別プロセス・system python3.8）である。
# 段 D（指令は出るが足は繋がない）はここまでで測る。
#
# ## なぜ専用の起動スクリプトが要るのか
#
# `pixi run python ~/nav_tools/cmd_vel_bridge.py` では**DDS の環境変数が入らない**ので
# Mac 側の Nav2 が出す `/cmd_vel` が見えない（購読者 0 のまま黙って待ち続ける）。
# PC2 では次の 3 つを明示しないと繋がらない:
#
#   ROS_DOMAIN_ID=0
#   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   CYCLONEDDS_URI の <Interfaces><NetworkInterface name="eth0" ...>
#
# ⚠️ **cyclonedds 0.10 系の書式である。** foxy 同梱の 0.7 の
# `<NetworkInterfaceAddress>` とは非互換で、混ぜると domain の作成ごと失敗する。
# `start_foxglove_bridge.sh` と同じ形にしてあるので、片方を直すときは両方見ること。
set -uo pipefail

PROJ="${G1_HUMBLE_PROJ:-$HOME/g1_humble}"
TOOLS="${G1_NAV_TOOLS:-$HOME/nav_tools}"
IFACE="${G1_IFACE:-eth0}"
PORT="${G1_UDP_PORT:-47600}"
LOG="$PROJ/cmd_vel_bridge.log"

[ -f "$PROJ/pixi.toml" ] || { echo "[bridge] $PROJ が無い。先に setup_pc2.sh" >&2; exit 1; }
[ -f "$TOOLS/cmd_vel_bridge.py" ] || {
    echo "[bridge] $TOOLS/cmd_vel_bridge.py が無い。Mac から deploy_to_pc2.sh" >&2; exit 1; }

# ⚠️ パターンにはスクリプト名の一部を入れない。`cmd_vel_bridge` だけだと
# **このスクリプト自身（start_cmd_vel_bridge.sh）の呼び出し行に当たって自殺する。**
# 引数まで含めて絞る（start_foxglove_bridge.sh が同じ理由で同じことをしている）。
PATTERN="cmd_vel_bridge.py --port"

stop_existing() {
    pkill -f -- "$PATTERN" >/dev/null 2>&1 && { echo "[bridge] 既存を停止"; sleep 1; }
}

if [ "${1:-}" = "stop" ]; then
    stop_existing; echo "[bridge] 停止しました"; exit 0
fi

stop_existing

cat > "$PROJ/_run_cmd_vel_bridge.sh" <<INNER
#!/usr/bin/env bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# cyclonedds 0.10 系の書式。0.7 の <NetworkInterfaceAddress> とは非互換
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="$IFACE" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
exec python "$TOOLS/cmd_vel_bridge.py" --port $PORT
INNER
chmod +x "$PROJ/_run_cmd_vel_bridge.sh"

cd "$PROJ"
rm -f "$LOG"
nohup "$HOME/.pixi/bin/pixi" run bash "$PROJ/_run_cmd_vel_bridge.sh" > "$LOG" 2>&1 &
echo "[bridge] 起動 pid=$!"

# 立ち上がりを待つ。**固定 sleep にしない**（pixi の初回解決が数十秒かかる）
for _ in $(seq 1 40); do
    grep -qE "cmd_vel|購読|Traceback|Error" "$LOG" 2>/dev/null && break
    sleep 1
done

if grep -qE "Traceback|Error|error" "$LOG" 2>/dev/null; then
    echo "[bridge] 起動に失敗した。ログ:" >&2
    tail -20 "$LOG" >&2
    exit 1
fi
pgrep -f -- "$PATTERN" >/dev/null || { echo "[bridge] プロセスが残っていない。ログ:" >&2
                                       tail -20 "$LOG" >&2; exit 1; }
echo "[bridge] 動いている（UDP 127.0.0.1:$PORT へ中継）"
echo "[bridge] ログ: $LOG"
echo
echo "⚠️ 足はまだ繋がっていない。動かすには**人が支えた状態で**次を別端末で:"
echo "    ssh -t g1 'python3 $TOOLS/loco_driver.py --network-interface $IFACE --max-vy 0 --arm'"
