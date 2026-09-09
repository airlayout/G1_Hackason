#!/usr/bin/env bash
# PC2 上で Foxglove Bridge を起動する。ブラウザから G1 の点群をライブで見るための入口。
#
#   ssh g1 'bash ~/mapping_tools/start_foxglove_bridge.sh'
#   Mac: ssh -N -L 8765:localhost:8765 g1
#        ブラウザ → https://app.foxglove.dev → Open connection
#                 → Foxglove WebSocket → ws://localhost:8765
#
# トンネルを挟む理由: app.foxglove.dev は HTTPS なので、素の ws:// でリモートに
# 繋ぐと混在コンテンツとして遮断される。localhost 宛だけは例外なので通る。
#
# rosbridge ではなく foxglove_bridge を使う理由: 点群は 441KB/フレーム 10Hz。
# rosbridge は JSON 化するが foxglove_bridge は CDR のまま流す（実測 4.5MB/s で
# 取りこぼしなし）。
set -uo pipefail

PROJ="${G1_HUMBLE_PROJ:-$HOME/g1_humble}"
IFACE="${G1_IFACE:-eth0}"
PORT="${FOXGLOVE_PORT:-8765}"
LOG="$PROJ/foxglove_bridge.log"

[ -f "$PROJ/pixi.toml" ] || { echo "[bridge] $PROJ が無い。先に setup_pc2.sh を実行" >&2; exit 1; }

# パターンは起動引数まで含める。単に "foxglove_bridge" だと、このスクリプト名
# (start_foxglove_bridge.sh) を含む呼び出し元シェル自身にマッチして自殺する。
PATTERN="foxglove_bridge --ros-args"

stop_existing() {
    pkill -f -- "$PATTERN" >/dev/null 2>&1 && { echo "[bridge] 既存を停止"; sleep 1; }
}

if [ "${1:-}" = "stop" ]; then
    stop_existing; echo "[bridge] 停止しました"; exit 0
fi

stop_existing

cat > "$PROJ/_run_bridge.sh" <<INNER
#!/usr/bin/env bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# cyclonedds 0.10 系の書式。0.7 の <NetworkInterfaceAddress> とは非互換
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="$IFACE" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
exec ros2 run foxglove_bridge foxglove_bridge \\
    --ros-args -p port:=$PORT -p address:=0.0.0.0 \\
               -p max_qos_depth:=5 -p send_buffer_limit:=100000000
INNER
chmod +x "$PROJ/_run_bridge.sh"

cd "$PROJ"
rm -f "$LOG"
nohup "$HOME/.pixi/bin/pixi" run bash "$PROJ/_run_bridge.sh" > "$LOG" 2>&1 &
echo "[bridge] 起動 pid=$!"
sleep 10

if grep -q "WebSocket server listening" "$LOG"; then
    echo "[bridge] $(grep -o 'ws://[^ ]*' "$LOG" | head -1) で待受中"
else
    echo "[bridge] 起動に失敗した可能性あり。ログ:" >&2
    tail -15 "$LOG" >&2
    exit 1
fi

# unitree_api / unitree_go / unitree_hg の独自型は登録できない（メッセージ定義が
# 環境に無いため）。点群・IMU・odom は標準型なので影響しない。
echo "[bridge] 登録できなかった独自型トピック: $(grep -c 'Failed to add channel' "$LOG") 件（想定内）"
for t in /utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360 \
         /unitree/slam_mapping/points /unitree/slam_mapping/odom; do
    grep -q "Failed to add channel for topic \"$t\"" "$LOG" \
        && echo "  $t : **登録失敗**" || echo "  $t : OK"
done
