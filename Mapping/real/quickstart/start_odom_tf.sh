#!/usr/bin/env bash
# PC2 上で odom_to_tf.py を起動する。~/g1_humble の pixi 環境で動かす。
#
#   ssh g1 'bash ~/mapping_tools/start_odom_tf.sh'
#   ssh g1 'bash ~/mapping_tools/start_odom_tf.sh stop'
#
# G1 は /tf を出していないので、これを流さないと 3D パネルに G1 の位置が出ない。
set -uo pipefail

PROJ="${G1_HUMBLE_PROJ:-$HOME/g1_humble}"
IFACE="${G1_IFACE:-eth0}"
SCRIPT="$HOME/mapping_tools/odom_to_tf.py"
LOG="$PROJ/odom_to_tf.log"
# 呼び出し元シェルのコマンドラインは "start_odom_tf.sh" であって "odom_to_tf.py" を
# 含まないので、スクリプト名で照合してよい（自殺しない）。
# 逆に "python.* odom_to_tf.py" のようにスペース区切りを要求すると、実際の
# コマンドラインが ".../mapping_tools/odom_to_tf.py"（直前が "/"）なので一致せず、
# 二重起動する（2026-09-03に実際に発生）。
# "python" を前置しないと、ファイル名を引数に含む呼び出し（chmod など）まで
# マッチして、その shell ごと殺してしまう（2026-09-03に実際に発生）
PATTERN="python.*odom_to_tf\.py"

stop_existing() {
    pkill -f -- "$PATTERN" >/dev/null 2>&1 && { echo "[odom_tf] 既存を停止"; sleep 1; }
}

if [ "${1:-}" = "stop" ]; then
    stop_existing; echo "[odom_tf] 停止しました"; exit 0
fi

[ -f "$SCRIPT" ] || { echo "[odom_tf] $SCRIPT が無い" >&2; exit 1; }
stop_existing

cat > "$PROJ/_run_odom_tf.sh" <<INNER
#!/usr/bin/env bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="$IFACE" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
exec python "$SCRIPT" "\$@"
INNER
chmod +x "$PROJ/_run_odom_tf.sh"

cd "$PROJ"
rm -f "$LOG"
shift_args=("$@")
nohup "$HOME/.pixi/bin/pixi" run bash "$PROJ/_run_odom_tf.sh" "${shift_args[@]}" > "$LOG" 2>&1 &
echo "[odom_tf] 起動 pid=$!"
sleep 8
if grep -q "件配信" "$LOG"; then
    grep "件配信" "$LOG" | tail -1 | sed 's/^/[odom_tf] /'
else
    echo "[odom_tf] まだ配信が確認できていません。ログ:" >&2
    tail -12 "$LOG" >&2
    exit 1
fi
