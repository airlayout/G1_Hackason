#!/usr/bin/env bash
# PC2 上で restamp_points.py を起動する。
#   ssh g1 'bash ~/mapping_tools/start_restamp.sh'
#   ssh g1 'bash ~/mapping_tools/start_restamp.sh stop'
set -uo pipefail

PROJ="${G1_HUMBLE_PROJ:-$HOME/g1_humble}"
IFACE="${G1_IFACE:-eth0}"
SCRIPT="$HOME/mapping_tools/restamp_points.py"
LOG="$PROJ/restamp_points.log"
# 呼び出し元は start_restamp.sh で "restamp_points.py" を含まないので自殺しない
# "python" を前置しないと、ファイル名を引数に含む呼び出し（chmod など）まで
# マッチして、その shell ごと殺してしまう（2026-09-03に実際に発生）
PATTERN="python.*restamp_points\.py"

stop_existing() {
    pkill -f -- "$PATTERN" >/dev/null 2>&1 && { echo "[restamp] 既存を停止"; sleep 1; }
}

if [ "${1:-}" = "stop" ]; then
    stop_existing; echo "[restamp] 停止しました"; exit 0
fi

[ -f "$SCRIPT" ] || { echo "[restamp] $SCRIPT が無い" >&2; exit 1; }
stop_existing

cat > "$PROJ/_run_restamp.sh" <<INNER
#!/usr/bin/env bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="$IFACE" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
exec python "$SCRIPT" "\$@"
INNER
chmod +x "$PROJ/_run_restamp.sh"

cd "$PROJ"
rm -f "$LOG"
nohup "$HOME/.pixi/bin/pixi" run bash "$PROJ/_run_restamp.sh" "$@" > "$LOG" 2>&1 &
echo "[restamp] 起動 pid=$!"
sleep 8
if grep -q "件を再配信" "$LOG"; then
    grep "件を再配信" "$LOG" | tail -1 | sed 's/^/[restamp] /'
else
    echo "[restamp] 再配信を確認できていません。ログ:" >&2
    tail -12 "$LOG" >&2
    exit 1
fi
