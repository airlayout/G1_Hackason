#!/usr/bin/env bash
# Nav2 の状態をブラウザから IP 直叩きで見る（nav_live_view.py の起動口）。
#
#   bash run_nav_live_view.sh            # 既定: PC2 の橋を読み、:8080 で出す
#   bash run_nav_live_view.sh 192.168.123.164
#
# PC2 では何も起動しない。橋（foxglove_bridge）を **読むだけ**。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PY:-$HERE/../../../Navigation/.venv/bin/python}"      # numpy/scipy/matplotlib 入り
BRIDGE_HOST="${1:-10.42.0.76}"
HTTP_PORT="${HTTP_PORT:-8080}"
# /map が来ないときに背景へ使う予備の事前地図
MAP="${MAP:-$HERE/../runs/20260906T135940_UiS_room_v3/map/nav_map_run.yaml}"

[ -x "$PY" ] || { echo "python が無い: $PY" >&2; exit 1; }
echo "ブラウザで開く URL:"
ifconfig | awk '/inet /{print $2}' | grep -v '^127\.' | sed "s|^|  http://|;s|$|:${HTTP_PORT}/|"
exec "$PY" -u "$HERE/nav_live_view.py" --host "$BRIDGE_HOST" --http-port "$HTTP_PORT" \
     ${MAP:+--map "$MAP"} "${@:2}"
