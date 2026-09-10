#!/usr/bin/env bash
# LiDAR の点群と IMU を rosbag2 に記録する。
#
#   ./scripts/record_bag.sh <名前>
#   ./scripts/record_bag.sh room1
#
# 先に別ターミナルで ./scripts/lidar_view.sh を起動しておくこと。
# 停止は Ctrl+C。保存先は bags/<名前>_<日時>/
#
# 記録した bag は次のように再生できる（RViz は lidar_view.sh のものを使う）:
#   source scripts/lib.sh && load_config && use_ros && ros2 bag play bags/<dir>

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib.sh"

NAME="${1:-}"
[ -n "$NAME" ] || _die "使い方: $0 <名前>   例: $0 room1"

load_config
use_ros

TOPICS=(/livox/lidar /livox/imu)
OUT="$REPO_DIR/bags/${NAME}_$(date +%Y%m%d_%H%M%S)"
AVAIL_GB=$(df --output=avail -BG "$REPO_DIR" | tail -1 | tr -dc '0-9')

cat <<EOF
==============================================
 bag 記録
  保存先        : $OUT
  トピック      : ${TOPICS[*]}
  ROS_DOMAIN_ID : $ROS_DOMAIN_ID
  ディスク空き  : ${AVAIL_GB} GB
----------------------------------------------
 ⚠️ 点群は 1 分あたり 300MB 程度になります（実測）。
    1 回 2〜3 分を目安に。長く録ると一気にディスクを食います。
==============================================
EOF

[ "$AVAIL_GB" -ge 5 ] || _die "ディスク残量が ${AVAIL_GB}GB しかありません。
     古い bag を消してください:  du -sh $REPO_DIR/bags/*"

echo "トピックを確認中..."
FOUND=$(ros2 topic list 2>/dev/null | grep -c -E '^/livox/(lidar|imu)$' || true)
if [ "$FOUND" -lt 2 ]; then
  echo "  ⚠️ /livox/lidar と /livox/imu が見えません（検出 ${FOUND}/2）。"
  echo "     先に別ターミナルで ./scripts/lidar_view.sh を起動してください。"
  echo "     このまま記録しても空の bag になります。"
  # GUI やスクリプトから呼ばれると stdin が無く、read が失敗して即終了してしまう。
  # 対話ターミナルのときだけ確認する。
  if [ -t 0 ]; then
    read -r -p "     それでも続けますか? [y/N] " ans
    [ "$ans" = "y" ] || exit 1
  else
    _die "中止しました（先に lidar_view.sh を起動してから、もう一度どうぞ）"
  fi
else
  _ok "2 トピックとも配信されています"
fi

mkdir -p "$REPO_DIR/bags"
echo ""
echo "記録開始。停止は Ctrl+C（Space で一時停止）。"
ros2 bag record "${TOPICS[@]}" -o "$OUT"

echo ""
echo "記録終了: $OUT"
du -sh "$OUT" 2>/dev/null || true
echo "再生: source scripts/lib.sh && load_config && use_ros && ros2 bag play $OUT"
