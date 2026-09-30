#!/usr/bin/env bash
# ROS アダプタを**実機なしで**通す。偽の ROS 側と一緒にコンテナの中で動かす。
#
#   bash mock_test.sh
#
# ⚠️ ここが通ってもアダプタが実機で通る保証にはならない（Nav2 も物理も無い）。
#    出せるのは「トピック名・型・QoS・JSON の形・地図の向き・断り文の通り道」まで。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_DIR="$(cd "$HERE/.." && pwd)"
IMAGE="${G1_UI_IMAGE:-g1-mapping-visualization:local}"
# ⚠️ ホストの ROS と混ざらないよう、専用の DOMAIN を使う
DOMAIN="${G1_UI_TEST_DOMAIN:-77}"

docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  echo "[テスト] イメージが無い: $IMAGE" >&2; exit 1; }

echo "########## ROS アダプタのモック確認（ROS_DOMAIN_ID=$DOMAIN）##########"
exec docker run --rm --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  -e PYTHONUNBUFFERED=1 \
  -v "$UI_DIR:/ui:ro" \
  "$IMAGE" bash -lc '
set -e
source /opt/ros/humble/setup.bash
python3 /ui/Adapter/mock/fake_ros.py &
FAKE=$!
python3 /ui/Adapter/ros_adapter.py --port 8099 &
ADAPTER=$!
# ⚠️ どちらかが落ちたら待ち続けずに終わる
trap "kill $FAKE $ADAPTER 2>/dev/null || true" EXIT
python3 /ui/Adapter/mock/check.py http://127.0.0.1:8099
'
