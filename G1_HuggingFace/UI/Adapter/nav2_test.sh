#!/usr/bin/env bash
# **本物の Nav2 スタック**に対してアダプタを通す。実機は要らない（SDK だけ偽物）。
#
#   bash nav2_test.sh
#
# Navigation トラックの tools/mock_patrol_test.sh と同じ作法で Nav2 を上げ、
# そこへ ros_adapter.py を繋いで UI の口から確かめる。
#
# ⚠️ はまりどころ（Navigation 側の知見をそのまま踏襲）:
#    - **コンテナ内でビルドする**（ホストのバイナリは glibc が合わない）
#    - **heartbeat_required:=false**（操作PC の heartbeat 送信を立てないため）
#    - **set -u は使わない**（ROS の setup.bash が未定義変数を参照して落ちる）
# ⚠️ Nav2 の起動に 1 分ほどかかる。全体で 3〜4 分。
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UI_DIR="$(cd "$HERE/.." && pwd)"
NAV2_DIR="$(cd "$UI_DIR/../../Navigation/nav2_stable" && pwd)"
IMAGE="${G1_UI_IMAGE:-g1-mapping-visualization:local}"
WORK="${G1_UI_NAV2_WORK:-/tmp/g1_ui_nav2}"
# ⚠️ ホストや他のモックテストと混ざらないよう専用の DOMAIN にする
DOMAIN="${G1_UI_NAV2_DOMAIN:-88}"
PORT="${G1_UI_NAV2_PORT:-8098}"
mkdir -p "$WORK"

[ -d "$NAV2_DIR/g1_ws/install" ] || {
  echo "[nav2] g1_ws がビルドされていない: $NAV2_DIR/g1_ws/install" >&2; exit 1; }

if [ ! -x "$WORK/build/g1_sdk_bridge_mock_server" ]; then
  echo "[nav2] 偽の SDK ブリッジをコンテナ内でビルドする（初回のみ・数分）"
  docker run --rm -v "$NAV2_DIR:/w:ro" -v "$WORK:/out" "$IMAGE" bash -c '
      cmake -S /w/g1_sdk_bridge_cpp -B /out/build -DG1_SDK_BRIDGE_BUILD_TESTS=OFF >/dev/null &&
      cmake --build /out/build --target g1_sdk_bridge_mock_server -j4 >/dev/null' \
      || { echo "[nav2] ビルドに失敗した" >&2; exit 1; }
fi

echo "########## 本物の Nav2 に対するアダプタの確認（ROS_DOMAIN_ID=$DOMAIN）##########"
docker rm -f g1-ui-nav2 >/dev/null 2>&1
exec docker run --rm --name g1-ui-nav2 --network host --ipc host \
  -e ROS_DOMAIN_ID="$DOMAIN" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  -e PYTHONUNBUFFERED=1 -e G1_UI_PORT="$PORT" \
  -v "$NAV2_DIR:/w" -v "$UI_DIR:/ui:ro" -v "$WORK:/out" \
  "$IMAGE" bash -c '
set -o pipefail
source /opt/ros/humble/setup.bash
source /w/g1_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

rm -rf /tmp/g1_bridge; mkdir -p /tmp/g1_bridge
echo "[1/4] 偽の SDK ブリッジ"
setsid nohup /out/build/g1_sdk_bridge_mock_server --like-g1 >/tmp/mock_sdk.log 2>&1 &
sleep 2

echo "[2/4] 本物の Nav2 スタック（map_server / planner / controller / cmd_router / patrol）"
setsid nohup ros2 launch g1_navigation navigation.launch.py backend:=mock \
    heartbeat_required:=false \
    patrol_waypoints:=/w/g1_ws/src/g1_navigation/config/patrol_synthetic.yaml \
    >/tmp/nav.log 2>&1 &
sleep 55
echo "  上がったノード: $(ros2 node list 2>/dev/null | tr "\n" " ")"

echo "[3/4] ROS アダプタ"
setsid nohup python3 /ui/Adapter/ros_adapter.py --port '"$PORT"' >/tmp/adapter.log 2>&1 &
sleep 5

echo "[4/4] 確認"
python3 /ui/Adapter/mock/nav2_check.py http://127.0.0.1:'"$PORT"'
RC=$?
if [ $RC -ne 0 ]; then
  echo "--- nav.log の末尾 ---"; tail -25 /tmp/nav.log
  echo "--- adapter.log の末尾 ---"; tail -15 /tmp/adapter.log
fi
pkill -f "lib/nav[2]_" 2>/dev/null; pkill -f "ros2 launc[h]" 2>/dev/null
pkill -f g1_sdk_bridge_mock_serve[r] 2>/dev/null
exit $RC
'
