#!/usr/bin/env bash
# 実機で Nav2 を運用するための RViz を起動する。
#
# ⚠️ **`view_map_rviz.sh` とは別物。** あちらは地図を眺めるためのもので、
# 保存地図と軌跡を**ファイルから**読むだけ。こちらは**動いている Nav2 に繋ぐ**。
#
# ## ⚠️ 繋がらないときに真っ先に疑う2つ
#
# 1. **`ROS_DOMAIN_ID` が PC2 と一致しているか。** 違うと何も見えない
# 2. **`RMW_IMPLEMENTATION` が PC2 と一致しているか。**
#    既定は `rmw_fastrtps_cpp`(D-03)。`view_map_rviz.sh` は CycloneDDS を使っており、
#    **そのまま真似ると食い違う。** 実装が違うと discovery が成立しないことがある
#
# ## 使い方
#
#     ./rviz_operate.sh                 # 既定(domain 0 / FastDDS)
#     G1_DOMAIN_ID=3 ./rviz_operate.sh
#     ./rviz_operate.sh stop
#
# ## Goal の送り方
#
# ツールバーの **「2D Goal Pose」**をクリックしてから地図上をドラッグする。
# `/goal_pose` に出た姿勢を `bt_navigator` が拾う。
#
# ⚠️ **「2D Pose Estimate」は使えない。** AMCL を動かしていないので誰も受け取らない。
# 自己位置の初期合わせは `match_scan_to_map_2d.py` の結果を
# `g1_slam_odom_tf.py --map-to-odom` に渡して行う。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_OPTION="$(cd "$HERE/.." && pwd)"
REPO_ROOT="$(cd "$NAV2_OPTION/../.." && pwd)"

IMAGE="${G1_RVIZ_IMAGE:-g1-mapping-visualization:local}"
NAME="${G1_RVIZ_NAME:-g1-nav2-rviz}"
# ⚠️ **PC2 側と揃えること。** 揃っていないと RViz には何も出ない
DOMAIN="${G1_DOMAIN_ID:-0}"
RMW="${G1_RMW:-rmw_fastrtps_cpp}"
CONFIG="${G1_RVIZ_CONFIG:-$HERE/nav2_operate.rviz}"

if [ "${1:-}" = "stop" ]; then
    docker rm -f "$NAME" >/dev/null 2>&1 && echo "[rviz] 止めました" || echo "[rviz] 動いていません"
    exit 0
fi

[ -f "$CONFIG" ] || { echo "[rviz] 設定が無い: $CONFIG" >&2; exit 1; }
if [ -z "${DISPLAY:-}" ]; then
    echo "[rviz] DISPLAY が未設定。X が動いている端末から実行すること" >&2
    exit 1
fi
docker image inspect "$IMAGE" >/dev/null 2>&1 || {
    echo "[rviz] イメージが無い: $IMAGE" >&2
    echo "       cd $REPO_ROOT/Mapping/real && docker compose build rviz" >&2
    exit 1
}

xhost +local:docker >/dev/null 2>&1 || echo "[rviz] xhost に失敗(表示できないかもしれない)" >&2
docker rm -f "$NAME" >/dev/null 2>&1

echo "[rviz] ROS_DOMAIN_ID=$DOMAIN  RMW=$RMW"
echo "[rviz] ⚠️ PC2 側と一致していないと何も表示されない"
echo "[rviz] 設定: $CONFIG"

TTY_ARGS=()
[ -t 0 ] && [ -t 1 ] && TTY_ARGS=(-it)

# --network host: DDS の discovery をホストのネットワークで行う(PC2 と同じ L2 に居る前提)
docker run --rm "${TTY_ARGS[@]}" --name "$NAME" \
    --network host --ipc host \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}" \
    -e RMW_IMPLEMENTATION="$RMW" \
    -e ROS_DOMAIN_ID="$DOMAIN" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$CONFIG:/cfg/nav2_operate.rviz:ro" \
    "$IMAGE" \
    bash -lc 'source /opt/ros/humble/setup.bash && exec rviz2 -d /cfg/nav2_operate.rviz'
