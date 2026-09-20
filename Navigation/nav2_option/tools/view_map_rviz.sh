#!/usr/bin/env bash
# A-7 が生成した占有格子地図を RViz2 で表示する。実機・ロボットには一切触らない。
#
#   bash tools/view_map_rviz.sh                                  # room_a_map を表示
#   bash tools/view_map_rviz.sh <地図.yaml> [<軌跡.tum>]          # 任意の地図
#   bash tools/view_map_rviz.sh stop                             # 止める
#
# ## なぜコンテナか
#
# このホストには ROS 2 が入っていない(2026-09-09 確認: /opt/ros が無い)。
# `Mapping/real/docker/Dockerfile` の visualization ステージに Nav2 と
# octomap_server を足したイメージ(`g1-mapping-visualization:local`)があるので、
# そこで map_server と RViz2 を動かし、X11 をホストへ出す。
#
# Mac 向けの `Mapping/real/quickstart/start_rviz_mac.sh` は colima + VNC
# (`tiryoh/ros2-desktop-vnc:humble`)という別経路。**こちらは Linux ホスト用**で、
# DISPLAY をそのまま使うぶん段が少ない。
#
# ## DDS はループバックに閉じる
#
# 地図を見るだけなので G1 の L2 には出さない。ROS_DOMAIN_ID も専用の値にして
# 他の作業(Mapping の記録など)と混ざらないようにする。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_OPTION="$(cd "$HERE/.." && pwd)"
REPO_ROOT="$(cd "$NAV2_OPTION/../.." && pwd)"

IMAGE="${G1_VIEW_IMAGE:-g1-mapping-visualization:local}"
NAME="${G1_VIEW_NAME:-g1-view-map}"
DOMAIN="${G1_VIEW_DOMAIN_ID:-47}"

if [ "${1:-}" = "stop" ]; then
    docker rm -f "$NAME" >/dev/null 2>&1 && echo "[view] 止めました" || echo "[view] 動いていません"
    exit 0
fi

MAP_YAML="${1:-$NAV2_OPTION/g1_ws/src/g1_navigation/maps/room_a_map.yaml}"
TRAJ="${2:-$REPO_ROOT/Mapping/real/runs/20260904_203726_room_a/trajectory/trajectory.tum}"

[ -f "$MAP_YAML" ] || { echo "[view] 地図が無い: $MAP_YAML" >&2; exit 1; }
if [ ! -f "$TRAJ" ]; then
    echo "[view] 軌跡が無いので地図だけ表示する: $TRAJ" >&2
    TRAJ=""
fi
if [ -z "${DISPLAY:-}" ]; then
    echo "[view] DISPLAY が未設定。X が動いている端末から実行すること" >&2
    exit 1
fi
docker image inspect "$IMAGE" >/dev/null 2>&1 || {
    echo "[view] イメージが無い: $IMAGE" >&2
    echo "       cd $REPO_ROOT/Mapping/real && docker compose build rviz" >&2
    exit 1
}

# コンテナから X へ繋ぐ許可。既に許可済みなら何も起きない
xhost +local:docker >/dev/null 2>&1 || echo "[view] xhost に失敗(表示できないかもしれない)" >&2

docker rm -f "$NAME" >/dev/null 2>&1

MAP_DIR="$(cd "$(dirname "$MAP_YAML")" && pwd)"
MAP_NAME="$(basename "$MAP_YAML")"
TRAJ_ARGS=""
TRAJ_MOUNT=()
if [ -n "$TRAJ" ]; then
    TRAJ_DIR="$(cd "$(dirname "$TRAJ")" && pwd)"
    TRAJ_MOUNT=(-v "$TRAJ_DIR:/traj:ro")
    TRAJ_ARGS="/traj/$(basename "$TRAJ")"
fi

echo "[view] 地図: $MAP_YAML"
[ -n "$TRAJ" ] && echo "[view] 軌跡: $TRAJ"
echo "[view] RViz2 を起動する(ウィンドウを閉じると全部止まる)"

# 端末から実行したときだけ -it を付ける(CI や自動実行では TTY が無く docker が失敗する)
TTY_ARGS=()
[ -t 0 ] && [ -t 1 ] && TTY_ARGS=(-it)

docker run --rm "${TTY_ARGS[@]}" --name "$NAME" \
    --network host --ipc host \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e ROS_DOMAIN_ID="$DOMAIN" \
    -e CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>' \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$MAP_DIR:/maps:ro" \
    -v "$HERE:/tools:ro" \
    "${TRAJ_MOUNT[@]}" \
    "$IMAGE" bash -c "
source /opt/ros/humble/setup.bash

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/maps/$MAP_NAME &
sleep 4
ros2 run nav2_util lifecycle_bringup map_server

# 地図が map フレームで出るだけだと TF ツリーが無く RViz が固定フレームを解決できない
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link &

if [ -n '$TRAJ_ARGS' ]; then
    python3 /tools/publish_trajectory_path.py '$TRAJ_ARGS' --frame map &
fi

rviz2 -d /tools/view_map.rviz
"
