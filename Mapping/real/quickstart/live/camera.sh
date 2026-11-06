#!/usr/bin/env bash
# **頭の IR カメラの映像を RViz2 に出す／止める。**
#
#   bash quickstart/live/camera.sh start    # up.sh の後に
#   bash quickstart/live/camera.sh stop
#   bash quickstart/live/camera.sh status
#
#   [PC2: apriltag/serve_view.py] --HTTP MJPEG :8080--> [コンテナ: camera/mjpeg_to_ros.py]
#       --/g1_camera/ir/image (mono8)--> [RViz2 の「Head camera (IR)」]
#
# ## ⚠️ タグとカメラを取り合う
#
# `serve_view.py` が /dev/video2 を握っている間、`localize_from_tags.py` は開けない。
# **start は up.sh（段 0 の初期姿勢）の後にする。**initial_pose.sh は段 0 の前に
# 自分で `serve_view.sh stop` を呼ぶので、映像を出したまま up.sh を回しても壊れない。
#
# ## 既定を軽くしてある
#
# PC2 は測位と Nav2 だけで load 6.2 / 8（2026-09-26）。**848x480・検出なし**で起こす。
# タグの枠も見たいときは G1_CAMERA_ARGS="--fast" など（serve_view.py の引数）。
#
# ⚠️ serve_view には**認証が無い**。使い終わったら stop する。
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/_common.sh"
QS="$(dirname "$HERE")"

VIEW_ARGS="${G1_CAMERA_ARGS:---width 848 --height 480 --no-annotate}"
BRIDGE_PY='mjpeg_to''_ros.py'       # ⚠️ 分割して書く（kill の自己一致よけ）
PORT=8080

pc2_view() { pc2 "bash \$HOME/g1_cfg/apriltag/serve_view.sh $*"; }

start() {
    say "PC2 で IR カメラの配信を起こす（${VIEW_ARGS}）"
    scp -q -i "$G1_KEY" "$QS/apriltag/serve_view.py" "$QS/apriltag/serve_view.sh" \
        "$G1_PC2_USER@$G1_PC2_HOST:g1_cfg/apriltag/" || die "serve_view を送れない"
    pc2_view start $VIEW_ARGS | sed 's/^/     /' >&2
    tcp_ok "$G1_PC2_HOST" "$PORT" || die "PC2 の :$PORT に届かない（serve_view が起きていない）"

    say "コンテナで中継を起こす（MJPEG → /g1_camera/ir/image）"
    ctr_kill "mjpeg_to" "_ros.py"
    docker exec -d -u ubuntu -e HOME=/home/ubuntu -e ROS_DOMAIN_ID=0 \
        -e CYCLONEDDS_URI="$G1_DDS_COL0" "$G1_CONTAINER" bash -c \
        "source /opt/ros/humble/setup.bash && exec python3 -u \
         /work/G1_Hackason/Mapping/real/quickstart/camera/$BRIDGE_PY \
         --url http://$G1_PC2_HOST:$PORT/stream.mjpg > /home/ubuntu/camera_bridge.log 2>&1"
    ctr_wait "最初の 1 枚" 20 'grep -q "1 枚目" /home/ubuntu/camera_bridge.log' \
        || ctr "tail -5 /home/ubuntu/camera_bridge.log | sed 's/^/     /'"
    ok "RViz2 の Displays にある「Head camera (IR)」にチェックを入れると映る"
}

stop() {
    say "映像を止める"
    ctr_kill "mjpeg_to" "_ros.py"
    pc2_view stop | sed 's/^/     /' >&2
}

case "${1:-status}" in
    start) start ;;
    stop) stop ;;
    status)
        pc2_view status | sed 's/^/     /' >&2
        ctr "tail -2 /home/ubuntu/camera_bridge.log 2>/dev/null | sed 's/^/     /'" ;;
    *) sed -n '2,6p' "$0" >&2; exit 2 ;;
esac
