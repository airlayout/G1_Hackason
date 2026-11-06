#!/usr/bin/env bash
# 作ったばかりの地図を RViz2 に出して、**目で確かめる**（機体もネットワークも要らない）。
#
#   bash quickstart/show_map_rviz.sh runs/<id>                  # 2D と 3D を出す
#   bash quickstart/show_map_rviz.sh runs/<id> --shot out.png    # 出したうえでスクショを撮る
#   bash quickstart/show_map_rviz.sh stop                        # 流すのをやめる
#
# ## なぜスクリプトにしたか（2026-09-18）
#
# `clean_map.py` の合否は数字で出るが、**数字は「地図が在る」ことまでしか言わない。**
# 09-18 に 0911 の記録で地図を差し替えたとき、確かめるのに必要だった段が 5 つあり、
# どれも一度は踏み抜いた。次に忘れるので 1 本にしてある。
#
# 1. **Fixed Frame が無いと RViz2 は何も描かない。** 静的 TF を 1 本流す
# 2. **map_server はライフサイクルノード。** 起こさないと `/map` が出ない
# 3. **`pcd_to_pointcloud` は VOLATILE。** 1 回だけ出すと RViz2 に残らないので周期発行する
# 4. **点数で描画レートが決まる。** 2026-09-08 の実測で 728k 点 = 1 fps、52.9k 点 = 20 fps。
#    そのまま出すと 244 万点で固まるので、表示用に間引いた `*_view.pcd` を作る
# 5. **窓が画面より大きいとスクショに半分しか写らない。** X の解像度と窓を揃える
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL="$(cd "$HERE/.." && pwd)"
# shellcheck source=live/_common.sh
. "$HERE/live/_common.sh"

NAME="${G1_CONTAINER:-rviz}"
CFG="$HERE/rviz/g1_map_check.rviz"
# コンテナの /work は Mac のワークスペースの根。ここで読み替える
WORK_REAL="/work/G1_Hackason/Mapping/real"
VIEW_VOXEL="${G1_VIEW_VOXEL:-0.30}"   # 表示用の間引き[m]。0.30 で 69k 点（実測 30 fps）
SCREEN="${G1_RVIZ_SCREEN:-1920x1080}"

say() { echo "[show_map] $*"; }
die() { echo "[show_map] $*" >&2; exit 1; }
x_exec() {
    docker exec -u ubuntu -e DISPLAY=:1 -e XAUTHORITY=/home/ubuntu/.Xauthority "$NAME" "$@"
}

if [ "${1:-}" = stop ]; then
    docker exec "$NAME" pkill -f 'map_server|pcd_to_pointcloud|static_transform_publisher' 2>/dev/null
    say "流すのをやめた（RViz2 とコンテナは残す）"
    exit 0
fi

SESSION="${1:-}"
[ -n "$SESSION" ] || die "記録を渡すこと: bash quickstart/show_map_rviz.sh runs/<id>"
SHOT=""
if [ "${2:-}" = "--shot" ]; then
    SHOT="${3:-}"
    [ -n "$SHOT" ] || die "--shot には出力先を渡すこと"
fi

MAP_DIR="$REAL/$SESSION/map"
# 2D は走る地図（nav_map_run）を優先。G1_SHOW_MAP=nav_map_clean で掃除前と見比べられる
if [ -n "${G1_SHOW_MAP:-}" ]; then MAP2D="$G1_SHOW_MAP"
elif [ -f "$MAP_DIR/nav_map_run.yaml" ]; then MAP2D=nav_map_run
else MAP2D=nav_map_clean; fi
[ -f "$MAP_DIR/$MAP2D.yaml" ] || die "2D が無い: $MAP_DIR/$MAP2D.yaml（先に clean_map.py）"
docker inspect "$NAME" >/dev/null 2>&1 || die "コンテナ $NAME が無い。先に start_rviz_mac.sh offline"

# ── 表示用に間引いた点群を用意する（無ければ作る）────────────────────
CLOUD="$MAP_DIR/map_octomap_sim.pcd"
VIEW="$MAP_DIR/map_octomap_sim_view.pcd"
if [ -f "$CLOUD" ] && { [ ! -f "$VIEW" ] || [ "$CLOUD" -nt "$VIEW" ]; }; then
    say "表示用に間引く（voxel ${VIEW_VOXEL} m）"
    "$REAL/../../Navigation/.venv/bin/python" - "$CLOUD" "$VIEW" "$VIEW_VOXEL" <<'PY'
import sys
import open3d as o3d
src, dst, voxel = sys.argv[1], sys.argv[2], float(sys.argv[3])
cloud = o3d.io.read_point_cloud(src)
thin = cloud.voxel_down_sample(voxel)
o3d.io.write_point_cloud(dst, thin)
print("  {:,} 点 -> {:,} 点".format(len(cloud.points), len(thin.points)))
PY
fi

# ── 流す ────────────────────────────────────────────────
docker exec "$NAME" pkill -f 'map_server|pcd_to_pointcloud|static_transform_publisher' 2>/dev/null
sleep 1
M="$WORK_REAL/$SESSION/map"

say "[1] map -> base_link の静的 TF（Fixed Frame を存在させる）"
docker exec -d "$NAME" bash -c \
    "source /opt/ros/humble/setup.bash; ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link > /tmp/show_tf.log 2>&1"

say "[2] 2D を /map に流す（$MAP2D.yaml）"
docker exec -d "$NAME" bash -c \
    "source /opt/ros/humble/setup.bash; ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$M/$MAP2D.yaml > /tmp/show_map_server.log 2>&1"
sleep 4
# ⚠️ ライフサイクルノードなので起こす。これを忘れると /map は「在るのに空」
docker exec "$NAME" bash -c "source /opt/ros/humble/setup.bash; ros2 run nav2_util lifecycle_bringup map_server" >/dev/null 2>&1 \
    || say "  ⚠️ lifecycle_bringup が失敗した（/map が出ないなら /tmp/show_map_server.log を見る）"

if [ -f "$VIEW" ]; then
    say "[3] 3D を /cloud_pcd に周期発行（1 Hz）"
    docker exec -d "$NAME" bash -c \
        "source /opt/ros/humble/setup.bash; ros2 run pcl_ros pcd_to_pointcloud --ros-args -p file_name:=$M/$(basename "$VIEW") -p interval:=1.0 -p frame_id:=map > /tmp/show_pcd.log 2>&1"
else
    say "[3] 3D は飛ばす（$VIEW が無い）"
fi

say "[4] X を $SCREEN にして RViz2 を起こす"
x_exec xrandr --output VNC-0 --mode "$SCREEN" 2>/dev/null || say "  ⚠️ 解像度を変えられなかった（そのまま続ける）"
[ -f "$CFG" ] || die "$CFG が無い"
docker cp "$CFG" "$NAME:/home/ubuntu/g1_map_check.rviz" >/dev/null
docker exec "$NAME" chown ubuntu:ubuntu /home/ubuntu/g1_map_check.rviz
docker exec "$NAME" pkill -f rviz2 2>/dev/null
sleep 3
docker exec -d -u ubuntu -e DISPLAY=:1 -e XAUTHORITY=/home/ubuntu/.Xauthority \
    -e LIBGL_ALWAYS_SOFTWARE=1 "$NAME" \
    bash -c "source /opt/ros/humble/setup.bash; rviz2 -d /home/ubuntu/g1_map_check.rviz > /home/ubuntu/rviz2.log 2>&1"
sleep 20
docker exec "$NAME" pgrep -f rviz2 >/dev/null 2>&1 || {
    docker exec "$NAME" cat /home/ubuntu/rviz2.log >&2
    die "RViz2 が起動しなかった（上のログ）"
}

if [ -n "$SHOT" ]; then
    x_exec import -window root /tmp/show_map_shot.png || die "スクショに失敗した"
    docker cp "$NAME:/tmp/show_map_shot.png" "$SHOT" >/dev/null
    say "スクショ: $SHOT"
fi

cat <<EOM

  ブラウザで開く: http://${G1_VM_IP:-192.168.123.201}/   （VNC のパスワード: ubuntu）
  止める:         bash quickstart/show_map_rviz.sh stop

EOM
