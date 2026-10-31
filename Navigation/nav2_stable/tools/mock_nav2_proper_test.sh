#!/usr/bin/env bash
# Dev/Nav2Proper（2026-10-31）の変更をモックで確かめる。**実機は要らない。操作PC の Docker だけで完結する。**
#
# ## 何を確かめるのか
#
# | # | 確かめること | 変更 |
# |---|---|---|
# | ① | 全ライフサイクルノード（collision_monitor を含む）が active、G1 用 BT が読まれている、behavior_server に spin/backup が無い | BT 差し替え・Collision Monitor |
# | ② | 機体の前に人が立つと `/cmd_vel_safe` が 0 になり、**FAULT にならず**、人が去ると Goal に着く | Collision Monitor（stop_pub_timeout） |
# | ③ | 人が長く立ちふさがり BT が **Wait** まで進んでも FAULT にならない | g1_cmd_router の nav2_silent_status_topics |
# | ④ | 足元の消し残りが ObstacleLayer では消え、VoxelLayer（旧設定）では残る | local costmap の層の変更 |
#
# ## 使い方（ホストから）
#
#     cd <repo>/Navigation/nav2_stable
#     ./tools/mock_nav2_proper_test.sh            # ①②④（約4分）
#     ./tools/mock_nav2_proper_test.sh stop       # ①② だけ（約2分）
#     ./tools/mock_nav2_proper_test.sh wait       # ③（約4分。人を 110 秒立たせる）
#     ./tools/mock_nav2_proper_test.sh clearing   # ④ だけ、新旧の層を両方（約3分）
#
# ⚠️ はまりどころは mock_deadlock_test.sh と同じ（コンテナ内ビルド /
#    heartbeat_required:=false / set -u を使わない）。
# ⚠️ イメージは Humble + navigation2 が入っていればよい。既定は
#    Mapping の g1-mapping-visualization:local。G1_MOCK_IMAGE で差し替えられる。
set -o pipefail

MODE="${1:-basic}"
case "$MODE" in basic|stop|wait|clearing) ;;
  *) echo "使い方: $0 [basic|stop|wait|clearing]" >&2; exit 2 ;; esac

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # nav2_stable
IMAGE="${G1_MOCK_IMAGE:-g1-mapping-visualization:local}"
WORK="${G1_MOCK_WORK:-/tmp/g1_mock_nav2_proper}"
DOMAIN="${G1_MOCK_DOMAIN:-88}"   # 実機とも他のモック試験とも混ざらない番号
mkdir -p "$WORK"

# --- ビルド（コンテナ内。ホストのバイナリは glibc が合わず動かない）-----------
# ⚠️ ワークスペースの成果物は $WORK に出す。リポジトリの g1_ws/ に build/ install/ を作らない
echo "[mock] コンテナ内でビルドする"
docker run --rm -v "$HERE:/w:ro" -v "$WORK:/out" "$IMAGE" bash -c '
    set -e
    source /opt/ros/humble/setup.bash
    if [ ! -x /out/build/g1_sdk_bridge_mock_server ]; then
        cmake -S /w/g1_sdk_bridge_cpp -B /out/build -DG1_SDK_BRIDGE_BUILD_TESTS=OFF >/dev/null
        cmake --build /out/build --target g1_sdk_bridge_mock_server -j4 >/dev/null
    fi
    cd /w/g1_ws
    colcon --log-base /out/ws_log build --base-paths src \
        --build-base /out/ws_build --install-base /out/ws_install >/out/colcon.log 2>&1' \
    || { echo "[mock] ビルドに失敗した（$WORK/colcon.log）" >&2; exit 1; }

# ⚠️ 本物の nav2_params.yaml から生成する（設定を二重に持たない）。
# ④の対照として、local costmap だけを旧設定（voxel_layer）に戻した版も作る。
PARAMS_SRC="$HERE/g1_ws/src/g1_navigation/config/nav2_params.yaml"
cp "$PARAMS_SRC" "$WORK/params.yaml"
# 対照実験用: G1_STOP_PUB_TIMEOUT=1.0 で Collision Monitor の既定値に戻す（②が FAULT になるはず）
if [ -n "${G1_STOP_PUB_TIMEOUT:-}" ]; then
    sed -i "s/^    stop_pub_timeout: .*/    stop_pub_timeout: ${G1_STOP_PUB_TIMEOUT}/" "$WORK/params.yaml"
    echo "[mock] 対照実験: stop_pub_timeout=${G1_STOP_PUB_TIMEOUT}"
fi
sed -e 's/^      plugins: \["obstacle_layer", "inflation_layer"\]/      plugins: ["voxel_layer", "inflation_layer"]/' \
    -e 's|^          topic: /utlidar/cloud_livox_mid360|          topic: /g1/cloud_dense|' \
    "$PARAMS_SRC" > "$WORK/params_voxel.yaml"
grep -q 'plugins: \["voxel_layer", "inflation_layer"\]' "$WORK/params_voxel.yaml" \
    || { echo "[mock] 旧設定（voxel_layer）の版を作れなかった。sed の対象が変わった" >&2; exit 1; }

cat > "$WORK/common.sh" <<'INNER'
set -o pipefail
source /opt/ros/humble/setup.bash
source /out/ws_install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

stop_all() {
    # ⚠️ ログは /out に写してから止める（コンテナは --rm なので /tmp は消える）
    cp /tmp/*.log /out/ 2>/dev/null
    pkill -f "lib/nav[2]_" 2>/dev/null; pkill -f g1_sdk_bridge_mock_serve[r] 2>/dev/null
    pkill -f "ros2 launc[h]" 2>/dev/null; pkill -f mock_perso[n].py 2>/dev/null
    pkill -f fake_sensor_publishe[r].py 2>/dev/null; pkill -f cloud_gat[e].py 2>/dev/null
    pkill -f patrol_nod[e].py 2>/dev/null; pkill -f static_transform_publishe[r] 2>/dev/null
    sleep 3
}

# 引数: params ファイル。ログは /tmp/nav.log
start_stack() {
    stop_all
    rm -rf /tmp/g1_bridge; mkdir -p /tmp/g1_bridge
    setsid nohup /out/build/g1_sdk_bridge_mock_server --like-g1 >/tmp/mock_sdk.log 2>&1 &
    sleep 2
    setsid nohup ros2 launch g1_navigation navigation.launch.py backend:=mock \
        params_file:="$1" heartbeat_required:=false >/tmp/nav.log 2>&1 &
    sleep 55
}

state() { timeout 8 ros2 topic echo /g1/bridge_status --once 2>/dev/null \
          | grep -E "^  message:" | head -1 | awk '{print $2}'; }
vx()    { timeout 5 ros2 topic echo --once "$1" geometry_msgs/msg/Twist 2>/dev/null \
          | awk '/linear:/{f=1} f&&/x:/{print $2; exit}'; }
goal()  { timeout 15 ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
          "{header: {frame_id: map}, pose: {position: {x: $1, y: $2}, orientation: {w: 1.0}}}" >/dev/null 2>&1; }
# 機体（base_link）の 0.55m 前に人（半径 0.15m、高さ 0.3〜0.5m）を立たせる。
# どの点も点群の原点（モックでは base_link）から **3 次元で** 0.9m 未満なので、costmap には
# 載らない（obstacle_min_range）→ **Collision Monitor だけが止める**状況になる。
# ⚠️ 高さを 0.9m まで取ると 3 次元の距離が 0.9m を超えて costmap に載り、RPP が先に止める
# （2026-10-31 の 1 回目がこれで、Collision Monitor を試せていなかった）。
# ⚠️⚠️ **点群に 1 秒の空白も作らないこと**（2026-10-31 に踏んだ）。g1_cmd_router は
# /g1/cloud_dense が 1 秒途切れると sensor_stale で FAULT にする。fake_sensor_publisher を
# 止めてから人を出す／人が消えてから戻すと、Python の起動（2〜3 秒）の間に途切れる。
# だから **人を先に出してから** fake を止め、**人が消える 4 秒前に** fake を戻す
# （重なっている間は空の点群と人が交互に届き、Collision Monitor は止まったり
#  進んだりする。実機の「1 点だけのフレーム」と同じ状況で、cloud_gate が無いとこうなる）。
person() { setsid nohup python3 /w/tools/mock_person.py --x 0.55 --y 0.0 --frame base_link \
               --radius 0.15 --z-min 0.3 --z-max 0.5 --seconds "$1" >/tmp/person.log 2>&1 &
           sleep 3; pkill -f fake_sensor_publishe[r].py 2>/dev/null; }
sensor_back() { setsid nohup ros2 run g1_navigation fake_sensor_publisher.py >/tmp/fake_sensor.log 2>&1 & }
ts() { date +%s; }
INNER

cat > "$WORK/basic.sh" <<'INNER'
source /out/common.sh
start_stack /out/params.yaml

echo
echo "① 起動"
for n in map_server controller_server planner_server behavior_server bt_navigator \
         velocity_smoother collision_monitor; do
    printf "   %-18s %s\n" "$n" "$(timeout 10 ros2 lifecycle get /$n 2>&1 | head -1)"
done
echo "   BT: $(timeout 10 ros2 param get /bt_navigator default_nav_to_pose_bt_xml 2>&1 | sed 's/.*: //')"
echo "   behavior_plugins: $(timeout 10 ros2 param get /behavior_server behavior_plugins 2>&1 | tr -d '\n' | sed 's/.*://')"
echo "   cmd_router の入力: $(grep -aoE '/cmd_vel_(safe|smoothed) を[^ ]*' /tmp/nav.log | head -1)"

echo
echo "② 人の前で止まり、FAULT にならずに再開するか"
timeout 20 ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
echo "   Goal 前: $(state)"
goal -3.0 -1.0           # 出発点 (-3,-4) から 3m 北
sleep 4
echo "   歩行中: $(state)  smoothed vx=$(vx /cmd_vel_smoothed)  safe vx=$(vx /cmd_vel_safe)"
T0=$(ts)
person 20                # 人は 20 秒。fake を止めるまでに 3 秒使う
echo "   人が居る(+$(( $(ts) - T0 ))s): $(state)  smoothed vx=$(vx /cmd_vel_smoothed)  safe vx=$(vx /cmd_vel_safe)"
echo "   ↑ smoothed が正で safe が 0 なら Collision Monitor が止めている"
while [ $(( $(ts) - T0 )) -lt 15 ]; do sleep 1; done
echo "   人が居る(+$(( $(ts) - T0 ))s): $(state)   ← NAVIGATING のままなら合格（以前は約 3 秒で FAULT）"
sensor_back              # 人が消える 4 秒前に空の点群を戻す
while [ $(( $(ts) - T0 )) -lt 45 ]; do sleep 1; done
echo "   人が去った後(+$(( $(ts) - T0 ))s): $(state)"
grep -aE "Reached the goal|Goal succeeded|Aborting|Robot to stop|Robot to continue" /tmp/nav.log \
    | sed 's/.*\] //' | tail -5 | sed 's/^/     /'
grep -aE "cmd_timeout|FAULT|fault|途絶|失われた|collision ahead" /tmp/nav.log | tail -5 | sed 's/^/     /'
stop_all
INNER

cat > "$WORK/wait.sh" <<'INNER'
source /out/common.sh
start_stack /out/params.yaml
echo
echo "③ 人が長く立ちふさがり、BT が Wait まで進んでも FAULT にならないか"
echo "   （進捗なしで FollowPath が失敗 → 消去してやり直し → … → Wait。約 90 秒かかる）"
timeout 20 ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
goal -3.0 -1.0
sleep 3
T0=$(ts)
person 120
for t in 30 60 90 112; do
    while [ $(( $(ts) - T0 )) -lt $t ]; do sleep 1; done
    echo "   +${t}s: $(state)  Wait の実行回数=$(grep -ac 'Running wait' /tmp/nav.log)"
done
sensor_back              # 人が消える前に空の点群を戻す
while [ $(( $(ts) - T0 )) -lt 150 ]; do sleep 1; done
echo "   人が去った後: $(state)"
echo "   Wait の実行回数: $(grep -ac 'Running wait' /tmp/nav.log)   ← 1 以上でないと試験不成立"
grep -aE "Reached the goal|Goal succeeded|Aborting|failed|Running wait" /tmp/nav.log \
    | sed 's/.*\] //' | tail -6 | sed 's/^/     /'
grep -aE "cmd_timeout|FAULT|Nav2 が死んだ" /tmp/nav.log | tail -3 | sed 's/^/     /'
stop_all
INNER

cat > "$WORK/clearing.sh" <<'INNER'
source /out/common.sh
for v in params params_voxel; do
    start_stack "/out/$v.yaml"
    # 実機の MID-360 と同じ高さにセンサーを置く（モックには base_link→livox_frame が無い）
    setsid nohup ros2 run tf2_ros static_transform_publisher 0 0 1.21 0 0 0 base_link livox_frame \
        >/dev/null 2>&1 &
    pkill -f fake_sensor_publishe[r].py 2>/dev/null
    sleep 2
    echo "   $v: $(timeout 40 python3 /w/tools/mock_clearing_probe.py 2>&1 | grep -a '\[clearing\]')"
    sensor_back
    stop_all
done
echo "   ↑ params（ObstacleLayer）で「消えた」、params_voxel（旧 VoxelLayer）で「残った」なら合格"
INNER

run() {
    docker run --rm --network host --ipc host -e ROS_DOMAIN_ID="$DOMAIN" \
        -v "$HERE:/w:ro" -v "$WORK:/out" "$IMAGE" bash "$1"
}
case "$MODE" in
    basic)    echo "########## ①② ##########"; run /out/basic.sh
              echo; echo "########## ④ ##########"; run /out/clearing.sh ;;
    stop)     echo "########## ①② ##########"; run /out/basic.sh ;;
    wait)     echo "########## ③ ##########"; run /out/wait.sh ;;
    clearing) echo "########## ④ ##########"; run /out/clearing.sh ;;
esac
