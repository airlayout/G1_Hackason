#!/usr/bin/env bash
# 旋回デッドロック（2026-09-15 に実機で踏んだもの）をモックで再現し、
# 対処が効くことを確認する。**実機は要らない。操作PC の Docker だけで完結する。**
#
# ## 何を確かめるのか
#
# RPP の rotate-to-heading は「**実測角速度 + max_angular_accel × dt**」で指令を作る。
# 二足は指令が小さすぎると歩容が成立せず動かないので、実測が 0 のまま固まると
# 指令が 1 周期ぶんの増分から永久に増えない:
#
#     0 + 0.40 × 0.05 = 0.02 rad/s   ← 実機で 2,363 周期この値が続き、機体は 4mm も動かなかった
#     0 + 6.00 × 0.05 = 0.30 rad/s   ← 歩容の閾値を超えるので回り出せる
#
# モックに「指令が閾値未満なら動かない」性質(`--like-g1`)を入れると、この現象を
# 物理シミュレータ無しで再現できる。**必要なのは物理ではなくフィードバックの構造**だから。
#
# ## 使い方（ホストから）
#
#     cd <repo>/Navigation/nav2_option
#     ./tools/mock_deadlock_test.sh old    # 再現する（動かないことを確認）
#     ./tools/mock_deadlock_test.sh new    # 直る（Goal に到達することを確認）
#
# ## ⚠️ はまりどころ（2026-09-16 に全部踏んだ）
#
# 1. **モックはコンテナ内でビルドすること。** ホストで焼いたバイナリは
#    `GLIBCXX_3.4.32 not found` で動かない（ホストの方が新しい）。
#    「起動したつもりで起動していない」＝ TF が出ないだけ、という分かりにくい形で出る
# 2. **`heartbeat_required:=false` が要る。** 既定 true だと `enable_navigation` が
#    拒否され、cmd_router はゼロしか転送しないのでモックが動かない（設計どおりの動作）
# 3. **`set -u` を使わないこと。** ROS の `setup.bash` が未定義変数を参照して落ちる
set -o pipefail

MODE="${1:-new}"
case "$MODE" in old|new) ;; *) echo "使い方: $0 [old|new]" >&2; exit 2 ;; esac

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # nav2_option
IMAGE="${G1_MOCK_IMAGE:-g1-mapping-visualization:local}"
WORK="${G1_MOCK_WORK:-/tmp/g1_mock_test}"
DOMAIN="${G1_MOCK_DOMAIN:-88}"   # 実機と混ざらないよう既定を分けてある
mkdir -p "$WORK"

# --- モックをコンテナ内でビルドする（はまりどころ1）-------------------------
if [ ! -x "$WORK/build/g1_sdk_bridge_mock_server" ]; then
    echo "[mock] コンテナ内でビルドする（ホストのバイナリは glibc が合わず動かない）"
    docker run --rm -v "$HERE:/w:ro" -v "$WORK:/out" "$IMAGE" bash -c '
        cmake -S /w/g1_sdk_bridge_cpp -B /out/build -DG1_SDK_BRIDGE_BUILD_TESTS=OFF >/dev/null &&
        cmake --build /out/build --target g1_sdk_bridge_mock_server -j4 >/dev/null' \
        || { echo "[mock] ビルドに失敗した" >&2; exit 1; }
fi

# --- 比較用の「旧設定」をその場で作る（設定を二重に持たない）----------------
# ⚠️ 本物の nav2_params.yaml から生成する。コピーを置くと必ず片方だけ直して食い違う。
PARAMS_SRC="$HERE/g1_ws/src/g1_navigation/config/nav2_params.yaml"
if [ "$MODE" = old ]; then
    sed -e 's/max_angular_accel: 6.0/max_angular_accel: 0.40/' \
        -e 's/nav2_controller::PoseProgressChecker/nav2_controller::SimpleProgressChecker/' \
        -e 's/^      required_movement_angle:.*/      # (旧設定には無い)/' \
        -e 's/movement_time_allowance: 20.0.*/movement_time_allowance: 10.0/' \
        "$PARAMS_SRC" > "$WORK/params.yaml"
    LABEL="旧設定 max_angular_accel=0.40 + SimpleProgressChecker（再現するはず）"
else
    cp "$PARAMS_SRC" "$WORK/params.yaml"
    LABEL="新設定 max_angular_accel=6.0 + PoseProgressChecker（直るはず）"
fi

cat > "$WORK/run.sh" <<'INNER'
set -o pipefail   # ⚠️ set -u は ROS の setup.bash を壊す
source /opt/ros/humble/setup.bash
source /w/g1_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
pkill -f g1_sdk_bridge_mock_serve[r] 2>/dev/null; pkill -f "lib/nav[2]_" 2>/dev/null
pkill -f "ros2 launc[h]" 2>/dev/null; sleep 3
rm -rf /tmp/g1_bridge; mkdir -p /tmp/g1_bridge

# --like-g1 = 実機に近い最小作動閾値(vx 0.2 / omega 0.15)。これが無いと再現しない
setsid nohup /out/build/g1_sdk_bridge_mock_server --like-g1 >/tmp/mock_sdk.log 2>&1 &
sleep 2
grep -a "最小作動閾値" /tmp/mock_sdk.log | sed 's/^/  /'
# ⚠️ heartbeat_required:=false が要る(はまりどころ2)
setsid nohup ros2 launch g1_navigation navigation.launch.py backend:=mock \
    params_file:=/out/params.yaml heartbeat_required:=false >/tmp/nav_mock.log 2>&1 &
sleep 55
grep -a "Created progress_checker" /tmp/nav_mock.log | sed 's/.*type /  checker: /'
grep -E "^      max_angular_accel:" /out/params.yaml | sed 's/^/  /'

timeout 20 ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
pose() { timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 \
         | grep -E "Translation|degree" | head -2 | tr '\n' ' '; }
echo "  出発: $(pose)"
timeout 15 ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: -3.0, y: -2.0}, orientation: {w: 1.0}}}" >/dev/null 2>&1
for t in 20 40 60; do sleep 20; echo "  ${t}s: $(pose)"; done
echo "  --- /cmd_vel_smoothed の wz ---"
timeout 15 ros2 topic echo /cmd_vel_smoothed 2>/dev/null | grep -A3 angular | grep " z:" \
    | awk '{printf "%.3f\n", $2}' | sort | uniq -c | sort -rn | head -3 | sed 's/^/  /'
echo "  --- 結末 ---"
grep -aE "Reached the goal|Goal succeeded|Aborting handle" /tmp/nav_mock.log | tail -3 | sed 's/^/  /'
pkill -f "lib/nav[2]_" 2>/dev/null; pkill -f g1_sdk_bridge_mock_serve[r] 2>/dev/null
pkill -f "ros2 launc[h]" 2>/dev/null
INNER

echo "########## $LABEL ##########"
docker run --rm --network host --ipc host -e ROS_DOMAIN_ID="$DOMAIN" \
    -v "$HERE:/w" -v "$WORK:/out" "$IMAGE" bash /out/run.sh
