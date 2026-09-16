#!/usr/bin/env bash
# 巡回モードをモックで通しで確認する。**実機は要らない。操作PC の Docker だけで完結する。**
#
# ## 何を確かめるのか（4件）
#
# | # | 確かめること | なぜ |
# |---|---|---|
# | ① | 走れる状態でないと `start` が**断られる** | 発進ゲート/FAULT を巡回が迂回しないこと |
# | ② | `start` で**2点を順に回る** | 巡回そのもの |
# | ③ | 手動 Goal を送ると巡回が **HOLD に退く** | 2モードが喧嘩しないこと |
# | ④ | `HOLD` から**自動復帰しない** | 人の「開始」の意思を必ず挟むこと |
#
# ## 使い方（ホストから）
#
#     cd <repo>/Navigation/nav2_option
#     ./tools/mock_patrol_test.sh          # 上の4件（約4分）
#     ./tools/mock_patrol_test.sh dwell    # 各点で長く止まると FAULT になるかを測る（約3分）
#
# `dwell` は別建てにしてある。**`dwell_s` を何秒まで伸ばせるかは
# `velocity_smoother` の velocity_timeout(既定 1.0) と `cmd_timeout`(0.30) で決まり、
# 巡回そのものの出来とは別の話**だから。
#
# ⚠️ はまりどころは `mock_deadlock_test.sh` と同じ3点（コンテナ内ビルド /
#    heartbeat_required:=false / set -u 禁止）。詳細はそちらの冒頭を読むこと。
set -o pipefail

MODE="${1:-basic}"
case "$MODE" in basic|dwell) ;; *) echo "使い方: $0 [basic|dwell]" >&2; exit 2 ;; esac

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # nav2_option
IMAGE="${G1_MOCK_IMAGE:-g1-mapping-visualization:local}"
WORK="${G1_MOCK_WORK:-/tmp/g1_mock_patrol}"
DOMAIN="${G1_MOCK_DOMAIN:-89}"
mkdir -p "$WORK"

if [ ! -x "$WORK/build/g1_sdk_bridge_mock_server" ]; then
    echo "[mock] コンテナ内でビルドする（ホストのバイナリは glibc が合わず動かない）"
    docker run --rm -v "$HERE:/w:ro" -v "$WORK:/out" "$IMAGE" bash -c '
        cmake -S /w/g1_sdk_bridge_cpp -B /out/build -DG1_SDK_BRIDGE_BUILD_TESTS=OFF >/dev/null &&
        cmake --build /out/build --target g1_sdk_bridge_mock_server -j4 >/dev/null' \
        || { echo "[mock] ビルドに失敗した" >&2; exit 1; }
fi

# 試験用の短い巡回路。**本番の patrol_synthetic.yaml は1周 22m あり試験には長すぎる。**
# 出発点(-3,-4) のすぐ北に2点だけ置く。どちらも旋回を伴う。
cat > "$WORK/waypoints.yaml" <<'WP'
frame_id: map
waypoints:
  - {name: a, x: -3.0, y: -2.5, yaw_deg: 90, dwell_s: 1.0}
  - {name: b, x: -4.3, y: -2.5, yaw_deg: 180, dwell_s: 1.0}
WP

# ⚠️⚠️ **dwell モード専用に、点ごとの `dwell_s` を書かない版を別に作る。**
# 点ごとの値はノードのパラメータより**優先される**。上の waypoints.yaml を
# 使い回すと `patrol_dwell_s:=5.0` が黙って無視され、**「FAULT が再現しない＝
# 問題は無い」という真逆の結論になる**（2026-09-16 に実際にそうなった）。
cat > "$WORK/waypoints_nodwell.yaml" <<'WP'
frame_id: map
waypoints:
  - {name: a, x: -3.0, y: -2.5, yaw_deg: 90}
  - {name: b, x: -4.3, y: -2.5, yaw_deg: 180}
WP

cat > "$WORK/dwell.sh" <<'INNER'
# 各点で長く止まったときに cmd_timeout(0.30秒) で FAULT にならないか。
#
# ⚠️ Goal が終わると `controller_server` は指令を出さなくなる。`velocity_smoother` は
# velocity_timeout(既定 1.0 秒)ぶん余分に出し続けてからゼロへ落として黙る。そこから
# 0.30 秒で `cmd_timeout` → FAULT。つまり**止まっていられるのは 1.3 秒ほどしかない**。
set -o pipefail
source /opt/ros/humble/setup.bash
source /w/g1_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
pkill -f g1_sdk_bridge_mock_serve[r] 2>/dev/null; pkill -f "lib/nav[2]_" 2>/dev/null
pkill -f "ros2 launc[h]" 2>/dev/null; sleep 3
rm -rf /tmp/g1_bridge; mkdir -p /tmp/g1_bridge
setsid nohup /out/build/g1_sdk_bridge_mock_server --like-g1 >/tmp/mock_sdk.log 2>&1 &
sleep 2
setsid nohup ros2 launch g1_navigation navigation.launch.py backend:=mock \
    heartbeat_required:=false patrol_waypoints:=/out/waypoints_nodwell.yaml \
    patrol_dwell_s:="${G1_DWELL:-5.0}" >/tmp/nav_dwell.log 2>&1 &
sleep 55
timeout 20 ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
timeout 15 ros2 service call /g1/patrol/start std_srvs/srv/Trigger "{}" >/dev/null 2>&1
echo "  dwell_s=${G1_DWELL:-5.0} で巡回を開始した"
# ⚠️ 本当にその秒数が効いたかを裏取りする。点ごとの dwell_s に負けていないこと
grep -a "秒待つ" /tmp/nav_dwell.log | tail -1 | sed 's/.*INFO[^]]*]//' | sed 's/^/   効いた dwell: /'
grep -a "待ち時間が" /tmp/nav_dwell.log | tail -1 | sed 's/.*INFO[^]]*]//' | sed 's/^/  /' 
for t in 30 60 90; do
    sleep 30
    B=$(timeout 5 ros2 topic echo --once /g1/bridge_status diagnostic_msgs/msg/DiagnosticArray 2>/dev/null | sed -n 's/^  message: //p')
    P=$(timeout 5 ros2 topic echo --once --full-length /g1/patrol/status std_msgs/msg/String 2>/dev/null | sed -n 's/^data: //p')
    echo "  ${t}s bridge=$B"
    echo "       $P"
done
echo "  --- fault_reason ---"
timeout 5 ros2 topic echo --once /g1/bridge_status diagnostic_msgs/msg/DiagnosticArray 2>/dev/null \
    | grep -A1 fault_reason | sed 's/^/  /'
pkill -f "lib/nav[2]_" 2>/dev/null; pkill -f g1_sdk_bridge_mock_serve[r] 2>/dev/null
pkill -f "ros2 launc[h]" 2>/dev/null
INNER

cat > "$WORK/run.sh" <<'INNER'
set -o pipefail
source /opt/ros/humble/setup.bash
source /w/g1_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
pkill -f g1_sdk_bridge_mock_serve[r] 2>/dev/null; pkill -f "lib/nav[2]_" 2>/dev/null
pkill -f "ros2 launc[h]" 2>/dev/null; pkill -f patrol_nod[e].py 2>/dev/null; sleep 3
rm -rf /tmp/g1_bridge; mkdir -p /tmp/g1_bridge

setsid nohup /out/build/g1_sdk_bridge_mock_server --like-g1 >/tmp/mock_sdk.log 2>&1 &
sleep 2
setsid nohup ros2 launch g1_navigation navigation.launch.py backend:=mock \
    heartbeat_required:=false patrol_waypoints:=/out/waypoints.yaml \
    patrol_dwell_s:=1.0 >/tmp/nav_patrol.log 2>&1 &
sleep 55

st()   { timeout 4 ros2 topic echo --once --full-length /g1/patrol/status std_msgs/msg/String 2>/dev/null \
         | sed -n 's/^data: //p'; }
# ⚠️ `tail -1` は空行を拾うので、応答行を明示的に抜くこと
call() { timeout 15 ros2 service call "/g1/patrol/$1" std_srvs/srv/Trigger "{}" 2>&1 \
         | sed -n "s/.*Trigger_Response(\(.*\))$/    \1/p"; }
pose() { timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 \
         | grep -E "Translation|degree" | head -2 \
         | sed -e 's/.*Translation: //' -e 's/- Rotation: in RPY (degree) //' \
         | tr '\n' ' '; }

echo
echo "① 走れる状態でないときに start を断るか（enable_navigation より前）"
echo "   期待: bridge=READY（TF/センサーは健全だが走行許可はまだ）→ **start は断られる**"
echo "   bridge: $(timeout 4 ros2 topic echo --once /g1/bridge_status diagnostic_msgs/msg/DiagnosticArray 2>/dev/null | sed -n 's/^  message: //p')"
call start
echo "   status: $(st)   ← state が IDLE のままなら合格"

echo
echo "② enable_navigation したうえで start → 2点を回るか"
timeout 20 ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
call start
echo "   出発: $(pose)"
for t in 30 60 90 120; do
    sleep 30
    echo "   ${t}s: $(pose)  $(st)"
done
echo "   到達ログ:"
grep -aE "✅ .* 到達|→ [0-9]+/[0-9]+" /tmp/nav_patrol.log | tail -6 | sed 's/^/    /'

echo
echo "③ 手動 Goal を送ると巡回が退くか"
timeout 15 ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: -3.0, y: -4.0}, orientation: {w: 1.0}}}" >/dev/null 2>&1
sleep 4
echo "   status: $(st)"

echo
echo "④ HOLD から自動復帰しないか（20秒放置）"
sleep 20
echo "   status: $(st)"

pkill -f "lib/nav[2]_" 2>/dev/null; pkill -f g1_sdk_bridge_mock_serve[r] 2>/dev/null
pkill -f "ros2 launc[h]" 2>/dev/null; pkill -f patrol_nod[e].py 2>/dev/null
INNER

if [ "$MODE" = dwell ]; then
    echo "########## 各点で止まっていられる時間 ##########"
    SCRIPT=/out/dwell.sh
else
    echo "########## 巡回モードのモック確認 ##########"
    SCRIPT=/out/run.sh
fi
docker run --rm --network host --ipc host -e ROS_DOMAIN_ID="$DOMAIN" \
    -e G1_DWELL="${G1_DWELL:-5.0}" \
    -v "$HERE:/w" -v "$WORK:/out" "$IMAGE" bash "$SCRIPT"
