#!/usr/bin/env bash
# 「人が経路を横切ったときに巡回が終わってしまう」問題をモックで再現し、
# 対処（`failure_tolerance`）が効くことを確認する。**実機は要らない。**
#
# ## 何を確かめるのか
#
# 2026-09-24 の実機で、人（か地図のノイズ）が経路に入ると:
#
#     RPP が collision ahead → **0.3 秒**で follow_path abort → BT の復帰動作へ
#       → 指令が 1 秒以上途切れる → cmd_timeout → **FAULT** → Goal 取り消し
#       → 人が /g1/clear_fault を叩くまで停止
#
# 対処は `failure_tolerance` を 0.3 → 10.0 にすること。**粘っている間、controller は
# ゼロ速度を出し続ける**はずなので、指令が途切れず FAULT に落ちない、という読み。
# ⚠️ **この「ゼロ速度を出し続ける」が本当かを確かめるのが、このテストの主目的。**
#
# ## 使い方（ホストから）
#
#     cd <repo>/Navigation/nav2_stable
#     ./tools/mock_obstacle_test.sh old    # failure_tolerance=0.3（FAULT に落ちるはず）
#     ./tools/mock_obstacle_test.sh new    # failure_tolerance=10.0（落ちないはず）
#
# ## 判定
#
# | 見るもの | old の期待 | new の期待 |
# |---|---|---|
# | `bridge_status` | **FAULT**（cmd_timeout） | **NAVIGATING のまま** |
# | 人が居る間の `/cmd_vel_smoothed` | **途切れる** | **来続ける（ゼロ速度）** |
#
# ⚠️ はまりどころは mock_deadlock_test.sh と同じ（コンテナ内ビルド /
#    heartbeat_required:=false / set -u を使わない）。
#
# ## ⚠️⚠️ 未完成（2026-09-24 時点）。**まだ「人」を costmap に載せられていない**
#
# 3 回試して、いずれも **lethal 0 セル / `collision ahead` 0 回**。点群自体は
# 9.9〜19.8Hz で流れているのに、local costmap がマークしない。つまり
# **この試験はまだ何も検証していない。** 分かっているのは:
#
# - 人を出すのが遅いと**ゴールに着いてしまう**（1回目）→ 5 秒後に出すよう直した
# - `fake_sensor_publisher` が同じトピックへ空の点群を流すので、最新観測を
#   上書きしうる（2回目）→ 止めるようにした。**それでも載らない**
#
# 📌 2026-09-24 追記: 5 回目で **lethal 209 セル**まで行った回があるので、
# 載る条件はある（再現が不安定）。また **人の配信が終わると sensor_stale で FAULT**
# に落ちるため、fake_sensor_publisher を戻すようにした（⚠️ この戻しは未検証）。
#
# 次に疑うところ: observation source の `sensor_frame` と `obstacle_min_range`
# （センサー原点からの距離で切られる）、点群の frame_id を `map` にしている点、
# voxel layer の z 範囲。**`ros2 topic echo /local_costmap/costmap` を直接見るのが早い。**
set -o pipefail

MODE="${1:-new}"
# none = 人を出さずに Goal だけ流す。**D1(Goal が無い間は cmd_timeout を数えない)の確認用。**
# 到達後に FAULT に落ちないことを見る。
case "$MODE" in old|new|none) ;; *) echo "使い方: $0 [old|new|none]" >&2; exit 2 ;; esac

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # nav2_stable
IMAGE="${G1_MOCK_IMAGE:-g1-mapping-visualization:local}"
WORK="${G1_MOCK_WORK:-/tmp/g1_mock_obstacle}"
DOMAIN="${G1_MOCK_DOMAIN:-89}"   # 実機とも deadlock テストとも混ざらない番号
mkdir -p "$WORK"

if [ ! -x "$WORK/build/g1_sdk_bridge_mock_server" ]; then
    echo "[mock] コンテナ内でビルドする"
    docker run --rm -v "$HERE:/w:ro" -v "$WORK:/out" "$IMAGE" bash -c '
        cmake -S /w/g1_sdk_bridge_cpp -B /out/build -DG1_SDK_BRIDGE_BUILD_TESTS=OFF >/dev/null &&
        cmake --build /out/build --target g1_sdk_bridge_mock_server -j4 >/dev/null' \
        || { echo "[mock] ビルドに失敗した" >&2; exit 1; }
fi

# ⚠️ 本物の nav2_params.yaml から生成する（設定を二重に持たない）
PARAMS_SRC="$HERE/g1_ws/src/g1_navigation/config/nav2_params.yaml"
if [ "$MODE" = old ]; then
    sed -e 's/^    failure_tolerance: .*/    failure_tolerance: 0.3/' "$PARAMS_SRC" > "$WORK/params.yaml"
    LABEL="旧設定 failure_tolerance=0.3（FAULT に落ちるはず）"
else
    cp "$PARAMS_SRC" "$WORK/params.yaml"
    LABEL="新設定 failure_tolerance=10.0（落ちないはず）"
fi
export G1_MOCK_NO_PERSON=0
[ "$MODE" = none ] && { export G1_MOCK_NO_PERSON=1; LABEL="人なし（Goal 到達後に FAULT に落ちないことの確認）"; }

cat > "$WORK/run.sh" <<'INNER'
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
    params_file:=/out/params.yaml heartbeat_required:=false >/tmp/nav_mock.log 2>&1 &
sleep 55
grep -E "^    failure_tolerance:" /out/params.yaml | sed 's/^/  /'

timeout 20 ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
state() { timeout 8 ros2 topic echo /g1/bridge_status --once 2>/dev/null \
          | grep -E "^  message:" | head -1 | awk '{print $2}'; }
echo "  Goal 前: $(state)"
timeout 15 ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: -3.0, y: -2.0}, orientation: {w: 1.0}}}" >/dev/null 2>&1
# ⚠️ **人を出すのが遅いとゴールに着いてしまう**(2026-09-24 の1回目がこれ)。
# モックは 0.28m/s で歩くので、3.6m の Goal には 13 秒ほどで着く。
sleep 5
echo "  歩行中: $(state)"

# --- ここで「人」が経路上（機体の少し先）に立つ ---------------------------
# ⚠️⚠️ **fake_sensor_publisher を止めてから置くこと**(2026-09-24 に踏んだ)。
# あちらは同じトピックへ**空の点群**を 10Hz で流し続ける。costmap の
# observation は既定で「最新の1件」しか見ないので、空の観測が人の観測を
# 上書きしてしまい、**点群は 19.8Hz 流れているのに lethal 0 セル**になる。
if [ "${G1_MOCK_NO_PERSON:-0}" = 1 ]; then
    echo "  --- 人は出さない（Goal 到達だけを見る）---"
else
    pkill -f fake_sensor_publishe[r].py 2>/dev/null
    echo "  --- 人が経路上に立つ（15秒。fake_sensor_publisher は止めた）---"
    setsid nohup python3 /w/tools/mock_person.py --x -2.2 --y -1.5 --frame map \
        --seconds 15 >/tmp/person.log 2>&1 &
fi
# ⚠️ **まず「人が本当に costmap に載ったか」を確かめる。** 載っていなければ
# この試験は何も試していないことになる(2026-09-24 の2回目がこれだった)。
sleep 4
P=$(timeout 6 ros2 topic hz /utlidar/cloud_livox_mid360 2>&1 | grep -oE "average rate: [0-9.]+" | head -1)
echo "  点群の配信: ${P:-来ていない}"
echo "  lethal セル数: $(timeout 12 python3 /w/tools/count_costmap.py 2>&1 | tail -2 | tr '\n' ' ')"
N=$(timeout 6 ros2 topic hz /cmd_vel_smoothed 2>&1 | grep -oE "average rate: [0-9.]+" | head -1)
echo "  人が居る間の /cmd_vel_smoothed: ${N:-来ていない}"
echo "  collision ahead の回数: $(grep -ac 'collision ahead' /tmp/nav_mock.log)"
echo "  人が居る間の状態: $(state)"
sleep 8
echo "  人が居る間の状態(後半): $(state)"

# ⚠️ **fake_sensor_publisher を止めっぱなしにしない**(2026-09-24 に踏んだ)。
# 人の配信が終わるとセンサートピックが無音になり、**sensor_stale で FAULT** に落ちる。
# 「人のせいで落ちた」ように見えるが、原因は試験側にある。
if [ "${G1_MOCK_NO_PERSON:-0}" != 1 ]; then
    setsid nohup ros2 run g1_navigation fake_sensor_publisher.py >/tmp/fake_sensor.log 2>&1 &
fi
sleep 8
echo "  --- 人が通り過ぎた後 ---"
echo "  状態: $(state)"
grep -aE "Reached the goal|Goal succeeded|Aborting handle|collision|patience" /tmp/nav_mock.log \
    | tail -4 | sed 's/^/  /'
grep -aE "cmd_timeout|FAULT" /tmp/nav_mock.log | tail -3 | sed 's/^/  /'

pkill -f "lib/nav[2]_" 2>/dev/null; pkill -f g1_sdk_bridge_mock_serve[r] 2>/dev/null
pkill -f "ros2 launc[h]" 2>/dev/null; pkill -f mock_perso[n].py 2>/dev/null
INNER

echo "########## $LABEL ##########"
docker run --rm --network host --ipc host -e ROS_DOMAIN_ID="$DOMAIN" \
    -e G1_MOCK_NO_PERSON="$G1_MOCK_NO_PERSON" \
    -v "$HERE:/w" -v "$WORK:/out" "$IMAGE" bash /out/run.sh
