#!/usr/bin/env bash
# PC2上で実行する。ROS 2 foxy が Unitree のトピックを掴めるかを切り分ける。
#
# 2026-09-02 の検証には穴があった。既定RMW(fastrtps)で試したときに `ros2 daemon` が
# 起動し、そのあと RMW_IMPLEMENTATION を変えても**壊れたfastrtpsデーモンに問い合わせて
# いた**可能性が高い。--no-daemon を試していなかった。ここではまずデーモンを落とす。
#
#   ssh g1 'bash ~/mapping_tools/diagnose_ros2.sh'
#
# 判定:
#   1 が通る            -> ROS 2 は正常。rosbridge / RViz2 が使える
#   1 は空だが 2 が通る -> グラフだけ見えない。直接購読は通るので RViz2 は動く可能性が高い
#   1 も 2 も駄目       -> ROS 2 経路は使えない。unitree_sdk2py 直接購読へ退避

set -u

IFACE="${G1_IFACE:-eth0}"
POINTS_TOPIC="${POINTS_TOPIC:-/utlidar/cloud_livox_mid360}"

echo "=============================================================="
echo " ROS 2 (foxy) 疎通診断   iface=$IFACE"
echo "=============================================================="

if [ ! -f /opt/ros/foxy/setup.bash ]; then
    echo "[FAIL] /opt/ros/foxy/setup.bash がありません"
    exit 1
fi
# ROS の setup.bash は未定義変数を前提にしているので、ここだけ set -u を外す
# shellcheck disable=SC1091
set +u
source /opt/ros/foxy/setup.bash
set -u

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 放置すると eth0/docker0/wlan0 から arbitrarily 選ばれる（2026-09-02に実際に出た）
# foxy の cyclonedds は 0.7.0。<Interfaces> は 0.8+ の構文で、0.7 では
# "unknown element" になりドメイン作成ごと失敗する。0.7 は NetworkInterfaceAddress。
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${IFACE}</NetworkInterfaceAddress></General></Domain></CycloneDDS>"

echo
echo "--- 前提 ---"
echo -n "  rmw_cyclonedds_cpp: "
[ -f /opt/ros/foxy/lib/librmw_cyclonedds_cpp.so ] && echo "あり" || echo "**無し**"
echo -n "  rviz2             : "
[ -x /opt/ros/foxy/lib/rviz2/rviz2 ] && echo "あり" || echo "無し"
echo -n "  rosbridge         : "
[ -d /opt/ros/foxy/share/rosbridge_server ] && echo "あり" || echo "無し（未導入）"
echo    "  NIC               : $(ip -br addr show dev "$IFACE" 2>/dev/null || echo '取得不可')"

echo
echo "--- 0. 壊れている可能性のあるデーモンを落とす ---"
ros2 daemon stop >/dev/null 2>&1
pkill -f "[_]ros2_daemon" >/dev/null 2>&1
sleep 1
echo "  停止した"

echo
echo "--- 1. ros2 topic list --no-daemon（グラフが見えるか）---"
LIST=$(timeout 20 ros2 topic list --no-daemon 2>&1)
if echo "$LIST" | grep -qE '\[ERROR\]|Unknown error|unknown element'; then
    echo "$LIST" | sed 's/^/  /' | head -8
    echo "  **エラー**（トピック一覧を取得できていない）"
    STEP1=fail
elif [ -z "$(echo "$LIST" | grep -v '^\s*$' | grep -v 'using network interface')" ]; then
    echo "  **空**（ROSグラフに何も出ていない）"
    STEP1=fail
else
    echo "$LIST" | sed 's/^/  /'
    STEP1=pass
fi

echo
echo "--- 2. 型を明示した直接購読（ros2 topic echo --no-daemon）---"
echo "    topic=$POINTS_TOPIC"
# foxy の echo には --no-daemon も --field も無い（Humble 以降の機能）。
# 点群は sensor_data QoS(best_effort) で出ているので既定の reliable では受からない。
# --no-arr で巨大な data 配列の出力を抑える。
ECHO=$(timeout 15 ros2 topic echo "$POINTS_TOPIC" \
        sensor_msgs/msg/PointCloud2 --qos-profile sensor_data --no-arr 2>&1 | head -20)
if echo "$ECHO" | grep -qE '^(width|height|point_step):'; then
    echo "$ECHO" | sed 's/^/  /'
    STEP2=pass
else
    echo "  受信できず:"
    echo "$ECHO" | sed 's/^/    /'
    STEP2=fail
fi

echo
echo "--- 3. ros2 node list --no-daemon ---"
timeout 15 ros2 node list --no-daemon 2>&1 | sed 's/^/  /' | head -10

echo
echo "--- 参考: unitree_sdk2py の直接購読（これは動くことが実証済み）---"
if [ -f "$HOME/mapping_tools/probe_dds_topics.py" ]; then
    # ROS を source した環境では pip 版 cyclonedds が ROS の libddsc 0.7 を掴んで
    # undefined symbol: ddsi_sertype_v0 で落ちる。素の環境で回す。
    env -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CYCLONEDDS_URI \
        -u RMW_IMPLEMENTATION -u PYTHONPATH \
        python3 "$HOME/mapping_tools/probe_dds_topics.py" 4 "$IFACE" 2>&1 | tail -6 | sed 's/^/  /'
else
    echo "  probe_dds_topics.py が無いので省略"
fi

echo
echo "=============================================================="
echo " 判定"
echo "=============================================================="
if [ "$STEP1" = pass ]; then
    echo "  [OK] ROS 2 が正常にトピックを掴めている"
    echo "       -> rosbridge_server を立てて Foxglove から見られる"
    echo "       -> RViz2 も使える"
elif [ "$STEP2" = pass ]; then
    echo "  [PARTIAL] グラフ一覧は空だが、型を明示した購読は通る"
    echo "       -> RViz2 / rosbridge は動く可能性が高い（購読は別経路のため）"
    echo "       -> まず rosbridge を試す価値がある"
else
    echo "  [NG] ROS 2 経路は使えない"
    echo "       -> unitree_sdk2py の直接購読へ退避する（stream_points.py 案）"
fi
