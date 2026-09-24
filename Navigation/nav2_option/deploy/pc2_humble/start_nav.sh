#!/usr/bin/env bash
# §5 の launch を PC2 の pixi(Humble) 環境で上げる。
#
# 第1引数: map_to_odom。"dx dy yaw[rad]" か `none`(連続localization を併用するとき)。
# 第2引数: lidar_yaw[度]。既定 0。
#   ⚠️ 2026-09-15 に水平化を heading_preserving_leveling に替えたので、yaw は
#   構成上ずれなくなった。それでも RViz で base_link の赤軸(前方)が実機の正面と
#   一致しているかは毎回目視すること。
#
# ⚠️ **CycloneDDS を使う**(2026-09-15 に FastDDS から変更)。D-03 は FastDDS を
# 指定しているが、**PC2 の FastDDS は点群(441KB/フレーム)を受け取れない**。
# net.core.rmem_max が既定の 212992 のままだと落ちる。sudo で広げれば直るが、
# 権限を要らなくするため CycloneDDS を選んだ(同条件で受信できることを実測)。
# A-10e で問題になった「Twist/TwistStamped 同時購読でクラッシュ」は実行時型判定に
# 直してあるので、CycloneDDS でも成立する。
set -u
MAP2ODOM="${1:-0 0 0}"
LIDAR_YAW="${2:-0}"
# 第3引数: operator_timeout_s。heartbeat が何秒途絶したら止めるか(D-31)。
# ⚠️ 2026-09-15 実測: **有線なら 1.0 で足りるが、スマホのテザリングでは足りない**。
# WiFi 省電力を切った上でも heartbeat の age が最大 0.628 秒まで伸び(60秒間で
# 0.5 秒超えが 3 回)、既定の 1.0 では operator_lost の誤作動が起きた
# (実際に 1 回、機体が一歩も動かないまま FAULT になった)。
# 代償: 検知が遅れるぶん、通信断からの停止までの前進距離が伸びる
# (max_vx=0.30 なので +1 秒 ≒ +0.3m)。D-31 の受入目標 0.35m はこの値では満たせない。
OP_TIMEOUT="${3:-1.0}"
# 第4引数: 巡回路の yaml。空なら巡回ノードは常駐するが**ウェイポイント0点で start を断る**
# （＝従来どおりの単純ゴール指定モードだけが使える）。現地で
# `tools/record_waypoints.py` を回して作ったファイルを渡すこと。
PATROL_WAYPOINTS="${4:-}"
# ⚠️ 2026-09-24（後半）に **既定を room_a へ戻した**（次も room_a で動かすため）。
# 使うのは **9/11 の記録から作り直した版**（未知 40.5%→27.4%、連結した自由空間 +10%）。
# 他の地図は第5引数で渡せる:
#   room_a_map_20260911.yaml … 手編集していない版
#   room_a_map.yaml … 9/07 の `map_20260907.pcd` 由来（旧）
# ⚠️ 既定の `_edited` は、実地の目視で「地図では塞がっているが実際は通路」と判断した
# 28 箇所(半径0.4m・計3.02m²)を開けたもの。**測ったのではなく人が判断した変更**なので、
# 何を開けたかは maps/grids/EDITS.md に残してある。
#   room_b_map_Sorasta_20260923.yaml … Sorasta。⚠️ /dog_odom 由来で壁が点線状
# ⚠️⚠️ **地図と実際の会場が違うと §7 の照合は必ず失敗する。** 会場に合わせて選ぶこと。
MAPS=/home/unitree/g1_nav2/g1_ws/install/g1_navigation/share/g1_navigation/maps
MAP="${5:-$MAPS/room_a_map_20260911_edited.yaml}"
CYCLONE_CFG=/home/unitree/g1_nav2/cyclonedds_eth0.xml
# ⚠️ **空の引数を渡してはいけない**(2026-09-24 に実機で踏んだ)。`ros2 launch` は
# `patrol_waypoints:=` を `malformed launch argument` として**起動前に**弾くため、
# 巡回路を指定しない(＝単純ゴール指定モードだけの)通常運用が丸ごと立たなかった。
# ログにも ERROR の1行しか出ないので、g1up.sh からは「校正が終わらない」に見える。
PATROL_ARG=""
[ -n "$PATROL_WAYPOINTS" ] && PATROL_ARG="patrol_waypoints:=$PATROL_WAYPOINTS"

# ⚠️ **前の Nav2 を必ず落としてから上げる**(2026-09-24 に実機で踏んだ)。
# g1up.sh は §5 で1回、§7b で map→odom を入れてもう1回このスクリプトを呼ぶので、
# **掃除しないと必ず2本立つ**。すると lifecycle_manager が2つになってノード名が衝突し、
# `map_server` / `planner_server` / `bt_navigator` が **unconfigured のまま残る**
# (controller/behavior/velocity_smoother だけ active という中途半端な状態になり、
#  Goal を投げても計画が出ない)。
pkill -f 'ros2 launch g1_navigatio[n]' 2>/dev/null || true
pkill -f 'g1_slam_odom_t[f].py'        2>/dev/null || true
pkill -f 'g1_state_bridge_nod[e]'      2>/dev/null || true
pkill -f 'g1_cmd_router_nod[e]'        2>/dev/null || true
pkill -f 'patrol_nod[e].py'            2>/dev/null || true
pkill -f 'envs/default/lib/nav[2]_'    2>/dev/null || true
sleep 3

cd /home/unitree/g1_nav2/pc2_humble
rm -f /tmp/nav_launch.log
setsid nohup ~/.pixi/bin/pixi run bash -lc "
  source /home/unitree/g1_nav2/g1_ws/install/setup.bash
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$CYCLONE_CFG
  exec ros2 launch g1_navigation navigation.launch.py backend:=real map:=$MAP \
       map_to_odom:='$MAP2ODOM' lidar_yaw:=$LIDAR_YAW \
       operator_timeout_s:=$OP_TIMEOUT $PATROL_ARG
" > /tmp/nav_launch.log 2>&1 < /dev/null &
echo "started (map_to_odom=$MAP2ODOM lidar_yaw=$LIDAR_YAW operator_timeout_s=$OP_TIMEOUT rmw=cyclonedds)"
echo "  巡回路: ${PATROL_WAYPOINTS:-(未指定。単純ゴール指定モードのみ)}"
echo "  地図  : $(basename "$MAP")"
