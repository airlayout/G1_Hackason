#!/usr/bin/env bash
# 手順書 §7 の推奨経路: 連続 localization。launch 側は map_to_odom:=none で上げること。
#
#     start_localizer.sh <dx> <dy> <yaw[rad]>     # find_map_offset.py の結果
#
# ⚠️⚠️ **初期値をこのファイルに書き込まないこと。** 2026-09-15 の値
# (-1.350 -3.850 2.7751) がハードコードで残っていて、**セッションが変われば
# 必ず間違いになる**状態だった(2026-09-16 修正)。odom 原点は 1801 を送った
# 瞬間の機体位置なので、毎回変わる。
# ⚠️ **地図を変えたら以前の値は全部無効**(2026-09-16 に 9/11 の地図へ切り替えた。
# 座標系が約10°違う)。
set -u
if [ $# -lt 3 ]; then
    echo "使い方: $0 <dx> <dy> <yaw[rad]>   # find_map_offset.py の結果を渡す" >&2
    exit 2
fi
INIT_X="$1"; INIT_Y="$2"; INIT_YAW="$3"; shift 3
# 4つ目以降はそのまま map_localizer.py へ渡す（--observe-only など）
EXTRA="$*"
cd /home/unitree/g1_nav2/pc2_humble
rm -f /tmp/localizer.log
setsid nohup ~/.pixi/bin/pixi run bash -lc "
  source /home/unitree/g1_nav2/g1_ws/install/setup.bash
  # ⚠️ **CycloneDDS を使う**(2026-09-24 修正)。Nav2 側は 2026-09-15 に FastDDS から
  # 切り替わっているのに、ここだけ fastrtps のままだった。揃っていないと
  # 「トピックは見えるのにデータが来ない」という分かりにくい壊れ方をする。
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///home/unitree/g1_nav2/cyclonedds_eth0.xml
  exec python3 /home/unitree/g1_nav2/tools/map_localizer.py --initial $INIT_X $INIT_Y $INIT_YAW $EXTRA
" > /tmp/localizer.log 2>&1 < /dev/null &
echo "started (initial=$INIT_X $INIT_Y $INIT_YAW${EXTRA:+ $EXTRA})"
