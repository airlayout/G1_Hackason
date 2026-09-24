#!/usr/bin/env bash
# `map→odom` の供給元を **Nav2 を上げ直さずに** 切り替える。
#
#   ./map2odom_ctl.sh static    <dx> <dy> <yaw>   # §7 の値で固定（いまの既定の挙動）
#   ./map2odom_ctl.sh localizer <dx> <dy> <yaw>   # 連続 localization（補正を適用する）
#   ./map2odom_ctl.sh observe   <dx> <dy> <yaw>   # ⭐ 測るだけ（TF は出さない・走行に影響しない）
#   ./map2odom_ctl.sh off                         # 全部止める
#   ./map2odom_ctl.sh status                      # いま何が出しているか
#
# ## ⚠️ 使う前提: Nav2 を `map_to_odom:=none` で上げること
#
#     bash ~/g1_nav2/start_nav.sh none 0 2.0
#
# `none` を渡すと `g1_slam_odom_tf.py` が `--no-map-to-odom` で起動し、
# **map→odom を出さなくなる**。その口をこのスクリプトが埋める。
# 既定の `start_nav.sh "0 0 0"` の形（launch が静的に出す）では、
# **こちらの出力は tf2 に無視される**ので切り替えられない。
#
# ## なぜこうするのか
#
# 従来は §7 の値を反映するのに **Nav2 ごと上げ直していた（20〜25秒）**。
# しかも巡回ノードも一緒に上がり直すので `IDLE` に戻り、巡回が途切れる。
# map→odom を外に出しておけば、**数秒で差し替えられる**。
#
# ## ⚠️ 同時に2つ出さないこと
#
# 静的と動的の両方が map→odom を出すと、tf2 は**静的側を常に最新として扱う**ため
# **補正が一切効かない**（エラーも出ない）。このスクリプトは切り替え時に必ず
# 相手を止める。`observe` だけは TF を出さないので、静的と**併走してよい**。
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G1_DIR="${G1_NAV2_DIR:-/home/unitree/g1_nav2}"
PIXI="${PIXI:-$HOME/.pixi/bin/pixi}"
CYCLONE_CFG="$G1_DIR/cyclonedds_eth0.xml"
LOG_DIR="${G1_LOG_DIR:-/tmp}"

# ⚠️ **CycloneDDS を使う。** Nav2 側が 2026-09-15 に FastDDS から変更された。
# 揃っていないと「トピックは見えるのにデータが来ない」で、原因が分かりにくい。
run_in_env() {   # run_in_env <ログ名> <コマンド...>
    local name="$1"; shift
    cd "$G1_DIR/pc2_humble" || return 1
    rm -f "$LOG_DIR/$name.log"
    setsid nohup "$PIXI" run bash -lc "
      source $G1_DIR/g1_ws/install/setup.bash
      export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export CYCLONEDDS_URI=file://$CYCLONE_CFG
      exec $*
    " > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
}

# ⚠️ pkill のパターンは括弧付きのまま使う（このスクリプト自身に当たらないため）
kill_static()    { pkill -f 'map_to_odom_ct[l]_static' 2>/dev/null; }
kill_localizer() { pkill -f 'map_localize[r].py' 2>/dev/null; }

need_xyz() {
    [ $# -ge 3 ] || { echo "使い方: $0 $CMD <dx> <dy> <yaw[rad]>   # find_map_offset.py の結果" >&2; exit 2; }
}

CMD="${1:-status}"; shift || true

case "$CMD" in
  static)
    need_xyz "$@"
    kill_localizer; kill_static; sleep 1
    run_in_env map2odom_static \
      "ros2 run tf2_ros static_transform_publisher --ros-args -r __node:=map_to_odom_ctl_static \
       -- $1 $2 0 $3 0 0 map odom"
    echo "static: map→odom = $1 $2 $3 を固定で出す"
    ;;
  localizer)
    need_xyz "$@"
    kill_static; kill_localizer; sleep 1
    run_in_env map2odom_localizer \
      "python3 $G1_DIR/tools/map_localizer.py --initial $1 $2 $3"
    echo "localizer: 連続 localization（2Hz で照合し map→odom を更新）"
    echo "  ⚠️ 実機では一度も使っていない。RViz で位置が飛ばないか見ること"
    ;;
  observe)
    need_xyz "$@"
    # ⭐ TF を出さないので、static と**併走してよい**（止めない）
    kill_localizer; sleep 1
    CSV="$G1_DIR/runs/drift_$(date +%Y%m%d_%H%M).csv"
    run_in_env map2odom_observe \
      "python3 $G1_DIR/tools/map_localizer.py --initial $1 $2 $3 \
       --observe-only --log-csv $CSV"
    echo "observe: 測るだけ（TF は出さない）。走行には影響しない"
    echo "  記録: $CSV"
    ;;
  off)
    kill_static; kill_localizer
    echo "止めた。⚠️ **map→odom を誰も出していない状態**になる"
    echo "   TF が途切れると g1_cmd_router が tf_stale で FAULT にする"
    ;;
  status)
    echo "--- 出している側 ---"
    pgrep -af 'map_to_odom_ct[l]_static' >/dev/null && echo "  static: 動いている" || echo "  static: 止まっている"
    if pgrep -af 'map_localize[r].py' >/dev/null; then
        if pgrep -af 'map_localize[r].py.*observe-only' >/dev/null; then
            echo "  localizer: **observe-only**（TF は出していない）"
        else
            echo "  localizer: 動いている（TF を出している）"
        fi
    else
        echo "  localizer: 止まっている"
    fi
    echo "--- いまの map→odom ---"
    cd "$G1_DIR/pc2_humble" 2>/dev/null && timeout 20 "$PIXI" run bash -lc "
      source $G1_DIR/g1_ws/install/setup.bash
      export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export CYCLONEDDS_URI=file://$CYCLONE_CFG
      timeout 8 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -A2 Translation | head -3
      echo '--- localizer の状態 ---'
      timeout 6 ros2 topic echo --once /g1/localizer_status 2>/dev/null | grep -E 'key|value' | head -12
    " 2>/dev/null
    ;;
  *)
    sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
