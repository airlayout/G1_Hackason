#!/usr/bin/env bash
# `map→odom` の出し方を **Nav2 を上げ直さずに** 切り替える。
#
#   ./map2odom_ctl.sh start   <dx> <dy> <yaw>   # 供給を開始（hold=補正しない。従来と同じ挙動）
#   ./map2odom_ctl.sh correct                   # 補正を **入れる**（走行中に切り替えてよい）
#   ./map2odom_ctl.sh hold                      # 補正を **止める**（初期値のまま出し続ける）
#   ./map2odom_ctl.sh observe <dx> <dy> <yaw>   # ⭐ 測るだけ（TF は出さない・併走してよい）
#   ./map2odom_ctl.sh status / off
#
# ## ⚠️ 使う前提: Nav2 を `map_to_odom:=none` で上げること
#
#     bash ~/g1_nav2/start_nav.sh none 0 2.0
#
# `none` なら `g1_slam_odom_tf.py` は map→odom を出さない。その口をここが埋める。
#
# ## ⚠️⚠️ なぜ「静的 TF と動的 TF の入れ替え」にしないのか（2026-09-24 にモックで実測）
#
# 当初は static_transform_publisher と map_localizer を**入れ替える**作りにしたが、
# **動かなかった**:
#
# - `map_localizer` は起動後、**距離場の作成に約3秒**かかり、その間 TF を出せない
# - そこで static を止めると **`tf_stale` → FAULT → 巡回が HOLD**
# - 順序を「先に上げてから止める」に変えても FAULT。**静的 TF は tf2 のバッファに
#   残り続ける**ため、そもそも静的と動的の入れ替えは成立しない
#
# → **供給者は常に `map_localizer` 1本**にし、その中で hold / correct を切り替える。
#   TF は途切れないので、走行中でも安全に切り替えられる。
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
    rm -f "$LOG_DIR/$name.log"
    # ⚠️ pixi が無い環境（モックのコンテナ等）では**そのまま実行する**。
    # PC2 では pixi の Humble を通す必要があるので、有無で分ける。
    if [ ! -x "$PIXI" ]; then
        setsid nohup bash -lc "
          source $G1_DIR/g1_ws/install/setup.bash 2>/dev/null || true
          exec $*
        " > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
        return 0
    fi
    cd "$G1_DIR/pc2_humble" || return 1
    setsid nohup "$PIXI" run bash -lc "
      source $G1_DIR/g1_ws/install/setup.bash
      export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export CYCLONEDDS_URI=file://$CYCLONE_CFG
      exec $*
    " > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
}

# ⚠️ pkill のパターンは括弧付きのまま使う（このスクリプト自身に当たらないため）
kill_localizer() { pkill -f 'map_localize[r].py' 2>/dev/null; }

need_xyz() {
    [ $# -ge 3 ] || { echo "使い方: $0 $CMD <dx> <dy> <yaw[rad]>   # find_map_offset.py の結果" >&2; exit 2; }
}

CMD="${1:-status}"; shift || true

svc() {   # svc <true|false>
    if [ ! -x "$PIXI" ]; then
        timeout 20 bash -lc "source $G1_DIR/g1_ws/install/setup.bash 2>/dev/null || true
          ros2 service call /g1/localizer/correct std_srvs/srv/SetBool '{data: $1}'" 2>&1 | tail -1
        return
    fi
    cd "$G1_DIR/pc2_humble" && timeout 20 "$PIXI" run bash -lc "
      source $G1_DIR/g1_ws/install/setup.bash
      export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export CYCLONEDDS_URI=file://$CYCLONE_CFG
      ros2 service call /g1/localizer/correct std_srvs/srv/SetBool '{data: $1}'" 2>&1 | tail -1
}

case "$CMD" in
  start)
    need_xyz "$@"
    kill_localizer; sleep 1
    # ⚠️ **hold で始める**（＝従来の静的 map→odom と同じ挙動）。補正を入れるかは
    # `correct` で人が決める。いきなり補正すると、実機で初めて姿勢が飛ぶ場面になる。
    run_in_env map2odom_localizer \
      "python3 $G1_DIR/tools/map_localizer.py --initial $1 $2 $3 --mode hold"
    echo "start: map→odom = $1 $2 $3 を出し始めた（mode=hold ＝ 補正しない）"
    echo "  ⚠️ 距離場の作成に数秒かかる。status で確認すること"
    ;;
  correct) svc true;  echo "→ mode=correct（補正を入れる）" ;;
  hold)    svc false; echo "→ mode=hold（補正しない・初期値のまま）" ;;
  observe)
    need_xyz "$@"
    # ⭐ TF を出さないので、上の供給者と**併走してよい**
    pkill -f 'map_localizer.py.*observe-onl[y]' 2>/dev/null; sleep 1
    CSV="$G1_DIR/runs/drift_$(date +%Y%m%d_%H%M).csv"
    run_in_env map2odom_observe \
      "python3 $G1_DIR/tools/map_localizer.py --initial $1 $2 $3 --observe-only --log-csv $CSV"
    echo "observe: 測るだけ（TF は出さない）。走行には影響しない"
    echo "  記録: $CSV"
    ;;
  off)
    kill_localizer
    echo "止めた。⚠️ **map→odom を誰も出していない状態**になる"
    echo "   TF が途切れると g1_cmd_router が tf_stale で FAULT にする"
    ;;
  status)
    echo "--- プロセス ---"
    pgrep -af 'map_localizer.py.*observe-onl[y]' >/dev/null && echo "  observe: 動いている（TF は出していない）" || echo "  observe: 止まっている"
    if pgrep -af 'map_localize[r].py' | grep -qv observe-only; then
        echo "  供給者: 動いている"
    else
        echo "  供給者: **止まっている**"
    fi
    echo "--- いまの map→odom と mode ---"
    if [ ! -x "$PIXI" ]; then
        timeout 20 bash -lc "source $G1_DIR/g1_ws/install/setup.bash 2>/dev/null || true
          timeout 6 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -A1 Translation | head -2
          timeout 5 ros2 topic echo --once /g1/localizer_status 2>/dev/null | grep -A1 -E 'mode|score|rejected' | head -9" 2>/dev/null
        exit 0
    fi
    cd "$G1_DIR/pc2_humble" 2>/dev/null && timeout 25 "$PIXI" run bash -lc "
      source $G1_DIR/g1_ws/install/setup.bash
      export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export CYCLONEDDS_URI=file://$CYCLONE_CFG
      timeout 6 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -A1 Translation | head -2
      timeout 5 ros2 topic echo --once /g1/localizer_status 2>/dev/null | grep -A1 -E 'mode|score|rejected' | head -9
    " 2>/dev/null
    ;;
  *)
    sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
