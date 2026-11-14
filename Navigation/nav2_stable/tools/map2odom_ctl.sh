#!/usr/bin/env bash
# `map→odom` の出し方を **Nav2 を上げ直さずに** 切り替える。
#
#   ./map2odom_ctl.sh start   <dx> <dy> <yaw>   # 供給を開始（hold=補正しない。従来と同じ挙動）
#   ./map2odom_ctl.sh correct                   # 補正を **入れる**（走行中に切り替えてよい）
#   ./map2odom_ctl.sh hold                      # 補正を **止める**（初期値のまま出し続ける）
#   ./map2odom_ctl.sh observe <dx> <dy> <yaw>   # ⭐ 測るだけ（TF は出さない・併走してよい）
#   ./map2odom_ctl.sh status / off
#   ./map2odom_ctl.sh relocalize                # 合わせ直しの候補を探す（**機体を止めてから**。適用はしない）
#   ./map2odom_ctl.sh apply   <dx> <dy> <yaw>   # 候補を適用する（プロセスは入れ替えない）
#
# ## 自己位置が lost になったら（2026-11-03 追加）
#
# 巡回は止まり、約 1.3 秒後に FAULT になる（patrol_node.py）。手順:
#
#   1. map_web で位置と向きを見る。合っているなら（人だかり等で一時的に下がっただけ）6 へ
#   2. ./map2odom_ctl.sh relocalize      # いまの推定の向き ±20° → だめなら全周を探す
#   3. 出た候補の「20cm以内」が 80% 以上か見る
#   4. ./map2odom_ctl.sh apply <dx> <dy> <yaw>
#   5. **map_web で位置と向きが実機と合っているか目で見る**（向きが 180° 逆でも一致率は高く出る）
#   6. clear_fault → patrol_ctl.sh start
#
# ⚠️ relocalize / apply は供給者（start で上げた map_localizer）に対してだけ効く。
#    observe（静的 TF と併走）のときは map→odom を出しているのが静的 TF なので効かない
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

ros_run() {   # ros_run <コマンド文字列>（PC2 の pixi か、手元の ROS で実行する）
    if [ ! -x "$PIXI" ]; then
        timeout 200 bash -lc "source $G1_DIR/g1_ws/install/setup.bash 2>/dev/null || true
          $1" 2>&1
        return
    fi
    (cd "$G1_DIR/pc2_humble" && timeout 200 "$PIXI" run bash -lc "
      source $G1_DIR/g1_ws/install/setup.bash
      export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export CYCLONEDDS_URI=file://$CYCLONE_CFG
      $1" 2>&1)
}

# find_map_offset.py を回し、RESULT 行（dx dy yaw[rad] 20cm以内 50cm以内）を返す
find_offset() {   # find_offset <yaw_center[度]> <yaw_range[度]>
    ros_run "python3 $G1_DIR/tools/find_map_offset.py --yaw-center $1 --yaw-range $2 --yaw-step 1" \
        | sed -n 's/^RESULT //p' | tail -1
}

case "$CMD" in
  relocalize)
    # いまの推定を /g1/localizer_status から読む（"x, y, Ndeg"）
    CUR=$(ros_run "timeout 5 ros2 topic echo --once /g1/localizer_status" \
          | grep -A1 "key: map_to_odom" | sed -n "s/.*value: '\{0,1\}\([^']*\)'\{0,1\}$/\1/p" | tail -1)
    if [ -z "$CUR" ]; then
        echo "❌ /g1/localizer_status が読めない。map_localizer が動いているか（status）を見ること" >&2; exit 1
    fi
    CUR_DEG=$(echo "$CUR" | awk -F', *' '{sub("deg","",$3); print $3}')
    echo "いまの map→odom: $CUR"
    echo "--- 1) いまの向き ${CUR_DEG}° の ±20° を探す（180° 逆向きを掴まないため狭く探す）"
    R1=$(find_offset "$CUR_DEG" 20)
    BEST="$R1"
    if [ -z "$R1" ] || awk -v p="$(echo "$R1" | cut -d' ' -f4)" 'BEGIN{exit !(p < 0.8)}'; then
        echo "  → ${R1:-見つからず}（20cm以内 80% 未満）。全周を探す"
        echo "--- 2) 全周（±180°）を探す。⚠️ 向きが逆の答えもありうるので必ず目で見ること"
        BEST=$(find_offset 0 180)
    fi
    if [ -z "$BEST" ]; then echo "❌ 候補が見つからない。点群・TF・地図を確認すること" >&2; exit 1; fi
    read -r DX DY YAW P20 P50 <<< "$BEST"
    echo
    awk -v x="$DX" -v y="$DY" -v t="$YAW" -v p="$P20" -v q="$P50" 'BEGIN{
        printf "候補: map→odom = (%.3f, %.3f, %.1f°)   20cm以内 %.0f%%  50cm以内 %.0f%%\n", x, y, t*180/3.14159265, p*100, q*100
        if (p < 0.8) print "⚠️ 20cm以内が 80% 未満。この場所では合わせにくい。特徴のある場所へ動かしてやり直すこと"}'
    echo
    echo "適用するなら:  $0 apply $DX $DY $YAW"
    echo "⚠️ 適用したら map_web で**位置と向き**が実機と合っているか必ず目で見ること"
    ;;
  apply)
    need_xyz "$@"
    # ⚠️ -w 1: 購読者（map_localizer）が見つかるまで待つ。見つかる前に1回だけ出すと届かない
    OUT=$(ros_run "ros2 topic pub --once -w 1 /g1/localizer/set_map_to_odom geometry_msgs/msg/Pose2D '{x: $1, y: $2, theta: $3}'")
    echo "$OUT" | tail -2
    echo "apply: map→odom = $1 $2 $3 を map_localizer に渡した（ログに「差し替えた」が出る）"
    echo "  ⚠️ map_web で位置と向きを確かめてから clear_fault → patrol_ctl.sh start"
    ;;
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
