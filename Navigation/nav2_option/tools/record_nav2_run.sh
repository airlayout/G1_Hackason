#!/usr/bin/env bash
# 巡回 1 回分を rosbag に記録する。Planning.md Phase 2c 完了条件:
#
#   > rosbag から Goal・経路・姿勢・速度指令・**停止理由**を追跡できる
#
# ## なぜ要るのか
#
# 実機を動かして「なぜ止まったのか」が後から分からないと、原因究明も改善もできない。
# 停止経路は複数ある(`cmd_timeout` / `operator_lost` / `tf_stale` / `sensor_stale` /
# `bridge_disconnected` / E-stop)ので、**どれで止まったのかが区別できて初めて意味がある。**
#
# 記録した bag は [explain_run.py](explain_run.py) で時系列に要約できる。
#
# ## プロファイル
#
# | | 内容 | 目安 |
# |---|---|---|
# | `diag`(既定) | 生センサーと costmap 格子を除く全部 | **数 MB/分**。常時付けっぱなしでよい |
# | `full` | + costmap 格子 + 生点群 | **数百 MB〜GB/分**。不具合を追うときだけ |
#
# ⚠️ **`full` は Mapping の記録と同じ桁になる**(実績: 331MB〜2.1GB/セッション)。
# Orin の内蔵ストレージを埋めないよう、録る前に空き容量を確認すること。
#
# ## 使い方
#
#     ./record_nav2_run.sh                      # diag プロファイル、runs/ 配下に出力
#     ./record_nav2_run.sh --profile full
#     ./record_nav2_run.sh --output /path/to/bag
#
# ⚠️ **ROS 環境を source した端末で実行すること**(SDK 側プロセスとは逆、D-07 参照)。

set -o pipefail   # ⚠️ `set -u` にしないこと。ROS の setup.bash が未定義変数を参照する

PROFILE="diag"
OUTPUT=""
while [ $# -gt 0 ]; do
    case "$1" in
        --profile) PROFILE="$2"; shift 2 ;;
        --output)  OUTPUT="$2"; shift 2 ;;
        --help|-h)
            sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) echo "[record] 未知の引数: $1" >&2; exit 2 ;;
    esac
done

if [ -z "$OUTPUT" ]; then
    OUTPUT="$(pwd)/runs/$(date +%Y%m%d_%H%M%S)_nav2"
fi

# --- 記録するトピック -------------------------------------------------------
#
# 「後から事故を再構成できる」ことが目的なので、**判断の材料になったもの**と
# **判断の結果**の両方を残す。
TOPICS=(
    # === 停止理由 ===
    # ⚠️ **これが最重要。** 状態・fault_reason・TF/センサー/heartbeat の健全性が
    # 20Hz で入っている。「なぜ止まったか」はほぼここで分かる
    /g1/bridge_status
    /g1/state_bridge_status
    /g1/estop
    # ⚠️ **/rosout を必ず含めること。** 「走行中に TF が失われた」等の
    # RCLCPP_ERROR はここにしか残らない
    /rosout

    # === Goal ===
    # ⚠️ アクションのトピックは**隠しトピック**なので、`-a` では録れない。明示が要る
    /navigate_to_pose/_action/goal
    /navigate_to_pose/_action/result
    /navigate_to_pose/_action/status
    /navigate_to_pose/_action/feedback
    /goal_pose                       # RViz の「2D Goal Pose」

    # === 経路 ===
    /plan                            # global planner の出力
    /local_plan                      # controller の局所経路
    /received_global_plan            # controller が受け取った経路

    # === 姿勢 ===
    /tf
    /tf_static
    /odom

    # === 速度指令 ===
    /cmd_vel                         # controller の出力
    /cmd_vel_smoothed                # velocity_smoother の出力 = g1_cmd_router の入力

    # === 参考 ===
    /diagnostics
    /clock
)

if [ "$PROFILE" = "full" ]; then
    # ⚠️ ここから先は桁が変わる。常時記録には向かない
    TOPICS+=(
        /local_costmap/costmap
        /global_costmap/costmap
        /g1/points_local             # 生点群。**これが支配的に大きい**
        /scan
    )
elif [ "$PROFILE" != "diag" ]; then
    echo "[record] --profile は diag か full" >&2
    exit 2
fi

mkdir -p "$(dirname "$OUTPUT")"
echo "[record] プロファイル=$PROFILE 出力=$OUTPUT"
echo "[record] トピック数=${#TOPICS[@]}"
if [ "$PROFILE" = "full" ]; then
    echo "[record] ⚠️ full は数百MB〜GB/分になる。空き容量: $(df -h "$(dirname "$OUTPUT")" | tail -1 | awk '{print $4}')"
fi
echo "[record] Ctrl-C で停止する"

# ⚠️ **`--include-hidden-topics` が要る。** アクションの
# `/navigate_to_pose/_action/*` は**隠しトピック**なので、名前を明示しても
# これが無いと黙って記録されない(2026-09-13 に実測で踏んだ。
# explain_run.py の記録漏れ検出が捕まえた)。
exec ros2 bag record --storage sqlite3 --include-hidden-topics \
    --output "$OUTPUT" "${TOPICS[@]}"
