#!/usr/bin/env bash
# 頭部 LiDAR の点群と IMU をライブ表示する。
#
#   ./scripts/lidar_view.sh                # 点群 + IMU 波形
#   ./scripts/lidar_view.sh --decay        # 点を 5 秒残す（積もる様子が見える）
#   ./scripts/lidar_view.sh --no-imu       # PlotJuggler を出さない
#   ./scripts/lidar_view.sh --no-rviz      # 画面なし（bag 記録だけしたいとき）
#   ./scripts/lidar_view.sh --custommsg    # SLAM 向けに Livox CustomMsg で出す
#   ./scripts/lidar_view.sh --fake         # LiDAR 無しで画面を再現（動作確認用）
#
# Ctrl+C で全部止まる。
#
# トピック: /livox/lidar (PointCloud2) と /livox/imu (sensor_msgs/Imu)

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib.sh"

DECAY=0
WITH_IMU=1
WITH_RVIZ=1
FAKE=0
XFER=0

while [ $# -gt 0 ]; do
  case "$1" in
    --decay)      DECAY=1 ;;
    --no-imu)     WITH_IMU=0 ;;
    --no-rviz)    WITH_RVIZ=0 ;;
    --fake)       FAKE=1 ;;
    --custommsg)  XFER=1 ;;
    -h|--help)    sed -n '2,13p' "$0"; exit 0 ;;
    *) _die "不明な引数: $1" ;;
  esac
  shift
done

load_config
use_ros

RVIZ_CFG="$REPO_DIR/config/rviz/lidar_view.rviz"
[ "$DECAY" = 1 ] && RVIZ_CFG="$REPO_DIR/config/rviz/lidar_view_decay.rviz"

# --- 子プロセスを Ctrl+C でまとめて落とす ---
PIDS=()
cleanup() {
  trap - INT TERM EXIT
  echo ""
  echo "終了処理中..."
  # プロセスグループ→単体PID の順に試す（setsid が失敗した子にも届かせる）
  for pid in "${PIDS[@]}"; do
    kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  done
  echo "停止しました。"
}
trap cleanup INT TERM EXIT

echo "=============================================="
echo " G1 LiDAR ライブ表示"
echo "  ROS_DOMAIN_ID : $ROS_DOMAIN_ID （ロボットの DDS domain 0 と分離）"
echo "  形式          : $([ "$XFER" = 1 ] && echo 'Livox CustomMsg' || echo PointCloud2)"
echo "  データ源      : $([ "$FAKE" = 1 ] && echo 'ダミー（動作確認用）' || echo 実機)"
echo "=============================================="

if [ "$FAKE" = 1 ]; then
  # 実機が無くても画面と記録の流れを確認できるようにするための擬似配信
  setsid python3 "$REPO_DIR/tools/fake_lidar.py" & PIDS+=($!)
  setsid ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 3.14159265 --pitch 0 --yaw 0 \
    --frame-id viz_base --child-frame-id livox_frame > /dev/null 2>&1 & PIDS+=($!)
  sleep 2
  [ "$WITH_RVIZ" = 1 ] && { setsid rviz2 --display-config "$RVIZ_CFG" > /dev/null 2>&1 & PIDS+=($!); }
else
  require_wired

  CONFIG="$REPO_DIR/config/livox/active.json"
  if [ ! -f "$CONFIG" ]; then
    _die "LiDAR 設定が未生成です: config/livox/active.json

     機種（Mid-360 / Mid-360S）で設定ファイルの形式が違い、間違えると
     ドライバは無言で何も出しません。まず自動検出してください:

       python3 tools/lidar_probe.py --write-config

     LiDAR が見つからない場合は tools/lidar_probe.py の出力に従ってください。"
  fi

  LIDAR_IP="$(/usr/bin/python3 -c "
import json,sys; print(json.load(open('$CONFIG'))['lidar_configs'][0]['ip'])" 2>/dev/null || echo "")"
  if [ -n "$LIDAR_IP" ]; then
    echo "LiDAR ($LIDAR_IP) の疎通を確認中 ..."
    if ping -c 1 -W 2 "$LIDAR_IP" > /dev/null 2>&1; then
      _ok "応答あり"
    else
      _warn "応答がありません。ケーブル / ロボットの起動 / 有線 IP を確認してください。"
      echo "     このまま起動します（点群が出なければ上を疑ってください）。"
    fi
  fi

  setsid ros2 launch "$REPO_DIR/launch/livox.launch.py" \
    config:="$CONFIG" \
    xfer_format:="$XFER" \
    flip:="$G1_LIDAR_FLIP" \
    rviz:="$([ "$WITH_RVIZ" = 1 ] && echo true || echo false)" \
    rviz_config:="$RVIZ_CFG" & PIDS+=($!)
fi

if [ "$WITH_IMU" = 1 ]; then
  sleep 3
  # PlotJuggler は起動後に手動でトピックを選ぶ（docs/05-lidar.md 参照）
  setsid ros2 run plotjuggler plotjuggler > /dev/null 2>&1 & PIDS+=($!)
fi

echo ""
echo "起動しました。Ctrl+C で全部止まります。"
echo "  記録する場合は別ターミナルで: ./scripts/record_bag.sh <名前>"

# 単に wait すると、点群を出す本体が死んでも RViz だけ残って
# 「起動しました」の表示のまま気づけない。本体を個別に監視する。
MAIN_PID="${PIDS[0]}"
while kill -0 "$MAIN_PID" 2>/dev/null; do
  sleep 1
done
echo ""
echo "⚠️ 点群の配信プロセスが終了しました。表示も終了します。"
echo "   原因は上のログを確認してください（設定の機種違いが最も多いです）。"
exit 1
