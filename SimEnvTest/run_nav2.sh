#!/usr/bin/env bash
# 作成済みの地図を使って Nav2 で自律走行させる。
#
# RViz で「2D Goal Pose」を指定すると、Nav2 が経路を作り G1 が自律的に歩く。
# Nav2 が出す /cmd_vel を Isaac Sim 側の歩行ポリシーが受け取る。
#
# 前提:
#   bash run_slam.sh で maps/warehouse.yaml を作ってあること
#
# 使い方:
#   bash run_nav2.sh                      # 既定の地図を使う
#   bash run_nav2.sh maps/other.yaml      # 地図を指定する
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

MAP_YAML="${1:-$SCRIPT_DIR/maps/warehouse.yaml}"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

if [[ ! -f "$MAP_YAML" ]]; then
    echo "[NG] 地図が見つかりません: $MAP_YAML"
    echo "     先に 'bash run_slam.sh' を実行して地図を作ってください。"
    exit 1
fi
echo "[INFO] 地図: $MAP_YAML"

SIM_PID=""
NAV_PID=""
cleanup() {
    echo "[INFO] 後片付けをしています..."
    [[ -n "$NAV_PID" ]] && kill "$NAV_PID" 2>/dev/null || true
    [[ -n "$SIM_PID" ]] && kill "$SIM_PID" 2>/dev/null || true
    sleep 2
    [[ -n "$NAV_PID" ]] && kill -9 "$NAV_PID" 2>/dev/null || true
    [[ -n "$SIM_PID" ]] && kill -9 "$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "[INFO] === 段階 1/2: Isaac Sim を起動します（Nav2 の指令で歩く） ==="
"$ISAAC_SIM/python.sh" "$SCRIPT_DIR/src/run_g1_twin.py" \
    --viz kit \
    --command-source ros \
    > "$LOG_DIR/nav2_sim.log" 2>&1 &
SIM_PID=$!

echo "[INFO] Isaac Sim の起動を待っています（数分かかります）..."
for _ in $(seq 1 180); do
    if grep -q "シミュレーションを開始します" "$LOG_DIR/nav2_sim.log" 2>/dev/null; then
        echo "[OK] Isaac Sim が起動しました"
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[NG] Isaac Sim が起動前に終了しました:"
        tail -20 "$LOG_DIR/nav2_sim.log"
        exit 1
    fi
    sleep 5
done

echo "[INFO] === 段階 2/2: Nav2 を起動します ==="
# map_server に地図を渡し、AMCL で自己位置を推定する構成
ros2 launch nav2_bringup bringup_launch.py \
    map:="$MAP_YAML" \
    params_file:="$SCRIPT_DIR/config/nav2.yaml" \
    use_sim_time:=true \
    autostart:=true \
    > "$LOG_DIR/nav2.log" 2>&1 &
NAV_PID=$!

sleep 20
if ! kill -0 "$NAV_PID" 2>/dev/null; then
    echo "[NG] Nav2 が起動できませんでした:"
    tail -30 "$LOG_DIR/nav2.log"
    exit 1
fi

echo "[OK] Nav2 が起動しました"
echo
echo "=============================================================="
echo " 使い方:"
echo "   1. 別ターミナルで RViz を起動する:"
echo "        source /opt/ros/jazzy/setup.bash"
echo "        ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"
echo "   2. RViz の「2D Pose Estimate」で G1 の現在位置を教える"
echo "   3. RViz の「2D Goal Pose」で目標地点を指定する"
echo
echo " ログ: $LOG_DIR/nav2.log / $LOG_DIR/nav2_sim.log"
echo " 終了: Ctrl-C"
echo "=============================================================="

wait "$NAV_PID"
