#!/usr/bin/env bash
# 自動巡回で Warehouse の地図を作る。
#
# G1 を LiDAR を見ながら自動で歩き回らせ、slam_toolbox に地図を作らせて
# 最後に .pgm / .yaml として保存する。人の操作は不要。
#
# 使い方:
#   bash run_slam.sh                 # 既定 (12000 step ≒ シム内 240 秒)
#   bash run_slam.sh 20000           # step 数を指定
#
# 出力:
#   maps/warehouse.pgm / warehouse.yaml   地図
#   logs/slam_sim.log                     Isaac Sim のログ
#   logs/slam_toolbox.log                 slam_toolbox のログ
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

# 巡回するステップ数（制御 50Hz なので 12000 step = シム内 240 秒）
MAX_STEPS="${1:-12000}"

MAP_DIR="$SCRIPT_DIR/maps"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$MAP_DIR" "$LOG_DIR"

MAP_NAME="${MAP_NAME:-warehouse}"

# 後片付け: 何があっても子プロセスを残さない
SIM_PID=""
SLAM_PID=""
cleanup() {
    echo "[INFO] 後片付けをしています..."
    [[ -n "$SLAM_PID" ]] && kill "$SLAM_PID" 2>/dev/null || true
    [[ -n "$SIM_PID" ]] && kill "$SIM_PID" 2>/dev/null || true
    sleep 2
    [[ -n "$SLAM_PID" ]] && kill -9 "$SLAM_PID" 2>/dev/null || true
    [[ -n "$SIM_PID" ]] && kill -9 "$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "[INFO] === 段階 1/3: Isaac Sim を起動して自動巡回を始めます ==="
echo "[INFO] step 数: $MAX_STEPS"
"$ISAAC_SIM/python.sh" "$SCRIPT_DIR/src/run_g1_twin.py" \
    --viz none \
    --command-source patrol \
    --max-steps "$MAX_STEPS" \
    > "$LOG_DIR/slam_sim.log" 2>&1 &
SIM_PID=$!

# Isaac Sim の起動には数分かかる。ループ突入を待つ。
echo "[INFO] Isaac Sim の起動を待っています（数分かかります）..."
for _ in $(seq 1 180); do
    if grep -q "シミュレーションを開始します" "$LOG_DIR/slam_sim.log" 2>/dev/null; then
        echo "[OK] Isaac Sim が起動しました"
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[NG] Isaac Sim が起動前に終了しました。ログを確認してください:"
        tail -20 "$LOG_DIR/slam_sim.log"
        exit 1
    fi
    sleep 5
done

if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "[NG] Isaac Sim が動いていません"
    exit 1
fi

echo "[INFO] === 段階 2/3: slam_toolbox を起動します ==="
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file "$SCRIPT_DIR/config/slam_toolbox.yaml" \
    > "$LOG_DIR/slam_toolbox.log" 2>&1 &
SLAM_PID=$!

sleep 10
if ! kill -0 "$SLAM_PID" 2>/dev/null; then
    echo "[NG] slam_toolbox が起動できませんでした:"
    tail -20 "$LOG_DIR/slam_toolbox.log"
    exit 1
fi

# slam_toolbox はライフサイクルノードなので、activate しないと
# /scan の購読を始めない（起動しただけでは地図が全く作られない）。
echo "[INFO] slam_toolbox を activate します"
ros2 lifecycle set /slam_toolbox configure || true
ros2 lifecycle set /slam_toolbox activate || true
STATE=$(ros2 lifecycle get /slam_toolbox 2>/dev/null || echo "不明")
echo "[INFO] slam_toolbox の状態: $STATE"
if [[ "$STATE" != active* ]]; then
    echo "[NG] slam_toolbox を active にできませんでした:"
    tail -20 "$LOG_DIR/slam_toolbox.log"
    exit 1
fi
echo "[OK] slam_toolbox が地図の作成を始めました"

echo "[INFO] === 巡回中です。Isaac Sim の終了を待ちます ==="
echo "[INFO] 進捗: tail -f $LOG_DIR/slam_sim.log"

# Isaac Sim が max-steps に達して終了するまで待つ
wait "$SIM_PID" || true
SIM_PID=""
echo "[OK] 巡回が完了しました"

# 最後のスキャンが地図に反映されるのを待つ
echo "[INFO] 地図の最終更新を待っています..."
sleep 15

echo "[INFO] === 段階 3/3: 地図を保存します ==="
cd "$MAP_DIR"
if ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" --ros-args -p save_map_timeout:=60.0; then
    echo "[OK] 地図を保存しました: $MAP_DIR/$MAP_NAME.pgm / .yaml"
    ls -lh "$MAP_DIR/$MAP_NAME".* 2>/dev/null || true
else
    echo "[NG] 地図の保存に失敗しました。slam_toolbox のログを確認してください:"
    tail -20 "$LOG_DIR/slam_toolbox.log"
    exit 1
fi
