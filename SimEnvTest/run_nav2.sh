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
#   bash run_nav2.sh                      # 自律走行（既定の地図）
#   bash run_nav2.sh --manual             # キーボード操作。Nav2 と RViz は
#                                         # 地図・自己位置の表示用に動かす
#   bash run_nav2.sh maps/other.yaml      # 地図を指定する
#   bash run_nav2.sh --manual maps/x.yaml # 両方
#
# --manual と既定（自律）は排他。実行中には切り替えられないので、
# 起動時にどちらで動かすかを決めること。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

# --manual があればキーボード操作にする
COMMAND_SOURCE="ros"
ARGS=()
for a in "$@"; do
    case "$a" in
        --manual) COMMAND_SOURCE="keyboard" ;;
        *) ARGS+=("$a") ;;
    esac
done

MAP_YAML="${ARGS[0]:-$SCRIPT_DIR/maps/warehouse.yaml}"
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

if [[ "$COMMAND_SOURCE" == "keyboard" ]]; then
    echo "[INFO] 手動操作モード（キーボードで歩かせる）"
else
    echo "[INFO] 自律モード（Nav2 の指令で歩く）"
fi
echo "[INFO] === 段階 1/2: Isaac Sim を起動します ==="
"$ISAAC_SIM/python.sh" "$SCRIPT_DIR/src/run_g1_twin.py" \
    --viz kit \
    --command-source "$COMMAND_SOURCE" \
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

# AMCL に初期位置を教える。
# RViz の「2D Pose Estimate」でクリックすると、この地図は原点が
# (-58, -53) にあるため大きくずれる（実測で位置 31 m / 向き 179 度）。
# Isaac Sim は真値を持っているので、それをそのまま渡す。
echo "[INFO] 初期姿勢を Isaac Sim の真値から設定します"
sleep 5
python3 "$SCRIPT_DIR/src/set_initial_pose.py" || \
    echo "[WARN] 初期姿勢の設定に失敗しました。手動で設定してください"

echo
echo "=============================================================="
if [[ "$COMMAND_SOURCE" == "keyboard" ]]; then
    echo " 使い方（手動操作モード）:"
    echo "   1. 別ターミナルで RViz を起動する:  bash run_rviz.sh"
    echo "   2. Isaac Sim のウィンドウをクリックしてフォーカスを当てる"
    echo "   3. W/S 前後  A/D 左右  Q/E 旋回  SPACE 停止  SHIFT 低速"
    echo
    echo "   RViz には地図と G1 の位置が表示される（動作確認用）。"
    echo "   Nav2 も起動しているが、2D Goal Pose を指定しないので指令は出ない。"
else
    echo " 使い方（自律モード）:"
    echo "   1. 別ターミナルで RViz を起動する:  bash run_rviz.sh"
    echo "   2. RViz の「2D Goal Pose」で目標地点を指定する"
    echo
    echo "   キーボードで操作したい場合は  bash run_nav2.sh --manual  で起動する。"
fi
echo
echo "   初期姿勢は上で自動設定済み（Isaac Sim の真値）。"
echo "   RViz の「2D Pose Estimate」は使わないこと。この地図は原点が"
echo "   ずれているためクリックでは大きく外れる。"
echo "   やり直したいときは: python3 src/set_initial_pose.py"
echo
echo " ログ: $LOG_DIR/nav2.log / $LOG_DIR/nav2_sim.log"
echo " 終了: Ctrl-C"
echo "=============================================================="

wait "$NAV_PID"
