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
#
# 環境変数:
#   SCENE_USD / SPAWN_X / SPAWN_Y  シーンと初期位置を上書きする
#   G1_PERFECT_LOC=1               **測位を完璧にする**（下記）
#
# ## G1_PERFECT_LOC=1（測位のせいか、それ以外かを割る実験用）
#
# Isaac Sim の /odom は真値そのもの（runner.py の _publish_ros が root_pos_w を
# そのまま流す）。だから map -> odom を恒等変換で出せば「測位が完璧」になる。
# この状態で着かなければ、残るのは制御・コストマップ・scan 高さの問題で、
# **それは実機にそのまま映る**。着けば残るのは測位だけと言える。
#
# AMCL は kill しない。ライフサイクル管理（lifecycle_manager_localization）が
# bond の切断を検出して起こし直してしまうため。代わりに tf_broadcast: false で
# 黙らせ、publish_map_odom_tf.py に map -> odom を出させる。
# AMCL 自体は生きているので /amcl_pose に「測位がどう外したか」が残り、
# 完璧な測位で歩いた軌跡と並べて比べられる。
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

# 既存のプロセスが残っていると、複数の Isaac Sim / Nav2 が同時に TF を
# 配信して互いに打ち消し合う（TF_OLD_DATA が大量に出て、RViz に
# 現在地が表示されなくなる）。起動前に検出して止める。
# 注意点が 2 つある:
#   1. pgrep は何も見つからないと終了コード 1 を返す。set -e で止まるので
#      || true が要る（これが無くて起動しなくなった）。
#   2. pgrep -f / pkill -f は自分自身のコマンドラインにもマッチする。
#      $$ を除外しないと自分を kill してしまう。
STALE=$(pgrep -f "run_g1_twin|component_container_isolated" 2>/dev/null \
        | grep -v "^$$\$" | wc -l || true)
if [[ "$STALE" -gt 0 ]]; then
    echo "[WARN] 既に $STALE 個のプロセスが動いています（前回の残骸の可能性）"
    echo "[INFO] 停止します..."
    for pid in $(pgrep -f "run_g1_twin|component_container_isolated" 2>/dev/null || true); do
        [[ "$pid" != "$$" ]] && kill -9 "$pid" 2>/dev/null || true
    done
    sleep 5
    echo "[OK] 停止しました"
fi

if [[ ! -f "$MAP_YAML" ]]; then
    echo "[NG] 地図が見つかりません: $MAP_YAML"
    echo "     先に 'bash run_slam.sh' を実行して地図を作ってください。"
    exit 1
fi
echo "[INFO] 地図: $MAP_YAML"

SIM_PID=""
NAV_PID=""
TF_PID=""
cleanup() {
    echo "[INFO] 後片付けをしています..."
    # ⚠️ NAV_PID（ros2 launch）には SIGTERM ではなく SIGINT を送ること。
    # launch_service.py の実装上、SIGTERM/SIGQUIT はサブプロセス（特に
    # component_container_isolated）を止めずに launch 自身だけ終了する
    # （ソースの TODO コメントに "using SIGTERM can result in orphaned
    # processes" と明記されている）。SIGINT だけが Ctrl-C 相当の正規の
    # シャットダウンシーケンスを子プロセスに伝播する。これを踏んで
    # component_container_isolated が PPID=1 の孤児として残り続けた
    # （2026-09-09 に実測で 2 回再現）。
    [[ -n "$TF_PID" ]] && kill "$TF_PID" 2>/dev/null || true
    [[ -n "$NAV_PID" ]] && kill -INT "$NAV_PID" 2>/dev/null || true
    [[ -n "$SIM_PID" ]] && kill "$SIM_PID" 2>/dev/null || true
    sleep 5
    [[ -n "$TF_PID" ]] && kill -9 "$TF_PID" 2>/dev/null || true
    [[ -n "$NAV_PID" ]] && kill -9 "$NAV_PID" 2>/dev/null || true
    [[ -n "$SIM_PID" ]] && kill -9 "$SIM_PID" 2>/dev/null || true
    # 保険。上記でも孤児が残ることがあるため、原因に関わらずここで確実に払う
    # （起動前の STALE チェックと同じパターンマッチ。$$ は自分自身を除外）。
    for pid in $(pgrep -f "run_g1_twin|component_container_isolated" 2>/dev/null || true); do
        [[ "$pid" != "$$" ]] && kill -9 "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT

if [[ "$COMMAND_SOURCE" == "keyboard" ]]; then
    echo "[INFO] 手動操作モード（キーボードで歩かせる）"
else
    echo "[INFO] 自律モード（Nav2 の指令で歩く）"
fi
echo "[INFO] === 段階 1/2: Isaac Sim を起動します ==="
# SCENE_USD 環境変数が指定されていれば Warehouse の代わりにそれを読み込む
# （実測地図から生成した障害物メッシュ等。地図と物理シーンを一致させたい場合に使う）
SCENE_ARGS=()
if [[ -n "${SCENE_USD:-}" ]]; then
    echo "[INFO] シーン: $SCENE_USD"
    SCENE_ARGS=(--scene-usd "$SCENE_USD")
fi
# SPAWN_X / SPAWN_Y も同様に環境変数で上書きできる（実測地図の自由空間に合わせる）
"$ISAAC_SIM/python.sh" "$SCRIPT_DIR/src/run_g1_twin.py" \
    --viz kit \
    --command-source "$COMMAND_SOURCE" \
    --x "${SPAWN_X:-0.0}" \
    --y "${SPAWN_Y:-0.0}" \
    "${SCENE_ARGS[@]}" \
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

# G1_PERFECT_LOC=1 なら、Nav2 より先に map -> odom の恒等変換を出しておく。
# 先に出すのは、AMCL / bt_navigator が起動直後から TF を引けるようにするため。
# /clock は Isaac Sim が出しているので、シム起動を待った今なら使える。
if [[ "${G1_PERFECT_LOC:-0}" == "1" ]]; then
    echo "[INFO] 測位を完璧にします（map -> odom を恒等変換で配信、AMCL は黙らせる）"
    python3 "$SCRIPT_DIR/src/publish_map_odom_tf.py" \
        > "$LOG_DIR/map_odom_tf.log" 2>&1 &
    TF_PID=$!
    sleep 3
    if ! kill -0 "$TF_PID" 2>/dev/null; then
        echo "[NG] map -> odom の配信が始まりませんでした:"
        cat "$LOG_DIR/map_odom_tf.log"
        exit 1
    fi
fi

# nav2.yaml の Behavior Tree のパスを実際の場所に合わせる。
# Nav2 は設定内の環境変数を展開しないため絶対パスで書く必要があり、
# リポジトリを別の場所へ置くと壊れる。起動のたびに書き換えて回避する。
GENERATED_PARAMS="$LOG_DIR/nav2_params_generated.yaml"
# 測位を完璧にするときだけ AMCL の TF 配信を止める。
# ⚠️ ros2 param set では効かない（configure 時に読まれた値で動く）。
# yaml を書き換えて起動しないと変わらない。always_send_full_costmap で同じ罠を踏んだ。
PARAM_EDITS=(
    -e "s|^\( *default_nav_to_pose_bt_xml: \).*|\1$SCRIPT_DIR/config/navigate_g1.xml|"
    -e "s|^\( *default_nav_through_poses_bt_xml: \).*|\1$SCRIPT_DIR/config/navigate_through_poses_g1.xml|"
)
if [[ "${G1_PERFECT_LOC:-0}" == "1" ]]; then
    PARAM_EDITS+=(-e "s|^\( *tf_broadcast: \).*|\1false|")
    echo "[INFO] AMCL の tf_broadcast を false にします（推定は続けるが TF は出さない）"
fi
sed "${PARAM_EDITS[@]}" "$SCRIPT_DIR/config/nav2.yaml" > "$GENERATED_PARAMS"

echo "[INFO] === 段階 2/2: Nav2 を起動します ==="
# map_server に地図を渡し、AMCL で自己位置を推定する構成
ros2 launch nav2_bringup bringup_launch.py \
    map:="$MAP_YAML" \
    params_file:="$GENERATED_PARAMS" \
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
INITIAL_POSE_OK=1
python3 "$SCRIPT_DIR/src/set_initial_pose.py" || INITIAL_POSE_OK=0
if [[ "$INITIAL_POSE_OK" -eq 0 ]]; then
    echo "[WARN] 初期姿勢の設定に失敗しました。"
    echo "       RViz に現在地が表示されない場合は、Nav2 の起動完了後に"
    echo "       別ターミナルで次を実行してください:"
    echo "         source env.sh && python3 src/set_initial_pose.py"
fi

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
if [[ "$INITIAL_POSE_OK" -eq 1 ]]; then
    echo "   初期姿勢は自動設定済み（Isaac Sim の真値）。"
else
    echo "   [!] 初期姿勢の設定に失敗している。現在地が出ない場合は:"
    echo "       source env.sh && python3 src/set_initial_pose.py"
fi
echo "   RViz の「2D Pose Estimate」は使わないこと。この地図は原点が"
echo "   ずれているためクリックでは大きく外れる。"
echo "   やり直したいときは: python3 src/set_initial_pose.py"
echo
echo " ログ: $LOG_DIR/nav2.log / $LOG_DIR/nav2_sim.log"
echo " 終了: Ctrl-C"
echo "=============================================================="

wait "$NAV_PID"
