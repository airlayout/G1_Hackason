#!/usr/bin/env bash
# MuJoCoシム上でNav2のフルスタック(map_server/amcl/controller_server/planner_server/
# behavior_server/bt_navigator)を起動し、ゴールを与えて実際に歩くかを検証する。
# 2026-09-07 作成。amcl_sim_verify.pyでのAMCL単体検証の続き。
#
# 使い方:
#   source /opt/ros/jazzy/setup.bash   # 呼び出し元で先にsourceしておくこと
#   cd Navigation/nav3
#   bash run_nav2_sim.sh [--params <yamlファイル>] [nav2_sim_bridge.pyへ渡す追加引数...]
#
# 例: ゴールを変える（⚠️ "--"は付けない。そのままnav2_sim_bridge.pyへ渡るので、
#      "--"を付けるとargparseがそれ自体を認識できない引数として拒否する）
#   bash run_nav2_sim.sh --goal-x 3.0 --goal-y 2.0
#
# 例: 実地図(PCD)で試す（--roomの代わりに--map。nav2_sim_bridge.pyの--map参照）。
#      初期位置は部屋ごとに違うので、spawnに合わせたparamsファイルを別途用意し--paramsで渡す
#   bash run_nav2_sim.sh --params nav2_sim_params_room_a.yaml \
#       --map sim/maps/room_a_cropped.pcd --interactive --viewer
#
# --paramsはこのスクリプト自身が消費し、nav2_sim_bridge.pyへは渡さない
# （map_server/amcl/controller_server/planner_server/behavior_server/bt_navigatorの
# --params-fileに使うだけで、nav2_sim_bridge.py側には無い引数のため）。
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
set -m   # ジョブ制御を有効化。&で起動したジョブを専用プロセスグループにするため
         # （`ros2 run`はexecではなくfork/execするラッパーなので、ラッパーの
         # PIDだけkillしても実体(map_server等)が孤児として残る。実際に
         # このバグで前回の実行が大量のプロセスを残した。プロセスグループ
         # ごとkillすればラッパーと実体を一緒に消せる）

if ! command -v ros2 >/dev/null; then
    echo "[run_nav2_sim] ros2が見つからない。先に 'source /opt/ros/jazzy/setup.bash' すること" >&2
    exit 1
fi

export ROS_DOMAIN_ID=44   # amcl_sim_verify.py(43)と衝突しない専用ドメイン
LOG_DIR="$(pwd)/verification/nav2_sim_logs"
mkdir -p "$LOG_DIR"

PARAMS="$(pwd)/nav2_sim_params.yaml"
BRIDGE_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --params)
            PARAMS="$2"
            shift 2
            ;;
        *)
            BRIDGE_ARGS+=("$1")
            shift
            ;;
    esac
done

echo "[run_nav2_sim] 前回実行の残留プロセスを掃除"
pkill -9 -f "nav2_map_server/map_server" 2>/dev/null || true
pkill -9 -f "nav2_amcl/amcl" 2>/dev/null || true
pkill -9 -f "nav2_controller/controller_server" 2>/dev/null || true
pkill -9 -f "nav2_planner/planner_server" 2>/dev/null || true
pkill -9 -f "nav2_behaviors/behavior_server" 2>/dev/null || true
pkill -9 -f "nav2_bt_navigator/bt_navigator" 2>/dev/null || true
pkill -9 -f "nav2_sim_bridge.py" 2>/dev/null || true
sleep 1

PIDS=()
cleanup() {
    echo "[run_nav2_sim] 後始末: $((${#PIDS[@]})) プロセスグループを停止"
    for pid in "${PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT

spawn() {
    local logfile="$1"; shift
    "$@" > "$LOG_DIR/$logfile" 2>&1 &
    PIDS+=("$!")
    echo "[run_nav2_sim] 起動: $* (log: $logfile, pid: $!)"
}

bringup() {
    # ⚠️ lifecycle_bringup バイナリは遷移要求を送るだけで、ノードが実際に
    # active へ到達したかは確認せずに終了コード0を返すことがある
    # （planner_serverのプラグイン名を間違えたとき、configureで落ちて
    # unconfiguredへ戻ったのに`bringup`は「成功」した実例がある）。
    # なので`ros2 lifecycle get`で最終状態を必ず確認する。
    local node="$1"
    echo "[run_nav2_sim] lifecycle_bringup $node"
    ros2 run nav2_util lifecycle_bringup "$node" > "$LOG_DIR/bringup_$node.log" 2>&1 || true
    # `ros2 lifecycle get`は直後だと空文字を返すことがある(bt_navigatorで実測)。
    # 数回リトライしてから最終判定する。
    local state=""
    for _ in 1 2 3 4 5; do
        state="$(ros2 lifecycle get "$node" 2>/dev/null || true)"
        [[ "$state" == active* ]] && break
        sleep 0.5
    done
    if [[ "$state" != active* ]]; then
        echo "[run_nav2_sim] ⚠️ $node がactiveにならなかった(state: $state)。$LOG_DIR/$node.log と bringup_$node.log を確認" >&2
        exit 1
    fi
}

echo "[run_nav2_sim] 地図を再生成(既定はsim/rooms.test_room由来。--mapで実地図に差し替え可。MuJoCoの世界と一致させる)"
.venv_amcl/bin/python nav2_sim_bridge.py --map-only "${BRIDGE_ARGS[@]}"

echo "[run_nav2_sim] === map_server ==="
# nav2_sim_params.yamlにmap_server:ブロックは無い（amcl_sim_verify.pyの検証時と同じく
# yaml_filenameは-pで直接渡す。README.mdの「2) map_serverとamclを起動」の手順と同じ）
spawn map_server.log ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="$(pwd)/amcl_test_map.yaml"
sleep 1
bringup map_server

echo "[run_nav2_sim] === amcl ==="
spawn amcl.log ros2 run nav2_amcl amcl --ros-args --params-file "$PARAMS"
sleep 1
bringup amcl

# ここでMuJoCoシムを先に起動する。/tf(odom->base_link)を流し続けないと、
# この後 bringup する controller_server/planner_server の local/global costmap が
# activate 時に「TFが来ない」まま待ち続けてデッドロックする（最初の実装で実際に発生）。
# nav2_sim_bridge.py 側は navigate_to_pose アクションサーバの出現を
# 非ブロッキングでポーリングするので、bt_navigator がまだ無くても起動して構わない。
echo "[run_nav2_sim] === MuJoCoシム + ブリッジ (先に起動してTFを流し続ける) ==="
.venv_amcl/bin/python nav2_sim_bridge.py "${BRIDGE_ARGS[@]}" > "$LOG_DIR/nav2_sim_bridge.log" 2>&1 &
BRIDGE_PID=$!
PIDS+=("$BRIDGE_PID")
echo "[run_nav2_sim] 起動: nav2_sim_bridge.py (log: nav2_sim_bridge.log, pid: $BRIDGE_PID)"
sleep 2

echo "[run_nav2_sim] === controller_server ==="
spawn controller_server.log ros2 run nav2_controller controller_server --ros-args --params-file "$PARAMS"
sleep 1
bringup controller_server

echo "[run_nav2_sim] === planner_server ==="
spawn planner_server.log ros2 run nav2_planner planner_server --ros-args --params-file "$PARAMS"
sleep 1
bringup planner_server

echo "[run_nav2_sim] === behavior_server ==="
spawn behavior_server.log ros2 run nav2_behaviors behavior_server --ros-args --params-file "$PARAMS"
sleep 1
bringup behavior_server

echo "[run_nav2_sim] === bt_navigator ==="
spawn bt_navigator.log ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file "$PARAMS"
sleep 1
bringup bt_navigator

echo "[run_nav2_sim] 全ノードactive。ブリッジ(pid $BRIDGE_PID)がゴール到達/タイムアウトするまで待つ"
set +e
wait "$BRIDGE_PID"
STATUS=$?
set -e

echo "[run_nav2_sim] --- nav2_sim_bridge.py の出力 ---"
cat "$LOG_DIR/nav2_sim_bridge.log"
echo "[run_nav2_sim] --- ここまで ---"
echo "[run_nav2_sim] 完了 (exit=$STATUS)。ログ: $LOG_DIR"
exit $STATUS
