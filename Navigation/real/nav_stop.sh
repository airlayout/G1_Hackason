#!/usr/bin/env bash
# Navigation（Nav2 方式）を止め、G1 の足を止める。Mac / Ubuntu どちらでもこれ 1 本でよい。
#
#   bash Navigation/real/nav_stop.sh              # 全部止める（G1 側 → ROS 側の順）
#   bash Navigation/real/nav_stop.sh --no-robot   # G1 に ssh しない（ROS 側だけ止める）
#   bash Navigation/real/nav_stop.sh --no-ros     # ROS 側に触らない（G1 側だけ止める）
#
# 手順の意味・手で打つ場合のコマンドは同じフォルダの STOP.md を見ること。
#
# ## なぜこの順番か
#
# 1. **G1 側を先に止める。** 足に指令を渡しているのは PC2 の `loco_driver.py` だけなので、
#    ここを止めれば ROS 側が何を出していても歩かない
# 2. `loco_driver.py` は **SIGINT で止める。** `pkill` の既定の SIGTERM では `finally` の
#    `StopMove()` が走らず、`continous_move=True`（duration=864000 秒）の最後の速度指令が
#    G1 の中に 10 日間残る。念のため SDK から速度 0 も直接送って上書きする
# 3. そのあと Nav2 の Goal を取り消し、Nav2 一式を止める。**Goal が生きている限り、
#    G1 を再起動しても Nav2 は同じ経路の続きを出し続ける**（2026-09-28 に踏んだ）
#
# ## ROS 側をどこで動かすか（自動で選ぶ）
#
# | 条件 | 実行先 |
# |---|---|
# | Docker コンテナ `rviz`（`G1_RVIZ_NAME`）が動いている | コンテナの中（Mac の `start_rviz_mac.sh` 構成） |
# | `/opt/ros/humble/setup.bash` がある | この PC の ROS 2（Ubuntu 直入れ構成） |
# | どちらも無い | ROS 側は飛ばす（G1 側だけ止める） |
#
# ## 環境変数
#
#   G1_SSH          G1 PC2 への ssh 先（既定: ~/.ssh/config に g1 があれば g1、無ければ unitree@192.168.123.164）
#   G1_RVIZ_NAME    ROS 2 を動かしているコンテナ名（既定: rviz）
#   NAV_STOP_IFACE  DDS を載せる NIC（既定: col0 → 192.168.123.x を持つ NIC → lo の順で探す）
#   ROS_DOMAIN_ID   既定 0
set -uo pipefail

NAME="${G1_RVIZ_NAME:-rviz}"
DO_ROBOT=1
DO_ROS=1
for arg in "$@"; do
    case "$arg" in
        --no-robot) DO_ROBOT=0 ;;
        --no-ros) DO_ROS=0 ;;
        -h|--help) sed -n '2,36p' "$0"; exit 0 ;;
        *) echo "[stop] 不明な引数: $arg" >&2; exit 2 ;;
    esac
done

say() { echo "[stop] $*"; }
FAILED=0
SKIPPED=0

# ------------------------------------------------------------------ G1 (PC2) 側

if [ -z "${G1_SSH:-}" ]; then
    if ssh -G g1 2>/dev/null | grep -q '^hostname 192\.168\.123\.'; then
        G1_SSH="g1"
    else
        G1_SSH="unitree@192.168.123.164"
    fi
fi

stop_robot() {
    say "1/3 G1 側を止める（ssh $G1_SSH）"
    # パターンは [] で括る。pkill -f は自分自身のコマンド行（ssh の引数）にも当たるため
    if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$G1_SSH" bash -s <<'REMOTE'
set -u
# SIGINT で止める。finally の StopMove() を走らせるため（SIGTERM では走らない）
pkill -INT -f "loco_drive[r].py" && echo "[stop]   loco_driver.py に SIGINT を送った"
pkill -INT -f "cmd_vel_bridg[e].py" && echo "[stop]   cmd_vel_bridge.py に SIGINT を送った"
sleep 2
# 2 秒待っても残っていたら強制終了する
pkill -KILL -f "loco_drive[r].py" && echo "[stop]   loco_driver.py を強制終了した"
pkill -KILL -f "cmd_vel_bridg[e].py" && echo "[stop]   cmd_vel_bridge.py を強制終了した"

# G1 の中に残っている速度指令を 0 で上書きする。loco_driver が強制終了された場合に
# StopMove() が送られていない可能性があるため、ここで必ず送る
python3 - <<'PY'
import sys
sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    ChannelFactoryInitialize(0, "eth0")
    client = LocoClient()
    client.SetTimeout(5.0)
    client.Init()
    code = client.SetVelocity(0.0, 0.0, 0.0, 1.0)
    print("[stop]   速度 0 を送った（SetVelocity 戻り値 {}）".format(code))
except Exception as error:
    print("[stop]   速度 0 を送れなかった: {}".format(error))
    sys.exit(1)
PY
SDK_RC=$?

LEFT=$(pgrep -af "loco_drive[r].py|cmd_vel_bridg[e].py")
if [ -n "$LEFT" ]; then
    echo "[stop]   まだ残っている:"; echo "$LEFT"; exit 1
fi
echo "[stop]   PC2 に loco_driver / cmd_vel_bridge は残っていない"
exit $SDK_RC
REMOTE
    then
        say "   ⚠️ G1 側を止め切れなかった（ssh 不通 / 速度 0 を送れなかった / プロセスが残っている）"
        say "   → リモコンでダンピング（L2+B）にすること。脱力するので支えてから"
        FAILED=1
    fi
}

# ------------------------------------------------------------------ ROS 側

# ROS 側で実行するスクリプト。コンテナの中でもホストでも同じものを流す
ros_script() {
    cat <<'ROS'
set -u
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
IFACE="${NAV_STOP_IFACE:-}"
if [ -z "$IFACE" ]; then
    if [ -e /sys/class/net/col0 ]; then
        IFACE=col0
    else
        IFACE=$(ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}')
    fi
fi
if [ -n "$IFACE" ]; then
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$IFACE\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
else
    # G1 に繋がっていない（offline 構成）。nav_stack.sh と同じくループバックに閉じる
    IFACE=lo
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
fi
echo "[stop]   DDS: $IFACE / ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# 空の CancelGoal（goal_id も stamp も 0）は「全 Goal の取り消し」を意味する
for action in navigate_to_pose navigate_through_poses follow_waypoints; do
    if timeout 8 ros2 service call "/$action/_action/cancel_goal" \
            action_msgs/srv/CancelGoal "{}" >/tmp/nav_stop_cancel.log 2>&1; then
        echo "[stop]   /$action の Goal を取り消した"
    fi
done

# Nav2 を止める前に 0 を流しておく（loco_driver が生きていても止まるように）
timeout 5 ros2 topic pub --times 5 -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 \
    && echo "[stop]   /cmd_vel に 0 を 5 回送った"

# nav_stack.sh stop と同じ対象。パターンは [] で括る（pkill -f が自分に当たらないように）
pkill -INT -f "navigation_launc[h]"
pkill -INT -f "nav2_[a-z]"
pkill -INT -f "octomap_serve[r]"
pkill -INT -f "odom_to_t[f]"
pkill -INT -f "ros2 bag pla[y]"
sleep 3
pkill -KILL -f "navigation_launc[h]|nav2_[a-z]|octomap_serve[r]|odom_to_t[f]|ros2 bag pla[y]"

LEFT=$(pgrep -af "navigation_launc[h]|nav2_[a-z]|octomap_serve[r]|odom_to_t[f]|ros2 bag pla[y]")
if [ -n "$LEFT" ]; then
    echo "[stop]   まだ残っている:"; echo "$LEFT" | cut -c1-110
    exit 1
fi
echo "[stop]   Nav2 一式は残っていない（RViz2 は残す）"

# /cmd_vel をまだ出している者が居ないか。居れば別の PC で Nav2 が動いている
PUBS=$(timeout 10 ros2 topic info /cmd_vel --verbose --no-daemon 2>/dev/null \
    | awk '/^Publisher count:/ {print $3}')
if [ -n "$PUBS" ] && [ "$PUBS" != "0" ]; then
    echo "[stop]   ⚠️ /cmd_vel の Publisher がまだ ${PUBS} 個いる（別の PC で Nav2 が動いている可能性）:"
    timeout 10 ros2 topic info /cmd_vel --verbose --no-daemon 2>/dev/null \
        | grep -E "Node name|Node namespace" | sed 's/^/[stop]     /'
    exit 1
fi
echo "[stop]   /cmd_vel の Publisher は 0"
ROS
}

stop_ros() {
    say "2/3 Nav2 の Goal を取り消し、Nav2 一式を止める"
    local rc
    if command -v docker >/dev/null 2>&1 \
            && [ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)" = "true" ]; then
        say "   実行先: コンテナ $NAME"
        ros_script | docker exec -i -u ubuntu \
            -e NAV_STOP_IFACE="${NAV_STOP_IFACE:-}" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
            "$NAME" bash -s
        rc=$?
    elif [ -f /opt/ros/humble/setup.bash ]; then
        say "   実行先: この PC の ROS 2（/opt/ros/humble）"
        ros_script | bash -s
        rc=$?
    else
        say "   ROS 2 が見つからない（コンテナ $NAME も /opt/ros/humble も無い）。ROS 側は飛ばす"
        say "   → Nav2 を別の PC で動かしているなら、その PC でもこのスクリプトを実行すること"
        SKIPPED=1
        return
    fi
    if [ "$rc" -ne 0 ]; then
        say "   ⚠️ ROS 側を止め切れなかった（上の表示を確認すること）"
        FAILED=1
    fi
}

# ------------------------------------------------------------------ 本体

if [ "$DO_ROBOT" -eq 1 ]; then stop_robot; else say "1/3 G1 側は飛ばす（--no-robot）"; fi
if [ "$DO_ROS" -eq 1 ]; then stop_ros; else say "2/3 ROS 側は飛ばす（--no-ros）"; fi

say "3/3 結果"
if [ "$FAILED" -eq 0 ] && [ "$SKIPPED" -eq 1 ]; then
    say "   G1 側は止めた。ROS 側はこの PC に無いので未確認（Nav2 を動かしている PC でも実行すること）"
    exit 0
fi
if [ "$FAILED" -eq 0 ]; then
    say "   止めた。ランニングモードに入れても Navigation の経路は歩かないはず"
    exit 0
fi
say "   ⚠️ 一部を止め切れていない。上の ⚠️ を見ること。手順は Navigation/real/STOP.md"
exit 1
