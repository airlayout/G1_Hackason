#!/usr/bin/env bash
# Navigation（Nav2 方式）を止め、G1 の足を止める。Mac / Ubuntu どちらの操作PCでもこれ 1 本でよい。
#
#   bash Navigation/real/nav_stop.sh              # 全部止める（G1 の PC2 → この PC の順）
#   bash Navigation/real/nav_stop.sh --no-robot   # G1 に ssh しない（この PC の ROS 側だけ止める）
#   bash Navigation/real/nav_stop.sh --no-ros     # この PC の ROS 側に触らない（G1 側だけ止める）
#
# 手順の意味・手で打つ場合のコマンドは同じフォルダの STOP.md を見ること。
#
# ## 止める対象
#
# | 構成 | どこで動いているか | 止め方 |
# |---|---|---|
# | **nav2_stable**（`g1up.sh`。現行） | **PC2**: Nav2・`cmd_router`・巡回ノード / systemd `g1-sdk-bridge` | `/g1/estop` → 巡回 stop → 走行許可取消 → Goal 取消 → launch を停止 → 発進ゲートを閉じる |
# | 旧 real/（`loco_driver.py`） | **PC2**: `loco_driver.py` / `cmd_vel_bridge.py` | SIGINT で止め、SDK から速度 0 |
# | 旧 nav_stack.sh | **操作PC**: Mac のコンテナ `rviz` / Ubuntu の `/opt/ros/humble` | Goal 取消 → `/cmd_vel` に 0 → Nav2 を停止 |
#
# ## なぜこの順番か
#
# 1. **G1 の PC2 を先に止める。** 足に指令を渡しているのは PC2 だけなので、ここを止めれば
#    操作PC側が何を出していても歩かない
# 2. nav2_stable は **まず `/g1/estop` で E_STOP に落とす。** `cmd_router` がその場でゼロ速度に
#    切り替え、`/g1/clear_estop` を呼ぶまで解除されない（自動解除しない設計）
# 3. **巡回（`patrol_ctl.sh start`）は巡回路を回り続ける。** Goal を取り消しても次の点の Goal を
#    送り直すので、必ず `/g1/patrol/stop` で巡回ノードごと止める
# 4. `loco_driver.py` は **SIGINT で止める。** SIGTERM では `finally` の `StopMove()` が走らず、
#    `continous_move=True`（duration=864000 秒）の最後の速度指令が G1 の中に 10 日間残る
# 5. 発進ゲート（`G1_ARM=--arm`）が開いていれば閉じる。開いたままだと、次に誰かが
#    Nav2 を上げた瞬間に歩ける状態になる。`sudo` にパスワードが要る PC2 では閉じられないので、
#    その場合はコマンドを表示する
#
# ## 環境変数
#
#   G1_SSH          G1 PC2 への ssh 先（既定: ~/.ssh/config に g1 があれば g1、無ければ unitree@192.168.123.164）
#   G1_RVIZ_NAME    操作PCで ROS 2 を動かしているコンテナ名（既定: rviz）
#   NAV_STOP_IFACE  操作PCで DDS を載せる NIC（既定: col0 → 192.168.123.x を持つ NIC → lo の順で探す）
#   ROS_DOMAIN_ID   既定 0
set -uo pipefail

NAME="${G1_RVIZ_NAME:-rviz}"
DO_ROBOT=1
DO_ROS=1
for arg in "$@"; do
    case "$arg" in
        --no-robot) DO_ROBOT=0 ;;
        --no-ros) DO_ROS=0 ;;
        -h|--help) sed -n '2,41p' "$0"; exit 0 ;;
        *) echo "[stop] 不明な引数: $arg" >&2; exit 2 ;;
    esac
done

say() { echo "[stop] $*"; }
FAILED=0
GATE_OPEN=0

# ------------------------------------------------------------------ G1 (PC2) 側

if [ -z "${G1_SSH:-}" ]; then
    if ssh -G g1 2>/dev/null | grep -q '^hostname 192\.168\.123\.'; then
        G1_SSH="g1"
    else
        G1_SSH="unitree@192.168.123.164"
    fi
fi

# PC2 で実行するスクリプト。終了コード: 0 = 止めた / 1 = 止め切れていない / 10 = 止めたがゲートが開いたまま
remote_script() {
    cat <<'REMOTE'
set -u
RC=0
# パターンは [] で括る。pkill -f は自分自身のコマンド行にも当たるため
NAV_PATTERN='ros2 launch g1_navigatio[n]|g1_cmd_router_nod[e]|g1_state_bridge_nod[e]|patrol_nod[e].py|g1_slam_odom_t[f].py|envs/default/lib/nav[2]_'

# --- nav2_stable（現行。g1up.sh / start_nav.sh で上げたもの） ---------------
NAV_HOME=/home/unitree/g1_nav2
PIXI=/home/unitree/.pixi/bin/pixi
if pgrep -f "$NAV_PATTERN" >/dev/null; then
    echo "[stop]   nav2_stable が動いている → E-stop・巡回停止・走行許可取消・Goal 取消"
    if [ -x "$PIXI" ] && [ -d "$NAV_HOME/pc2_humble" ]; then
        # start_nav.sh と同じ環境。**素の shell には ROS を入れない**(D-07)
        ( cd "$NAV_HOME/pc2_humble" && timeout 90 "$PIXI" run bash -lc '
            source /home/unitree/g1_nav2/g1_ws/install/setup.bash
            export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export CYCLONEDDS_URI=file:///home/unitree/g1_nav2/cyclonedds_eth0.xml
            # 発見が済む前の数発は落ちうるので 2 秒間流し続ける。E_STOP はラッチされる
            timeout 8 ros2 topic pub --times 20 -r 10 /g1/estop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 \
                && echo "[stop]   /g1/estop に true を送った（E_STOP。解除は /g1/clear_estop）"
            timeout 10 ros2 service call /g1/patrol/stop std_srvs/srv/Trigger "{}" >/dev/null 2>&1 \
                && echo "[stop]   巡回を止めた（/g1/patrol/stop）"
            timeout 10 ros2 service call /g1/enable_navigation std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 \
                && echo "[stop]   走行許可を取り消した（/g1/enable_navigation false）"
            # 空の CancelGoal（goal_id も stamp も 0）は「全 Goal の取り消し」を意味する
            for a in navigate_to_pose navigate_through_poses follow_waypoints; do
                timeout 10 ros2 service call /$a/_action/cancel_goal action_msgs/srv/CancelGoal "{}" >/dev/null 2>&1 \
                    && echo "[stop]   /$a の Goal を取り消した"
            done
            true
        ' ) || echo "[stop]   ⚠️ ROS のサービス呼び出しが時間切れ。launch の停止に進む"
    else
        echo "[stop]   ⚠️ pixi 環境（$NAV_HOME/pc2_humble）が無い。E-stop を送れないので launch の停止に進む"
    fi
    # launch を止める。cmd_router が居なくなれば SDK 側は cmd_timeout（0.30 秒）で止まる
    pkill -INT -f "$NAV_PATTERN"
    sleep 4
    pkill -KILL -f "$NAV_PATTERN" && echo "[stop]   残っていた nav2_stable のプロセスを強制終了した"
    if pgrep -f "$NAV_PATTERN" >/dev/null; then
        echo "[stop]   ⚠️ nav2_stable のプロセスが残っている:"; pgrep -af "$NAV_PATTERN" | cut -c1-110
        RC=1
    else
        echo "[stop]   nav2_stable（Nav2・cmd_router・巡回）は残っていない"
    fi
else
    echo "[stop]   nav2_stable は動いていない"
fi

# --- 旧 real/（loco_driver.py / cmd_vel_bridge.py） -------------------------
if pgrep -f "loco_drive[r].py|cmd_vel_bridg[e].py" >/dev/null; then
    # SIGINT で止める。finally の StopMove() を走らせるため（SIGTERM では走らない）
    pkill -INT -f "loco_drive[r].py" && echo "[stop]   loco_driver.py に SIGINT を送った"
    pkill -INT -f "cmd_vel_bridg[e].py" && echo "[stop]   cmd_vel_bridge.py に SIGINT を送った"
    sleep 2
    pkill -KILL -f "loco_drive[r].py|cmd_vel_bridg[e].py" && echo "[stop]   loco_driver / cmd_vel_bridge を強制終了した"
fi

# G1 の中に残っている速度指令を 0 で上書きする。loco_driver が強制終了されていると
# StopMove() が送られておらず、continous_move=True の指令が残っているため
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
    print("[stop]   SDK から速度 0 を送った（SetVelocity 戻り値 {}）".format(code))
except Exception as error:
    print("[stop]   ⚠️ SDK から速度 0 を送れなかった: {}".format(error))
    sys.exit(1)
PY
if [ $? -ne 0 ] && ! pgrep -f "g1_sdk_bridge_real_serve[r]" >/dev/null; then
    # g1-sdk-bridge が動いていれば、あちらが cmd_timeout でゼロ速度を送るので致命ではない
    RC=1
fi

if pgrep -f "loco_drive[r].py|cmd_vel_bridg[e].py" >/dev/null; then
    echo "[stop]   ⚠️ loco_driver / cmd_vel_bridge が残っている"; RC=1
fi

# --- 発進ゲート（g1-sdk-bridge の --arm） -----------------------------------
BRIDGE=$(pgrep -af "g1_sdk_bridge_real_serve[r]" | head -1)
PERSIST=$(grep '^G1_ARM=' /etc/default/g1-sdk-bridge 2>/dev/null)
case "$BRIDGE $PERSIST" in
    *--arm*)
        if sudo -n sed -i 's/^G1_ARM=.*/G1_ARM=/' /etc/default/g1-sdk-bridge 2>/dev/null \
                && sudo -n systemctl restart g1-sdk-bridge 2>/dev/null; then
            echo "[stop]   発進ゲートを閉じた（G1_ARM= にして g1-sdk-bridge を restart）"
        else
            echo "[stop]   ⚠️ 発進ゲートが開いたまま（sudo にパスワードが要るので閉じられなかった）"
            echo "[stop]      いまのプロセス: ${BRIDGE:-(停止中)}"
            echo "[stop]      /etc/default: ${PERSIST:-(無し)}"
            [ "$RC" -eq 0 ] && RC=10
        fi
        ;;
    *)
        echo "[stop]   発進ゲートは閉じている（--arm 無し）"
        ;;
esac
exit $RC
REMOTE
}

stop_robot() {
    say "1/3 G1 の PC2 を止める（ssh $G1_SSH）"
    local rc
    # スクリプトは base64 にして引数で渡し、PC2 でファイルに戻してから実行する。
    # - 標準入力で渡さない: PC2 の ~/.bashrc はログイン時に「ros:foxy(1) noetic(2) ?」と
    #   read するので、食われうる（-n で塞ぐ）
    # - `bash -c <本文>` で渡さない: 本文がコマンド行に載り、中の `pkill -f` が
    #   **自分自身に当たって止まる**（[] で括ったパターンも本文中の文字列には当たる）
    local b64
    b64=$(remote_script | base64 | tr -d '\n')
    ssh -n -o ConnectTimeout=5 -o BatchMode=yes "$G1_SSH" \
        "f=\$(mktemp /tmp/nav_stop.XXXXXX) && echo $b64 | base64 -d > \$f && bash \$f; rc=\$?; rm -f \$f; exit \$rc"
    rc=$?
    case "$rc" in
        0) ;;
        10)
            GATE_OPEN=1
            ;;
        *)
            say "   ⚠️ G1 側を止め切れなかった（ssh 不通 / プロセスが残っている / 速度 0 を送れなかった）"
            say "   → リモコンでダンピング（L2+B）にすること。脱力するので支えてから"
            FAILED=1
            ;;
    esac
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
    say "2/3 この PC の ROS 側（旧 nav_stack.sh 構成）の Goal を取り消し、Nav2 を止める"
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
        say "   この PC には ROS 2 が無い（コンテナ $NAME も /opt/ros/humble も無い）。飛ばす"
        say "   （現行の nav2_stable は PC2 で動くので、1/3 で止まっていれば問題ない）"
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
if [ "$FAILED" -eq 0 ] && [ "$DO_ROBOT" -eq 0 ]; then
    say "   この PC の ROS 側は止めた。G1 の PC2 は触っていない（--no-robot）"
    exit 0
fi
if [ "$FAILED" -eq 0 ] && [ "$GATE_OPEN" -eq 1 ]; then
    say "   止めた。ただし発進ゲートが開いたまま。PC2 で人が閉じること（パスワードを聞かれる）:"
    say "     ssh -t $G1_SSH \"sudo sed -i 's/^G1_ARM=.*/G1_ARM=/' /etc/default/g1-sdk-bridge && sudo systemctl restart g1-sdk-bridge\""
    exit 0
fi
if [ "$FAILED" -eq 0 ]; then
    say "   止めた。ランニングモードに入れても Navigation の経路は歩かないはず"
    say "   （nav2_stable を再開するときは /g1/clear_estop が要る）"
    exit 0
fi
say "   ⚠️ 一部を止め切れていない。上の ⚠️ を見ること。手順は Navigation/real/STOP.md"
exit 1
