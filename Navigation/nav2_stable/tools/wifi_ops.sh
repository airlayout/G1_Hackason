#!/usr/bin/env bash
# 有線で立ち上げた Nav2 を、ケーブルを抜いて無線で操作するための道具。**操作PC で叩く。**
#
#   tools/wifi_ops.sh ip                 # 有線経由で PC2 の無線 IP を調べる（最初に1回）
#   export G1_WIFI_IP=<上で出た IP>
#   tools/wifi_ops.sh check              # 無線で ssh が通るか・PC2 の WiFi 省電力
#   tools/wifi_ops.sh fix                # PC2 の WiFi 省電力を切る（sudo のパスワードを聞かれる）
#   tools/wifi_ops.sh handover           # heartbeat を有線→無線に切り替え、無線だけで alive か確かめる
#   （ここでケーブルを抜く）
#   tools/wifi_ops.sh status             # bridge_status と巡回の状態
#   tools/wifi_ops.sh pose [秒]          # 地図上の位置と向き（map→base_link）を文字で流す。既定 1 回
#   tools/wifi_ops.sh enable             # 走行許可（g1up.sh --enable）
#   tools/wifi_ops.sh patrol start|pause|stop|status
#   tools/wifi_ops.sh clear              # FAULT を解除する（自動では解除しない。人が叩く）
#   tools/wifi_ops.sh clearloop start|stop   # local costmap を 20 秒ごとに消すループ（PC2 で常駐）
#   tools/wifi_ops.sh mapview start|stop     # 地図上の現在地をブラウザで見る（http://$G1_WIFI_IP:8090/）
#   tools/wifi_ops.sh hbtest [秒]        # heartbeat の遅れを測る（既定 60 秒）
#   tools/wifi_ops.sh hb-stop            # このスクリプトが上げた無線 heartbeat を止める
#
# 止めるときは: G1_SSH=unitree@$G1_WIFI_IP bash Navigation/real/nav_stop.sh --no-ros
#
# ## なぜこの形か（2026-10-13）
#
# 9/15 にテザリングで踏んだ失敗（「トピックは見えるのにデータが来ない」、RViz が真っ暗、
# AP のマルチキャスト遮断）は、**すべて DDS を無線に通そうとして起きた**。
# 一方、無線で要るものは DDS を使わずに済む:
#   - heartbeat は UDP で、PC2 側は全インターフェースで受けている（heartbeat.cpp の INADDR_ANY）
#   - 走行許可・巡回の開始停止・FAULT 解除は、PC2 上で ROS コマンドを叩けばよい（ssh で足りる）
# そこで **DDS は無線に一切通さない**。PC2 の CycloneDDS は eth0 固定のまま触らない。
# 代償: 歩行中に RViz は見られない（状態は status、機体は目視）。Goal は巡回路で与える。
#
# ## 切り替えの順番（handover）
#
# 1. 有線の heartbeat を生かしたまま、無線 IP 宛ての heartbeat を足す
# 2. 有線の heartbeat を止める → **無線だけで** operator_heartbeat が alive か確かめる
# 3. alive でなければ有線の heartbeat を戻して中止する
# ⚠️ **enable（走行許可）の前にやること。** heartbeat が走行に効くのは NAVIGATING 以降なので、
#    READY のうちなら切り替えに失敗しても機体には影響しない。
#
# ## 環境変数
#
#   G1_WIFI_IP      PC2 の無線 IP（ip サブコマンドで調べる）。ip 以外では必須
#   G1_SSH          有線での ssh 先（ip サブコマンドで使う。既定: g1 があれば g1、無ければ unitree@192.168.123.164）
#   G1_WIRED_IP     有線 heartbeat の宛先（既定 192.168.123.164）
#   G1_HB_SENDER    heartbeat 送信プログラム（既定: PATH 上の g1_heartbeat_sender）
set -uo pipefail

CMD="${1:-}"
[ $# -gt 0 ] && shift

WIRED_IP="${G1_WIRED_IP:-192.168.123.164}"
HB_SENDER="${G1_HB_SENDER:-g1_heartbeat_sender}"
HB_PIDFILE="${XDG_RUNTIME_DIR:-/tmp}/g1_wifi_heartbeat.pid"
HB_LOG="${XDG_RUNTIME_DIR:-/tmp}/g1_wifi_heartbeat.log"
# 1 回パスワードを打てば 10 分間は ssh を使い回す（当日の時短）
SSH_OPTS=(-o ConnectTimeout=5 -o ServerAliveInterval=2 -o ServerAliveCountMax=3
          -o ControlMaster=auto -o "ControlPath=$HOME/.ssh/cm-g1-%r@%h:%p" -o ControlPersist=10m)

say()  { echo "[wifi] $*"; }
ok()   { echo "[wifi]   ✅ $*"; }
warn() { echo "[wifi]   ⚠️  $*"; }
die()  { echo "[wifi]   ❌ $*" >&2; exit 1; }

need_ip() {
    [ -n "${G1_WIFI_IP:-}" ] || die "G1_WIFI_IP が無い。先に '$0 ip' で調べて export すること"
}

# PC2 で ROS コマンドを叩く。start_nav.sh / nav_stop.sh と同じ環境
# （**素の shell には ROS を入れない**。D-07）
ros_remote() {
    local host="$1"; shift
    ssh "${SSH_OPTS[@]}" "unitree@$host" "cd /home/unitree/g1_nav2/pc2_humble && timeout 60 /home/unitree/.pixi/bin/pixi run bash -lc '
        source /home/unitree/g1_nav2/g1_ws/install/setup.bash
        export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI=file:///home/unitree/g1_nav2/cyclonedds_eth0.xml
        $*'"
}

# bridge_status（DiagnosticArray なので字下げされて出る）から状態と各項目を抜く
bridge_summary() {
    ros_remote "$1" "timeout 12 ros2 topic echo /g1/bridge_status --once 2>&1" \
        | grep -A1 -E "message:|key: (fault_reason|operator_heartbeat|operator_heartbeat_age_s|tf|sensor)$" \
        | grep -E "message:|key:|value:" | sed 's/^ */[wifi]     /'
}

hb_alive() {
    ros_remote "$1" "timeout 12 ros2 topic echo /g1/bridge_status --once 2>&1" \
        | grep -A1 -E "key: operator_heartbeat$" | grep -q "alive"
}

wired_ssh() {
    if [ -n "${G1_SSH:-}" ]; then echo "$G1_SSH"
    elif ssh -G g1 2>/dev/null | grep -q '^hostname 192\.168\.123\.'; then echo g1
    else echo "unitree@$WIRED_IP"; fi
}

case "$CMD" in
ip)
    H="$(wired_ssh)"
    say "有線（$H）で PC2 の無線の状態を見る"
    ssh "${SSH_OPTS[@]}" "$H" '
        echo "[wifi]   wlan0: $(ip -4 -o addr show wlan0 2>/dev/null | awk "{print \$4}")"
        echo "[wifi]   SSID : $(iwgetid -r 2>/dev/null || nmcli -t -f active,ssid dev wifi 2>/dev/null | grep "^yes" | cut -d: -f2)"
        echo "[wifi]   既定ルート: $(ip route show default | head -1)"
    ' || die "有線で ssh できない（ケーブルと g1-link を確認）"
    say "IP が出たら: export G1_WIFI_IP=<wlan0 の / より前>"
    say "wlan0 が空なら PC2 が WiFi に繋がっていない: ssh -t $H 'sudo nmcli device wifi connect <SSID> password <パスワード>'"
    ;;

check)
    need_ip
    say "無線（$G1_WIFI_IP）の疎通を見る"
    ping -c 3 -W 2 "$G1_WIFI_IP" >/dev/null 2>&1 && ok "ping が通る" \
        || die "ping が通らない。操作PC が同じルータに繋がっているか／ルータの AP 分離（端末同士の通信遮断）が無効か"
    ssh "${SSH_OPTS[@]}" "unitree@$G1_WIFI_IP" true && ok "無線で ssh できる" || die "無線で ssh できない"
    PS="$(ssh "${SSH_OPTS[@]}" "unitree@$G1_WIFI_IP" 'iw dev wlan0 get power_save 2>/dev/null')"
    case "$PS" in
        *off*) ok "PC2 の WiFi 省電力: off" ;;
        *on*)  warn "PC2 の WiFi 省電力が on。**heartbeat が途切れて FAULT になる**（9/15 実機）→ '$0 fix'" ;;
        *)     warn "PC2 の WiFi 省電力が読めない: $PS" ;;
    esac
    CY="$(ssh "${SSH_OPTS[@]}" "unitree@$G1_WIFI_IP" 'grep -o "NetworkInterface name=\"[a-z0-9]*\"" /home/unitree/g1_nav2/cyclonedds_eth0.xml 2>/dev/null')"
    case "$CY" in
        *eth0*) ok "PC2 の CycloneDDS は eth0 固定（DDS は無線に出ない）" ;;
        *)      warn "PC2 の CycloneDDS が eth0 固定になっていない: '$CY'" ;;
    esac
    command -v "$HB_SENDER" >/dev/null 2>&1 && ok "heartbeat 送信プログラムがある" \
        || warn "この PC に $HB_SENDER が無い（G1_HB_SENDER で場所を指定）"
    ;;

fix)
    need_ip
    say "PC2 の WiFi 省電力を切る。**sudo のパスワード（PC2 の unitree）を聞かれる**"
    # 再起動後も効くよう NetworkManager の接続設定にも書く（powersave 2 = 無効）
    ssh -t "${SSH_OPTS[@]}" "unitree@$G1_WIFI_IP" '
        sudo iw dev wlan0 set power_save off
        C="$(nmcli -t -f NAME,DEVICE connection show --active | grep ":wlan0$" | cut -d: -f1)"
        [ -n "$C" ] && sudo nmcli connection modify "$C" 802-11-wireless.powersave 2
        echo "[wifi]   $(iw dev wlan0 get power_save)（接続: $C）"
    '
    ;;

handover)
    need_ip
    command -v "$HB_SENDER" >/dev/null 2>&1 || die "$HB_SENDER が無い（G1_HB_SENDER で場所を指定）"
    if [ -f "$HB_PIDFILE" ] && kill -0 "$(cat "$HB_PIDFILE")" 2>/dev/null; then
        ok "無線の heartbeat はすでに動いている（pid $(cat "$HB_PIDFILE")）"
    else
        nohup "$HB_SENDER" --host "$G1_WIFI_IP" >"$HB_LOG" 2>&1 &
        echo $! >"$HB_PIDFILE"
        sleep 1
        kill -0 "$(cat "$HB_PIDFILE")" 2>/dev/null || die "無線の heartbeat が起動しない（$HB_LOG）"
        ok "無線の heartbeat を開始した → $G1_WIFI_IP（pid $(cat "$HB_PIDFILE")）"
    fi
    # パターンは [] で括る。pkill -f は自分自身のコマンド行にも当たるため
    WIRED_PAT="g1_heartbeat_sende[r] --host $WIRED_IP"
    if pgrep -f "$WIRED_PAT" >/dev/null; then
        pkill -f "$WIRED_PAT" && ok "有線の heartbeat を止めた（無線だけで確かめるため）"
        WIRED_WAS_RUNNING=1
    else
        WIRED_WAS_RUNNING=0
    fi
    sleep 3
    if hb_alive "$G1_WIFI_IP"; then
        ok "**無線だけで operator_heartbeat が alive。ケーブルを抜いてよい**"
        bridge_summary "$G1_WIFI_IP"
    else
        warn "無線だけでは operator_heartbeat が alive にならない"
        if [ "$WIRED_WAS_RUNNING" = 1 ]; then
            nohup "$HB_SENDER" --host "$WIRED_IP" >/dev/null 2>&1 &
            warn "有線の heartbeat を戻した。**ケーブルは抜かないこと**"
        fi
        die "原因の候補: PC2 の受信ポートがルータ側で遮断されている／IP 違い。'$0 check' から"
    fi
    ;;

status)
    need_ip
    say "bridge_status（$G1_WIFI_IP）"
    bridge_summary "$G1_WIFI_IP"
    ros_remote "$G1_WIFI_IP" "/home/unitree/g1_nav2/tools/patrol_ctl.sh status 2>&1 | tail -2" | sed 's/^/[wifi]   巡回: /'
    ;;

pose)
    # 無線では RViz が見られない（DDS を無線に通さない）ので、位置は数値で見る
    need_ip
    SEC="${1:-}"
    if [ -z "$SEC" ]; then
        ros_remote "$G1_WIFI_IP" "timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -m2 -E \"Translation|degree\"" \
            | sed 's/^ *- */[wifi]   /'
    else
        say "位置を ${SEC} 秒流す（x y [m] / 向き yaw は RPY の3つ目 [度]）"
        ros_remote "$G1_WIFI_IP" "timeout $SEC ros2 run tf2_ros tf2_echo map base_link 1.0 2>&1 | grep --line-buffered -E \"Translation|degree\"" \
            | sed -u 's/^ *- */[wifi]   /'
    fi
    ;;

enable)
    need_ip
    say "走行許可を出す（発進ゲートは開けない。D-07）"
    ssh "${SSH_OPTS[@]}" "unitree@$G1_WIFI_IP" "/home/unitree/g1_nav2/g1up.sh --enable"
    ;;

patrol)
    need_ip
    SUB="${1:-status}"
    case "$SUB" in start|pause|stop|skip|status) ;; *) die "patrol の引数は start|pause|stop|skip|status" ;; esac
    ros_remote "$G1_WIFI_IP" "/home/unitree/g1_nav2/tools/patrol_ctl.sh $SUB 2>&1 | tail -5"
    ;;

clear)
    need_ip
    say "FAULT を解除する（STANDBY/READY に戻るだけ。走らせるには再度 enable）"
    ros_remote "$G1_WIFI_IP" "ros2 service call /g1/clear_fault std_srvs/srv/Trigger {} 2>&1 | tail -2"
    ;;

clearloop)
    # 2026-09-25 に巡回を1周させた暫定処置（findings/collision_ahead_cause_20260925.md §4）。
    # 死角の中に消し残りが溜まり `collision ahead` で止まるのを、定期的に消して避ける。
    # ⚠️⚠️ 消す瞬間は本物の障害物も消える。死角(〜1.6m)の中のものは立ち直らない。人が見ている場でのみ
    # ssh が切れても続くよう PC2 側で setsid する（ケーブルを抜く前に上げても生き残る）
    need_ip
    case "${1:-}" in
    # ⚠️ pgrep -f で探すと、ssh で渡したこのコマンド自身（ループの文字列を含む）に当たって
    #    「すでに動いている」と誤判定した（2026-10-14 実機）。PID ファイルで管理する
    start)
        ssh "${SSH_OPTS[@]}" "unitree@$G1_WIFI_IP" '
            P=/tmp/g1_costmap_clear.pid
            if [ -f $P ] && kill -0 $(cat $P) 2>/dev/null; then echo "[wifi]   ✅ すでに動いている（pid $(cat $P)）"; exit 0; fi
            # cd を && で繋ぐと全体がサブシェルになり、$! がループではなくそのシェルを指す
            cd /home/unitree/g1_nav2/pc2_humble || exit 1
            setsid nohup /home/unitree/.pixi/bin/pixi run bash -lc "
                source /home/unitree/g1_nav2/g1_ws/install/setup.bash
                export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
                export CYCLONEDDS_URI=file:///home/unitree/g1_nav2/cyclonedds_eth0.xml
                while true; do
                    ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap \"{}\" >/dev/null 2>&1
                    sleep 20
                done" >/tmp/costmap_clear.log 2>&1 </dev/null &
            echo $! > $P
            sleep 2
            kill -0 $(cat $P) 2>/dev/null && echo "[wifi]   ✅ costmap 消去ループを開始した（20秒ごと、pid $(cat $P)）" \
                || { echo "[wifi]   ❌ 起動しない（/tmp/costmap_clear.log）"; exit 1; }'
        ;;
    stop)
        # setsid でプロセスグループの長になっているので、グループごと止める
        ssh "${SSH_OPTS[@]}" "unitree@$G1_WIFI_IP" '
            P=/tmp/g1_costmap_clear.pid
            if [ -f $P ] && kill -- -$(cat $P) 2>/dev/null; then echo "[wifi]   ✅ costmap 消去ループを止めた"; else echo "[wifi]   動いていない"; fi
            rm -f $P'
        ;;
    *) die "clearloop の引数は start|stop" ;;
    esac
    ;;

mapview)
    # 無線では RViz が使えないので、PC2 で地図に現在地を描いて HTTP で配る（tools/map_web.py）。
    # ROS の購読は PC2 の中で済み、無線には HTTP だけが流れる（DDS は無線に出ない）
    need_ip
    PORT="${G1_MAPVIEW_PORT:-8090}"
    case "${1:-}" in
    start)
        ssh "${SSH_OPTS[@]}" "unitree@$G1_WIFI_IP" "
            P=/tmp/g1_map_web.pid
            [ -f /home/unitree/g1_nav2/tools/map_web.py ] || { echo '[wifi]   ❌ PC2 に map_web.py が無い。rsync -a tools/ g1:/home/unitree/g1_nav2/tools/ で配置する'; exit 1; }
            if [ -f \$P ] && kill -0 \$(cat \$P) 2>/dev/null; then echo '[wifi]   ✅ すでに動いている'; exit 0; fi
            cd /home/unitree/g1_nav2/pc2_humble || exit 1
            setsid nohup /home/unitree/.pixi/bin/pixi run bash -lc '
                source /home/unitree/g1_nav2/g1_ws/install/setup.bash
                export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
                export CYCLONEDDS_URI=file:///home/unitree/g1_nav2/cyclonedds_eth0.xml
                exec python3 /home/unitree/g1_nav2/tools/map_web.py --port $PORT' >/tmp/map_web.log 2>&1 </dev/null &
            echo \$! > \$P
            sleep 4
            kill -0 \$(cat \$P) 2>/dev/null || { echo '[wifi]   ❌ 起動しない（/tmp/map_web.log）'; tail -5 /tmp/map_web.log; exit 1; }"
        ok "ブラウザで開く: http://$G1_WIFI_IP:$PORT/"
        ;;
    stop)
        ssh "${SSH_OPTS[@]}" "unitree@$G1_WIFI_IP" '
            P=/tmp/g1_map_web.pid
            if [ -f $P ] && kill -- -$(cat $P) 2>/dev/null; then echo "[wifi]   ✅ 地図表示を止めた"; else echo "[wifi]   動いていない"; fi
            rm -f $P'
        ;;
    *) die "mapview の引数は start|stop" ;;
    esac
    ;;

hbtest)
    need_ip
    SEC="${1:-60}"
    say "heartbeat の遅れを ${SEC} 秒測る（2.0 秒運用なら、最大が 1.5 秒未満であること）"
    ssh "${SSH_OPTS[@]}" "unitree@$G1_WIFI_IP" "bash /home/unitree/g1_nav2/tools/hb_stability.sh $SEC"
    ;;

hb-stop)
    if [ -f "$HB_PIDFILE" ] && kill "$(cat "$HB_PIDFILE")" 2>/dev/null; then
        ok "無線の heartbeat を止めた（**NAVIGATING 中なら operator_lost で FAULT になる**）"
    else
        say "無線の heartbeat は動いていない"
    fi
    rm -f "$HB_PIDFILE"
    ;;

*)
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
    [ -z "$CMD" ] || [ "$CMD" = "-h" ] || [ "$CMD" = "--help" ] || exit 2
    ;;
esac
