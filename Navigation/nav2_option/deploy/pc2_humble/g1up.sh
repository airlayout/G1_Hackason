#!/usr/bin/env bash
# G1 の Nav2 を §2〜§7 まで一気に立ち上げる。**PC2 で叩く。**
#
#     ssh g1          # ros:foxy(1) noetic(2)? には **Enter だけ**
#     ~/g1_nav2/g1up.sh
#
# ⚠️ **配置先は `~/g1_nav2/g1up.sh`**（`~/g1_nav2/{pc2_humble,g1_ws,tools}` と並ぶ位置）。
# スクリプトは自分の居場所を基準に他の部品を探すので、別の場所に置くと動かない。
#
# ## なぜ要るのか
#
# 2026-09-15 の実機セッションでは、立ち上げに**10ステップの手作業**が必要で、
# 付きっきりで30分かかった。週2時間・初心者中心のチームではこれが回らない。
# **現地の時間は計測に使いたい。**
#
# ## やること / やらないこと
#
# やる  : §1 姿勢の検査 → §2 ブリッジ起動(ゲート閉) → §3 内蔵SLAM → §4 記録
#         → §5 Nav2 → §7 自己位置合わせ
#
# ⚠️ **実行前に、機体を「歩かせるときと同じ通常の立位」にして、出発位置に置くこと。**
# §5 の校正と §7 の照合は**いまの姿勢と位置で決まる**ので、あとで動かすと無効になる。
# 人は機体から 2m 以上離れること(近いと costmap と地図照合の両方が劣化する)。
# やらない: **発進ゲートの開放と Goal 送信（巡回の開始も含む）**。
#          ここは人が判断して叩く(D-07 の設計意図)。
#          操作PC 側の heartbeat と RViz も別途(最後に手順を表示する)
#
# ## 前提
#
# `sudo systemctl start g1-sdk-bridge` を NOPASSWD にしておくこと(一度だけ):
#
#     sudo tee /etc/sudoers.d/g1-bridge >/dev/null <<'EOS'
#     unitree ALL=(root) NOPASSWD: /bin/systemctl start g1-sdk-bridge
#     EOS
#     sudo chmod 440 /etc/sudoers.d/g1-bridge
#
# ⚠️ **発進ゲートの開放(`G1_ARM=--arm` + restart)は意図的に NOPASSWD にしない。**
# 「再起動したら勝手に動けるようになっていた」を構造的に防ぐため(D-07)。
#
# ## 巡回モードを使うとき
#
#     ~/g1_nav2/g1up.sh --patrol ~/g1_nav2/patrol_room_a.yaml
#
# 巡回路を渡しても**ここでは走り出さない**。⑥で `patrol_ctl.sh start` を叩くまで
# 巡回ノードは IDLE のまま。巡回路は現地で `tools/record_waypoints.py` で作る
# (地図が 9/07 取得で現状と合っていないので、座標を手で書かないこと)。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIXI_DIR="$HERE/pc2_humble"
[ -d "$PIXI_DIR" ] || PIXI_DIR="$HERE"          # deploy/pc2_humble に置いた場合
WS="$HERE/g1_ws"
TOOLS="$HERE/tools"
CYCLONE_CFG="$HERE/cyclonedds_eth0.xml"
PIXI="$HOME/.pixi/bin/pixi"
# ⚠️ 2026-09-24（後半）に **既定を room_a（9/11 版）へ戻した**。差し替えは --map で:
#   --map .../room_a_map_20260911.yaml            手編集していない版
#   --map .../room_a_map.yaml                     9/07 の map_20260907.pcd 由来（旧）
# ⚠️ 既定の `_edited` は手で 28 箇所開けたもの（maps/grids/EDITS.md に理由と座標）。
#   --map .../room_b_map_Sorasta_20260923.yaml    Sorasta（⚠️ /dog_odom 由来）
MAP_DEFAULT="$WS/install/g1_navigation/share/g1_navigation/maps/room_a_map_20260911_edited.yaml"

MAP="$MAP_DEFAULT"
LIDAR_YAW=0
OP_TIMEOUT=2.0
USE_LOCALIZER=0
SKIP_RECORD=0
DRY=0
DO_ENABLE=0
FORCE_POSTURE=0
PATROL=""

while [ $# -gt 0 ]; do
    case "$1" in
        --map)              MAP="$2"; shift 2 ;;
        --lidar-yaw)        LIDAR_YAW="$2"; shift 2 ;;
        --operator-timeout) OP_TIMEOUT="$2"; shift 2 ;;
        --localizer)        USE_LOCALIZER=1; shift ;;
        --skip-record)      SKIP_RECORD=1; shift ;;
        --dry-run)          DRY=1; shift ;;
        --force-posture)    FORCE_POSTURE=1; shift ;;
        --patrol)           PATROL="$2"; shift 2 ;;
        --enable)           DO_ENABLE=1; shift ;;
        -h|--help)          sed -n '2,45p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知の引数: $1" >&2; exit 2 ;;
    esac
done

# --- 表示 -------------------------------------------------------------------
c_ok=$'\e[32m'; c_ng=$'\e[31m'; c_w=$'\e[33m'; c_b=$'\e[1m'; c_0=$'\e[0m'
step() { echo; echo "${c_b}=== $* ===${c_0}"; }
ok()   { echo "  ${c_ok}✅${c_0} $*"; }
warn() { echo "  ${c_w}⚠️${c_0}  $*"; }
die()  { echo "  ${c_ng}❌ $*${c_0}" >&2; echo; echo "${c_ng}ここで止めた。上の理由を潰してからやり直すこと。${c_0}" >&2; exit 1; }
run()  { if [ "$DRY" = 1 ]; then echo "  [dry] $*"; else eval "$@"; fi; }

# ROS 環境を使うコマンドはすべてこれ経由。**素の shell には ROS を入れない**(D-07)
# ⚠️ `--manifest-path` ではなく **cd してから run** する。2026-09-15 に実機で
# 通したのがこの形で、未検証の書き方に変えて現地で詰まるのを避けるため。
ros() {
    ( cd "$PIXI_DIR" && "$PIXI" run bash -lc "
        source '$WS/install/setup.bash'
        export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI=file://$CYCLONE_CFG
        $*" )
}

# --- --enable: 走行許可だけを出すモード -------------------------------------
# ⚠️ 発進ゲート(--arm)は開けない。**ゲートを開けるのは人の手**(D-07)。
# ここが許可するのは「Nav2 の指令を SDK 側へ流してよい」という ROS 側の状態遷移だけ。
if [ "$DO_ENABLE" = 1 ]; then
    ARM_NOW="$(pgrep -af 'g1_sdk_bridge_real_serve[r]' | head -1)"
    case "$ARM_NOW" in
        *--arm*) echo "  発進ゲート: 開" ;;
        "")      echo "  ❌ ブリッジが動いていない" >&2; exit 1 ;;
        *)       echo "  ⚠️  発進ゲートが閉じている。許可しても機体は動かない（配線確認には有用）" ;;
    esac
    ros "ros2 service call /g1/enable_navigation std_srvs/srv/SetBool '{data: true}' 2>&1 | tail -2"
    ros "timeout 12 ros2 topic echo /g1/bridge_status --once 2>&1 | sed -n '9,12p'"
    exit 0
fi

echo "${c_b}G1 Nav2 立ち上げ${c_0}  地図=$(basename "$MAP")  lidar_yaw=$LIDAR_YAW  operator_timeout=$OP_TIMEOUT"
[ "$USE_LOCALIZER" = 1 ] && echo "  自己位置: 連続(map_localizer)" || echo "  自己位置: 静的(find_map_offset を1回)"
[ "$DRY" = 1 ] && warn "dry-run。実際には何もしない"

# --- 0. 前提の確認 ----------------------------------------------------------
step "0. 前提の確認"
[ -n "${ROS_DISTRO:-}" ] && die "この端末で ROS が source されている(ROS_DISTRO=$ROS_DISTRO)。ログイン時のプロンプトでは **Enter だけ**を押すこと(D-06/D-07)"
ok "ROS 環境は汚染されていない"
[ -x "$PIXI" ] || die "pixi が無い: $PIXI"
[ -f "$PIXI_DIR/pixi.toml" ] || die "pixi の環境定義が無い: $PIXI_DIR/pixi.toml"
[ -f "$MAP" ] || die "地図が無い: $MAP"
[ -f "$CYCLONE_CFG" ] || die "CycloneDDS 設定が無い: $CYCLONE_CFG"
ok "pixi 環境・地図・DDS 設定がある"
if [ -n "$PATROL" ]; then
    [ -f "$PATROL" ] || die "巡回路が無い: $PATROL（tools/record_waypoints.py で現地で作ること）"
    N=$(grep -c '^  *- *{' "$PATROL" 2>/dev/null || echo 0)
    [ "$N" -gt 0 ] || die "巡回路にウェイポイントが1点も無い: $PATROL"
    ok "巡回路 $N 点: $(basename "$PATROL")"
fi

# --- 掃除: 前回の残骸を消す（二重起動を防ぐ）--------------------------------
step "0.5 前回の残骸を掃除する"
# ⚠️ 2026-09-15 に systemd 版と手起動版の**ブリッジが2本**立って事故った。
# arm する前に1本であることを保証したいので、ROS 側は毎回まっさらにする。
for pat in "g1_slam_odom_t[f].py" "g1_state_bridge_nod[e]" "g1_cmd_router_nod[e]" \
           "envs/default/lib/nav[2]_" "map_localize[r].py"; do
    run "pkill -f '$pat' 2>/dev/null || true"
done
run "sleep 3"
ok "ROS 側のプロセスを掃除した"

# --- 1. 機体の姿勢 ----------------------------------------------------------
# ⚠️⚠️ **ここが 2026-09-15 に最も時間を溶かした落とし穴。**
# §5 の自動校正は**その瞬間の姿勢を重力整列として焼き込む**。座位や傾いた立位で
# 校正すると、歩き出して姿勢が変わった分だけ点群が傾き、地図と噛み合わなくなる。
# 実測: 通常の立位 1.59〜4.35°(基準 3.81°) / 座位 15.67° / 傾いた立位 19.04°
step "1. 機体の姿勢"
if [ "$DRY" = 0 ]; then
    TILT=$(ros "timeout 30 python3 '$TOOLS/check_imu_attitude.py' 2>&1 | grep -o '傾き = [0-9.]*' | grep -o '[0-9.]*'")
    if [ -z "$TILT" ]; then
        warn "傾きを測れなかった（IMU が届いていない）。姿勢の検査を飛ばす"
    else
        BAD=$(python3 -c "print(1 if float('$TILT') > 10 else 0)")
        MEH=$(python3 -c "print(1 if float('$TILT') > 6 else 0)")
        if [ "$BAD" = 1 ] && [ "$FORCE_POSTURE" = 0 ]; then
            warn "センサーの傾き ${TILT}°（通常の立位は 2〜4°）"
            warn "**座位か、傾いた立位の可能性が高い。** この姿勢で校正すると、"
            warn "歩き出したときに点群が地図と噛み合わなくなる。"
            echo "      → 純正リモコンで**通常の立位**（歩かせるときと同じ姿勢）にしてから再実行する"
            echo "      → 意図的に進めるなら --force-posture"
            die "姿勢が立位でない（傾き ${TILT}°）"
        elif [ "$MEH" = 1 ]; then
            warn "センサーの傾き ${TILT}°。やや大きい（通常は 2〜4°）。RViz での目視確認を特に丁寧に"
        else
            ok "センサーの傾き ${TILT}°（通常の立位の範囲）"
        fi
    fi
fi
warn "⚠️ **ここから先、機体を動かさないこと。** §5 の校正と §7 の照合は、いまの姿勢と位置で決まる"

# --- 2. SDK側プロセス（発進ゲートは閉じたまま）------------------------------
step "2. SDK側プロセス（発進ゲートは閉じたまま）"
ARM_LINE="$(grep '^G1_ARM=' /etc/default/g1-sdk-bridge 2>/dev/null || echo 'G1_ARM=?')"
if [ "$ARM_LINE" != "G1_ARM=" ]; then
    warn "$ARM_LINE  ← **発進ゲートが閉じていない**"
    warn "このまま起動すると、準備中ずっと機体が動ける状態になる。閉じてから続けること:"
    echo "      sudo sed -i 's/^G1_ARM=.*/G1_ARM=/' /etc/default/g1-sdk-bridge"
    die "発進ゲートが閉じていない"
fi
ok "発進ゲートは閉じている (G1_ARM=)"

if systemctl is-active --quiet g1-sdk-bridge; then
    ok "g1-sdk-bridge は既に動いている"
else
    run "sudo -n systemctl start g1-sdk-bridge" \
        || die "起動できなかった。NOPASSWD が入っていないなら手で: sudo systemctl start g1-sdk-bridge"
    run "sleep 3"
fi
N_BRIDGE=$(pgrep -cf "g1_sdk_bridge_real_serve[r]" || true)
[ "$DRY" = 1 ] || [ "$N_BRIDGE" = 1 ] || die "ブリッジが ${N_BRIDGE} 本ある。**1本でないと arm できない**。pgrep -af g1_sdk_bridge_real_server で確認して余分を落とすこと"
[ "$DRY" = 1 ] || ok "ブリッジは1本だけ (--arm 無し)"
[ "$DRY" = 1 ] || { [ -S /run/g1_bridge/cmd.sock ] && [ -S /run/g1_bridge/state.sock ]; } \
    || die "/run/g1_bridge/{cmd,state}.sock が無い"
ok "ソケットがある (/run/g1_bridge/)"

# --- 3. 内蔵SLAM ------------------------------------------------------------
step "3. 内蔵SLAM（1801）"
# ⚠️ U-17: 約16分で勝手に止まる。**既に動いているなら再送しない**。
# 再送すると odom 原点がリセットされ §7 が無効になるため。
# 加えて「時間で切れるのか無動作で切れるのか」を判別するため、
# **1セッション中は1回だけ送る**のが望ましい(HANDOVER §2-③)。
SLAM_ALIVE=0
if [ "$DRY" = 0 ]; then
    ros "timeout 12 ros2 topic echo /unitree/slam_mapping/odom --once >/dev/null 2>&1" && SLAM_ALIVE=1
fi
if [ "$SLAM_ALIVE" = 1 ]; then
    ok "内蔵SLAM は既に動いている（**再送しない**。odom 原点を保つため）"
else
    run "cd '$TOOLS' && /usr/bin/python3 send_slam_api.py 1801" || die "1801 の送信に失敗した"
    run "sleep 4"
fi

if [ "$DRY" = 0 ]; then
    HZ=$(ros "timeout 18 ros2 topic hz /unitree/slam_mapping/odom 2>&1 | grep -m1 'average rate' | awk '{print \$3}'")
    [ -n "$HZ" ] || die "/unitree/slam_mapping/odom が流れていない。1801 が効いていない"
    ok "odom が $HZ Hz で流れている（判定: 約9〜10Hz）"
fi

# 生存監視（16分で落ちたときに気づけるように）
if ! pgrep -f "watch_slam_aliv[e]" >/dev/null 2>&1; then
    run "setsid nohup bash '$TOOLS/watch_slam_alive.sh' >/dev/null 2>&1 < /dev/null &"
    ok "生存監視を開始した（/tmp/slam_watch.log）"
else
    ok "生存監視は既に動いている"
fi

# --- 4. 記録 ----------------------------------------------------------------
step "4. 記録（Nav2 より先に）"
if [ "$SKIP_RECORD" = 1 ]; then
    warn "--skip-record が指定された。**何かあっても原因を追えない**"
else
    run "bash '$HERE/start_record.sh'" || die "記録を開始できなかった"
    run "sleep 8"
    if [ "$DRY" = 0 ]; then
        grep -q "トピック数=19" /tmp/record.log 2>/dev/null \
            && ok "記録中（プロファイル=diag / トピック数=19）" \
            || warn "トピック数が 19 でない。/tmp/record.log を確認すること"
    fi
fi

# --- 5+7. Nav2 と自己位置合わせ ---------------------------------------------
launch_nav() {   # $1 = map_to_odom
    run "bash '$HERE/start_nav.sh' '$1' $LIDAR_YAW $OP_TIMEOUT '$PATROL' '$MAP'" || die "launch できなかった"
    [ "$DRY" = 1 ] && return 0
    local i
    for i in $(seq 1 40); do
        grep -aq "自動校正した" /tmp/nav_launch.log 2>/dev/null && break
        sleep 2
    done
    grep -aq "自動校正した" /tmp/nav_launch.log 2>/dev/null \
        || die "自動校正が終わらない。IMU か内蔵SLAM の odom が届いていない可能性が高い（/tmp/nav_launch.log）"
    grep -a "自動校正した" /tmp/nav_launch.log | tail -1 | sed 's/^/  /'
    # 焼き込まれた値そのものを検査する（§1 の事前チェックの裏取り）
    local raw
    raw=$(grep -a "自動校正した" /tmp/nav_launch.log | tail -1 | grep -o '生センサーの傾き [0-9.]*' | grep -o '[0-9.]*')
    if [ -n "$raw" ] && [ "$FORCE_POSTURE" = 0 ]; then
        if [ "$(python3 -c "print(1 if float('$raw') > 10 else 0)")" = 1 ]; then
            die "校正に焼き込まれた傾きが ${raw}° と大きすぎる。立位にしてからやり直すこと"
        fi
    fi
}

step "5. Nav2 を起動する（機体は静止させたまま）"
launch_nav "0 0 0"
ok "Nav2 が起動し、自動校正が終わった"

step "7. 自己位置を地図に合わせる"
if [ "$DRY" = 0 ]; then
    # ⚠️ --yaw-range 180 は必須。既定の ±20° は偽のピークを掴む(2026-09-15 実測)
    OUT=$(ros "timeout 240 python3 '$TOOLS/find_map_offset.py' --yaw-range 180 --yaw-step 1 2>&1")
    echo "$OUT" | grep -E "最良|20cm以内" | sed 's/^/  /'
    BEST=$(echo "$OUT" | grep -m1 '最良' || true)
    [ -n "$BEST" ] || die "地図照合が結果を返さなかった"
    DX=$(echo "$BEST" | sed -n 's/.*dx=\([-0-9.]*\).*/\1/p')
    DY=$(echo "$BEST" | sed -n 's/.*dy=\([-0-9.]*\).*/\1/p')
    YAWDEG=$(echo "$BEST" | sed -n 's/.*yaw=\([-0-9.]*\).*/\1/p')
    P20=$(echo "$OUT" | grep -m1 '20cm以内' | sed -n 's/.*20cm以内=\([0-9]*\)%.*/\1/p')
    P50=$(echo "$OUT" | grep -m1 '50cm以内' | sed -n 's/.*50cm以内=\([0-9]*\)%.*/\1/p')
    YAWRAD=$(python3 -c "import math;print(f'{math.radians($YAWDEG):.4f}')")

    # ⚠️ 2026-09-15 の実測: 部屋が地図と変わっていると 20cm以内 は 40〜45% まで落ちるが、
    # **自己位置自体は正しい**ことが目視で確認できた。**50cm以内 のほうが素性がよい。**
    if [ "${P50:-0}" -lt 80 ]; then
        warn "50cm以内 が ${P50}% しかない。地図と部屋が食い違っている可能性が高い"
        warn "**RViz での目視確認を必ず行うこと**（位置と向きの両方）"
    else
        ok "50cm以内 ${P50}% / 20cm以内 ${P20}%"
    fi
else
    DX=0; DY=0; YAWRAD=0; YAWDEG=0
fi

if [ "$USE_LOCALIZER" = 1 ]; then
    step "7b. 連続 localization を使う"
    launch_nav "none"
    # ⚠️ **引数で渡すこと。** 2026-09-16 まで引数なしで呼んでおり、
    # start_localizer.sh 内の 2026-09-15 のハードコード値が使われていた。
    # それでいて下の ok() は $DX $DY $YAWRAD と表示するので、**ログが嘘をついていた**。
    run "bash '$HERE/start_localizer.sh' $DX $DY $YAWRAD" || die "map_localizer を起動できなかった"
    run "sleep 6"
    ok "map_localizer を起動した（初期値 $DX $DY $YAWRAD）"
else
    step "7b. 求めた値を適用して上げ直す"
    launch_nav "$DX $DY $YAWRAD"
    ok "map→odom = $DX $DY $YAWRAD（yaw ${YAWDEG}°）を適用した"
fi

# --- 検収 -------------------------------------------------------------------
step "検収"
if [ "$DRY" = 0 ]; then
    BAD=0
    for n in map_server planner_server controller_server behavior_server bt_navigator velocity_smoother; do
        S=$(ros "timeout 10 ros2 lifecycle get /$n 2>&1 | head -1")
        case "$S" in *"active"*) ;; *) echo "  ${c_ng}$n: $S${c_0}"; BAD=1 ;; esac
    done
    [ "$BAD" = 0 ] && ok "Nav2 6ノードすべて active" || die "active でないノードがある"

    ST=$(ros "timeout 12 ros2 topic echo /g1/bridge_status --once 2>&1")
    echo "$ST" | grep -A1 -E "message:|key: (tf|sensor|operator_heartbeat)$" | grep -E "message:|value:" | sed 's/^/  /'
    echo "$ST" | grep -q "DISCONNECTED" && die "cmd_router がブリッジに繋がっていない。bridge_sock_dir と /run/g1_bridge を確認"

    POSE=$(ros "timeout 12 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -E 'Translation|degree' | head -2")
    echo "$POSE" | sed 's/^/  /'

    CM=$(ros "timeout 60 python3 '$TOOLS/count_costmap.py' 2>&1 | tail -2")
    echo "$CM" | sed 's/^/  /'
    echo "$CM" | grep -q "経路計画できる" || warn "**footprint の内側に障害物がある。このままでは経路が出ない。** 機体の周囲を空けること（人は2m以上離れる）"
fi

# --- 次にやること -----------------------------------------------------------
cat <<EOS

${c_b}=== ここまでで機体は動かない。ここから先は人の判断 ===${c_0}

${c_b}① 操作PC で（別の端末）${c_0}
   g1_heartbeat_sender --host 192.168.123.164 &
   cd <repo>/Navigation/nav2_option/tools && G1_RMW=rmw_cyclonedds_cpp ./rviz_operate.sh

${c_b}② RViz で目視確認（省かない）${c_0}
   ・機体の位置が実際の立ち位置と合っているか
   ・${c_b}赤軸（前方）が実機の正面と同じ向きか${c_0}  ← 逆なら --lidar-yaw を 0↔180 で入れ替えて再実行
   ・マゼンタ（LocalCostmap）が機体の周りを囲んでいないか

${c_b}③ 安全確認（声に出して）${c_0}
   ・停止係が純正リモコンを持ち、機体を見ている
   ・前方 2m 以上空いている（指令を止めても約2秒動き続ける）
   ・${c_b}全員が機体から 2m 以上離れている（後ろも含む）${c_0}

${c_b}④ 発進ゲートを開く（ここは意図的に自動化していない）${c_0}
   sudo sed -i 's/^G1_ARM=.*/G1_ARM=--arm/' /etc/default/g1-sdk-bridge
   sudo systemctl restart g1-sdk-bridge
   pgrep -af g1_sdk_bridge_real_server      # → 末尾に --arm があること

${c_b}⑤ 走行を許可する${c_0}
   $0 --enable   （または手で ros2 service call /g1/enable_navigation ...）
   → bridge_status が ${c_b}NAVIGATING${c_0} になること（READY のままなら許可できていない）

${c_b}⑥ 走らせる。2つのモードは再起動なしで切り替わる${c_0}
   ${c_b}単純ゴール指定${c_0}: RViz の「2D Goal Pose」で 1〜2m 先を指す
   ${c_b}巡回${c_0}          : $TOOLS/patrol_ctl.sh start
                    $TOOLS/patrol_ctl.sh watch     # 状態を見る
                    $TOOLS/patrol_ctl.sh pause     # 止める（index は保つ）
   ⚠️ 巡回中に RViz から Goal を送ると${c_b}巡回のほうが退く${c_0}。戻すときは再度 start
   ⚠️ 内蔵SLAM が16分で落ちると FAULT → 巡回は HOLD する。${c_b}自動では戻らない${c_0}

${c_b}撤収${c_0}
   sudo sed -i 's/^G1_ARM=.*/G1_ARM=/' /etc/default/g1-sdk-bridge && sudo systemctl restart g1-sdk-bridge
   pkill -f 'ros2 bag recor[d]' ; cd $TOOLS && python3 send_slam_api.py 1901
EOS
