#!/usr/bin/env bash
# Mac から実機までの経路を、**切れる順番どおりに**上から確かめる。
#
#   bash quickstart/check_link.sh          # 実機まで全部
#   bash quickstart/check_link.sh offline  # 実機を飛ばし、コンテナと DDS の中だけ
#
# ## なぜ要るのか
#
# 2026-09-07 の 1 セッションで**疎通の切り分けを 3 回間違えた**。
#
# | 誤診 | 実際 |
# |---|---|
# | 「VM から実機に到達しない」 | **VM に `ping` が入っていない** |
# | 「コンテナから実機に到達しない」 | **コンテナにも `ping` が入っていない**（TCP で測ったら到達していた） |
# | 「有線 LAN は繋がっている」 | **Mac の `en8` が OS から消えていた**（USB アダプタが抜けていた） |
#
# `ping` は colima の VM にもコンテナにも**入っていない**。
# `ping ... && echo OK || echo NG` と書くと、
# 「到達しない」と「コマンドが無い」が**同じ NG になって区別できない**。
# ここでは **TCP の接続可否**（bash 組み込みの `/dev/tcp`）で測る。
#
# ## 経路
#
#   Mac (en8 / 192.168.123.202)
#     └─ colima VM (col0 / 192.168.123.201)   ← ケーブル抜き差しでここが切れる
#          └─ コンテナ rviz (--network host なので VM と同じ netns)
#               └─ G1 の内蔵スイッチ (L2)
#                    ├─ PC1 192.168.123.161  ポート全閉。LiDAR と SLAM はここが出す
#                    └─ PC2 192.168.123.164  ssh で入れる。loco_driver はここで動かす
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
. "$HERE/_common.sh"
G1_TAG=link

MODE="live"
[ "${1:-}" = "offline" ] && MODE="offline"

FAIL=0
step() { echo; echo "── $*"; }
note() { echo "     → $*"; }

echo "=============================================================="
echo " 経路の確認   mode=$MODE"
echo "=============================================================="

# ── 1. Mac の有線 NIC ────────────────────────────────────────────
if [ "$MODE" = "live" ]; then
    step "1. Mac の有線 NIC（192.168.123.0/24 を持っているか）"
    NIC="$(g1_wired_nic)"
    if [ -n "$NIC" ]; then
        g1_ok "$NIC = $(ifconfig "$NIC" 2>/dev/null | awk '/inet 192\.168\.123\./{print $2}')"
        [ "$NIC" != "$G1_MAC_IFACE" ] && note "既定の $G1_MAC_IFACE ではなく ${NIC}。G1_MAC_IFACE で上書きできる"
    else
        g1_bad "192.168.123.0/24 を持つ NIC が無い"
        note "USB-Ethernet アダプタが Mac 側で抜けている。挿し直す"
        note "挿し直したら bash quickstart/start_rviz_mac.sh live（ブリッジの張り直しは自分でやる）"
        FAIL=1
    fi
else
    step "1. Mac の有線 NIC — offline なので飛ばす"
fi

# ── 2. Mac → colima VM ──────────────────────────────────────────
step "2. Mac → colima VM（${G1_VM_IP}）"
if ! command -v colima >/dev/null 2>&1; then
    g1_bad "colima が無い"
    FAIL=1
elif ! docker ps >/dev/null 2>&1; then
    g1_bad "docker に繋がらない（VM が起きていない）"
    note "colima start（または colima restart）"
    FAIL=1
else
    g1_ok "docker は応答する"
    # ブリッジ NIC の IPv4。colima を作り直すと消えるので start_rviz_mac.sh が付け直す
    VMADDR="$(colima ssh -- ip -4 -brief addr show "$G1_VM_NIC" 2>/dev/null | awk '{print $3}')"
    if [ -n "$VMADDR" ]; then
        g1_ok "VM の $G1_VM_NIC = $VMADDR"
    elif [ "$MODE" = "live" ]; then
        g1_bad "VM の $G1_VM_NIC に IPv4 が無い"
        note "colima restart の後は必ずこうなる。bash quickstart/start_rviz_mac.sh live が付け直す"
        FAIL=1
    else
        g1_warn "VM の $G1_VM_NIC に IPv4 が無い（offline なら要らない）"
    fi
    if [ "$MODE" = "live" ]; then
        if g1_tcp_ok "$G1_VM_IP" 80 3; then
            g1_ok "Mac → $G1_VM_IP:80（RViz2 の Web UI）"
            note "ブラウザ: http://$G1_VM_IP/  VNC パスワード ubuntu"
        else
            g1_warn "Mac → $G1_VM_IP:80 が閉じている（コンテナか RViz2 が動いていないだけかもしれない）"
        fi
    fi
fi

# ── 3. コンテナ ─────────────────────────────────────────────────
step "3. コンテナ $G1_RVIZ_NAME"
if ! docker inspect "$G1_RVIZ_NAME" >/dev/null 2>&1; then
    g1_bad "コンテナ $G1_RVIZ_NAME が無い"
    note "bash quickstart/start_rviz_mac.sh $MODE で作る"
    FAIL=1
else
    STATE="$(docker inspect -f '{{.State.Status}}' "$G1_RVIZ_NAME" 2>/dev/null)"
    if [ "$STATE" = "running" ]; then
        g1_ok "起動している"
    else
        g1_bad "状態 = $STATE"
        note "docker start ${G1_RVIZ_NAME}、または start_rviz_mac.sh $MODE"
        FAIL=1
    fi
fi

# ── 4. コンテナに要るパッケージ ─────────────────────────────────
if docker inspect -f '{{.State.Running}}' "$G1_RVIZ_NAME" 2>/dev/null | grep -q true; then
    step "4. コンテナに入っているもの（手で apt した分は docker rm で消える）"
    check_path() {  # 表示名 パス 直し方
        if docker exec "$G1_RVIZ_NAME" test -e "$2" 2>/dev/null; then
            g1_ok "$1"
        else
            g1_bad "$1 が無い"
            note "$3"
            FAIL=1
        fi
    }
    check_path "rmw_cyclonedds_cpp" /opt/ros/humble/lib/librmw_cyclonedds_cpp.so \
        "start_rviz_mac.sh が入れる"
    check_path "mola-lidar-odometry-cli" /opt/ros/humble/bin/mola-lidar-odometry-cli \
        "start_rviz_mac.sh が入れる（G1_SKIP_MOLA=1 で飛ばしていないか）"
    check_path "libmola_metric_maps.so（MOLA のプラグイン）" \
        /opt/ros/humble/lib/aarch64-linux-gnu/libmola_metric_maps.so \
        "無いと MOLA-LO が最初の 1 スキャンで fatal になり 0 キーフレームで終わる"
    # 無くても NG にしない。2026-09-11 から octomap_server は opt-in（G1_OCTOMAP=1）で、
    # 既定の live では起こさない。Nav2 は /projected_map を一切参照しない
    check_path_opt() {  # 表示名 パス 補足
        if docker exec "$G1_RVIZ_NAME" test -e "$2" 2>/dev/null; then
            g1_ok "$1"
        else
            g1_warn "$1 が無い（$3）"
        fi
    }
    check_path_opt "octomap_server" /opt/ros/humble/lib/octomap_server/octomap_server_node \
        "既定 off なので支障は無い。G1_OCTOMAP=1 で出すときだけ要る"
    check_path "nav2 planner_server" /opt/ros/humble/lib/nav2_planner/planner_server \
        "start_rviz_mac.sh が入れる（G1_SKIP_NAV2=1 で飛ばしていないか）"
    check_path "/work のマウント" /work/G1_Hackason/Mapping/real/quickstart \
        "コンテナを作り直す（-v \$REPO_ROOT:/work が要る）"
fi

# ── 5. 実機（TCP で測る。ping は使わない）────────────────────────
if [ "$MODE" = "live" ]; then
    step "5. 実機（TCP。⚠️ ping は VM にもコンテナにも入っていないので使わない）"
    if g1_tcp_ok "$G1_PC2_IP" "$G1_PC2_SSH_PORT" 3; then
        g1_ok "Mac → PC2 $G1_PC2_IP:$G1_PC2_SSH_PORT"
    else
        g1_bad "Mac → PC2 $G1_PC2_IP:$G1_PC2_SSH_PORT に届かない"
        note "実機の電源、または有線の経路（手順 1）"
        FAIL=1
    fi
    # コンテナ側からも測る。ここだけ切れていると DDS が通らない
    if docker inspect -f '{{.State.Running}}' "$G1_RVIZ_NAME" 2>/dev/null | grep -q true; then
        # 判定は _common.sh の g1_tcp_ok_container に置いてある。
        # start_rviz_mac.sh の自動修復（G1_COLIMA_RESTART）と**同じものを使う**
        if g1_tcp_ok_container "$G1_PC2_IP" "$G1_PC2_SSH_PORT"; then
            g1_ok "コンテナ → PC2（DDS が載る経路）"
        else
            g1_bad "コンテナ → PC2 に届かない"
            note "VM の $G1_VM_NIC に IPv4 が無い（手順 2）か、ブリッジが切れている"
            note "bash quickstart/start_rviz_mac.sh live が自分で張り直す（手作業の colima restart は要らない）"
            FAIL=1
        fi
    fi
    # PC1 はポート全閉なので TCP では測れない。ARP が引けるかで見る
    if docker inspect -f '{{.State.Running}}' "$G1_RVIZ_NAME" 2>/dev/null | grep -q true; then
        if docker exec "$G1_RVIZ_NAME" bash -c \
            "ip neigh show $G1_PC1_IP 2>/dev/null | grep -qv FAILED"; then
            g1_ok "PC1 $G1_PC1_IP の ARP が引けている"
        else
            g1_warn "PC1 $G1_PC1_IP の ARP が無い（まだ通信していないだけのこともある）"
            note "PC1 はポート全閉なので TCP では測れない。次の手順 6 で判断する"
        fi
    fi
fi

# ── 6. DDS ──────────────────────────────────────────────────────
if docker inspect -f '{{.State.Running}}' "$G1_RVIZ_NAME" 2>/dev/null | grep -q true; then
    step "6. DDS（mode=$MODE の URI で見る）"
    # ⚠️ --no-daemon が要る。ros2 daemon は**先に起動したときの DDS 設定で作った
    # グラフをキャッシュ**しているので、CYCLONEDDS_URI を変えても古い答えを返す。
    # 2026-09-07 に offline で実機のトピックが 184 件見えて気づいた
    # （diagnose_ros2.sh の冒頭に書いてある罠と同じもの）。
    TOPICS="$(g1_exec "$MODE" "timeout 15 ros2 topic list --no-daemon 2>/dev/null" || true)"
    N="$(printf '%s\n' "$TOPICS" | grep -c . || true)"
    if [ "${N:-0}" -gt 0 ]; then
        g1_ok "トピックが $N 件見える"
        for t in /utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360 \
                 /unitree/slam_mapping/odom /unitree/slam_mapping/points; do
            if printf '%s\n' "$TOPICS" | grep -qx "$t"; then
                g1_ok "  $t"
            else
                g1_warn "  $t が見えない"
            fi
        done
    else
        g1_bad "トピックが 1 件も見えない"
        if [ "$MODE" = "live" ]; then
            note "手順 5 が OK なのにここが駄目なら、DDS が別の NIC に載っている"
            note "CYCLONEDDS_URI の NetworkInterface が $G1_VM_NIC を指しているか"
        else
            note "offline では自分で publish しない限り何も出ない（bag を再生する）"
        fi
        FAIL=1
    fi

    # レートは publisher が居るときだけ意味がある
    if [ "$MODE" = "live" ] && printf '%s\n' "$TOPICS" | grep -qx /utlidar/cloud_livox_mid360; then
        step "7. レート（実測値と比べる）"
        for spec in "/utlidar/cloud_livox_mid360 9.95" "/utlidar/imu_livox_mid360 200.1"; do
            set -- $spec
            # ⚠️ hz は --no-daemon を受け付けない（list/info/echo だけの verb 引数）。
            # 付けると argparse が rc=2 で落ち、常に「取れない」になる（2026-09-09 に踏んだ）。
            # hz は自分で subscription を張るので daemon のグラフには依存しない。
            # SIGTERM だと集計を印字せずに死ぬので -s INT を渡す。
            R="$(g1_exec "$MODE" "stdbuf -oL timeout -s INT 12 ros2 topic hz $1 2>/dev/null" \
                 | sed -n 's/.*average rate: \([0-9.]*\).*/\1/p' | head -1 || true)"
            if [ -n "$R" ]; then
                g1_ok "$1 = $R Hz（2026-09-07 の実測 $2 Hz）"
            else
                g1_warn "$1 のレートが取れない"
            fi
        done
    fi
fi

# ── 8. 記録の定義（機体もコンテナも要らない。offline でも見る）─────────
step "8. 記録の定義 record_topics.txt（直書きが復活していないか）"
if [ ! -f "$HERE/record_topics.txt" ]; then
    g1_bad "$HERE/record_topics.txt が無い"
    note "記録するトピックの唯一の定義。無いと run_stage.sh が走り出す前に落ちる"
    FAIL=1
else
    # (a) 直書きの禁止。**トピック名を並べた `ros2 bag record` を探す。**
    #     変数で渡している行（`"${TOPICS[@]}"` など）は当たらない＝それが正しい形。
    #     ⚠️ 自分自身は外す（この検査の**パターン文字列が自分に当たる**。
    #        2026-09-10 の pgrep 自己マッチと同じ型）。
    #     `../docker/record-topics.sh` は別系統の古い実装で、この系では使わないので範囲外。
    HARD="$(grep -rn -- 'ros2 bag record' "$HERE" 2>/dev/null \
            | grep -v "$HERE/check_link.sh:" \
            | grep -v "$HERE/record_topics.txt:" \
            | grep -E '/utlidar/|/unitree/|/tf' || true)"
    if [ -z "$HARD" ]; then
        g1_ok "quickstart/ に記録トピックの直書きは無い"
    else
        g1_bad "記録トピックを直書きしている箇所がある"
        printf '%s\n' "$HARD" | sed 's/^/       /'
        note "record_topics.txt を読む形に直す:"
        note "  TOPICS=\"\$(awk '/^[[:space:]]*#/{next} \$2==\"sensor\"||\$2==\"ros\"{printf \"%s \", \$1}' quickstart/record_topics.txt)\""
        FAIL=1
    fi

    # (b) sensor 行は record_dds_to_bag.py の KNOWN_TOPICS に型が要る。
    #     片方だけ足すと PC2 の記録が「未知のトピックです」で落ちる。
    MISSING=""
    for T in $(awk '/^[[:space:]]*#/{next} $2=="sensor"{print $1}' "$HERE/record_topics.txt"); do
        [ -n "$(grep -F -- "\"$T\"" "$HERE/record_dds_to_bag.py" 2>/dev/null || true)" ] \
            || MISSING="$MISSING $T"
    done
    if [ -z "$MISSING" ]; then
        g1_ok "sensor 行は全部 record_dds_to_bag.py の KNOWN_TOPICS にある"
    else
        g1_bad "KNOWN_TOPICS に型が無い sensor 行:$MISSING"
        note "record_dds_to_bag.py の KNOWN_TOPICS に \"<トピック>\": \"<型>\" を足す"
        FAIL=1
    fi
fi

echo
echo "=============================================================="
if [ "$FAIL" = "0" ]; then
    echo " 判定: 経路は通っている"
else
    echo " 判定: **上の NG を上から順に潰すこと**（下流だけ直しても通らない）"
fi
echo "=============================================================="
exit "$FAIL"
