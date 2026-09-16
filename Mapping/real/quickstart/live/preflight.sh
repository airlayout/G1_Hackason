#!/usr/bin/env bash
# **起こす前に、切れる順番どおりに上から確かめる。** 何も起動しない・何も変えない。
#
#   bash quickstart/live/preflight.sh
#
# ## なぜ要るのか
#
# 2026-09-16 の 1 セッションで、**起動してから気づいた不具合が 3 つ**あった。
# どれも起動前に 10 秒で分かるものだった:
#
# | 症状 | 実際の原因 | ここで捕まえる |
# |---|---|---|
# | 測位が `Waiting for Odometry_loc` から進まない | **LiDAR の IMU が配信されていない**（起動ごとに出ないことがある）| 6. |
# | Nav2 が計算しているのに足が動かない | `cmd_vel_bridge` が **wlan0 を掴んでいた** | 起動時に固定（up.sh）|
# | コンテナから機体が見えない | Mac の `bridge101` に `en8` が入っていない | 2. |
#
# ## 終了コード
#   0  全部通った。up.sh に進んでよい
#   1  致命的。直さないと進めない（理由と手当てを出す）
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/_common.sh"

FAIL=0
step() { printf '\n[live] %s\n' "$*" >&2; }

# ── 1. Mac の有線 ──────────────────────────────────────────────────
step "1. Mac の有線（en8）"
if ifconfig en8 >/dev/null 2>&1; then
    ok "en8 = $(ifconfig en8 | awk '/inet /{print $2}')"
else
    bad "en8 が無い。**USB-Ethernet アダプタが抜けている**"
    echo "     → 挿してから もう一度" >&2
    exit 1
fi

# ── 2. colima のブリッジ（ここが切れるとコンテナだけ孤立する）──────
step "2. colima の L2（bridge101 に en8 が入っているか）"
if ifconfig bridge101 2>/dev/null | grep -q 'member: en8'; then
    ok "bridge101 に en8 が入っている"
else
    bad "bridge101 に en8 が無い。コンテナが物理網に出られない"
    echo "     → bash quickstart/repair_bridge.sh（sudo を聞かれる）" >&2
    echo "     ⚠️ repair_bridge.sh の合否判定は 192.168.123.164 を見るので、" >&2
    echo "        AP 経由の構成では「戻らなかった」と出るが addm 自体は効いている" >&2
    FAIL=1
fi

# ── 3. AP（OMEN）────────────────────────────────────────────────────
step "3. AP を出している Ubuntu（$G1_AP_HOST）"
if tcp_ok "$G1_AP_HOST" 22; then
    ok "$G1_AP_HOST:22 に届く"
    APDEV="$(ap 'nmcli -t -f DEVICE,STATE,CONNECTION device status | grep ^wlp' 2>/dev/null || true)"
    case "$APDEV" in
        *g1-teleop-ap*) ok "AP が上がっている（${APDEV%%:*}）" ;;
        "")             warn "AP の状態が読めなかった" ;;
        *)              bad "AP が上がっていない（いま: ${APDEV##*:}）"
                        echo "     → ssh $G1_AP_USER@$G1_AP_HOST 'nmcli connection up g1-teleop-ap'" >&2
                        echo "     ⚠️ 無線 1 枚で AP と STA は同居できない。社内 Wi-Fi は切れる" >&2
                        FAIL=1 ;;
    esac
    NAT="$(ap 'grep -c "nat POSTROUTING" /etc/NetworkManager/dispatcher.d/50-g1teleop-forward 2>/dev/null' 2>/dev/null || echo 0)"
    [ "${NAT:-0}" -gt 0 ] && ok "dispatcher に NAT が入っている（$NAT 件）" \
                          || { bad "AP の filter/NAT が無い"; FAIL=1; }
else
    bad "$G1_AP_HOST に届かない。ケーブルの行き先を確認する"
    FAIL=1
fi

# ── 4. PC2（無線）──────────────────────────────────────────────────
step "4. PC2（$G1_PC2_HOST）"
if tcp_ok "$G1_PC2_HOST" 22; then
    ok "$G1_PC2_HOST:22 に届く"
    W="$(pc2 'nmcli -t -f DEVICE,STATE,CONNECTION device status | grep ^wlan0' 2>/dev/null || true)"
    case "$W" in
        *g1-teleop-client*) ok "PC2 が AP に付いている" ;;
        *) bad "PC2 の wlan0 が AP に付いていない（いま: ${W##*:}）"
           echo "     → ssh 先で nmcli connection up g1-teleop-client" >&2
           echo "     ⚠️ AP より先に PC2 が起動すると社内 Wi-Fi に付く" >&2
           FAIL=1 ;;
    esac
    SC="$(pc2 'echo $SSH_CLIENT' 2>/dev/null | awk '{print $1}')"
    [ "$SC" = "10.42.0.1" ] && ok "AP の NAT が効いている（SSH_CLIENT=$SC）" \
                            || warn "SSH_CLIENT=$SC（10.42.0.1 のはず）"
else
    bad "PC2 に届かない。機体の電源と AP を確認する"
    exit 1
fi

# ── 5. PC2 の排他 ──────────────────────────────────────────────────
step "5. PC2 の排他ロック"
if pc2 'mkdir /tmp/pc2_lock 2>/dev/null'; then
    ok "/tmp/pc2_lock を取得した"
else
    warn "/tmp/pc2_lock は既に在る（他のセッションが使っているかも）"
    pc2 'ls -ld /tmp/pc2_lock' 2>/dev/null | sed 's/^/       /' >&2
fi

# ── 6. センサ（ここが今日いちばん効いた）────────────────────────────
step "6. センサ（⚠️ LiDAR の IMU は起動ごとに出ないことがある）"
SENS="$(pc2 'cd "$HOME/g1_cfg/apriltag" 2>/dev/null || cd "$HOME"
             python3 /tmp/probe_imu_sdk.py 8 eth0 2>/dev/null' 2>/dev/null || true)"
if [ -z "$SENS" ]; then
    warn "probe_imu_sdk.py が PC2 に無い。ROS 経由で見る"
    SENS="$(pc2_ros 'true' 2>/dev/null; echo)"
    warn "→ up.sh が起動直後に測るので、そこで確認する"
else
    printf '%s\n' "$SENS" | sed 's/^/     /' >&2
    case "$SENS" in
        *imu_livox_mid360*0\ 件*|*imu_livox_mid360*⛔*)
            bad "**LiDAR の IMU が出ていない。FAST-LIO2 は初期化できない**"
            echo "     → PC1 に入れないので、**機体の電源を入れ直す**しかない" >&2
            echo "     （2026-09-16 実測: 入れ直したら 200.0 Hz で復活した）" >&2
            FAIL=1 ;;
        *) ok "LiDAR と IMU が来ている" ;;
    esac
fi

# ── 7. 既に何か動いていないか ───────────────────────────────────────
step "7. 二重起動の確認（⚠️ 測位が 2 つ出ると /tf が壊れる）"
RUNNING="$(pc2 'ps -eo args | grep -cE "fastlio_mapping|global_localization_node|mola-cli|nav2_amcl"' 2>/dev/null || echo 1)"
if [ "${RUNNING:-1}" -le 1 ]; then
    ok "測位は動いていない（これから起こす）"
else
    warn "測位らしきものが $((RUNNING-1)) 個動いている"
    echo "     → 続けるなら bash quickstart/live/down.sh で一度落とす" >&2
fi

printf '\n'
if [ "$FAIL" -eq 0 ]; then
    say "点検おわり。**bash quickstart/live/up.sh** に進んでよい"
else
    die "上の NG を直してから もう一度"
fi
