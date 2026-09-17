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
# ⚠️ **有線構成では AP は要らない。**2026-09-17 に PC2 を有線（192.168.123.164）で
# 繋いだところ、コンテナ（col0 = 192.168.123.201）から機体の DDS が全部見え、
# foxglove 中継も不要だった。AP は「PC2 を無線にしたいとき」だけの仕掛けである。
#   G1_LINK=wired … AP の項目を情報表示にする（既定。PC2 の有線 IP に届けばこちら）
#   G1_LINK=ap    … AP 構成として厳しく見る
LINK="${G1_LINK:-auto}"
if [ "$LINK" = "auto" ]; then
    if tcp_ok "$G1_PC2_WIRED" 22; then LINK=wired; else LINK=ap; fi
fi
say "リンク構成: ${LINK}（G1_LINK で固定できる）"

step "3. AP を出している Ubuntu（${G1_AP_HOST}）"
if [ "$LINK" = "wired" ]; then
    note "有線構成なので AP は見ない（G1_LINK=ap で厳しく見る）"
elif tcp_ok "$G1_AP_HOST" 22; then
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
    NAT="$(ap 'grep -c "nat POSTROUTING" /etc/NetworkManager/dispatcher.d/50-g1teleop-forward 2>/dev/null || true' 2>/dev/null)"
    [ "${NAT:-0}" -gt 0 ] && ok "dispatcher に NAT が入っている（$NAT 件）" \
                          || { bad "AP の filter/NAT が無い"; FAIL=1; }
else
    bad "$G1_AP_HOST に届かない。ケーブルの行き先を確認する"
    FAIL=1
fi

# ── 4. PC2（無線）──────────────────────────────────────────────────
step "4. PC2（${G1_PC2_HOST}）"
if tcp_ok "$G1_PC2_HOST" 22; then
    ok "$G1_PC2_HOST:22 に届く"
    if [ "$LINK" = "wired" ]; then
        note "有線構成なので wlan0 は見ない"
        # ⚠️ **2 NIC は残る。**wlan0 が社内 Wi-Fi に付いたままでも、
        # CycloneDDS は放っておくと wlan0 を掴んで eth0 側の PC1 が見えなくなる。
        # live の各スクリプトが CYCLONEDDS_URI を eth0 に固定しているのが前提
        NICS="$(pc2 'ip -4 -o addr show scope global | awk "{print \$2}" | tr "\n" " "' 2>/dev/null || true)"
        note "PC2 の NIC: ${NICS}（eth0 固定が要る）"
    else
        W="$(pc2 'nmcli -t -f DEVICE,STATE,CONNECTION device status | grep ^wlan0' 2>/dev/null || true)"
        case "$W" in
            *g1-teleop-client*) ok "PC2 が AP に付いている" ;;
            *) bad "PC2 の wlan0 が AP に付いていない（いま: ${W##*:}）"
               echo "     → ssh 先で nmcli connection up g1-teleop-client" >&2
               echo "     ⚠️ AP より先に PC2 が起動すると社内 Wi-Fi に付く" >&2
               FAIL=1 ;;
        esac
        SC="$(pc2 'echo $SSH_CLIENT' 2>/dev/null | awk '{print $1}')"
        [ "$SC" = "10.42.0.1" ] && ok "AP の NAT が効いている（SSH_CLIENT=${SC}）" \
                                || warn "SSH_CLIENT=${SC}（10.42.0.1 のはず）"
    fi
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
# ⚠️ **有線構成ではコンテナから直接測れる。**機体の DDS が col0 に載っているので、
# PC2 に何も置かずに `ros2 topic hz` で見られる（2026-09-17 に確認）。
# 以前ここが参照していた `probe_imu_sdk.py` は**一度も書かれていない**。
step "6. センサ（⚠️ LiDAR の IMU は起動ごとに出ないことがある）"
probe_hz() {   # $1=topic → 実測レートを印字（取れなければ空）
    local out
    if [ "$LINK" = "wired" ]; then
        # 有線なら機体の DDS が col0 に載っているのでコンテナから測れる
        out="$(ctr "source /opt/ros/humble/setup.bash >/dev/null 2>&1
                    timeout 10 ros2 topic hz $1 2>/dev/null \
                      | awk '/average rate/{print \$3; exit}'" 2>/dev/null)"
    else
        # ⚠️ AP 構成では DDS が AP を越えられないので**コンテナからは測れない**。
        # PC2 の上で測る。jros2（シェル関数）を timeout に渡さないこと
        out="$(pc2_ros2 14 topic hz "$1" 2>/dev/null \
               | awk '/average rate/{print $3; exit}')"
    fi
    printf '%s' "$(printf '%s' "$out" | tr -d '[:space:]')"
}
for spec in "/utlidar/cloud_livox_mid360 10" "/utlidar/imu_livox_mid360 200" "/dog_odom 900"; do
    topic="${spec%% *}"; want="${spec##* }"
    rate="$(probe_hz "$topic")"
    if [ -z "$rate" ]; then
        if [ "$topic" = "/utlidar/imu_livox_mid360" ]; then
            bad "**LiDAR の IMU が出ていない。FAST-LIO2 は初期化できない**"
            echo "     → PC1 に入れないので、**機体の電源を入れ直す**しかない" >&2
            echo "     （2026-09-16 実測: 入れ直したら 200.0 Hz で復活した）" >&2
            FAIL=1
        else
            bad "${topic} が来ていない"
            FAIL=1
        fi
    else
        # awk で整数比較（bash に浮動小数は無い）
        if awk -v r="$rate" -v w="$want" 'BEGIN{exit !(r > w*0.5)}'; then
            ok "${topic} = ${rate} Hz（期待 ${want} 前後）"
        else
            warn "${topic} = ${rate} Hz（期待 ${want} 前後より遅い）"
        fi
    fi
done

# ── 7. 既に何か動いていないか ───────────────────────────────────────
step "7. 二重起動の確認（⚠️ 測位が 2 つ出ると /tf が壊れる）"
# ⚠️ **パターンを角括弧で割る。**そうしないと `ps -eo args` の出力に
# この grep 自身（と ssh の bash -c）のコマンド行が入り、**常に 2 件見つかる**。
# 2026-09-17 に「測位らしきものが 1 個動いている」と誤報した（実際は 0 個）。
# comm では見分けられない —— PC2 の ROS ノードは jammy のローダ経由なので
# comm が全部 `ld-linux-aarch6` になる。
# ⚠️ `grep -c` は 0 件のとき終了コード 1 を返すので、リモート側で `|| true` して
# **数字を 1 個だけ**返させる（外で `|| echo 0` を足すと "0\n0" になって
# `[: integer expression expected` で落ちる。2026-09-17 に踏んだ）
RUNNING="$(pc2 'ps -eo args | grep -cE "fastlio[_]mapping|global[_]localization_node|mola[-]cli|nav2[_]amcl" || true' 2>/dev/null | tr -d "[:space:]")"
if [ "${RUNNING:-0}" -eq 0 ] 2>/dev/null; then
    ok "測位は動いていない（これから起こす）"
else
    warn "測位らしきものが ${RUNNING} 個動いている"
    echo "     → 続けるなら bash quickstart/live/down.sh で一度落とす" >&2
fi

printf '\n'
if [ "$FAIL" -eq 0 ]; then
    say "点検おわり。**bash quickstart/live/up.sh** に進んでよい"
else
    die "上の NG を直してから もう一度"
fi
