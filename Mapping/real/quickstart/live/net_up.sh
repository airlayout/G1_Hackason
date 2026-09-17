#!/usr/bin/env bash
# **ネットワークを正しい順番で 1 コマンドで立ち上げる。**冪等（健全なら何もしない）。
#
#   bash quickstart/live/net_up.sh           # 自動判定（有線が生きていれば有線）
#   bash quickstart/live/net_up.sh --ap      # AP 構成にする（無線で歩かせる）
#   bash quickstart/live/net_up.sh --wired   # 有線で使う（AP は上げない）
#   bash quickstart/live/net_up.sh --check   # 測るだけ。何も変えない
#
# ## なぜ要るか
#
# 2026-09-17 のセッションで、**ここだけで診断込み 10 分**溶けた。手順は 3 つしかないが、
# **順番を間違えると PC2 に届かなくなって詰む。**
#
# ## ⚠️ 順番が決まっている理由
#
#   1. Mac の L2（bridge101 に en8）… ここが死んでいるとコンテナが機体を見られない。
#      ケーブルを抜き差しすると**後段だけが永久に死ぬ**（repair_bridge.sh の注記）
#   2. OMEN の AP（g1-teleop-ap）    … ⚠️ 上げると **OMEN の社内 Wi-Fi が切れる**
#      （無線 1 枚で AP と STA は同居できない。カーネルが未対応）
#   3. PC2 を AP に付ける（g1-teleop-client）
#      ⚠️⚠️ **PC2 は電源を入れると社内 Wi-Fi に付く。**この時点で PC2 へ届く経路は
#      「有線」か「既に AP に付いている」かのどちらかしか無い。**両方無いと詰む。**
#      ⇒ 恒久策は PC2 側で 1 回:
#           nmcli connection modify g1-teleop-client connection.autoconnect-priority 100
#           nmcli connection modify Fujitsu_free_Wi-Fi connection.autoconnect-priority 0
#         これで電源 ON → AP を上げる → PC2 が自分で付く、になり有線が不要になる。
#         このスクリプトは届いたときに**自動でこの設定を入れる**（--no-persist で抑止）。
#
# ## 判定に使う道具（⚠️ 壊れている計器を避ける）
#
# - `ping` は使わない … **コンテナに入っていない**（自分の IP にも「応答なし」と出る）
# - macOS の `/dev/tcp` は使わない … **zsh に無い**
# - PC2 側で `ps | grep` しない … 自己マッチ（`_common.sh` の `pc2_procs` を使う）
# ⇒ 到達性は **ssh が通るか**で見る。ARP は診断に添えるだけ。
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/_common.sh"

MODE=auto
PERSIST=1
while [ $# -gt 0 ]; do
    case "$1" in
        --ap)    MODE=ap ;;
        --wired) MODE=wired ;;
        --check) MODE=check ;;
        --no-persist) PERSIST=0 ;;
        *) sed -n '2,8p' "$0" >&2; exit 2 ;;
    esac
    shift
done

ssh_ok() {   # $1=user@host → ssh が通るか
    ssh -i "$G1_KEY" -o BatchMode=yes -o ConnectTimeout=8 -o IdentitiesOnly=yes \
        "$1" true 2>/dev/null
}
ap_ok() { ssh -o BatchMode=yes -o ConnectTimeout=8 "$G1_AP_USER@$G1_AP_HOST" true 2>/dev/null; }

FAIL=0

# ── 1. Mac の L2 ───────────────────────────────────────────────────
say "1. Mac の L2（bridge101 に en8 が入っているか）"
if [ "$MODE" = "check" ]; then
    bash "$(dirname "$HERE")/repair_bridge.sh" check || FAIL=1
else
    # repair_bridge.sh は冪等（健全なら何もしない）。壊れているときだけ addm する
    bash "$(dirname "$HERE")/repair_bridge.sh" || true
fi

# ── 2. PC2 にどこから届くか ────────────────────────────────────────
say "2. PC2 への経路を探す"
REACH=""
if ssh_ok "$G1_PC2_USER@$G1_PC2_WIRED"; then
    REACH=wired; ok "有線 $G1_PC2_WIRED に届く"
fi
if ssh_ok "$G1_PC2_USER@$G1_PC2_WIRELESS"; then
    REACH="${REACH:+$REACH+}ap"; ok "AP 経由 $G1_PC2_WIRELESS に届く"
fi
[ -n "$REACH" ] || bad "PC2 にどこからも届かない"

if [ "$MODE" = auto ]; then
    case "$REACH" in
        *wired*) MODE=wired ;;
        *ap*)    MODE=ap ;;
        *)       MODE=ap ;;      # 届かないなら AP を立てて待つしかない
    esac
fi
say "   構成: ${MODE}"

# ── 3. AP（ap 構成のときだけ）───────────────────────────────────────
if [ "$MODE" = "ap" ] || [ "$MODE" = "check" ]; then
    say "3. OMEN の AP（${G1_AP_HOST}）"
    if ! ap_ok; then
        bad "$G1_AP_HOST に ssh が通らない。ケーブルの行き先と電源を確認する"
        FAIL=1
    else
        APDEV="$(ap 'nmcli -t -f DEVICE,STATE,CONNECTION device status | grep ^wlp' 2>/dev/null || true)"
        case "$APDEV" in
            *g1-teleop-ap*) ok "AP は既に上がっている（${APDEV%%:*}）" ;;
            *)
                if [ "$MODE" = "check" ]; then
                    bad "AP が上がっていない（いま: ${APDEV##*:}）"; FAIL=1
                else
                    say "   AP を上げる ⚠️ **OMEN の社内 Wi-Fi は切れます**"
                    # ⚠️ 有線（G1-ethernet）経由で叩くので この ssh は切れない
                    ap 'setsid nohup nmcli connection up g1-teleop-ap >/tmp/ap_up.log 2>&1 </dev/null; sleep 12; cat /tmp/ap_up.log' \
                        | sed 's/^/     /' >&2
                    APIP="$(ap "ip -4 -o addr show 2>/dev/null | awk '{print \$2, \$4}' | grep -E '^wlp'" 2>/dev/null || true)"
                    [ -n "$APIP" ] && ok "AP の IP: $APIP" || { bad "AP の IP が付かない"; FAIL=1; }
                fi ;;
        esac
        NAT="$(ap 'grep -c "nat POSTROUTING" /etc/NetworkManager/dispatcher.d/50-g1teleop-forward 2>/dev/null || true' 2>/dev/null | tr -d '[:space:]')"
        [ "${NAT:-0}" -gt 0 ] 2>/dev/null && ok "dispatcher に NAT が入っている（$NAT 件）" \
                                          || { bad "AP の filter/NAT が無い（50-g1teleop-forward）"; FAIL=1; }
    fi
fi

# ── 4. PC2 を AP に付ける ──────────────────────────────────────────
if [ "$MODE" = "ap" ]; then
    say "4. PC2 を AP に付ける"
    case "$REACH" in
        *ap*) ok "既に AP に付いている（${G1_PC2_WIRELESS}）" ;;
        *wired*)
            say "   有線経由で切り替える（この ssh は eth0 なので切れない）"
            G1_PC2_HOST="$G1_PC2_WIRED" pc2 \
                'setsid nohup nmcli connection up g1-teleop-client >/tmp/cl.log 2>&1 </dev/null
                 sleep 15; cat /tmp/cl.log; ip -4 -o addr show wlan0 | awk "{print \$4}"' \
                | sed 's/^/     /' >&2 ;;
        *)
            bad "PC2 に届かないので切り替えられない"
            echo "     → ⚠️ **有線を 1 回繋いでから** もう一度このスクリプトを走らせる" >&2
            echo "        （または PC2 のコンソールで nmcli connection up g1-teleop-client）" >&2
            FAIL=1 ;;
    esac
fi

# ── 5. 恒久化（次回から有線が要らなくなる）─────────────────────────
if [ "$MODE" = "ap" ] && [ "$PERSIST" = "1" ] && [ "$FAIL" = "0" ]; then
    say "5. PC2 の自動接続を AP 優先にする（次回から有線が不要になる）"
    G1_PC2_HOST="$G1_PC2_WIRELESS" pc2 \
        'p=$(nmcli -g connection.autoconnect-priority connection show g1-teleop-client 2>/dev/null)
         if [ "$p" = "100" ]; then echo "     既に AP 優先（priority 100）"; else
           nmcli connection modify g1-teleop-client connection.autoconnect-priority 100 2>&1
           nmcli connection modify g1-teleop-client connection.autoconnect yes 2>&1
           for c in $(nmcli -t -f NAME connection show | grep -vE "^g1-teleop-client$|^lo$|^unitree"); do
             nmcli connection modify "$c" connection.autoconnect-priority 0 2>/dev/null || true
           done
           echo "     AP 優先にした（priority 100）。次回は電源 ON で自分で付く"
         fi' 2>&1 | sed 's/^/  /' >&2 || warn "恒久化に失敗した（次回も有線が要る）"
fi

# ── 6. 通しの確認 ──────────────────────────────────────────────────
printf '\n'
say "確認"
TARGET="$G1_PC2_WIRELESS"
[ "$MODE" = "wired" ] && TARGET="$G1_PC2_WIRED"
if G1_PC2_HOST="$TARGET" pc2 'echo "SSH_CLIENT=$SSH_CLIENT"' 2>/dev/null | sed 's/^/     /' >&2; then
    ok "PC2（${TARGET}）に届く"
    # ⚠️ AP 構成なら SSH_CLIENT が 10.42.0.1（OMEN の NAT）になるのが正しい
else
    bad "PC2（${TARGET}）に届かない"; FAIL=1
fi

printf '\n'
if [ "$FAIL" = "0" ]; then
    say "ネットワークは立った。次: bash quickstart/live/preflight.sh"
    [ "$MODE" = "ap" ] && say "  （以降 G1_PC2_HOST=$G1_PC2_WIRELESS G1_LINK=ap を渡す）"
    exit 0
fi
say "⛔ 上の NG を直してから もう一度"
exit 1
