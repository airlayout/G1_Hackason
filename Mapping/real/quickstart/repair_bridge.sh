#!/usr/bin/env bash
# ケーブルを抜き差ししたあと、**VM を落とさずに** L2 を張り直す。
#
#   bash quickstart/repair_bridge.sh          # 壊れていたら直す（冪等。健全なら何もしない）
#   bash quickstart/repair_bridge.sh check    # 測るだけ。直さない
#
# ## なぜこれで直るのか（2026-09-12 に実測して分かった）
#
# colima の bridged は **2 段**になっている。
#
#   VM ─ col0 ─ vmenet1 ─ [bridge101] ─ (vmnet.framework の内部の口) ─ en8 ─ G1
#                          ^^^^^^^^^^^   ^^^^^^^^^^^^^^^^^^^^^^^^^^
#                          macOS の普通のブリッジ      触れない
#
# ケーブルを抜くと **後段だけが永久に死ぬ**。socket_vmnet は同じ pid で生き続け、
# `col0` も UP のまま IP も付いたままなので、症状は「VM だけ G1 に届かない」としてしか出ない。
# しかも **socket_vmnet を入れ替えても直らない**（lima は VM 生成時に unix socket を
# 1 度掴むきりで、再接続しない。実測で確認済み）。だから今まで VM 再起動しか手が無かった。
#
# ところが前段の bridge101 は **macOS の普通のブリッジ**で、我々が触れる。
# ここに en8 を自分でメンバとして足すと、死んだ内部の口を迂回して L2 が戻る。
# VM も socket_vmnet も落とさずに済む（実測: 再起動 2 分 → これは 1 秒未満）。
#
# ## ⚠️ 健全なときに足してはいけない
#
# vmnet の内部の口が**生きている**状態で en8 を足すと、VM のフレームが
# 「ブリッジ経由」と「内部の口経由」の 2 本で G1 に届き、二重配送になる。
# だからこのスクリプトは **壊れていることを確かめてからしか足さない。**
#
# ## 判定に TCP だけを使ってはいけない
#
# `col0` が**存在しない**とき、VM は default route（lima の NAT, eth0）から
# 192.168.123.0/24 へ出られてしまい、PC2 への TCP が**通ってしまう**。
# 09-10 に入れた `g1_bridge_ok`（コンテナ/VM → PC2 の TCP）はこの型を
# 「生きている」と誤判定する。ここでは **col0 の ARP の状態**で見る。
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
. "$HERE/_common.sh"
G1_TAG=repair

MODE="${1:-fix}"

# col0 経由で PC2 に届くか。
#   0 = 生きている / 1 = 壊れている / 2 = col0 が無い（addm では直せない）
#
# ⚠️ **ARP の状態だけで判定してはいけない。** REACHABLE 以外は
# STALE / DELAY / PROBE いずれも「まだ分からない」であって健全の証拠ではない
# （2026-09-12 に DELAY を健全と誤判定して 1 回外した）。
# ⚠️ **TCP だけで判定してもいけない。** col0 が**無い**とき VM は lima の NAT
# （eth0, default route）から 192.168.123.0/24 に出られてしまい、TCP が通る。
#
# 正しい順序は「col0 が在るか → 経路が col0 か → TCP が張れるか」。
# col0 に /24 が付いていれば直結経路が default より優先されるので、
# その 3 つを通した TCP は L2 の状態そのものになる。ARP は診断用に添えるだけ。
l2_state() {
    local remote out
    remote=$(printf '
        ip link show %s >/dev/null 2>&1 || { echo NOCOL0; exit 0; }
        dev=$(ip route get %s 2>/dev/null | sed -n "s/.* dev \\([^ ]*\\).*/\\1/p" | head -1)
        [ "$dev" = "%s" ] || { echo NOTVIA:${dev:-none}; exit 0; }
        if timeout 3 bash -c "exec 3<>/dev/tcp/%s/%s" >/dev/null 2>&1; then printf OK; else printf NG; fi
        printf ":%%s\\n" "$(ip neigh show %s dev %s | awk "NF{print \\$NF}" | tail -1)"
    ' "$G1_VM_NIC" "$G1_PC2_IP" "$G1_VM_NIC" "$G1_PC2_IP" "$G1_PC2_SSH_PORT" "$G1_PC2_IP" "$G1_VM_NIC")
    out="$(colima ssh -- sh -c "$remote" 2>/dev/null | tr -d '\r' | tail -1)"
    case "$out" in
        NOCOL0)   echo nocol0;            return 2 ;;
        NOTVIA:*) echo "${out}";          return 2 ;;
        OK:*)     echo "${out#OK:}";      return 0 ;;
        NG:*)     echo "${out#NG:}";      return 1 ;;
        *)        echo "${out:-unknown}"; return 1 ;;
    esac
}

# VM が居る macOS 側のブリッジを探す。**名前は固定ではない**
# （socket_vmnet を起動し直すたびに bridge101 → bridge102 … と増える）。
# col0 の MAC を学習しているブリッジが答え。macOS の Address cache は
# 先頭の 0 を落として表示する（02:... → 2:...）ので、そこを揃えてから探す。
find_vm_bridge() {
    local mac short b
    mac="$(colima ssh -- cat "/sys/class/net/$G1_VM_NIC/address" 2>/dev/null | tr -d '\r')"
    [ -n "$mac" ] || return 1
    short="$(printf '%s' "$mac" | sed 's/\b0\([0-9a-f]\)/\1/g')"
    for b in $(ifconfig -l | tr ' ' '\n' | grep '^bridge'); do
        if ifconfig "$b" 2>/dev/null | grep -qi "$short"; then echo "$b"; return 0; fi
    done
    return 1
}

# ── 測る ───────────────────────────────────────────────────────────
ifconfig "$G1_MAC_IFACE" >/dev/null 2>&1 \
    || g1_die "$G1_MAC_IFACE が無い。有線アダプタを挿すこと"
[ -n "$(ifconfig "$G1_MAC_IFACE" | awk '/inet /{print $2}')" ] \
    || g1_die "$G1_MAC_IFACE に IPv4 が無い。ケーブルを挿すこと"
colima status >/dev/null 2>&1 || g1_die "VM が動いていない。start_rviz_mac.sh を通すこと"

state="$(l2_state)"; rc=$?
case $rc in
    0) g1_ok "L2 は生きている（$G1_PC2_IP の ARP = ${state}）。何もしない"; exit 0 ;;
    2) g1_bad "$G1_VM_NIC が無い。**これは addm では直せない**"
       echo "  古い socket_vmnet が残っていると colima start は黙ってブリッジ無しで上がる。"
       echo "  次を順に通すこと:"
       echo "    sudo pkill -f '/opt/colima/bin/socket_vmnet'"
       echo "    G1_COLIMA_RESTART=always bash $HERE/start_rviz_mac.sh"
       exit 1 ;;
esac
g1_bad "L2 が切れている（$G1_PC2_IP の ARP = ${state}）"
[ "$MODE" = check ] && exit 1

# ── 直す ───────────────────────────────────────────────────────────
BR="$(find_vm_bridge)" || g1_die "VM が居るブリッジを特定できなかった（ifconfig -a を見ること）"
g1_say "VM が居るブリッジ: $BR"

if ifconfig "$BR" | grep -q "member: $G1_MAC_IFACE"; then
    g1_say "$G1_MAC_IFACE は既にメンバ。抜き差しで落ちていない＝別の原因"
else
    t0=$(date +%s)
    if sudo -n /sbin/ifconfig "$BR" addm "$G1_MAC_IFACE" 2>/dev/null; then
        :
    elif [ -t 0 ]; then
        sudo /sbin/ifconfig "$BR" addm "$G1_MAC_IFACE" || g1_die "addm に失敗した"
    else
        g1_bad "sudo にパスワードが要る。手で次を打つこと:"
        echo "    sudo ifconfig $BR addm $G1_MAC_IFACE"
        echo "  毎回打ちたくなければ sudoers に 1 行足す（repair_bridge.sh の末尾を見ること）"
        exit 1
    fi
    g1_say "$BR に $G1_MAC_IFACE を足した（$(( $(date +%s) - t0 )) 秒）"
fi

sleep 1
state="$(l2_state)"; rc=$?
if [ $rc -eq 0 ]; then
    g1_ok "戻った（$G1_PC2_IP の ARP = ${state}）。VM も socket_vmnet も落としていない"
    echo "  ⚠️ ros2 daemon は切れていた頃のグラフを握っている。見えなければ:"
    echo "     docker exec $G1_RVIZ_NAME bash -c 'source /opt/ros/humble/setup.bash && ros2 daemon stop'"
    exit 0
fi
g1_bad "足しても戻らなかった（ARP = ${state}）。VM 再起動に落とすこと:"
echo "    sudo pkill -f '/opt/colima/bin/socket_vmnet'"
echo "    G1_COLIMA_RESTART=always bash $HERE/start_rviz_mac.sh"
exit 1

# ── パスワード無しで打てるようにする（一度だけ手で）───────────────
#
# これを入れると repair_bridge.sh が自分で addm を打てるようになり、
# 「壊れたら 1 コマンドで 1 秒未満」になる。許すのは ifconfig の
# 「ブリッジに en8 を足す」だけなので影響は狭い。
#
#   printf '%%staff ALL=(root:wheel) NOPASSWD:NOSETENV: /sbin/ifconfig bridge[0-9]* addm en8\n' \
#       | sudo tee /etc/sudoers.d/g1-bridge >/dev/null
#   sudo chmod 0440 /etc/sudoers.d/g1-bridge
#   sudo visudo -c        # 構文を確かめる（必ず打つこと。壊すと sudo 全体が使えなくなる）
