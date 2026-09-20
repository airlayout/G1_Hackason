#!/usr/bin/env bash
# G1実機に有線Ethernetで直結するための設定。
#
# ## 既存プロファイルに触らない理由
# enp3s0 は既に NetworkManager 管理下(netplan-enp3s0 プロファイル、DHCP=auto)にある。
# この既存プロファイルを `nmcli connection modify` で static IP に書き換えようとしたが、
# ipv4.addresses は反映されても ipv4.method だけ何度やっても "auto" に戻ってしまう
# (netplan生成プロファイル特有の挙動と見られる。原因未特定)。
#
# そのため既存プロファイルには一切触れず、G1接続専用の新しい接続プロファイル
# ("g1-link", autoconnect=no)を別途作成する方式にした。有効化は明示的に
# `nmcli connection up g1-link` した時だけなので、普段のDHCP接続
# (netplan-enp3s0)に影響しない。
#
# ## アドレスを決め打ちしない理由
# G1本体のIPは 192.168.123.164 固定。操作PC側は同一サブネットの static IP が要る。
# **G1の内蔵スイッチにはG1本体以外の作業者もぶら下がる。** 以前はここで .200 を
# 決め打ちしていたが、全員がこのスクリプトを実行すると全員が .200 を要求して衝突した。
#
#   2026-08-26  nmcli が「IP 設定を確保できませんでした」で有効化に失敗
#   2026-09-06  macOS 側が重複を検知して .200 を放棄（設定は Manual .200 のまま残るので
#               サービスを off/on しても戻らない）
#
# どちらも設定だけが残って疎通が無言で消えるので気付きにくい。そこで既定では
# arping の DAD (Duplicate Address Detection) で空きアドレスを探して選ぶ。
# 使うアドレスを決めているときは引数で渡す。
#
# 使い方:
#   bash Common/network/setup_ethernet_for_g1.sh                    # 空きを自動で選ぶ
#   bash Common/network/setup_ethernet_for_g1.sh 222                # .222 を使う
#   bash Common/network/setup_ethernet_for_g1.sh 192.168.123.222    # 同上
#   bash Common/network/setup_ethernet_for_g1.sh --revert           # 通常のDHCP接続に戻す
#
# 環境変数:
#   G1_IFACE      有線NIC名 (既定 enp3s0)
#   G1_DHCP_CONN  --revert で戻す先のプロファイル名 (既定 netplan-enp3s0)
#   G1_SCAN_FROM / G1_SCAN_TO  自動選択で走査するホスト部の範囲 (既定 200〜250)
set -euo pipefail

IFACE="${G1_IFACE:-enp3s0}"
G1_CONN="g1-link"
DHCP_CONN="${G1_DHCP_CONN:-netplan-enp3s0}"

SUBNET="192.168.123"
PREFIX_LEN=24
SCAN_FROM="${G1_SCAN_FROM:-200}"
SCAN_TO="${G1_SCAN_TO:-250}"

# G1側で固定的に使われているホスト部。自動選択の候補から外し、明示指定も拒否する。
#   .1   PC2 の unitree1 プロファイルが default gateway に指定している(実体は不在)
#   .120 LiDAR (Livox MID-360) / .161 PC1(运控PC) / .164 PC2(NX开发板)
RESERVED=(0 1 120 161 164 255)

say()  { echo "[setup_ethernet_for_g1] $*"; }
die()  { echo "[setup_ethernet_for_g1] $*" >&2; exit 1; }

usage() {
    sed -n '/^# 使い方:/,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//; /^set -euo/d'
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

if [[ "${1:-}" == "--revert" ]]; then
    say "${G1_CONN} を無効化し、通常のDHCP接続に戻します..."
    sudo nmcli connection down "$G1_CONN" 2>/dev/null || true
    if nmcli -t -f NAME connection show | grep -qx "$DHCP_CONN"; then
        sudo nmcli connection up "$DHCP_CONN"
    else
        say "${DHCP_CONN} が無いので autoconnect に任せます (G1_DHCP_CONN で指定できます)"
    fi
    ip -br addr show "$IFACE" || true
    exit 0
fi

REQUESTED="${1:-}"

[ -e "/sys/class/net/$IFACE" ] || die "NIC ${IFACE} がありません。G1_IFACE で指定してください。
利用可能なNIC:
$(ip -br link show | awk '{print "  " $1}')"

# "222" も "192.168.123.222" も受ける。ホスト部だけを返す。
normalize_host() {
    local given="$1" host
    if [[ "$given" == *.* ]]; then
        [ "${given%.*}" = "$SUBNET" ] || die "${given} は ${SUBNET}.0/${PREFIX_LEN} の外です。"
        host="${given##*.}"
    else
        host="$given"
    fi
    if ! [[ "$host" =~ ^[0-9]+$ ]] || [ "$host" -lt 1 ] || [ "$host" -gt 254 ]; then
        die "アドレスの指定が不正です: ${given}"
    fi
    printf '%s' "$host"
}

is_reserved() {
    local h="$1" r
    for r in "${RESERVED[@]}"; do
        if [ "$h" = "$r" ]; then return 0; fi
    done
    return 1
}

have_arping() { command -v arping >/dev/null 2>&1; }

# arping の DAD モード。応答が無ければ 0(空き)、応答があれば 1(使用中)。
# 自分にまだIPが無くてもL2で問い合わせるので、static IP を振る前に使える。
is_free() {
    sudo arping -D -q -c 2 -w 2 -I "$IFACE" "$1"
}

pick_free_host() {
    local h
    for ((h = SCAN_FROM; h <= SCAN_TO; h++)); do
        if is_reserved "$h"; then continue; fi
        printf '  %s.%-3s ... ' "$SUBNET" "$h" >&2
        if is_free "${SUBNET}.${h}"; then
            echo "空き" >&2
            printf '%s' "$h"
            return 0
        fi
        echo "使用中" >&2
    done
    return 1
}

say "G1とのEthernetケーブルが ${IFACE} に接続されていることを確認してください。"
read -rp "接続済みですか？ [y/N]: " ans
if [[ "${ans,,}" != "y" ]]; then
    echo "中断しました。ケーブルを接続してから再実行してください。"
    exit 1
fi

if [ -n "$REQUESTED" ]; then
    HOST="$(normalize_host "$REQUESTED")"
    if is_reserved "$HOST"; then
        die "${SUBNET}.${HOST} はG1側が使っています。別のアドレスを指定してください。"
    fi
    if have_arping; then
        say "${SUBNET}.${HOST} が空いているか確認します..."
        if ! is_free "${SUBNET}.${HOST}"; then
            die "${SUBNET}.${HOST} は既に使われています。別のアドレスを指定するか、引数無しで実行して自動選択させてください。"
        fi
    else
        say "警告: arping が無いので重複確認をスキップします (sudo apt install -y iputils-arping)"
    fi
else
    have_arping || die "空きアドレスの自動選択には arping が要ります。
  sudo apt install -y iputils-arping
入れずに進めるなら、使うアドレスを明示してください:
  bash $0 222"
    say "${SUBNET}.${SCAN_FROM}〜.${SCAN_TO} から空きを探します..."
    HOST="$(pick_free_host)" \
        || die "${SUBNET}.${SCAN_FROM}〜.${SCAN_TO} に空きがありませんでした。G1_SCAN_FROM / G1_SCAN_TO で範囲を変えてください。"
fi

PC_IP="${SUBNET}.${HOST}/${PREFIX_LEN}"

# 既存プロファイルがあっても必ずアドレスを入れ直す。以前は「既存を使い回す」だけで、
# アドレスを変えたいのに古い値が残り続ける事故があった。
if nmcli -t -f NAME connection show | grep -qx "$G1_CONN"; then
    say "既存の ${G1_CONN} プロファイルを ${PC_IP} に更新します。"
    sudo nmcli connection modify "$G1_CONN" \
        connection.interface-name "$IFACE" \
        connection.autoconnect no \
        ipv4.method manual \
        ipv4.addresses "$PC_IP"
else
    say "${G1_CONN} プロファイルを新規作成します (static IP ${PC_IP})..."
    sudo nmcli connection add \
        type ethernet \
        ifname "$IFACE" \
        con-name "$G1_CONN" \
        autoconnect no \
        ipv4.method manual \
        ipv4.addresses "$PC_IP"
fi

say "設定内容(反映確認):"
nmcli connection show "$G1_CONN" | grep -iE "ipv4\.method|ipv4\.addresses"

say "${G1_CONN} を有効化します..."
sudo nmcli connection up "$G1_CONN"

say "現在の設定:"
ip -br addr show "$IFACE"

# nmcli が成功を返してもアドレスが載らないことがある(衝突時など)ので実物を確認する。
if ! ip -4 -o addr show dev "$IFACE" | grep -q " ${SUBNET}\.${HOST}/"; then
    die "${PC_IP} が ${IFACE} に載っていません。衝突を疑ってください:
  ip neigh show dev ${IFACE}"
fi
say "OK: ${IFACE} = ${SUBNET}.${HOST}"

echo
say "疎通確認を行う場合:"
echo "  python3 $(dirname "$0")/check_g1_connectivity.py --ssh-password 123"
say "G1本体にインターネットを共有する場合、G1側のdefault routeはこのPCのアドレスに向ける:"
echo "  ssh g1 'sudo ip route replace default via ${SUBNET}.${HOST} dev eth0'"
say "通常のDHCP接続に戻す場合:"
echo "  bash $0 --revert"
