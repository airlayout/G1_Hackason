#!/usr/bin/env bash
# G1実機に有線Ethernetで直結するための設定。
#
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
# G1本体のIPは192.168.123.164固定。操作PC側は同一サブネット192.168.123.x
# (x!=164) のstatic IPが必要。
#
# 使い方:
#   bash scripts/setup_ethernet_for_g1.sh            # G1用の接続に切り替える
#   bash scripts/setup_ethernet_for_g1.sh --revert   # 通常のDHCP接続に戻す
set -euo pipefail

IFACE="enp3s0"
G1_CONN="g1-link"
DHCP_CONN="netplan-enp3s0"
PC_IP="192.168.123.200/24"

if [[ "${1:-}" == "--revert" ]]; then
    echo "[setup_ethernet_for_g1] ${G1_CONN} を無効化し、通常のDHCP接続に戻します..."
    sudo nmcli connection down "$G1_CONN" 2>/dev/null || true
    sudo nmcli connection up "$DHCP_CONN"
    ip -br addr show "$IFACE"
    exit 0
fi

echo "[setup_ethernet_for_g1] G1とのEthernetケーブルが ${IFACE} に接続されていることを確認してください。"
read -rp "接続済みですか？ [y/N]: " ans
if [[ "${ans,,}" != "y" ]]; then
    echo "中断しました。ケーブルを接続してから再実行してください。"
    exit 1
fi

if nmcli -t -f NAME connection show | grep -qx "$G1_CONN"; then
    echo "[setup_ethernet_for_g1] 既存の ${G1_CONN} プロファイルを使います。"
else
    echo "[setup_ethernet_for_g1] ${G1_CONN} プロファイルを新規作成します (static IP ${PC_IP})..."
    sudo nmcli connection add \
        type ethernet \
        ifname "$IFACE" \
        con-name "$G1_CONN" \
        autoconnect no \
        ipv4.method manual \
        ipv4.addresses "$PC_IP"
fi

echo "[setup_ethernet_for_g1] 設定内容(反映確認):"
nmcli connection show "$G1_CONN" | grep -iE "ipv4\.method|ipv4\.addresses"

echo "[setup_ethernet_for_g1] ${G1_CONN} を有効化します..."
sudo nmcli connection up "$G1_CONN"

echo "[setup_ethernet_for_g1] 現在の設定:"
ip -br addr show "$IFACE"

echo "[setup_ethernet_for_g1] 疎通確認を行う場合:"
echo "  python3 $(dirname "$0")/check_g1_connectivity.py --ssh-password 123"
echo "[setup_ethernet_for_g1] 通常のDHCP接続に戻す場合:"
echo "  bash $0 --revert"
