#!/usr/bin/env bash
# G1実機に有線Ethernetで直結するための設定。
#
# enp3s0 は NetworkManager 管理下(netplan-enp3s0 プロファイル, 現状 ipv4.method=auto)。
# `ip addr add` で直接IPを入れると、NetworkManagerがDHCPで上書き/再設定してしまう
# ことがあるため、既存プロファイル自体をstatic IPに変更する。
#
# G1本体のIPは192.168.123.164固定。操作PC側は同一サブネット192.168.123.x
# (x!=164) のstatic IPが必要。
#
# 使い方:
#   bash scripts/setup_ethernet_for_g1.sh            # 接続
#   bash scripts/setup_ethernet_for_g1.sh --revert   # DHCPに戻す
set -euo pipefail

IFACE="enp3s0"
CONN="netplan-enp3s0"
PC_IP="192.168.123.200/24"

if [[ "${1:-}" == "--revert" ]]; then
    echo "[setup_ethernet_for_g1] ${CONN} をDHCP(auto)へ戻します..."
    sudo nmcli connection modify "$CONN" ipv4.method auto ipv4.addresses "" ipv4.gateway ""
    sudo nmcli connection up "$CONN"
    ip -br addr show "$IFACE"
    exit 0
fi

echo "[setup_ethernet_for_g1] G1とのEthernetケーブルが ${IFACE} に接続されていることを確認してください。"
read -rp "接続済みですか？ [y/N]: " ans
if [[ "${ans,,}" != "y" ]]; then
    echo "中断しました。ケーブルを接続してから再実行してください。"
    exit 1
fi

echo "[setup_ethernet_for_g1] ${CONN} に static IP ${PC_IP} を設定します..."
sudo nmcli connection modify "$CONN" ipv4.method manual ipv4.addresses "$PC_IP" ipv4.gateway ""
sudo nmcli connection up "$CONN"

echo "[setup_ethernet_for_g1] 現在の設定:"
ip -br addr show "$IFACE"

echo "[setup_ethernet_for_g1] 疎通確認を行う場合:"
echo "  python3 $(dirname "$0")/check_g1_connectivity.py --ssh-password 123"
