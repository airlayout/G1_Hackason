#!/usr/bin/env python
"""実機G1と疎通できるかを、実際にロボットを動かす前に確認するスクリプト。

lerobot/unitree_sdk2pyの環境構築は不要(標準ライブラリのみ)。ネットワークが
繋がっているかどうかだけを、walk_forward_real.py等を実行する前に素早く
確認するためのもの。

以下を順に確認する:
  1. このPCに 192.168.123.0/24 の static IP が設定されているか(有線接続時の前提)
  2. G1(デフォルト 192.168.123.164)への ping 応答
  3. SSH(22番ポート)への到達性
  4. (--ssh-password 指定時のみ) 実際に ssh <user>@<host> でログインできるか
     (sshpassコマンドが必要)
  5. (--check-bridge-ports 指定時のみ) run_g1_server.py が起動済みなら開くはずの
     ZMQポート(lowcmd:6000, lowstate:6001, camera:5555)への到達性
     (run_g1_server.py未起動なら閉じているのが正常なので、これだけは
     失敗してもREADY判定に影響させない)

前提知識(このプロジェクトでの接続情報):
  - 有線: G1のIPは192.168.123.164固定。操作PCは同一サブネット
    192.168.123.x (x!=164) の static IPをEthernetに設定して直結する。
  - WiFi: G1のWiFiは初期状態で無効。有線で入って
    `rfkill unblock all` → nmcli で有効化する必要がある(初回のみ、
    lerobotドキュメントの"Enable WiFi on the Robot"参照)。有効化後は
    ルーターが割り当てたIPへ ssh unitree@<WiFiのIP> で入れる
    (このスクリプトでは --host で指定する)。
  - SSHユーザー/パスワードは "unitree"/"123" が工場出荷時デフォルト
    (Unitree代理店ドキュメント記載)。この機体で有効かは未確認のため、
    --ssh-password で実際に試して確認できるようにしている。

使い方:
  python scripts/check_g1_connectivity.py
  python scripts/check_g1_connectivity.py --host 192.168.123.164 --ssh-password 123
  python scripts/check_g1_connectivity.py --host <WiFiのIP> --check-bridge-ports
"""
import argparse
import ipaddress
import shutil
import socket
import subprocess
import sys


def check_local_static_ip(subnet: str):
    """このPCのネットワークインターフェースに subnet 内のIPが設定されているか調べる。
    戻り値: [(iface, ip), ...] のリスト。`ip`コマンドが無い場合は None。
    """
    net = ipaddress.ip_network(subnet)
    try:
        result = subprocess.run(
            ["ip", "-o", "-4", "addr", "show"], capture_output=True, text=True, timeout=5
        )
    except FileNotFoundError:
        return None

    matches = []
    for line in result.stdout.splitlines():
        parts = line.split()
        try:
            idx = parts.index("inet")
            addr = parts[idx + 1]  # 例: "192.168.123.200/24"
            ip = ipaddress.ip_interface(addr).ip
            iface = parts[1]
        except (ValueError, IndexError):
            continue
        if ip in net:
            matches.append((iface, str(ip)))
    return matches


def check_ping(host: str, count: int = 3, timeout_s: int = 1):
    cmd = ["ping", "-c", str(count), "-W", str(timeout_s), host]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=count * timeout_s + 5)
    return result.returncode == 0, result.stdout


def check_tcp_port(host: str, port: int, timeout_s: float = 3.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def check_ssh_login(host: str, user: str, password: str, timeout_s: int = 10):
    """sshpass経由で実際にログインを試す。sshpassが無ければ None を返す(スキップ扱い)。"""
    if shutil.which("sshpass") is None:
        return None
    cmd = [
        "sshpass",
        "-p",
        password,
        "ssh",
        "-o",
        "StrictHostKeyChecking=accept-new",
        "-o",
        f"ConnectTimeout={timeout_s}",
        f"{user}@{host}",
        "echo",
        "SSH_OK",
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s + 5)
    except subprocess.TimeoutExpired:
        return False
    return result.returncode == 0 and "SSH_OK" in result.stdout


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--host", default="192.168.123.164", help="G1のIP(有線は固定、WiFiは可変)")
    parser.add_argument("--subnet", default="192.168.123.0/24", help="有線接続時に確認するサブネット")
    parser.add_argument("--ssh-user", default="unitree")
    parser.add_argument(
        "--ssh-password",
        default=None,
        help="指定すると実際にsshログインを試す(要sshpass)。工場出荷時デフォルトは'123'",
    )
    parser.add_argument(
        "--check-bridge-ports",
        action="store_true",
        help="run_g1_server.py起動後に開くZMQポート(6000/6001/5555)も確認する(任意)",
    )
    args = parser.parse_args()

    ok = True

    print(f"[1/3] このPCの {args.subnet} 上のIPを確認中...")
    local_ips = check_local_static_ip(args.subnet)
    if local_ips is None:
        print("  SKIP: `ip`コマンドが見つからないため確認できません。")
    elif local_ips:
        for iface, ip in local_ips:
            print(f"  OK: {iface} に {ip} が設定されています。")
    else:
        print(f"  NG: {args.subnet} 上のIPがこのPCに見つかりません。")
        print(
            "      有線接続なら、例えば "
            "`sudo ip addr add 192.168.123.200/24 dev <interface名>` "
            "`sudo ip link set <interface名> up` を実行してください。"
        )
        ok = False

    print(f"\n[2/3] {args.host} への ping...")
    ping_ok, ping_out = check_ping(args.host)
    print("  OK: 応答あり" if ping_ok else "  NG: 応答なし")
    if not ping_ok:
        print("  " + "\n  ".join(ping_out.strip().splitlines()[-3:]))
        ok = False

    print(f"\n[3/3] {args.host}:22 (SSH) への到達性...")
    ssh_port_ok = check_tcp_port(args.host, 22)
    print("  OK: 到達可能" if ssh_port_ok else "  NG: 到達不可")
    ok = ok and ssh_port_ok

    if args.ssh_password is not None:
        print(f"\n[任意] ssh {args.ssh_user}@{args.host} でログイン試行...")
        login_ok = check_ssh_login(args.host, args.ssh_user, args.ssh_password)
        if login_ok is None:
            print("  SKIP: sshpassが見つかりません(`sudo apt install sshpass`で導入できます)。")
        elif login_ok:
            print("  OK: ログイン成功。")
        else:
            print("  NG: ログイン失敗。この機体ではパスワードが工場出荷時既定(123)と異なる可能性があります。")
            ok = False

    if args.check_bridge_ports:
        print("\n[任意] run_g1_server.py用ZMQポートの到達性(未起動ならNGが正常):")
        for name, port in [("lowcmd", 6000), ("lowstate", 6001), ("camera", 5555)]:
            reachable = check_tcp_port(args.host, port)
            print(f"  {name}({port}): {'到達可能' if reachable else '到達不可'}")

    print("\n=== SUMMARY ===")
    print("READY" if ok else "NOT_READY")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
