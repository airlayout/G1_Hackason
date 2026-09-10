#!/usr/bin/env python3
"""Quest から この PC に届くかを、テレオペを起動せずに確認する。

テレオペが繋がらない原因の多くはネットワーク側にある。これはテレオペ本体と
**同じポート 8012・同じ証明書**で待ち受けるので、「Quest からこのページが開ける
= テレオペにも到達できる」と判断してよい。ロボットは不要。

とくに引っかかりやすいのが WiFi の**クライアント分離**（プライバシーセパレータ）。
これが有効だと Quest → PC の直接通信が遮断され、PC 側では回避できない。

使い方:
    ./scripts/quest_check.sh              # ラッパー経由
    python3 tools/quest_check.py

    --port  待ち受けポート (default: 8012 = teleop と同じ)

Ctrl+C で終了。teleop とは同時に起動できない（ポートが競合する）。
"""

import argparse
import http.server
import re
import socket
import ssl
import subprocess
import sys
from pathlib import Path

CERT_DIR = Path.home() / "xr_teleoperate/teleop/televuer"
ROBOT_SUBNET = "192.168.123."
INTERNET_HOST = "github.com"


def section(title):
    print(f"\n\033[1m=== {title} ===\033[0m")


def ok(msg):
    print(f"  \033[32m[OK]\033[0m   {msg}")


def warn(msg):
    print(f"  \033[33m[注意]\033[0m {msg}")


def ng(msg):
    print(f"  \033[31m[NG]\033[0m   {msg}")


def wifi_status():
    """接続中の WiFi インターフェース・SSID・IP を返す。"""
    section("1. WiFi 接続状態")

    dev = ssid = None
    out = subprocess.run(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "dev", "status"],
                         capture_output=True, text=True).stdout
    for line in out.splitlines():
        f = line.split(":")
        if len(f) >= 4 and f[1] == "wifi" and f[2] == "connected":
            dev, ssid = f[0], f[3]
            break
    if dev is None:
        ng("WiFi に繋がっていません。Quest と同じ SSID に接続してから再実行してください。")
        return None, None

    addr = subprocess.run(["ip", "-4", "-br", "addr", "show", dev],
                          capture_output=True, text=True).stdout
    m = re.search(r"(\d+\.\d+\.\d+\.\d+)/(\d+)", addr)
    if not m:
        ng(f"{dev} に IPv4 アドレスが付いていない（DHCP 待ち / 認証未通過の可能性）。")
        return dev, None
    ip, prefix = m.group(1), m.group(2)

    print(f"  interface : {dev}")
    print(f"  SSID      : {ssid}")
    print(f"  IP        : {ip}/{prefix}")
    ok("WiFi 接続あり")

    if ip.startswith(ROBOT_SUBNET):
        ng(f"この WiFi が {ROBOT_SUBNET}0/24 を使っています。G1 の有線と衝突します。\n"
           "     → 別の WiFi に切り替えてください。")
    return dev, ip


def check_internet():
    """外向きの経路が生きているかを確認する（テレオペ自体には不要）。

    DNS が引けない場合は認証ページ（キャプティブポータル）未通過の可能性がある。
    """
    section("2. インターネット到達性（任意）")

    try:
        socket.setdefaulttimeout(5)
        socket.getaddrinfo(INTERNET_HOST, 443)
        ok(f"DNS 解決できる ({INTERNET_HOST})")
    except OSError as e:
        warn(f"DNS が引けません ({e})。認証ページ未通過の可能性があります。\n"
             "     → ブラウザでネットワークのポータルを通してください。")
        return False

    try:
        with socket.create_connection((INTERNET_HOST, 443), timeout=5):
            ok(f"{INTERNET_HOST}:443 に接続できる")
    except OSError as e:
        warn(f"{INTERNET_HOST}:443 に繋がりません ({e})。\n"
             "     → テレオペ自体はローカル通信なので、これが NG でも操作はできます。")
        return False

    # キャプティブポータル（認証ページ）の検知
    try:
        with socket.create_connection(("connectivitycheck.gstatic.com", 80), timeout=5) as s:
            s.sendall(b"GET /generate_204 HTTP/1.1\r\nHost: connectivitycheck.gstatic.com\r\n"
                      b"Connection: close\r\n\r\n")
            head = s.recv(64).decode("ascii", "replace")
        if "204" in head.split("\r\n")[0]:
            ok("キャプティブポータルなし（素通し）")
        else:
            warn(f"認証ページに横取りされている可能性 (応答: {head.splitlines()[0]})。\n"
                 "     → PC と Quest の両方でブラウザから認証を通す必要がある。")
    except OSError:
        warn("ポータル判定に失敗（外向き HTTP がブロックされている）。")
    return True


class Handler(http.server.BaseHTTPRequestHandler):
    """到達した Quest に成功ページを返し、クライアント IP をログする。"""

    def do_GET(self):
        print(f"  \033[32m[到達]\033[0m クライアント {self.client_address[0]} からアクセスあり "
              f"({self.headers.get('User-Agent', '?')[:60]})")
        body = ("<meta name=viewport content='width=device-width,initial-scale=1'>"
                "<body style='background:#111;color:#0f0;font:bold 6vw sans-serif;"
                "text-align:center;padding-top:20vh'>"
                "OK<br><span style='font-size:3vw;color:#ccc'>"
                "Quest → PC のポート 8012 に到達できました。<br>"
                "この WiFi でテレオペが使えます。</span></body>").encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass  # 既定のアクセスログは抑制（上で自前に出している）


def serve(ip, port):
    """teleop と同じポート・同じ証明書で待ち受け、Quest からの到達を確認する。"""
    section("3. Quest → PC 到達テスト（クライアント分離の判定）")

    cert, key = CERT_DIR / "cert.pem", CERT_DIR / "key.pem"
    if not cert.exists() or not key.exists():
        ng(f"証明書が無い: {cert}\n     televuer の手順で openssl 生成が必要。")
        sys.exit(1)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert, key)
    httpd = http.server.HTTPServer(("0.0.0.0", port), Handler)
    httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)

    print(f"""
  Quest の Meta Quest ブラウザで次の URL を開いてください:

      \033[1mhttps://{ip}:{port}/\033[0m

  1. 証明書の警告 → 「詳細設定 / Advanced」→「アクセスする / Proceed」
  2. 緑色の "OK" が出れば合格（= teleop の vuer にも到達できる）

  \033[33m開けない場合\033[0m … WiFi のクライアント分離（プライバシーセパレータ）が濃厚。
  PC 側では回避できません。別の WiFi（スマホのテザリング等）に切り替えてください。
  詳しくは docs/02-network.md を参照。

  待ち受け中（Ctrl+C で終了）… ※このテスト中は teleop を起動できない
""")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n  終了しました。")


def main():
    p = argparse.ArgumentParser(description="Quest から PC に届くかを確認")
    p.add_argument("--port", type=int, default=8012)
    args = p.parse_args()

    print("\033[1mQuest 到達性チェック — ロボットは不要（何も動きません）\033[0m")

    dev, ip = wifi_status()
    if ip is None:
        sys.exit(1)
    check_internet()
    serve(ip, args.port)

    section("4. 合格したら")
    print(f"""
  テレオペのときに Quest で開く URL はこれです（IP は環境ごとに変わります）:

      https://{ip}:{args.port}/?ws=wss://{ip}:{args.port}
""")


if __name__ == "__main__":
    main()
