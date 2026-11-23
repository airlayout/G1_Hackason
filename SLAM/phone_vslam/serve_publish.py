"""スマホの送信ページ（publish/index.html）を https で配り、MediaMTX への送信の要求を中継する小さなサーバー。

    python3 serve_publish.py --cert server.crt --key server.key [--port 8443] [--mtx https://127.0.0.1:8889]

- スマホのブラウザはカメラを https でしか使わせないので、MediaMTX と同じ自己署名の証明書で配る
- 送信の要求（WHIP: /<名前>/whip・/<名前>/whip/<id>）と publisher.js は MediaMTX に中継する。
  ページと別の番号（8889）に直接送ると、Safari は 8889 の証明書の警告も別に越える必要があり、その許可が
  切れると「TypeError: Load failed」で送れなかった（11-17 の iPhone）。中継すれば警告を越えるのは 8443 の 1 回。
  映像そのもの（UDP 8189）は MediaMTX に直接届く（DTLS の鍵は SDP に載るので証明書は関わらない）
- /log はページの状態の知らせ（スマホの画面を見なくても、何が起きたか記録に残す）

配るのは publish/ の中だけ。標準の Python だけで動く（OMEN の /usr/bin/python3）。
"""

from __future__ import annotations

import argparse
import functools
import http.client
import http.server
import json
import pathlib
import re
import ssl
import time
import urllib.parse

HERE = pathlib.Path(__file__).resolve().parent
MAX_LOG_BODY = 2048
MAX_WHIP_BODY = 256 * 1024  # SDP は数 KB
PROXY_PATH = re.compile(r"^/[A-Za-z0-9_-]+/(publisher\.js|whip(/[A-Za-z0-9_-]+)?)$")
PASS_REQ = ("Content-Type", "If-Match", "Authorization")
PASS_RESP = ("Content-Type", "Location", "Link", "ETag", "Accept-Patch", "Content-Length")
MTX_TIMEOUT_S = 15


class PublishHandler(http.server.SimpleHTTPRequestHandler):
    mtx: urllib.parse.SplitResult  # main で入れる

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")  # 直したページがすぐ届くように
        super().end_headers()

    def log_message(self, format: str, *args) -> None:  # noqa: A002
        print(f"[送信ページ] {time.strftime('%H:%M:%S')} {self.address_string()} {format % args}", flush=True)

    # ---- MediaMTX への中継 ----

    def _proxy(self) -> None:
        path = urllib.parse.urlsplit(self.path).path
        if not PROXY_PATH.match(path):
            self.send_error(404)
            return
        n = int(self.headers.get("Content-Length", 0) or 0)
        if n > MAX_WHIP_BODY:
            self.send_error(413)
            return
        body = self.rfile.read(n) if n else None
        headers = {k: self.headers[k] for k in PASS_REQ if self.headers.get(k)}
        headers["X-Forwarded-For"] = self.client_address[0]
        ctx = ssl._create_unverified_context()  # 相手は同じ機械の MediaMTX（証明書は IP 入りの自己署名）
        conn = http.client.HTTPSConnection(self.mtx.hostname, self.mtx.port, context=ctx, timeout=MTX_TIMEOUT_S)
        try:
            conn.request(self.command, self.path, body=body, headers=headers)
            resp = conn.getresponse()
            data = resp.read()
        except OSError as e:
            print(f"[送信ページ] 中継に失敗: {self.command} {self.path}: {e}", flush=True)
            self.send_error(502)
            return
        finally:
            conn.close()
        self.send_response(resp.status)
        for k in PASS_RESP:
            v = resp.getheader(k)
            if v is not None and k != "Content-Length":
                self.send_header(k, v)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(data)

    def do_GET(self) -> None:  # noqa: N802
        if urllib.parse.urlsplit(self.path).path.endswith("/publisher.js"):
            self._proxy()
        else:
            super().do_GET()

    def do_OPTIONS(self) -> None:  # noqa: N802
        self._proxy()

    def do_PATCH(self) -> None:  # noqa: N802
        self._proxy()

    def do_DELETE(self) -> None:  # noqa: N802
        self._proxy()

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/log":
            self._proxy()
            return
        n = min(int(self.headers.get("Content-Length", 0) or 0), MAX_LOG_BODY)
        body = self.rfile.read(n).decode("utf-8", "replace")
        try:
            d = json.loads(body)
            text = f"{d.get('cam', '?')} {d.get('event', '?')}: {d.get('detail', '')}"
        except json.JSONDecodeError:
            text = body[:200]
        print(f"[スマホ] {time.strftime('%H:%M:%S')} {self.client_address[0]} {text}", flush=True)
        self.send_response(204)
        self.end_headers()


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--cert", required=True)
    p.add_argument("--key", required=True)
    p.add_argument("--port", type=int, default=8443)
    p.add_argument("--dir", default=str(HERE / "publish"))
    p.add_argument("--mtx", default="https://127.0.0.1:8889", help="中継先の MediaMTX の WebRTC の口")
    args = p.parse_args()

    PublishHandler.mtx = urllib.parse.urlsplit(args.mtx)
    handler = functools.partial(PublishHandler, directory=args.dir)
    server = http.server.ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(args.cert, args.key)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)
    print(f"[送信ページ] https://0.0.0.0:{args.port}/ で {args.dir} を配り、送信を {args.mtx} に中継する", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
