"""認識した映像を HTTP の MJPEG でブラウザへ配信する(Python の標準ライブラリだけ。G1 の Python 3.8 でも動く)。

real/person_alert.py --stream <port> が使う。ブラウザで http://<G1のアドレス>:<port>/ を開くと見える。
  /             映像を 1 枚表示するだけのページ
  /stream       MJPEG(multipart/x-mixed-replace)。<img src> や VLC でも開ける
  /snapshot.jpg 最新の 1 枚
見ている人がいないときは publish() を呼ばなくてよい(viewers で分かる)。JPEG を作る手間(CPU)を省くため。
"""
from __future__ import annotations

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_BOUNDARY = "frame"
_PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>G1 カメラ</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{margin:0;background:#111;color:#ccc;font-family:sans-serif;text-align:center}
img{max-width:100%;max-height:92vh}</style></head>
<body><img src="/stream" alt="G1 の映像(届かなければ person_alert.py が動いているかを確かめる)">
<div>G1 の Web カメラ(YOLO の人の検出つき)</div></body></html>"""


class MjpegServer:
    """最新の JPEG を 1 枚だけ持ち、つないでいる全員に配る。遅い相手には途中の枚数を飛ばして最新を送る。"""

    def __init__(self, port: int, host: str = "0.0.0.0") -> None:
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._viewers = 0
        self._closed = False
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args) -> None:  # 1 枚ごとのアクセスの記録は出さない
                pass

            def do_GET(self) -> None:
                path = self.path.split("?", 1)[0]
                if path == "/":
                    self._send(200, "text/html; charset=utf-8", _PAGE.encode("utf-8"))
                elif path == "/snapshot.jpg":
                    jpeg = server.latest()
                    if jpeg is None:
                        self._send(503, "text/plain; charset=utf-8", "まだ映像が無い(誰かが /stream を開くと作り始める)".encode("utf-8"))
                    else:
                        self._send(200, "image/jpeg", jpeg)
                elif path == "/stream":
                    self._stream()
                else:
                    self._send(404, "text/plain; charset=utf-8", b"not found")

            def _send(self, code: int, ctype: str, body: bytes) -> None:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _stream(self) -> None:
                self.send_response(200)
                self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={_BOUNDARY}")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                server._add_viewer(1)
                try:
                    seq = -1
                    while True:
                        got = server.wait_next(seq, timeout=5.0)
                        if got is None:
                            if server.closed:
                                return
                            continue
                        seq, jpeg = got
                        self.wfile.write(f"--{_BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n"
                                         .encode("ascii") + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass  # 相手がページを閉じた
                finally:
                    server._add_viewer(-1)

        self._httpd = ThreadingHTTPServer((host, port), Handler)
        self._httpd.daemon_threads = True
        self.port = self._httpd.server_address[1]  # port=0 なら空いている番号が入る(テスト用)
        self._thread = threading.Thread(target=self._httpd.serve_forever, name="mjpeg", daemon=True)
        self._thread.start()

    @property
    def viewers(self) -> int:
        return self._viewers

    @property
    def closed(self) -> bool:
        return self._closed

    def _add_viewer(self, n: int) -> None:
        with self._cond:
            self._viewers += n

    def publish(self, jpeg: bytes) -> None:
        """最新の 1 枚を差し替え、待っている相手に知らせる。"""
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def latest(self) -> bytes | None:
        with self._cond:
            return self._jpeg

    def wait_next(self, last_seq: int, timeout: float) -> tuple[int, bytes] | None:
        """last_seq より新しい 1 枚が来るまで待つ。timeout 秒来なければ(または閉じたら) None。"""
        with self._cond:
            if not self._cond.wait_for(lambda: self._closed or (self._jpeg is not None and self._seq != last_seq),
                                       timeout=timeout):
                return None
            if self._closed or self._jpeg is None:
                return None
            return self._seq, self._jpeg

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()
        self._httpd.shutdown()
        self._httpd.server_close()
