#!/usr/bin/env python3
"""G1 の頭カメラ（IR）をブラウザへ流す。タグを貼りながら見るための道具。

## なぜ MJPEG なのか

PC2 に GUI は無く、Mac からは SSH しか通っていない。RViz2 をコンテナで動かすと
**MOLA を痩せさせる**（2026-09-11 実測: 107 % CPU）。ブラウザで見るだけなら
JPEG を HTTP で投げるのがいちばん軽く、追加のインストールも要らない（標準ライブラリ）。

## 使い方

    # PC2 で起動（Ctrl-C で止まる）
    python3 ~/g1_cfg/apriltag/serve_view.py

    # Mac のブラウザで開く
    open http://10.42.0.76:8080/

タグを検出すると枠と ID・距離を描く。貼る位置を動かしながら見られる。

## 負荷

`--annotate`（既定）は検出まで回すので 1280x720 で 11 fps ほど。
`--fast` で角の精密化を切ると 28 fps 出る（⚠️ 姿勢の精度は落ちる）。
`--no-annotate` なら検出そのものをやめる。

⚠️ **認証は無い。** 機体が繋がっている網の誰からでも見える。
研究室の中で使う前提。使い終わったら止めること。
"""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import tag_detect
from see_tags import IR_DEVICE, open_ir

BOUNDARY = "g1frame"
PAGE = """<!doctype html><meta charset="utf-8">
<title>G1 head camera (IR)</title>
<style>
 body{{margin:0;background:#111;color:#ddd;font:14px system-ui,sans-serif}}
 header{{padding:8px 12px;display:flex;gap:16px;align-items:baseline}}
 h1{{font-size:15px;margin:0;font-weight:600}}
 span{{color:#888}}
 img{{display:block;width:100%;height:auto;background:#000}}
</style>
<header><h1>G1 head camera — /dev/video2 (IR)</h1>
<span>{width}x{height} / {mode}</span></header>
<img src="/stream.mjpg" alt="live">
"""


class Camera:
    """取り込みと検出を 1 本のスレッドで回し、最新の JPEG だけを持つ。"""

    def __init__(self, arguments) -> None:
        self.arguments = arguments
        self.jpeg = None
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.fps = 0.0
        self.detections = 0
        self.detector = (tag_detect.make_detector(arguments.family,
                                                  refine=not arguments.fast)
                         if arguments.annotate else None)
        self.camera_matrix = tag_detect.default_camera_matrix(
            arguments.width, arguments.height, arguments.hfov_deg)
        self.distortion = np.zeros((1, 5))
        if arguments.intrinsics:
            self.camera_matrix, self.distortion, _ = tag_detect.load_intrinsics(
                arguments.intrinsics)

    def run(self) -> None:
        capture = open_ir(self.arguments.device, self.arguments.width,
                          self.arguments.height, self.arguments.fps)
        tag_m = self.arguments.tag_mm / 1000.0
        count, since = 0, time.time()
        try:
            while not self.stopped.is_set():
                ok, frame = capture.read()
                if not ok:
                    continue
                gray = tag_detect.to_gray(frame)
                if self.detector is not None:
                    found = tag_detect.detect(self.detector, gray)
                    for item in found:
                        try:
                            tag_detect.solve_pose(item, tag_m, self.camera_matrix,
                                                  self.distortion)
                        except RuntimeError:
                            pass
                    canvas = tag_detect.annotate(gray, found)
                    self.detections = len(found)
                else:
                    canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                count += 1
                elapsed = time.time() - since
                if elapsed >= 1.0:
                    self.fps, count, since = count / elapsed, 0, time.time()
                cv2.putText(canvas, "%.1f fps  tags:%d" % (self.fps, self.detections),
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 255), 2, cv2.LINE_AA)
                ok, buffer = cv2.imencode(".jpg", canvas,
                                          [int(cv2.IMWRITE_JPEG_QUALITY),
                                           self.arguments.quality])
                if ok:
                    with self.lock:
                        self.jpeg = buffer.tobytes()
        finally:
            capture.release()

    def latest(self):
        with self.lock:
            return self.jpeg


def make_handler(camera: Camera, arguments):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def log_message(self, *_):  # アクセスログは出さない
            pass

        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
                mode = ("検出あり" if arguments.annotate else "検出なし") + \
                       ("・精密化なし" if arguments.fast else "")
                body = PAGE.format(width=arguments.width, height=arguments.height,
                                   mode=mode).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/snapshot.jpg":
                frame = camera.latest()
                if frame is None:
                    self.send_error(503, "no frame yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type",
                             f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = None
            try:
                while not camera.stopped.is_set():
                    frame = camera.latest()
                    if frame is None or frame is last:
                        time.sleep(0.005)
                        continue
                    last = frame
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # ブラウザを閉じただけ
    return Handler


def local_addresses() -> list:
    """開くべき URL を当てるため、この機体が持つ IPv4 を全部並べる。

    ⚠️ UDP connect でローカル側アドレスを見る手は**既定経路の 1 つしか出ない**。
    G1 は有線（192.168.123.x）と無線（10.42.0.x）を同時に持つので、両方出す。
    """

    addresses = []
    try:
        output = subprocess.run(["ip", "-4", "-o", "addr", "show"],
                                capture_output=True, text=True, timeout=5).stdout
        for line in output.splitlines():
            fields = line.split()
            if len(fields) >= 4 and fields[2] == "inet":
                address = fields[3].split("/")[0]
                if not address.startswith("127."):
                    addresses.append(address)
    except (OSError, subprocess.SubprocessError):
        pass
    if not addresses:                       # `ip` が無い環境向けの保険
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("192.168.123.1", 1))
            addresses.append(probe.getsockname()[0])
        except OSError:
            pass
        finally:
            probe.close()
    return sorted(set(addresses))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default=IR_DEVICE)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--quality", type=int, default=75)
    parser.add_argument("--family", default="36h11", choices=sorted(tag_detect.FAMILIES))
    parser.add_argument("--tag-mm", type=float, default=160.0)
    parser.add_argument("--intrinsics", default=None)
    parser.add_argument("--hfov-deg", type=float, default=87.0)
    parser.add_argument("--fast", action="store_true", help="角の精密化を切る（速い）")
    parser.add_argument("--no-annotate", dest="annotate", action="store_false",
                        help="検出そのものをやめる（いちばん軽い）")
    arguments = parser.parse_args()

    camera = Camera(arguments)
    worker = threading.Thread(target=camera.run, daemon=True)
    worker.start()
    for _ in range(100):           # 最初の 1 枚が出るまで待つ
        if camera.latest() is not None:
            break
        time.sleep(0.05)
    if camera.latest() is None:
        print("⛔ カメラから 1 枚も取れない。他のプロセスが握っていないか見る")
        return 1

    server = ThreadingHTTPServer((arguments.bind, arguments.port),
                                 make_handler(camera, arguments))
    # ⚠️ リダイレクト先ではブロックバッファになる。flush しないとログに出ない。
    for address in local_addresses():
        print(f"  http://{address}:{arguments.port}/", flush=True)
    print("止めるときは Ctrl-C", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n止めます")
    finally:
        camera.stopped.set()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
