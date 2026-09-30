#!/usr/bin/env python3
"""G1 のカメラを MJPEG（HTTP）で配信する。**ロボット本体（Jetson）で動かす。**

  python3 camera_stream.py --list
  python3 camera_stream.py --camera std=/dev/v4l/by-id/usb-SunplusIT_..._webcam_...-video-index0 \
                           --camera d435i=/dev/v4l/by-id/usb-Intel_R__RealSense_..._435i_...-video-index0

  http://<Jetson>:8081/<名前>   ← Console の server.py が中継する（/camera/<名前>）

⚠️ モーターには触らない（カメラを読んで流すだけ）。認証なしなので、信頼できる網でだけ使う。
⚠️ /dev/videoN の番号ではなく by-id の安定名で指定する（再認識で番号が変わるため）。
Jetson の Python 3.8 で動くよう、標準ライブラリ + cv2 のみ。
"""
import argparse
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

DEFAULT_PORT = 8081
FRAME_W, FRAME_H, FPS = 640, 480, 15
JPEG_QUALITY = 80
BOUNDARY = "frame"
REOPEN_WAIT_S = 2.0


class Grabber(threading.Thread):
    """1 台のカメラを読み続け、最新の JPEG だけ保持する。切断されたら開き直す。"""

    def __init__(self, name: str, device: str):
        super().__init__(daemon=True)
        self.name, self.device = name, device
        self._lock = threading.Lock()
        self._jpeg = None
        self._seq = 0

    def latest(self):
        with self._lock:
            return self._seq, self._jpeg

    def run(self):
        while True:
            # OpenCV は by-id の名前では開けない（"can't be used to capture by name"）。
            # 開く直前に実体（/dev/videoN）へ解決して番号で開く（再認識で番号が変わっても追従）。
            real = os.path.realpath(self.device)
            cap = cv2.VideoCapture(int(real.replace("/dev/video", "")), cv2.CAP_V4L2) \
                if real.startswith("/dev/video") else cv2.VideoCapture(real, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
            cap.set(cv2.CAP_PROP_FPS, FPS)
            if not cap.isOpened():
                print("[camera] %s: 開けません (%s)" % (self.name, self.device), flush=True)
            while cap.isOpened():
                ok, frame = cap.read()
                if not ok or frame is None:
                    break
                ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if ok:
                    with self._lock:
                        self._jpeg, self._seq = enc.tobytes(), self._seq + 1
            cap.release()
            with self._lock:
                self._jpeg = None  # 古い映像を出し続けない
            time.sleep(REOPEN_WAIT_S)


def make_handler(grabbers):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            grabber = grabbers.get(self.path.strip("/"))
            if grabber is None:
                self.send_error(404, "unknown camera")
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=" + BOUNDARY)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            try:
                while True:
                    seq, jpeg = grabber.latest()
                    if jpeg is None or seq == last:
                        time.sleep(1.0 / (FPS * 2))
                        continue
                    last = seq
                    self.wfile.write(("--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                      % (BOUNDARY, len(jpeg))).encode() + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # 見ている側が閉じただけ

        def log_message(self, fmt, *args):
            pass

    return Handler


def list_devices():
    base = "/dev/v4l/by-id"
    for name in sorted(os.listdir(base)) if os.path.isdir(base) else []:
        print("%s -> %s" % (name, os.path.realpath(os.path.join(base, name))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--camera", action="append", default=[], metavar="名前=デバイス")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()
    if args.list:
        return list_devices()
    grabbers = {}
    for spec in args.camera:
        name, _, device = spec.partition("=")
        if not name or not device:
            parser.error("--camera は 名前=デバイス の形で指定する: %r" % spec)
        grabbers[name] = Grabber(name, device)
    if not grabbers:
        parser.error("--camera を 1 つ以上指定する（--list で候補を確認）")
    for grabber in grabbers.values():
        grabber.start()
    print("[camera] :%d で配信: %s" % (args.port, ", ".join(grabbers)), flush=True)
    ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(grabbers)).serve_forever()


if __name__ == "__main__":
    main()
