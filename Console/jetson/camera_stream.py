#!/usr/bin/env python3
"""G1 のカメラを MJPEG（HTTP）で配信する。**ロボット本体（Jetson）で動かす。**

  python3 camera_stream.py --list
  python3 camera_stream.py --camera std=/dev/v4l/by-id/usb-SunplusIT_..._webcam_...-video-index0 \
                           --camera d435i=videoclient:eth0   # 公式経路（videohub を止めずに RGB）

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
import numpy as np

DEFAULT_PORT = 8081
FRAME_W, FRAME_H, FPS = 640, 480, 15
JPEG_QUALITY = 80
BOUNDARY = "frame"
REOPEN_WAIT_S = 2.0
DEPTH_MAX_MM = 4000  # 深度の表示上限（これ以上は同色）。Z16 の 1 単位は 1mm


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


class DepthGrabber(Grabber):
    """D435i の深度（Z16, 1 単位=1mm）を pyrealsense2 で読み、RGB（1920x1080）の視点に位置合わせした
    カラーマップの JPEG にする。深度は USB インターフェース 1.0 で、RGB を握る videohub_pc4（1.3）とは別。
    位置合わせはカメラ内部・外部パラメータ（プロファイル情報。カラーは流さない）で 3 次元に戻して投影する。
    0（測距不能）と RGB の視野外は黒、近いほど赤、遠いほど青。受信のみ。"""

    WIDTH, HEIGHT, DEPTH_FPS, WAIT_MS = 640, 480, 15, 5000
    COLOR_W, COLOR_H, OUT_SCALE = 1920, 1080, 0.5  # 出力は RGB を縮小した 960x540（同じ 16:9）

    def _calibration(self, rs):
        """(深度の光線格子 xn,yn, 回転 R, 並進 t, カラーの fx,fy,ppx,ppy) を返す。"""
        dev = rs.context().query_devices()[0]
        color = next(s for s in dev.query_sensors() if s.get_info(rs.camera_info.name) == "RGB Camera")
        cp = next(p for p in color.get_stream_profiles() if p.stream_type() == rs.stream.color
                  and p.format() == rs.format.rgb8 and p.as_video_stream_profile().width() == self.COLOR_W
                  and p.as_video_stream_profile().height() == self.COLOR_H).as_video_stream_profile()
        dp = next(p for p in dev.first_depth_sensor().get_stream_profiles() if p.stream_type() == rs.stream.depth
                  and p.format() == rs.format.z16 and p.as_video_stream_profile().width() == self.WIDTH
                  and p.as_video_stream_profile().height() == self.HEIGHT).as_video_stream_profile()
        di, ci, ex = dp.get_intrinsics(), cp.get_intrinsics(), dp.get_extrinsics_to(cp)
        us, vs = np.meshgrid(np.arange(self.WIDTH), np.arange(self.HEIGHT))
        xn, yn = (us - di.ppx) / di.fx, (vs - di.ppy) / di.fy
        rot = np.array(ex.rotation, dtype=np.float64).reshape(3, 3).T  # librealsense は列優先
        return xn, yn, rot, np.array(ex.translation, dtype=np.float64), (ci.fx, ci.fy, ci.ppx, ci.ppy)

    def _align(self, depth, calib):
        """深度（mm, 480x640）を RGB 視点の奥行き(m, 540x960。無効は 0)にする。近い点を優先して 2x2 で埋める。"""
        xn, yn, rot, trans, (fx, fy, ppx, ppy) = calib
        z = depth.astype(np.float64) * 0.001
        ok = z > 0
        z, x, y = z[ok], xn[ok] * z[ok], yn[ok] * z[ok]
        zc = rot[2, 0] * x + rot[2, 1] * y + rot[2, 2] * z + trans[2]
        u = (fx * (rot[0, 0] * x + rot[0, 1] * y + rot[0, 2] * z + trans[0]) / zc + ppx) * self.OUT_SCALE
        v = (fy * (rot[1, 0] * x + rot[1, 1] * y + rot[1, 2] * z + trans[1]) / zc + ppy) * self.OUT_SCALE
        w, h = int(self.COLOR_W * self.OUT_SCALE), int(self.COLOR_H * self.OUT_SCALE)
        u, v = np.floor(u).astype(np.int32), np.floor(v).astype(np.int32)
        order = np.argsort(-zc)  # 遠い順に書く＝最後に書いた近い点が残る
        out = np.full((h, w), np.inf)
        for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)):
            uu, vv = u[order] + du, v[order] + dv
            inside = (uu >= 0) & (uu < w) & (vv >= 0) & (vv < h)
            layer = np.full((h, w), np.inf)
            layer[vv[inside], uu[inside]] = zc[order][inside]
            out = np.minimum(out, layer)
        out[~np.isfinite(out)] = 0
        return out

    def run(self):
        import pyrealsense2 as rs
        calib = self._calibration(rs)
        while True:
            pipe = rs.pipeline()
            try:
                cfg = rs.config()
                cfg.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, self.DEPTH_FPS)
                pipe.start(cfg)
                while True:
                    depth = np.asanyarray(pipe.wait_for_frames(self.WAIT_MS).get_depth_frame().get_data())
                    aligned_mm = self._align(depth, calib) * 1000.0
                    scaled = (255 - np.clip(aligned_mm, 0, DEPTH_MAX_MM) * 255 / DEPTH_MAX_MM).astype(np.uint8)
                    color = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
                    color[aligned_mm == 0] = 0
                    ok, enc = cv2.imencode(".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                    if ok:
                        with self._lock:
                            self._jpeg, self._seq = enc.tobytes(), self._seq + 1
            except RuntimeError as exc:  # 抜けた・タイムアウトなど。開き直す
                print("[camera] %s: 深度を読めません (%s)" % (self.name, exc), flush=True)
            try:
                pipe.stop()
            except RuntimeError:
                pass
            with self._lock:
                self._jpeg = None
            time.sleep(REOPEN_WAIT_S)


class VideoClientGrabber(Grabber):
    """公式経路 VideoClient.GetImageSample()（受信専用）。videohub_pc4 が /dev/video4 を握っていても取れる。
    JPEG をデコードせずそのまま保持する。ロボットへの指令は一切送らない。"""

    ERROR_WAIT_S = 1.0

    def __init__(self, name: str, interface: str):
        super().__init__(name, "videoclient:" + interface)
        self.interface = interface

    def run(self):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_client import VideoClient
        ChannelFactoryInitialize(0, self.interface)
        client = VideoClient()
        client.SetTimeout(3.0)
        client.Init()
        while True:
            code, data = client.GetImageSample()
            if code == 0 and data:
                with self._lock:
                    self._jpeg, self._seq = bytes(data), self._seq + 1
                continue
            print("[camera] %s: GetImageSample 失敗 code=%s" % (self.name, code), flush=True)
            with self._lock:
                self._jpeg = None
            time.sleep(self.ERROR_WAIT_S)


def make_grabber(name: str, device: str):
    """device が videoclient:<NIC> なら公式経路、それ以外は V4L2 デバイス。"""
    if device.startswith("videoclient:"):
        return VideoClientGrabber(name, device.split(":", 1)[1])
    if device.startswith("depth:"):
        return DepthGrabber(name, device)
    return Grabber(name, device)


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
        grabbers[name] = make_grabber(name, device)
    if not grabbers:
        parser.error("--camera を 1 つ以上指定する（--list で候補を確認）")
    for grabber in grabbers.values():
        grabber.start()
    print("[camera] :%d で配信: %s" % (args.port, ", ".join(grabbers)), flush=True)
    ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(grabbers)).serve_forever()


if __name__ == "__main__":
    main()
