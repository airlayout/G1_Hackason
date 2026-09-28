"""深度付きカメラサーバ。G1 の PC2 で RealSense を直接読み、ZMQ で配信する。タスク3。

    # PC2 で（使い方と準備は real/depth_server/README.md）
    python Button_Press/J1-gen/real/depth_server/rgbd_server.py
    python Button_Press/J1-gen/real/depth_server/rgbd_server.py --list-devices
    python Button_Press/J1-gen/real/depth_server/rgbd_server.py --source dummy     # RealSense 無しで確認

配信するもの（ポートは configs/depth_server.yaml で変えられる）:
- 5556: 深度付き（カラー JPEG + カラーに位置合わせした 16bit 深度 + 内部パラメータ）。形式は common/rgbd_protocol.py
- 5555: RGB 互換（run_g1_server.py --camera と同じ形式）。既存の ZmqFrameSource がそのまま動く

⚠️ RealSense は 1 つのプログラムしか開けない。このサーバを使うときは、run_g1_server.py を
   **--camera なしで**起動すること（lowcmd / lowstate の中継はそのまま使える）。

run_g1_server.py（lerobot 側のファイル）は変更していない。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path
from types import FrameType
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common.config import load_config  # noqa: E402
from common.rgbd_protocol import Intrinsics, RgbdFrame, encode_legacy_rgb, encode_rgbd  # noqa: E402


class CapturedFrame:
    """カメラから取った 1 フレーム（RGB の並びのカラー、カラーに位置合わせした深度）。"""

    def __init__(self, color_rgb: np.ndarray, depth: np.ndarray, timestamp: float) -> None:
        self.color_rgb = color_rgb
        self.depth = depth
        self.timestamp = timestamp


class RealSenseCamera:
    """pyrealsense2 で RealSense を読む。深度はカラーに位置合わせする（rs.align）。"""

    def __init__(self, cam_cfg: dict[str, Any]) -> None:
        import pyrealsense2 as rs

        self._rs = rs
        self.cfg = cam_cfg
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.depth_scale = 0.001
        self.intrinsics: Intrinsics | None = None

    @staticmethod
    def list_devices() -> list[tuple[str, str]]:
        import pyrealsense2 as rs

        out = []
        for dev in rs.context().query_devices():
            out.append((dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.serial_number)))
        return out

    def start(self) -> None:
        rs, c = self._rs, self.cfg
        config = rs.config()
        serial = str(c.get("serial") or "")
        if serial:
            config.enable_device(serial)
        w, h, fps = int(c["width"]), int(c["height"]), int(c["fps"])
        # カラーは RGB の並びで受け取る（互換ストリームは lerobot と同じく RGB の配列をそのまま JPEG にする）
        config.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
        config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        profile = self.pipeline.start(config)
        dev = profile.get_device()
        self.depth_scale = float(dev.first_depth_sensor().get_depth_scale())
        vp = profile.get_stream(rs.stream.color).as_video_stream_profile()
        i = vp.get_intrinsics()
        self.intrinsics = Intrinsics(
            width=i.width, height=i.height, fx=i.fx, fy=i.fy, cx=i.ppx, cy=i.ppy,
            model=str(i.model), coeffs=[float(x) for x in i.coeffs],
        )
        print(f"[rgbd_server] RealSense 開始: {dev.get_info(rs.camera_info.name)} "
              f"（シリアル {dev.get_info(rs.camera_info.serial_number)}）{w}x{h}@{fps}、"
              f"深度の単位 {self.depth_scale} m")
        print(f"[rgbd_server] 内部パラメータ（カラー）: fx={i.fx:.1f} fy={i.fy:.1f} cx={i.ppx:.1f} cy={i.ppy:.1f}")

    def read(self) -> CapturedFrame | None:
        frames = self.pipeline.wait_for_frames(int(self.cfg.get("timeout_ms", 5000)))
        aligned = self.align.process(frames)
        color = aligned.get_color_frame()
        depth = aligned.get_depth_frame()
        if not color or not depth:
            return None
        return CapturedFrame(
            color_rgb=np.asanyarray(color.get_data()).copy(),
            depth=np.asanyarray(depth.get_data()).astype(np.uint16, copy=True),
            timestamp=time.time(),
        )

    def stop(self) -> None:
        self.pipeline.stop()


class DummyCamera:
    """RealSense が無いマシンでの確認用。灰色の机（1.0 m）の上に、赤い箱（0.6 m）がある作り物の画像と深度。

    左上の 20x20 画素は深度 0（測れなかった画素）にしてある。
    """

    def __init__(self, cam_cfg: dict[str, Any]) -> None:
        self.cfg = cam_cfg
        w, h = int(cam_cfg["width"]), int(cam_cfg["height"])
        self.depth_scale = 0.001
        # D435 のカラー 640x480 に近い値
        f = 615.0 * w / 640.0
        self.intrinsics = Intrinsics(width=w, height=h, fx=f, fy=f, cx=w / 2 - 0.5, cy=h / 2 - 0.5)
        self._period = 1.0 / float(cam_cfg["fps"])
        self._next = 0.0

    def start(self) -> None:
        print("[rgbd_server] ダミーカメラ開始（RealSense は使わない）")

    def read(self) -> CapturedFrame:
        now = time.monotonic()
        if self._next > now:
            time.sleep(self._next - now)
        self._next = max(now, self._next) + self._period
        w, h = self.intrinsics.width, self.intrinsics.height
        color = np.full((h, w, 3), 128, np.uint8)  # RGB
        depth = np.full((h, w), 1000, np.uint16)
        y0, y1, x0, x1 = h // 3, 2 * h // 3, w // 3, w // 2
        color[y0:y1, x0:x1] = (255, 0, 0)  # 赤（RGB の並び）
        depth[y0:y1, x0:x1] = 600
        depth[:20, :20] = 0
        return CapturedFrame(color_rgb=color, depth=depth, timestamp=time.time())

    def stop(self) -> None:
        pass


def make_camera(cam_cfg: dict[str, Any]) -> RealSenseCamera | DummyCamera:
    src = cam_cfg["source"]
    if src == "realsense":
        return RealSenseCamera(cam_cfg)
    if src == "dummy":
        return DummyCamera(cam_cfg)
    raise ValueError(f"camera.source は realsense か dummy: {src}")


class RgbdServer:
    def __init__(self, cfg: dict[str, Any]) -> None:
        import zmq

        self.cfg = cfg
        pub = cfg["publish"]
        self.name = pub["camera_name"]
        self.ctx = zmq.Context()
        self.sockets: dict[str, Any] = {}
        for key in ("rgbd", "legacy_rgb"):
            if pub[key]["enabled"]:
                s = self.ctx.socket(zmq.PUB)
                s.setsockopt(zmq.SNDHWM, 2)
                s.setsockopt(zmq.LINGER, 0)
                s.bind(f"tcp://{pub['bind_address']}:{int(pub[key]['port'])}")
                self.sockets[key] = s
        if not self.sockets:
            raise ValueError("publish.rgbd と publish.legacy_rgb の両方が無効になっている")
        self.camera = make_camera(cfg["camera"])
        self._stop = False

    def request_stop(self, signum: int = 0, frame: FrameType | None = None) -> None:
        self._stop = True

    def run(self, max_frames: int | None = None) -> int:
        import zmq

        pub = self.cfg["publish"]
        self.camera.start()
        for key in self.sockets:
            print(f"[rgbd_server] 配信: {key} → ポート {pub[key]['port']}")
        max_fps = float(pub.get("max_fps", 0) or 0)
        min_interval = 1.0 / max_fps if max_fps > 0 else 0.0
        print(f"[rgbd_server] 配信の上限: {f'{max_fps:g} fps' if max_fps > 0 else 'なし'}")
        n = 0  # 送ったフレーム数
        t_next_pub: float | None = None  # 次に送る予定の時刻
        # カメラのフレームは 1/camera.fps ごとに届くので、予定の時刻より周期の半分以内の早さなら送る
        # （これが無いと、予定の直前に届いたフレームを見送って 1 周期待つことが続き、平均が上限より下がる）
        slack = 0.5 / float(self.cfg["camera"]["fps"])
        t_log = time.monotonic()
        n_log = 0
        try:
            while not self._stop and (max_frames is None or n < max_frames):
                # カメラは読み続ける（読まないと古いフレームが溜まる）。送る回数だけを間引く
                f = self.camera.read()
                if f is None:
                    continue
                now = time.monotonic()
                if t_next_pub is not None and now < t_next_pub - slack:
                    continue
                # 予定の時刻は積み上げる（平均を上限に合わせるため）。1 周期以上遅れたら今を基準に戻す
                if t_next_pub is None or now > t_next_pub + min_interval:
                    t_next_pub = now
                t_next_pub += min_interval
                n += 1
                if "rgbd" in self.sockets:
                    msg = encode_rgbd(
                        RgbdFrame(
                            color_bgr=f.color_rgb[:, :, ::-1],  # RGBD の形式は BGR で送る
                            depth=f.depth, depth_scale=self.camera.depth_scale,
                            intrinsics=self.camera.intrinsics,  # type: ignore[arg-type]
                            timestamp=f.timestamp, frame_id=n, camera=self.name,
                        ),
                        jpeg_quality=int(pub["rgbd"]["jpeg_quality"]),
                        depth_compression=str(pub["rgbd"]["depth_compression"]),
                    )
                    try:
                        self.sockets["rgbd"].send(msg, zmq.NOBLOCK)
                    except zmq.Again:
                        pass
                if "legacy_rgb" in self.sockets:
                    s = encode_legacy_rgb(f.color_rgb, self.name, f.timestamp, int(pub["legacy_rgb"]["jpeg_quality"]))
                    try:
                        self.sockets["legacy_rgb"].send_string(s, zmq.NOBLOCK)
                    except zmq.Again:
                        pass
                n_log += 1
                if time.monotonic() - t_log >= 5.0:
                    fps = n_log / (time.monotonic() - t_log)
                    print(f"[rgbd_server] {n} フレーム配信（直近 {fps:.1f} fps）")
                    t_log, n_log = time.monotonic(), 0
        finally:
            self.camera.stop()
            for s in self.sockets.values():
                s.close()
            self.ctx.term()
            print(f"[rgbd_server] 終了（{n} フレーム）")
        return n


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default="depth_server.yaml", help="設定ファイル（configs/ 基準）")
    p.add_argument("--source", choices=["realsense", "dummy"], help="カメラ（既定は設定ファイル）")
    p.add_argument("--serial", help="RealSense のシリアル番号（既定は設定ファイル）")
    p.add_argument("--rgbd-port", type=int, help="深度付きストリームのポート（既定は設定ファイル）")
    p.add_argument("--rgb-port", type=int, help="RGB 互換ストリームのポート（既定は設定ファイル）")
    p.add_argument("--no-legacy-rgb", action="store_true", help="RGB 互換ストリームを出さない")
    p.add_argument("--max-fps", type=float, help="配信の上限 [fps]。0 なら間引かない（既定は設定ファイル）")
    p.add_argument("--list-devices", action="store_true", help="つながっている RealSense を表示して終わる")
    p.add_argument("--max-frames", type=int, help="このフレーム数を送ったら終わる（確認用）")
    args = p.parse_args()

    if args.list_devices:
        devs = RealSenseCamera.list_devices()
        if not devs:
            print("[rgbd_server] RealSense が見つからない（lsusb で USB に見えているか確認する）")
            return 1
        for name, serial in devs:
            print(f"[rgbd_server] {name}  シリアル {serial}")
        return 0

    cfg = load_config(args.config)
    if args.source:
        cfg["camera"]["source"] = args.source
    if args.serial is not None:
        cfg["camera"]["serial"] = args.serial
    if args.rgbd_port:
        cfg["publish"]["rgbd"]["port"] = args.rgbd_port
    if args.rgb_port:
        cfg["publish"]["legacy_rgb"]["port"] = args.rgb_port
    if args.no_legacy_rgb:
        cfg["publish"]["legacy_rgb"]["enabled"] = False
    if args.max_fps is not None:
        cfg["publish"]["max_fps"] = args.max_fps

    server = RgbdServer(cfg)
    signal.signal(signal.SIGINT, server.request_stop)
    signal.signal(signal.SIGTERM, server.request_stop)
    server.run(max_frames=args.max_frames)
    return 0


if __name__ == "__main__":
    sys.exit(main())
