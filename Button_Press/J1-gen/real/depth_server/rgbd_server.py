"""深度付きカメラサーバ。G1 の PC2 で RealSense を直接読み、ZMQ で配信する。タスク3。

    # PC2 で（使い方と準備は real/depth_server/README.md）
    python Button_Press/J1-gen/real/depth_server/rgbd_server.py
    python Button_Press/J1-gen/real/depth_server/rgbd_server.py --list-devices
    python Button_Press/J1-gen/real/depth_server/rgbd_server.py --source dummy     # RealSense 無しで確認
    python Button_Press/J1-gen/real/depth_server/rgbd_server.py --source videohub  # カラーは videohub から

配信するもの（ポートは configs/depth_server.yaml で変えられる）:
- 5556: 深度付き（カラー JPEG + カラーに位置合わせした 16bit 深度 + 内部パラメータ）。形式は common/rgbd_protocol.py
- 5555: RGB 互換（run_g1_server.py --camera と同じ形式）。既存の ZmqFrameSource がそのまま動く

⚠️ RealSense は 1 つのプログラムしか開けない。このサーバを使うときは、run_g1_server.py を
   **--camera なしで**起動すること（lowcmd / lowstate の中継はそのまま使える）。
   Unitree の videohub_pc4 がカラー（/dev/video4）を開いているときは --source videohub を使う
   （カラーは videohub に頼んで受け取り、深度だけを RealSense から開く。videohub は止めない）。

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

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common.config import load_config  # noqa: E402
from common.depth_align import Extrinsics, align_depth_to_color, scale_intrinsics  # noqa: E402
from common.rgbd_protocol import Intrinsics, RgbdFrame, encode_legacy_rgb, encode_rgbd  # noqa: E402


class CapturedFrame:
    """カメラから取った 1 フレーム（RGB の並びのカラー、カラーに位置合わせした深度）。

    VideohubCamera の read() は深度だけを入れて返す（color_rgb は None）。送ると決めたフレームだけ
    complete() でカラーを受け取って位置合わせする（送らないフレームで計算しないため）。
    """

    def __init__(self, color_rgb: np.ndarray | None, depth: np.ndarray, timestamp: float) -> None:
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


class VideohubCamera:
    """カラーは Unitree の videohub から、深度は RealSense から直接受け取り、深度をカラーに位置合わせする。

    PC2 では videohub_pc4 がカラー（/dev/video4）を開いているので、RealSenseCamera は起動できない
    （Device or resource busy）。深度（/dev/video0）は USB の別の部分なので、深度だけなら開ける。
    カラーは videohub に 1 枚ずつ頼んで受け取る（unitree_sdk2py の Go2 用 VideoClient。1920x1080 の JPEG）。
    2026-09-29 に PC2 で、どちらも videohub を止めずに受け取れることを確かめた（depth_only_check.py、videohub_check.py）。

    - カラーは縦横比を保って camera.videohub.width（既定 640 → 640x360）に縮めて送る
    - カラーの内部パラメータと、深度 → カラーの位置関係は、カラーを開かずに RealSense の設定から読む
    - カラーと深度は別の経路なので、撮った瞬間は揃わない（止まっている物なら問題ない）。差は 5 秒ごとの表示に出す
    - DDS は PC2 の中だけで話す（camera.videohub.network_interface。videohub の設定は eth0）
    """

    def __init__(self, cam_cfg: dict[str, Any]) -> None:
        import pyrealsense2 as rs

        self._rs = rs
        self.cfg = cam_cfg
        self.vh = cam_cfg["videohub"]
        self.pipeline = rs.pipeline()
        self.depth_scale = 0.001
        # 送るカラーの内部パラメータ（1 枚目のカラーを受け取ったときに決まる）
        self.intrinsics: Intrinsics | None = None
        self._color_src: Intrinsics | None = None  # RealSense の設定から読んだカラーの内部パラメータ
        self._depth_intr: Intrinsics | None = None
        self._extr: Extrinsics | None = None
        self._client: Any = None
        self._fail = 0  # 続けてカラーを受け取れなかった回数
        self._dt_ms: list[float] = []  # カラーと深度の時刻の差
        self._align_ms: list[float] = []  # 位置合わせにかかった時間

    @staticmethod
    def _to_intr(i: Any) -> Intrinsics:
        return Intrinsics(width=i.width, height=i.height, fx=i.fx, fy=i.fy, cx=i.ppx, cy=i.ppy,
                          model=str(i.model), coeffs=[float(x) for x in i.coeffs])

    def _make_client(self) -> Any:
        """videohub に頼む窓口（テストでは差し替える）。"""
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_client import VideoClient

        ChannelFactoryInitialize(0, str(self.vh["network_interface"]))
        client = VideoClient()
        client.SetTimeout(float(self.vh["timeout_s"]))
        client.Init()
        return client

    def _fetch_color(self) -> np.ndarray | None:
        """videohub からカラーを 1 枚受け取る（BGR）。受け取れなければ None。"""
        code, data = self._client.GetImageSample()
        img = None
        if code == 0:
            img = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self._fail += 1
            if self._fail in (1, 10) or self._fail % 100 == 0:
                print(f"[rgbd_server] ⚠️ videohub からカラーを受け取れない（code={code}、{self._fail} 回続けて）")
            return None
        self._fail = 0
        return img

    def start(self) -> None:
        rs, c = self._rs, self.cfg
        w, h, fps = int(c["width"]), int(c["height"]), int(c["fps"])
        sw, sh = int(self.vh["source_width"]), int(self.vh["source_height"])
        serial = str(c.get("serial") or "")
        devs = [d for d in rs.context().query_devices()
                if not serial or d.get_info(rs.camera_info.serial_number) == serial]
        if not devs:
            raise RuntimeError(f"RealSense が見つからない（シリアル {serial or '指定なし'}）")
        dev = devs[0]

        # カラーは開かずに、設定からカラーの内部パラメータと深度 → カラーの位置関係を読む
        color_prof = depth_prof = None
        for sensor in dev.query_sensors():
            for sp in sensor.get_stream_profiles():
                if not sp.is_video_stream_profile():
                    continue
                vp = sp.as_video_stream_profile()
                size = (vp.width(), vp.height())
                if sp.stream_type() == rs.stream.color and size == (sw, sh):
                    if color_prof is None or vp.fps() == fps:
                        color_prof = vp
                if sp.stream_type() == rs.stream.depth and size == (w, h) and vp.fps() == fps:
                    depth_prof = vp
        if color_prof is None or depth_prof is None:
            raise RuntimeError(f"RealSense の設定に、カラー {sw}x{sh} か深度 {w}x{h}@{fps} が無い")
        self._color_src = self._to_intr(color_prof.get_intrinsics())
        e = depth_prof.get_extrinsics_to(color_prof)
        self._extr = Extrinsics.from_realsense(list(e.rotation), list(e.translation))

        config = rs.config()
        config.enable_device(dev.get_info(rs.camera_info.serial_number))
        config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        profile = self.pipeline.start(config)
        self.depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
        self._depth_intr = self._to_intr(profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics())
        # 起動直後の深度は明るさの調整が済んでおらず、測れない画素が多い（2026-09-29: 1 枚目 45%、30 枚目 96%）
        for _ in range(int(self.vh.get("warmup_frames", 0))):
            self.pipeline.wait_for_frames(int(c.get("timeout_ms", 5000)))

        self._client = self._make_client()
        img = None
        for _ in range(10):
            img = self._fetch_color()
            if img is not None:
                break
        if img is None:
            raise RuntimeError("videohub からカラーを受け取れない（videohub_check.py で確かめる）")
        self._setup_output(img.shape[1], img.shape[0])
        ci, di, t = self.intrinsics, self._depth_intr, self._extr.translation
        assert ci is not None
        print(f"[rgbd_server] 深度: {dev.get_info(rs.camera_info.name)}（シリアル "
              f"{dev.get_info(rs.camera_info.serial_number)}）{w}x{h}@{fps}、深度の単位 {self.depth_scale} m")
        print(f"[rgbd_server] カラー: videohub の {img.shape[1]}x{img.shape[0]} を {ci.width}x{ci.height} に縮めて送る")
        print(f"[rgbd_server] 内部パラメータ（送るカラー）: fx={ci.fx:.1f} fy={ci.fy:.1f} cx={ci.cx:.1f} cy={ci.cy:.1f}"
              f"、（深度）fx={di.fx:.1f} fy={di.fy:.1f} cx={di.cx:.1f} cy={di.cy:.1f}")
        print(f"[rgbd_server] 深度 → カラーの平行移動 {[round(float(x), 4) for x in t]} m")
        if any(abs(x) > 1e-6 for x in self._color_src.coeffs):
            print(f"[rgbd_server] ⚠️ カラーのゆがみの係数が 0 ではない（位置合わせでは無視する）: {self._color_src.coeffs}")

    def _setup_output(self, img_w: int, img_h: int) -> None:
        """受け取ったカラーの大きさから、送る大きさと内部パラメータを決める。"""
        src = self._color_src
        assert src is not None
        if (img_w, img_h) != (src.width, src.height):
            if abs(img_w / img_h - src.width / src.height) > 0.01:
                raise RuntimeError(f"videohub のカラー {img_w}x{img_h} と、内部パラメータ {src.width}x{src.height} の縦横比が違う"
                                   "（camera.videohub.source_width / source_height を合わせる）")
            print(f"[rgbd_server] ⚠️ videohub のカラーは {img_w}x{img_h}（設定は {src.width}x{src.height}）。内部パラメータを縮めて使う")
            src = scale_intrinsics(src, img_w, img_h)
        out_w = int(self.vh["width"])
        out_h = int(round(img_h * out_w / float(img_w)))
        self.intrinsics = scale_intrinsics(src, out_w, out_h)

    def read(self) -> CapturedFrame | None:
        frames = self.pipeline.wait_for_frames(int(self.cfg.get("timeout_ms", 5000)))
        depth = frames.get_depth_frame()
        if not depth:
            return None
        return CapturedFrame(color_rgb=None, depth=np.asanyarray(depth.get_data()).astype(np.uint16, copy=True),
                             timestamp=time.time())

    def complete(self, f: CapturedFrame) -> CapturedFrame | None:
        """深度だけのフレームに、videohub のカラーを足し、深度をカラーに位置合わせする。"""
        img = self._fetch_color()
        if img is None:
            return None
        t_color = time.time()
        ci = self.intrinsics
        assert ci is not None and self._depth_intr is not None and self._extr is not None
        if (img.shape[1], img.shape[0]) != (ci.width, ci.height):
            img = cv2.resize(img, (ci.width, ci.height), interpolation=cv2.INTER_AREA)
        t0 = time.monotonic()
        depth = align_depth_to_color(f.depth, self.depth_scale, self._depth_intr, ci, self._extr)
        self._align_ms.append((time.monotonic() - t0) * 1000.0)
        self._dt_ms.append((t_color - f.timestamp) * 1000.0)
        return CapturedFrame(color_rgb=np.ascontiguousarray(img[:, :, ::-1]), depth=depth, timestamp=t_color)

    def status(self) -> str:
        """5 秒ごとの表示に足す: カラーと深度の時刻の差、位置合わせの時間（前回の表示からの平均）。"""
        if not self._dt_ms:
            return "カラーを受け取れていない"
        dt = float(np.mean(self._dt_ms))
        al = float(np.mean(self._align_ms))
        self._dt_ms, self._align_ms = [], []
        return f"カラーと深度の時刻の差 {dt:.0f} ms、位置合わせ {al:.0f} ms"

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


def make_camera(cam_cfg: dict[str, Any]) -> RealSenseCamera | VideohubCamera | DummyCamera:
    src = cam_cfg["source"]
    if src == "realsense":
        return RealSenseCamera(cam_cfg)
    if src == "videohub":
        return VideohubCamera(cam_cfg)
    if src == "dummy":
        return DummyCamera(cam_cfg)
    raise ValueError(f"camera.source は realsense / videohub / dummy のどれか: {src}")


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
                if f.color_rgb is None:
                    # 深度だけのフレーム（videohub）: 送るものだけ、ここでカラーを受け取って位置合わせする
                    f = self.camera.complete(f)  # type: ignore[union-attr]
                    if f is None:
                        continue
                # 予定の時刻は積み上げる（平均を上限に合わせるため）。1 周期以上遅れたら今を基準に戻す
                if t_next_pub is None or now > t_next_pub + min_interval:
                    t_next_pub = now
                t_next_pub += min_interval
                n += 1
                if "rgbd" in self.sockets:
                    msg = encode_rgbd(
                        RgbdFrame(
                            color_bgr=f.color_rgb[:, :, ::-1],  # type: ignore[index]  # RGBD の形式は BGR で送る
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
                    status = getattr(self.camera, "status", None)
                    extra = f"、{status()}" if status is not None else ""
                    print(f"[rgbd_server] {n} フレーム配信（直近 {fps:.1f} fps{extra}）")
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
    p.add_argument("--source", choices=["realsense", "videohub", "dummy"],
                   help="カメラ（既定は設定ファイル）。videohub = カラーは Unitree の videohub、深度は RealSense")
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
