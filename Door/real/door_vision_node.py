"""11-05: 頭のカラーの画像を取り、画像の網に通して特徴を door_run_stock.py へ送る（PC2 の別のプロセス）。

カラーは純正の映像サービス（videohub_pc4）が握っていて直接は開けないので、純正の VideoClient.GetImageSample() で
1920×1080 の JPEG を受ける（PC2 の eth0・domain 0 なら 50 枚/秒。エンタメ班の 10-25 の実績）。
受ける糸は最新の JPEG だけを覚え、主の輪が --hz（既定 10＝シミュレータの描く間隔）で 1/4 に復号 → 192×108 へ縮める →
網（onnxruntime か numpy）→ 特徴 128 を UDP で送る（door_vision_source.py の電文）。
終わるとき（Ctrl+C・SIGTERM・--seconds）に、送った特徴と縮めた画像・時刻を npz に書く（後で答え合わせに使う）。
--full-every 秒ごとに元の JPEG もそのまま書く（画角・露出を後で確かめる用）。

11-06（カラー＋深度、パッチ 0094 の網）: --rgbd-onnx を渡すと、頭の D435i の深度も受け（既定 --depth shm＝Python 3.8 の中継
pc2/depth_relay.py が /dev/shm に置く枚。PC2 の 3.12 には pyrealsense2 が入らない）、学習と同じ画角・形（54×96）に取り出して
網に入れる。--depth-npz は機体なしの空回し（記録した深度を繰り返す）。
--color-off で深度だけの出力にする（学習で回の 3 割ほどカラーを落としている）。どちらも既定は切（今までと同じ画像だけ）。

例（PC2。door_run_stock.py より先に起こす）:
  python3 door_vision_node.py --encoder door_vision_d2_encoder.npz --onnx door_vision_d2_encoder.onnx --iface eth0
  python3 door_vision_node.py --encoder ... --jpeg-dir <JPEG のフォルダ>   # 機体なしの空回し（同じフォルダを繰り返す）
"""
from __future__ import annotations

import argparse
import os
import signal
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
if Path("/home/unitree/unitree_sdk2_python").is_dir():  # PC2 の SDK は site-packages に入っていない（door_run_real.py と同じ）
    sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
import door_depth_shm as dds  # noqa: E402
import door_vision_encoder as dve  # noqa: E402
import door_vision_source as dvs  # noqa: E402

STATS_EVERY_S = 2.0


def init_dds(iface: str, drop_tracing: bool) -> None:
    """DDS を eth0・domain 0 で起こす。drop_tracing: CycloneDDS の設定から Tracing を外す（エンタメ班の手当て。
    CycloneDDS 0.10.2 の設定の書き出しが do_print_uint32_bitset で落ちる環境があった）"""
    from unitree_sdk2py.core import channel

    if not drop_tracing:
        channel.ChannelFactoryInitialize(0, iface)
        return
    import xml.etree.ElementTree as ET

    original = channel.ChannelConfigHasInterface
    root = ET.fromstring(original)
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "Tracing":
                parent.remove(child)
    channel.ChannelConfigHasInterface = ET.tostring(root, encoding="unicode")
    try:
        channel.ChannelFactoryInitialize(0, iface)
    finally:
        channel.ChannelConfigHasInterface = original


class CameraRx:
    """VideoClient から JPEG を受け続け、最新の 1 枚と受け取った時刻を覚える"""

    def __init__(self, timeout_s: float) -> None:
        from unitree_sdk2py.go2.video.video_client import VideoClient

        self.client = VideoClient()
        self.client.SetTimeout(timeout_s)
        self.client.Init()
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[object, float, int]] = None
        self.n_rx = 0
        self.n_err = 0
        self.last_code = 0
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                code, data = self.client.GetImageSample()
            except Exception as e:  # noqa: BLE001  受けられない回は数えて続ける
                self.n_err += 1
                self.last_code = -1
                print(f"[vision] GetImageSample で例外: {type(e).__name__}: {e}", flush=True)
                time.sleep(0.1)
                continue
            if code != 0 or not data:
                self.n_err += 1
                self.last_code = code
                time.sleep(0.01)
                continue
            with self._lock:
                self.n_rx += 1
                self._latest = (data, time.monotonic(), self.n_rx)  # bytes への直しは使う時だけ（50 枚/秒ぶん直さない）

    def latest(self) -> Optional[Tuple[object, float, int]]:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()


class RealSenseDepthRx:
    """11-06: D435i の深度だけを librealsense（pyrealsense2）で受け続け、最新の 1 枚（m）と受けた時刻を覚える。
    カラーは純正の映像サービスが握っているので開かない。内部の値（fx・fy・cx・cy）は深度の流れから読む"""

    def __init__(self, width: int = 848, height: int = 480, fps: int = 30) -> None:
        import pyrealsense2 as rs

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        prof = self.pipe.start(cfg)
        self.scale = float(prof.get_device().first_depth_sensor().get_depth_scale())
        intr = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.intrinsics = (float(intr.fx), float(intr.fy), float(intr.ppx), float(intr.ppy))
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[np.ndarray, float, int]] = None
        self.n_rx = 0
        self.n_err = 0
        self._stop = threading.Event()
        print(f"[vision] 深度 {width}×{height}@{fps}・尺度 {self.scale}・内部の値 {self.intrinsics}", flush=True)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frames = self.pipe.wait_for_frames(1000)
                d = frames.get_depth_frame()
                arr = np.asanyarray(d.get_data()).astype(np.float32) * self.scale
            except Exception as e:  # noqa: BLE001  受けられない回は数えて続ける
                self.n_err += 1
                print(f"[vision] 深度を受けられない: {type(e).__name__}: {e}", flush=True)
                time.sleep(0.1)
                continue
            with self._lock:
                self.n_rx += 1
                self._latest = (arr, time.monotonic(), self.n_rx)

    def latest(self) -> Optional[Tuple[np.ndarray, float, int]]:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()
        try:
            self.pipe.stop()
        except Exception:  # noqa: BLE001
            pass


class ShmDepthRx:
    """11-06: Python 3.8 の中継（pc2/depth_relay.py・pyrealsense2）が /dev/shm に置く最新の深度を読む（PC2 の lerobot の 3.12 には
    pyrealsense2 が入らない）。受け口は RealSenseDepthRx と同じ。中継が止まると同じ枚を返し続け、撮った時刻が古くなるので
    走らせ役の見張り（撮ってからの古さ）が止める。n_rx は読めた別々の枚の数"""

    def __init__(self, path: str = dds.PATH) -> None:
        self.path = path
        self.intrinsics = (0.0, 0.0, 0.0, 0.0)
        self._latest: Optional[Tuple[np.ndarray, float, int]] = None
        self.n_rx = 0
        self.n_err = 0

    def latest(self) -> Optional[Tuple[np.ndarray, float, int]]:
        fr = dds.read_frame(self.path)
        if fr is None:
            self.n_err += 1
            return self._latest
        if self._latest is None or fr.seq != self._latest[2]:
            self.intrinsics = fr.intrinsics
            self.n_rx += 1
            self._latest = (fr.z16.astype(np.float32) * np.float32(fr.scale), fr.t, fr.seq)
        return self._latest

    def stop(self) -> None:
        pass


class NpzDepthRx:
    """機体なしの空回し: npz（depth [N, H, W] m・intrinsics [fx, fy, cx, cy]）の深度を 30 枚/秒で順に「受けた」ことにする"""

    def __init__(self, path: str) -> None:
        z = np.load(path)
        self.frames = z["depth"].astype(np.float32)
        self.intrinsics = tuple(float(v) for v in z["intrinsics"])
        self.t0 = time.monotonic()
        self.n_rx = 0
        self.n_err = 0

    def latest(self) -> Optional[Tuple[np.ndarray, float, int]]:
        k = int((time.monotonic() - self.t0) * 30.0)
        self.n_rx = k + 1
        return self.frames[k % len(self.frames)], self.t0 + k / 30.0, k + 1

    def stop(self) -> None:
        pass


class DirRx:
    """機体なしの空回し: フォルダの JPEG を 30 枚/秒で順に「受けた」ことにする"""

    def __init__(self, folder: str) -> None:
        self.files = sorted(Path(folder).glob("*.jpg"))
        if not self.files:
            raise ValueError(f"{folder} に JPEG が無い")
        self.t0 = time.monotonic()
        self.n_rx = 0
        self.n_err = 0
        self.last_code = 0

    def latest(self) -> Optional[Tuple[object, float, int]]:
        k = int((time.monotonic() - self.t0) * 30.0)
        self.n_rx = k + 1
        return self.files[k % len(self.files)].read_bytes(), self.t0 + k / 30.0, k + 1

    def stop(self) -> None:
        pass


def pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--encoder", default=None, help="export_vision_student.py の <name>_encoder.npz（画像だけの網）")
    ap.add_argument("--rgbd-onnx", default=None, help="11-06: カラー＋深度の網の <name>_encoder.onnx（渡すと深度も受ける）")
    ap.add_argument("--depth", choices=("shm", "rs"), default="shm",
                    help="深度の受け方（shm＝3.8 の中継 pc2/depth_relay.py が /dev/shm に置く枚・rs＝この Python の pyrealsense2）")
    ap.add_argument("--depth-shm", default=dds.PATH, help="中継の置き場")
    ap.add_argument("--depth-npz", default=None, help="機体なしの空回し: 記録した深度の npz（depth・intrinsics）")
    ap.add_argument("--color-off", action="store_true", help="カラーの枝を落とす（深度だけで扉を当てる）")
    ap.add_argument("--onnx", default=None, help="<name>_encoder.onnx（onnxruntime があれば使う）")
    ap.add_argument("--backend", choices=("auto", "ort", "numpy"), default="auto")
    ap.add_argument("--threads", type=int, default=4, help="onnxruntime の糸（numpy は OPENBLAS_NUM_THREADS で決まる）")
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--port", type=int, default=dvs.PORT)
    ap.add_argument("--timeout", type=float, default=3.0, help="VideoClient の返事を待つ上限 [s]")
    ap.add_argument("--keep-tracing", action="store_true", help="CycloneDDS の Tracing を外さない")
    ap.add_argument("--jpeg-dir", default=None, help="機体なしの空回し: このフォルダの JPEG を使う")
    ap.add_argument("--seconds", type=float, default=0.0, help="この秒で終わる（0 は Ctrl+C まで）")
    ap.add_argument("--parent-pid", type=int, default=0, help="このプロセスが居なくなったら終わる（run_vision.sh が走らせ役の pid を渡す）")
    ap.add_argument("--log-dir", default="logs")
    ap.add_argument("--save-max", type=int, default=9000, help="npz に残す縮めた画像の上限（10 Hz で 15 分）")
    ap.add_argument("--full-every", type=float, default=2.0, help="元の JPEG を書く間隔 [s]（0 で書かない）")
    return ap.parse_args(argv)


def main(argv=None) -> int:  # noqa: C901  起動・輪・後始末を 1 か所で読めるように
    a = parse_args(argv)
    depth_rx = None
    if a.rgbd_onnx:
        enc, backend = dve.OrtRGBDEncoder(a.rgbd_onnx, a.threads), "ort-rgbd"
        if a.depth_npz:
            depth_rx = NpzDepthRx(a.depth_npz)
        elif a.depth == "rs":
            depth_rx = RealSenseDepthRx()
        else:
            depth_rx = ShmDepthRx(a.depth_shm)
        print(f"[vision] カラー＋深度の網・カラーを落とす {a.color_off}", flush=True)
    else:
        if not a.encoder:
            raise SystemExit("--encoder（画像だけの網）か --rgbd-onnx（カラー＋深度の網）を渡す")
        enc, backend = dve.make_encoder(a.encoder, a.onnx, a.backend, a.threads)
        print(f"[vision] 画像の網: {backend}・{dve.bench(enc, 5):.0f} ms/枚（空回し）", flush=True)
    if a.jpeg_dir:
        cam = DirRx(a.jpeg_dir)
    else:
        init_dds(a.iface, drop_tracing=not a.keep_tracing)
        cam = CameraRx(a.timeout)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", a.port)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    log_dir = Path(a.log_dir)
    full_dir = log_dir / f"vision_{stamp}_jpeg"
    log_dir.mkdir(parents=True, exist_ok=True)
    if a.full_every > 0:
        full_dir.mkdir(parents=True, exist_ok=True)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rows: dict = {k: [] for k in ("seq", "cam_seq", "t_capture", "t_done", "decode_ms", "encode_ms", "feat")}
    if depth_rx is not None:
        rows["t_depth"] = []
    frames: List[np.ndarray] = []
    depth_frames: List[np.ndarray] = []
    period = 1.0 / a.hz
    t_start = time.monotonic()
    next_t = t_start
    last_cam_seq = -1
    last_full = -1e9
    seq = 0
    last_stats, n_sent_prev, rx_prev, d_prev = t_start, 0, 0, 0
    enc_ms_win: List[float] = []
    print(f"[vision] 始める: {a.hz:.0f} Hz・UDP {dest[1]}・{'フォルダ ' + a.jpeg_dir if a.jpeg_dir else 'VideoClient ' + a.iface}", flush=True)
    try:
        while not stop.is_set():
            now = time.monotonic()
            if a.seconds and now - t_start >= a.seconds:
                break
            got = cam.latest()
            if got is not None and got[2] != last_cam_seq:
                data, t_cap, cam_seq = got
                last_cam_seq = cam_seq
                jpeg = data if isinstance(data, (bytes, bytearray)) else bytes(data)
                t0 = time.monotonic()
                try:
                    bgr = dve.decode_small_bgr(jpeg)
                except ValueError as e:
                    print(f"[vision] {e}", flush=True)
                    stop.wait(period)
                    continue
                t1 = time.monotonic()
                rgb = dve.bgr_to_rgb(bgr)
                if depth_rx is None:
                    feat = enc(dve.preprocess_rgb(rgb))
                else:
                    got_d = depth_rx.latest()
                    if got_d is None:  # 深度がまだ来ていない
                        stop.wait(period)
                        continue
                    d_small = dve.depth_from_intrinsics(got_d[0], *depth_rx.intrinsics)
                    feat = enc(dve.preprocess_rgb(rgb), dve.depth_input(d_small), a.color_off)
                    rows["t_depth"].append(got_d[1])
                    if len(depth_frames) < a.save_max:
                        depth_frames.append(d_small.astype(np.float16))
                    # 撮った時刻は古い方（遅れの見積もりを甘くしない）
                    t_cap = min(t_cap, got_d[1])
                t_done = time.monotonic()
                enc_ms = (t_done - t1) * 1000.0
                sock.sendto(dvs.pack(seq, feat, t_cap, t_done, enc_ms), dest)
                for k, v in (("seq", seq), ("cam_seq", cam_seq), ("t_capture", t_cap), ("t_done", t_done),
                             ("decode_ms", (t1 - t0) * 1000.0), ("encode_ms", enc_ms), ("feat", feat)):
                    rows[k].append(v)
                if len(frames) < a.save_max:
                    frames.append(rgb)
                if a.full_every > 0 and now - last_full >= a.full_every:
                    last_full = now
                    (full_dir / f"{seq:06d}_{t_cap:.3f}.jpg").write_bytes(jpeg)
                enc_ms_win.append(enc_ms)
                if seq == 0:
                    print(f"[vision] 最初の特徴を送った（復号 {(t1 - t0) * 1000:.0f} ms・網 {enc_ms:.0f} ms）", flush=True)
                seq += 1
            if now - last_stats >= STATS_EVERY_S and a.parent_pid and not pid_alive(a.parent_pid):
                print(f"[vision] 走らせ役（pid {a.parent_pid}）が終わったので終わる", flush=True)
                break
            if now - last_stats >= STATS_EVERY_S:
                dt = now - last_stats
                ages = [rows["t_done"][-1] - rows["t_capture"][-1]] if rows["t_done"] else [float("nan")]
                d_txt = "" if depth_rx is None else f"・深度 {(depth_rx.n_rx - d_prev) / dt:4.1f} 枚/秒"
                print(f"[vision] 受けた {(cam.n_rx - rx_prev) / dt:4.1f} 枚/秒・送った {(seq - n_sent_prev) / dt:4.1f} 回/秒{d_txt}・"
                      f"網 {np.mean(enc_ms_win) if enc_ms_win else float('nan'):.0f} ms・撮ってから {ages[0] * 1000:.0f} ms・"
                      f"失敗 {cam.n_err}（code {cam.last_code}）", flush=True)
                last_stats, n_sent_prev, rx_prev, enc_ms_win = now, seq, cam.n_rx, []
                d_prev = 0 if depth_rx is None else depth_rx.n_rx
            next_t += period
            slack = next_t - time.monotonic()
            if slack > 0:
                stop.wait(slack)
            else:
                next_t = time.monotonic()
    finally:
        cam.stop()
        if depth_rx is not None:
            depth_rx.stop()
        sock.close()
        if rows["seq"]:
            out = log_dir / f"vision_{stamp}.npz"
            extra = {"depth": np.stack(depth_frames)} if depth_frames else {}
            np.savez_compressed(out, rgb=np.stack(frames), **extra, **{k: np.array(v) for k, v in rows.items()},
                                meta=np.array(repr(dict(args=vars(a), backend=backend))))
            print(f"[vision] 書いた: {out}（{len(rows['seq'])} 回・画像 {len(frames)} 枚）", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
