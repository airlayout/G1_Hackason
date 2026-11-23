"""2 台のスマホを並べたステレオで DROID-SLAM を回し、地図を Rerun のウェブ表示に出す（実寸）。

    ~/miniforge3/envs/droid/bin/python run_stereo_slam.py [--calib calib/<日時>/calib.json] [--sync calib/sync_<日時>.json]

- 較正（stereo_calib.py）の calib.json で左右を平行化し、DROID-SLAM のステレオに渡す
- 2 台はシャッターがそろわない。右は「左が届いた時刻＋ずれ（measure_sync.py）」にいちばん近いこまを使い、
  差が --max-pair-dt を超える組は捨てる
- DROID-SLAM のステレオは基線を 0.1 と決め打ちしている（projective_ops の [-0.1, 0, 0]）。
  較正の基線 B から、位置と点に B / 0.1 を掛けて実寸（メートル）で出す
- 止める（SIGINT・SIGTERM）と、全体の最適化を 1 回かけてから軌跡（traj.txt）と点群（map.ply）を書く
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import json
import pathlib
import signal
import sys
import threading
import time

import cv2
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
WORK = pathlib.Path.home() / "work_inoue/phone_slam"
sys.path.insert(0, str(HERE))

from stereo_core import load_calib, rectified_intrinsics, rectify_maps  # noqa: E402

DROID_BASELINE = 0.1  # DROID-SLAM のステレオが決め打ちしている基線
FPS_WINDOW = 30
PAIR_WAIT_S = 10.0


def latest(pattern: str) -> pathlib.Path | None:
    found = sorted(WORK.glob(pattern), key=lambda p: p.stat().st_mtime)
    return found[-1] if found else None


def droid_size(w: int, h: int, long_side: int) -> tuple[int, int]:
    """DROID に渡す (W, H)。長い辺を long_side に、どちらも 8 の倍数に"""
    s = long_side / max(w, h)
    return max(8, int(round(w * s / 8)) * 8), max(8, int(round(h * s / 8)) * 8)


@dataclasses.dataclass
class PairStats:
    pairs: int = 0
    skipped: int = 0
    dts: collections.deque = dataclasses.field(default_factory=lambda: collections.deque(maxlen=100))


class StereoSource:
    """2 つの RTSP を読み、時刻で組にして平行化し、DROID の大きさに縮めて渡す"""

    def __init__(self, calib: dict, offset_s: float, long_side: int, max_pair_dt: float) -> None:
        from rtsp_grabber import LatestFrameGrabber

        self.w, self.h = calib["size"]
        self.offset_s = offset_s
        self.max_pair_dt = max_pair_dt
        self.maps = rectify_maps(calib)
        self.W, self.H = droid_size(self.w, self.h, long_side)
        fx, fy, cx, cy = rectified_intrinsics(calib)
        sx, sy = self.W / self.w, self.H / self.h
        self.intrinsics = np.array([fx * sx, fy * sy, cx * sx, cy * sy], np.float32)
        self.left_url, self.right_url = calib["left_stream"], calib["right_stream"]
        self.gl = LatestFrameGrabber(self.left_url, out_wh=(self.w, self.h), history_len=30)
        self.gr = LatestFrameGrabber(self.right_url, out_wh=(self.w, self.h), history_len=30)
        self.stats = PairStats()
        self._last_seq = -1

    def next_pair(self, stop) -> tuple[float, np.ndarray, np.ndarray] | None:
        """次の組 (左の届いた時刻, 左, 右)。平行化して DROID の大きさにした BGR。止めの合図なら None"""
        t_wait = time.time()
        while not stop():
            item = self.gl.latest()
            if item is None or item[1] == self._last_seq:
                if time.time() - t_wait > PAIR_WAIT_S:
                    print(f"[ステレオ] {PAIR_WAIT_S:.0f} 秒こまが来ない（{self.left_url}・{self.right_url}）", flush=True)
                    t_wait = time.time()
                time.sleep(0.003)
                continue
            tl, seq, img_l = item
            self._last_seq = seq
            near = self.gr.frame_near(tl + self.offset_s)
            if near is None:
                continue
            dt = near[0] - (tl + self.offset_s)
            if abs(dt) > self.max_pair_dt:
                self.stats.skipped += 1
                continue
            self.stats.pairs += 1
            self.stats.dts.append(abs(dt))
            return tl, self._rectify(img_l, 0), self._rectify(near[2], 1)
        return None

    def _rectify(self, img: np.ndarray, side: int) -> np.ndarray:
        mx, my = (self.maps[0], self.maps[1]) if side == 0 else (self.maps[2], self.maps[3])
        r = cv2.remap(img, mx, my, cv2.INTER_LINEAR)
        return cv2.resize(r, (self.W, self.H), interpolation=cv2.INTER_AREA)

    def close(self) -> None:
        self.gl.close()
        self.gr.close()


def droid_args(weights: str, W: int, H: int, buffer: int) -> argparse.Namespace:
    """上流の test_euroc.py（ステレオ）の既定に合わせる"""
    return argparse.Namespace(
        weights=weights, buffer=buffer, image_size=[H, W], disable_vis=True, stereo=True,
        beta=0.3, filter_thresh=2.4, warmup=15, keyframe_thresh=3.0,
        frontend_thresh=17.5, frontend_window=20, frontend_radius=2, frontend_nms=1,
        backend_thresh=24.0, backend_radius=2, backend_nms=2,
        upsample=False, asynchronous=False, frontend_device="cuda", backend_device="cuda",
    )


class StopFlag:
    def __init__(self) -> None:
        self.stop = False
        signal.signal(signal.SIGINT, self._handle)
        signal.signal(signal.SIGTERM, self._handle)

    def _handle(self, signum, frame) -> None:  # noqa: ARG002
        print(f"[ステレオ] 止めの合図（{signal.Signals(signum).name}）。最適化して保存してから終わる", flush=True)
        self.stop = True


def map_points(video, scale: float, filter_thresh: float = 0.02, filter_count: int = 2, idx=None):
    """DROID の地図の点（世界の座標・メートル）と色。idx を渡すとそのキーフレームだけ"""
    import droid_backends
    import torch
    from lietorch import SE3

    t = video.counter.value
    with video.get_lock():
        poses = video.poses[:t].clone()
        disps = video.disps[:t].clone()
        intr = video.intrinsics[0].clone()
        images = video.images[:t, :, 4::8, 4::8].clone()
    index = torch.arange(t, device="cuda") if idx is None else torch.as_tensor(idx, device="cuda", dtype=torch.long)
    if len(index) == 0:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)
    thresh = filter_thresh * torch.ones_like(disps.mean(dim=[1, 2]))
    points = droid_backends.iproj(SE3(poses).inv().data, disps, intr)[index]
    counts = droid_backends.depth_filter(poses, disps, intr, index, thresh[index])
    mask = (counts >= filter_count) & (disps[index] > 0.25 * disps.mean())
    colors = images[index][:, [2, 1, 0]].permute(0, 2, 3, 1)
    return (points[mask] * scale).cpu().numpy(), colors[mask].cpu().numpy().astype(np.uint8)


def camera_poses(video, scale: float) -> np.ndarray:
    """キーフレームのカメラ→世界の姿勢 (N, 7) [tx ty tz qx qy qz qw]（メートル）"""
    from lietorch import SE3

    t = video.counter.value
    with video.get_lock():
        poses = video.poses[:t].clone()
    c2w = SE3(poses).inv().data.cpu().numpy()
    c2w[:, :3] *= scale
    return c2w


class DroidRerunViz(threading.Thread):
    """DROID の地図を Rerun に出す（同じプロセスの別の糸）。

    前の窓（frontend_window）より古いキーフレームは局所の最適化で動かないので、点を 1 度だけ送る。
    窓の中のキーフレームは まとめて recent_period_s おきに送り直す。全体の最適化の後は全部を送り直す。
    """

    def __init__(self, droid, source: StereoSource, scale: float, status: dict, opts) -> None:
        super().__init__(name="rerun-viz", daemon=True)
        self.droid, self.source, self.scale, self.status, self.opts = droid, source, scale, status, opts
        self.window = droid.args.frontend_window + 2
        self.sent_settled = 0
        self.recent_at = 0.0
        self.image_at = 0.0
        self.resend_all = threading.Event()
        self.stop_evt = threading.Event()
        self.latest_pair: tuple[np.ndarray, np.ndarray] | None = None

    def run(self) -> None:
        import rerun as rr
        import rerun.blueprint as rrb

        import viewer_hub

        # 表示の口は固定（viewer_hub。start_mediamtx.sh が表示のサーバーを常に動かしている）
        viewer_hub.connect(
            rr,
            "phone_vslam stereo",
            rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(name="map (meters)", origin="world"),
                    rrb.Vertical(
                        rrb.Spatial2DView(name="stereo (rectified)", origin="stereo"),
                        rrb.TextDocumentView(name="status", origin="status"),
                        row_shares=[3, 2],
                    ),
                    column_shares=[3, 1],
                ),
                collapse_panels=True,
            ),
            clear=("world", "stereo", "status"),
        )
        rr.log("world", rr.ViewCoordinates.RDF, static=True)
        for host in [h for h in self.opts.hosts.split(",") if h]:
            print(f"[表示] {viewer_hub.viewer_url(host)}", flush=True)
        while not self.stop_evt.is_set():
            try:
                self.step(rr)
            except Exception as e:  # 表示の失敗で SLAM を止めない
                print(f"[表示] 失敗: {type(e).__name__}: {e}", flush=True)
            self.stop_evt.wait(0.5)

    def step(self, rr) -> None:
        video = self.droid.video
        t = video.counter.value
        now = time.time()
        if self.resend_all.is_set():
            self.resend_all.clear()
            rr.log("world/settled", rr.Clear(recursive=True), static=True)
            self.sent_settled = 0
        if t >= 2:
            settled = max(0, t - self.window)
            if settled > self.sent_settled:
                pts, cols = map_points(video, self.scale, idx=list(range(self.sent_settled, settled)))
                rr.log(f"world/settled/{self.sent_settled}_{settled}", rr.Points3D(pts, colors=cols, radii=-1.5), static=True)
                self.sent_settled = settled
            if now - self.recent_at > 2.0:
                self.recent_at = now
                pts, cols = map_points(video, self.scale, idx=list(range(settled, t)))
                rr.log("world/recent", rr.Points3D(pts, colors=cols, radii=-1.5), static=True)
            c2w = camera_poses(video, self.scale)
            rr.log("world/trajectory", rr.LineStrips3D([c2w[:, :3]], colors=[(230, 60, 60)], radii=-1.5), static=True)
            last = c2w[-1]
            rr.log("world/camera", rr.Transform3D(translation=last[:3], quaternion=rr.Quaternion(xyzw=last[3:7])), static=True)
            fx, fy, cx, cy = self.source.intrinsics
            rr.log(
                "world/camera/image",
                rr.Pinhole(focal_length=[float(fx), float(fy)], principal_point=[float(cx), float(cy)],
                           width=self.source.W, height=self.source.H, camera_xyz=rr.ViewCoordinates.RDF,
                           image_plane_distance=0.15),
                static=True,
            )
        pair = self.latest_pair
        if pair is not None and now - self.image_at > 0.5:
            self.image_at = now
            both = np.hstack(pair).copy()
            for y in range(0, both.shape[0], max(16, both.shape[0] // 16)):
                cv2.line(both, (0, y), (both.shape[1] - 1, y), (0, 220, 255), 1)
            ok, jpg = cv2.imencode(".jpg", both, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                rr.log("stereo", rr.EncodedImage(contents=jpg.tobytes(), media_type="image/jpeg"), static=True)
            ok, jpg = cv2.imencode(".jpg", pair[0], [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok and t >= 2:
                rr.log("world/camera/image", rr.EncodedImage(contents=jpg.tobytes(), media_type="image/jpeg"), static=True)
        st = self.source.stats
        dt_ms = 1000 * float(np.mean(st.dts)) if st.dts else 0.0
        rr.log(
            "status",
            rr.TextDocument(
                f"## STEREO {'WAITING' if self.status.get('waiting') else 'TRACKING'}\n\n"
                f"- SLAM: **{self.status.get('fps', 0.0):.1f} fps**, keyframes **{t}**\n"
                f"- pairs used {st.pairs}, skipped {st.skipped} (|dt| > {1000 * self.source.max_pair_dt:.0f} ms), "
                f"mean |dt| {dt_ms:.1f} ms\n"
                f"- baseline {self.status.get('baseline_m', 0):.3f} m, sync offset {1000 * self.source.offset_s:+.0f} ms\n"
                f"- input {self.source.W}x{self.source.H} (rectified)\n"
                f"- left `{self.source.left_url}` / right `{self.source.right_url}`",
                media_type=rr.MediaType.MARKDOWN,
            ),
            static=True,
        )


def save_results(droid, wall_times: list[float], scale: float, out: pathlib.Path) -> None:
    from plyfile import PlyData, PlyElement

    out.mkdir(parents=True, exist_ok=True)
    video = droid.video
    t = video.counter.value
    c2w = camera_poses(video, scale)
    with video.get_lock():
        tst = video.tstamp[:t].cpu().numpy().astype(int)
    with open(out / "traj.txt", "w") as f:
        for k, pose in zip(tst, c2w):
            ts = wall_times[k] if 0 <= k < len(wall_times) else float(k)
            f.write(f"{ts:.6f} " + " ".join(f"{v:.6f}" for v in pose) + "\n")
    pts, cols = map_points(video, scale)
    v = np.empty(len(pts), dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    v["x"], v["y"], v["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    v["red"], v["green"], v["blue"] = cols[:, 0], cols[:, 1], cols[:, 2]
    PlyData([PlyElement.describe(v, "vertex")]).write(str(out / "map.ply"))
    print(f"[ステレオ] 保存した: {out}/traj.txt・map.ply（キーフレーム {t}・点 {len(pts):,}）", flush=True)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calib", default="", help="calib.json（既定は calib/ の最新）")
    p.add_argument("--sync", default="", help="measure_sync.py の json（既定は calib/ の最新。測れなかった回や無い時は calib.json の値）")
    p.add_argument("--droid-repo", default=str(WORK / "DROID-SLAM"))
    p.add_argument("--long-side", type=int, default=512, help="DROID に渡す画の長い辺")
    p.add_argument("--buffer", type=int, default=512)
    p.add_argument("--max-pair-dt", type=float, default=0.02, help="左右の届いた時刻の差の上限（秒）")
    p.add_argument("--global-ba-every-s", type=float, default=0.0, help="全体の最適化をかける間隔（0 は止める時だけ）")
    p.add_argument("--save-dir", default="")
    p.add_argument("--hosts", default="127.0.0.1")
    args = p.parse_args()

    calib_path = pathlib.Path(args.calib) if args.calib else latest("calib/*/calib.json")
    if calib_path is None or not calib_path.exists():
        raise SystemExit("[ステレオ] calib.json が無い。先に stereo_calib.py で較正する")
    calib = load_calib(calib_path)
    sync_path = pathlib.Path(args.sync) if args.sync else latest("calib/sync_*.json")
    offset = float(calib.get("sync_offset_s", 0.0))
    if sync_path is not None and sync_path.exists():
        sync = json.loads(sync_path.read_text())
        if "offset_s" in sync and sync.get("ok", True):
            # ずれは「json の right が json の left より何秒遅れて届くか」。較正の左右と向きを合わせる
            offset = float(sync["offset_s"])
            if sync.get("left") == calib["right_stream"] and sync.get("right") == calib["left_stream"]:
                offset = -offset
            elif sync.get("left") != calib["left_stream"]:
                print(f"[ステレオ] ⚠️ 時刻のずれの json の流れ（{sync.get('left')}・{sync.get('right')}）が較正と違う", flush=True)
    scale = float(calib["baseline_m"]) / DROID_BASELINE
    print(f"[ステレオ] 較正 {calib_path}（基線 {calib['baseline_m']:.3f} m・平行化の誤差 {calib['epipolar_err_px']:.2f} px）", flush=True)
    print(f"[ステレオ] 時刻のずれ {offset * 1000:+.0f} ms（{sync_path or 'calib.json'}）", flush=True)

    sys.path.insert(0, str(pathlib.Path(args.droid_repo) / "droid_slam"))
    import torch

    torch.multiprocessing.set_start_method("spawn")
    from droid import Droid

    stop = StopFlag()
    source = StereoSource(calib, offset, args.long_side, args.max_pair_dt)
    print(f"[ステレオ] DROID に {source.W}x{source.H} で渡す（fx {source.intrinsics[0]:.1f}）", flush=True)
    droid = Droid(droid_args(str(pathlib.Path(args.droid_repo) / "droid.pth"), source.W, source.H, args.buffer))
    status = {"baseline_m": float(calib["baseline_m"]), "fps": 0.0, "waiting": True}
    viz = DroidRerunViz(droid, source, scale, status, args)
    viz.start()

    wall_times: list[float] = []
    stamps: collections.deque = collections.deque(maxlen=FPS_WINDOW)
    intr = torch.as_tensor(source.intrinsics)
    ba_at = time.time()
    k = 0
    while not stop.stop:
        pair = source.next_pair(lambda: stop.stop)
        if pair is None:
            break
        tl, il, ir = pair
        status["waiting"] = False
        viz.latest_pair = (il, ir)
        images = torch.from_numpy(np.stack([il, ir])).permute(0, 3, 1, 2)  # [2, 3, H, W] BGR uint8
        droid.track(k, images, intrinsics=intr)
        wall_times.append(tl)
        stamps.append(time.time())
        status["fps"] = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
        if k % 30 == 0:
            st = source.stats
            print(f"[ステレオ] こま {k}  {status['fps']:.1f} fps  キーフレーム {droid.video.counter.value}  "
                  f"組 {st.pairs}・捨てた {st.skipped}", flush=True)
        if args.global_ba_every_s > 0 and time.time() - ba_at > args.global_ba_every_s and droid.video.counter.value > 20:
            droid.backend(7)
            viz.resend_all.set()
            ba_at = time.time()
        k += 1

    source.close()
    if droid.video.counter.value > 2:
        print("[ステレオ] 全体の最適化（数秒〜）", flush=True)
        droid.backend(7)
        viz.resend_all.set()
        save_dir = pathlib.Path(args.save_dir or WORK / "results" / time.strftime("%m%d_%H%M%S_stereo"))
        save_results(droid, wall_times, scale, save_dir)
    viz.stop_evt.set()
    print(f"[ステレオ] 終わり（こま {k}・キーフレーム {droid.video.counter.value}）", flush=True)


if __name__ == "__main__":
    main()
