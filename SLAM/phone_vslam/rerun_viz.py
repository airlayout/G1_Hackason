"""MASt3R-SLAM の地図・軌跡・いまのこま・状態を Rerun のウェブ表示に出す（上流の in3d の窓の代わり）。

SLAM の本体と同じく別のプロセスで動き、共有の SharedKeyframes・SharedStates を読むだけ。
地図は「キーフレームごとの点群（カメラの座標）＋その姿勢」で出す。姿勢は大域の最適化で後から
動くので、点群は描き直さずに姿勢だけを送り直す。

重い物（点群・画像）は static で送る＝表示の側に履歴が積もらない。時系列に残すのは速さなどの数だけ。
ただし rerun 0.22 の送り口は、送った物を全部（server_memory_limit まで）抱え、後からつないだ
ブラウザに最初から流し直す。なので送る量そのものを絞る:
- キーフレームの点群は、次のキーフレームができて変わらなくなった時に 1 度だけ送る
  （いちばん新しいキーフレームは追跡のたびに育つので、latest_kf_period_s おきにだけ送る）
- キーフレームの姿勢は pose_period_s おきに、pose_tol より動いた物だけ
  （大域の最適化は新しいキーフレームのたびに全部の姿勢を少しずつ動かす。1 本ずつの送りに
  約 1 KB の枠が付くので、そのまま送ると 80 キーフレームで 0.5 MB/s になった）
- いまのこまは image_period_s おき、軌跡・つなぎの線・錐は変わった時だけ

表示の文字は英語にする（Rerun の表示は日本語の字を持たず □ になる）。

rerun-sdk は 0.22.1 に固定する。0.23 から numpy 2 が要り、上流（numpy 1.26 固定・np.unicode_ を使う）
と並べられない。0.22 は表示とのつなぎが WebSocket（ws_port）で、0.23 以降の gRPC とは口が違う。
"""

from __future__ import annotations

import collections
import dataclasses
import faulthandler
import math
import queue
import signal
import time

import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import torch

import viewer_hub
from mast3r_slam.config import set_global_config
from mast3r_slam.frame import Mode

MODE_NAMES = {
    Mode.INIT: "INIT",
    Mode.TRACKING: "TRACKING",
    Mode.RELOC: "LOST (relocalizing)",
    Mode.TERMINATED: "STOPPED",
}
KF_COLOR = (230, 60, 60)
LOOP_COLOR = (250, 200, 40)
JPEG_QUALITY = 80


@dataclasses.dataclass(frozen=True)
class VizOptions:
    name: str = "cam1"  # 表示の記録の名前（2 台目を別に回すときに分ける）
    cams: tuple[str, ...] = ()  # 2 台以上を 1 つの地図にするとき（multicam_viz）のカメラの名前
    recording_id: str = ""  # 表示の記録の名前（回ごとに新しく。空なら道具の名前に固定）
    hosts: tuple[str, ...] = ("127.0.0.1",)  # 表示を開く人に知らせる IP
    period_s: float = 0.2  # 描き直しの間隔
    latest_kf_period_s: float = 3.0  # 育っている途中のキーフレームの点群を送り直す間隔
    image_period_s: float = 0.5  # いまのこまを送る間隔
    pose_period_s: float = 1.0  # キーフレームの姿勢を送り直す間隔
    pose_tol: float = 1e-3  # これより動いた姿勢だけ送り直す（Sim3 の 8 つの値の差の最大）
    pts_stride: int = 2  # 点群を縦横この間隔で間引く（288x512 で 1 キーフレーム 約 3.7 万点）
    conf_thresh: float = 1.5  # 上流の窓の既定と同じ
    fov_deg: float = 65.0  # いまのカメラの錐の見た目だけに使う（較正はしない）
    frustum_depth: float = 0.15


def viewer_urls(opts: VizOptions) -> list[str]:
    return [viewer_hub.viewer_url(host) for host in opts.hosts]


def make_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(name="map", origin="world"),
            rrb.Vertical(
                rrb.Spatial2DView(name="camera", origin="world/camera/image"),
                rrb.TextDocumentView(name="status", origin="status"),
                rrb.TimeSeriesView(name="speed", origin="metrics"),
                row_shares=[3, 2, 2],
            ),
            column_shares=[3, 1],
        ),
        collapse_panels=True,
    )


def sim3_parts(data: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """lietorch の Sim3 の並び [tx ty tz qx qy qz qw s] を分ける"""
    return data[:3], data[3:7], float(data[7])


def log_sim3(path: str, data: np.ndarray, axis_length: float = 0.0) -> None:
    t, q, s = sim3_parts(data)
    rr.log(
        path,
        rr.Transform3D(
            translation=t, quaternion=rr.Quaternion(xyzw=q), scale=s, axis_length=axis_length
        ),
        static=True,
    )


def take_dirty(kf, n: int) -> list[int]:
    """変わったキーフレームの番号を取り、その印だけを下ろす（上流の get_dirty_idx の代わり）。

    上流は GPU の上で torch.where(is_dirty) をとる。torch.where（nonzero）は「数える → 書き出す」の 2 段で、
    その間に別のプロセス（追跡・後ろの最適化）の is_dirty[i] = True が GPU で走ると、数えたより多く書いて
    はみ出す（数えた数が 0 なら書き先は 0 番地）。錠は命令を出す所しか守らず、GPU が実際に書く時刻は
    守れない。11-17 の 2 台の実機で表示のプロセスが Xid 31（0 番地への書き込み）で落ちた。
    CPU に写して（その時の写し）から探し、見つけた印だけを下ろす（全部を下ろすと、その間に立った印を消す）。
    """
    if n <= 0:
        return []
    flags = kf.is_dirty[:n].reshape(-1).cpu().numpy()
    idx = np.flatnonzero(flags)
    if len(idx):
        kf.is_dirty[torch.as_tensor(idx, device=kf.is_dirty.device)] = False
    return idx.tolist()


class MapLogger:
    """共有の地図を読み、変わった所だけを Rerun に送る"""

    def __init__(self, states, keyframes, status, opts: VizOptions) -> None:
        self.states = states
        self.keyframes = keyframes
        self.status = status
        self.opts = opts
        self.h, self.w = keyframes.h, keyframes.w
        self.sent_T: dict[int, np.ndarray] = {}
        self.n_sent_kf = 0
        self.tick = 0
        self.pending: set[int] = set()  # 点群を送り直す必要のあるキーフレーム
        self.points_sent_at: dict[int, float] = {}
        self.image_sent_at: dict[str, float] = {}  # いまのこまを送った時刻（カメラの枝ごと）
        self.sent_traj: bytes = b""
        self.sent_edges: tuple = ()
        self.sent_status = ""
        self.pinhole_sent_paths: set[str] = set()
        self.poses_sent_at = 0.0
        self.sent = collections.Counter()  # 送った回数と量（記録に 30 秒おきに出す）
        self.sent_report_at = time.time()
        f = (self.w / 2) / math.tan(math.radians(opts.fov_deg) / 2)
        self.focal = float(f)

    def log_keyframe_points(self, i: int) -> None:
        s = self.opts.pts_stride
        kf = self.keyframes
        with kf.lock:
            X = kf.X[i].view(self.h, self.w, 3)[::s, ::s].reshape(-1, 3).cpu().numpy()
            n = max(int(kf.N[i]), 1)
            C = (kf.C[i].view(self.h, self.w)[::s, ::s] / n).reshape(-1).cpu().numpy()
            rgb = kf.uimg[i][::s, ::s].reshape(-1, 3).numpy().copy()
        keep = C > self.opts.conf_thresh
        colors = (np.clip(rgb[keep], 0, 1) * 255).astype(np.uint8)
        self.sent["points"] += 1
        self.sent["points_kB"] += int(keep.sum()) * 16 // 1000
        rr.log(
            f"world/keyframes/{i}/points",
            rr.Points3D(X[keep], colors=colors, radii=-1.2),
            static=True,
        )

    def growing_keyframes(self, n: int) -> set[int]:
        """追跡のたびに点群が育つキーフレーム（1 台ならいちばん新しい 1 枚）"""
        return {n - 1} if n else set()

    def log_keyframes(self) -> np.ndarray:
        kf = self.keyframes
        with kf.lock:
            n = len(kf)
            dirty = take_dirty(kf, n)
            T = kf.T_WC[:n, 0].cpu().numpy() if n else np.zeros((0, 8), np.float32)
        self.pending.update(i for i in dirty if i < n)
        # まだ送っていないキーフレーム（起こし直した表示では全部）も点群を送る
        self.pending.update(i for i in range(n) if i not in self.sent_T)
        now = time.time()
        growing = self.growing_keyframes(n)
        for i in sorted(self.pending):
            if i >= n:
                self.pending.discard(i)
                continue
            is_growing = i in growing
            if is_growing and now - self.points_sent_at.get(i, 0.0) < self.opts.latest_kf_period_s:
                continue  # 育っている途中。間隔を空けて送る（次のキーフレームができたら最終形を送る）
            self.log_keyframe_points(i)
            self.points_sent_at[i] = now
            self.pending.discard(i)
        resend_moved = now - self.poses_sent_at >= self.opts.pose_period_s
        if resend_moved:
            self.poses_sent_at = now
        for i in range(n):
            prev = self.sent_T.get(i)
            is_new = prev is None
            if is_new or (resend_moved and np.abs(prev - T[i]).max() > self.opts.pose_tol):
                log_sim3(f"world/keyframes/{i}", T[i])
                self.sent_T[i] = T[i].copy()
                self.sent["poses"] += 1
        # 外れたキーフレーム（再測位の失敗で pop された物）を消す
        for i in range(n, self.n_sent_kf):
            rr.log(f"world/keyframes/{i}", rr.Clear(recursive=True), static=True)
            self.sent_T.pop(i, None)
            self.points_sent_at.pop(i, None)
        if n < self.n_sent_kf and n < 2:  # 地図を作り直した
            rr.log("world/trajectory", rr.Clear(recursive=True), static=True)
            rr.log("world/loop_edges", rr.Clear(recursive=True), static=True)
            self.sent_traj = b""
            self.sent_edges = ()
        self.n_sent_kf = n
        self.log_trajectory(T)
        return T

    def log_trajectory(self, T: np.ndarray) -> None:
        traj = np.round(T[:, :3], 4).tobytes()
        if len(T) >= 2 and traj != self.sent_traj:
            self.sent_traj = traj
            self.sent["traj"] += 1
            rr.log(
                "world/trajectory",
                rr.LineStrips3D([T[:, :3]], colors=[KF_COLOR], radii=-1.0),
                static=True,
            )

    def log_loop_edges(self, T: np.ndarray) -> int:
        with self.states.lock:
            ii = list(self.states.edges_ii)
            jj = list(self.states.edges_jj)
        n = len(T)
        pairs = tuple((i, j) for i, j in zip(ii, jj) if abs(i - j) > 1 and i < n and j < n)
        segs = [[T[i, :3], T[j, :3]] for i, j in pairs]
        key = (pairs, np.round(T[:, :3], 3).tobytes())
        if segs and key != self.sent_edges:
            self.sent_edges = key
            self.sent["edges"] += 1
            rr.log(
                "world/loop_edges",
                rr.LineStrips3D(segs, colors=[LOOP_COLOR], radii=-1.0),
                static=True,
            )
        return len(segs)

    def log_cameras(self) -> None:
        if self.states.get_mode() != Mode.INIT:
            with self.states.lock:
                T = self.states.T_WC[0].cpu().numpy()
                img = None
                if self.image_due("world/camera"):
                    img = self.states.uimg.numpy().copy()
            self.log_camera("world/camera", T, img)

    def image_due(self, path: str) -> bool:
        return time.time() - self.image_sent_at.get(path, 0.0) >= self.opts.image_period_s

    def log_camera(self, path: str, T: np.ndarray, img: np.ndarray | None) -> None:
        """いまのカメラの姿勢と錐、間隔を空けていまのこま（img は 0..1 の RGB。None なら画は送らない）"""
        log_sim3(path, T)
        if path not in self.pinhole_sent_paths:
            rr.log(
                f"{path}/image",
                rr.Pinhole(
                    focal_length=self.focal,
                    width=self.w,
                    height=self.h,
                    camera_xyz=rr.ViewCoordinates.RDF,
                    image_plane_distance=self.opts.frustum_depth,
                ),
                static=True,
            )
            self.pinhole_sent_paths.add(path)
        if img is not None:
            self.log_image(path, img)

    def log_image(self, path: str, img: np.ndarray) -> None:
        self.image_sent_at[path] = time.time()
        bgr = cv2.cvtColor((np.clip(img, 0, 1) * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)
        ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            self.sent["images"] += 1
            self.sent["images_kB"] += len(jpg) // 1000
            rr.log(
                f"{path}/image",
                rr.EncodedImage(contents=jpg.tobytes(), media_type="image/jpeg"),
                static=True,
            )

    def log_status(self, n_kf: int, n_loops: int) -> None:
        st = dict(self.status)
        mode = MODE_NAMES.get(self.states.get_mode(), "?")
        rh, rw = st.get("recv_shape", (0, 0))
        lines = [
            f"## {mode}",
            "",
            f"- SLAM: **{st.get('slam_fps', 0.0):.1f} fps** (frame {st.get('frame_idx', 0)})",
            f"- keyframes: **{n_kf}** / {st.get('kf_buffer', '?')}, loop edges: {n_loops}",
            f"- input: {rw}x{rh} @ {st.get('recv_fps', 0.0):.1f} fps, "
            f"received {st.get('received', 0)}, used {st.get('used', 0)}, "
            f"reconnects {st.get('reconnects', 0)}",
            f"- source: `{st.get('source', '')}`",
        ]
        if st.get("waiting"):
            lines.insert(1, "**waiting for video...**")
        elif st.get("lost_s", 0.0) > 0:
            lines.insert(1, f"**lost for {st['lost_s']:.0f} s** - move back to a place seen before")
        if st.get("note"):
            lines.append(f"- note: {st['note']}")
        text = "\n".join(lines)
        if text != self.sent_status and self.tick % 2 == 0:
            self.sent_status = text
            rr.log("status", rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN), static=True)
        rr.set_time_sequence("tick", self.tick)
        rr.log("metrics/slam_fps", rr.Scalar(float(st.get("slam_fps", 0.0))))
        rr.log("metrics/input_fps", rr.Scalar(float(st.get("recv_fps", 0.0))))

    def step(self) -> None:
        T = self.log_keyframes()
        n_loops = self.log_loop_edges(T)
        self.log_cameras()
        self.log_status(len(T), n_loops)
        self.tick += 1
        if time.time() - self.sent_report_at >= 30.0:
            c = self.sent
            print(
                f"[表示] 30 秒で送った物: 点群 {c['points']} 回 {c['points_kB']} kB・"
                f"画 {c['images']} 枚 {c['images_kB']} kB・姿勢 {c['poses']} 本・"
                f"軌跡 {c['traj']} 回・つなぎ {c['edges']} 回",
                flush=True,
            )
            self.sent.clear()
            self.sent_report_at = time.time()


def publish_hint(status, opts: VizOptions) -> str:
    """映像を待つ間に出す案内（スマホで開く URL）"""
    name = str(status.get("source", "")).rstrip("/").rsplit("/", 1)[-1] or "cam1"
    return f"https://{opts.hosts[0]}:8889/{name}/publish"


def wait_for_map(attach_q, status, opts: VizOptions):
    """共有の地図が渡されるまで「映像待ち」を出しながら待つ"""
    text = (
        "## WAITING FOR VIDEO\n\n"
        f"Open **{publish_hint(status, opts)}** on the phone (Wi-Fi: Physical_AI_5G), "
        "choose the back camera and press publish."
    )
    rr.log("status", rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN), static=True)
    while not status.get("stop", False):
        try:
            return attach_q.get(timeout=1.0)
        except queue.Empty:
            status["viz_beat"] = time.time()
    return None


def run_rerun_viz(cfg: dict, attach_q, status, opts: VizOptions) -> None:
    """可視化のプロセスの入口（spawn で起こす）。地図は最初のこまの後に attach_q で届く"""
    faulthandler.register(signal.SIGUSR1, all_threads=True)  # kill -USR1 で止まった所を記録に出す
    print("[表示] 起きた", flush=True)
    set_global_config(cfg)
    # 表示の口は固定（viewer_hub。start_mediamtx.sh が表示のサーバーを常に動かしている）
    viewer_hub.connect(rr, f"phone_vslam mono {opts.name}", make_blueprint(), clear=("world", "status", "metrics"),
                       recording_id=opts.recording_id or None)
    rr.log("world", rr.ViewCoordinates.RDF, static=True)  # 最初のカメラの向きが基準（y が下）
    for url in viewer_urls(opts):
        print(f"[表示] {url}", flush=True)
    shared = wait_for_map(attach_q, status, opts)
    if shared is None:
        print("[表示] 終わる（映像が来る前に止まった）", flush=True)
        return
    states, keyframes = shared
    print("[表示] 地図を受け取った", flush=True)
    logger = MapLogger(states, keyframes, status, opts)
    while not status.get("stop", False):
        t0 = time.time()
        try:
            logger.step()
            status["viz_beat"] = time.time()  # 本体はこれが止まったら表示を起こし直す
        except (BrokenPipeError, EOFError, ConnectionError):
            break  # 本体が先に終わった
        time.sleep(max(0.0, opts.period_s - (time.time() - t0)))
    print("[表示] 終わる", flush=True)
