"""2 台以上のスマホで作る 1 つの地図を、Rerun のウェブ表示に出す（rerun_viz の単眼の表示を広げた物）。

- 点群はキーフレームの色のまま（どのカメラが撮ったかは軌跡の色で分かる）
- 軌跡はカメラごとに色を分ける（cam1 赤・cam2 青・…）
- 別のカメラのキーフレーム同士のつなぎ（＝合流した所）を黄緑の線で出す。同じカメラのループは黄
- 右の列にカメラごとのいまのこま

表示の文字は英語（Rerun の表示は日本語の字を持たない）。
"""

from __future__ import annotations

import queue
import time

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

import viewer_hub
from viewer_hub import cam_color
from mast3r_slam.config import set_global_config
from mast3r_slam.frame import Mode
from rerun_viz import LOOP_COLOR, MapLogger, VizOptions, viewer_urls

LINK_COLOR = (170, 240, 60)
APP_ID = "phone_vslam multicam"


def make_blueprint(names: tuple[str, ...]) -> rrb.Blueprint:
    views = [rrb.Spatial2DView(name=name, origin=f"world/cams/{name}/image") for name in names]
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(name="map (one color per phone)", origin="world"),
            rrb.Vertical(*views, rrb.TextDocumentView(name="status", origin="status"), row_shares=[2] * len(views) + [2]),
            column_shares=[3, 1],
        ),
        collapse_panels=True,
    )


def classify_edges(ii: list[int], jj: list[int], cam_ids: np.ndarray) -> tuple[list[tuple[int, int]], list[tuple[int, int]]]:
    """後ろの最適化のつなぎを、別のカメラとのつなぎ（合流）と、同じカメラのループに分ける。

    同じカメラの「直前のキーフレーム」とのつなぎ（ふつうの前進）はどちらにも入れない。
    """
    n = len(cam_ids)
    order = {}  # キーフレーム → そのカメラの中で何枚目か
    for c in np.unique(cam_ids):
        for k, i in enumerate(np.flatnonzero(cam_ids == c)):
            order[int(i)] = k
    links, loops = [], []
    for i, j in zip(ii, jj):
        if i >= n or j >= n:
            continue
        if cam_ids[i] != cam_ids[j]:
            links.append((i, j))
        elif abs(order[i] - order[j]) > 1:
            loops.append((i, j))
    return links, loops


class MultiCamMapLogger(MapLogger):
    def __init__(self, states, keyframes, status, opts: VizOptions) -> None:
        super().__init__(states, keyframes, status, opts)
        self.names = opts.cams
        self.sent_traj_cam: dict[int, bytes] = {}
        self.sent_links: tuple = ()

    def _cam_ids(self, n: int) -> np.ndarray:
        ids = self.keyframes.cam_ids()
        return ids[:n] if len(ids) >= n else np.concatenate([ids, np.full(n - len(ids), -1)])

    def growing_keyframes(self, n: int) -> set[int]:
        """カメラごとの最新の 1 枚（どれも追跡のたびに育つ。1 台用の「いちばん新しい 1 枚」だけだと、
        もう一方のカメラの最新の 1 枚を描き直しのたびに送り直し、30 秒で 57 MB になった）"""
        ids = self._cam_ids(n)
        return {int(np.flatnonzero(ids == c)[-1]) for c in np.unique(ids) if c >= 0}

    def log_trajectory(self, T: np.ndarray) -> None:
        ids = self._cam_ids(len(T))
        for c, name in enumerate(self.names):
            pts = T[ids == c, :3]
            key = np.round(pts, 4).tobytes()
            if len(pts) < 2 or key == self.sent_traj_cam.get(c):
                continue
            self.sent_traj_cam[c] = key
            self.sent["traj"] += 1
            rr.log(
                f"world/trajectory/{name}",
                rr.LineStrips3D([pts], colors=[cam_color(c)], radii=-1.5),
                static=True,
            )
        if len(T) < 2 and self.sent_traj_cam:  # 地図を作り直した
            rr.log("world/trajectory", rr.Clear(recursive=True), static=True)
            rr.log("world/links", rr.Clear(recursive=True), static=True)
            self.sent_traj_cam.clear()
            self.sent_links = ()

    def log_loop_edges(self, T: np.ndarray) -> int:
        with self.states.lock:
            ii = list(self.states.edges_ii)
            jj = list(self.states.edges_jj)
        links, loops = classify_edges(ii, jj, self._cam_ids(len(T)))
        key = (tuple(links), tuple(loops), np.round(T[:, :3], 3).tobytes())
        if key != self.sent_links:
            self.sent_links = key
            self.sent["edges"] += 1
            if links:
                rr.log("world/links", rr.LineStrips3D([[T[i, :3], T[j, :3]] for i, j in links], colors=[LINK_COLOR], radii=-1.5), static=True)
            if loops:
                rr.log("world/loop_edges", rr.LineStrips3D([[T[i, :3], T[j, :3]] for i, j in loops], colors=[LOOP_COLOR], radii=-1.0), static=True)
        self.status["n_links"] = len(links)
        return len(links) + len(loops)

    def log_cameras(self) -> None:
        for c, name in enumerate(self.names):
            path = f"world/cams/{name}"
            mode = self.states.get_camera_mode(c)
            due = self.image_due(path)
            with self.states.lock:
                T = self.states.T_WC[c, 0].cpu().numpy()
                img = self.states.uimg[c].numpy().copy() if due else None
            if img is not None and not img.any():
                img = None  # まだ 1 こまも来ていない
            if mode == Mode.INIT:
                # 地図に重なる前は姿勢が無い。画だけ出す（2D の窓で届いているのが分かる）
                if img is not None:
                    self.log_image(path, img)
                continue
            self.log_camera(path, T, img)

    def log_status(self, n_kf: int, n_loops: int) -> None:
        st = dict(self.status)
        ids = self._cam_ids(n_kf)
        lines = [f"## {st.get('headline', '')}", ""]
        for c, name in enumerate(self.names):
            cs = st.get(f"cam_{name}", {})
            mode = self.states.get_camera_mode(c)
            if mode == Mode.INIT:
                label = "waiting for video" if not cs.get("received") else ("starts the map" if n_kf == 0 else "joining: show a place the map has seen")
            elif mode == Mode.TRACKING:
                label = "TRACKING"
            else:
                label = f"LOST {cs.get('lost_s', 0):.0f}s - go back to a place seen before"
            if cs.get("problem") and mode != Mode.TRACKING:
                label += f" ({cs['problem']} - point at a textured place 1-2 m away)"
            rh, rw = cs.get("recv_shape", (0, 0))
            lines.append(
                f"- **{name}**: {label} - {cs.get('slam_fps', 0.0):.1f} fps, "
                f"keyframes {int((ids == c).sum())}, input {rw}x{rh} @ {cs.get('recv_fps', 0.0):.0f} fps"
            )
        lines += ["", f"- keyframes total **{n_kf}** / {st.get('kf_buffer', '?')}, links between phones: **{st.get('n_links', 0)}**"]
        if st.get("note"):
            lines.append(f"- note: {st['note']}")
        text = "\n".join(lines)
        if text != self.sent_status and self.tick % 2 == 0:
            self.sent_status = text
            rr.log("status", rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN), static=True)


def wait_for_map(attach_q, status, opts: VizOptions):
    text = (
        "## WAITING FOR VIDEO\n\n"
        f"Open **https://{opts.hosts[0]}:8443/?cam=cam1** on one phone and **?cam=cam2** on the other "
        "(Wi-Fi: Physical_AI_5G), then press start on both. Start with both phones looking at the same place."
    )
    rr.log("status", rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN), static=True)
    while not status.get("stop", False):
        try:
            return attach_q.get(timeout=1.0)
        except queue.Empty:
            status["viz_beat"] = time.time()
    return None


def run_multicam_viz(cfg: dict, attach_q, status, opts: VizOptions) -> None:
    """可視化のプロセスの入口（spawn で起こす。run_phone_slam.VizSupervisor から使う）"""
    print("[表示] 起きた", flush=True)
    set_global_config(cfg)
    viewer_hub.connect(rr, APP_ID, make_blueprint(opts.cams), clear=("world", "status", "metrics"),
                       recording_id=opts.recording_id or None)
    rr.log("world", rr.ViewCoordinates.RDF, static=True)
    for url in viewer_urls(opts):
        print(f"[表示] {url}", flush=True)
    shared = wait_for_map(attach_q, status, opts)
    if shared is None:
        print("[表示] 終わる（映像が来る前に止まった）", flush=True)
        return
    states, keyframes = shared
    print("[表示] 地図を受け取った", flush=True)
    logger = MultiCamMapLogger(states, keyframes, status, opts)
    while not status.get("stop", False):
        t0 = time.time()
        try:
            logger.step()
            status["viz_beat"] = time.time()
        except (BrokenPipeError, EOFError, ConnectionError):
            break
        time.sleep(max(0.0, opts.period_s - (time.time() - t0)))
    print("[表示] 終わる", flush=True)
