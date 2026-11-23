"""2 台のスマホを並べたステレオを、部屋の景色で較正する（市松模様は使わない — ユーザーの決定 11-17）。

    # 集める → 解く（様子は表示の固定の口 :9090）
    python scene_calib.py capture --baseline-mm 120
    # 集めた組から解き直す
    python scene_calib.py solve ~/work_inoue/phone_slam/calib/1117_1530_scene --baseline-mm 120

様子は表示の口（固定 :9090）に出る。
集め方: 2 台をテープなどで固定し、模様のある所（机・棚・ポスター・本。白い壁だけは駄目）に向ける。
少し止める → 向きや場所を少し変える、を繰り返す。2 台がそろって止まっていて、模様が十分あり、
前に集めた画と違えば 1 組拾う（既定 30 組）。近くと遠く、左右上下に振った向きを混ぜると良い。

解き方: COLMAP（pycolmap）で 2 台の画をまとめて復元する（各カメラのレンズの値 SIMPLE_RADIAL も解く）。
同時に写した組ごとに「左のカメラ → 右のカメラ」の姿勢を出し、外れを除いて平均する。
景色からは大きさが決まらないので、2 台のレンズの中心の間の距離（--baseline-mm。定規で測る）で実寸にする。

出す物: <out>/<日時>_scene/calib.json（stereo_core.rectify の形。run_stereo_slam.py がそのまま読む）と
images/left・right（集めた画）・colmap/（復元）・rectified_check.jpg（平行化した 1 組に水平線）。
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import json
import pathlib
import shutil
import sys
import time

import cv2
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from stereo_core import (  # noqa: E402
    RIGID_DIR_DEG,
    RIGID_ROT_DEG,
    epipolar_dy,
    rectify,
    rectify_maps,
    robust_relative_pose,
    save_calib,
)

CALIB_LONG_SIDE = 1280
PAIR_MAX_DT_S = 0.05
STEP_S = 0.2
THUMB_W = 160  # 止まり・新しさを比べる小さい画の幅
# 止まりは「画全体のずれ」（位相相関）で見る。明るさの差（平均 2.5 未満）で見ていた時は、撮像のざらつき・
# 圧縮の揺れ・カーペットの細かい模様で、手で止めても「hold still」のまま 1 組も拾えなかった（11-17 の実機）
STILL_SHIFT_PX = 0.8  # 小さい画（幅 160）でのずれ。720 幅で約 3.6 画素
MIN_RESPONSE = 0.05  # 位相相関の山がこれより低ければ、景色が変わった＝動いている
NOVEL_DIFF = 10.0  # 前に拾った画との明るさの平均の差がこれ以上なら新しい
MIN_GAP_S = 0.8
MIN_CORNERS = 150  # 模様の少ない画（白い壁）は拾わない（半分に縮めた画の角の数）
PREVIEW_W = 400


def thumb(img: np.ndarray) -> np.ndarray:
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = g.shape
    return cv2.resize(g, (THUMB_W, int(h * THUMB_W / w)), interpolation=cv2.INTER_AREA).astype(np.float32)


_WINDOWS: dict[tuple[int, int], np.ndarray] = {}


def shift_px(a: np.ndarray, b: np.ndarray) -> float:
    """2 枚の小さい画の、画全体のずれ（画素）。景色が変わって測れなければ無限"""
    key = (a.shape[1], a.shape[0])
    if key not in _WINDOWS:
        _WINDOWS[key] = cv2.createHanningWindow(key, cv2.CV_32F)
    (dx, dy), resp = cv2.phaseCorrelate(cv2.GaussianBlur(a, (3, 3), 0), cv2.GaussianBlur(b, (3, 3), 0), _WINDOWS[key])
    return float(np.hypot(dx, dy)) if resp >= MIN_RESPONSE else float("inf")


def n_corners(img: np.ndarray) -> int:
    g = cv2.cvtColor(cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2)), cv2.COLOR_BGR2GRAY)
    pts = cv2.goodFeaturesToTrack(g, maxCorners=2000, qualityLevel=0.01, minDistance=7)
    return 0 if pts is None else len(pts)


@dataclasses.dataclass
class SceneCollector:
    """2 台がそろって止まり、模様があり、前と違う組を拾う"""

    want: int
    prev: tuple[np.ndarray, np.ndarray] | None = None
    taken: list[np.ndarray] = dataclasses.field(default_factory=list)
    last_t: float = float("-inf")  # 前に拾った時刻（まだ拾っていなければ -inf）
    motion: tuple[float, float] = (float("inf"), float("inf"))  # いまの左右のずれ（画面に出す）

    def offer(self, img_l: np.ndarray, img_r: np.ndarray, now: float) -> str:
        tl, tr = thumb(img_l), thumb(img_r)
        prev, self.prev = self.prev, (tl, tr)
        if prev is None:
            return "hold still"
        self.motion = (shift_px(tl, prev[0]), shift_px(tr, prev[1]))
        if max(self.motion) > STILL_SHIFT_PX:
            return "hold still"
        if now - self.last_t < MIN_GAP_S:
            return "taken - move a little"
        if self.taken and min(np.abs(tl - t).mean() for t in self.taken) < NOVEL_DIFF:
            return "move to a new view"
        if n_corners(img_l) < MIN_CORNERS or n_corners(img_r) < MIN_CORNERS:
            return "not enough texture - point at desks / shelves / posters"
        self.taken.append(tl)
        self.last_t = now
        return "taken"

    @property
    def done(self) -> bool:
        return len(self.taken) >= self.want


# ---- 集める ----

LIVE_S = 2.0  # これより新しいこまが届いていれば「送っている」


def is_live(item) -> bool:
    return item is not None and time.time() - item[0] < LIVE_S


def sending_lines(L, R) -> str:
    def one(name: str, item) -> str:
        return f"- {name}: " + ("sending" if is_live(item) else "**not sending - press the green start button and keep it on**")
    return one("left = iPhone (cam1)", L) + "\n" + one("right = Pixel (cam2)", R)


def fit_to(img: np.ndarray, wh: tuple[int, int]) -> np.ndarray:
    """縦横比を保って wh に収め、余りを黒で埋める（左右の画の大きさをそろえる。変換はいつも同じ）"""
    w, h = wh
    ih, iw = img.shape[:2]
    if (iw, ih) == (w, h):
        return img
    s = min(w / iw, h / ih)
    nw, nh = int(round(iw * s)), int(round(ih * s))
    out = np.zeros((h, w, 3), img.dtype)
    x0, y0 = (w - nw) // 2, (h - nh) // 2
    out[y0 : y0 + nh, x0 : x0 + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    return out


def _log_jpeg(rr, path: str, bgr: np.ndarray) -> None:
    s = PREVIEW_W / bgr.shape[1]
    small = cv2.resize(bgr, (PREVIEW_W, int(bgr.shape[0] * s)))
    ok, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if ok:
        rr.log(path, rr.EncodedImage(contents=jpg.tobytes(), media_type="image/jpeg"), static=True)


def start_viewer(args):
    import rerun as rr
    import rerun.blueprint as rrb

    import viewer_hub

    # 表示の口は固定（viewer_hub。start_mediamtx.sh が表示のサーバーを常に動かしている）
    viewer_hub.connect(
        rr,
        "phone_vslam calibration",
        rrb.Blueprint(
            rrb.Vertical(
                rrb.Horizontal(
                    rrb.Spatial2DView(name="left (cam1)", origin="left"),
                    rrb.Spatial2DView(name="right (cam2)", origin="right"),
                ),
                rrb.TextDocumentView(name="status", origin="status"),
                row_shares=[3, 1],
            ),
            collapse_panels=True,
        ),
        clear=("left", "right", "status"),
    )
    for host in [h for h in args.hosts.split(",") if h]:
        print(f"[較正] 画面: {viewer_hub.viewer_url(host)}", flush=True)
    return rr


def status(rr, text: str) -> None:
    rr.log("status", rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN), static=True)


def capture(args, rr, session: pathlib.Path) -> None:
    from rtsp_grabber import LatestFrameGrabber

    (session / "images/left").mkdir(parents=True, exist_ok=True)
    (session / "images/right").mkdir(parents=True, exist_ok=True)
    # 2 台を同時に待ち、届いた方から画を出す（前は左がそろうまで右を待たず、2 台そろうまで画面が黒かった）
    gl = LatestFrameGrabber(args.left, long_side=CALIB_LONG_SIDE, history_len=30)
    gr = LatestFrameGrabber(args.right, long_side=CALIB_LONG_SIDE, history_len=30)
    while True:
        L, R = gl.latest(), gr.latest()
        if L is not None:
            _log_jpeg(rr, "left", L[2])
        if R is not None:
            _log_jpeg(rr, "right", R[2])
        status(rr, "## waiting for both phones\n\n" + sending_lines(L, R))
        if L is not None and R is not None and is_live(L) and is_live(R):
            break
        time.sleep(0.5)
    h, w = gl.latest()[2].shape[:2]
    print(f"[較正] 景色の組を集める（{args.pairs} 組・{w}x{h}）", flush=True)
    col = SceneCollector(args.pairs)
    reasons: collections.Counter = collections.Counter()
    report_at = time.time() + 10
    last_seq = -1
    while not col.done:
        time.sleep(STEP_S)
        if time.time() >= report_at:
            print(f"[較正] この 10 秒: {dict(reasons)}（拾った {len(col.taken)}/{args.pairs}）", flush=True)
            reasons.clear()
            report_at = time.time() + 10
        tl, seq, img_l = gl.latest()
        R = gr.latest()
        if not is_live((tl, seq, img_l)) or R is None or not is_live(R):
            status(rr, f"## views {len(col.taken)} / {args.pairs}\n\n**a phone stopped sending**\n\n" + sending_lines(gl.latest(), R))
            continue
        if seq == last_seq:
            continue
        last_seq = seq
        near = gr.frame_near(tl + args.offset)
        if near is None or abs(near[0] - (tl + args.offset)) > PAIR_MAX_DT_S:
            reasons["no right frame near in time"] += 1
            continue
        img_r = fit_to(near[2], (w, h))
        why = col.offer(img_l, img_r, time.time())
        reasons[why] += 1
        if why == "taken":
            k = len(col.taken)
            cv2.imwrite(str(session / f"images/left/{k:03d}.png"), img_l)
            cv2.imwrite(str(session / f"images/right/{k:03d}.png"), img_r)
            print(f"[較正] {k}/{args.pairs} 組目を拾った", flush=True)
        vis_l, vis_r = img_l.copy(), img_r.copy()
        if why == "taken":
            for v in (vis_l, vis_r):
                cv2.rectangle(v, (0, 0), (v.shape[1] - 1, v.shape[0] - 1), (0, 255, 0), 16)
        _log_jpeg(rr, "left", vis_l)
        _log_jpeg(rr, "right", vis_r)
        ml, mr = col.motion
        status(
            rr,
            f"## views {len(col.taken)} / {args.pairs}\n\n**{why}**\n\n"
            f"motion left {ml:.1f} / right {mr:.1f} px (still when both < {STILL_SHIFT_PX})\n\n"
            "Keep the two phones taped together. Point them at a textured scene (desk, shelves, posters), "
            "hold still for a moment, then turn / move a little. Mix near and far, left / right / up / down.",
        )
    gl.close()
    gr.close()


# ---- 解く ----


def run_colmap(session: pathlib.Path) -> "object":
    import pycolmap

    img_dir, out = session / "images", session / "colmap"
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    db = out / "database.db"
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "SIMPLE_RADIAL"  # スマホの動画はひずみが小さい。f・cx・cy・k1
    print("[較正] 特徴を取る（SIFT）", flush=True)
    pycolmap.extract_features(db, img_dir, camera_mode=pycolmap.CameraMode.PER_FOLDER, reader_options=reader)
    print("[較正] 全部の組で対応を取る", flush=True)
    pycolmap.match_exhaustive(db)
    print("[較正] 復元する（COLMAP）", flush=True)
    maps = pycolmap.incremental_mapping(db, img_dir, out)
    if not maps:
        raise SystemExit("[較正] 復元できなかった（模様の多い所で、向きを変えながら集め直す）")
    return max(maps.values(), key=lambda r: r.num_reg_images())


def camera_of(rec, prefix: str):
    for img in rec.images.values():
        if img.name.startswith(prefix):
            return rec.cameras[img.camera_id]
    raise SystemExit(f"[較正] {prefix} の画が 1 枚も復元に入っていない")


def k_and_d(cam) -> tuple[np.ndarray, np.ndarray]:
    f, cx, cy, k = cam.params  # SIMPLE_RADIAL
    return np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]]), np.array([k, 0, 0, 0, 0], float)


def pair_poses(rec) -> tuple[list, list, list[str]]:
    """同時に写した組ごとの「左 → 右」の (R, t)（COLMAP の大きさ）"""
    by_name = {img.name: img for img in rec.images.values() if img.has_pose}
    Rs, ts, names = [], [], []
    for name, il in sorted(by_name.items()):
        if not name.startswith("left/"):
            continue
        ir = by_name.get("right/" + name.split("/", 1)[1])
        if ir is None:
            continue
        Tl, Tr = np.vstack([il.cam_from_world().matrix(), [0, 0, 0, 1]]), np.vstack([ir.cam_from_world().matrix(), [0, 0, 0, 1]])
        rel = Tr @ np.linalg.inv(Tl)
        Rs.append(rel[:3, :3])
        ts.append(rel[:3, 3])
        names.append(name)
    return Rs, ts, names


def matched_points(rec) -> tuple[np.ndarray, np.ndarray]:
    """同じ組の左と右に写った同じ 3D 点の画素の組（平行化の確かめ用）"""
    frame_of = {}
    for img in rec.images.values():
        side, rest = img.name.split("/", 1)
        frame_of[img.image_id] = (side, rest)
    lp, rp = [], []
    for p in rec.points3D.values():
        seen: dict[tuple[str, str], np.ndarray] = {}
        for el in p.track.elements:
            side, rest = frame_of[el.image_id]
            seen[(side, rest)] = rec.images[el.image_id].points2D[el.point2D_idx].xy
        for (side, rest), xy in seen.items():
            if side == "left" and ("right", rest) in seen:
                lp.append(xy)
                rp.append(seen[("right", rest)])
    return np.asarray(lp, float).reshape(-1, 2), np.asarray(rp, float).reshape(-1, 2)


def solve(session: pathlib.Path, baseline_mm: float, left_url: str, right_url: str, offset: float) -> dict:
    rec = run_colmap(session)
    print(f"[較正] 復元: 画 {rec.num_reg_images()} 枚・点 {rec.num_points3D()}・再投影の誤差 {rec.compute_mean_reprojection_error():.3f} px", flush=True)
    K1, D1 = k_and_d(camera_of(rec, "left/"))
    K2, D2 = k_and_d(camera_of(rec, "right/"))
    cam_l = camera_of(rec, "left/")
    size = (int(cam_l.width), int(cam_l.height))
    Rs, ts, _ = pair_poses(rec)
    R, t, n_used, spread = robust_relative_pose(Rs, ts)
    rigid = spread["rot_spread_deg"] < RIGID_ROT_DEG and spread["dir_spread_deg"] < RIGID_DIR_DEG
    print(f"[較正] 2 台の固定の目安: 組ごとの向きのばらつき {spread['rot_spread_deg']:.2f} 度"
          f"（{RIGID_ROT_DEG} 度未満で固定）・基線の向き {spread['dir_spread_deg']:.2f} 度・長さ {100 * spread['len_spread']:.0f}%"
          + ("" if rigid else "  ⚠️ 2 台が固定されていない（互いに動いた）見込み。固定して集め直す"), flush=True)
    scale = (baseline_mm / 1000.0) / float(np.linalg.norm(t))  # COLMAP の大きさ → メートル
    calib = rectify(K1, D1, K2, D2, R, t * scale, size)
    lp, rp = matched_points(rec)
    if calib["swapped"]:
        lp, rp = rp, lp
    dy = epipolar_dy(calib, lp, rp) if len(lp) else np.array([np.nan])
    calib.update(
        epipolar_err_px=float(np.median(dy)),
        epipolar_err_p90_px=float(np.percentile(dy, 90)),
        n_matched_points=int(len(lp)),
        reproj_err_px=float(rec.compute_mean_reprojection_error()),
        n_pairs=int(len(Rs)),
        n_pairs_used=int(n_used),
        rig_rot_spread_deg=spread["rot_spread_deg"],
        rig_dir_spread_deg=spread["dir_spread_deg"],
        rig_len_spread=spread["len_spread"],
        rig_ok=bool(rigid),
        method="scene (COLMAP SIMPLE_RADIAL)",
        baseline_source=f"ruler {baseline_mm:.1f} mm",
        fx_left=float(calib["K1"][0, 0]),
        fx_right=float(calib["K2"][0, 0]),
    )
    extra = {
        "left_stream": right_url if calib["swapped"] else left_url,
        "right_stream": left_url if calib["swapped"] else right_url,
        "sync_offset_s": -offset if calib["swapped"] else offset,
    }
    save_calib(session / "calib.json", calib, extra)
    rectified_check(calib, session)
    return calib


def rectified_check(calib: dict, session: pathlib.Path) -> None:
    lefts = sorted((session / "images/left").glob("*.png"))
    if not lefts:
        return
    lp = lefts[len(lefts) // 2]
    rp = session / "images/right" / lp.name
    il, ir = cv2.imread(str(lp)), cv2.imread(str(rp))
    if calib["swapped"]:
        il, ir = ir, il
    m1x, m1y, m2x, m2y = rectify_maps(calib)
    both = np.hstack([cv2.remap(il, m1x, m1y, cv2.INTER_LINEAR), cv2.remap(ir, m2x, m2y, cv2.INTER_LINEAR)])
    for y in range(0, both.shape[0], max(20, both.shape[0] // 24)):
        cv2.line(both, (0, y), (both.shape[1] - 1, y), (0, 255, 255), 1)
    cv2.imwrite(str(session / "rectified_check.jpg"), both)


def summary_text(calib: dict) -> dict:
    return {
        "baseline_m": round(calib["baseline_m"], 4),
        "fx_left": round(calib["fx_left"], 1),
        "fx_right": round(calib["fx_right"], 1),
        "epipolar_err_median_px": round(calib["epipolar_err_px"], 3),
        "epipolar_err_p90_px": round(calib["epipolar_err_p90_px"], 3),
        "reproj_err_px": round(calib["reproj_err_px"], 3),
        "pairs_used": f"{calib['n_pairs_used']}/{calib['n_pairs']}",
        "rig_rot_spread_deg": round(calib["rig_rot_spread_deg"], 2),
        "rig_ok": calib["rig_ok"],
        "swapped": calib["swapped"],
        "vertical_baseline": calib["vertical_baseline"],
    }


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["capture", "solve"])
    p.add_argument("session", nargs="?", default="", help="solve: 集めた組のフォルダ")
    p.add_argument("--left", default="rtsp://127.0.0.1:8554/cam1", help="左に置いたカメラ（iPhone）")
    p.add_argument("--right", default="rtsp://127.0.0.1:8554/cam2", help="右に置いたカメラ（Pixel）")
    p.add_argument("--baseline-mm", type=float, default=0.0, help="2 台のレンズの中心の間の距離（定規）")
    p.add_argument("--pairs", type=int, default=30)
    p.add_argument("--offset", type=float, default=0.0, help="右が左より遅れて届く秒（measure_sync.py）")
    p.add_argument("--out", default=str(pathlib.Path.home() / "work_inoue/phone_slam/calib"))
    p.add_argument("--hosts", default="127.0.0.1")
    p.add_argument("--linger-s", type=float, default=600.0)
    args = p.parse_args()

    rr = None
    if args.mode == "capture":
        rr = start_viewer(args)
        session = pathlib.Path(args.out).expanduser() / (time.strftime("%m%d_%H%M") + "_scene")
        capture(args, rr, session)
    else:
        session = pathlib.Path(args.session).expanduser().resolve()
    if args.baseline_mm <= 0:
        print(f"[較正] 集め終わった: {session}。レンズの間の距離を測って solve に --baseline-mm で渡す", flush=True)
        if rr is not None:
            status(rr, "## views collected\n\nMeasure the distance between the two lenses and run solve.")
            time.sleep(args.linger_s)
        return
    if rr is not None:
        status(rr, "## solving with COLMAP (a few minutes) ...")
    calib = solve(session, args.baseline_mm, args.left, args.right, args.offset)
    s = summary_text(calib)
    print("[較正] 結果: " + json.dumps(s, ensure_ascii=False), flush=True)
    print(f"[較正] 保存した: {session}/calib.json・rectified_check.jpg", flush=True)
    if rr is not None:
        notes = []
        if calib["vertical_baseline"]:
            notes.append("**cameras look stacked vertically - place them side by side**")
        if not calib["rig_ok"]:
            notes.append("**the two phones moved relative to each other - tape them together firmly and collect again**")
        if calib["epipolar_err_px"] > 1.0:
            notes.append("**epipolar error > 1 px - collect again (more texture, more varied views)**")
        status(rr, "## calibration done\n\n" + "\n".join(f"- {k}: {v}" for k, v in s.items()) + ("\n\n" + "\n".join(notes) if notes else ""))
        il = cv2.imread(str(session / "rectified_check.jpg"))
        if il is not None:
            half = il.shape[1] // 2
            _log_jpeg(rr, "left", il[:, :half])
            _log_jpeg(rr, "right", il[:, half:])
        time.sleep(args.linger_s)


if __name__ == "__main__":
    main()
