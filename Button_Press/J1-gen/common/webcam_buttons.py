"""前方の Web カメラ（Innomaker U20CAM、深度なし）から、遠くでも ▲▼ の位置を推定する。立つ位置まで寄るときの「おまけ」。

    cam = WebcamModel.from_config(load_config("webcam.yaml"))       # 焦点距離・取り付け（較正の値）
    pair = find_pair(detector.detect(rgb), cfg)                       # 画像の ▲▼（見つからなければ None）
    p = from_plane(cam, pair.mid_px, face.normal, face.point)        # 頭カメラの柱の面との交点（pelvis 座標）
    p = from_size(cam, pair, cfg)                                     # ボタンの大きさ・間隔から距離
    p = triangulate(cam, uv_before, uv_after, motion)                 # 動いた前後の見え方の差（視差）

なぜ: 頭カメラは下を向いていて、柱から約 50 cm より離れるとボタンが画像の上に外れる。前方の Web カメラはもっと水平に
近い向きなので、遠くからボタンが写る（はず）。ただし深度が無いので、距離を別の方法で出す。3 つの方法を用意して比べる:
1. 柱の面との交点（from_plane）: Web カメラの画素の方向の線と、頭カメラの深度で求めた柱の面（common/column_face.py）が
   交わる点。距離は頭カメラが決めるので、Web カメラの誤差は方向だけに効く（取り付けの向きが 1° ずれると 1 m 先で約 1.7 cm）
2. 大きさから（from_size）: ボタンの直径（3.5 cm）と ▲▼ の間隔（7 cm）を先に測っておき、画像での大きさとの比で距離を出す。
   合成のテスト（焦点距離 500 画素、1 m 先、中心と直径に ±2 画素の雑音）では、どちらも距離の誤差の中央値が 4〜5%
3. 視差（triangulate）: 1 回動いた前後で同じボタンを見て、2 本の線の交点を出す。動いた量は頭カメラ（柱の面の見え方の変化。
   common/column_face.py の motion_between）で測る。横に動いたときによく効き、前に動いただけ（線がほぼ重なる）だと弱い

Web カメラが無い・映像が来ない・較正していない日は使わない（寄る流れは頭カメラだけで動く）。
座標: Web カメラの光学座標は x 右・y 下・z 前（受け取って回したあとの画像の向き）。pelvis 座標は x 前・y 左・z 上。
⚠️ 焦点距離と取り付けは未較正（configs/webcam.yaml。real/webcam_check.py --collect / --fit で求める）。実機では一度も試していない。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .camera_geometry import rpy_to_matrix

# 光学座標（x 右・y 下・z 前）の軸を、カメラの体（x 前・y 左・z 上）で表したもの（common/camera_geometry.py と同じ）
R_BODY_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


class WebcamNotReady(RuntimeError):
    """較正していない・映像が無いなど、Web カメラを使えない（理由は文で持つ）。"""


@dataclass
class WebcamModel:
    """ピンホールの Web カメラ。pose は pelvis 座標でのカメラの体（x 前・y 左・z 上）の位置と向き（rpy [rad]）。"""

    fx: float
    fy: float
    cx: float
    cy: float
    xyz: np.ndarray
    rpy: np.ndarray

    @classmethod
    def from_config(cls, cfg: dict[str, Any]) -> "WebcamModel":
        c = cfg["camera"]
        if c.get("fx") is None or c.get("xyz") is None:
            raise WebcamNotReady("Web カメラの焦点距離・取り付けが未較正（configs/webcam.yaml。real/webcam_check.py --fit）")
        w, h = (int(v) for v in c["size"])
        return cls(float(c["fx"]), float(c.get("fy") or c["fx"]), float(c.get("cx") or (w - 1) / 2.0),
                   float(c.get("cy") or (h - 1) / 2.0), np.asarray(c["xyz"], dtype=float),
                   np.radians(np.asarray(c["rpy_deg"], dtype=float)))

    @property
    def R(self) -> np.ndarray:
        """光学座標 → pelvis 座標の回転。"""
        return rpy_to_matrix(*self.rpy) @ R_BODY_OPTICAL

    def ray(self, uv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """画素 (u, v) の方向の線（pelvis 座標の原点と単位ベクトル）。"""
        d = np.array([(uv[0] - self.cx) / self.fx, (uv[1] - self.cy) / self.fy, 1.0])
        d = self.R @ d
        return self.xyz.copy(), d / np.linalg.norm(d)

    def project(self, p: np.ndarray) -> np.ndarray:
        """pelvis 座標の点 → 画素 (u, v)。カメラの後ろなら nan。"""
        q = self.R.T @ (np.asarray(p, dtype=float) - self.xyz)
        if q[2] <= 1e-6:
            return np.array([np.nan, np.nan])
        return np.array([self.fx * q[0] / q[2] + self.cx, self.fy * q[1] / q[2] + self.cy])


@dataclass
class PairPx:
    """画像の ▲▼（一般用でも車いす用でも、押す組の 2 つ）。中心 (u, v) と、見えている直径 [px]（縦横の大きいほう）。"""

    up_px: np.ndarray
    down_px: np.ndarray
    up_diam_px: float
    down_diam_px: float

    @property
    def mid_px(self) -> np.ndarray:
        return (self.up_px + self.down_px) / 2.0

    @property
    def spacing_px(self) -> float:
        return float(np.linalg.norm(self.up_px - self.down_px))


def find_pair(cands: list[Any], cfg: dict[str, Any]) -> PairPx | None:
    """画像の候補（common/elevator_buttons.py の Candidate2D）から、縦に並んだ ▲▼ の組を 1 つ選ぶ。

    深度が無いので、「間隔 ÷ 直径」が実物（▲▼ の間隔 7 cm ÷ 直径 3.5 cm = 2）に近いこと、縦に並ぶこと、記号の順（上が ▲）で選ぶ。
    押す組（button_set）が wheelchair なら、条件に合う組のうち画像でいちばん下、general ならいちばん上。
    """
    g = cfg["buttons"]
    want = float(g["spacing_m"]) / float(g["diameter_m"])
    tol = float(g["ratio_tolerance"])
    items = []
    for c in cands:
        x0, y0, x1, y1 = c.bbox
        items.append((np.array([(x0 + x1) / 2.0, (y0 + y1) / 2.0]), float(max(x1 - x0, y1 - y0)), c.arrow))
    pairs = []
    for i, (pa, da, aa) in enumerate(items):
        for j, (pb, db, ab) in enumerate(items):
            if j == i or pb[1] <= pa[1]:
                continue  # a が上
            dv = pb - pa
            if abs(dv[0]) > 0.5 * max(da, db):
                continue  # 縦に並んでいない
            ratio = float(np.linalg.norm(dv)) / (0.5 * (da + db))
            if abs(ratio - want) > tol * want:
                continue
            if (aa is not None and aa != "up") or (ab is not None and ab != "down"):
                continue
            pairs.append(PairPx(pa, pb, da, db))
    if not pairs:
        return None
    pairs.sort(key=lambda p: p.mid_px[1])
    return pairs[-1] if str(g.get("button_set", "general")) == "wheelchair" else pairs[0]


def from_plane(cam: WebcamModel, uv: np.ndarray, normal: np.ndarray, point: np.ndarray) -> np.ndarray | None:
    """画素 uv の方向の線と、平面（pelvis 座標の法線と 1 点）の交点。平行・後ろなら None。"""
    o, d = cam.ray(uv)
    den = float(np.asarray(normal) @ d)
    if abs(den) < 1e-6:
        return None
    t = float(np.asarray(normal) @ (np.asarray(point) - o)) / den
    return o + t * d if t > 0 else None


def from_size(cam: WebcamModel, pair: PairPx, cfg: dict[str, Any]) -> dict[str, Any]:
    """ボタンの大きさ・間隔から、▲▼ の中点の位置（pelvis 座標）を 2 通り出す（直径から / 間隔から）。

    カメラからの距離 ≈ 焦点距離 × 実物の大きさ ÷ 画像の大きさ（斜めから見ると横幅が縮むので、直径は縦横の大きいほう、
    間隔は縦に並んだ 2 つの中心の距離を使う）。
    """
    g = cfg["buttons"]
    f = 0.5 * (cam.fx + cam.fy)
    out: dict[str, Any] = {}
    o, d = cam.ray(pair.mid_px)
    diam = 0.5 * (pair.up_diam_px + pair.down_diam_px)
    if diam > 0:
        r = f * float(g["diameter_m"]) / diam
        out["by_diameter"] = o + r * d
    if pair.spacing_px > 0:
        r = f * float(g["spacing_m"]) / pair.spacing_px
        out["by_spacing"] = o + r * d
    return out


def body_motion_matrix(motion: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    """動く前の体の座標で (dx, dy, dθ) 動いたとき、動いたあとの pelvis 座標 → 動く前の pelvis 座標の (R, t)。"""
    dx, dy, dth = (float(v) for v in motion)
    c, s = np.cos(dth), np.sin(dth)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]), np.array([dx, dy, 0.0])


def triangulate(cam: WebcamModel, uv_before: np.ndarray, uv_after: np.ndarray,
                motion: tuple[float, float, float]) -> tuple[np.ndarray, float] | None:
    """動く前と後に見た同じ点の位置（動いたあとの pelvis 座標）と、2 本の線のすき間 [m]。線がほぼ平行なら None。

    動いた量 motion は、動く前の体の座標での (前 dx, 左 dy [m], 左回り dθ [rad])。2 本の線の最も近い 2 点の中点を返す。
    """
    R, t = body_motion_matrix(motion)
    o1, d1 = cam.ray(uv_before)                 # 動く前の pelvis 座標
    o2a, d2a = cam.ray(uv_after)                # 動いたあとの pelvis 座標
    o2, d2 = R @ o2a + t, R @ d2a               # → 動く前の座標へ
    w = o1 - o2
    b = float(d1 @ d2)
    den = 1.0 - b * b
    if den < 1e-6:
        return None
    s1 = (b * float(d2 @ w) - float(d1 @ w)) / den
    s2 = (float(d2 @ w) - b * float(d1 @ w)) / den
    if s1 <= 0 or s2 <= 0:
        return None
    p1, p2 = o1 + s1 * d1, o2 + s2 * d2
    mid = 0.5 * (p1 + p2)
    return R.T @ (mid - t), float(np.linalg.norm(p1 - p2))  # 動いたあとの座標へ


def looming_range(diam_before_px: float, diam_after_px: float, toward_m: float) -> float | None:
    """カメラに向かってまっすぐ toward_m 近づいたとき、大きさの比から、近づいたあとのカメラからの距離。

    大きさは距離に反比例するので、d_after = toward × s_before ÷ (s_after − s_before)。大きさが変わらなければ None。
    """
    ds = float(diam_after_px) - float(diam_before_px)
    if ds <= 1e-6 or toward_m <= 0:
        return None
    return float(toward_m) * float(diam_before_px) / ds


# ---- 較正（焦点距離と取り付け） -------------------------------------------------------------


def fit_camera(pairs: list[tuple[np.ndarray, np.ndarray]], cam0: WebcamModel, fit_xyz: bool = False,
               iterations: int = 30) -> tuple[WebcamModel, float]:
    """頭カメラで測ったボタンの位置（pelvis 座標）と、同じボタンの Web カメラの画素の組から、焦点距離と向き（と位置）を求める。

    pairs: [(p_pelvis (3,), uv (2,)), ...]。焦点距離（fx = fy）と rpy の 4 つ（fit_xyz なら位置の 3 つも）を、画素の当てはめの
    ずれが小さくなるように Gauss-Newton 法（数値微分）で求める。返り値は (求めたカメラ, 画素のずれの RMS)。
    位置は、巻き尺で測った値（cam0.xyz）のままにするのがふつう（ボタンが遠くにしか無いと、位置と向きを分けにくい）。
    """
    P = np.array([p for p, _ in pairs], dtype=float)
    UV = np.array([uv for _, uv in pairs], dtype=float)

    def unpack(x: np.ndarray) -> WebcamModel:
        xyz = x[4:7] if fit_xyz else cam0.xyz
        return WebcamModel(float(x[0]), float(x[0]), cam0.cx, cam0.cy, np.asarray(xyz, dtype=float), x[1:4].copy())

    def resid(x: np.ndarray) -> np.ndarray:
        cam = unpack(x)
        pr = np.array([cam.project(p) for p in P])
        r = (pr - UV).ravel()
        return np.where(np.isfinite(r), r, 1e3)

    x = np.concatenate([[cam0.fx], cam0.rpy, cam0.xyz if fit_xyz else []]).astype(float)
    steps = np.concatenate([[1.0], np.full(3, 1e-4), np.full(3, 1e-4) if fit_xyz else []])
    lam = 1e-3
    r = resid(x)
    for _ in range(iterations):
        J = np.stack([(resid(x + e) - r) / h for e, h in ((np.eye(len(x))[k] * steps[k], steps[k]) for k in range(len(x)))],
                     axis=1)
        A = J.T @ J
        g = J.T @ r
        dx = -np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), g)
        x_new = x + dx
        r_new = resid(x_new)
        if r_new @ r_new < r @ r:
            x, r, lam = x_new, r_new, lam * 0.3
            if np.abs(dx).max() < 1e-9:
                break
        else:
            lam *= 10.0
    return unpack(x), float(np.sqrt(np.mean(r ** 2)))


# ---- 映像の受け取り ------------------------------------------------------------------


class WebcamStream:
    """Mapping 班の配信（innomaker_camera_server.py、ZMQ）から Web カメラの映像を受け取り、回して RGB にする。

    read() は (RGB の画像, 受け取ってからの秒数) か、届かなければ None。配信は最新の 1 枚だけを持つ（CONFLATE）ので、
    受け取った直後の画像の古さは 0 とみなす（送った側の時刻は配信に無い）。
    """

    ROTATIONS = {"cw": 0, "ccw": 2, "180": 1}  # cv2.ROTATE_90_CLOCKWISE / ROTATE_180 / ROTATE_90_COUNTERCLOCKWISE

    def __init__(self, host: str, cfg: dict[str, Any]) -> None:
        from .perception_bridge import perception

        s = cfg["stream"]
        self.rotate = str(s.get("rotate") or "none")
        self.source = perception("camera").ZmqFrameSource(host, int(s["port"]), camera_name=s.get("camera_name") or "",
                                                          timeout_ms=int(s["timeout_ms"]), rgb=not bool(s["bgr"]))

    def open(self) -> None:
        self.source.open()

    def close(self) -> None:
        self.source.close()

    def read(self) -> tuple[np.ndarray, float] | None:
        import cv2

        bgr = self.source.read()
        if bgr is None:
            return None
        if self.rotate in self.ROTATIONS:
            bgr = cv2.rotate(bgr, self.ROTATIONS[self.rotate])
        return bgr[:, :, ::-1].copy(), 0.0
