"""ボタンが見えない遠さ（柱まで約 1 m 〜 55 cm）から、頭カメラの深度で柱の面（距離・向き・左右の端）を求める。

    finder = FaceFinder.from_config(robot_cfg, load_config("approach.yaml")["face"])
    face = finder.find(depth_m, K, q_waist, imu_quat)   # 見つからなければ FaceNotFound
    face.distance_m, face.yaw_rad, face.left_edge_m      # pelvis 基準

なぜ: 頭カメラは下に約 48°（実機は 51〜54°）傾いていて、縦の画角は約 43° しかない。柱から離れると、ボタン
（床から 0.87〜1.24 m。カメラは床から約 1.2 m）は画像の上に外れる（ラボの車いす用でも約 50 cm より遠いと切れる）。
一方、柱の面の下のほうと床は写る（1 m 離れると、床から約 0.6 m より下の面と、その手前の床）。そこで、ボタンを
使わずに面の距離と向きを出し、寄るのに使う。ボタンが見えてからの合わせ込みは common/elevator_buttons.py。

流れ（座標は pelvis。x 前・y 左・z 上）:
1. 深度を間引いて点にする
2. 床を探す: 法線がほぼ真上の平面だけを RANSAC で探す。見つかれば、床の法線が IMU の真上と一致するように
   点をカメラの位置のまわりに回す（首の向きのずれを、その場で直す。11/3 は URDF 47.6° に対して実機 51〜54°。
   直さないと、1 m 先の面の距離が数 cm ずれる）。近いと床が写らない（約 50 cm より近く）ので、そのときは前のフレームで
   床から求めた直し方を使う（FaceFinder が覚えておく。無ければ直さない。55 cm で 5.4° ずれたままだと約 4 cm 遠く出る）
3. 立った面（法線がほぼ水平で、こちらを向いたもの）を RANSAC で数枚探す。前方の通り道（左右 ±corridor_half_m）に
   かかるもののうち、いちばん近い面を柱の面とする（柱が壁より出ていれば柱、扉の奥や後ろの壁ではない）
4. 選んだ面に、垂直という条件つきで当てはめ直し、距離・向きを出す
5. 面に乗った点を面に沿って並べ、つながった範囲の両端を柱の端とする。画像の左右の縁で切れていれば、端は分からない（None）

画像処理は numpy だけを使う（RGB は使わない。黒い柱でも深度は取れる前提。⚠️ 実機の遠くからの深度は未確認）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .camera_geometry import HeadCameraTransform
from .elevator_buttons import calibration_offset
from .head_tilt import imu_up_in_pelvis


class FaceNotFound(RuntimeError):
    """柱の面が見つからない（理由は文で持つ）。"""


@dataclass
class Plane:
    normal: np.ndarray  # 単位ベクトル
    point: np.ndarray   # 平面上の 1 点
    inliers: np.ndarray  # 点の添字（bool）


@dataclass
class ColumnFace:
    normal: np.ndarray        # 面の法線（pelvis 座標、水平、こちらを向く）
    point: np.ndarray         # pelvis の原点から面に下ろした足（pelvis 座標）
    distance_m: float         # pelvis の原点から面までの水平の距離
    yaw_rad: float            # 面に正対するのに回る角度（+ は左回り。0 なら正対）
    left_edge_m: float | None   # 面の左の端（足から面に沿って左へ測る。+ が左）。画像で切れていれば None
    right_edge_m: float | None  # 面の右の端（同じ向きで測る。ふつうは −）
    n_points: int
    floor_found: bool
    pitch_correction_deg: float  # 首の向きを直した量（直していなければ 0）
    floor_z_m: float | None      # 直したあとの床の高さ（pelvis 座標。立っていれば約 −0.73）
    others_m: list[float] = field(default_factory=list)  # 選ばなかった立った面の距離（記録用）
    correction: np.ndarray = field(default_factory=lambda: np.eye(3))  # 直すのに使った回転（次のフレームに渡せる）
    correction_from: str = "none"  # floor（このフレームの床）/ previous（前のフレームの床）/ none

    def summary(self) -> str:
        def edge(v: float | None) -> str:
            return "?" if v is None else f"{v * 100:+.0f}"

        if self.floor_found:
            floor = f"床あり（首の向きを {self.pitch_correction_deg:.1f}° 直した、床 z {self.floor_z_m:+.2f} m）"
        elif self.correction_from == "previous":
            floor = f"床なし（前の床の直し方で {self.pitch_correction_deg:.1f}° 直した）"
        else:
            floor = "床なし（直していない）"
        return (f"柱の面まで {self.distance_m * 100:.1f} cm、向き {np.degrees(self.yaw_rad):+.1f}°、"
                f"端 左 {edge(self.left_edge_m)} / 右 {edge(self.right_edge_m)} cm、{self.n_points} 点、{floor}")


def _ransac(points: np.ndarray, accept: Any, threshold_m: float, iterations: int,
            rng: np.random.Generator) -> np.ndarray | None:
    """accept(法線, 点) を満たす平面のうち、乗る点がいちばん多いものの inlier（bool）。無ければ None。"""
    n_pts = len(points)
    if n_pts < 3:
        return None
    best: np.ndarray | None = None
    best_count = 0
    for _ in range(iterations):
        p = points[rng.choice(n_pts, 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if not accept(n, p[0]):
            continue
        inl = np.abs((points - p[0]) @ n) < threshold_m
        c = int(inl.sum())
        if c > best_count:
            best, best_count = inl, c
    return best


def _rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """単位ベクトル a を b に重ねる最小の回転。"""
    v = np.cross(a, b)
    s = float(np.linalg.norm(v))
    c = float(np.clip(a @ b, -1.0, 1.0))
    if s < 1e-12:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s ** 2)


def _segment_around(s: np.ndarray, center: float, bin_m: float, gap_bins: int) -> tuple[float, float]:
    """面に沿った座標 s の点が、center からつながっている範囲 (下端, 上端)。gap_bins 個までの空きはつながっているとみなす。"""
    lo, hi = float(s.min()), float(s.max())
    nb = max(1, int(np.ceil((hi - lo) / bin_m)) + 1)
    counts = np.bincount(((s - lo) / bin_m).astype(int), minlength=nb)
    filled = counts > 0
    c = int(np.clip((center - lo) / bin_m, 0, nb - 1))
    if not filled[c]:
        # center の真上に点が無いとき（ボタンの穴など）は、いちばん近い点のある区画から始める
        idx = np.flatnonzero(filled)
        c = int(idx[np.argmin(np.abs(idx - c))])
    a = b = c
    gap = 0
    while a - 1 >= 0 and gap <= gap_bins:
        a -= 1
        gap = 0 if filled[a] else gap + 1
    a += gap
    gap = 0
    while b + 1 < nb and gap <= gap_bins:
        b += 1
        gap = 0 if filled[b] else gap + 1
    b -= gap
    return lo + a * bin_m, lo + (b + 1) * bin_m


def find_face(depth_m: np.ndarray, K: np.ndarray, rot: np.ndarray, pos: np.ndarray, up: np.ndarray,
              cfg: dict[str, Any], prior_correction: np.ndarray | None = None) -> ColumnFace:
    """深度（m）から柱の面を求める。rot・pos は光学座標 → pelvis 座標（p_pelvis = rot x + pos）、up は pelvis 座標の真上。

    prior_correction: 床が写らないときに使う、前のフレームの直し方（ColumnFace.correction）。
    """
    step = int(cfg["step_px"])
    z = np.asarray(depth_m, dtype=np.float64)[::step, ::step]
    h, w = np.asarray(depth_m).shape
    v, u = np.mgrid[0:h:step, 0:w:step]
    ok = (z > float(cfg["min_depth_m"])) & (z < float(cfg["max_depth_m"]))
    zz, uu, vv = z[ok], u[ok].astype(float), v[ok].astype(float)
    if len(zz) < int(cfg["wall"]["min_points"]):
        raise FaceNotFound(f"深度の点が足りない（{len(zz)} 点）")
    pc = np.stack([(uu - K[0, 2]) * zz / K[0, 0], (vv - K[1, 2]) * zz / K[1, 1], zz], axis=1)
    pos = np.asarray(pos, dtype=float)
    P = pc @ np.asarray(rot, dtype=float).T + pos
    up = np.asarray(up, dtype=float)
    up = up / np.linalg.norm(up)
    rng = np.random.default_rng(int(cfg.get("seed", 0)))
    iters = int(cfg["ransac_iterations"])

    # 2. 床
    fc = cfg["floor"]
    cos_floor = np.cos(np.radians(float(fc["max_tilt_deg"])))
    z_lo, z_hi = (float(x) for x in fc["z_range_m"])

    def floor_like(n: np.ndarray, p0: np.ndarray) -> bool:
        return abs(float(n @ up)) > cos_floor and z_lo < float(p0 @ up) < z_hi

    floor_inl = _ransac(P, floor_like, float(fc["threshold_m"]), iters, rng)
    floor_found = floor_inl is not None and int(floor_inl.sum()) >= int(fc["min_points"])
    correction_deg, floor_z = 0.0, None
    R, corr_from = np.eye(3), "none"
    if floor_found:
        fp = P[floor_inl]
        c = fp.mean(axis=0)
        n_floor = np.linalg.svd(fp - c, full_matrices=False)[2][-1]
        if n_floor @ up < 0:
            n_floor = -n_floor
        correction_deg = float(np.degrees(np.arccos(np.clip(n_floor @ up, -1.0, 1.0))))
        if correction_deg > float(fc["max_correction_deg"]):
            raise FaceNotFound(f"床から求めた首の向きのずれが {correction_deg:.1f}° と大きすぎる"
                               f"（上限 {float(fc['max_correction_deg']):.0f}°）。首の向きを確かめる（real/check_head_tilt.py）")
        if fc.get("correct_pitch", True):
            R, corr_from = _rotation_between(n_floor, up), "floor"
            P = (P - pos) @ R.T + pos
            c = R @ (c - pos) + pos
        else:
            correction_deg = 0.0
        floor_z = float(c @ up)
    else:
        floor_inl = np.zeros(len(P), bool)
        if prior_correction is not None and fc.get("correct_pitch", True):
            R, corr_from = np.asarray(prior_correction, dtype=float), "previous"
            P = (P - pos) @ R.T + pos
            correction_deg = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))))

    # 3. 立った面
    wc = cfg["wall"]
    sin_vert = np.sin(np.radians(float(wc["vertical_tol_deg"])))
    cos_yaw = np.cos(np.radians(float(wc["max_yaw_deg"])))
    fwd = np.array([1.0, 0.0, 0.0]) - up * up[0]
    fwd /= np.linalg.norm(fwd)
    height = P @ up
    base = floor_z if floor_z is not None else z_hi
    rest = ~floor_inl & (height > base + float(wc["min_above_floor_m"]))

    def wall_like(n: np.ndarray, p0: np.ndarray) -> bool:
        if abs(float(n @ up)) > sin_vert:
            return False
        if n @ (pos - p0) < 0:
            n = -n  # カメラに向ける
        return float(-n @ fwd) > cos_yaw

    left = np.array([0.0, 1.0, 0.0]) - up * up[1]
    left /= np.linalg.norm(left)
    corridor = np.abs(P @ left) < float(wc["corridor_half_m"])
    candidates: list[tuple[float, Plane]] = []
    for _ in range(int(wc["max_planes"])):
        idx = np.flatnonzero(rest)
        if len(idx) < int(wc["min_points"]):
            break
        inl_sub = _ransac(P[idx], wall_like, float(wc["threshold_m"]), iters, rng)
        if inl_sub is None or int(inl_sub.sum()) < int(wc["min_points"]):
            break
        inl = np.zeros(len(P), bool)
        inl[idx[inl_sub]] = True
        rest &= ~inl
        n, p0 = _fit_vertical(P[inl], up, pos)
        # 当てはめ直した面で点を選び直す（RANSAC の 3 点のぶれを除く）
        inl = (np.abs((P - p0) @ n) < float(wc["threshold_m"])) & ~floor_inl & (height > base + float(wc["min_above_floor_m"]))
        if int((inl & corridor).sum()) < int(wc["min_corridor_points"]):
            continue
        dist = float(n @ (np.zeros(3) - p0))
        if dist <= 0:
            continue
        candidates.append((dist, Plane(n, p0, inl)))
    if not candidates:
        raise FaceNotFound("前方に立った面（柱・壁）が見つからない"
                           f"（{'床はあった' if floor_found else '床も無い'}。柱に向いているか、近すぎ・遠すぎでないか）")
    candidates.sort(key=lambda t: t[0])
    dist, plane = candidates[0]
    n = plane.normal

    # 4. 距離・向き
    f = -n
    yaw = float(np.arctan2(f @ left, f @ fwd))
    foot = -dist * n  # 原点から面への足（原点 + dist × (−n)）

    # 5. 端
    t = np.cross(up, f)
    t /= np.linalg.norm(t)
    pts = P[plane.inliers]
    s = (pts - foot) @ t
    center = float(np.median(s[corridor[plane.inliers]])) if (corridor & plane.inliers).any() else 0.0
    ec = cfg["edges"]
    lo, hi = _segment_around(s, center, float(ec["bin_m"]), int(ec["gap_bins"]))
    border = float(ec["border_px"])
    us = uu[plane.inliers]
    seg = (s >= lo) & (s <= hi)
    near_lo = seg & (s < lo + float(ec["bin_m"]))
    near_hi = seg & (s > hi - float(ec["bin_m"]))
    clipped = lambda m: bool(m.any()) and bool(((us[m] < border) | (us[m] > w - 1 - border)).any())  # noqa: E731
    right_edge = None if clipped(near_lo) else lo
    left_edge = None if clipped(near_hi) else hi

    return ColumnFace(normal=n, point=foot, distance_m=dist, yaw_rad=yaw, left_edge_m=left_edge,
                      right_edge_m=right_edge, n_points=int(plane.inliers.sum()), floor_found=floor_found,
                      pitch_correction_deg=correction_deg, floor_z_m=floor_z,
                      others_m=[d for d, _ in candidates[1:]], correction=R, correction_from=corr_from)


def _fit_vertical(points: np.ndarray, up: np.ndarray, pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """点に垂直な平面を当てはめる（水平面に落として直線を当てはめる）。(こちらを向く水平の法線, 平面上の点)。"""
    c = points.mean(axis=0)
    flat = (points - c) - np.outer((points - c) @ up, up)
    _, _, vt = np.linalg.svd(flat, full_matrices=False)
    # 水平面の中で広がりの大きい向きが面に沿う向き、それと up に垂直な向きが法線
    along = vt[0] - up * (vt[0] @ up)
    along /= np.linalg.norm(along)
    n = np.cross(along, up)
    if n @ (pos - c) < 0:
        n = -n
    return n, c


class FaceFinder:
    """頭カメラの深度から柱の面を求める（ButtonFinder と同じ呼び方）。"""

    def __init__(self, cam_tf: HeadCameraTransform, cfg: dict[str, Any]) -> None:
        self.cam_tf = cam_tf
        self.cfg = cfg
        # 最後に床から求めた直し方（床が写らない近さで使う）。立ち直したり首を触ったりしたら reset する
        self.last_correction: np.ndarray | None = None

    def reset(self) -> None:
        self.last_correction = None

    @classmethod
    def from_config(cls, robot_cfg: dict[str, Any], cfg: dict[str, Any],
                    offset: np.ndarray | None = None) -> "FaceFinder":
        """offset は頭カメラの位置の補正（pelvis 座標）。省略時はボタンと同じ値（elevator.yaml の calibration）。"""
        if offset is None:
            from .config import load_config

            offset = calibration_offset(load_config("elevator.yaml"))
        return cls(HeadCameraTransform(robot_cfg, offset), cfg)

    def find(self, depth_m: np.ndarray, K: np.ndarray, q_waist: np.ndarray,
             imu_quat: np.ndarray | None = None) -> ColumnFace:
        rot, pos = self.cam_tf.pelvis_from_optical(np.asarray(q_waist, dtype=float))
        pos = pos + np.asarray(getattr(self.cam_tf, "offset", np.zeros(3)), dtype=float)
        face = find_face(depth_m, np.asarray(K, dtype=float), rot, pos, imu_up_in_pelvis(imu_quat), self.cfg,
                         self.last_correction)
        if face.correction_from == "floor":
            self.last_correction = face.correction
        return face


class FaceTracker:
    """新しいフレームで続けて測り、ばらつきが小さければ中央値を返す（ButtonTracker と同じ考え方）。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.frames = int(cfg["frames"])
        self.max_spread_m = float(cfg["max_spread_m"])
        self.max_yaw_spread_deg = float(cfg["max_yaw_spread_deg"])
        self.samples: list[ColumnFace] = []

    def reset(self) -> None:
        self.samples.clear()

    def add(self, face: ColumnFace) -> ColumnFace | None:
        """1 フレーム分を足す。そろってばらつきが小さければ、まとめた結果を返す（そうでなければ None）。"""
        self.samples.append(face)
        if len(self.samples) < self.frames:
            return None
        s = self.samples[-self.frames:]
        d = np.array([f.distance_m for f in s])
        y = np.array([f.yaw_rad for f in s])
        if np.ptp(d) > self.max_spread_m or np.degrees(np.ptp(y)) > self.max_yaw_spread_deg:
            # 古いほうから捨てて、次のフレームを待つ
            self.samples.pop(0)
            return None
        k = int(np.argsort(d)[len(d) // 2])  # 距離が中央値のフレームの法線・足を使う

        def med(vals: list[float | None]) -> float | None:
            known = [v for v in vals if v is not None]
            # 半分以上のフレームで分かったときだけ
            return float(np.median(known)) if len(known) * 2 > len(vals) else None

        last = s[-1]
        return ColumnFace(normal=s[k].normal, point=s[k].point, distance_m=float(np.median(d)),
                          yaw_rad=float(np.median(y)), left_edge_m=med([f.left_edge_m for f in s]),
                          right_edge_m=med([f.right_edge_m for f in s]), n_points=last.n_points,
                          floor_found=last.floor_found, pitch_correction_deg=last.pitch_correction_deg,
                          floor_z_m=last.floor_z_m, others_m=last.others_m, correction=last.correction,
                          correction_from=last.correction_from)
