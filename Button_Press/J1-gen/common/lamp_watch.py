"""押している間・戻る間も、頭カメラの画像でボタンのランプと乗り場の扉を見る（common/elevator_press.py が使う）。

ランプ（LampWatch・hand_circles・hand_mask）:
  点灯の確認は、これまで当たってから約 3.5 秒後の 1 枚だけだった（保持 0.6 秒 + 戻り 2.5 秒 + 待ち 0.3 秒）。
  かごがその階にいると、ランプは押している間だけ点き、離すと消える（2026-11-03 の動画）。すぐ来るときも数秒で消える。
  そのため人の目では点灯していたのに「点かず」と判定し、強く押し直した（11/3、8 回）。
  押し込み・保持・戻り・確認の間の新しい画像を毎回見て、一度でも点灯していれば「点灯した」とする。
  手（ゴムの手は白い）がランプの枠に被さると明るく見えるので、関節の角度から手・前腕が画像のどこに写るかを求め、
  その画素は使わない。手はゴムの手のメッシュ（URDF の rubber_hand.STL）の頂点を写して、余裕の幅をつけて塗る。
  前腕は円の並び。見える部分が少ない画像は判定に使わない。
  最初は指を中指 1 本の細い線で表したが、曲がった指が上下に重なって並ぶので、上の指が枠に被さって明るく見え、
  押し込みの途中（触れる前）に「点灯」と判定した（2026-11-10、MuJoCo）。

扉（DoorWatch）:
  頭カメラの画像の左（ロボットの左）に乗り場の扉が写る（11/3 の画像）。扉が開くと、その範囲の深度が奥へ大きく変わる。
  開いた時刻を記録するだけで、押し直すかの判断には使わない（車いす用のボタンに対応するかごは 2 つあり、
  ロボットの真後ろの扉に来ることもあるため。2026-11-10 にユーザーが決めた）。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


def project(points: np.ndarray, K: np.ndarray, rot: np.ndarray, pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """pelvis 座標の点 (N, 3) → 画素 (N, 2) と、カメラからの奥行き z (N,)。rot・pos は光学座標 → pelvis（p = rot x + pos）。"""
    o = (np.asarray(points, dtype=float) - pos) @ rot
    z = o[:, 2]
    zs = np.where(z > 1e-6, z, np.nan)
    uv = np.stack([K[0, 0] * o[:, 0] / zs + K[0, 2], K[1, 1] * o[:, 1] / zs + K[1, 2]], axis=1)
    return uv, z


def load_stl_vertices(path: str | Path, max_points: int = 4000) -> np.ndarray:
    """STL（バイナリ）の頂点 (N, 3)。重なりを除き、多ければ間引く。"""
    b = Path(path).read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    tri = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("x", "<u2")]))
    v = np.unique(tri["v"].reshape(-1, 3).astype(float), axis=0)
    if len(v) > max_points:
        v = v[np.linspace(0, len(v) - 1, max_points).astype(int)]
    return v


def segment_circles(a: np.ndarray, b: np.ndarray, radius_m: float, K: np.ndarray, rot: np.ndarray, pos: np.ndarray,
                    n: int = 6) -> list[tuple[float, float, float]]:
    """a → b の線分を、半径 radius_m の球の並びにして画像に写した円 (u, v, 半径 px) の一覧（前腕に使う）。
    カメラの後ろの点は除く。"""
    pts = np.array([a + s * (b - a) for s in np.linspace(0.0, 1.0, n)])
    uv, z = project(pts, K, rot, pos)
    return [(float(u), float(v), float(K[0, 0] * radius_m / zz))
            for (u, v), zz in zip(uv, z) if zz > 0.05 and np.isfinite(u) and np.isfinite(v)]


def mesh_circles(vertices_pelvis: np.ndarray, margin_m: float, K: np.ndarray, rot: np.ndarray,
                 pos: np.ndarray) -> list[tuple[float, float, float]]:
    """手のメッシュの頂点（pelvis 座標）を画像に写し、余裕の幅 margin_m [m] の円にしたもの。"""
    uv, z = project(vertices_pelvis, K, rot, pos)
    ok = (z > 0.05) & np.all(np.isfinite(uv), axis=1)
    return [(float(u), float(v), float(K[0, 0] * margin_m / zz)) for (u, v), zz in zip(uv[ok], z[ok])]


def hand_mask(shape: tuple[int, int], circles: list[tuple[float, float, float]],
              region: tuple[int, int, int, int] | None = None) -> np.ndarray:
    """円の並びで覆われる画素（True）。region (x1, y1, x2, y2) を渡すと、その中だけを塗る（速くするため）。"""
    h, w = shape
    mask = np.zeros((h, w), dtype=bool)
    rx1, ry1, rx2, ry2 = region if region is not None else (0, 0, w, h)
    rx1, ry1, rx2, ry2 = max(int(rx1), 0), max(int(ry1), 0), min(int(rx2), w), min(int(ry2), h)
    for u, v, r in circles:
        x1, x2 = max(int(np.floor(u - r)), rx1), min(int(np.ceil(u + r)) + 1, rx2)
        y1, y2 = max(int(np.floor(v - r)), ry1), min(int(np.ceil(v + r)) + 1, ry2)
        if x2 <= x1 or y2 <= y1:
            continue
        yy, xx = np.mgrid[y1:y2, x1:x2]
        mask[y1:y2, x1:x2] |= (xx + 0.5 - u) ** 2 + (yy + 0.5 - v) ** 2 <= r * r
    return mask


@dataclass
class LampWatch:
    """押し込みから確認までの画像ごとのランプの明るさ。一度でも点灯と判定した画像があれば seen に残す。"""

    trace: list[dict[str, Any]] = field(default_factory=list)
    seen: dict[str, Any] | None = None

    def add(self, entry: dict[str, Any]) -> bool:
        """1 枚分を足す。この画像で初めて点灯を見たら True。"""
        self.trace.append(entry)
        if entry["lit"] and self.seen is None:
            self.seen = entry
            return True
        return False

    def summary(self) -> dict[str, Any]:
        used = [e for e in self.trace if e["used"]]
        return {"frames": len(self.trace), "used": len(used), "seen": self.seen,
                "max_rise": max((e["rise"] for e in used), default=None)}


class DoorWatch:
    """柱の面のうち、ボタンの左（ロボットの左）の範囲を乗り場の扉とみなし、深度が奥へ変わった画素の割合を見る。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.cfg = cfg
        self.region: np.ndarray | None = None
        self.base: np.ndarray | None = None
        self.opened: dict[str, Any] | None = None
        self.max_fraction = 0.0
        self.frames = 0
        self._streak = 0

    def start(self, mid: np.ndarray, normal: np.ndarray, depth: np.ndarray, K: np.ndarray, rot: np.ndarray,
              pos: np.ndarray) -> int:
        """扉の範囲（画像の画素）と、そのときの深度を基準として覚える。範囲に入った有効な画素の数を返す。"""
        n = np.asarray(normal, dtype=float)
        left = np.cross([0.0, 0.0, 1.0], n)
        left /= np.linalg.norm(left)
        (l1, l2), (h1, h2) = self.cfg["left_m"], self.cfg["height_m"]
        corners = np.array([mid + a * left + b * np.array([0.0, 0.0, 1.0])
                            for a, b in ((l1, h1), (l2, h1), (l2, h2), (l1, h2))])
        uv, z = project(corners, K, rot, pos)
        depth = np.asarray(depth, dtype=float)
        if not (np.all(z > 0.05) and np.all(np.isfinite(uv))):
            self.region = np.zeros(depth.shape, dtype=bool)
            return 0
        self.region = convex_mask(depth.shape, uv) & (depth > 0.0)
        self.base = depth.copy()
        return int(self.region.sum())

    def observe(self, depth: np.ndarray, covered: np.ndarray | None, t: float) -> float | None:
        """深度が基準より min_depth_increase_m 以上奥になった画素の割合（手に覆われた・深度の無い画素は除く）。
        min_fraction 以上が 2 枚続いたら、開いたとして時刻を残す。範囲が無ければ None。"""
        if self.region is None or self.base is None:
            return None
        depth = np.asarray(depth, dtype=float)
        use = self.region & (depth > 0.0)
        if covered is not None:
            use &= ~covered
        if int(use.sum()) < int(self.cfg["min_pixels"]):
            return None
        frac = float(np.mean(depth[use] - self.base[use] > float(self.cfg["min_depth_increase_m"])))
        self.frames += 1
        self.max_fraction = max(self.max_fraction, frac)
        self._streak = self._streak + 1 if frac >= float(self.cfg["min_fraction"]) else 0
        if self._streak >= 2 and self.opened is None:
            self.opened = {"t": float(t), "fraction": frac}
        return frac

    def summary(self) -> dict[str, Any]:
        return {"region_px": int(self.region.sum()) if self.region is not None else 0, "frames": self.frames,
                "max_fraction": self.max_fraction, "opened": self.opened}


def convex_mask(shape: tuple[int, int], poly: np.ndarray) -> np.ndarray:
    """凸の多角形（画素の頂点 (N, 2)、順に並ぶ）の内側の画素。画像の外にはみ出した分は切る。"""
    h, w = shape
    yy, xx = np.mgrid[0:h, 0:w]
    px, py = xx + 0.5, yy + 0.5
    signs = []
    for (x1, y1), (x2, y2) in zip(poly, np.roll(poly, -1, axis=0)):
        signs.append((x2 - x1) * (py - y1) - (y2 - y1) * (px - x1) >= 0)
    signs = np.array(signs)
    return np.all(signs, axis=0) | np.all(~signs, axis=0)
