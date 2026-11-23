"""部品と組み立ての絵（matplotlib で三角形を陰影つきで描く。確かめ用の簡単な絵）。"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Hiragino Sans", "DejaVu Sans"]
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

LIGHT = np.array([0.4, -0.6, 0.7]) / np.linalg.norm([0.4, -0.6, 0.7])

COLORS = {
    "body": "#d9b8a0",
    "back": "#b9b2a6",
    "carrier": "#a69e92",
    "pocket": "#e8e2d6",
    "phone": "#3b4a5a",
}


def _color(name: str) -> str:
    for key, c in COLORS.items():
        if name.startswith(key):
            return c
    return "#bbbbbb"


def _tris(part, tol: float = 0.4) -> np.ndarray:
    """build123d の Part か trimesh の Trimesh を三角形の並びに。"""
    if hasattr(part, "triangles"):
        return np.asarray(part.triangles)
    verts, faces = part.tessellate(tol, 0.3)
    v = np.array([[p.X, p.Y, p.Z] for p in verts])
    return v[np.array(faces)]


def _shade(tris: np.ndarray, base: str) -> list:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    k = 0.45 + 0.55 * np.clip(np.abs(n @ LIGHT), 0, 1)
    rgb = np.array(matplotlib.colors.to_rgb(base))
    return [tuple(np.clip(rgb * kk, 0, 1)) for kk in k]


def draw(parts: dict, path: Path, views: list[tuple[float, float, str]], title: str) -> None:
    """views: (仰角, 方位角, 見出し) の並び。1 枚の画に横に並べる。

    三角形は全部 1 つの集まりにまとめる（部品ごとに分けると、重ね順が部品単位になり奥の物が手前に出る）。
    """
    tris, cols = [], []
    for name, part in parts.items():
        t = _tris(part)
        tris.append(t)
        cols.extend(_shade(t, _color(name)))
    allt = np.vstack(tris)
    pts = allt.reshape(-1, 3)
    lo, hi = pts.min(0), pts.max(0)
    fig = plt.figure(figsize=(6 * len(views), 6))
    for i, (elev, azim, label) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(allt, facecolors=cols, edgecolors="none"))
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.set_zlim(lo[2], hi[2])
        ax.set_box_aspect(hi - lo)
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        ax.set_title(label, fontsize=11)
    fig.suptitle(title, fontsize=13)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
