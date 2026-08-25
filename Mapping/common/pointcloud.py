"""点群のダウンサンプル・蓄積・ファイル出力。

依存は numpy のみ。open3d は使っていない。`G1_HuggingFace/venv/` は
`SimpleWalk/` の動作確認済みフローと共有しているため、点群処理のためだけに
open3d（numpy を巻き込む重い依存）を入れてその venv を壊すリスクを避けた。
出力する PLY は MeshLab / CloudCompare / open3d のいずれでも読める標準形式なので、
可視化したいときは別環境で開けばよい。
"""
from __future__ import annotations

from pathlib import Path

import numpy as np


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """`voxel_size`[m] の立方体グリッドで点群を間引く（各ボクセルの重心を代表点にする）。

    ICP に生の点群をそのまま渡すと、近距離の面だけ極端に密になって
    そちらに位置合わせが引っ張られる。密度をそろえるのが目的。
    """
    if len(points) == 0:
        return points.reshape(0, 3)
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    n_voxels = int(inverse.max()) + 1
    sums = np.zeros((n_voxels, 3), dtype=np.float64)
    np.add.at(sums, inverse, points)
    counts = np.bincount(inverse, minlength=n_voxels).reshape(-1, 1)
    return sums / counts


def transform_points(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    """同次変換行列 `pose` (4x4) を点群に適用する。"""
    if len(points) == 0:
        return points.reshape(0, 3)
    return points @ pose[:3, :3].T + pose[:3, 3]


class VoxelMap:
    """ボクセルグリッドで重複を潰しながら点群を蓄積していく地図。

    スキャンを素朴に concatenate すると、同じ壁を何百フレームも見るぶんだけ
    点が増え続けて数千万点になる。ボクセル単位で 1 点に潰すことで、
    地図のサイズが「見た空間の広さ」に比例する形（フレーム数に依存しない）に収まる。
    """

    def __init__(self, voxel_size: float = 0.05) -> None:
        self.voxel_size = voxel_size
        # ボクセル添字 -> (点の総和, 点数)。重心を保つことで量子化誤差を減らす。
        self._sums: dict[tuple[int, int, int], np.ndarray] = {}
        self._counts: dict[tuple[int, int, int], int] = {}

    def __len__(self) -> int:
        return len(self._sums)

    def is_empty(self) -> bool:
        return len(self._sums) == 0

    def add(self, points_world: np.ndarray) -> None:
        """ワールド座標系の点群を地図に追加する。"""
        if len(points_world) == 0:
            return
        keys = np.floor(points_world / self.voxel_size).astype(np.int64)
        uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
        n_voxels = len(uniq)
        sums = np.zeros((n_voxels, 3), dtype=np.float64)
        np.add.at(sums, inverse, points_world)
        counts = np.bincount(inverse, minlength=n_voxels)
        for i, key in enumerate(map(tuple, uniq)):
            if key in self._sums:
                self._sums[key] += sums[i]
                self._counts[key] += int(counts[i])
            else:
                self._sums[key] = sums[i].copy()
                self._counts[key] = int(counts[i])

    def points(self) -> np.ndarray:
        """蓄積した地図を `(N, 3)` の点群として取り出す。"""
        if not self._sums:
            return np.zeros((0, 3), dtype=np.float64)
        sums = np.array(list(self._sums.values()), dtype=np.float64)
        counts = np.array(list(self._counts.values()), dtype=np.float64).reshape(-1, 1)
        return sums / counts


def save_ply(path: str | Path, points: np.ndarray, colors: np.ndarray | None = None) -> None:
    """点群を binary little-endian の PLY として保存する。

    ASCII だと数百万点で数百 MB になり書き出しも遅いためバイナリにしている。
    `colors` は 0-255 の uint8 `(N, 3)`。
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points, dtype=np.float32)

    header = [
        "ply",
        "format binary_little_endian 1.0",
        f"element vertex {len(points)}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if colors is not None:
        header += ["property uchar red", "property uchar green", "property uchar blue"]
    header.append("end_header")

    if colors is None:
        body = points.tobytes()
    else:
        dtype = np.dtype([("xyz", "<f4", 3), ("rgb", "u1", 3)])
        rows = np.empty(len(points), dtype=dtype)
        rows["xyz"] = points
        rows["rgb"] = np.asarray(colors, dtype=np.uint8)
        body = rows.tobytes()

    with open(path, "wb") as f:
        f.write(("\n".join(header) + "\n").encode("ascii"))
        f.write(body)


def height_colors(points: np.ndarray) -> np.ndarray:
    """z 座標に応じた色を付ける（地図を目視確認するとき床と壁を見分けるため）。"""
    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.uint8)
    z = points[:, 2]
    lo, hi = float(z.min()), float(z.max())
    t = np.zeros_like(z) if hi - lo < 1e-6 else (z - lo) / (hi - lo)
    # 低い(青) -> 高い(赤) の単純なグラデーション
    rgb = np.stack([t, 0.3 * np.ones_like(t), 1.0 - t], axis=-1)
    return (rgb * 255).astype(np.uint8)
