"""PCDをnumpy配列として読む。

`pcd.py` は「依存ライブラリなし」を約束しているのでnumpyを持ち込めない。
一方で点群を実際に処理する側はnumpyが要る。ヘッダー解析だけ `pcd.py` から
借りて、本体の読み出しをこちらに置く。

VIEWPOINTも返す。可視性ベースの地図掃除（OctoMap等）はこの欄をレイの始点に
使うため、姿勢つきPCDを扱うときに要る。PCLの並びは並進が先で四元数はqwが先:

    VIEWPOINT tx ty tz qw qx qy qz
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .pcd import _pcd_header, _point_count, _xyz_offsets

DEFAULT_VIEWPOINT = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class PcdData:
    """PCDから取り出した点と姿勢。"""

    points: np.ndarray                                   # (N, 3) float64
    viewpoint: "tuple[float, float, float, float, float, float, float]"

    @property
    def origin(self) -> np.ndarray:
        """VIEWPOINTの並進部分。レイキャストの始点。"""
        return np.array(self.viewpoint[:3], dtype=np.float64)

    @property
    def quaternion_xyzw(self) -> "tuple[float, float, float, float]":
        """VIEWPOINTの回転を、scipy等が使う (x, y, z, w) の並びで返す。"""
        qw, qx, qy, qz = self.viewpoint[3:]
        return (qx, qy, qz, qw)


def _parse_viewpoint(header: "dict[str, list[str]]") -> "tuple[float, ...]":
    raw = header.get("VIEWPOINT")
    if not raw:
        return DEFAULT_VIEWPOINT
    if len(raw) != 7:
        raise ValueError(f"VIEWPOINTは7要素である必要があります: {raw}")
    return tuple(float(value) for value in raw)


def read_pcd(path: Path, finite_only: bool = True) -> PcdData:
    """binary PCDのx/y/zとVIEWPOINTを読む。ascii PCDには対応しない。"""
    header, data_offset = _pcd_header(path)
    mode = header["DATA"][0].lower()
    if mode != "binary":
        raise ValueError(f"binary PCDのみ対応しています（{path} は {mode}）")

    count = _point_count(header)
    x_offset, y_offset, z_offset, point_step, code = _xyz_offsets(header)
    width = 4 if code == "f" else 8
    dtype = np.float32 if code == "f" else np.float64

    with path.open("rb") as stream:
        stream.seek(data_offset)
        payload = stream.read(count * point_step)
    usable = len(payload) // point_step
    if usable == 0:
        return PcdData(np.empty((0, 3)), _parse_viewpoint(header))
    block = np.frombuffer(payload, dtype=np.uint8, count=usable * point_step)
    block = block.reshape(usable, point_step)

    points = np.stack(
        [block[:, off:off + width].copy().view(dtype).ravel()
         for off in (x_offset, y_offset, z_offset)],
        axis=1,
    ).astype(np.float64)
    if finite_only:
        points = points[np.isfinite(points).all(axis=1)]
    return PcdData(points, _parse_viewpoint(header))


def write_pcd_array(path: Path, points: np.ndarray,
                    viewpoint: "tuple[float, ...] | None" = None) -> None:
    """(N,3) の ndarray を binary PCD として書く。`read_pcd` の対。

    `rebuild.write_pcd` と同じ形式だが、あちらは numpy を持ち込まない約束で
    1 点ずつ struct.pack する。数百万点を書くとそこが支配的になるため、
    numpy 側の入口にまとめて書く版を置く。

    viewpoint は (tx, ty, tz, qw, qx, qy, qz)。PCL の並びは並進が先で qw が先。
    """
    points = np.ascontiguousarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"(N,3) の配列である必要があります: {points.shape}")
    values = DEFAULT_VIEWPOINT if viewpoint is None else tuple(viewpoint)
    if len(values) != 7:
        raise ValueError(f"viewpointは7要素(tx,ty,tz,qw,qx,qy,qz)である必要があります: {values}")

    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT {}\n".format(" ".join(f"{v:.9g}" for v in values)) +
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(points.tobytes())
