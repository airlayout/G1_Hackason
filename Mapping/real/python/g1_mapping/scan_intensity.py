"""生の Livox トピックから、反射強度つきのスキャンを姿勢込みで取り出す。

## なぜ要るのか

`export_benchmark_data.py` が書き出す姿勢つき PCD は `write_pcd()` を使っており、
**x/y/z しか書かない**。そのため 2026-09-05 の 3DGS では「色が無い」と思い込んで
反射強度を捨てていた。実際には生トピックに入っている:

    /utlidar/cloud_livox_mid360   x y z intensity ring time   (point_step 22)
    /unitree/slam_mapping/points  x y z normal_xyz intensity curvature (point_step 48)

同じ 10 cm ボクセルを 5 枚以上のスキャンから見たときの intensity のばらつきは
中央値 7.2（場所による違いは標準偏差 28.8）で、**視点によらない成分が 4 倍支配的**。
1 チャンネルの「色」として 3DGS に入れられる水準である。

## 姿勢の付け方

`benchmark/poses.txt` の第 1 列は **db3 の記録時刻**で、これがそのまま
生メッセージの `timestamp` に対応する。ICP で精密化した後の姿勢が同じ行にあるので、
記録時刻でメッセージを引けば「生スキャン＋確定した姿勢」がそろう。
書き出し済み PCD を経由しないので、間引きも自分でやる。
"""

from __future__ import annotations

import glob
import sqlite3
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .rebuild import _CdrReader

RAW_TOPIC = "/utlidar/cloud_livox_mid360"
# lidar_gs と同じ正規化を使う（循環 import を避けるためここに同値を置く）
INTENSITY_SCALE_HINT = 255.0
_NUMPY_TYPE = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "<f4", 8: "<f8"}


@dataclass(frozen=True)
class IntensityScan:
    """1 スキャンぶんの、world 座標の点と反射強度とセンサ位置。"""

    origin: np.ndarray        # (3,)
    points: np.ndarray        # (N, 3) world
    intensity: np.ndarray     # (N,) 生の値（0〜155 程度）


def _parse_fields(payload: bytes) -> "tuple[dict[str, tuple[int, int]], int, int, int]":
    reader = _CdrReader(payload)
    reader.int32()                      # header.stamp.sec
    reader.uint32()                     # header.stamp.nanosec
    reader.string()                     # frame_id
    reader.uint32()                     # height
    reader.uint32()                     # width
    fields: dict[str, tuple[int, int]] = {}
    for _ in range(reader.uint32()):
        name = reader.string()
        offset = reader.uint32()
        datatype = reader.uint8()
        reader.uint32()                 # count
        fields[name] = (offset, datatype)
    reader.uint8()                      # is_bigendian
    point_step = reader.uint32()
    reader.uint32()                     # row_step
    data_length = reader.uint32()
    return fields, point_step, reader.position + 4, data_length


def _column(block: np.ndarray, offset: int, datatype: int) -> np.ndarray:
    dtype = np.dtype(_NUMPY_TYPE[datatype])
    return block[:, offset:offset + dtype.itemsize].copy().view(dtype).ravel()


class IntensityScanReader:
    """db3 と poses.txt を突き合わせて、姿勢つき・強度つきスキャンを返す。"""

    def __init__(self, session: Path, tolerance: float = 0.02) -> None:
        bags = sorted(glob.glob(str(session / "raw/rosbag2/*.db3")))
        if not bags:
            raise SystemExit(f"rosbag2 が見つかりません: {session}")
        self._connection = sqlite3.connect(f"file:{bags[0]}?mode=ro", uri=True)
        topic = self._connection.execute(
            "SELECT id FROM topics WHERE name=?", (RAW_TOPIC,)).fetchone()
        if topic is None:
            raise SystemExit(f"{RAW_TOPIC} が記録されていません")
        rows = self._connection.execute(
            "SELECT id, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic[0],)).fetchall()
        self._ids = np.array([row[0] for row in rows], dtype=np.int64)
        self._stamps = np.array([row[1] for row in rows], dtype=np.float64) / 1e9

        poses = np.loadtxt(session / "benchmark/poses.txt")
        # poses.txt の第 1 列は db3 の記録時刻。それで生メッセージを引く
        nearest = np.abs(self._stamps[None, :] - poses[:, 0][:, None]).argmin(axis=1)
        gap = np.abs(self._stamps[nearest] - poses[:, 0])
        keep = gap < tolerance
        self.message_ids = self._ids[nearest[keep]]
        self.poses = poses[keep]
        self.dropped = int((~keep).sum())

    def __len__(self) -> int:
        return len(self.message_ids)

    def close(self) -> None:
        self._connection.close()

    def read(self, index: int, voxel: float = 0.05,
             min_range: float = 0.1) -> IntensityScan:
        payload = self._connection.execute(
            "SELECT data FROM messages WHERE id=?",
            (int(self.message_ids[index]),)).fetchone()[0]
        fields, point_step, start, length = _parse_fields(payload)
        block = np.frombuffer(payload, dtype=np.uint8, count=length,
                              offset=start).reshape(-1, point_step)
        sensor = np.stack([_column(block, *fields[axis]) for axis in "xyz"], axis=1)
        intensity = _column(block, *fields["intensity"])

        finite = np.isfinite(sensor).all(axis=1)
        finite &= np.linalg.norm(sensor, axis=1) > min_range
        sensor, intensity = sensor[finite], intensity[finite]

        pose = self.poses[index]
        rotation = _quaternion_to_matrix(pose[4:8])
        # この環境（macOS / numpy 2.2 / Accelerate）の matmul は、入力も出力も有限なのに
        # divide-by-zero / overflow / invalid の FP フラグを立てる。einsum と手計算に対して
        # 最大差 3.6e-15 で一致することを確認済みなので、偽陽性として黙らせる。
        with np.errstate(all="ignore"):
            world = sensor.astype(np.float64) @ rotation.T + pose[1:4]

        if voxel > 0:
            world, intensity = _voxel_average(world, intensity, voxel)
        return IntensityScan(origin=pose[1:4].copy(), points=world, intensity=intensity)


def _quaternion_to_matrix(xyzw: np.ndarray) -> np.ndarray:
    """(x, y, z, w) を回転行列にする。scipy を持ち込まずに済ませる。"""
    x, y, z, w = xyzw / np.linalg.norm(xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _voxel_average(points: np.ndarray, values: np.ndarray,
                   voxel: float) -> "tuple[np.ndarray, np.ndarray]":
    """ボクセル内の点と値を平均する。書き出し済み PCD と同じ 0.05 m で間引くため。"""
    keys = np.floor(points / voxel).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    summed = np.zeros((len(counts), 3))
    np.add.at(summed, inverse, points)
    intensity = np.zeros(len(counts))
    np.add.at(intensity, inverse, values)
    return summed / counts[:, None], intensity / counts


def intensity_for_map(session: Path, map_points: np.ndarray, *,
                      stride: int = 5, radius: float = 0.05) -> np.ndarray:
    """地図の各点に、いちばん近い実測の反射強度を割り当てる。

    ガウシアンの初期色に使う。届かなかった点は全体の中央値で埋める。
    """
    from scipy.spatial import cKDTree

    reader = IntensityScanReader(session)
    clouds, values = [], []
    for index in range(0, len(reader), stride):
        scan = reader.read(index)
        clouds.append(scan.points)
        values.append(scan.intensity)
    reader.close()
    observed = np.concatenate(clouds)
    measured = np.concatenate(values)

    distance, nearest = cKDTree(observed).query(map_points, workers=-1)
    assigned = measured[nearest]
    missing = distance > radius
    assigned[missing] = np.median(measured)
    return assigned, missing
