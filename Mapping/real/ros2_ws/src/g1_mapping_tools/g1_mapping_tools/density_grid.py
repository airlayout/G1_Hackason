"""ROSに依存しない、スキャン単位のボクセル密度集計。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Iterable


DENSITY_POINT_STEP = 28
UINT32_MAX = (1 << 32) - 1


@dataclass
class VoxelStats:
    sum_x: float = 0.0
    sum_y: float = 0.0
    sum_z: float = 0.0
    sum_intensity: float = 0.0
    hit_count: int = 0
    scan_count: int = 0

    def add_scan(
        self,
        *,
        sum_x: float,
        sum_y: float,
        sum_z: float,
        sum_intensity: float,
        hits: int,
    ) -> None:
        self.sum_x += sum_x
        self.sum_y += sum_y
        self.sum_z += sum_z
        self.sum_intensity += sum_intensity
        self.hit_count += hits
        self.scan_count += 1


class DensityGrid:
    """点数と観測スキャン数を保持する、上限付きボクセル地図。"""

    def __init__(
        self, *, voxel_size: float, max_points: int, target_scan_count: int
    ) -> None:
        if voxel_size <= 0.0:
            raise ValueError("voxel_sizeは0より大きい必要があります")
        if max_points <= 0:
            raise ValueError("max_pointsは0より大きい必要があります")
        if target_scan_count <= 0:
            raise ValueError("target_scan_countは0より大きい必要があります")
        self.voxel_size = voxel_size
        self.max_points = max_points
        self.target_scan_count = target_scan_count
        self.voxels: dict[tuple[int, int, int], VoxelStats] = {}
        self.dropped_new_voxels = 0

    def integrate_scan(
        self, points: Iterable[tuple[float, float, float, float]]
    ) -> int:
        """1スキャンを統合し、更新したボクセル数を返す。"""

        per_scan: dict[tuple[int, int, int], list[float]] = {}
        for x, y, z, intensity in points:
            x, y, z, intensity = float(x), float(y), float(z), float(intensity)
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            if not math.isfinite(intensity):
                intensity = 0.0
            key = (
                math.floor(x / self.voxel_size),
                math.floor(y / self.voxel_size),
                math.floor(z / self.voxel_size),
            )
            aggregate = per_scan.setdefault(key, [0.0, 0.0, 0.0, 0.0, 0.0])
            aggregate[0] += x
            aggregate[1] += y
            aggregate[2] += z
            aggregate[3] += intensity
            aggregate[4] += 1.0

        updated = 0
        for key, aggregate in per_scan.items():
            stats = self.voxels.get(key)
            if stats is None:
                if len(self.voxels) >= self.max_points:
                    self.dropped_new_voxels += 1
                    continue
                stats = VoxelStats()
                self.voxels[key] = stats
            stats.add_scan(
                sum_x=aggregate[0],
                sum_y=aggregate[1],
                sum_z=aggregate[2],
                sum_intensity=aggregate[3],
                hits=int(aggregate[4]),
            )
            updated += 1
        return updated

    def snapshot(self) -> list[tuple[float, float, float, float, float, int, int]]:
        """PointCloud2へ格納する統計のsnapshotを返す。"""

        output: list[tuple[float, float, float, float, float, int, int]] = []
        for stats in self.voxels.values():
            hits = max(1, stats.hit_count)
            density = min(stats.scan_count / self.target_scan_count, 1.0)
            output.append(
                (
                    stats.sum_x / hits,
                    stats.sum_y / hits,
                    stats.sum_z / hits,
                    stats.sum_intensity / hits,
                    density,
                    min(stats.hit_count, UINT32_MAX),
                    min(stats.scan_count, UINT32_MAX),
                )
            )
        return output


def pack_density_points(
    points: Iterable[tuple[float, float, float, float, float, int, int]],
) -> bytes:
    """density PointCloud2のdata部分をlittle-endianで構築する。"""

    values = list(points)
    data = bytearray(len(values) * DENSITY_POINT_STEP)
    for index, value in enumerate(values):
        struct.pack_into("<fffffII", data, index * DENSITY_POINT_STEP, *value)
    return bytes(data)
