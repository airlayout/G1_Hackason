"""過去rayを一度ずつ統合し、時刻付きworld voxelから局所場の教師を読む。"""

import numpy as np

from g1_e2e.data.geometry import transform_points
from g1_e2e.data.targets import FieldTargets, RayEvidence


class RollingField:
    def __init__(self, *, window_s=40.96, ray_stride=20, resolution=0.2):
        self.window_s, self.ray_stride, self.resolution = window_s, ray_stride, resolution
        self.bounds = np.array([[-32, 16], [-40, 72], [-2, 6]], float)
        self.shape = np.rint(np.diff(self.bounds)[:, 0] / resolution).astype(int)
        self.seen = np.full(tuple(self.shape), -np.inf, np.float32)
        self.hit = np.full_like(self.seen, -np.inf)
        local_bounds = np.array([[-6, 6], [-6, 6], [-1.5, 2.5]])
        axes = [np.arange(a + resolution / 2, b, resolution) for a, b in local_bounds]
        self.queries = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
        self.last_time = -np.inf

    def add(self, frame):
        now = float(frame["time_s"])
        if now <= self.last_time:
            raise ValueError("LiDAR教師は時刻順に一度だけ統合してください")
        self.last_time = now
        origin = frame["world_from_sensor"][:3, 3]
        low = np.maximum(np.floor((origin - [9, 9, 4] - self.bounds[:, 0]) / self.resolution), 0).astype(int)
        high = np.minimum(
            np.ceil((origin + [9, 9, 4] - self.bounds[:, 0]) / self.resolution), self.shape
        ).astype(int)
        if np.any(high <= low):
            return
        bounds = self.bounds[:, :1] + np.column_stack((low, high)) * self.resolution
        evidence = RayEvidence(bounds, self.resolution)
        selected = np.flatnonzero(frame["valid"])[:: self.ray_stride]
        points = frame.get("points_world_m")
        if points is None:
            points = transform_points(frame["points_sensor_m"], frame["world_from_sensor"])
        origins = frame.get("measurement_positions_world_m")
        if origins is None:
            origins = np.broadcast_to(origin, points.shape)
        evidence.add(origins[selected], points[selected])
        for mask, target in ((evidence.known, self.seen), (evidence.occupied, self.hit)):
            cells = np.argwhere(mask) + low
            inside = np.all(cells < self.shape, axis=1)
            target[tuple(cells[inside].T)] = now

    def sample(self, world_from_base, now, *, count=96, seed=0):
        if self.last_time > now + 1e-8:
            raise ValueError("未来のLiDARを統合した場を教師にすることはできません")
        world = transform_points(self.queries, world_from_base)
        cells = np.floor((world - self.bounds[:, 0]) / self.resolution).astype(int)
        inside = np.all((cells >= 0) & (cells < self.shape), axis=1)
        known, occupied = np.zeros(len(cells), bool), np.zeros(len(cells), bool)
        index = tuple(cells[inside].T)
        cutoff = now - self.window_s
        known[inside] = self.seen[index] >= cutoff
        occupied[inside] = self.hit[index] >= cutoff
        groups = [np.flatnonzero(occupied), np.flatnonzero(known & ~occupied), np.flatnonzero(~known)]
        groups = [group for group in groups if len(group)]
        rng = np.random.default_rng(seed)
        chosen = np.concatenate(
            [
                rng.choice(group, count // len(groups) + (i < count % len(groups)), replace=True)
                for i, group in enumerate(groups)
            ]
        )
        rng.shuffle(chosen)
        return FieldTargets(
            self.queries[chosen].astype(np.float32),
            occupied[chosen].astype(np.float32),
            known[chosen].astype(np.float32),
        )
