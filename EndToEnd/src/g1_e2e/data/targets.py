"""保存した因果的センサー・速度指令・実測未来姿勢から3出力の教師を作る。"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .geometry import transform_points


@dataclass(frozen=True)
class FieldTargets:
    coordinates: np.ndarray  # [Q,3], 現在base座標m、voxel中心
    occupied: np.ndarray  # [Q], 未知点の値はlossに使わない
    observed: np.ndarray  # [Q], 過去センサーのray evidenceがあるvoxel

    @property
    def occupancy_valid(self):
        return self.observed.astype(bool)


@dataclass(frozen=True)
class TrajectoryTargets:
    times_s: np.ndarray  # [K], 現在からの秒
    poses: np.ndarray  # [K,7], 現在baseに対するXYZ + quaternion xyzw
    valid: np.ndarray  # [K], episodeの終了・未記録の未来を越えない


class RayEvidence:
    """局所voxelへ有効rayだけを統合する。表面より奥・無効rayはfreeにしない。"""

    def __init__(self, bounds=((-6, 6), (-6, 6), (-1.5, 2.5)), voxel_size_m=0.2):
        self.bounds = np.asarray(bounds, float)
        self.resolution = voxel_size_m
        if (
            self.bounds.shape != (3, 2)
            or not np.isfinite(self.bounds).all()
            or np.any(self.bounds[:, 0] >= self.bounds[:, 1])
            or not np.isfinite(voxel_size_m)
            or voxel_size_m <= 0
        ):
            raise ValueError("voxelの範囲とサイズが不正です")
        self.shape = np.ceil(np.diff(self.bounds, axis=1)[:, 0] / voxel_size_m).astype(int)
        self.known = np.zeros(tuple(self.shape), bool)
        self.occupied = np.zeros_like(self.known)

    def _mark(self, points, occupied=False):
        cells = np.floor((points - self.bounds[:, 0]) / self.resolution).astype(int)
        inside = np.all((cells >= 0) & (cells < self.shape), axis=1)
        cells = cells[inside]
        if len(cells):
            indices = tuple(cells.T)
            self.known[indices] = True
            if occupied:
                self.occupied[indices] = True

    def add(self, origins, endpoints):
        origins, endpoints = np.broadcast_arrays(np.asarray(origins, float), np.asarray(endpoints, float))
        if origins.ndim != 2 or origins.shape[1] != 3 or not np.isfinite([origins, endpoints]).all():
            raise ValueError("rayの原点・有効反射点は有限の[N,3]です")
        self._mark(endpoints, occupied=True)
        # rayを局所boxへclipしてから細かくsampleし、遠距離・多数beamでも一括巨大配列を作らない。
        for start in range(0, len(endpoints), 512):
            origin, end = origins[start : start + 512], endpoints[start : start + 512]
            delta = end - origin
            length = np.linalg.norm(delta, axis=1)
            direction = delta / np.maximum(length[:, None], 1e-12)
            parallel = np.abs(direction) < 1e-12
            divisor = np.where(parallel, 1, direction)
            first = (self.bounds[:, 0] - origin) / divisor
            last = (self.bounds[:, 1] - origin) / divisor
            entry, exit = np.minimum(first, last), np.maximum(first, last)
            outside = (origin < self.bounds[:, 0]) | (origin > self.bounds[:, 1])
            entry = np.where(parallel, np.where(outside, np.inf, -np.inf), entry)
            exit = np.where(parallel, np.where(outside, -np.inf, np.inf), exit)
            near = np.maximum(0, entry.max(axis=1))
            far = np.minimum(length - self.resolution, exit.min(axis=1))
            valid = np.isfinite(near) & (far >= near)
            if not valid.any():
                continue
            origin, direction, near, far = origin[valid], direction[valid], near[valid], far[valid]
            steps = np.arange(int(np.ceil((far - near).max() / (self.resolution / 2))) + 1)
            times = near[:, None] + steps * (self.resolution / 2)
            positions = origin[:, None] + times[..., None] * direction[:, None]
            self._mark(positions[times <= far[:, None]])

    def sample(self, count=512, seed=0):
        if count < 1:
            raise ValueError("field query数は正数です")
        rng = np.random.default_rng(seed)
        groups = [
            np.argwhere(self.occupied),
            np.argwhere(self.known & ~self.occupied),
            np.argwhere(~self.known),
        ]
        groups = [cells for cells in groups if len(cells)]
        cells = []
        for index, size in enumerate(np.full(len(groups), count // len(groups))):
            size += int(index < count % len(groups))
            cells.append(groups[index][rng.integers(len(groups[index]), size=int(size))])
        selected = np.concatenate(cells)
        rng.shuffle(selected)
        indices = tuple(selected.T)
        coordinates = self.bounds[:, 0] + (selected + 0.5) * self.resolution
        return FieldTargets(
            coordinates.astype(np.float32),
            self.occupied[indices].astype(np.float32),
            self.known[indices].astype(np.float32),
        )


class SupervisionReader:
    """オフライン教師生成。未来データを使うのはtrajectoryのlabelだけ。"""

    def __init__(self, reader):
        self.reader = reader
        self._episodes = {}

    def velocity(self, index):
        if not 0 <= index < len(self.reader):
            raise IndexError(index)
        group = self.reader.file.get("velocity_command")
        if (
            not self.reader.file.attrs.get("patrol_training_accepted", True)
            or not self.reader.file.attrs.get("input_integrity_accepted", True)
            or group is None
            or not bool(group["valid"][index])
        ):
            return np.zeros(3, np.float32), False
        return group["applied"][index].astype(np.float32), True

    def trajectory(self, index, times_s):
        times_s = np.asarray(times_s, np.float64)
        if times_s.ndim != 1 or not np.isfinite(times_s).all() or (times_s < 0).any():
            raise ValueError("未来時刻は有限の非負な[K]秒です")
        if not 0 <= index < len(self.reader):
            raise IndexError(index)
        file = self.reader.file
        episode = int(file["state/episode_id"][index])
        if episode not in self._episodes:
            ids = np.flatnonzero(file["state/episode_id"][: len(self.reader)] == episode)
            poses = file["state/base_pose_world"][ids]
            stamps = file["state/time_s"][ids]
            poses = np.concatenate([poses, file["next/base_pose_world"][ids[-1]][None]])
            stamps = np.append(stamps, file["next/time_s"][ids[-1]])
            self._episodes[episode] = stamps, poses
        stamps, poses = self._episodes[episode]
        now, anchor = float(file["state/time_s"][index]), file["state/base_pose_world"][index]
        future = now + times_s
        valid = (future >= stamps[0] - 1e-8) & (future <= stamps[-1] + 1e-8)
        clipped = np.clip(future, stamps[0], stamps[-1])
        positions = np.column_stack([np.interp(clipped, stamps, poses[:, axis, 3]) for axis in range(3)])
        rotations = Slerp(stamps, Rotation.from_matrix(poses[:, :3, :3]))(clipped).as_matrix()
        relative_xyz = (positions - anchor[:3, 3]) @ anchor[:3, :3]
        relative_rotation = np.einsum("ij,njk->nik", anchor[:3, :3].T, rotations)
        result = np.concatenate(
            [relative_xyz, Rotation.from_matrix(relative_rotation).as_quat()], axis=1
        ).astype(np.float32)
        result[~valid] = np.array([0, 0, 0, 0, 0, 0, 1], np.float32)
        return TrajectoryTargets(times_s.astype(np.float32), result, valid)

    def field(self, index, *, count=512, history_seconds=40.96, seed=0):
        """直近1024 M×2周期×20msを既定窓に、過去rayの観測済み・表面・freeを統合する。

        knownは選んだvoxel解像度で蓄積したLiDAR ray evidenceを表す。幾何の完全な真値ではない。
        """
        if not 0 <= index < len(self.reader):
            raise IndexError(index)
        if history_seconds <= 0 or not np.isfinite(history_seconds):
            raise ValueError("教師の履歴時間は正数です")
        file = self.reader.file
        episode, now = int(file["state/episode_id"][index]), float(file["state/time_s"][index])
        anchor_from_world = np.linalg.inv(file["state/base_pose_world"][index])
        evidence = RayEvidence()
        for frame in self.reader.sensor_frames_between("lidar", episode, max(0, now - history_seconds), now):
            valid = frame["valid"]
            points = frame.get("points_world_m")
            if points is None:
                points = transform_points(frame["points_sensor_m"], frame["world_from_sensor"])
            origins = frame.get("measurement_positions_world_m")
            if origins is None:
                origins = np.broadcast_to(frame["world_from_sensor"][:3, 3], points.shape)
            evidence.add(
                transform_points(origins[valid], anchor_from_world),
                transform_points(points[valid], anchor_from_world),
            )
        return evidence.sample(count=count, seed=seed)
