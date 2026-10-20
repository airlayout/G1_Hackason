"""LiDAR方式ごとの走査を、共通の静的raycastと時刻付きframeへ接続する。"""

from collections import deque

import numpy as np

from g1_e2e.data.frames import LidarFrame
from g1_e2e.data.geometry import lidar_pattern


def pop_scan(pending, episode_id, time_s):
    if not pending:
        return None
    frame = pending[0]
    if frame.episode_id != episode_id:
        raise RuntimeError("別episodeのLiDARが残っています")
    return pending.popleft() if frame.time_s <= time_s + 1e-9 else None


class StaticRaycaster:
    def __init__(self, mesh, device, config, mesh_paths):
        self.mesh, self.device, self.config, self.mesh_paths = mesh, device, config, mesh_paths

    def cast(self, origins, directions, *, normals=False):
        import torch
        from isaaclab.utils.warp import raycast_mesh

        _, ranges, surface_normals, _ = raycast_mesh(
            torch.as_tensor(origins[None], device=self.device),
            torch.as_tensor(directions[None], device=self.device),
            self.mesh,
            max_dist=self.config.far_m,
            return_distance=True,
            return_normal=normals,
        )
        ranges = ranges[0].cpu().numpy()
        valid = np.isfinite(ranges) & (ranges >= self.config.near_m)
        ranges = np.where(valid, ranges, 0).astype(np.float32)
        return ranges, valid, surface_normals[0].cpu().numpy() if normals else None


class GridLidar:
    def __init__(self, env, config, raycaster, pose, base_pose):
        self.config, self.raycaster, self.pose, self.base_pose = config, raycaster, pose, base_pose
        self.physics_dt = env.config.physics_dt
        self.stride = round(config.period_s / self.physics_dt)
        self.directions, self.rings = lidar_pattern(config.channels, config.columns, config.vertical_fov_deg)
        self.pending = deque()

    @property
    def calibration(self):
        return {
            "ray_directions_sensor": self.directions,
            "ring": self.rings.astype(np.int32),
            "point_time_offset_s": np.zeros(len(self.rings), np.float32),
        }

    @property
    def metadata(self):
        return {
            "model": "instantaneous ideal 360-degree raycast; static geometry only",
            "intensity": "normalized cosine of incidence; no material reflectivity model",
            "ring": "vertical beam index",
            "point_time_offset_s": "0: simultaneous rays",
            "self_returns": False,
            "mesh_paths": self.raycaster.mesh_paths,
            "no_hit": "valid=false; never used as an occupied or free-space target",
        }

    def reset(self, episode_id):
        self.episode = episode_id
        self.pending.clear()
        self._capture(0.0)

    def advance(self, episode_id, time_s):
        if episode_id != self.episode:
            raise RuntimeError("新しいepisodeではLiDARのresetが必要です")
        if round(time_s / self.physics_dt) % self.stride == 0:
            self._capture(time_s)

    def _capture(self, time_s):
        world_from_sensor = self.pose()
        directions = self.directions @ world_from_sensor[:3, :3].T
        origins = np.broadcast_to(world_from_sensor[:3, 3], directions.shape).copy()
        ranges, valid, normals = self.raycaster.cast(origins, directions, normals=True)
        intensity = np.zeros(len(valid), np.float32)
        intensity[valid] = np.clip(-np.sum(directions[valid] * normals[valid], axis=-1), 0, 1)
        self.pending.append(
            LidarFrame(
                self.episode,
                time_s,
                self.directions * ranges[:, None],
                ranges,
                valid,
                intensity,
                world_from_sensor,
                np.linalg.inv(self.base_pose()) @ world_from_sensor,
            )
        )

    def pop(self, episode_id, time_s):
        return pop_scan(self.pending, episode_id, time_s)


def make_lidar(env, config, raycaster, pose, base_pose):
    if config.model == "livox_csv":
        from .livox import LivoxScanner

        return LivoxScanner(env, config, raycaster, pose, base_pose)
    return GridLidar(env, config, raycaster, pose, base_pose)
