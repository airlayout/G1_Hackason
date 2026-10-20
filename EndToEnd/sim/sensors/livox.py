"""時刻付きメーカー光線を各物理stepで照射し、完了したscanだけを公開する。"""

from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from g1_e2e.data.frames import LidarFrame
from g1_e2e.data.livox import load_livox_pattern

from .lidar import pop_scan


def interpolate_poses(start, end, fractions):
    """物理stepの両端の実測Sim姿勢をSE(3)上で補間する。外挿はしない。"""
    fractions = np.asarray(fractions, dtype=np.float64)
    if np.any(fractions < -1e-8) or np.any(fractions > 1 + 1e-8):
        raise ValueError("pose補間は物理step内に限ります")
    fractions = np.clip(fractions, 0, 1)
    delta = Rotation.from_matrix(end[:3, :3] @ start[:3, :3].T).as_rotvec()
    rotations = Rotation.from_rotvec(fractions[:, None] * delta).as_matrix() @ start[:3, :3]
    positions = start[:3, 3] + fractions[:, None] * (end[:3, 3] - start[:3, 3])
    return rotations, positions


class LivoxScanner:
    def __init__(self, env, config, raycaster, pose, base_pose):
        self.config, self.raycaster, self.pose, self.base_pose = config, raycaster, pose, base_pose
        self.physics_dt = env.config.physics_dt
        self.pattern = load_livox_pattern(env.root, config.pattern_path, config.repeat_pattern)
        self.points_per_scan = round(config.period_s * self.pattern.rate)
        if not np.isclose(config.period_s * self.pattern.rate, self.points_per_scan):
            raise ValueError("scan周期とpoint rateが整合しません")
        if not np.isclose(
            config.period_s / env.config.physics_dt, round(config.period_s / env.config.physics_dt)
        ):
            raise ValueError("scan終了時刻は物理step境界に合わせてください")
        self.pending = deque()
        self.episode = None

    @property
    def calibration(self):
        return {
            "ray_directions_sensor": np.empty((0, 3), np.float32),
            "ring": np.empty(0, np.int32),
            "point_time_offset_s": np.empty(0, np.float64),
        }

    @property
    def metadata(self):
        return {
            **self.pattern.metadata,
            **self.pattern.source,
            "scan_period_s": self.config.period_s,
            "clock": "frame time is scan END; point_time_offset_s <= 0",
            "raw_xyz": "each point is in sensor frame at its individual measurement time",
            "points_world_m": "deskew reference using simulation truth, not deployable odometry",
            "pose_interpolation_interval_s": self.physics_dt,
            "intensity": "unmodeled; intensity_valid=false, not synthetic reflectivity",
            "geometry": "static scene only; self occlusion and dynamic objects unmodeled",
            "range_model": "ideal geometric first return; no material-dependent detection",
            "range_limits_m": [self.config.near_m, self.config.far_m],
            "noise_model": "not calibrated; no fabricated hardware noise parameters",
        }

    def reset(self, episode):
        self.episode, self.previous_time, self.next_index = episode, 0.0, 0
        self.previous_pose = self.pose()
        self.pieces = []
        self.pending.clear()

    def advance(self, episode, time_s):
        if episode != self.episode:
            raise RuntimeError("新しいepisodeではLiDARのresetが必要です")
        current = self.pose()
        # 物理周期が点周期の整数倍でなくても、未来の点を生成しない。
        end_index = int(np.floor(time_s * self.pattern.rate + 1e-7))
        indices = np.arange(self.next_index, end_index, dtype=np.int64)
        times = (indices + 1) / self.pattern.rate
        directions = self.pattern.sample(indices)
        rotations, origins = interpolate_poses(
            self.previous_pose,
            current,
            (times - self.previous_time) / (time_s - self.previous_time),
        )
        world_directions = np.einsum("nij,nj->ni", rotations, directions).astype(np.float32)
        origins = origins.astype(np.float32)
        ranges, valid, _ = self.raycaster.cast(origins, world_directions)
        world_points = origins + world_directions * ranges[:, None]
        world_points[~valid] = 0
        self.pieces.append(
            {
                "directions": directions,
                "times": times,
                "ranges": ranges,
                "valid": valid,
                "world_points": world_points,
                "origins": origins,
                "quaternions": Rotation.from_matrix(rotations).as_quat().astype(np.float32),
            }
        )
        self.previous_pose, self.previous_time, self.next_index = current, time_s, end_index
        if end_index % self.points_per_scan == 0:
            values = {key: np.concatenate([piece[key] for piece in self.pieces]) for key in self.pieces[0]}
            if len(values["ranges"]) != self.points_per_scan:
                raise RuntimeError("scan内の点番号に重複・欠損があります")
            n = self.points_per_scan
            # base_from_sensorはscan終了時点。raw XYZは各点の取得時点のsensor座標。
            self.pending.append(
                LidarFrame(
                    episode,
                    time_s,
                    values["directions"] * values["ranges"][:, None],
                    values["ranges"],
                    values["valid"],
                    np.zeros(n, np.float32),
                    current,
                    np.linalg.inv(self.base_pose()) @ current,
                    ray_directions_sensor=values["directions"],
                    point_time_offset_s=values["times"] - time_s,
                    ring=np.zeros(n, np.int32),
                    ring_valid=np.zeros(n, bool),
                    intensity_valid=np.zeros(n, bool),
                    points_world_m=values["world_points"],
                    measurement_positions_world_m=values["origins"],
                    measurement_orientation_xyzw=values["quaternions"],
                    scan_start_time_s=time_s - self.config.period_s,
                    profile_start_index=end_index - n,
                )
            )
            self.pieces.clear()

    def pop(self, episode_id, time_s):
        return pop_scan(self.pending, episode_id, time_s)
