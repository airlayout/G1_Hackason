"""保存形式やシミュレーターへ依存しない、時刻付きセンサー観測。"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    episode_id: int
    time_s: float
    rgb: np.ndarray
    depth_m: np.ndarray
    depth_valid: np.ndarray
    world_from_optical: np.ndarray
    base_from_optical: np.ndarray
    native_depth_z16: np.ndarray | None = None
    native_depth_valid: np.ndarray | None = None
    world_from_depth_optical: np.ndarray | None = None


@dataclass(frozen=True)
class LidarFrame:
    episode_id: int
    time_s: float
    points_sensor_m: np.ndarray
    ranges_m: np.ndarray
    valid: np.ndarray
    intensity: np.ndarray
    world_from_sensor: np.ndarray
    base_from_sensor: np.ndarray
    ray_directions_sensor: np.ndarray | None = None
    point_time_offset_s: np.ndarray | None = None
    ring: np.ndarray | None = None
    ring_valid: np.ndarray | None = None
    intensity_valid: np.ndarray | None = None
    points_world_m: np.ndarray | None = None
    measurement_positions_world_m: np.ndarray | None = None
    measurement_orientation_xyzw: np.ndarray | None = None
    scan_start_time_s: float | None = None
    profile_start_index: int | None = None


@dataclass(frozen=True)
class ImuFrame:
    episode_id: int
    time_s: float
    orientation_xyzw: np.ndarray
    angular_velocity_sensor_rad_s: np.ndarray
    specific_force_sensor_m_s2: np.ndarray
    projected_gravity_sensor: np.ndarray
    world_from_sensor: np.ndarray
    base_from_sensor: np.ndarray
    acceleration_valid: bool
