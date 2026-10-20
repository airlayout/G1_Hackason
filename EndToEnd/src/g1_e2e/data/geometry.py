"""xyzw四元数と列ベクトルの同次変換によるセンサー座標の共通処理。"""

from __future__ import annotations

import numpy as np


def rotation_xyzw(quaternion: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError("有限で非ゼロのxyzw四元数が必要です")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def pose_matrix(position: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float32)
    position = np.asarray(position, dtype=np.float32)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("位置は有限のXYZです")
    result[:3, :3] = rotation_xyzw(quaternion_xyzw)
    result[:3, 3] = position
    return result


def mount_matrix(position_m: tuple[float, ...], rpy_deg: tuple[float, ...]) -> np.ndarray:
    roll, pitch, yaw = np.deg2rad(rpy_deg)
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    result = np.eye(4, dtype=np.float32)
    result[:3, :3] = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    result[:3, 3] = position_m
    return result


def transform_points(points: np.ndarray, destination_from_source: np.ndarray) -> np.ndarray:
    return np.asarray(points) @ destination_from_source[:3, :3].T + destination_from_source[:3, 3]


def lidar_pattern(channels: int, columns: int, vertical_fov_deg: tuple[float, float]):
    """各ringの全周を等角度で照射する、瞬時スナップショット型の理想パターン。"""
    elevation = np.deg2rad(np.linspace(*vertical_fov_deg, channels))
    azimuth = np.linspace(-np.pi, np.pi, columns, endpoint=False)
    el, az = np.meshgrid(elevation, azimuth, indexing="ij")
    directions = np.stack((np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)), axis=-1)
    return directions.reshape(-1, 3).astype(np.float32), np.repeat(np.arange(channels), columns)


# Optical: X right, Y down, Z forward. Mount/base: X forward, Y left, Z up.
MOUNT_FROM_OPTICAL = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=np.float32)


def base_pose(observation) -> np.ndarray:
    """身体の基準リンクから世界座標への変換。IMU取り付け座標とは分ける。"""
    return pose_matrix(observation.privileged["base_position_world_m"], observation.orientation_xyzw)
