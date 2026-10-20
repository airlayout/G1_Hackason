"""剛体の運動学と公式取り付け変換から、IMU位置の慣性量を求める。"""

import numpy as np
from scipy.spatial.transform import Rotation


from .frames import ImuFrame


def measure_imu(
    *,
    episode_id,
    time_s,
    world_from_body,
    body_from_sensor,
    com_position_world_m,
    com_acceleration_world_m_s2,
    angular_velocity_world_rad_s,
    angular_acceleration_world_rad_s2,
    gravity_world_m_s2,
    world_from_base,
    acceleration_valid=True,
):
    """a_sensor = a_COM + α×r + ω×(ω×r)。rはCOMからセンサーまでの世界座標。

    姿勢はsensor→worldのxyzw。比力と角速度は取り付け後のsensor座標。
    電子回路・ノイズ・フィルタの仮定は幾何変換へ混在させない。
    """
    world_from_sensor = np.asarray(world_from_body, np.float64) @ body_from_sensor
    rotation = world_from_sensor[:3, :3]
    lever = world_from_sensor[:3, 3] - com_position_world_m
    omega = np.asarray(angular_velocity_world_rad_s)
    acceleration = np.asarray(com_acceleration_world_m_s2) + np.cross(
        angular_acceleration_world_rad_s2, lever
    )
    acceleration += np.cross(omega, np.cross(omega, lever))
    gravity = np.asarray(gravity_world_m_s2)
    if not np.isfinite(gravity).all() or np.linalg.norm(gravity) < 1e-8:
        raise ValueError("IMUの重力基準には有限の非ゼロ重力が必要です")
    return ImuFrame(
        episode_id,
        time_s,
        Rotation.from_matrix(rotation).as_quat().astype(np.float32),
        (rotation.T @ omega).astype(np.float32),
        (rotation.T @ (acceleration - gravity)).astype(np.float32),
        (rotation.T @ (gravity / np.linalg.norm(gravity))).astype(np.float32),
        world_from_sensor.astype(np.float32),
        (np.linalg.inv(world_from_base) @ world_from_sensor).astype(np.float32),
        acceleration_valid,
    )
