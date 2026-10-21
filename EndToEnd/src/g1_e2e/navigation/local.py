"""取得済みLiDARと姿勢だけで空いている進行方向を選ぶ、初期学習用の教師。"""

import numpy as np

from g1_e2e.data.geometry import base_pose, transform_points
from .patrol import PatrolTracker


class LocalPatrolTeacher(PatrolTracker):
    """地図はspawnと品質計測だけに使う。非公開の目標位置を行動へ混ぜない。"""

    def reset(self):
        super().reset()
        self.points_world = None
        self.scan_time_s = None
        self.heading_world = None

    def update_scan(self, points_world, valid, time_s):
        points = np.asarray(points_world)[valid]
        self.points_world = points[np.isfinite(points).all(axis=1)].copy()
        self.scan_time_s = float(time_s)

    def command(self, observation):
        _, yaw = self.observe(observation)
        if self.scan_time_s is None or observation.simulation_time_s - self.scan_time_s > 0.3:
            return np.zeros(3, np.float32)
        if self.scan_time_s > observation.simulation_time_s + 1e-9:
            raise ValueError("教師へ未来のLiDARを渡すことはできません")
        points = transform_points(self.points_world, np.linalg.inv(base_pose(observation)))
        # 床の反射を障害物としない。実機と同じG1の足元〜胴体を保護する高さ帯。
        obstacles = points[(points[:, 2] > -0.65) & (points[:, 2] < 0.9), :2]
        angles = np.linspace(-np.pi, np.pi, 49)
        directions = np.column_stack((np.cos(angles), np.sin(angles)))
        radius = self.config.robot_radius_m + self.config.tracking_margin_m
        if len(obstacles):
            along = obstacles @ directions.T
            lateral = np.abs(obstacles[:, :1] * directions[:, 1] - obstacles[:, 1:] * directions[:, 0])
            blocking = (along > 0) & (lateral < radius)
            clear = np.where(blocking, along, 6.0).min(axis=0)
        else:
            clear = np.full(len(angles), 6.0)
        # 空き距離を優先し、継続方向への小さい優先度で左右の振動を抑える。
        score = np.clip(clear - radius, 0, 2.5) - 0.45 * np.abs(angles)
        if self.heading_world is not None:
            previous = self.heading_world - yaw
            score += 0.3 * np.cos(angles - previous)
        score += 0.001 * np.sin(angles)  # 対称な場合は左を選び、隠れた乱数を使わない。
        best = int(np.argmax(score))
        angle = angles[best]
        available = max(0.0, clear[best] - radius - 0.15)
        speed = min(self.config.cruise_speed_m_s, 0.25, available * 0.5)
        self.heading_world = yaw + angle
        return np.array(
            [speed * np.cos(angle), speed * np.sin(angle), np.clip(0.8 * angle, -0.2, 0.2)],
            np.float32,
        )
