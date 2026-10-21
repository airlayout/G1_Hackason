"""未訪問の領域へ向かい続ける、速度指令の幾何教師。"""

import numpy as np

from g1_e2e.data.geometry import rotation_xyzw


class PatrolTeacher:
    """真値地図・姿勢を使う収集専用教師。目標や経路はモデルの入力へ混ぜない。"""

    def __init__(self, grid, config):
        self.grid, self.config = grid, config
        cells = np.argwhere(grid.walkable)
        # 同じ部屋の微小な格子差より、フロア内の離れた領域を巡回する。
        coarse = np.floor(grid.xy(cells) / 1.5).astype(int)
        _, indices = np.unique(coarse, axis=0, return_index=True)
        self.candidates = grid.xy(cells[np.sort(indices)])
        self.reset()

    def reset(self):
        self.visits = np.zeros(len(self.candidates), np.int64)
        self.route = np.empty((0, 2), np.float32)
        self.route_index = self.goals_reached = self.replans = 0
        self.goal = None
        self.distance_m = 0.0
        self.previous_xy = None
        self.minimum_clearance_m = float("inf")

    def spawn(self, seed):
        rng = np.random.default_rng(seed)
        suitable = np.argwhere(
            self.grid.walkable & (self.grid.clearance_m >= self.config.robot_radius_m + 0.35)
        )
        if not len(suitable):
            suitable = np.argwhere(self.grid.walkable)
        # 同じseedで同じ開始位置・向きになり、episodeごとには場所を変えられる。
        xy = self.grid.xy(suitable[rng.integers(len(suitable))])
        return float(xy[0]), float(xy[1]), float(rng.uniform(-np.pi, np.pi))

    def _plan(self, xy, yaw):
        offset = self.candidates - xy
        distance = np.linalg.norm(offset, axis=1)
        directions = offset / np.maximum(distance[:, None], 1e-6)
        alignment = directions @ np.array([np.cos(yaw), np.sin(yaw)])
        # 訪問数を主目的にし、近過ぎる目標と不必要な大旋回を避ける。
        score = -10 * self.visits + 2 * alignment - np.abs(distance - 5.0)
        score[distance < 2.0] = -np.inf
        if not np.isfinite(score).any():
            score = distance
        self.goal = self.candidates[int(np.argmax(score))]
        self.route = self.grid.path(xy, self.goal)
        self.route_index = 0
        self.replans += 1

    def command(self, observation):
        xy = observation.privileged["base_position_world_m"][:2].astype(float)
        rotation = rotation_xyzw(observation.orientation_xyzw)
        yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
        if self.previous_xy is not None:
            self.distance_m += float(np.linalg.norm(xy - self.previous_xy))
        self.previous_xy = xy.copy()
        self.visits[np.linalg.norm(self.candidates - xy, axis=1) < 1.25] += 1
        self.minimum_clearance_m = min(self.minimum_clearance_m, self.grid.clearance(xy))
        if self.goal is None or np.linalg.norm(self.goal - xy) < self.config.goal_tolerance_m:
            self.goals_reached += int(self.goal is not None)
            self._plan(xy, yaw)
        window_end = min(
            len(self.route),
            self.route_index + max(2, round(2 * self.config.lookahead_m / self.grid.resolution_m)),
        )
        distances = np.linalg.norm(self.route[self.route_index : window_end] - xy, axis=1)
        self.route_index += int(np.argmin(distances))
        target_index = self.route_index
        length = 0.0
        while target_index + 1 < len(self.route) and length < self.config.lookahead_m:
            if not self.grid.segment_walkable(xy, self.route[target_index + 1]):
                break
            length += float(np.linalg.norm(self.route[target_index + 1] - self.route[target_index]))
            target_index += 1
        offset = self.route[target_index] - xy
        angle = np.arctan2(offset[1], offset[0]) - yaw
        error = float(np.arctan2(np.sin(angle), np.cos(angle)))
        # 公開v0歩行方策は並進ゼロの旋回にほぼ応答しない。G1の横歩きも使い、
        # 進行方向へ並進しながらheadingを追従させる。速度範囲はv0の契約内。
        speed = min(self.config.cruise_speed_m_s, 0.3, 1.5 * float(np.linalg.norm(offset)))
        command = np.array(
            [speed * np.cos(error), speed * np.sin(error), np.clip(0.8 * error, -0.2, 0.2)], np.float32
        )
        return command

    @property
    def summary(self):
        return {
            "distance_m": self.distance_m,
            "goals_reached": self.goals_reached,
            "replans": self.replans,
            "visited_regions": int((self.visits > 0).sum()),
            "total_regions": len(self.visits),
            "minimum_map_clearance_m": self.minimum_clearance_m if self.previous_xy is not None else None,
        }
