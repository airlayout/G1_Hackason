"""ロボット半径を含む通行可能格子と、角を切らないA*探索。"""

from dataclasses import dataclass
import heapq

import numpy as np


@dataclass(frozen=True)
class NavigationGrid:
    origin_xy_m: np.ndarray
    resolution_m: float
    walkable: np.ndarray
    clearance_m: np.ndarray

    def __post_init__(self):
        if self.walkable.ndim != 2 or self.walkable.dtype != np.bool_ or not self.walkable.any():
            raise ValueError("通行可能な2次元bool格子が必要です")
        if self.clearance_m.shape != self.walkable.shape or self.resolution_m <= 0:
            raise ValueError("格子の解像度・clearance形状が不正です")

    def cell(self, xy):
        x, y = np.floor((np.asarray(xy) - self.origin_xy_m) / self.resolution_m).astype(int)
        return int(y), int(x)

    def xy(self, cells):
        return self.origin_xy_m + (np.asarray(cells)[..., ::-1] + 0.5) * self.resolution_m

    def nearest(self, xy):
        row, column = self.cell(xy)
        if (
            0 <= row < self.walkable.shape[0]
            and 0 <= column < self.walkable.shape[1]
            and self.walkable[row, column]
        ):
            return row, column
        cells = np.argwhere(self.walkable)
        return tuple(cells[np.argmin(np.sum((self.xy(cells) - xy) ** 2, axis=1))])

    def clearance(self, xy):
        row, column = self.cell(xy)
        if not 0 <= row < self.walkable.shape[0] or not 0 <= column < self.walkable.shape[1]:
            return 0.0
        return float(self.clearance_m[row, column])

    def segment_walkable(self, start_xy, end_xy):
        # 等間隔sampleでは角をかすめる短い区間を見落とす。格子境界の全交点で分割する。
        start = (np.asarray(start_xy) - self.origin_xy_m) / self.resolution_m
        end = (np.asarray(end_xy) - self.origin_xy_m) / self.resolution_m
        delta = end - start
        crossings = [0.0, 1.0]
        for axis in range(2):
            if abs(delta[axis]) < 1e-12:
                continue
            low, high = sorted([start[axis], end[axis]])
            lines = np.arange(np.floor(low) + 1, np.ceil(high))
            crossings.extend(((lines - start[axis]) / delta[axis]).tolist())
        times = np.unique(crossings)
        samples = np.concatenate([[0.0, 1.0], (times[:-1] + times[1:]) / 2])
        cells = np.floor(start + samples[:, None] * delta).astype(int)
        x, y = cells[:, 0], cells[:, 1]
        if np.any((x < 0) | (y < 0) | (x >= self.walkable.shape[1]) | (y >= self.walkable.shape[0])):
            return False
        return bool(self.walkable[y, x].all())

    def path(self, start_xy, goal_xy):
        start, goal = self.nearest(start_xy), self.nearest(goal_xy)
        queue, costs, previous = [(0.0, start)], {start: 0.0}, {}
        height, width = self.walkable.shape
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                cells = [goal]
                while cells[-1] != start:
                    cells.append(previous[cells[-1]])
                return self.xy(np.asarray(cells[::-1])).astype(np.float32)
            y, x = current
            for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                yy, xx = y + dy, x + dx
                if not (0 <= yy < height and 0 <= xx < width and self.walkable[yy, xx]):
                    continue
                if dx and dy and not (self.walkable[y, xx] and self.walkable[yy, x]):
                    continue
                neighbor = yy, xx
                # 狭い障害物際より、同じ距離なら通路中央を優先する。
                distance = np.hypot(dx, dy) * (1 + 0.15 / max(float(self.clearance_m[yy, xx]), 0.05))
                cost = costs[current] + distance
                if cost >= costs.get(neighbor, float("inf")):
                    continue
                costs[neighbor], previous[neighbor] = cost, current
                heuristic = np.hypot(yy - goal[0], xx - goal[1])
                heapq.heappush(queue, (cost + heuristic, neighbor))
        raise ValueError("同じ連結領域内に巡回経路を作れません")

    def save(self, path):
        np.savez_compressed(
            path,
            origin_xy_m=self.origin_xy_m,
            resolution_m=self.resolution_m,
            walkable=self.walkable,
            clearance_m=self.clearance_m,
        )
