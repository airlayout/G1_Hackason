"""`walk_scene.py` の「端から端」の選び方の試験。

実測地図の自由空間は**壁や未知でいくつもの島に割れる**（UiS_room_v3 で 130 個）。
島をまたげない以上、出発点をどの島に置くかで「端から端」の意味が変わる。
"""
from __future__ import annotations

import importlib.util
import sys
import unittest
from collections import deque
from pathlib import Path

import numpy as np

QUICKSTART = Path(__file__).resolve().parents[1] / "quickstart"


def _load_walk_scene():
    spec = importlib.util.spec_from_file_location(
        "walk_scene_under_test", QUICKSTART / "walk_scene.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


walk_scene = _load_walk_scene()


def path_length(free: np.ndarray, start, goal) -> int:
    """4 近傍の最短経路長。届かなければ -1。"""
    distance = np.full(free.shape, -1, np.int32)
    distance[start] = 0
    queue = deque([start])
    while queue:
        row, col = queue.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            r, c = row + dr, col + dc
            if 0 <= r < free.shape[0] and 0 <= c < free.shape[1] \
                    and free[r, c] and distance[r, c] < 0:
                distance[r, c] = distance[row, col] + 1
                queue.append((r, c))
    return int(distance[goal])


class FarthestPairTest(unittest.TestCase):
    def test_picks_the_longest_path_in_the_largest_island(self) -> None:
        """小さい島が先頭にあっても、長い島の端から端を選ぶ。

        自由セルを走査順に見つけた最初の 1 個から幅優先で広げると、
        **その島の外へは出られない**。地図の隅にある小部屋が先に見つかると、
        「端から端」がその小部屋の差し渡しになる。
        """
        free = np.zeros((20, 20), bool)
        free[0:2, 0:3] = True      # 走査順で先に当たる小さな島（差し渡し 3）
        free[10, 2:19] = True      # 本命の長い島（差し渡し 17）

        start, goal = walk_scene.farthest_pair(free)

        self.assertEqual(path_length(free, start, goal), 16,
                         f"長い島の端から端(16)を選ぶべきだが {start}->{goal} を選んだ")

    def test_returns_reachable_points(self) -> None:
        """返す 2 点は必ず互いに到達できる（島をまたがない）。"""
        free = np.zeros((12, 12), bool)
        free[1, 1:4] = True
        free[8, 2:11] = True

        start, goal = walk_scene.farthest_pair(free)

        self.assertGreaterEqual(path_length(free, start, goal), 0,
                                "到達できない 2 点を返した")


if __name__ == "__main__":
    unittest.main()
