"""地図生成 (build_map) の単体テスト。

Isaac Sim も ROS も使わず、人工的な観測から占有格子を作って検証する。

最も重要なのは「同じ壁を何度も観測しても壁が消えないこと」。
実測では 9385 スキャンから作った地図で障害物が 0.0% しか残らず、
レイの通過（空き）が当たり（障害物）を打ち消していた。

実行方法:
    python3 src/test_build_map.py
"""

from __future__ import annotations

import math
import pathlib
import sys

# このファイルの位置から src/ を求める（リポジトリの置き場所に依存しない）
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from build_map import (  # noqa: E402
    FREE,
    OCCUPIED,
    RESOLUTION,
    UNKNOWN,
    bresenham_free_cells,
    build_grid,
)


def make_room_observation(
    origin: tuple[float, float], half_size: float, num_beams: int = 360
) -> tuple[np.ndarray, tuple[float, float]]:
    """原点を中心とした正方形の部屋を観測した点群を作る。

    Args:
        origin: センサ位置
        half_size: 部屋の半分の一辺 [m]
        num_beams: ビーム数

    Returns:
        (点群, センサ位置)
    """
    ox, oy = origin
    points = []
    for i in range(num_beams):
        angle = -math.pi + 2 * math.pi * i / num_beams
        # 正方形の壁までの距離を求める（原点中心の正方形との交点）
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        candidates = []
        if abs(cos_a) > 1e-6:
            for wall_x in (-half_size, half_size):
                t = (wall_x - ox) / cos_a
                if t > 0 and abs(oy + t * sin_a) <= half_size + 1e-6:
                    candidates.append(t)
        if abs(sin_a) > 1e-6:
            for wall_y in (-half_size, half_size):
                t = (wall_y - oy) / sin_a
                if t > 0 and abs(ox + t * cos_a) <= half_size + 1e-6:
                    candidates.append(t)
        if not candidates:
            continue
        d = min(candidates)
        points.append((ox + d * cos_a, oy + d * sin_a))
    return np.array(points), origin


def test_bresenham_excludes_endpoint() -> bool:
    """Bresenham は終点（障害物のセル）を含めない。"""
    cells = bresenham_free_cells(0, 0, 5, 0)
    ok = (5, 0) not in cells and (0, 0) in cells and len(cells) == 5
    print(f"[{'OK' if ok else 'NG'}] Bresenham: {len(cells)} セル、終点を除外={ok}")
    return ok


def test_few_observations_make_walls() -> bool:
    """数回の観測で壁が障害物として記録される。

    同じ場所から何度観測してもビームは常に同じ点を叩くため、壁のセルは
    飛び飛びのままで孤立点の除去に消される。実運用では移動しながら
    観測するので隙間が埋まる。ここでも少し動かしながら 20 回観測する。

    空きは 1 回では確定しない。MISS_GAIN を小さく (0.05) してあるため、
    空きと確定するには同じセルを何度も通る必要がある。これは壁が
    消えないための意図的な設計。
    """
    obs = [
        make_room_observation((i * 0.05, i * 0.03), 5.0) for i in range(20)
    ]
    grid, _, _ = build_grid(obs)
    occupied = int((grid == OCCUPIED).sum())
    ok = occupied > 100
    print(f"[{'OK' if ok else 'NG'}] 20 回の観測で壁が立つ: 障害物 {occupied} セル")
    return ok


def test_walls_survive_many_observations() -> bool:
    """同じ壁を何度観測しても壁が消えない。

    これが今回の主眼。miss が hit を打ち消すと壁が消える。
    """
    # 部屋の中を動きながら 200 回観測する
    obs = []
    for i in range(200):
        t = i / 200.0 * 2 * math.pi
        pos = (2.0 * math.cos(t), 2.0 * math.sin(t))
        obs.append(make_room_observation(pos, 5.0))

    grid, _, _ = build_grid(obs)
    occupied = int((grid == OCCUPIED).sum())
    free = int((grid == FREE).sum())
    total = grid.size
    ratio = occupied / total

    # 5m 四方の部屋の周囲長は 40 m。解像度 0.05 なら約 800 セルが壁になるはず。
    ok = occupied > 400
    print(
        f"[{'OK' if ok else 'NG'}] 200 回の観測: 障害物 {occupied} セル "
        f"({ratio:.2%}) / 空き {free} セル"
        f"{'' if ok else ' <- 壁が消えている'}"
    )
    return ok


def test_unknown_outside_room() -> bool:
    """部屋の外は未知のままになる。"""
    obs = [make_room_observation((0.0, 0.0), 5.0)]
    grid, ox, oy = build_grid(obs)
    # 地図の四隅は部屋の外なので未知のはず
    corners = [grid[0, 0], grid[0, -1], grid[-1, 0], grid[-1, -1]]
    ok = all(c == UNKNOWN for c in corners)
    print(f"[{'OK' if ok else 'NG'}] 部屋の外は未知: 四隅={corners}")
    return ok


def main() -> None:
    """全テストを実行する。"""
    results = [
        test_bresenham_excludes_endpoint(),
        test_few_observations_make_walls(),
        test_unknown_outside_room(),
        test_walls_survive_many_observations(),
    ]
    passed = sum(results)
    print(f"\n{passed}/{len(results)} 件が成功しました")
    sys.exit(0 if passed == len(results) else 1)


if __name__ == "__main__":
    main()
