"""ウェイポイント列を、1102に投げられる直線区間の列へ変換する。

1102の制約（公式「4.位姿导航」の注意書き）:

> 目标点与当前位置之间的距离不超过10m，机器人会沿直线行走

つまり **(a) 1回10m以内 (b) 直線で歩く (c) 障害物では止まるだけで避けない**。
したがってこのモジュールがやることは2つ:

1. **壁を避ける折れ線を作る** — 直線で行けないならA*で迂回点を挟む。
   PC1は避けてくれないので、避けるのはこちら側の仕事
2. **折れ線を10m未満の区間に刻む** — 各区間がそのまま1102の1リクエストになる

速度は指定できない（`speed`パラメータが無い）ので、ここでは扱わない。
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from .geometry import Pose2D, interpolate, normalize_angle
from .occupancy import OccupancyGrid

# 1回の1102で進む距離の上限[m]。
# 公式の上限は10.0mだが、その10mは「目標点と**現在位置**の距離」であり、
# 現在位置は定位の推定値で誤差を持つ。さらに区間の終端でぴたりと止まらず
# 行き過ぎると、次の区間の始点がずれて距離が伸びる。
# 2.0mを誤差の吸収に充てて8.0mを既定にする。実測後に詰める。
MAX_SEGMENT_M = 8.0

# 公式の上限。ここを超えた区間を作ったらバグなので、生成後に自己検査する。
HARD_LIMIT_M = 10.0

# これ未満の移動は区間として作らない[m]。
# 折れ線の頂点が密なときに、ほぼ移動しない1102を投げ続けるのを防ぐ。
MIN_SEGMENT_M = 0.05

# その場旋回だけの1102を投げる下限[rad]。これ未満の向き直しは指示しない。
MIN_ROTATION_RAD = math.radians(3.0)

# A*で使う8近傍。斜めのコストは sqrt(2)。
_NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
]


class RouteError(ValueError):
    """経路を作れないときに投げる。原因が分かる文面にすること。"""


@dataclass(frozen=True)
class Segment:
    """1102の1リクエストに対応する直線区間。"""

    index: int
    start: Pose2D
    target: Pose2D
    is_waypoint: bool  # この区間の終点が利用者の指定したウェイポイントか

    @property
    def length(self) -> float:
        return self.start.distance_to(self.target)


def plan_route(
    grid: OccupancyGrid,
    waypoints: list[Pose2D],
    *,
    max_segment_m: float = MAX_SEGMENT_M,
) -> list[Segment]:
    """ウェイポイント列 -> 1102に投げる区間列。

    waypoints[0] は出発点（＝1804で合わせる現在位置）として扱い、
    そこへ向かう区間は作らない。
    """

    if len(waypoints) < 2:
        raise RouteError(f"ウェイポイントは出発点を含めて2点以上必要: {len(waypoints)}点")
    if max_segment_m <= MIN_SEGMENT_M:
        raise RouteError(f"max_segment_mが小さすぎる: {max_segment_m}")

    for order, pose in enumerate(waypoints):
        _reject_unreachable(grid, pose, order)

    segments: list[Segment] = []
    for leg_start, leg_goal in zip(waypoints, waypoints[1:]):
        polyline = _plan_leg(grid, leg_start, leg_goal)
        segments.extend(
            _split_polyline(polyline, leg_goal, max_segment_m, start_index=len(segments))
        )

    _assert_within_hard_limit(segments)
    return segments


def _reject_unreachable(grid: OccupancyGrid, pose: Pose2D, order: int) -> None:
    """壁の中／地図の外のウェイポイントを、投げる前に弾く。"""

    if grid.is_free(pose.x, pose.y):
        return
    raise RouteError(
        f"{order}番目のウェイポイント({pose.x:.2f}, {pose.y:.2f})は"
        "障害物の中か地図の外にある。"
        "壁から機体半径ぶん（既定0.40m）以上離れた点を指定すること"
    )


def _plan_leg(grid: OccupancyGrid, start: Pose2D, goal: Pose2D) -> list[Pose2D]:
    """2点間の、障害物に触れない折れ線を返す。返り値は start と goal を含む。"""

    if grid.is_segment_free(start.x, start.y, goal.x, goal.y):
        return [start, goal]

    cells = _astar(grid, start, goal)
    if cells is None:
        raise RouteError(
            f"({start.x:.2f}, {start.y:.2f}) から ({goal.x:.2f}, {goal.y:.2f}) へ"
            "たどり着ける経路が地図上に無い。"
            "地図が途切れているか、通路が機体幅より狭い"
        )
    points = [start] + [Pose2D(*grid.spec.to_world(c, r)) for c, r in cells] + [goal]
    return _shortcut(grid, points)


def _astar(grid: OccupancyGrid, start: Pose2D, goal: Pose2D) -> list[tuple[int, int]] | None:
    """8近傍A*。返すのは中間セルのみ（start/goalのセルは含めない）。"""

    start_cell = grid.spec.to_cell(start.x, start.y)
    goal_cell = grid.spec.to_cell(goal.x, goal.y)
    if start_cell == goal_cell:
        return []

    open_heap: list[tuple[float, tuple[int, int]]] = [(0.0, start_cell)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost_so_far: dict[tuple[int, int], float] = {start_cell: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal_cell:
            return _rebuild_path(came_from, start_cell, goal_cell)
        for dc, dr, step in _NEIGHBORS:
            neighbor = (current[0] + dc, current[1] + dr)
            if grid.is_occupied_cell(*neighbor):
                continue
            new_cost = cost_so_far[current] + step
            if new_cost >= cost_so_far.get(neighbor, math.inf):
                continue
            cost_so_far[neighbor] = new_cost
            came_from[neighbor] = current
            priority = new_cost + math.hypot(goal_cell[0] - neighbor[0], goal_cell[1] - neighbor[1])
            heapq.heappush(open_heap, (priority, neighbor))
    return None


def _rebuild_path(
    came_from: dict[tuple[int, int], tuple[int, int]],
    start_cell: tuple[int, int],
    goal_cell: tuple[int, int],
) -> list[tuple[int, int]]:
    path: list[tuple[int, int]] = []
    cursor = goal_cell
    while cursor != start_cell:
        path.append(cursor)
        cursor = came_from[cursor]
    path.reverse()
    return path[:-1]  # goalのセルは呼び側がgoal自身を足すので落とす


def _shortcut(grid: OccupancyGrid, points: list[Pose2D]) -> list[Pose2D]:
    """A*のギザギザを、見通しの利く範囲で直線に詰める（string pulling）。

    格子上の最短経路をそのまま区間にすると、0.1m刻みの1102が何百回も飛ぶ。
    見通せる最も遠い点まで一気に飛ばして頂点を減らす。
    """

    if len(points) <= 2:
        return points
    pulled = [points[0]]
    anchor = 0
    while anchor < len(points) - 1:
        farthest = anchor + 1
        for candidate in range(len(points) - 1, anchor, -1):
            if grid.is_segment_free(
                points[anchor].x, points[anchor].y, points[candidate].x, points[candidate].y
            ):
                farthest = candidate
                break
        pulled.append(points[farthest])
        anchor = farthest
    return pulled


def _split_polyline(
    polyline: list[Pose2D],
    leg_goal: Pose2D,
    max_segment_m: float,
    *,
    start_index: int,
) -> list[Segment]:
    """折れ線の各辺を max_segment_m 以内に刻む。

    刻み位置は**その辺の始点を基準に一度だけ**計算する。
    区間を1つ進めるたびに基準を動かすと、2区間目以降の目標点がずれる。
    """

    segments: list[Segment] = []
    cursor = polyline[0]
    last_edge = len(polyline) - 1
    for edge_no, edge_end in enumerate(polyline[1:], start=1):
        targets = _edge_targets(cursor, edge_end, max_segment_m)
        is_last_edge = edge_no == last_edge
        if is_last_edge:
            # 利用者が指定したウェイポイントのyawを尊重する。
            # そこで何かをする向きが決まっていることがあるため。
            targets[-1] = leg_goal
        for target in targets:
            is_leg_goal = is_last_edge and target is targets[-1]
            if not _worth_commanding(cursor, target, is_leg_goal):
                continue
            segments.append(
                Segment(
                    index=start_index + len(segments),
                    start=cursor,
                    target=target,
                    is_waypoint=is_leg_goal,
                )
            )
            cursor = target
    return segments


def _edge_targets(start: Pose2D, end: Pose2D, max_segment_m: float) -> list[Pose2D]:
    """1本の辺を刻んだ目標点列。末尾は必ず end。yawは進行方向を向く。"""

    pieces = max(1, int(math.ceil(start.distance_to(end) / max_segment_m)))
    heading = start.heading_to(end)
    targets = [interpolate(start, end, index / pieces) for index in range(1, pieces)]
    targets.append(end.with_yaw(heading))
    return targets


def _worth_commanding(cursor: Pose2D, target: Pose2D, is_leg_goal: bool) -> bool:
    """この区間を1102として投げる価値があるか。

    ほぼ動かない区間を投げ続けると到達判定が空回りする。
    ただしウェイポイント上での**その場旋回**は意味があるので落とさない
    （1102は位置だけでなく姿勢も指定するため、旋回だけの指令が成立する）。
    """

    if cursor.distance_to(target) >= MIN_SEGMENT_M:
        return True
    if not is_leg_goal:
        return False
    return abs(normalize_angle(target.yaw - cursor.yaw)) > MIN_ROTATION_RAD


def _assert_within_hard_limit(segments: list[Segment]) -> None:
    """公式の10m制限を破っていないことを生成側で確かめる。

    ここで落ちたら分割器のバグ。実機に投げてから気付くより早い。
    """

    for segment in segments:
        if segment.length > HARD_LIMIT_M:
            raise RouteError(
                f"区間{segment.index}が公式上限{HARD_LIMIT_M}mを超えている: "
                f"{segment.length:.2f}m"
            )
