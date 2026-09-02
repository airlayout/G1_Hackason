"""ウェイポイント列を、1102 に投げられる直線区間の列へ変換する。

1102 の制約（公式「4.位姿导航」の注意書き）:

> 目标点与当前位置之间的距离不超过10m，机器人会沿直线行走

つまり **(a) 1回10m以内 (b) 直線で歩く (c) 障害物では止まるだけで避けない**。
したがってここでやることは 2 つ。

1. **壁を避ける折れ線を作る** — 直線で行けないなら迂回点を挟む。
   PC1 は避けてくれないので、避けるのはこちら側の仕事
2. **折れ線を 10m 未満の区間に刻む** — 各区間がそのまま 1102 の 1 リクエストになる

**格子上の経路探索は `skimage.graph.route_through_array` に任せる。**
自作するのは 1102 固有の決め事（8m 分割、姿勢の与え方）だけ。
速度は指定できない（`speed` パラメータが無い）のでここでは扱わない。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from skimage.graph import route_through_array

from .occupancy import OccupancyGrid
from .protocol import Pose2D

# 1回の 1102 で進む距離の上限[m]。
# 公式の上限は 10.0m だが、その 10m は「目標点と**現在位置**の距離」であり、
# 現在位置は定位の推定値で誤差を持つ。さらに区間の終端で行き過ぎると、
# 次の区間の始点がずれて距離が伸びる。2.0m を誤差の吸収に充てて 8.0m を既定にする。
MAX_SEGMENT_M = 8.0

# 公式の上限。ここを超えた区間を作ったらバグなので、生成後に自己検査する。
HARD_LIMIT_M = 10.0

# これ未満の移動は区間として作らない[m]。
MIN_SEGMENT_M = 0.05

# その場旋回だけの 1102 を投げる下限[rad]。これ未満の向き直しは指示しない。
MIN_ROTATION_RAD = math.radians(3.0)


class RouteError(ValueError):
    """経路を作れないときに投げる。原因が分かる文面にすること。"""


@dataclass(frozen=True)
class Segment:
    """1102 の 1 リクエストに対応する直線区間。"""

    index: int
    start: Pose2D
    target: Pose2D
    is_waypoint: bool  # この区間の終点が利用者の指定したウェイポイントか

    @property
    def length(self) -> float:
        return math.hypot(self.target.x - self.start.x, self.target.y - self.start.y)


def plan_route(
    grid: OccupancyGrid,
    waypoints: list[Pose2D],
    *,
    max_segment_m: float = MAX_SEGMENT_M,
) -> list[Segment]:
    """ウェイポイント列 -> 1102 に投げる区間列。

    waypoints[0] は出発点（＝1804 で合わせる現在位置）として扱い、
    そこへ向かう区間は作らない。
    """

    if len(waypoints) < 2:
        raise RouteError(f"ウェイポイントは出発点を含めて2点以上必要: {len(waypoints)}点")
    if max_segment_m <= MIN_SEGMENT_M:
        raise RouteError(f"max_segment_m が小さすぎる: {max_segment_m}")
    for order, pose in enumerate(waypoints):
        if not grid.is_free(pose.x, pose.y):
            raise RouteError(
                f"{order}番目のウェイポイント({pose.x:.2f}, {pose.y:.2f})は"
                "障害物の中か地図の外にある。"
                "壁から機体半径ぶん（既定0.40m）以上離れた点を指定すること"
            )

    segments: list[Segment] = []
    for start, goal in zip(waypoints, waypoints[1:]):
        polyline = _plan_leg(grid, start, goal)
        segments.extend(_split_polyline(polyline, goal, max_segment_m, len(segments)))

    for segment in segments:
        if segment.length > HARD_LIMIT_M:
            raise RouteError(
                f"区間{segment.index}が公式上限{HARD_LIMIT_M}mを超えている: {segment.length:.2f}m"
            )
    return segments


def _plan_leg(grid: OccupancyGrid, start: Pose2D, goal: Pose2D) -> list[Pose2D]:
    """2点間の、障害物に触れない折れ線。返り値は start と goal を含む。"""

    if grid.is_segment_free(start.x, start.y, goal.x, goal.y):
        return [start, goal]

    # 通行不可セルを無限コストにして最短経路を解く（skimage が MCP で解く）
    cost = np.where(grid.blocked, np.inf, 1.0)
    c0, r0 = grid.spec.to_cell(start.x, start.y)
    c1, r1 = grid.spec.to_cell(goal.x, goal.y)
    try:
        # fully_connected=False（4近傍）にするのは、**斜め移動が壁の角を横切る**のを
        # 防ぐため。8近傍だと planner は「通れる」とする経路を作るが、こちらの線分判定
        # （line_aa。かすめたセルも数える）は同じ動きを「通れない」と判定し、
        # 生成した経路が自分の検査に落ちる。実測で 59 セル中 3 箇所が食い違った。
        # 経路は少し長くなるが、後段の string pulling で直線に詰め直されるので影響は小さい。
        cells, _ = route_through_array(
            cost, (int(r0), int(c0)), (int(r1), int(c1)),
            fully_connected=False, geometric=True,
        )
    except ValueError as error:
        raise RouteError(
            f"({start.x:.2f}, {start.y:.2f}) から ({goal.x:.2f}, {goal.y:.2f}) へ"
            f"たどり着ける経路が地図上に無い。地図が途切れているか、通路が機体幅より狭い（{error}）"
        ) from error

    xs, ys = grid.spec.to_world(np.array([c for _, c in cells]), np.array([r for r, _ in cells]))
    points = [start] + [Pose2D(float(x), float(y)) for x, y in zip(xs, ys)] + [goal]
    return _shortcut(grid, points)


def _shortcut(grid: OccupancyGrid, points: list[Pose2D]) -> list[Pose2D]:
    """格子経路のギザギザを、見通しの利く範囲で直線に詰める（string pulling）。

    格子上の最短経路をそのまま区間にすると、0.1m 刻みの 1102 が何百回も飛ぶ。
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
    polyline: list[Pose2D], leg_goal: Pose2D, max_segment_m: float, start_index: int
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
            # 利用者が指定したウェイポイントの yaw を尊重する。
            # そこで何かをする向きが決まっていることがあるため。
            targets[-1] = leg_goal
        for target in targets:
            is_leg_goal = is_last_edge and target is targets[-1]
            if not _worth_commanding(cursor, target, is_leg_goal):
                continue
            segments.append(
                Segment(start_index + len(segments), cursor, target, is_leg_goal)
            )
            cursor = target
    return segments


def _edge_targets(start: Pose2D, end: Pose2D, max_segment_m: float) -> list[Pose2D]:
    """1本の辺を刻んだ目標点列。末尾は必ず end。yaw は進行方向を向く。"""

    length = math.hypot(end.x - start.x, end.y - start.y)
    pieces = max(1, math.ceil(length / max_segment_m))
    heading = math.atan2(end.y - start.y, end.x - start.x) if length > 1e-9 else start.yaw
    targets = [
        Pose2D(
            start.x + (end.x - start.x) * i / pieces,
            start.y + (end.y - start.y) * i / pieces,
            heading,
        )
        for i in range(1, pieces)
    ]
    targets.append(Pose2D(end.x, end.y, heading))
    return targets


def _worth_commanding(cursor: Pose2D, target: Pose2D, is_leg_goal: bool) -> bool:
    """この区間を 1102 として投げる価値があるか。

    ほぼ動かない区間を投げ続けると到達判定が空回りする。ただしウェイポイント上での
    **その場旋回**は意味があるので落とさない（1102 は位置だけでなく姿勢も指定する）。
    """

    if math.hypot(target.x - cursor.x, target.y - cursor.y) >= MIN_SEGMENT_M:
        return True
    if not is_leg_goal:
        return False
    delta = math.atan2(math.sin(target.yaw - cursor.yaw), math.cos(target.yaw - cursor.yaw))
    return abs(delta) > MIN_ROTATION_RAD
