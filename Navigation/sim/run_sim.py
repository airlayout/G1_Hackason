#!/usr/bin/env python3
"""地図PCDを相手に、Navigationのミッションを最後まで走らせて評価する。

**実機・PC2・DDS・numpyのいずれも不要。** 標準ライブラリだけで動く。

```bash
# 既定の地図(sim/maps/sim_room.pcd)で四隅を1周する
python3 Navigation/sim/run_sim.py

# 3周させ、途中に居座る障害物を1つ置く
python3 Navigation/sim/run_sim.py --laps 3 --obstacle 0,0,0.8

# 5秒後に現れて15秒後に消える障害物（人が横切る想定）
python3 Navigation/sim/run_sim.py --obstacle 1.5,0,0.6,5,15

# 運動を積分しないmockモード（プロトコルと状態遷移だけ見る）
python3 Navigation/sim/run_sim.py --mock
```

`--kinematics`（既定）と `--mock` の違いは相手の作りだけで、流すMissionは同じ。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav.geometry import Pose2D
from nav.mission import Mission, MissionOptions
from nav.occupancy import DEFAULT_INFLATION_M, OccupancyGrid, load_grid
from nav.route import plan_route
from sim.fake_service import FakeOptions, FakeTransport, Obstacle

DEFAULT_MAP = Path(__file__).resolve().parent / "maps" / "sim_room.pcd"

# 巡回地点を地図の縁ぎりぎりに置くと、定位のずれで壁に寄りすぎる。
# 自由空間の重心側へこの割合だけ引き込む。
CORNER_PULL_IN = 0.12

# ASCII表示の横幅の上限[文字]。これを超える地図は間引いて描く。
RENDER_MAX_WIDTH = 110


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    grid = load_grid(Path(args.map))
    regions = grid.free_regions()
    if not regions:
        print(f"地図 {args.map} に歩ける場所が無い", file=sys.stderr)
        return 1

    waypoints = _resolve_waypoints(args, grid, regions[0])
    _print_map_summary(args.map, grid, regions)

    try:
        segments = plan_route(grid, waypoints, max_segment_m=args.max_segment)
    except Exception as error:  # RouteError も含めて理由を出して終わる
        print(f"\n経路を引けない: {error}", file=sys.stderr)
        return 1
    _print_route(segments, waypoints)
    print(_render(grid, segments, waypoints))

    report = _run_mission(args, grid, waypoints)
    _print_report(report, args)
    return 0 if report.succeeded else 2


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", default=str(DEFAULT_MAP), help="地図PCD（binary, FIELDS x y z）")
    parser.add_argument("--waypoint", action="append", default=[], metavar="X,Y[,YAW]",
                        help="巡回地点。省略すると自由空間の四隅を自動で選ぶ")
    parser.add_argument("--laps", type=int, default=1, help="周回数")
    parser.add_argument("--max-segment", type=float, default=8.0, help="1区間の上限[m]（公式上限は10）")
    parser.add_argument("--obstacle", action="append", default=[], metavar="X,Y,R[,FROM,TO]",
                        help="偽の障害物。FROM/TOを付けると途中で現れて消える")
    parser.add_argument("--mock", action="store_true", help="運動を積分しない（プロトコルだけ見る）")
    parser.add_argument("--obstacle-wait", type=float, default=15.0, help="迂回に切り替えるまで待つ秒数")
    parser.add_argument("--detour-limit", type=int, default=3, help="許す迂回の回数")
    parser.add_argument("--map-address", default="/home/unitree/test1.pcd",
                        help="1804に渡すPC1上のパス（simでは値の一致だけを見る）")
    return parser.parse_args(argv)


def _resolve_waypoints(args, grid: OccupancyGrid, region: list) -> list[Pose2D]:
    if args.waypoint:
        return [_parse_pose(text) for text in args.waypoint]
    return _auto_patrol(grid, region)


def _parse_pose(text: str) -> Pose2D:
    parts = [float(piece) for piece in text.split(",")]
    if len(parts) == 2:
        return Pose2D(parts[0], parts[1])
    if len(parts) == 3:
        return Pose2D(parts[0], parts[1], parts[2])
    raise SystemExit(f"--waypoint は X,Y または X,Y,YAW で指定する: {text!r}")


def _parse_obstacle(text: str) -> Obstacle:
    parts = [float(piece) for piece in text.split(",")]
    if len(parts) == 3:
        return Obstacle(parts[0], parts[1], parts[2])
    if len(parts) == 5:
        return Obstacle(parts[0], parts[1], parts[2], parts[3], parts[4])
    raise SystemExit(f"--obstacle は X,Y,R または X,Y,R,FROM,TO で指定する: {text!r}")


def _auto_patrol(grid: OccupancyGrid, region: list) -> list[Pose2D]:
    """最大の自由領域の四隅を回るコースを作り、始点に戻る。"""

    world = [grid.spec.to_world(col, row) for col, row in region]
    center_x = sum(point[0] for point in world) / len(world)
    center_y = sum(point[1] for point in world) / len(world)

    def corner(sign_x: int, sign_y: int) -> Pose2D:
        x, y = max(world, key=lambda p: sign_x * p[0] + sign_y * p[1])
        return Pose2D(
            x + (center_x - x) * CORNER_PULL_IN, y + (center_y - y) * CORNER_PULL_IN
        )

    corners = [corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)]
    return corners + [corners[0]]


def _run_mission(args, grid: OccupancyGrid, waypoints: list[Pose2D]):
    transport = FakeTransport(
        waypoints[0],
        FakeOptions(
            kinematics=not args.mock,
            known_maps=frozenset({args.map_address}),
            obstacles=tuple(_parse_obstacle(text) for text in args.obstacle),
        ),
    )
    mission = Mission(
        transport,
        grid,
        waypoints,
        MissionOptions(
            map_address=args.map_address,
            laps=args.laps,
            max_segment_m=args.max_segment,
            obstacle_wait_s=args.obstacle_wait,
            detour_limit=args.detour_limit,
        ),
    )
    return mission.run()


# ---------------------------------------------------------------------- 表示


def _print_map_summary(path: str, grid: OccupancyGrid, regions: list) -> None:
    spec = grid.spec
    free = sum(len(region) for region in regions)
    print(f"地図: {path}")
    print(f"  格子 {spec.width}x{spec.height} / 解像度 {spec.resolution}m / 膨張 {DEFAULT_INFLATION_M}m")
    print(f"  自由セル {free:,} / {spec.width * spec.height:,}")
    print(f"  連結領域 {len(regions)}個（大きい順 {[len(r) for r in regions[:5]]}）")
    if len(regions) > 1:
        print("  ⚠ 分断された領域がある。またぐ巡回地点を指定すると経路を引けない")


def _print_route(segments: list, waypoints: list[Pose2D]) -> None:
    total = sum(segment.length for segment in segments)
    longest = max((segment.length for segment in segments), default=0.0)
    print(f"\n経路: {len(waypoints)}地点 / {len(segments)}区間 / 総距離 {total:.2f}m / 最長 {longest:.2f}m")
    for segment in segments:
        mark = "★" if segment.is_waypoint else " "
        print(
            f"  {mark} #{segment.index:2d} "
            f"({segment.start.x:6.2f},{segment.start.y:6.2f}) -> "
            f"({segment.target.x:6.2f},{segment.target.y:6.2f})  {segment.length:5.2f}m"
        )


def _render(grid: OccupancyGrid, segments: list, waypoints: list[Pose2D]) -> str:
    """地図と経路をASCIIで描く。'#'=通行不可 '.'=自由 '*'=経路 数字=巡回地点。

    1文字が複数セルを束ねるので、束の中で**一番伝えたいもの**を出す。
    優先順位は 巡回地点 > 経路 > 障害物 > 自由。経路が壁に隠れると
    「壁を貫いていないか」を目で確かめられなくなるため。
    """

    spec = grid.spec
    col_step = max(1, -(-spec.width // RENDER_MAX_WIDTH))
    row_step = col_step * 2  # 文字は横より縦に長いので行を余分に間引く

    marks: dict[tuple[int, int], str] = {}
    for segment in segments:
        for cell in _cells_on(spec, segment.start, segment.target):
            marks.setdefault(cell, "*")
    for order, pose in enumerate(waypoints):
        marks[spec.to_cell(pose.x, pose.y)] = str(order % 10)

    lines = ["", "地図と経路（実行前の計画。迂回すると実際の軌跡はこれと変わる）:",
             "  '#'=通行不可 '.'=自由 '*'=経路 数字=巡回地点"]
    for row in range(spec.height - 1, -1, -row_step):
        line = "".join(
            _block_char(grid, marks, col, row, col_step, row_step)
            for col in range(0, spec.width, col_step)
        )
        lines.append("  " + line)
    return "\n".join(lines)


def _block_char(
    grid: OccupancyGrid,
    marks: dict[tuple[int, int], str],
    col: int,
    row: int,
    col_step: int,
    row_step: int,
) -> str:
    """1文字が受け持つセルの束から、表示する1文字を選ぶ。"""

    best = "."
    for drow in range(row_step):
        for dcol in range(col_step):
            cell = (col + dcol, row - drow)
            if not grid.spec.contains(*cell):
                continue
            mark = marks.get(cell)
            if mark is not None and mark != "*":
                return mark  # 巡回地点が最優先
            if mark == "*":
                best = "*"
            elif best == "." and grid.is_occupied_cell(*cell):
                best = "#"
    return best


def _cells_on(spec, start: Pose2D, end: Pose2D) -> list[tuple[int, int]]:
    length = start.distance_to(end)
    steps = max(1, int(length / (spec.resolution * 0.5)))
    cells = []
    for index in range(steps + 1):
        ratio = index / steps
        cells.append(
            spec.to_cell(
                start.x + (end.x - start.x) * ratio, start.y + (end.y - start.y) * ratio
            )
        )
    return cells


def _print_report(report, args) -> None:
    print(f"\n結果: {report.outcome.value}")
    print(f"  到達したウェイポイント {report.waypoints_reached}")
    print(f"  投げた1102          {report.segments_executed}")
    print(f"  迂回                {report.detours}")
    print(f"  所要時間            {report.elapsed_s:.1f}s（{'仮想時間・mock' if args.mock else '仮想時間'}）")
    if report.message:
        print(f"  理由                {report.message}")
    print("\nログ:")
    for line in report.log:
        print(f"  {line}")


if __name__ == "__main__":
    raise SystemExit(main())
