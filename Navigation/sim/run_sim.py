#!/usr/bin/env python3
"""シナリオを 1 本走らせて、Navigation が最後まで巡回できるか見る。

```bash
cd Navigation

# 既定のテスト部屋を 1 周（MuJoCo + 学習済みポリシーで実際に歩く）
uv run python sim/run_sim.py

# 3 周する
uv run python sim/run_sim.py --laps 3

# 5 秒後に現れて 18 秒後に消える障害物（人が横切る想定）
uv run python sim/run_sim.py --obstacle 0.5,-1.3,0.35,5,18

# 居座る障害物（迂回する）
uv run python sim/run_sim.py --obstacle 0.5,-1.3,0.35

# 実地図(PCD)で経路だけ検証する。歩かない
uv run python sim/run_sim.py --map sim/maps/uis_main_floor.pcd --route-only

# MuJoCo のビューアで見る（macOS は mjpython が要る）
uv run mjpython sim/run_sim.py --viewer
```

`--route-only` 以外は本物の物理で歩く。前の実装（自作の等速直線）と違い、
**転倒・壁への接触・目標の行き過ぎが実際に起きる**ので、
経路が引けても走り切れないことがある。それがこの sim の目的。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav.mission import Mission, MissionOptions  # noqa: E402
from nav.occupancy import DEFAULT_INFLATION_M, OccupancyGrid  # noqa: E402
from nav.protocol import Pose2D  # noqa: E402
from nav.route import RouteError, plan_route  # noqa: E402
from sim.rooms import ROOMS, TEST_ROOM, DynamicObstacle, get_room, patrol_waypoints  # noqa: E402

# sim では 1804 に渡すパスの一致だけを見る。実機では PC1 上の実在パス。
SIM_MAP_ADDRESS = "/home/unitree/test1.pcd"

# ASCII 表示の横幅の上限[文字]。これを超える地図は間引いて描く。
RENDER_MAX_WIDTH = 110

# 実地図(PCD)から巡回地点を自動で選ぶとき、自由空間の重心側へ引き込む割合。
# 縁ぎりぎりに置くと定位のずれで壁に寄りすぎる。
CORNER_PULL_IN = 0.12


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    room, grid, waypoints = _load_world(args)

    _print_world(args, grid, waypoints)
    try:
        segments = plan_route(grid, waypoints, max_segment_m=args.max_segment)
    except (RouteError, ValueError) as error:
        # RouteError は「引けない理由」を文面に持っている。ValueError は
        # 巡回地点が地図の外など、引数の作り方が悪いとき。どちらも
        # 利用者が直せるものなので、traceback ではなく理由を出して終わる。
        print(f"\n経路を引けない: {error}", file=sys.stderr)
        return 1
    _print_route(segments, waypoints)
    print(_render(grid, segments, waypoints))

    if args.route_only:
        print("\n--route-only なのでここで終わり（歩かない）")
        return 0
    if room is None:
        print(
            "\n実地図(PCD)は MuJoCo の世界を持たないので歩けない。--route-only を付けること",
            file=sys.stderr,
        )
        return 1
    return _walk(args, room, grid, waypoints)


# ------------------------------------------------------------------- 引数


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--room", default=TEST_ROOM.name, choices=sorted(ROOMS),
                        help="MuJoCo で歩く部屋")
    parser.add_argument("--map", default=None, metavar="PCD",
                        help="部屋の代わりに実地図(PCD)を使う。経路の検証のみ（--route-only が要る）")
    parser.add_argument("--route-only", action="store_true", help="経路を引くだけで歩かない")
    parser.add_argument("--waypoint", action="append", default=[], metavar="X,Y[,YAW]",
                        help="巡回地点。省略すると部屋の四隅を回る")
    parser.add_argument("--laps", type=int, default=1, help="周回数")
    parser.add_argument("--max-segment", type=float, default=8.0,
                        help="1 区間の上限[m]（公式上限は 10）")
    parser.add_argument("--obstacle", action="append", default=[], metavar="X,Y,R[,FROM,TO]",
                        help="障害物。FROM/TO を付けると途中で現れて消える")
    parser.add_argument("--obstacle-wait", type=float, default=15.0,
                        help="迂回に切り替えるまで待つ秒数")
    parser.add_argument("--detour-limit", type=int, default=3, help="許す迂回の回数")
    parser.add_argument("--viewer", action="store_true",
                        help="MuJoCo のビューアを出す（macOS は mjpython で起動すること）")
    parser.add_argument("--speed", type=float, default=None, metavar="X",
                        help="sim を実時間の X 倍で進める。--viewer のとき既定 1.0。"
                             "省略かつ --viewer 無しなら全速（約50倍）")
    parser.add_argument("--quiet", action="store_true", help="ログを出さない")
    return parser.parse_args(argv)


def _parse_pose(text: str) -> Pose2D:
    parts = [float(piece) for piece in text.split(",")]
    if len(parts) == 2:
        return Pose2D(parts[0], parts[1])
    if len(parts) == 3:
        return Pose2D(parts[0], parts[1], parts[2])
    raise SystemExit(f"--waypoint は X,Y または X,Y,YAW で指定する: {text!r}")


def _parse_obstacle(text: str) -> DynamicObstacle:
    parts = [float(piece) for piece in text.split(",")]
    if len(parts) == 3:
        return DynamicObstacle(parts[0], parts[1], parts[2])
    if len(parts) == 5:
        return DynamicObstacle(parts[0], parts[1], parts[2],
                               appears_at_s=parts[3], disappears_at_s=parts[4])
    raise SystemExit(f"--obstacle は X,Y,R または X,Y,R,FROM,TO で指定する: {text!r}")


# ------------------------------------------------------------------- 世界


def _load_world(args) -> tuple[object | None, OccupancyGrid, list[Pose2D]]:
    if args.map:
        from nav.occupancy import load_grid

        grid = load_grid(Path(args.map))
        regions = grid.free_regions()
        if not regions:
            raise SystemExit(f"地図 {args.map} に歩ける場所が無い")
        waypoints = (
            [_parse_pose(text) for text in args.waypoint]
            if args.waypoint
            else _auto_patrol(grid, regions[0])
        )
        return None, grid, waypoints

    room = get_room(args.room)
    grid = room.grid()
    waypoints = (
        [_parse_pose(text) for text in args.waypoint] if args.waypoint else patrol_waypoints(room)
    )
    if not args.waypoint and waypoints[0] != room.spawn:
        # 1804 は機体を動かさない。出発点と実際の位置が違うと初手から経路がずれる
        waypoints = [Pose2D(room.spawn.x, room.spawn.y, waypoints[0].yaw), *waypoints[1:]]
    return room, grid, waypoints


def _auto_patrol(grid: OccupancyGrid, region) -> list[Pose2D]:
    """実地図の最大の自由領域の四隅を回るコース。"""

    world = [grid.spec.to_world(col, row) for col, row in region]
    center_x = sum(point[0] for point in world) / len(world)
    center_y = sum(point[1] for point in world) / len(world)

    def corner(sign_x: int, sign_y: int) -> Pose2D:
        x, y = max(world, key=lambda p: sign_x * p[0] + sign_y * p[1])
        return Pose2D(x + (center_x - x) * CORNER_PULL_IN, y + (center_y - y) * CORNER_PULL_IN)

    corners = [corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)]
    return corners + [corners[0]]


# ------------------------------------------------------------------- 実行


def _walk(args, room, grid: OccupancyGrid, waypoints: list[Pose2D]) -> int:
    import time

    from sim.slam_service import SimOptions, SimTransport

    obstacles = tuple(_parse_obstacle(text) for text in args.obstacle)
    # ビューアを出すなら既定で実時間。全速（約50倍）だと 100 秒の巡回が 2 秒で
    # 終わって目で追えない。--speed で明示されていればそちらを優先する。
    realtime_factor = args.speed if args.speed is not None else (1.0 if args.viewer else None)
    transport = SimTransport(
        room,
        SimOptions(
            known_maps=frozenset({SIM_MAP_ADDRESS}),
            obstacles=obstacles,
            realtime_factor=realtime_factor,
        ),
    )
    mission = Mission(
        transport,
        grid,
        waypoints,
        MissionOptions(
            map_address=SIM_MAP_ADDRESS,
            laps=args.laps,
            max_segment_m=args.max_segment,
            obstacle_wait_s=args.obstacle_wait,
            detour_limit=args.detour_limit,
        ),
    )

    pace = "全速" if realtime_factor is None else f"実時間の{realtime_factor:g}倍"
    print(f"\n歩行: MuJoCo + unitree_rl_gym 12DoF ポリシー / 障害物 {len(obstacles)}個 / {pace}")
    started = time.time()
    report = _run_with_optional_viewer(mission, transport, args.viewer)
    wall = time.time() - started

    _print_report(report, transport, wall, quiet=args.quiet)
    if transport.has_fallen:
        print("  ⚠ 転倒している。歩行ポリシーが指令に追いつけていない")
        return 3
    return 0 if report.succeeded else 2


def _run_with_optional_viewer(mission: Mission, transport, want_viewer: bool):
    """ビューアを出す場合は、描き替えをミッションの中へ差し込んで回す。

    **スレッドを使わない。** 別スレッドから `viewer.sync()` すると
    ミッション側の `mj_step` と衝突し、
    `mj_copyDataVisual: attempting to copy mjData while stack is in use` で
    プロセスごと落ちる（実測。exit 133）。
    `SimTransport` が 1 コマごとにフックを呼んでくれるので、そこで描く。
    """

    if not want_viewer:
        return mission.run()

    import mujoco.viewer

    with mujoco.viewer.launch_passive(transport.walker.model, transport.walker.data) as viewer:
        closed = False

        def draw_frame() -> None:
            nonlocal closed
            if viewer.is_running():
                viewer.sync()
            elif not closed:
                # 窓を閉じられた。待たせる相手が居ないので全速で片付ける
                closed = True
                print("\nビューアが閉じられた。残りは全速で走らせる")
                transport.set_realtime_factor(None)

        transport.set_frame_hook(draw_frame)
        try:
            return mission.run()
        finally:
            transport.set_frame_hook(None)


# ------------------------------------------------------------------- 表示


def _print_world(args, grid: OccupancyGrid, waypoints: list[Pose2D]) -> None:
    spec = grid.spec
    regions = grid.free_regions()
    source = args.map if args.map else f"部屋 {args.room}"
    print(f"世界: {source}")
    print(f"  格子 {spec.width}x{spec.height} / 解像度 {spec.resolution}m / 膨張 {DEFAULT_INFLATION_M}m")
    print(f"  自由セル {grid.free_count:,} / {spec.width * spec.height:,}")
    print(f"  連結領域 {len(regions)}個（大きい順 {[len(r) for r in regions[:5]]}）")
    if len(regions) > 1:
        print("  ⚠ 分断された領域がある。またぐ巡回地点を指定すると経路を引けない")
    print(f"  巡回地点 {len(waypoints)}個 / 周回 {args.laps}")


def _print_route(segments: list, waypoints: list[Pose2D]) -> None:
    total = sum(segment.length for segment in segments)
    longest = max((segment.length for segment in segments), default=0.0)
    print(f"\n経路: {len(segments)}区間 / 総距離 {total:.2f}m / 最長 {longest:.2f}m")
    for segment in segments:
        mark = "★" if segment.is_waypoint else " "
        print(
            f"  {mark} #{segment.index:2d} "
            f"({segment.start.x:6.2f},{segment.start.y:6.2f}) -> "
            f"({segment.target.x:6.2f},{segment.target.y:6.2f})  {segment.length:5.2f}m"
        )


def _render(grid: OccupancyGrid, segments: list, waypoints: list[Pose2D]) -> str:
    """地図と経路を ASCII で描く。'#'=通行不可 '.'=自由 '*'=経路 数字=巡回地点。

    1 文字が複数セルを束ねるので、束の中で**一番伝えたいもの**を出す。
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

    lines = [
        "",
        "地図と経路（実行前の計画。迂回すると実際の軌跡はこれと変わる）:",
        "  '#'=通行不可 '.'=自由 '*'=経路 数字=巡回地点",
    ]
    for row in range(spec.height - 1, -1, -row_step):
        lines.append(
            "  "
            + "".join(
                _block_char(grid, marks, col, row, col_step, row_step)
                for col in range(0, spec.width, col_step)
            )
        )
    return "\n".join(lines)


def _block_char(grid, marks, col: int, row: int, col_step: int, row_step: int) -> str:
    """1 文字が受け持つセルの束から、表示する 1 文字を選ぶ。"""

    best = "."
    for drow in range(row_step):
        for dcol in range(col_step):
            cell_col, cell_row = col + dcol, row - drow
            if not grid.spec.contains(cell_col, cell_row):
                continue
            mark = marks.get((cell_col, cell_row))
            if mark is not None and mark != "*":
                return mark  # 巡回地点が最優先
            if mark == "*":
                best = "*"
            elif best == "." and bool(grid.blocked[cell_row, cell_col]):
                best = "#"
    return best


def _cells_on(spec, start: Pose2D, end: Pose2D) -> list[tuple[int, int]]:
    length = math.hypot(end.x - start.x, end.y - start.y)
    steps = max(1, int(length / (spec.resolution * 0.5)))
    return [
        spec.to_cell(
            start.x + (end.x - start.x) * index / steps,
            start.y + (end.y - start.y) * index / steps,
        )
        for index in range(steps + 1)
    ]


def _print_report(report, transport, wall: float, *, quiet: bool) -> None:
    sim_time = transport.now()
    print(f"\n結果: {report.outcome.value}")
    print(f"  到達したウェイポイント {report.waypoints_reached}")
    print(f"  投げた1102          {report.segments_executed}")
    print(f"  迂回                {report.detours}")
    print(f"  sim 時間            {sim_time:.1f}s")
    print(f"  実時間              {wall:.1f}s（{sim_time / wall:.0f} 倍速）")
    print(f"  最終位置            ({transport.pose.x:+.2f}, {transport.pose.y:+.2f}) "
          f"高さ {transport.walker.height:.2f}  |action|max {transport.walker.peak_action:.2f}")
    if report.message:
        print(f"  理由                {report.message}")
    if not quiet:
        print("\nログ:")
        for line in report.log:
            print(f"  {line}")


if __name__ == "__main__":
    raise SystemExit(main())
