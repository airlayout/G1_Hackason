"""sim の部屋を、**MuJoCo の世界と占有格子の両方**として出す。

ここが Phase 3.5 で一番大事なところ。前の sim（自作の等速直線）は
「地図の上での経路」と「機体が実際に歩く世界」が別物だったので、
経路が正しくても歩けるとは限らなかった。

そこで**寸法を 1 か所に持ち**、そこから

1. MuJoCo に食わせる MJCF（床・壁・障害物）
2. `nav/occupancy.py` に食わせる点群

を生成する。地図と物理が食い違うことが原理的に起きない。

部屋は 2 通りある:

| 名前 | 由来 | いつ使うか |
|---|---|---|
| `test_room` | ここで定義した仮想の部屋 | 既定。地図と物理が必ず一致する |
| `uis_main_floor` | 実機の `artifacts/scans.npz`（UiS メインフロア） | 実データでの経路検証。MuJoCo の壁は作らない |

`uis_main_floor` を MuJoCo に持ち込むには点群をメッシュ化する必要があり、
それは経路の検証には要らない。実データは `nav/route.py` の検証に使い、
歩行は `test_room` で見る、と役割を分ける。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from nav.occupancy import OccupancyGrid, build_grid
from nav.protocol import Pose2D

ASSET_DIR = Path(__file__).resolve().parent / "assets"
ROBOT_XML = ASSET_DIR / "g1_description" / "g1_12dof.xml"

# 居ない間の障害物を退避させる高さ[m]。床（z=0）と機体から十分に離す。
PARKED_Z_M = -10.0

# 機体を置く高さ[m]。`g1_12dof.xml` の `<body name="pelvis" pos="0 0 0.793">` と揃える。
SPAWN_HEIGHT_M = 0.793

# 壁の高さ[m]。`nav/occupancy.py` が障害物とみなす帯（0.30〜1.80m）を確実にまたぐ。
WALL_HEIGHT_M = 2.0

# 壁の厚み（半分）[m]。MuJoCo の box は半寸法で指定する。
WALL_HALF_THICKNESS_M = 0.10

# 点群を打つ間隔[m]。占有格子の解像度(0.10m)より細かくして、
# 「壁の面にセルが 1 つも当たらない」隙間を作らない。
SAMPLE_STEP_M = 0.04

# 壁面に点を打つ高さ[m]。実機の LiDAR が壁を見たときに相当する。
# 障害物判定の帯(0.30〜1.80)の中に複数入れる。
WALL_SAMPLE_Z = (0.35, 0.80, 1.25, 1.70)

# 床に点を打つ間隔[m]。床の点は障害物にはならないが、
# **「観測できた場所」**として効く（未観測は通行不可になるため、
# 床を打たないと部屋の中が丸ごと通行不可になる）。
FLOOR_STEP_M = 0.10


@dataclass(frozen=True)
class Box:
    """軸に沿った直方体。中心 (x,y) と半寸法 (half_x, half_y)、高さ。

    壁も障害物もこれ 1 つで表す。MuJoCo の geom も、点群のサンプリングも、
    同じこのインスタンスから作るので食い違わない。
    """

    name: str
    x: float
    y: float
    half_x: float
    half_y: float
    height: float = WALL_HEIGHT_M

    def surface_points(self, step: float = SAMPLE_STEP_M) -> np.ndarray:
        """側面 4 面に点を打つ。返り値は (N,3)。

        上面は打たない。実機の LiDAR が壁の天面を見ることはあっても、
        それを障害物として拾うと 1.80m の上限で切られるだけで意味が無い。
        """

        xs = _inclusive_range(self.x - self.half_x, self.x + self.half_x, step)
        ys = _inclusive_range(self.y - self.half_y, self.y + self.half_y, step)
        rim = [
            (xs, np.full_like(xs, self.y - self.half_y)),
            (xs, np.full_like(xs, self.y + self.half_y)),
            (np.full_like(ys, self.x - self.half_x), ys),
            (np.full_like(ys, self.x + self.half_x), ys),
        ]
        columns = []
        for z in WALL_SAMPLE_Z:
            if z > self.height:
                continue
            for side_x, side_y in rim:
                columns.append(np.stack([side_x, side_y, np.full_like(side_x, z)], axis=1))
        return np.concatenate(columns) if columns else np.zeros((0, 3))

    def to_mjcf(self) -> str:
        return (
            f'    <geom name="{self.name}" type="box" '
            f'pos="{self.x:.3f} {self.y:.3f} {self.height / 2:.3f}" '
            f'size="{self.half_x:.3f} {self.half_y:.3f} {self.height / 2:.3f}" '
            f'rgba="0.55 0.55 0.60 1"/>'
        )


@dataclass(frozen=True)
class DynamicObstacle:
    """途中で現れて消える円柱の障害物（人が横切る状況を作る）。

    MuJoCo では geom を実行中に消せないので、**mocap body を床下へ退避させる**
    ことで「居ない」を表す。mocap は物理に従わず、位置を直接書ける body。

    `radius` は実体の半径[m]。`nav/mission.py` が迂回で地図に書き込む円とは別物
    （あちらは推定、こちらは真値）。
    """

    x: float
    y: float
    radius: float = 0.35
    height: float = 1.6
    appears_at_s: float = 0.0
    disappears_at_s: float = float("inf")

    def is_active(self, now: float) -> bool:
        return self.appears_at_s <= now < self.disappears_at_s

    def position_at(self, now: float) -> tuple[float, float, float]:
        """mocap に書き込む位置。居ないときは床下へ落とす。"""

        if not self.is_active(now):
            return (self.x, self.y, PARKED_Z_M)
        return (self.x, self.y, self.height / 2)


@dataclass(frozen=True)
class Room:
    """部屋 1 つ。MJCF と点群の両方をここから出す。"""

    name: str
    inner_x: tuple[float, float]
    inner_y: tuple[float, float]
    """壁の**内側**の範囲[m]。歩ける床はこの矩形。"""

    boxes: tuple[Box, ...] = ()
    """内側に置いた障害物（柱・棚など）。外周の壁は inner_x/inner_y から自動で作る。"""

    spawn: Pose2D = field(default=Pose2D(0.0, 0.0, 0.0))
    """機体を置く場所。MuJoCo の freejoint の初期値になる。"""

    @property
    def walls(self) -> tuple[Box, ...]:
        """外周の壁 4 枚。内側の範囲のすぐ外に、内寸を削らないように置く。"""

        (x0, x1), (y0, y1) = self.inner_x, self.inner_y
        t = WALL_HALF_THICKNESS_M
        return (
            Box("wall_south", (x0 + x1) / 2, y0 - t, (x1 - x0) / 2 + 2 * t, t),
            Box("wall_north", (x0 + x1) / 2, y1 + t, (x1 - x0) / 2 + 2 * t, t),
            Box("wall_west", x0 - t, (y0 + y1) / 2, t, (y1 - y0) / 2 + 2 * t),
            Box("wall_east", x1 + t, (y0 + y1) / 2, t, (y1 - y0) / 2 + 2 * t),
        )

    @property
    def solids(self) -> tuple[Box, ...]:
        return self.walls + self.boxes

    def point_cloud(self) -> np.ndarray:
        """この部屋を LiDAR で見たらこう写る、という点群 (N,3)。

        壁と障害物の側面 + 床。床は「観測できた」印として要る
        （`nav/occupancy.py` は未観測を通行不可にするため）。
        """

        parts = [box.surface_points() for box in self.solids]
        parts.append(self._floor_points())
        return np.concatenate(parts)

    def grid(self, **kwargs) -> OccupancyGrid:
        """`nav/route.py` に渡す通行判定格子。"""

        return build_grid(self.point_cloud(), **kwargs)

    def to_mjcf(self, *, with_robot: bool = True) -> str:
        """MuJoCo のシーン。`g1_12dof.xml` を include する。

        **このテキストは `assets/g1_description/` の中に書き出さないと動かない。**
        `include` もメッシュの相対パスもそのディレクトリ基準で解決されるため。
        書き出しは `write_scene()` が面倒を見る。
        """

        include = '  <include file="g1_12dof.xml"/>\n' if with_robot else ""
        geoms = "\n".join(box.to_mjcf() for box in self.solids)
        spawn = ""
        if with_robot:
            # qpos[0:3] は骨盤位置。高さは g1_12dof.xml の pelvis pos と同じ 0.793 にする。
            # ここがずれると歩き出す前に落下するか床にめり込む。
            spawn = (
                f'  <keyframe><key name="spawn" qpos="'
                f'{self.spawn.x:.3f} {self.spawn.y:.3f} {SPAWN_HEIGHT_M:.3f} '
                f'{np.cos(self.spawn.yaw / 2):.6f} 0 0 {np.sin(self.spawn.yaw / 2):.6f} '
                + " ".join(["-0.1 0 0 0.3 -0.2 0"] * 2)
                + '"/></keyframe>\n'
            )
        return f"""<mujoco model="{self.name}">
{include}  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0.2 0.2 0.2"/>
    <global azimuth="120" elevation="-20"/>
  </visual>
  <asset>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.20 0.30 0.40" rgb2="0.10 0.20 0.30" markrgb="0.8 0.8 0.8"
             width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5"/>
  </asset>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="groundplane"/>
{geoms}
  </worldbody>
{spawn}</mujoco>
"""

    def write_scene(self, *, with_robot: bool = True) -> Path:
        """MJCF を `assets/g1_description/` に書き出してパスを返す。

        `assets/` は `.gitignore` 済みで `fetch_assets.sh` が作る場所。
        生成物をそこに置くのは、`include` とメッシュの相対パスが
        そのディレクトリ基準でしか解決できないため。
        """

        if with_robot and not ROBOT_XML.exists():
            raise FileNotFoundError(
                f"{ROBOT_XML} が無い。先に `bash Navigation/sim/fetch_assets.sh` を実行すること"
            )
        path = ROBOT_XML.parent / f"_scene_{self.name}.xml"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.to_mjcf(with_robot=with_robot), encoding="utf-8")
        return path

    def _floor_points(self) -> np.ndarray:
        (x0, x1), (y0, y1) = self.inner_x, self.inner_y
        xs = _inclusive_range(x0, x1, FLOOR_STEP_M)
        ys = _inclusive_range(y0, y1, FLOOR_STEP_M)
        grid_x, grid_y = np.meshgrid(xs, ys)
        return np.stack(
            [grid_x.ravel(), grid_y.ravel(), np.zeros(grid_x.size)], axis=1
        )


def _inclusive_range(low: float, high: float, step: float) -> np.ndarray:
    """low から high までを step 刻みで。**両端を必ず含む。**

    `np.arange` だと浮動小数の丸めで終端が落ちることがあり、
    壁の角に点が無い＝角をすり抜けられる格子になる。
    """

    count = max(1, round((high - low) / step))
    return np.linspace(low, high, count + 1)


# ---------------------------------------------------------------- 部屋の定義

# 既定のテスト部屋。11m x 6m の一室に、柱を 1 本と東壁から伸びる仕切りを 1 枚。
#
# 寸法の根拠（**すべて「歩行ポリシーが実際に通れるか」で決めている**。
# 経路計画だけなら通る幅でも、歩容の揺れで壁に当たれば sim は落ちる）:
#
# - 11m x 6m: 巡回地点を壁から 1.0m 内側に取ると北辺が **9.0m** になり、
#   `nav/route.py` の 8m 分割が必ず 2 区間に割る。分割器が効いていることを
#   sim で確かめられる最小の広さ
# - 柱 (0.3m 角) を南辺の直線上に置く: 直線では行けないので `plan_route` が
#   折れ線を作る。柱を北に回り込むときの実クリアランスは 0.55m
# - 仕切り (4.0m) を東壁から伸ばす: 東側を南北に分断し、機体を x<1.1 まで
#   西へ戻らせる。**東側に隙間を残さない**のは、隙間を残すと膨張後 0.45m の
#   通路ができ、経路計画は通すが歩行ポリシーが通れないため
# - どの通路も膨張後の自由幅が 1.0m 以上ある
TEST_ROOM = Room(
    name="test_room",
    inner_x=(-5.5, 5.5),
    inner_y=(-3.0, 3.0),
    boxes=(
        # 南辺の直線をふさぐ柱。北へ回り込ませる
        Box("pillar", 0.0, -2.0, 0.15, 0.15),
        # 東壁から西へ 4.0m 伸びる仕切り。東側を南北に分断する
        Box("divider", 3.5, 0.0, 2.0, 0.15),
    ),
    spawn=Pose2D(-4.5, -2.0, 0.0),
)

# 障害物を置かない素の部屋。歩行そのものの確認用。
EMPTY_ROOM = Room(
    name="empty_room",
    inner_x=(-5.5, 5.5),
    inner_y=(-3.0, 3.0),
    spawn=Pose2D(-4.5, -2.0, 0.0),
)

ROOMS = {room.name: room for room in (TEST_ROOM, EMPTY_ROOM)}


def get_room(name: str) -> Room:
    try:
        return ROOMS[name]
    except KeyError:
        raise SystemExit(f"知らない部屋: {name}（あるのは {', '.join(ROOMS)}）") from None


def patrol_waypoints(room: Room, *, inset: float = 1.0) -> list[Pose2D]:
    """部屋の四隅を回って戻るコース。壁から inset[m] だけ内側に取る。

    inset の既定 1.0m は膨張半径 0.40m + 余裕。これより小さいと
    ウェイポイントが膨張後の壁の中に入って `plan_route` が弾く。
    """

    (x0, x1), (y0, y1) = room.inner_x, room.inner_y
    corners = [
        Pose2D(x0 + inset, y0 + inset),
        Pose2D(x1 - inset, y0 + inset),
        Pose2D(x1 - inset, y1 - inset),
        Pose2D(x0 + inset, y1 - inset),
    ]
    return corners + [corners[0]]


def _main() -> int:
    """`python -m sim.rooms` で部屋の様子を確かめる。"""

    import argparse

    parser = argparse.ArgumentParser(description="部屋の点群と占有格子を確かめる")
    parser.add_argument("--room", default=TEST_ROOM.name, choices=sorted(ROOMS))
    parser.add_argument("--write-scene", action="store_true", help="MJCF を書き出す")
    args = parser.parse_args()

    room = get_room(args.room)
    points = room.point_cloud()
    grid = room.grid()
    regions = grid.free_regions()
    print(f"部屋 {room.name}: 内寸 {room.inner_x} x {room.inner_y} / 障害物 {len(room.boxes)}個")
    print(f"  点群 {len(points):,}点")
    print(f"  格子 {grid.spec.width}x{grid.spec.height} / 解像度 {grid.spec.resolution}m")
    print(f"  自由セル {grid.free_count:,} / {grid.spec.width * grid.spec.height:,}")
    print(f"  連結領域 {len(regions)}個（大きい順 {[len(r) for r in regions[:5]]}）")
    for pose in patrol_waypoints(room):
        mark = "OK " if grid.is_free(pose.x, pose.y) else "NG!"
        print(f"  {mark} 巡回地点 ({pose.x:+.2f}, {pose.y:+.2f})")
    if args.write_scene:
        print(f"  MJCF -> {room.write_scene()}")
    return 0


if __name__ == "__main__":
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    raise SystemExit(_main())
