"""地図PCDから2Dの占有格子を作る。標準ライブラリのみ。

なぜ必要か: 1102は**目標点まで直線で歩き、障害物に遭遇したら止まる**
（`mode`は`1`固定で绕障モードが無い）。壁を貫く指令を投げると必ず詰まるので、
「その直線を歩けるか」を投げる前に地図で判定する必要がある。
この判定に使う地図がここで作る占有格子。

numpyを使わないのはCIの都合。`scripts/ci/run_all_tests.sh` は `pip install` を
持たず、`Mapping/` も同じ理由で標準ライブラリだけで書かれている。
10m×8mの部屋を0.1mで切っても100×80=8000セルなので、これで十分速い。
"""

from __future__ import annotations

import math
import struct
from collections import deque
from dataclasses import dataclass
from pathlib import Path

# 格子の解像度[m]。G1の足の置き場を議論する解像度ではなく、
# 「その直線が壁を貫くか」を見るための解像度。
DEFAULT_RESOLUTION_M = 0.10

# 障害物とみなす高さの範囲[m]。座標系の原点はMid360-IMU（公式「一、介绍」）。
# 床と天井を除くために両端を切る。
# 下限0.3m: 床の点（実測でz≒0に151万点）を障害物にしないため。
#   なお公式の「障害物は高さ50cm以上ないとLiDARが検知しない」は**実機の走行時**の
#   話であって、地図に写っている壁を無視してよいという意味ではない。
#   地図側はより低い0.3mから拾い、安全側に倒す。
# 上限1.8m: 天井（実測2.52m）と、頭上を通過できる張り出しを除くため。
DEFAULT_OBSTACLE_Z_MIN = 0.30
DEFAULT_OBSTACLE_Z_MAX = 1.80

# 障害物を膨らませる半径[m]。G1の肩幅は約0.45mなので機体半径は約0.25m。
# 定位誤差と歩容の揺れの分を足して0.40mにしている。
# これにより経路判定は「点が通れるか」で書けて、機体の幅を毎回考えなくてよくなる。
DEFAULT_INFLATION_M = 0.40

_PCD_HEADER_END = b"DATA binary\n"
_MAX_HEADER_BYTES = 4096


@dataclass(frozen=True)
class GridSpec:
    """格子の張り方。原点は格子(0,0)セルの左下隅のworld座標。"""

    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return col, row

    def to_world(self, col: int, row: int) -> tuple[float, float]:
        """セル中心のworld座標。"""

        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def contains(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height


class OccupancyGrid:
    """通行判定の格子。True=通れない（膨張済み）。

    「通れない」には2種類が入る。障害物そのものと、**一度も観測されていない
    外側**。地図に無い場所へ歩かせるのは、そこに何も無いという保証が
    無いから。格子の範囲外も同じ理由で通行不可とする。
    """

    def __init__(self, spec: GridSpec, occupied: bytearray) -> None:
        if len(occupied) != spec.width * spec.height:
            raise ValueError(
                f"セル数が仕様と合わない: {len(occupied)} != {spec.width * spec.height}"
            )
        self.spec = spec
        self._occupied = occupied

    @property
    def occupied_count(self) -> int:
        return sum(self._occupied)

    def is_occupied_cell(self, col: int, row: int) -> bool:
        if not self.spec.contains(col, row):
            return True  # 地図の外＝未知＝通れない
        return bool(self._occupied[row * self.spec.width + col])

    def is_free(self, x: float, y: float) -> bool:
        col, row = self.spec.to_cell(x, y)
        return not self.is_occupied_cell(col, row)

    def is_segment_free(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """線分 (x0,y0)-(x1,y1) 上に障害物が無いか。

        解像度の半分ずつ標本化する。障害物は必ず `DEFAULT_INFLATION_M`(=0.4m)
        で膨張させてあり、最小でも半径4セル分の厚みを持つので、
        0.05m刻みの標本が壁をすり抜けることはない。
        """

        length = math.hypot(x1 - x0, y1 - y0)
        step = self.spec.resolution * 0.5
        steps = max(1, int(math.ceil(length / step)))
        for index in range(steps + 1):
            ratio = index / steps
            if not self.is_free(x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio):
                return False
        return True

    def with_extra_obstacle(self, x: float, y: float, radius: float) -> "OccupancyGrid":
        """一時的な障害物を足した**新しい**格子を返す。元の格子は変えない。

        巡回中に人や荷物で塞がれたときの迂回に使う。`slam_operate` は
        障害物の位置を教えてくれない（`obsInfo` は有無と経過秒だけ）ので、
        呼び側が「機体の前方どこか」と推定した円をここに置く。

        radius は**機体中心が入ってはいけない半径**。この格子は既に
        `DEFAULT_INFLATION_M` で膨張済みなので、障害物の実寸に機体半径を
        足した値を渡すこと。
        """

        painted = bytearray(self._occupied)
        cells = int(math.ceil(radius / self.spec.resolution))
        center_col, center_row = self.spec.to_cell(x, y)
        for drow in range(-cells, cells + 1):
            for dcol in range(-cells, cells + 1):
                if math.hypot(dcol, drow) * self.spec.resolution > radius:
                    continue
                col, row = center_col + dcol, center_row + drow
                if self.spec.contains(col, row):
                    painted[row * self.spec.width + col] = 1
        return OccupancyGrid(self.spec, painted)

    def free_regions(self) -> list[list[tuple[int, int]]]:
        """自由セルを4近傍でつないだ塊の一覧。大きい順。

        地図は自由に見えても、機体半径ぶんの膨張で通路が塞がって分断される
        ことがある（`sim_room.pcd` では壁際に32セルの孤立ポケットができた）。
        巡回地点を選ぶ前にここを見ておくと、実機に投げてから詰まらずに済む。
        """

        free = set(self.free_cells())
        regions: list[list[tuple[int, int]]] = []
        seen: set[tuple[int, int]] = set()
        for cell in free:
            if cell in seen:
                continue
            seen.add(cell)
            queue, region = deque([cell]), []
            while queue:
                col, row = queue.popleft()
                region.append((col, row))
                for dcol, drow in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    neighbor = (col + dcol, row + drow)
                    if neighbor in free and neighbor not in seen:
                        seen.add(neighbor)
                        queue.append(neighbor)
            regions.append(region)
        regions.sort(key=len, reverse=True)
        return regions

    def free_cells(self) -> list[tuple[int, int]]:
        return [
            (col, row)
            for row in range(self.spec.height)
            for col in range(self.spec.width)
            if not self._occupied[row * self.spec.width + col]
        ]


def build_grid(
    points: list[tuple[float, float, float]],
    *,
    resolution: float = DEFAULT_RESOLUTION_M,
    z_min: float = DEFAULT_OBSTACLE_Z_MIN,
    z_max: float = DEFAULT_OBSTACLE_Z_MAX,
    inflation: float = DEFAULT_INFLATION_M,
    margin: float = 1.0,
) -> OccupancyGrid:
    """点群から占有格子を作る。

    margin は地図の外周に足す余白[m]。膨張が地図の縁で切れると、
    壁際の通路が実際より広く見えてしまうため。
    """

    if not points:
        raise ValueError("点が1つも無い")
    if resolution <= 0.0:
        raise ValueError(f"解像度は正でなければならない: {resolution}")

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    spec = GridSpec(
        origin_x=min(xs) - margin,
        origin_y=min(ys) - margin,
        resolution=resolution,
        width=int(math.ceil((max(xs) - min(xs) + 2.0 * margin) / resolution)) + 1,
        height=int(math.ceil((max(ys) - min(ys) + 2.0 * margin) / resolution)) + 1,
    )

    observed = bytearray(spec.width * spec.height)
    obstacle = bytearray(spec.width * spec.height)
    for x, y, z in points:
        col, row = spec.to_cell(x, y)
        if not spec.contains(col, row):
            continue
        index = row * spec.width + col
        observed[index] = 1  # 高さに関係なく「見えた場所」として記録する
        if z_min <= z <= z_max:
            obstacle[index] = 1

    blocked = _block_unobserved_outside(spec, observed, obstacle)
    return OccupancyGrid(spec, _inflate(spec, blocked, inflation))


def load_pcd(path: Path) -> list[tuple[float, float, float]]:
    """`FIELDS x y z` / `TYPE F` / `DATA binary` のPCDだけを読む。

    `Navigation/sim/npz_to_pcd.py` と `Mapping/` が書くのはこの形式。
    ASCII PCDや追加フィールド付きのPCDは、黙って誤読するくらいなら弾く。
    """

    blob = path.read_bytes()
    marker = blob.find(_PCD_HEADER_END, 0, _MAX_HEADER_BYTES)
    if marker < 0:
        raise ValueError(f"binary PCDのヘッダが見つからない: {path}")
    header = blob[: marker + len(_PCD_HEADER_END)].decode("ascii", errors="replace")
    _check_pcd_header(header, path)

    body = blob[marker + len(_PCD_HEADER_END) :]
    count = len(body) // 12
    return [
        (float(x), float(y), float(z))
        for x, y, z in struct.iter_unpack("<fff", body[: count * 12])
    ]


def load_grid(path: Path, **kwargs: float) -> OccupancyGrid:
    """PCDを読んでそのまま占有格子にする。"""

    return build_grid(load_pcd(path), **kwargs)  # type: ignore[arg-type]


def _check_pcd_header(header: str, path: Path) -> None:
    fields = _header_value(header, "FIELDS")
    if fields.split() != ["x", "y", "z"]:
        raise ValueError(f"FIELDSがx y zではない({fields!r}): {path}")
    if _header_value(header, "SIZE").split() != ["4", "4", "4"]:
        raise ValueError(f"SIZEが4 4 4ではない: {path}")
    if _header_value(header, "TYPE").split() != ["F", "F", "F"]:
        raise ValueError(f"TYPEがF F Fではない: {path}")


def _header_value(header: str, key: str) -> str:
    for line in header.splitlines():
        if line.startswith(key + " "):
            return line[len(key) + 1 :].strip()
    raise ValueError(f"PCDヘッダに{key}が無い")


def _block_unobserved_outside(
    spec: GridSpec, observed: bytearray, obstacle: bytearray
) -> bytearray:
    """一度も点が返ってこなかった「外側」を通行不可にする。

    これが無いと、外周に足した余白リングが「点が無い＝自由」と判定され、
    **建物の外側を回る経路**が引けてしまう（実データで実際に起きた）。
    未観測は自由ではなく未知であり、未知は通れないものとして扱う。

    ただし部屋の中にできる小さな未観測の穴（LiDARの死角、家具の陰）まで
    塞ぐと通路が消えるので、**外周から4近傍でつながっている未観測だけ**を
    通行不可にする。内部に孤立した穴は自由のままにする。

    床の点は `z_min` で障害物からは外れるが、ここでは「見えた場所」の
    証拠として効く。床が写っていない地図ではこの判定は使えない。
    """

    width, height = spec.width, spec.height
    outside = bytearray(width * height)
    queue: deque[int] = deque()

    for col in range(width):
        for row in (0, height - 1):
            _seed_outside(observed, outside, queue, row * width + col)
    for row in range(height):
        for col in (0, width - 1):
            _seed_outside(observed, outside, queue, row * width + col)

    while queue:
        index = queue.popleft()
        col, row = index % width, index // width
        for dcol, drow in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ncol, nrow = col + dcol, row + drow
            if spec.contains(ncol, nrow):
                _seed_outside(observed, outside, queue, nrow * width + ncol)

    return bytearray(a | b for a, b in zip(obstacle, outside))


def _seed_outside(
    observed: bytearray, outside: bytearray, queue: "deque[int]", index: int
) -> None:
    if observed[index] or outside[index]:
        return
    outside[index] = 1
    queue.append(index)


def _inflate(spec: GridSpec, raw: bytearray, inflation: float) -> bytearray:
    """障害物セルを半径 inflation[m] の円で膨らませる。

    セル数が数千〜数万の規模なので、素直な「占有セルごとに円を塗る」で足りる。
    距離変換にすると速いが、読みにくくなる割に効かない。
    """

    if inflation <= 0.0:
        return raw
    radius = int(math.ceil(inflation / spec.resolution))
    offsets = [
        (dc, dr)
        for dr in range(-radius, radius + 1)
        for dc in range(-radius, radius + 1)
        if math.hypot(dc, dr) * spec.resolution <= inflation
    ]

    inflated = bytearray(raw)
    for row in range(spec.height):
        base = row * spec.width
        for col in range(spec.width):
            if not raw[base + col]:
                continue
            for dc, dr in offsets:
                ncol, nrow = col + dc, row + dr
                if spec.contains(ncol, nrow):
                    inflated[nrow * spec.width + ncol] = 1
    return inflated
