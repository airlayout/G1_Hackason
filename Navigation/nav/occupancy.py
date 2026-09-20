"""地図の点群から2Dの通行判定格子を作る。

**計算は scipy に任せる。** 自作するのは「G1 のナビ固有の決め事」だけ:
どの高さを障害物とみなすか、未観測をどう扱うか、機体半径をいくつにするか。

なぜ必要か: `slam_operate` の 1102 は**目標点まで直線で歩き、障害物に遭遇したら止まる**
（`mode` は `1` 固定で绕障モードが無い）。壁を貫く指令を投げると必ず詰まるので、
「その直線を歩けるか」を投げる前に地図で判定する。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy import ndimage
from skimage.draw import line_aa as raster_line

# 格子の解像度[m]。足の置き場ではなく「その直線が壁を貫くか」を見る粒度。
DEFAULT_RESOLUTION_M = 0.10

# 障害物とみなす高さの範囲[m]。**床からの相対高さ**（2026-09-08修正。以下参照）。
# 下限 0.30: 床の点を障害物にしないため。公式の「障害物は高さ50cm以上ないとLiDARが
#   検知しない」は**走行時**の話で、地図に写る壁は安全側に 0.3m から拾う。
# 上限 1.80: 天井と、頭上を通過できる張り出しを除くため。
#
# ⚠️ 2026-09-08まで絶対座標のZとして扱っていた（座標系の原点はMid360-IMU、
# 公式ドキュメントの想定では床がZ≈0という前提）。しかしMapping班のSLAM出力
# （`map_20260907.pcd`）では実測で床が絶対Z≈-1.30mにあり、この前提が崩れていた。
# 絶対Zのまま適用すると「床から1.6〜3.1m」（天井を突き抜けた範囲）を見てしまい、
# 天井の点を障害物として大量に誤検出する
# （`Navigation/nav3/README.md`「重大なバグ発覚」節で先に発見・修正済み）。
# `build_grid()`は点群自体から`find_floor()`で床を検出し、そこからの相対高さで
# 判定するよう変更した。床がZ≈0の点群（`sim/rooms.py`由来の合成点群等）では
# 挙動は変わらない。
DEFAULT_OBSTACLE_Z_MIN = 0.30
DEFAULT_OBSTACLE_Z_MAX = 1.80


def find_floor(z: np.ndarray) -> float:
    """Zヒストグラムの下半分の最頻ビンを床とみなす。

    床は面として広がっているため、天井や什器より圧倒的に点数が多い
    （`nav3/pcd_to_ros_map.py`と同じ考え方。詳細はそちらのdocstring参照）。
    """

    low, high = np.percentile(z, [1.0, 99.0])
    core = z[(z >= low) & (z <= high)]
    if len(core) < 100:
        core = z
    hist, edges = np.histogram(core, bins=80)
    centers = (edges[:-1] + edges[1:]) / 2.0
    middle = (centers[0] + centers[-1]) / 2.0
    lower = centers < middle
    return float(centers[lower][np.argmax(hist[lower])])

# 障害物を膨らませる半径[m]。G1 の肩幅は約 0.45m ＝ 機体半径 約 0.25m。
# 定位誤差と歩容の揺れを足して 0.40m。これで経路判定を「点が通れるか」で書ける。
DEFAULT_INFLATION_M = 0.40

# 地図の外周に足す余白[m]。膨張が縁で切れると壁際の通路が実際より広く見える。
DEFAULT_MARGIN_M = 1.0


@dataclass(frozen=True)
class GridSpec:
    """格子の張り方。原点は (0,0) セルの左下隅の world 座標。"""

    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int

    def to_cell(self, x, y):
        """world 座標 -> (col, row)。配列でもスカラーでも受ける。"""

        col = np.floor((np.asarray(x) - self.origin_x) / self.resolution).astype(int)
        row = np.floor((np.asarray(y) - self.origin_y) / self.resolution).astype(int)
        return col, row

    def to_world(self, col, row):
        """(col, row) -> セル中心の world 座標。"""

        return (
            self.origin_x + (np.asarray(col) + 0.5) * self.resolution,
            self.origin_y + (np.asarray(row) + 0.5) * self.resolution,
        )

    def contains(self, col, row) -> bool:
        return bool(0 <= col < self.width and 0 <= row < self.height)


class OccupancyGrid:
    """通行判定の格子。True = 通れない（膨張済み）。

    「通れない」には障害物そのものと、**一度も観測されていない外側**の両方が入る。
    地図に無い場所へ歩かせるのは、そこに何も無いという保証が無いから。
    格子の範囲外も同じ理由で通行不可とする。
    """

    def __init__(self, spec: GridSpec, blocked: np.ndarray) -> None:
        if blocked.shape != (spec.height, spec.width):
            raise ValueError(f"格子の形が仕様と合わない: {blocked.shape} != {(spec.height, spec.width)}")
        self.spec = spec
        self.blocked = blocked.astype(bool)

    @property
    def free_count(self) -> int:
        return int((~self.blocked).sum())

    def is_free(self, x: float, y: float) -> bool:
        col, row = self.spec.to_cell(x, y)
        if not self.spec.contains(col, row):
            return False  # 地図の外＝未知＝通れない
        return not bool(self.blocked[row, col])

    def is_segment_free(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """線分が通れるか。`skimage.draw.line_aa` が通過セルを列挙する。

        `line`（Bresenham）ではなく `line_aa` を使うのは、**線がかすめたセルまで含める**
        ため。Bresenham は 1 行につき 1 セルしか取らないので、斜めの線が壁の角を
        すり抜けて「通れる」と誤判定する（旧実装との差分テストで実際に 3 件出た）。
        判定は安全側に倒す。
        """

        c0, r0 = self.spec.to_cell(x0, y0)
        c1, r1 = self.spec.to_cell(x1, y1)
        for c, r in ((c0, r0), (c1, r1)):
            if not self.spec.contains(c, r):
                return False
        rows, cols, _ = raster_line(int(r0), int(c0), int(r1), int(c1))
        return not bool(self.blocked[rows, cols].any())

    def free_regions(self) -> list[np.ndarray]:
        """自由セルを4近傍でつないだ塊の一覧（大きい順）。各要素は (N,2) の [col,row]。

        地図は自由に見えても、機体半径ぶんの膨張で通路が塞がって分断されることがある。
        巡回地点を選ぶ前に見ておくと、実機に投げてから詰まらずに済む。
        """

        labels, count = ndimage.label(~self.blocked)
        regions = []
        for index in range(1, count + 1):
            rows, cols = np.nonzero(labels == index)
            regions.append(np.stack([cols, rows], axis=1))
        regions.sort(key=len, reverse=True)
        return regions

    def with_extra_obstacle(self, x: float, y: float, radius: float) -> "OccupancyGrid":
        """一時的な障害物を足した**新しい**格子を返す。元は変えない。

        巡回中に人や荷物で塞がれたときの迂回に使う。`slam_operate` は障害物の位置を
        教えてくれない（`obsInfo` は有無と経過秒だけ）ので、呼び側が推定した円を置く。
        radius は**機体中心が入ってはいけない半径**。
        """

        return OccupancyGrid(self.spec, self.blocked | self._disk(x, y, radius))

    def with_extra_blocked(self, mask: np.ndarray) -> "OccupancyGrid":
        """真偽の面を重ねた**新しい**格子を返す。元は変えない。

        `with_extra_obstacle` が円 1 つなのに対し、こちらは呼び側が組んだ面を
        そのまま重ねる。`nav/mission.py` が「迂回の憶測をまとめた層」を作り、
        そこにだけ穴を空けてから重ねるのに使う
        （実測の壁に穴を空けないための分離）。
        """

        if mask.shape != self.blocked.shape:
            raise ValueError(f"面の形が格子と合わない: {mask.shape} != {self.blocked.shape}")
        return OccupancyGrid(self.spec, self.blocked | mask)

    def disk(self, x: float, y: float, radius: float) -> np.ndarray:
        """中心 (x,y) 半径 radius[m] の円の中を True にした面。格子と同じ形。"""

        return self._disk(x, y, radius)

    def _disk(self, x: float, y: float, radius: float) -> np.ndarray:
        col, row = self.spec.to_cell(x, y)
        rows, cols = np.ogrid[: self.spec.height, : self.spec.width]
        return ((cols - col) ** 2 + (rows - row) ** 2) * self.spec.resolution**2 <= radius**2


def build_grid(
    points: np.ndarray,
    *,
    resolution: float = DEFAULT_RESOLUTION_M,
    z_min: float = DEFAULT_OBSTACLE_Z_MIN,
    z_max: float = DEFAULT_OBSTACLE_Z_MAX,
    inflation: float = DEFAULT_INFLATION_M,
    margin: float = DEFAULT_MARGIN_M,
) -> OccupancyGrid:
    """点群 (N,3) から通行判定格子を作る。"""

    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(points) == 0:
        raise ValueError("点が1つも無い")
    if resolution <= 0.0:
        raise ValueError(f"解像度は正でなければならない: {resolution}")

    low = points[:, :2].min(axis=0) - margin
    high = points[:, :2].max(axis=0) + margin
    spec = GridSpec(
        origin_x=float(low[0]),
        origin_y=float(low[1]),
        resolution=resolution,
        width=int(np.ceil((high[0] - low[0]) / resolution)) + 1,
        height=int(np.ceil((high[1] - low[1]) / resolution)) + 1,
    )
    col, row = spec.to_cell(points[:, 0], points[:, 1])
    inside = (col >= 0) & (col < spec.width) & (row >= 0) & (row < spec.height)

    # 高さに関係なく点があったセル＝「観測できた場所」。床の点がここで効く。
    observed = np.zeros((spec.height, spec.width), bool)
    observed[row[inside], col[inside]] = True

    # z_min/z_maxは床からの相対高さ。点群自体から床を検出する
    # （DEFAULT_OBSTACLE_Z_MIN/MAXのコメント参照）。
    floor_z = find_floor(points[:, 2])
    height_above_floor = points[:, 2] - floor_z
    is_obstacle = inside & (height_above_floor >= z_min) & (height_above_floor <= z_max)
    obstacle = np.zeros((spec.height, spec.width), bool)
    obstacle[row[is_obstacle], col[is_obstacle]] = True

    # 外周とつながる未観測を通行不可にする。内部の穴（死角）は自由のまま残す。
    # これが無いと、外周の余白を通って**建物の外側を回る経路**が引ける。
    outside = ~ndimage.binary_fill_holes(observed)
    blocked = obstacle | outside

    if inflation > 0.0:
        # 距離変換で真円に膨らませる（自前で円を塗るより速く正確）
        distance = ndimage.distance_transform_edt(~blocked, sampling=resolution)
        blocked = distance <= inflation
    return OccupancyGrid(spec, blocked)


def load_points(path: Path) -> np.ndarray:
    """点群ファイルを (N,3) で読む。open3d が対応する形式ならなんでもよい。

    **open3d はここでだけ import する**（依存グループ `pcd`）。約 400MB あり、
    格子を作るのに要るのは numpy 配列だけなので、`sim/rooms.py` のように
    点群を自分で組む経路では入っていなくても動くようにしておく。

    `.npy`（`np.save`で保存した (N,3) 配列）は open3d 無しで読む。
    `Navigation/nav3/.venv_amcl`（rclpy用の別venv）には open3d を
    足していないため、そちらから実地図を使うにはこの経路が要る
    （前処理で範囲を絞った点群を`.npy`で渡す運用を想定）。
    """

    path = Path(path)
    if path.suffix == ".npy":
        points = np.load(path)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f".npyの形が(N,3)ではない: {points.shape} ({path})")
        return points.astype(float)

    import open3d as o3d

    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points)
    if len(points) == 0:
        raise ValueError(f"点群を読めなかった（形式が非対応か空）: {path}")
    return points


def load_grid(path: Path, **kwargs) -> OccupancyGrid:
    """点群ファイルを読んでそのまま通行判定格子にする。"""

    return build_grid(load_points(path), **kwargs)
