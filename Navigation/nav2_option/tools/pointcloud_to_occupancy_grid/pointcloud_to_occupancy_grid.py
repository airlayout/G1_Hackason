"""点群地図(.pcd/.ply) から Nav2 map_server 形式(.pgm + .yaml)へ変換する。

Planning.md A-7・D-21 に対応。Global costmap 用の 2D 地図を作る
(Local costmap は別途 3D 点群ベースで運用する、D-21)。

未観測領域を「自由」と誤判定すると経路が地図の外側を回る問題が起きうる
(点が1つも無いセルは「未知」であって「自由」ではない)。これを避けるため 3 値で出力する:

    occupied(黒=0)   : 高さフィルタ範囲内に点があるセル
    free(白=254)     : 観測済みで、高さフィルタ範囲内には点が無いセル
    unknown(灰=205)  : 観測されていないセル

## ⚠️ 高さは「床からの相対高さ」で判定する(2026-09-09 修正)

**旧版は `--min-height`/`--max-height` を絶対 Z 座標に対して適用していた。**
これは「床が Z≒0 にある」という暗黙の前提であり、実機データでは成り立たない。
`map_20260907.pcd`(room_a セッションの SLAM 出力)の実測では:

    床   : 絶対 Z ≈ -1.25m (全体の 34%)
    天井 : 絶対 Z ≈ +1.50m (全体の 約18%)

この状態で旧既定値(絶対 Z の 0.3〜1.8m)を適用すると、実際には「床上 1.55〜3.05m」
＝天井付近だけを見ることになり、**机・椅子はほとんど拾えず、代わりに天井を
障害物として大量に誤検出する。** 姉妹実装 `Navigation/nav3/pcd_to_ros_map.py` が
同じバグを踏んで修正済みで、本ツールもそれに合わせた。

以後 `--min-height`/`--max-height` は **床からの高さ[m]** として扱う。床は
`find_floor()` が点群自身の Z ヒストグラムから検出する(`--floor-z` で明示指定も可)。

## ⚠️ 自由空間はレイトレーシングで繋ぐ(2026-09-09 追加)

**旧版は「点があるセル」だけを free にしていた。** 床面の観測は疎なので、これだけでは
自由空間が孤立した小片に分断され、経路計画に使えない(Planning.md A-9 で実測。
実点群由来の地図で最大連結成分が 3m² 未満になり、合成地図に切り替える羽目になった)。

`--trajectory` に mapping 実行時の軌跡(TUM 形式)を渡すと、各姿勢をセンサー原点として
2D の光線を飛ばし、**最初の occupied セルに当たるまでの経路上のセルを free にする**。
ロボットが実際に歩いた場所から見えていた範囲が自由空間として繋がる。

    制約: 壁のセルが観測漏れで欠けていると、そこから光線が室外へ漏れる。
    `--max-range` を実際のセンサー到達距離程度に抑えて影響を限定する。
    (OctoMap の sensor_model.max_range と同じ考え方)

軌跡を渡さない場合は旧来どおり「点があるセル」だけが free になり、連結性の警告を出す。

## 使い方

    # 推奨(床検出 + レイトレーシング + 外れ値の除外)
    python3 pointcloud_to_occupancy_grid.py map_20260907.pcd \\
        --trajectory trajectory.tum --bounds -6 20 -18 14 --out room_a_map

    # 軌跡が無い場合(自由空間は分断される可能性あり)
    python3 pointcloud_to_occupancy_grid.py map.ply --out map

    → <out>.pgm と <out>.yaml を生成する(Nav2 map_server がそのまま読める)。
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:  # 任意依存。無ければ内蔵の PCD リーダーで読む
    import open3d as o3d
except ImportError:  # pragma: no cover - 環境依存
    o3d = None

try:  # 任意依存。連結成分の計測と --fill-interior-unknown に使う
    from scipy import ndimage
except ImportError:  # pragma: no cover - 環境依存
    ndimage = None

FREE = 254
OCCUPIED = 0
UNKNOWN = 205


@dataclass
class GridResult:
    grid: np.ndarray  # shape (height, width), uint8。行0が最も小さいyに対応(書き出し時に反転)
    resolution: float
    origin_x: float
    origin_y: float


# --- 点群の読み込み ---------------------------------------------------------

_PCD_TYPE_MAP = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "<u1", ("U", 2): "<u2",
                 ("U", 4): "<u4", ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4"}


def load_points_pcd(path: Path) -> np.ndarray:
    """x/y/z だけを取り出す最小の PCD リーダー(ascii / binary)。

    open3d が無い環境(実機の PC2 など)でも動かせるようにするための内蔵実装。
    `binary_compressed` は未対応なので、その場合は open3d を使うこと。
    """
    header: dict[str, str] = {}
    with open(path, "rb") as f:
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError(f"PCD ヘッダの終端(DATA 行)が見つからない: {path}")
            line = raw.decode("ascii", errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            header[key.upper()] = value.strip()
            if key.upper() == "DATA":
                break
        data_offset = f.tell()

    fields = header.get("FIELDS", "").split()
    sizes = [int(v) for v in header.get("SIZE", "").split()]
    types = header.get("TYPE", "").split()
    counts = [int(v) for v in header.get("COUNT", "").split()] or [1] * len(fields)
    n_points = int(header.get("POINTS") or header.get("WIDTH", 0))
    data_mode = header.get("DATA", "").lower()

    for axis in ("x", "y", "z"):
        if axis not in fields:
            raise ValueError(f"PCD に {axis} フィールドが無い: {path} (FIELDS={fields})")

    if data_mode == "ascii":
        # np.loadtxt はヘッダ行(数値でない)を読めないため、DATA 以降を自分で読む
        cols = [fields.index(a) for a in ("x", "y", "z")]
        rows = []
        with open(path, "r", encoding="ascii", errors="replace") as f:
            f.seek(data_offset)
            for line in f:
                parts = line.split()
                if len(parts) >= len(fields):
                    rows.append([float(parts[c]) for c in cols])
        return np.asarray(rows, dtype=np.float64).reshape(-1, 3)

    if data_mode != "binary":
        raise ValueError(
            f"未対応の PCD DATA 形式: {data_mode}。open3d を入れるか ascii/binary に変換すること")

    dtype_fields = []
    for name, size, type_char, count in zip(fields, sizes, types, counts):
        np_type = _PCD_TYPE_MAP.get((type_char.upper(), size))
        if np_type is None:
            raise ValueError(f"未対応の PCD フィールド型: {name} TYPE={type_char} SIZE={size}")
        # 同名フィールドの重複や padding(_)も dtype 上は区別が必要なので連番を付ける
        dtype_fields.append((f"{name}_{len(dtype_fields)}", np_type, (count,) if count > 1 else ()))

    structured = np.fromfile(path, dtype=np.dtype(dtype_fields), count=n_points, offset=data_offset)
    idx = {name: i for i, name in enumerate(fields)}
    xyz = np.empty((structured.shape[0], 3), dtype=np.float64)
    for col, axis in enumerate(("x", "y", "z")):
        xyz[:, col] = structured[dtype_fields[idx[axis]][0]].astype(np.float64)
    return xyz


def load_points(path: Path) -> np.ndarray:
    """点群を (N,3) の float64 配列で返す。open3d があればそれを使う。"""
    if o3d is not None:
        pcd = o3d.io.read_point_cloud(str(path))
        if not pcd.is_empty():
            return np.asarray(pcd.points)
        # open3d が空を返しても内蔵リーダーで読める場合があるので落とさず続行する
        print(f"[warn] open3d が空を返した。内蔵リーダーで再試行する: {path}", file=sys.stderr)
    if path.suffix.lower() != ".pcd":
        raise ValueError(
            f"open3d が無い環境では .pcd のみ対応: {path}(open3d を入れれば .ply 等も読める)")
    points = load_points_pcd(path)
    if points.shape[0] == 0:
        raise ValueError(f"点群の読み込みに失敗した、または空だった: {path}")
    return points


def load_trajectory_tum(path: Path) -> np.ndarray:
    """TUM 形式(timestamp tx ty tz qx qy qz qw)から (N,2) の XY を返す。

    mapping 実行時のセンサー姿勢。地図と同じ座標系である前提(同一セッションの出力)。
    """
    xy = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = re.split(r"[\s,]+", line)
            if len(parts) < 4:
                continue
            try:
                xy.append((float(parts[1]), float(parts[2])))
            except ValueError:
                continue
    if not xy:
        raise ValueError(f"軌跡を1件も読めなかった: {path}")
    return np.asarray(xy, dtype=np.float64)


# --- 床の検出 ---------------------------------------------------------------

def find_floor(z: np.ndarray) -> float:
    """Z ヒストグラムの下半分の最頻ビンを床とみなす。

    床は面として広がっているため、天井や什器より圧倒的に点数が多い。
    `Navigation/nav3/pcd_to_ros_map.py` および `Navigation/nav/occupancy.py` と
    同じ考え方だが、nav2_option は独立トラック(Planning.md の管理単位)なので
    import せずに同等の実装を置いている。
    """
    low, high = np.percentile(z, [1.0, 99.0])
    core = z[(z >= low) & (z <= high)]
    if len(core) < 100:
        core = z
    hist, edges = np.histogram(core, bins=80)
    centers = (edges[:-1] + edges[1:]) / 2.0
    middle = (centers[0] + centers[-1]) / 2.0
    lower = centers < middle
    if not np.any(lower):  # 極端に薄い点群でも落とさない
        return float(centers[np.argmax(hist)])
    return float(centers[lower][np.argmax(hist[lower])])


# --- レイトレーシング -------------------------------------------------------

def carve_trajectory(
    occupied: np.ndarray,
    poses_xy: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    radius_m: float,
) -> np.ndarray:
    """ロボットが実際に通った帯を True にして返す(占有判定を上書きするために使う)。

    ## なぜ占有を削ってよいのか

    **ロボットの機体がそこに在ったこと自体が、そこが通行可能である最強の証拠**である。
    それでも占有と判定されるのは、地図側に以下のような偽の障害物が残っているため:

    - 追従者(PC を持った人物)がロボットの直後を歩いており、その胴体(床上 0.3〜1.8m)が
      経路上に焼き付いている(`Navigation/nav3/README.md` が「閾値を上げても消えない
      斜めの筋」として報告した動的物体の残り)
    - 至近距離では機体自身が頭部 LiDAR に写る
      (`Navigation/nav2/g1_nav2.yaml` の `obstacle_min_range: 0.30` も同じ理由)

    実測(2026-09-09、map_20260907.pcd): 削る前は軌跡 3113 姿勢のうち 476 姿勢(15.3%)が
    occupied セルに乗っており、Nav2 の planner が「開始点が障害物内」で計画に失敗する
    状態だった。

    ## 危険側の注意

    軌跡自体がドリフトしていると、本物の障害物を削ってしまう。半径は機体半径
    (G1 の肩幅約 0.45m ＝ 半径約 0.25m)にとどめ、それ以上広げないこと。
    """
    height, width = occupied.shape
    col = np.floor((poses_xy[:, 0] - origin_x) / resolution).astype(np.int64)
    row = np.floor((poses_xy[:, 1] - origin_y) / resolution).astype(np.int64)
    inside = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    carved = np.zeros((height, width), dtype=bool)
    carved[row[inside], col[inside]] = True

    r_cells = int(np.floor(radius_m / resolution))
    if r_cells >= 1:
        offsets = [(dy, dx) for dy in range(-r_cells, r_cells + 1)
                   for dx in range(-r_cells, r_cells + 1)
                   if dy * dy + dx * dx <= r_cells * r_cells]
        base_rows, base_cols = row[inside], col[inside]
        for dy, dx in offsets:
            rr = np.clip(base_rows + dy, 0, height - 1)
            cc = np.clip(base_cols + dx, 0, width - 1)
            carved[rr, cc] = True
    return carved


def raytrace_free(
    occupied: np.ndarray,
    poses_xy: np.ndarray,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    max_range: float,
    n_rays: int,
) -> np.ndarray:
    """各姿勢から光線を飛ばし、最初の occupied に当たるまでのセルを True にして返す。

    実装は「角度 × 距離」の等間隔サンプリング(Bresenham と同じ結果を、
    numpy で一括処理できる形にしたもの)。距離方向に occupied の累積和を取り、
    累積が 0 の区間＝最初の衝突より手前だけを可視とする。
    衝突したセル自身は可視にしない(占有のまま残す)。
    """
    height, width = occupied.shape
    visible = np.zeros_like(occupied, dtype=bool)

    # 同じセルに入る姿勢は光線もほぼ同じなので、セル単位で間引いて計算量を落とす
    pose_col = np.floor((poses_xy[:, 0] - origin_x) / resolution).astype(np.int64)
    pose_row = np.floor((poses_xy[:, 1] - origin_y) / resolution).astype(np.int64)
    inside = (pose_col >= 0) & (pose_col < width) & (pose_row >= 0) & (pose_row < height)
    if not np.any(inside):
        raise ValueError("軌跡が地図の範囲内に1点もない(--bounds と軌跡の座標系を確認すること)")
    unique_cells = np.unique(np.stack([pose_row[inside], pose_col[inside]], axis=1), axis=0)

    # セル中心をセンサー原点として使う
    origins_x = origin_x + (unique_cells[:, 1] + 0.5) * resolution
    origins_y = origin_y + (unique_cells[:, 0] + 0.5) * resolution

    angles = np.linspace(0.0, 2.0 * np.pi, n_rays, endpoint=False)
    cos_a = np.cos(angles)[:, None]
    sin_a = np.sin(angles)[:, None]
    # 刻みはセル半分。これより粗いとセルを飛び越えて壁を貫通する
    step = resolution * 0.5
    radii = np.arange(step, max_range + step, step)[None, :]

    dx = cos_a * radii
    dy = sin_a * radii

    for ox, oy in zip(origins_x, origins_y):
        col = np.floor((ox + dx - origin_x) / resolution).astype(np.int64)
        row = np.floor((oy + dy - origin_y) / resolution).astype(np.int64)
        out = (col < 0) | (col >= width) | (row < 0) | (row >= height)
        col_c = np.clip(col, 0, width - 1)
        row_c = np.clip(row, 0, height - 1)
        # 地図外も「そこで打ち切る」扱いにする(外へ抜けた先を自由と主張しない)
        blocking = occupied[row_c, col_c] | out
        before_hit = np.cumsum(blocking, axis=1) == 0
        sel = before_hit & ~out
        visible[row_c[sel], col_c[sel]] = True

    # センサー原点のセル自身も自由(ロボットがそこに居た)
    visible[unique_cells[:, 0], unique_cells[:, 1]] = True
    visible &= ~occupied
    return visible


# --- グリッド生成 -----------------------------------------------------------

def build_occupancy_grid(
    points: np.ndarray,
    *,
    resolution: float,
    min_height: float,
    max_height: float,
    padding_m: float,
    bounds: "tuple[float, float, float, float] | None" = None,
    floor_z: "float | None" = None,
    occupied_min_points: int = 1,
    trajectory_xy: "np.ndarray | None" = None,
    max_range: float = 15.0,
    n_rays: int = 720,
    carve_radius: "float | None" = 0.25,
    fill_interior_unknown: bool = False,
) -> GridResult:
    if points.shape[0] == 0:
        raise ValueError("点群が空")

    if bounds is not None:
        min_x, max_x, min_y, max_y = bounds
    else:
        min_x, min_y = points[:, :2].min(axis=0) - padding_m
        max_x, max_y = points[:, :2].max(axis=0) + padding_m
        extent = max(max_x - min_x, max_y - min_y)
        if extent > 40.0:
            print(f"[warn] 点群の広がりが {extent:.1f}m と非常に大きい。外れ値が混じっている"
                  "可能性が高い。--bounds で範囲を絞ることを推奨", file=sys.stderr)

    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    if width <= 0 or height <= 0:
        raise ValueError("グリッドサイズが不正(点群の広がりを確認すること)")

    in_bounds = ((points[:, 0] >= min_x) & (points[:, 0] <= max_x) &
                 (points[:, 1] >= min_y) & (points[:, 1] <= max_y))
    pts = points[in_bounds]
    if pts.shape[0] == 0:
        raise ValueError("指定範囲に点が1つも無い(--bounds を確認すること)")
    print(f"[grid] {width} x {height} セル ({resolution} m/cell) / "
          f"範囲 x[{min_x:.2f},{max_x:.2f}] y[{min_y:.2f},{max_y:.2f}]")
    print(f"[grid] 範囲内の点: {pts.shape[0]}/{points.shape[0]}")

    xy = pts[:, :2]
    z = pts[:, 2]

    if floor_z is None:
        floor_z = find_floor(z)
        print(f"[grid] 床 Z(絶対座標)={floor_z:+.3f}m と推定")
    else:
        print(f"[grid] 床 Z(絶対座標)={floor_z:+.3f}m を指定値として使用")
    print(f"[grid] 障害物とみなす高さ: 床上 {min_height}〜{max_height}m (相対高さ)")

    def to_index(pts_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ix = np.floor((pts_xy[:, 0] - min_x) / resolution).astype(np.int64)
        iy = np.floor((pts_xy[:, 1] - min_y) / resolution).astype(np.int64)
        np.clip(ix, 0, width - 1, out=ix)
        np.clip(iy, 0, height - 1, out=iy)
        return ix, iy

    observed = np.zeros((height, width), dtype=bool)
    ix_all, iy_all = to_index(xy)
    observed[iy_all, ix_all] = True

    height_above_floor = z - floor_z
    height_mask = (height_above_floor >= min_height) & (height_above_floor <= max_height)
    # セルごとに点数を数え、閾値以上のセルだけを占有にする(1点のノイズで塞がらないように)
    obstacle_count = np.zeros((height, width), dtype=np.int32)
    if np.any(height_mask):
        ix_occ, iy_occ = to_index(xy[height_mask])
        np.add.at(obstacle_count, (iy_occ, ix_occ), 1)
    occ_mask = obstacle_count >= occupied_min_points
    print(f"[grid] 高さ帯内の点: {int(np.count_nonzero(height_mask))} "
          f"(occupied 判定の下限 {occupied_min_points} 点/セル)")

    # 軌跡に沿った偽の障害物(追従者・機体の自己観測)を先に削る。
    # レイトレーシングより前に行うこと。偽の壁が残っていると光線がそこで止まる
    carved = None
    if trajectory_xy is not None and carve_radius is not None:
        carved = carve_trajectory(
            occ_mask, trajectory_xy, origin_x=min_x, origin_y=min_y,
            resolution=resolution, radius_m=carve_radius,
        )
        n_carved = int(np.count_nonzero(occ_mask & carved))
        occ_mask &= ~carved
        print(f"[grid] 軌跡に沿って占有を削った: {n_carved} セル "
              f"(機体半径 {carve_radius}m 相当)")

    free_mask = observed & ~occ_mask
    if carved is not None:
        free_mask |= carved  # 機体が在った場所は観測点の有無に関わらず自由
    if trajectory_xy is not None:
        traced = raytrace_free(
            occ_mask, trajectory_xy, origin_x=min_x, origin_y=min_y,
            resolution=resolution, max_range=max_range, n_rays=n_rays,
        )
        n_added = int(np.count_nonzero(traced & ~free_mask))
        free_mask |= traced
        print(f"[grid] レイトレーシング: {n_rays}方向 / 最大 {max_range}m / "
              f"free セルを {n_added} 増やした")
    else:
        print("[warn] --trajectory が無いので「点があるセル」だけを free にする。"
              "自由空間が分断され経路計画に使えない場合がある", file=sys.stderr)

    unknown = ~(observed | free_mask)
    if fill_interior_unknown:
        if ndimage is None:
            raise RuntimeError("--fill-interior-unknown には scipy が必要")
        # 外周と繋がっている未観測だけを未知として残し、内部の孤立した死角は自由に倒す
        unknown = ~ndimage.binary_fill_holes(observed | free_mask)

    grid = np.full((height, width), UNKNOWN, dtype=np.uint8)
    grid[free_mask] = FREE
    grid[occ_mask] = OCCUPIED  # occupied が free を上書きする
    grid[unknown & ~occ_mask] = UNKNOWN

    return GridResult(grid=grid, resolution=resolution, origin_x=min_x, origin_y=min_y)


def report_connectivity(
    grid: np.ndarray,
    result: GridResult,
    trajectory_xy: "np.ndarray | None" = None,
) -> None:
    """自由空間の連結性を測る。経路計画に使えるかの判断材料(Planning.md A-9 の教訓)。

    軌跡を渡した場合は「ロボットが実際に歩いた姿勢が、ひとつの連結した自由空間に
    収まっているか」も測る。**こちらが実質的な合否判定**で、Nav2 の planner は
    開始点・目標点が占有セルだと計画自体を拒否する(`Navigation/nav3/README.md` が
    MuJoCo 検証で踏んだ失敗)。
    """
    if ndimage is None:
        print("[info] scipy が無いので連結性の計測は省略した")
        return
    free = grid == FREE
    labels, n = ndimage.label(free)
    if n == 0:
        print("[warn] free セルが1つも無い", file=sys.stderr)
        return
    sizes = np.bincount(labels.ravel())[1:]
    largest_label = int(np.argmax(sizes)) + 1
    largest = int(sizes.max())
    cell_area = result.resolution * result.resolution
    print(f"[conn] free の連結成分: {n} 個 / 最大 {largest} セル "
          f"({largest * cell_area:.1f} m², free 全体の {100*largest/int(free.sum()):.1f}%)")
    if largest * cell_area < 10.0:
        print("[warn] 最大連結成分が 10m² 未満。経路計画には使えない見込み。"
              "--trajectory の指定と高さ帯を見直すこと", file=sys.stderr)

    if trajectory_xy is None:
        return
    height, width = grid.shape
    col = np.floor((trajectory_xy[:, 0] - result.origin_x) / result.resolution).astype(np.int64)
    row = np.floor((trajectory_xy[:, 1] - result.origin_y) / result.resolution).astype(np.int64)
    inside = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    if not np.any(inside):
        print("[warn] 軌跡が地図の範囲外(座標系を確認すること)", file=sys.stderr)
        return
    at = labels[row[inside], col[inside]]
    total = int(inside.sum())
    in_largest = int((at == largest_label).sum())
    on_occupied = int((grid[row[inside], col[inside]] == OCCUPIED).sum())
    print(f"[conn] 軌跡 {total} 姿勢中: 最大連結成分の上 {in_largest} "
          f"({100*in_largest/total:.1f}%) / occupied の上 {on_occupied} "
          f"({100*on_occupied/total:.1f}%)")
    if on_occupied > 0:
        print(f"[warn] ロボットが実際に立っていた {on_occupied} 姿勢が occupied 判定になっている。"
              "Nav2 の planner が「開始点が障害物内」で失敗する原因になる", file=sys.stderr)


def write_pgm(path: Path, grid: np.ndarray) -> None:
    """P5(binary)形式で書き出す。map_server慣例(原点=左下)に合わせ、行を上下反転して保存する。"""
    height, width = grid.shape
    flipped = np.flipud(grid)  # 画像は上が大きいy、mapのyamlはorigin=左下を仮定するため反転
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(flipped.tobytes())


def write_yaml(path: Path, pgm_name: str, result: GridResult) -> None:
    content = (
        f"image: {pgm_name}\n"
        f"resolution: {result.resolution}\n"
        f"origin: [{result.origin_x}, {result.origin_y}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    path.write_text(content, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", type=Path, help="入力点群ファイル(.pcd。open3dがあれば.ply等も)")
    parser.add_argument("--out", type=Path, default=Path("map"), help="出力ファイル名の接頭辞(既定: map)")
    parser.add_argument("--resolution", type=float, default=0.05, help="グリッド解像度[m/cell](既定: 0.05)")
    parser.add_argument("--min-height", type=float, default=0.3,
                        help="障害物とみなす最低高さ[m]。**床からの相対高さ**(既定: 0.3、D-21)")
    parser.add_argument("--max-height", type=float, default=1.8,
                        help="障害物とみなす最高高さ[m]。**床からの相対高さ**(既定: 1.8、D-21)")
    parser.add_argument("--floor-z", type=float, default=None,
                        help="床の絶対Z[m]を明示指定する(既定: 点群から自動検出)")
    parser.add_argument("--padding", type=float, default=1.0, help="点群の外周に足す余白[m](既定: 1.0)")
    parser.add_argument("--bounds", type=float, nargs=4, default=None,
                        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"),
                        help="地図の範囲を手動指定する。外れ値混入時に地図が巨大化するのを防ぐ")
    parser.add_argument("--occupied-min-points", type=int, default=1,
                        help="高さ帯内の点がこの数以上あるセルだけを占有とみなす(既定: 1)")
    parser.add_argument("--trajectory", type=Path, default=None,
                        help="mapping 実行時の軌跡(TUM形式)。渡すとレイトレーシングで自由空間を繋ぐ")
    parser.add_argument("--max-range", type=float, default=15.0,
                        help="レイトレーシングの最大距離[m](既定: 15.0)")
    parser.add_argument("--rays", type=int, default=720,
                        help="1姿勢あたりの光線本数(既定: 720 = 0.5度刻み)")
    parser.add_argument("--carve-radius", type=float, default=0.25,
                        help="軌跡に沿って占有を削る半径[m](既定: 0.25 = G1の機体半径)。"
                             "追従者・機体の自己観測による偽の障害物を除く")
    parser.add_argument("--no-carve", action="store_true",
                        help="軌跡に沿った占有の削除を行わない(生の判定結果を見たいとき)")
    parser.add_argument("--fill-interior-unknown", action="store_true",
                        help="内部の孤立した未観測(死角)も自由として塗る(既定: 未知のまま残す)")
    args = parser.parse_args()

    # 0以下だと全セルが occupied になり、無言で使えない地図ができてしまう
    if args.occupied_min_points < 1:
        parser.error("--occupied-min-points は 1 以上でなければならない")
    if args.carve_radius < 0:
        parser.error("--carve-radius は 0 以上でなければならない(削らないなら --no-carve)")

    points = load_points(args.input)
    print(f"[info] 読み込んだ点数: {points.shape[0]}")
    print(f"[info] z範囲: {points[:, 2].min():.3f} .. {points[:, 2].max():.3f}")

    trajectory_xy = None
    if args.trajectory is not None:
        trajectory_xy = load_trajectory_tum(args.trajectory)
        print(f"[info] 軌跡: {trajectory_xy.shape[0]} 姿勢 "
              f"(x[{trajectory_xy[:,0].min():.2f},{trajectory_xy[:,0].max():.2f}] "
              f"y[{trajectory_xy[:,1].min():.2f},{trajectory_xy[:,1].max():.2f}])")

    result = build_occupancy_grid(
        points,
        resolution=args.resolution,
        min_height=args.min_height,
        max_height=args.max_height,
        padding_m=args.padding,
        bounds=tuple(args.bounds) if args.bounds is not None else None,
        floor_z=args.floor_z,
        occupied_min_points=args.occupied_min_points,
        trajectory_xy=trajectory_xy,
        max_range=args.max_range,
        n_rays=args.rays,
        carve_radius=None if args.no_carve else args.carve_radius,
        fill_interior_unknown=args.fill_interior_unknown,
    )

    pgm_path = args.out.with_suffix(".pgm")
    yaml_path = args.out.with_suffix(".yaml")
    write_pgm(pgm_path, result.grid)
    write_yaml(yaml_path, pgm_path.name, result)

    n_occ = int(np.sum(result.grid == OCCUPIED))
    n_free = int(np.sum(result.grid == FREE))
    n_unknown = int(np.sum(result.grid == UNKNOWN))
    total = result.grid.size
    print(f"[info] グリッドサイズ: {result.grid.shape[1]} x {result.grid.shape[0]} "
          f"({result.resolution} m/cell)")
    print(f"[info] occupied={n_occ} ({100*n_occ/total:.1f}%) "
          f"free={n_free} ({100*n_free/total:.1f}%) "
          f"unknown={n_unknown} ({100*n_unknown/total:.1f}%)")
    report_connectivity(result.grid, result, trajectory_xy)
    print(f"[info] 書き出し: {pgm_path}, {yaml_path}")


if __name__ == "__main__":
    main()
