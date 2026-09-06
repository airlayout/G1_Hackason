#!/usr/bin/env python3
"""点群を MuJoCo のシーンにする。(A) 2.5D占有格子→箱 ＋ (C) 床のハイトフィールド。

## なぜ点群をそのまま入れられないのか

MuJoCo の当たり判定に使える型は plane / hfield / mesh / プリミティブ / sdf だけで、
点群という型が無い。必ず幾何へ変換する工程が要る。

メッシュ化（Poisson 等）は見た目は良いが、**MuJoCo は mesh の当たり判定を凸包で行う**。
部屋のメッシュを入れると凸包＝直方体の塊になり、ロボットが部屋に入れなくなる。
凸分解すれば回避できるが、部屋規模では破片が数千個になり箱に対する利点が消える。

そこで2つを併用する。

- **(A) 2.5D占有格子 → 箱**: 高さ帯を格子に落とし、セルごとの最大高さまで箱を立てる。
  箱は MuJoCo で最も安定・軽い当たり判定で、机は「机の高さの箱」になる（床から天井まで
  伸びた壁にはならない）。**散らばった人の残骸は連結成分の大きさで落とせる**。
- **(C) 床のハイトフィールド**: 床面の高さだけを別に持つ。段差・傾斜・敷居が残る。
  hfield は各 (x,y) に高さを1つしか持てない（机の下をくぐれない）ので、単独では使えない。

MuJoCo では world に属する geom 同士は衝突しないので、hfield と箱が重なっても問題ない。

## 実測で確定させた2点（2026-09-05）

- **hfield PNG は 行0が +Y 側・列0が −X 側**（`mj_ray` で確認）。配列を `flipud` して
  から書かないと地図が南北反転する。
- **シーンは `g1_12dof.xml` と同じディレクトリに置く。** `meshdir="meshes/"` は最上位
  ファイルの位置から解決されるため、別ディレクトリから include すると
  `meshes/assets/g1_description/*.STL` を探して失敗する。既存の `_scene_*.xml` が
  あの場所に居るのはこれが理由。

## 出力

    Navigation/sim/assets/g1_description/_scene_<name>_<変種>.xml   MuJoCo シーン
    Navigation/sim/assets/g1_description/_hfield_<name>_<変種>.png  床の高さ
    runs/<session>/sim/<変種>.npz                                    格子（HTML表示用）
    runs/<session>/sim/scenes.json                                   変種ごとの指標

## 使い方

    ../../Navigation/.venv/bin/python quickstart/pcd_to_mjcf.py \\
        runs/20260904T183457_UiS_room_v2 \\
        map_octomap_r2.pcd map_octomap_r3.pcd map_octomap_r4.pcd map_octomap_r5.pcd
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree

QUICKSTART = Path(__file__).resolve().parent
sys.path.insert(0, str(QUICKSTART))

from eval_removal import read_trajectory  # noqa: E402



# 既定値。CELL は既存の _scene_uis_main_floor.xml と同じ 0.10m に合わせてある
CELL = 0.10
OBSTACLE_BAND = (0.15, 2.00)   # 床上のこの範囲を障害物とみなす（天井は入れない）
FLOOR_BAND = (-0.25, 0.35)     # 床面の高さを測る帯
HEIGHT_STEP = 0.25             # 箱の高さの量子化。粗いほど箱が横に長く繋がって軽くなる
MIN_POINTS = 3                 # セルを占有とみなす最低点数
MIN_CLUSTER = 4                # 連結成分がこれ未満なら捨てる（人の残骸・遠方の外れ値）
FLOOR_PERCENTILE = 20.0        # 床高さに使う分位。足元や置物に引っ張られないよう低め
PHANTOM_RADIUS = 0.5           # 軌跡からこれ以内の占有セルは「偽の障害物」
MAX_RANGE = 12.0               # 軌跡からこれ以上離れたセルは捨てる（0で無効）
FILL_RADIUS = 1.5              # 床の高さを最近傍で埋めてよい距離[m]。以遠は中央値で平らに
FLOOR_TOLERANCE = 0.08         # 全体の床面からこれ以上ずれた床セルは捨てる[m]
FLOOR_MIN_REGION = 100         # ただし塊がこのセル数以上なら本物の段差として残す
FLOOR_SMOOTH = 0.35            # 床を均す幅[m]。0で無効
FAR_RADIUS = 4.0               # 軌跡からこれ以上離れたセルは追従者が居られない＝構造
TALL_LEVEL = 1.75              # これ以上の高さの箱は壁・什器とみなす
SPAWN_CLEARANCE = 0.45         # G1 を置くのに要る自由空間の半径[m]
SPAWN_HEIGHT = 0.793           # 既存シーンの keyframe と同じ
SPAWN_JOINTS = "-0.1 0 0 0.3 -0.2 0 -0.1 0 0 0.3 -0.2 0"
ROBOT_INCLUDE = "g1_12dof.xml"


# ── 座標系の正規化 ────────────────────────────────────────────
def rotation_to_z(normal: np.ndarray) -> np.ndarray:
    """法線を +z に向ける回転（ロドリゲス）。"""
    target = np.array([0.0, 0.0, 1.0])
    axis = np.cross(normal, target)
    sine = float(np.linalg.norm(axis))
    if sine < 1e-9:
        return np.eye(3)
    axis = axis / sine
    angle = float(np.arctan2(sine, float(np.dot(normal, target))))
    skew = np.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


# 当てはめを絞り込む閾値[m]。粗→細で、最後は床の粗さ程度まで詰める
TRIM_SCHEDULE = (0.20, 0.10, 0.05, 0.03)


def fit_plane(points: np.ndarray) -> "tuple[np.ndarray, np.ndarray]":
    """最小二乗で平面を当てる。戻りは (単位法線, 重心)。"""
    centroid = points.mean(axis=0)
    _, _, basis = np.linalg.svd(points - centroid, full_matrices=False)
    normal = basis[-1]
    return normal / np.linalg.norm(normal), centroid


def level_cloud(points: np.ndarray) -> "tuple[np.ndarray, np.ndarray, float, float]":
    """床平面を見つけ、重力方向に整列して床を z=0 に置く。

    天井のほうが点数が多いことがある（この部屋では床 143k に対し天井 224k）ので、
    素の平面当てはめは天井を拾う。**下半分に絞ってからヒストグラムの山を種にする**。

    RANSAC を使わないのは**再現性のため**。`segment_plane` は確率的で、走らせるたび
    整列が僅かに動き、下流の格子が 0.2% ほど揺れる。maxRange の比較をしたいのに
    比較器そのものが揺れては困る。閾値を粗→細に絞る最小二乗なら決定的になる。

    戻りは (整列後の点, 回転行列, 引いた床の高さ, 補正した傾き[deg])。
    """
    z = points[:, 2]
    lower = z[z <= np.median(z)]
    counts, edges = np.histogram(lower, bins=200)
    seed = float(edges[int(counts.argmax())] + (edges[1] - edges[0]) / 2.0)
    near = points[np.abs(z - seed) <= 0.30]
    subset = near
    with np.errstate(all="ignore"):
        for threshold in TRIM_SCHEDULE:
            normal, centroid = fit_plane(subset)
            residual = np.abs((near - centroid) @ normal)
            kept = near[residual <= threshold]
            if len(kept) < 100:
                break
            subset = kept
        normal, centroid = fit_plane(subset)
    if normal[2] < 0.0:
        normal = -normal
    rotation = rotation_to_z(normal)
    tilt = float(np.degrees(np.arccos(float(np.clip(normal[2], -1.0, 1.0)))))
    # numpy の matmul は SIMD の端数処理で偽の divide-by-zero 警告を出す。
    # 黙らせたうえで、結果が有限かを自分で確かめる。
    with np.errstate(all="ignore"):
        leveled = points @ rotation.T
        floor_z = float(np.median((subset @ rotation.T)[:, 2]))
    if not np.isfinite(leveled).all():
        raise SystemExit("整列後に非有限値が出た。入力点群を確認する")
    leveled[:, 2] -= floor_z
    return leveled, rotation, floor_z, tilt


# ── 格子を作る ────────────────────────────────────────────────
def group_percentile(index: np.ndarray, values: np.ndarray, size: int,
                     percentile: float) -> np.ndarray:
    """index ごとに values の分位を返す。点が無いセルは NaN。"""
    order = np.lexsort((values, index))
    sorted_index, sorted_values = index[order], values[order]
    span = np.arange(size)
    starts = np.searchsorted(sorted_index, span, side="left")
    counts = np.searchsorted(sorted_index, span, side="right") - starts
    result = np.full(size, np.nan)
    present = counts > 0
    offset = np.floor((counts[present] - 1) * percentile / 100.0).astype(np.int64)
    result[present] = sorted_values[starts[present] + offset]
    return result


def build_grids(points: np.ndarray, cell: float, min_points: int) -> dict:
    """占有格子（点数と最大高さ）と床の高さ格子を作る。"""
    origin = np.floor(points[:, :2].min(axis=0) / cell) * cell
    column = ((points[:, 0] - origin[0]) / cell).astype(np.int64)
    row = ((points[:, 1] - origin[1]) / cell).astype(np.int64)
    ncol, nrow = int(column.max()) + 1, int(row.max()) + 1
    flat = row * ncol + column
    size = nrow * ncol
    height = points[:, 2]

    obstacle = (height >= OBSTACLE_BAND[0]) & (height < OBSTACLE_BAND[1])
    counts = np.bincount(flat[obstacle], minlength=size)
    top = np.zeros(size)
    np.maximum.at(top, flat[obstacle], height[obstacle])

    ground = (height >= FLOOR_BAND[0]) & (height < FLOOR_BAND[1])
    elevation = group_percentile(flat[ground], height[ground], size, FLOOR_PERCENTILE)

    return {
        "origin": origin,
        "cell": cell,
        "occupied": (counts >= min_points).reshape(nrow, ncol),
        "top": top.reshape(nrow, ncol),
        "elevation": elevation.reshape(nrow, ncol),
    }


def filter_small_clusters(mask: np.ndarray, min_cells: int) -> "tuple[np.ndarray, dict]":
    """小さい連結成分を捨てる。人の残骸と遠方の外れ値がここで落ちる。"""
    labels, total = ndimage.label(mask, structure=np.ones((3, 3), dtype=bool))
    if total == 0:
        return mask, {"clusters": 0, "dropped_clusters": 0, "dropped_cells": 0}
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    keep = sizes >= min_cells
    cleaned = keep[labels]
    return cleaned, {
        "clusters": int(total),
        "dropped_clusters": int(total - keep.sum()),
        "dropped_cells": int(mask.sum() - cleaned.sum()),
    }


def crop_by_range(mask: np.ndarray, cell: float, origin: np.ndarray,
                  trajectory: np.ndarray, max_range: float) -> "tuple[np.ndarray, int]":
    """軌跡から max_range より遠いセルを落とす。

    **なぜ要るのか。** この記録では軌跡がわずか 3.4×7.7m しかないのに、占有セルは
    軌跡から 24.6m 先まで散っていた。遠方の疎な外れ値が外接矩形を 35×43m まで
    引き伸ばし、床のハイトフィールドが部屋の外まで作られてしまう。
    12m で切ると 23.8×30.4m になり、別の道具が出しているコア寸法と一致する。

    連結成分の大きさで切る手もあるが、この地図は壁が繋がっていない
    （最大成分でも 1,876 セル・全体の 19%）ので、大きさでは選り分けられない。
    """
    if max_range <= 0 or len(trajectory) == 0:
        return mask, 0
    rows, cols = np.nonzero(mask)
    centers = np.column_stack([origin[0] + (cols + 0.5) * cell,
                               origin[1] + (rows + 0.5) * cell])
    far = cKDTree(trajectory[:, :2]).query(centers)[0] > max_range
    cleaned = mask.copy()
    cleaned[rows[far], cols[far]] = False
    return cleaned, int(far.sum())


def crop_to_content(grids: dict, mask: np.ndarray, margin: int = 2) -> dict:
    """占有セルの外接矩形へ切り詰める。遠方の外れ値を落としたあとの実寸になる。"""
    rows, cols = np.nonzero(mask)
    r0 = max(int(rows.min()) - margin, 0)
    r1 = min(int(rows.max()) + margin + 1, mask.shape[0])
    c0 = max(int(cols.min()) - margin, 0)
    c1 = min(int(cols.max()) + margin + 1, mask.shape[1])
    return {
        "origin": grids["origin"] + np.array([c0, r0]) * grids["cell"],
        "cell": grids["cell"],
        "occupied": mask[r0:r1, c0:c1],
        "top": grids["top"][r0:r1, c0:c1],
        "elevation": grids["elevation"][r0:r1, c0:c1],
    }


# ── (A) 箱にする ──────────────────────────────────────────────
def boxes_from_grid(mask: np.ndarray, level: np.ndarray, cell: float,
                    origin: np.ndarray) -> "list[tuple[float, float, float, float, float]]":
    """行ごとに、同じ高さ段の連続セルを1つの箱へまとめる（行方向のランレングス）。

    高さを量子化してから繋げるのが要点。生の最大高さで繋げると1セルごとに段が
    変わって箱が減らない。
    """
    boxes = []
    nrow, ncol = mask.shape
    for row in range(nrow):
        occupied_row, level_row = mask[row], level[row]
        col = 0
        while col < ncol:
            if not occupied_row[col]:
                col += 1
                continue
            height = level_row[col]
            end = col
            while end + 1 < ncol and occupied_row[end + 1] and level_row[end + 1] == height:
                end += 1
            width = end - col + 1
            boxes.append((
                float(origin[0] + (col + width / 2.0) * cell),   # 中心 x
                float(origin[1] + (row + 0.5) * cell),           # 中心 y
                float(height / 2.0),                             # 中心 z
                float(width * cell / 2.0),                       # 半幅 x
                float(height / 2.0),                             # 半高 z
            ))
            col = end + 1
    return boxes


# ── (C) ハイトフィールドにする ────────────────────────────────
def reject_floor_outliers(elevation: np.ndarray, tolerance: float,
                          min_region: int) -> "tuple[np.ndarray, int]":
    """全体の床面から外れたセルを捨てる。ただし広く連続していれば段差として残す。

    **なぜ局所中央値ではだめだったか（2026-09-05 実測）。** 外れセルは孤立せず塊で
    出るので、5×5 の局所中央値も一緒にずれて検出できなかった。

    **なぜ点数で切れないか。** 点が多いセルほど床高さが狂う。10点以上のセルは
    床高さの中央値が +0.108m で、52.5% が 0.1m 超だった。床は垂直FOV −7° の
    斜入射で数点しか返らないが、壁や什器の垂直面は高さ方向に何度も当たるためである。
    「点が多い＝信頼できる」は、この LiDAR の床については逆になる。

    そこで**全体の床面からの外れ**で判定する。本物の段差やスロープは広い塊として
    現れるので、塊の大きさで免除する。
    """
    valid = ~np.isnan(elevation)
    if not valid.any():
        return elevation, 0
    base = float(np.nanmedian(elevation))
    deviating = valid & (np.abs(elevation - base) > tolerance)
    labels, total = ndimage.label(deviating, structure=np.ones((3, 3), dtype=bool))
    if total:
        sizes = np.bincount(labels.ravel())
        sizes[0] = 0
        drop = deviating & (sizes < min_region)[labels]
    else:
        drop = deviating
    cleaned = elevation.copy()
    cleaned[drop] = np.nan
    return cleaned, int(drop.sum())


def build_heightfield(elevation: np.ndarray, cell: float, fill_radius: float,
                      tolerance: float, min_region: int, smooth: float
                      ) -> "tuple[np.ndarray, float, float, int, float, float]":
    """欠けたセルを埋め、0〜1に正規化する。戻りは (0〜1の格子, 最低, 幅)。

    最近傍で埋めるのは実測値の近くだけにする。無制限に埋めると、床を観測して
    いない部屋の外まで一番近い床の高さが引き伸ばされ、斜めの尾根が生える
    （2026-09-05 に描画して確認）。遠い所は中央値で平らにする。
    """
    missing = np.isnan(elevation)
    if missing.all():
        return np.zeros_like(elevation), 0.0, 0.0, 0, 0.0, 0.0
    measured = float((~np.isnan(elevation)).mean())
    elevation, rejected = reject_floor_outliers(elevation, tolerance, min_region)
    missing = np.isnan(elevation)
    if missing.any():
        median = float(np.nanmedian(elevation))
        distance, nearest = ndimage.distance_transform_edt(missing, return_indices=True)
        elevation = elevation[tuple(nearest)]
        elevation[distance * cell > fill_radius] = median
    if smooth > 0.0:
        # **平滑化しないと歩けない（2026-09-05 実測）。** 実測できるのは床の 36% だけで、
        # 残りは最近傍で埋めた外挿。そのモザイクの継ぎ目が数 cm の段差になり、
        # 平地で学習した歩行ポリシーが 13.7m で転んだ。均すと 44.6m 歩いて到達する。
        elevation = ndimage.gaussian_filter(elevation, sigma=smooth / cell, mode="nearest")
    low, high = float(elevation.min()), float(elevation.max())
    span = max(high - low, 1e-3)
    slope = float(np.degrees(np.arctan(
        np.hypot(*np.gradient(elevation, cell)).max()))) if elevation.size else 0.0
    return (elevation - low) / span, low, span, rejected, measured, slope


# ── 指標 ──────────────────────────────────────────────────────
def cell_metrics(mask: np.ndarray, level: np.ndarray, cell: float, origin: np.ndarray,
                 trajectory: np.ndarray, radius: float) -> dict:
    """占有セルを軌跡からの距離で仕分ける。**指標は必ず対で見る。**

    - `phantom`: 軌跡から radius 以内に残った占有セル。ロボットが実際に歩いた所に
      静止物はありえないので、ここに残る箱は**シミュレータの中で偽の障害物になる**。
      少ないほどよい。
    - `structure`: 軌跡から FAR_RADIUS 以上離れた占有セル。追従者が居られない距離
      なので壁・机・什器である。**多いほどよい**。
    - `tall`: 高さ TALL_LEVEL 以上の箱。ただし追従者の頭（〜1.8m）もここに入る。
    - `tall_far`: 遠方かつ壁高。**追従者が居られない距離の壁**なので、これが減ったら
      本当に壁を削っている。壁の削れを判定するのはこちらであって `tall` ではない。

    phantom だけを見ると「削るほど良い」になり、壁まで消した設定が最良に見える。
    """
    rows, cols = np.nonzero(mask)
    if len(rows) == 0:
        return {"phantom": 0, "structure": 0, "tall": 0, "tall_far": 0}
    tall = int((level[rows, cols] >= TALL_LEVEL).sum())
    if len(trajectory) == 0:
        return {"phantom": -1, "structure": -1, "tall": tall, "tall_far": -1}
    centers = np.column_stack([origin[0] + (cols + 0.5) * cell,
                               origin[1] + (rows + 0.5) * cell])
    distance = cKDTree(trajectory[:, :2]).query(centers)[0]
    far = distance >= FAR_RADIUS
    return {"phantom": int((distance <= radius).sum()),
            "structure": int(far.sum()),
            "tall": tall,
            "tall_far": int((far & (level[rows, cols] >= TALL_LEVEL)).sum())}


def find_spawn(mask: np.ndarray, cell: float, origin: np.ndarray,
               trajectory: np.ndarray) -> "tuple[float, float]":
    """軌跡の始点に最も近い、十分に開けた自由セルを返す。"""
    clearance = ndimage.distance_transform_edt(~mask) * cell
    free = clearance >= SPAWN_CLEARANCE
    if not free.any():
        free = clearance >= clearance.max() * 0.9
    rows, cols = np.nonzero(free)
    centers = np.column_stack([origin[0] + (cols + 0.5) * cell,
                               origin[1] + (rows + 0.5) * cell])
    anchor = trajectory[0, :2] if len(trajectory) else centers.mean(axis=0)
    best = int(np.argmin(np.linalg.norm(centers - anchor, axis=1)))
    return float(centers[best, 0]), float(centers[best, 1])


# ── MJCF を書く ───────────────────────────────────────────────
SCENE_TEMPLATE = """<mujoco model="{model}">
  <!-- {provenance} -->
  <include file="{robot}"/>
  <statistic center="{cx:.3f} {cy:.3f} 1.000" extent="{extent:.3f}"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0.2 0.2 0.2"/>
    <global azimuth="-90" elevation="-45" offwidth="1920" offheight="1080"/>
  </visual>
  <asset>
    <!-- 行0が +Y 側・列0が −X 側。書き出し側で flipud 済み（2026-09-05 実測）。
         file は "../" 始まり。include した g1_12dof.xml の meshdir="meshes/" が
         hfield の file にも効くため、素の名前だと meshes/ の中を探して失敗する。 -->
    <hfield name="floor_hf" file="../{hfield}" size="{rx:.4f} {ry:.4f} {rz:.4f} 0.500"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.20 0.30 0.40" rgb2="0.10 0.20 0.30" markrgb="0.8 0.8 0.8"
             width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="{repeat:.1f} {repeat:.1f}"/>
  </asset>
  <worldbody>
    <light pos="{cx:.3f} {cy:.3f} 6" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="hfield" hfield="floor_hf" material="groundplane"
          pos="{fx:.4f} {fy:.4f} {fz:.4f}"/>
{geoms}
  </worldbody>
  <keyframe><key name="spawn" qpos="{sx:.3f} {sy:.3f} {sz:.3f} 1 0 0 0 {joints}"/></keyframe>
</mujoco>
"""


def write_scene(path: Path, model: str, provenance: str, hfield_name: str,
                boxes: list, grids: dict, field: np.ndarray, low: float, span: float,
                spawn: "tuple[float, float]") -> None:
    cell = grids["cell"]
    nrow, ncol = grids["occupied"].shape
    origin = grids["origin"]
    center = (float(origin[0] + ncol * cell / 2.0), float(origin[1] + nrow * cell / 2.0))
    lines = []
    for i, (x, y, z, half_x, half_z) in enumerate(boxes):
        lines.append(
            f'    <geom name="map_{i}" type="box" pos="{x:.4f} {y:.4f} {z:.4f}" '
            f'size="{half_x:.4f} {cell / 2.0:.4f} {half_z:.4f}" rgba="0.55 0.55 0.60 1"/>')
    path.write_text(SCENE_TEMPLATE.format(
        model=model, provenance=provenance, robot=ROBOT_INCLUDE,
        cx=center[0], cy=center[1],
        extent=max(ncol, nrow) * cell / 2.0,
        hfield=hfield_name,
        rx=ncol * cell / 2.0, ry=nrow * cell / 2.0, rz=max(span, 1e-3),
        repeat=max(ncol, nrow) * cell / 4.0,
        fx=center[0], fy=center[1], fz=low,
        geoms="\n".join(lines),
        sx=spawn[0], sy=spawn[1], sz=SPAWN_HEIGHT + low,
        joints=SPAWN_JOINTS), encoding="utf-8")
    _ = field


def build_one(session: Path, pcd_path: Path, trajectory: np.ndarray, args: argparse.Namespace,
              scene_dir: Path, sim_dir: Path, name: str) -> dict:
    variant = pcd_path.stem.removeprefix("map_")
    print(f"[{variant}] 読み込み {pcd_path.name}", flush=True)
    points = np.asarray(o3d.io.read_point_cloud(str(pcd_path)).points)
    points = points[np.isfinite(points).all(axis=1)]

    leveled, rotation, floor_z, tilt = level_cloud(points)
    with np.errstate(all="ignore"):
        traj = (trajectory @ rotation.T) if len(trajectory) else trajectory
    if len(traj):
        traj = traj.copy()
        traj[:, 2] -= floor_z
    print(f"[{variant}] 重力整列: 傾き {tilt:.2f}° を補正、床を z=0 へ（元 {floor_z:.3f}m）",
          flush=True)

    grids = build_grids(leveled, args.cell, args.min_points)
    raw_cells = int(grids["occupied"].sum())
    mask, cluster_stats = filter_small_clusters(grids["occupied"], args.min_cluster)
    mask, far_cells = crop_by_range(mask, grids["cell"], grids["origin"], traj, args.max_range)
    grids = crop_to_content(grids, mask)
    mask = grids["occupied"]
    print(f"[{variant}] 占有セル {raw_cells:,} → {int(mask.sum()):,}"
          f"（小さい塊 {cluster_stats['dropped_cells']:,} / 遠方 {far_cells:,} セルを除去）"
          f"  実寸 {mask.shape[1] * grids['cell']:.1f}×{mask.shape[0] * grids['cell']:.1f} m",
          flush=True)

    level = np.ceil(np.clip(grids["top"], 0.0, OBSTACLE_BAND[1]) / args.height_step)
    level = np.clip(level * args.height_step, args.height_step, OBSTACLE_BAND[1])
    boxes = boxes_from_grid(mask, level, grids["cell"], grids["origin"])

    field, low, span, rejected, measured, slope = build_heightfield(
        grids["elevation"], grids["cell"], args.fill_radius, args.floor_tolerance,
        args.floor_min_region, args.floor_smooth)
    print(f"[{variant}] 床: 実測できたセル {measured * 100:.1f}%（残りは外挿）、"
          f"外れ {rejected:,} 個を除去、高低差 {span * 100:.1f} cm / 最大勾配 {slope:.1f}°",
          flush=True)
    hfield_name = f"_hfield_{name}_{variant}.png"
    # 行0を +Y 側にするため flipud する（2026-09-05 に mj_ray で実測）
    Image.fromarray(np.flipud((field * 255.0).round()).astype(np.uint8), mode="L").save(
        scene_dir / hfield_name)

    metrics = cell_metrics(mask, level, grids["cell"], grids["origin"],
                           traj, args.phantom_radius)
    spawn = find_spawn(mask, grids["cell"], grids["origin"], traj)
    scene_path = scene_dir / f"_scene_{name}_{variant}.xml"
    write_scene(scene_path, f"{name}_{variant}",
                f"{session.name} / {pcd_path.name} から pcd_to_mjcf.py が生成",
                hfield_name, boxes, grids, field, low, span, spawn)

    nrow, ncol = mask.shape
    stats = {
        "variant": variant,
        "source": pcd_path.name,
        "points": int(len(points)),
        "scene": scene_path.name,
        "hfield": hfield_name,
        "tilt_corrected_deg": round(tilt, 3),
        "floor_offset_m": round(floor_z, 4),
        "cell": args.cell,
        "grid": [nrow, ncol],
        "extent_m": [round(ncol * args.cell, 2), round(nrow * args.cell, 2)],
        "cells_raw": raw_cells,
        "cells_kept": int(mask.sum()),
        "dropped_clusters": cluster_stats["dropped_clusters"],
        "dropped_cells": cluster_stats["dropped_cells"],
        "dropped_far_cells": far_cells,
        "boxes": len(boxes),
        "phantom_cells": metrics["phantom"],
        "structure_cells": metrics["structure"],
        "tall_cells": metrics["tall"],
        "tall_far_cells": metrics["tall_far"],
        "phantom_area_m2": None,
        "hfield_range_m": round(span, 4),
        "floor_outliers": rejected,
        "floor_measured_pct": round(measured * 100.0, 1),
        "floor_max_slope_deg": round(slope, 1),
        "spawn": [round(spawn[0], 3), round(spawn[1], 3)],
    }
    stats["phantom_area_m2"] = round(stats["phantom_cells"] * args.cell ** 2, 3)
    np.savez_compressed(sim_dir / f"{variant}.npz", occupied=mask, level=level,
                        elevation=field * span + low, origin=grids["origin"],
                        cell=np.array(grids["cell"]), trajectory=traj[:, :2] if len(traj) else np.zeros((0, 2)))
    print(f"[{variant}] 箱 {len(boxes):,} 個 / 偽障害物 {stats['phantom_cells']:,} セル"
          f"（{stats['phantom_area_m2']:.2f} m²） / 遠方構造 {stats['structure_cells']:,} セル"
          f" / 遠方の壁高 {stats['tall_far_cells']:,} セル → {scene_path.name}", flush=True)
    return stats


def main() -> None:
    parser = argparse.ArgumentParser(description="点群を MuJoCo シーン（箱＋hfield）にする")
    parser.add_argument("session", type=Path, help="runs/<session_id>")
    parser.add_argument("maps", nargs="+", help="map/ 以下の PCD 名")
    parser.add_argument("--cell", type=float, default=CELL, help=f"格子の一辺[m]（既定 {CELL}）")
    parser.add_argument("--height-step", type=float, default=HEIGHT_STEP,
                        help=f"箱の高さの量子化[m]（既定 {HEIGHT_STEP}）")
    parser.add_argument("--min-points", type=int, default=MIN_POINTS,
                        help=f"セルを占有とみなす最低点数（既定 {MIN_POINTS}）")
    parser.add_argument("--min-cluster", type=int, default=MIN_CLUSTER,
                        help=f"捨てる連結成分のセル数の閾値（既定 {MIN_CLUSTER}）")
    parser.add_argument("--max-range", type=float, default=MAX_RANGE,
                        help=f"軌跡からこれ以上離れたセルを捨てる[m]、0で無効（既定 {MAX_RANGE}）")
    parser.add_argument("--fill-radius", type=float, default=FILL_RADIUS,
                        help=f"床の高さを最近傍で埋めてよい距離[m]（既定 {FILL_RADIUS}）")
    parser.add_argument("--floor-tolerance", type=float, default=FLOOR_TOLERANCE,
                        help=f"床の外れセル判定[m]（既定 {FLOOR_TOLERANCE}）")
    parser.add_argument("--floor-min-region", type=int, default=FLOOR_MIN_REGION,
                        help=f"本物の段差として残す塊のセル数（既定 {FLOOR_MIN_REGION}）")
    parser.add_argument("--floor-smooth", type=float, default=FLOOR_SMOOTH,
                        help=f"床を均す幅[m]、0で無効（既定 {FLOOR_SMOOTH}）")
    parser.add_argument("--phantom-radius", type=float, default=PHANTOM_RADIUS,
                        help=f"軌跡からこれ以内を偽障害物とみなす[m]（既定 {PHANTOM_RADIUS}）")
    parser.add_argument("--scene-dir", type=Path,
                        default=QUICKSTART.parents[2] / "Navigation/sim/assets/g1_description",
                        help="MJCF の出力先。g1_12dof.xml と同じ場所である必要がある")
    args = parser.parse_args()

    session = args.session.resolve()
    scene_dir = args.scene_dir.resolve()
    if not (scene_dir / ROBOT_INCLUDE).exists():
        raise SystemExit(f"{ROBOT_INCLUDE} が {scene_dir} に無い。--scene-dir を確認する")
    sim_dir = session / "sim"
    sim_dir.mkdir(parents=True, exist_ok=True)
    name = json.loads((session / "manifest.json").read_text())["name"]

    try:
        trajectory = read_trajectory(session)
    except Exception as error:                      # noqa: BLE001
        print(f"[WARN] 軌跡を読めなかったので偽障害物の指標は出ない: {error}", flush=True)
        trajectory = np.zeros((0, 3))

    results = [build_one(session, session / "map" / m, trajectory, args, scene_dir, sim_dir, name)
               for m in args.maps]
    report = {"session": session.name, "name": name,
              "settings": {"cell": args.cell, "height_step": args.height_step,
                           "min_points": args.min_points, "min_cluster": args.min_cluster,
                           "max_range": args.max_range, "fill_radius": args.fill_radius,
                           "floor_tolerance": args.floor_tolerance,
                           "floor_smooth": args.floor_smooth,
                           "obstacle_band": list(OBSTACLE_BAND), "floor_band": list(FLOOR_BAND),
                           "phantom_radius": args.phantom_radius},
              "variants": results}
    (sim_dir / "scenes.json").write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                         encoding="utf-8")
    print(f"\n[完了] {sim_dir / 'scenes.json'}", flush=True)


if __name__ == "__main__":
    main()
