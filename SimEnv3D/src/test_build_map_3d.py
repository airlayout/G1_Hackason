"""build_map_3d.py の voxel 化と保存を検証する（Isaac Sim 不要）。

既知の形（前方 3 m の壁）を合成して流し、地図がその形になるかを見る。
座標変換の誤りは地図が壊れて初めて分かるが、実走行で気付くのは高コストなので
ここで先に潰す（2D では yaw の二重適用で 11 m ずれた地図を「正常」と
誤認した前例がある）。

実行方法:
    cd /home/spacedata/isaac_dev/G1/SimEnv3D
    source env.sh
    python3 src/test_build_map_3d.py
"""

from __future__ import annotations

import math
import os
import shutil
import sys
import tempfile

import numpy as np

sys.path.insert(0, "src")

PASS = "[OK]"
FAIL = "[NG]"
failures = 0


def make_builder(monkey_odom: tuple[float, float, float]):
    """PointMapBuilder を ROS 無しで作る。

    rclpy.init() を避けたいので、Node の初期化を通さずに必要な属性だけ持つ
    軽量なスタブを使う。検証したいのは _on_points の座標変換なので十分。
    """
    import build_map_3d as bm

    class Stub:
        """_on_points だけを借用するためのスタブ。"""

        def __init__(self) -> None:
            self._latest_odom = monkey_odom
            self.voxel_hits: dict[tuple[int, int, int], int] = {}
            self.num_clouds = 0
            self.num_points = 0

        _on_points = bm.PointMapBuilder._on_points

    return Stub(), bm


class FakeCloud:
    """point_cloud2.read_points_numpy の戻りを差し替えるための入れ物。"""

    def __init__(self, points: np.ndarray) -> None:
        self.points = points


def test_wall_position() -> None:
    """センサ前方 3 m の壁が、ワールド座標の正しい位置に焼かれる。"""
    global failures
    print("[Test] 壁の位置（odom が原点・yaw=0）")
    import build_map_3d as bm

    # センサ座標で前方 3 m、幅 2 m、センサ基準 z=-0.5〜+0.5 の壁
    ys = np.arange(-1.0, 1.0, 0.05)
    zs = np.arange(-0.5, 0.5, 0.05)
    yy, zz = np.meshgrid(ys, zs)
    xx = np.full_like(yy, 3.0)
    pts = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1).astype(np.float32)

    builder, _ = make_builder((0.0, 0.0, 0.0))
    # read_points_numpy を差し替える
    orig = bm.point_cloud2.read_points_numpy
    bm.point_cloud2.read_points_numpy = lambda msg, field_names: msg.points
    try:
        builder._on_points(FakeCloud(pts))
    finally:
        bm.point_cloud2.read_points_numpy = orig

    if not builder.voxel_hits:
        failures += 1
        print(f"  {FAIL} voxel が 1 つも作られなかった")
        return

    keys = np.array(list(builder.voxel_hits.keys()))
    x_m = keys[:, 0] * bm.VOXEL_SIZE
    z_m = keys[:, 2] * bm.VOXEL_SIZE
    print(f"  voxel 数: {len(builder.voxel_hits)}")
    print(f"  X 範囲: {x_m.min():.2f} 〜 {x_m.max():.2f} m（期待 3.0 付近）")
    print(f"  Z 範囲: {z_m.min():.2f} 〜 {z_m.max():.2f} m（期待 0.6〜1.6）")

    if abs(x_m.mean() - 3.0) < 0.2:
        print(f"  {PASS} 壁が前方 3 m にある")
    else:
        failures += 1
        print(f"  {FAIL} X がずれている（平均 {x_m.mean():.2f} m）")

    # センサは地上 1.1 m。壁は センサ基準 -0.5〜+0.5 なので 0.6〜1.6 m。
    if 0.5 < z_m.min() < 0.8 and 1.4 < z_m.max() < 1.8:
        print(f"  {PASS} 高さがセンサ高さ 1.1 m を基準にしている")
    else:
        failures += 1
        print(f"  {FAIL} Z がずれている")


def test_yaw_applied() -> None:
    """yaw が適用される（2D 版とは逆に、3D では適用が必要）。"""
    global failures
    print("[Test] yaw の適用（yaw=90 度なら壁は +Y 方向へ）")
    import build_map_3d as bm

    pts = np.array([[3.0, 0.0, 0.0]], dtype=np.float32)
    builder, _ = make_builder((0.0, 0.0, math.radians(90.0)))
    orig = bm.point_cloud2.read_points_numpy
    bm.point_cloud2.read_points_numpy = lambda msg, field_names: msg.points
    try:
        builder._on_points(FakeCloud(pts))
    finally:
        bm.point_cloud2.read_points_numpy = orig

    if not builder.voxel_hits:
        failures += 1
        print(f"  {FAIL} voxel が作られなかった")
        return

    key = list(builder.voxel_hits.keys())[0]
    x_m, y_m = key[0] * bm.VOXEL_SIZE, key[1] * bm.VOXEL_SIZE
    print(f"  点の位置: ({x_m:.2f}, {y_m:.2f}) m（期待 (0.0, 3.0)）")
    if abs(x_m) < 0.2 and abs(y_m - 3.0) < 0.2:
        print(f"  {PASS} yaw が正しく適用されている")
    else:
        failures += 1
        print(f"  {FAIL} yaw の適用が誤っている")


def test_translation_applied() -> None:
    """odom の位置が適用される。"""
    global failures
    print("[Test] 平行移動の適用（odom が (10, -5) にいる）")
    import build_map_3d as bm

    pts = np.array([[3.0, 0.0, 0.0]], dtype=np.float32)
    builder, _ = make_builder((10.0, -5.0, 0.0))
    orig = bm.point_cloud2.read_points_numpy
    bm.point_cloud2.read_points_numpy = lambda msg, field_names: msg.points
    try:
        builder._on_points(FakeCloud(pts))
    finally:
        bm.point_cloud2.read_points_numpy = orig

    key = list(builder.voxel_hits.keys())[0]
    x_m, y_m = key[0] * bm.VOXEL_SIZE, key[1] * bm.VOXEL_SIZE
    print(f"  点の位置: ({x_m:.2f}, {y_m:.2f}) m（期待 (13.0, -5.0)）")
    if abs(x_m - 13.0) < 0.2 and abs(y_m + 5.0) < 0.2:
        print(f"  {PASS} 平行移動が正しく適用されている")
    else:
        failures += 1
        print(f"  {FAIL} 平行移動が誤っている")


def test_height_filter() -> None:
    """高さ範囲の外（床・天井）が除外される。"""
    global failures
    print("[Test] 高さフィルタ")
    import build_map_3d as bm

    # センサ基準 z: -1.1 は地上 0.0（床）、+2.0 は地上 3.1（天井の上）
    pts = np.array(
        [[3.0, 0.0, -1.09], [3.0, 0.0, 0.0], [3.0, 0.0, 2.0]], dtype=np.float32
    )
    builder, _ = make_builder((0.0, 0.0, 0.0))
    orig = bm.point_cloud2.read_points_numpy
    bm.point_cloud2.read_points_numpy = lambda msg, field_names: msg.points
    try:
        builder._on_points(FakeCloud(pts))
    finally:
        bm.point_cloud2.read_points_numpy = orig

    # 中央の 1 点（地上 1.1 m）だけ残るはず
    if len(builder.voxel_hits) == 1:
        print(f"  {PASS} 床と天井が除外された（3 点 -> 1 voxel）")
    else:
        failures += 1
        print(f"  {FAIL} voxel が {len(builder.voxel_hits)} 個（期待 1）")


def test_min_hits_and_save() -> None:
    """観測回数の閾値が効き、保存とスライス画像が出る。"""
    global failures
    print("[Test] 観測回数の閾値と保存")
    import build_map_3d as bm

    # 1 回だけの voxel と 5 回の voxel を混ぜる
    hits = {(30, 0, 11): 5, (99, 99, 11): 1}
    tmp = tempfile.mkdtemp()
    try:
        stem = os.path.join(tmp, "test_map")
        bm.save_map(hits, stem, min_hits=2)

        if not os.path.exists(f"{stem}.npz"):
            failures += 1
            print(f"  {FAIL} npz が保存されなかった")
            return

        data = np.load(f"{stem}.npz")
        n = data["voxels"].shape[0]
        if n == 1:
            print(f"  {PASS} 1 回だけの voxel が除外された（2 -> 1）")
        else:
            failures += 1
            print(f"  {FAIL} voxel が {n} 個（期待 1）")

        slice_dir = f"{stem}_slices"
        if os.path.isdir(slice_dir) and len(os.listdir(slice_dir)) >= 4:
            print(f"  {PASS} スライス画像が {len(os.listdir(slice_dir))} 枚出た")
        else:
            failures += 1
            print(f"  {FAIL} スライス画像が出ていない")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main() -> None:
    print("=" * 60)
    print("build_map_3d.py の検証")
    print("=" * 60)
    test_wall_position()
    test_yaw_applied()
    test_translation_applied()
    test_height_filter()
    test_min_hits_and_save()
    print("=" * 60)
    if failures:
        print(f"{FAIL} {failures} 件が失敗しました")
        sys.exit(1)
    print(f"{PASS} すべて成功しました")


main()
