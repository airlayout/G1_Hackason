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


def sensor_to_world(
    pts: np.ndarray, x: float, y: float, yaw: float
) -> np.ndarray:
    """期待値を独立に計算する（実装と同じ式を書かないための参照実装）。

    センサ座標 -> 前傾を戻す -> yaw -> 平行移動、の順。
    """
    import build_map_3d as bm

    tilt = math.radians(bm.FORWARD_TILT_DEG)
    ct, st = math.cos(tilt), math.sin(tilt)
    bx = pts[:, 0] * ct + pts[:, 2] * st
    by = pts[:, 1]
    bz = -pts[:, 0] * st + pts[:, 2] * ct
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.stack(
        [x + bx * cy - by * sy, y + bx * sy + by * cy, bz + bm.LIDAR_HEIGHT], axis=1
    )


def test_wall_position() -> None:
    """センサ前方の壁が、前傾を考慮した正しい位置に焼かれる。

    前傾 20 度があるため、センサ座標で真正面 3 m の点はワールドでは
    水平 2.82 m・高さ 1.1-1.03=0.07 m 付近に来る。前傾を無視すると
    水平 3.0 m・高さ 1.1 m になり、10 m 先では 3.4 m もずれる。
    """
    global failures
    print("[Test] 壁の位置（前傾 20 度を考慮）")
    import build_map_3d as bm

    # センサ座標で前方 3 m、幅 2 m、センサ基準 z=-0.5〜+0.5 の壁
    ys = np.arange(-1.0, 1.0, 0.05)
    zs = np.arange(-0.5, 0.5, 0.05)
    yy, zz = np.meshgrid(ys, zs)
    xx = np.full_like(yy, 3.0)
    pts = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1).astype(np.float32)

    builder, _ = make_builder((0.0, 0.0, 0.0))
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

    # 参照実装で期待値を出す（高さフィルタで残るものだけ）
    expected = sensor_to_world(pts, 0.0, 0.0, 0.0)
    keep = (expected[:, 2] >= bm.MIN_Z) & (expected[:, 2] <= bm.MAX_Z)
    expected = expected[keep]

    keys = np.array(list(builder.voxel_hits.keys()))
    x_m = keys[:, 0] * bm.VOXEL_SIZE
    z_m = keys[:, 2] * bm.VOXEL_SIZE
    print(f"  voxel 数: {len(builder.voxel_hits)}")
    print(f"  X 範囲: {x_m.min():.2f} 〜 {x_m.max():.2f} m "
          f"(期待 {expected[:, 0].min():.2f} 〜 {expected[:, 0].max():.2f})")
    print(f"  Z 範囲: {z_m.min():.2f} 〜 {z_m.max():.2f} m "
          f"(期待 {expected[:, 2].min():.2f} 〜 {expected[:, 2].max():.2f})")

    # voxel の量子化（0.1 m）ぶんの誤差を許す
    if abs(x_m.min() - expected[:, 0].min()) < 0.15:
        print(f"  {PASS} 前傾を考慮した X 位置になっている")
    else:
        failures += 1
        print(f"  {FAIL} X がずれている")

    # 前傾を無視すると壁は「X=3.0 の平面」に潰れる（Z によらず一定）。
    # 前傾があると壁が傾くため X に幅が出る。ここでそれを確かめる。
    #
    # 注意: 壁は z=-0.5〜+0.5 の広がりを持つので、傾けると上端の X は
    # 3.0 に達する。「max が 3.0 でない」ではなく「幅がある」で判定する
    # （最初この判定を誤って書き、正しい実装を NG と誤判定した）。
    # 期待される幅は sin(20 度) x 壁の高さ(1.0 m) = 0.34 m だが、
    # voxel 0.1 m に量子化されるため 0.1 m 刻みでしか現れない。
    x_spread = float(x_m.max() - x_m.min())
    expected_spread = float(expected[:, 0].max() - expected[:, 0].min())
    print(f"  X の幅: {x_spread:.2f} m "
          f"（期待 {expected_spread:.2f} m / 前傾を無視すると 0.0）")
    if x_spread > 0.05:
        print(f"  {PASS} 前傾により壁が傾いている")
    else:
        failures += 1
        print(f"  {FAIL} 壁が平面のまま（前傾が適用されていない）")


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

    exp = sensor_to_world(pts, 0.0, 0.0, math.radians(90.0))[0]
    key = list(builder.voxel_hits.keys())[0]
    x_m, y_m = key[0] * bm.VOXEL_SIZE, key[1] * bm.VOXEL_SIZE
    print(f"  点の位置: ({x_m:.2f}, {y_m:.2f}) m"
          f"（期待 ({exp[0]:.2f}, {exp[1]:.2f})）")
    if abs(x_m - exp[0]) < 0.15 and abs(y_m - exp[1]) < 0.15:
        print(f"  {PASS} yaw が正しく適用されている（+Y 方向へ回った）")
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

    exp = sensor_to_world(pts, 10.0, -5.0, 0.0)[0]
    key = list(builder.voxel_hits.keys())[0]
    x_m, y_m = key[0] * bm.VOXEL_SIZE, key[1] * bm.VOXEL_SIZE
    print(f"  点の位置: ({x_m:.2f}, {y_m:.2f}) m"
          f"（期待 ({exp[0]:.2f}, {exp[1]:.2f})）")
    if abs(x_m - exp[0]) < 0.15 and abs(y_m - exp[1]) < 0.15:
        print(f"  {PASS} 平行移動が正しく適用されている")
    else:
        failures += 1
        print(f"  {FAIL} 平行移動が誤っている")


def test_height_filter() -> None:
    """高さ範囲の外（床・天井）が除外される。

    前傾を適用したあとの高さで判定されるため、期待値も参照実装から出す。
    """
    global failures
    print("[Test] 高さフィルタ")
    import build_map_3d as bm

    # 真下（床）・正面・真上（天井の上）の 3 点を、センサ座標で作る。
    # 前傾があるので「真正面」でも地上高は 1.1 m にならない。
    pts = np.array(
        [
            [0.0, 0.0, -3.0],   # 真下 3 m -> 地上 -1.9 m（床より下）
            [1.0, 0.0, 0.0],    # 正面 1 m -> 地上 0.76 m（範囲内）
            [0.0, 0.0, 3.0],    # 真上 3 m -> 地上 4.1 m（天井の上）
        ],
        dtype=np.float32,
    )
    builder, _ = make_builder((0.0, 0.0, 0.0))
    orig = bm.point_cloud2.read_points_numpy
    bm.point_cloud2.read_points_numpy = lambda msg, field_names: msg.points
    try:
        builder._on_points(FakeCloud(pts))
    finally:
        bm.point_cloud2.read_points_numpy = orig

    world = sensor_to_world(pts, 0.0, 0.0, 0.0)
    expected_n = int(
        ((world[:, 2] >= bm.MIN_Z) & (world[:, 2] <= bm.MAX_Z)).sum()
    )
    heights = ", ".join(f"{z:.2f}" for z in world[:, 2])
    print(f"  3 点の地上高: {heights} m（採用範囲 {bm.MIN_Z}〜{bm.MAX_Z} m）")

    if len(builder.voxel_hits) == expected_n:
        print(f"  {PASS} 範囲外が除外された（3 点 -> {expected_n} voxel）")
    else:
        failures += 1
        print(
            f"  {FAIL} voxel が {len(builder.voxel_hits)} 個（期待 {expected_n}）"
        )


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
