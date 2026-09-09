"""PointCloud2 のエンコードが正しいかを検証する（Isaac Sim 不要）。

point_step や offset を間違えても ROS は黙って受け取るため、購読側で
点がぐちゃぐちゃになるまで気付けない。ここで往復（エンコード -> デコード）を
確認しておく。

実行方法:
    cd <このリポジトリ>/SimEnv3D
    source env.sh
    python3 src/test_pointcloud_encoding.py
"""

from __future__ import annotations

import struct
import sys

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

PASS = "[OK]"
FAIL = "[NG]"
failures = 0

FRAME_LIDAR3D = "livox_frame"


def build_message(points: np.ndarray) -> PointCloud2:
    """ros_bridge.publish_points と同じ手順で PointCloud2 を組む。

    実装を変えたらこちらも合わせること（重複だが、Isaac Sim 無しで
    検証できる価値の方が大きい）。
    """
    msg = PointCloud2()
    msg.header.frame_id = FRAME_LIDAR3D
    pts = np.asarray(points, dtype=np.float32)
    num_points = int(pts.shape[0])

    msg.height = 1
    msg.width = num_points
    msg.is_dense = True
    msg.is_bigendian = False

    fields = []
    for i, name in enumerate(("x", "y", "z")):
        field = PointField()
        field.name = name
        field.offset = 4 * i
        field.datatype = PointField.FLOAT32
        field.count = 1
        fields.append(field)
    fields.extend(
        [
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
            PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
            PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
        ]
    )
    msg.fields = fields

    msg.point_step = 26
    msg.row_step = msg.point_step * num_points
    data = bytearray(msg.row_step)
    for index, (x, y, z) in enumerate(pts):
        struct.pack_into(
            "<ffffBBd",
            data,
            index * msg.point_step,
            float(x),
            float(y),
            float(z),
            100.0,
            0,
            index % 4,
            0.1 * index / max(1, num_points - 1),
        )
    msg.data = bytes(data)
    return msg


def test_roundtrip() -> None:
    """エンコードした点が、公式デコーダで元の値に戻る。"""
    global failures
    print("[Test] エンコード -> デコードの往復")
    original = np.array(
        [
            [1.0, 2.0, 3.0],
            [-4.5, 0.25, 12.75],
            [0.0, 0.0, 0.0],
            [30.0, -30.0, 1.5],
        ],
        dtype=np.float32,
    )
    msg = build_message(original)
    decoded = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))

    if decoded.shape != original.shape:
        failures += 1
        print(f"  {FAIL} 形状が違う: {decoded.shape} != {original.shape}")
        return

    diff = float(np.abs(decoded - original).max())
    if diff < 1e-6:
        print(f"  {PASS} 全 {len(original)} 点が一致（最大差 {diff:.9f}）")
    else:
        failures += 1
        print(f"  {FAIL} 値がずれた（最大差 {diff}）")
        print(f"       元: {original.tolist()}")
        print(f"       後: {decoded.tolist()}")


def test_metadata() -> None:
    """メッセージのメタデータが順序なし点群の規約に合っている。"""
    global failures
    print("[Test] メタデータ")
    pts = np.zeros((100, 3), dtype=np.float32)
    msg = build_message(pts)

    checks = [
        ("height", msg.height, 1),
        ("width", msg.width, 100),
        ("point_step", msg.point_step, 26),
        ("row_step", msg.row_step, 2600),
        ("data の長さ", len(msg.data), 2600),
        ("frame_id", msg.header.frame_id, FRAME_LIDAR3D),
    ]
    for name, actual, expected in checks:
        if actual == expected:
            print(f"  {PASS} {name} = {actual}")
        else:
            failures += 1
            print(f"  {FAIL} {name} = {actual} (期待 {expected})")


def test_empty() -> None:
    """当たりが 0 本でも壊れない。"""
    global failures
    print("[Test] 空の点群")
    msg = build_message(np.zeros((0, 3), dtype=np.float32))
    if msg.width == 0 and len(msg.data) == 0 and msg.row_step == 0:
        print(f"  {PASS} 空でも整合している")
    else:
        failures += 1
        print(f"  {FAIL} width={msg.width} data={len(msg.data)} row_step={msg.row_step}")


def test_large() -> None:
    """Mid-360公称200k points/sを10Hzで区切った点数で動く。"""
    global failures
    print("[Test] 実サイズ（20000 点）")
    rng = np.random.default_rng(0)
    pts = (rng.random((20000, 3), dtype=np.float32) - 0.5) * 60.0
    msg = build_message(pts)
    decoded = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
    diff = float(np.abs(decoded - pts).max())
    size_kb = len(msg.data) / 1024.0
    if diff < 1e-6:
        print(f"  {PASS} 20000 点が一致（{size_kb:.0f} KB / スキャン）")
        # 10Hz 配信時の帯域を出しておく（参考値）
        print(f"  [INFO] 10Hz なら {size_kb * 10 / 1024:.1f} MB/s")
    else:
        failures += 1
        print(f"  {FAIL} 値がずれた（最大差 {diff}）")


def main() -> None:
    print("=" * 60)
    print("PointCloud2 エンコードの検証")
    print("=" * 60)
    test_roundtrip()
    test_metadata()
    test_empty()
    test_large()
    print("=" * 60)
    if failures:
        print(f"{FAIL} {failures} 件が失敗しました")
        sys.exit(1)
    print(f"{PASS} すべて成功しました")


main()
