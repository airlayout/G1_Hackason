"""ROS 標準の `sensor_msgs/PointCloud2` を numpy の点群に変換する。

G1 の LiDAR は DDS 上で `PointCloud2_`（`unitree_sdk2py.idl.sensor_msgs.msg.dds_`）
として流れてくる。この型は「点 1 つあたり何バイトで、どのオフセットに x/y/z が
あるか」を `fields` で自己記述するバイト列なので、機種ごとの決め打ちをせずに
そのメタデータから読み解く。
"""
from __future__ import annotations

import numpy as np

# PointField の datatype 定数（ROS の sensor_msgs/PointField と同じ）
_DATATYPES: dict[int, str] = {
    1: "i1",  # INT8
    2: "u1",  # UINT8
    3: "i2",  # INT16
    4: "u2",  # UINT16
    5: "i4",  # INT32
    6: "u4",  # UINT32
    7: "f4",  # FLOAT32
    8: "f8",  # FLOAT64
}


def describe(msg) -> str:  # type: ignore[no-untyped-def]
    """メッセージの中身を 1 行で説明する（トピック探索時の確認用）。"""
    fields = ", ".join(f"{f.name}@{f.offset}:{_DATATYPES.get(f.datatype, '?')}" for f in msg.fields)
    return (
        f"{msg.width}x{msg.height} point_step={msg.point_step} "
        f"frame_id={msg.header.frame_id!r} fields=[{fields}]"
    )


def to_xyz(msg) -> np.ndarray:  # type: ignore[no-untyped-def]
    """`PointCloud2_` を `(N, 3)` float64 の点群にする。

    x/y/z 以外のフィールド（intensity, ring, timestamp など）は捨てる。
    測距できなかった点が NaN / inf で入っていることがあるので、ここで落とす。
    """
    by_name = {f.name: f for f in msg.fields}
    missing = [n for n in ("x", "y", "z") if n not in by_name]
    if missing:
        raise ValueError(f"PointCloud2 に {missing} が無い（fields: {[f.name for f in msg.fields]}）")

    names: list[str] = []
    formats: list[str] = []
    offsets: list[int] = []
    for name in ("x", "y", "z"):
        field = by_name[name]
        if field.datatype not in _DATATYPES:
            raise ValueError(f"未対応の datatype: {field.datatype} (field {name})")
        names.append(name)
        # is_bigendian は普通 False。念のためバイトオーダーを明示する。
        formats.append((">" if msg.is_bigendian else "<") + _DATATYPES[field.datatype])
        offsets.append(field.offset)

    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    buffer = bytes(msg.data)
    n_points = msg.width * msg.height
    if len(buffer) < n_points * msg.point_step:
        # 途中で切れたフレームは、読める範囲だけ使う
        n_points = len(buffer) // msg.point_step

    raw = np.frombuffer(buffer, dtype=dtype, count=n_points)
    points = np.stack([raw["x"], raw["y"], raw["z"]], axis=-1).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]
