"""rosbag2に記録したPointCloud2から map_raw.pcd を再構成する。

現場でG1へ「Mapping終了」を送れなかった場合（通信断など）、G1側にPCDが
書き出されない。しかしrosbagには地図点群がそのまま残っているため、
持ち帰ってから地図を作り直せる。本モジュールがその再処理を担う。

依存ライブラリは使わない。rosbag2のdb3はSQLite、メッセージ本体はCDRなので、
標準ライブラリの sqlite3 と struct だけで読める（pcd.pyと同じ方針）。

対象は「既に地図座標系へ変換済みの増分スキャン」を前提とする。
onboard backendの ONBOARD_POINTS_TOPIC がこれにあたり、frame_id は map で
1メッセージあたり数百〜千点、フレームをまたいで蓄積すると地図になる。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import sqlite3
import struct

# sensor_msgs/msg/PointField の datatype
_FLOAT32 = 7
_FLOAT64 = 8


class _CdrReader:
    """CDR(little-endian)のリーダ。

    境界整列はカプセル化ヘッダー(先頭4バイト)を除いた本体先頭からの相対で行う。
    ROS 2の既定シリアライズ形式であり、rosbag2のdb3にはこの形式で入っている。
    """

    def __init__(self, payload: bytes) -> None:
        if len(payload) < 4:
            raise ValueError("CDRペイロードが短すぎます")
        # payload[0:2] が 00 01 なら little-endian。00 00 は big-endian。
        if payload[0:2] not in (b"\x00\x01",):
            raise ValueError(f"未対応のCDRエンディアンです: {payload[0:2].hex()}")
        self._buffer = payload[4:]
        self.position = 0

    def _align(self, size: int) -> None:
        remainder = self.position % size
        if remainder:
            self.position += size - remainder

    def uint8(self) -> int:
        value = self._buffer[self.position]
        self.position += 1
        return value

    def uint32(self) -> int:
        self._align(4)
        value = struct.unpack_from("<I", self._buffer, self.position)[0]
        self.position += 4
        return value

    def int32(self) -> int:
        self._align(4)
        value = struct.unpack_from("<i", self._buffer, self.position)[0]
        self.position += 4
        return value

    def string(self) -> str:
        length = self.uint32()
        raw = self._buffer[self.position : self.position + length]
        self.position += length
        return raw.rstrip(b"\x00").decode("utf-8", "replace")


@dataclass(frozen=True)
class CloudLayout:
    """1メッセージ分のPointCloud2のうち、点の取り出しに要る情報だけ。"""

    frame_id: str
    point_count: int
    point_step: int
    data_start: int
    data_length: int
    x_offset: int
    y_offset: int
    z_offset: int
    format_code: str


def parse_pointcloud2(payload: bytes) -> CloudLayout:
    """PointCloud2のCDRを解析し、点データの位置とx/y/zのoffsetを返す。"""

    reader = _CdrReader(payload)
    reader.int32()  # header.stamp.sec
    reader.uint32()  # header.stamp.nanosec
    frame_id = reader.string()
    height = reader.uint32()
    width = reader.uint32()

    offsets: dict[str, tuple[int, int]] = {}
    for _ in range(reader.uint32()):
        name = reader.string()
        offset = reader.uint32()
        datatype = reader.uint8()
        reader.uint32()  # count（x/y/zは常に1なので使わない）
        offsets[name] = (offset, datatype)

    if reader.uint8():  # is_bigendian
        raise ValueError("ビッグエンディアンの点群には対応していません")
    point_step = reader.uint32()
    reader.uint32()  # row_step
    data_length = reader.uint32()
    # uint8[] は1バイト境界なので、長さの直後がデータ本体。
    # _buffer はカプセル化ヘッダーを除いてあるので、payload基準では +4。
    data_start = reader.position + 4

    for axis in ("x", "y", "z"):
        if axis not in offsets:
            raise ValueError(f"点群に{axis} fieldがありません")
    datatypes = {offsets[axis][1] for axis in ("x", "y", "z")}
    if datatypes == {_FLOAT32}:
        format_code = "f"
    elif datatypes == {_FLOAT64}:
        format_code = "d"
    else:
        raise ValueError(f"x/y/zはfloat32かfloat64である必要があります: {datatypes}")

    return CloudLayout(
        frame_id=frame_id,
        point_count=width * height,
        point_step=point_step,
        data_start=data_start,
        data_length=data_length,
        x_offset=offsets["x"][0],
        y_offset=offsets["y"][0],
        z_offset=offsets["z"][0],
        format_code=format_code,
    )


def iter_points(payload: bytes) -> "list[tuple[float, float, float]]":
    """1メッセージ分の有限なx/y/zを返す。"""

    layout = parse_pointcloud2(payload)
    unpack = struct.Struct("<" + layout.format_code).unpack_from
    points: list[tuple[float, float, float]] = []
    base = layout.data_start
    for index in range(layout.point_count):
        record = base + index * layout.point_step
        if record + layout.point_step > len(payload):
            break  # 途中で切れたメッセージは、読めたところまでで打ち切る
        x = unpack(payload, record + layout.x_offset)[0]
        y = unpack(payload, record + layout.y_offset)[0]
        z = unpack(payload, record + layout.z_offset)[0]
        # NaN/Infは点群に混ざるのが普通なので黙って捨てる
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    return points


def find_bag_files(session_dir: Path) -> list[Path]:
    """セッション配下のdb3を記録順に返す。"""

    bag_dir = session_dir / "raw" / "rosbag2"
    if not bag_dir.is_dir():
        raise ValueError(f"rosbag2ディレクトリがありません: {bag_dir}")
    files = sorted(bag_dir.glob("*.db3"))
    if not files:
        raise ValueError(f"db3がありません: {bag_dir}")
    return files


def bag_topics(bag_path: Path) -> dict[str, str]:
    """db3に入っているトピック名 -> 型名。"""

    connection = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    try:
        rows = connection.execute("SELECT name, type FROM topics").fetchall()
    finally:
        connection.close()
    return {name: type_name for name, type_name in rows}


@dataclass
class RebuildResult:
    output_path: Path
    message_count: int
    raw_points: int
    written_points: int
    voxel_size: float
    minimum_xyz: tuple[float, float, float]
    maximum_xyz: tuple[float, float, float]


def rebuild_map(
    session_dir: Path,
    *,
    topic: str,
    output_path: Path,
    voxel_size: float,
    progress: "callable | None" = None,
) -> RebuildResult:
    """rosbagの点群を蓄積してPCDに書き出す。

    voxel_size > 0 のときはボクセル格子で間引く。同じボクセルに落ちた点は
    最初の1点だけを残す。平均を取るほうが滑らかになるが、蓄積中に全点を
    保持する必要が出てメモリが跳ねるため、こちらを既定にしている。
    """

    bag_files = find_bag_files(session_dir)
    available = bag_topics(bag_files[0])
    if topic not in available:
        raise ValueError(
            f"bagに{topic}がありません。含まれるトピック: {', '.join(sorted(available))}"
        )
    if available[topic] != "sensor_msgs/msg/PointCloud2":
        raise ValueError(f"{topic}はPointCloud2ではありません: {available[topic]}")

    voxels: dict[tuple[int, int, int], tuple[float, float, float]] = {}
    plain: list[tuple[float, float, float]] = []
    message_count = 0
    raw_points = 0
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]

    for bag_path in bag_files:
        connection = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
        try:
            topic_id = connection.execute(
                "SELECT id FROM topics WHERE name=?", (topic,)
            ).fetchone()
            if topic_id is None:
                continue
            cursor = connection.execute(
                "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (topic_id[0],),
            )
            for (payload,) in cursor:
                message_count += 1
                try:
                    points = iter_points(payload)
                except (ValueError, struct.error, IndexError):
                    # 壊れたメッセージが1つ混ざっても再構成全体は止めない
                    continue
                raw_points += len(points)
                for x, y, z in points:
                    for index, value in enumerate((x, y, z)):
                        if value < minimum[index]:
                            minimum[index] = value
                        if value > maximum[index]:
                            maximum[index] = value
                    if voxel_size > 0.0:
                        key = (
                            int(math.floor(x / voxel_size)),
                            int(math.floor(y / voxel_size)),
                            int(math.floor(z / voxel_size)),
                        )
                        if key not in voxels:
                            voxels[key] = (x, y, z)
                    else:
                        plain.append((x, y, z))
                if progress is not None and message_count % 200 == 0:
                    progress(message_count, raw_points, len(voxels) or len(plain))
        finally:
            connection.close()

    output = list(voxels.values()) if voxel_size > 0.0 else plain
    if not output:
        raise ValueError(f"{topic}から有効な点を取り出せませんでした")

    write_pcd(output_path, output)
    return RebuildResult(
        output_path=output_path,
        message_count=message_count,
        raw_points=raw_points,
        written_points=len(output),
        voxel_size=voxel_size,
        minimum_xyz=(minimum[0], minimum[1], minimum[2]),
        maximum_xyz=(maximum[0], maximum[1], maximum[2]),
    )


def write_pcd(path: Path, points: "list[tuple[float, float, float]]") -> None:
    """x/y/zのみのbinary PCDとして書き出す（pcd.pyのinspect_pcdが読める形式）。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    )
    packer = struct.Struct("<fff")
    with path.open("wb") as stream:
        stream.write(header.encode("ascii"))
        # 1点ずつwriteするとI/Oが支配的になるのでまとめて書く
        chunk: list[bytes] = []
        for x, y, z in points:
            chunk.append(packer.pack(x, y, z))
            if len(chunk) >= 65536:
                stream.write(b"".join(chunk))
                chunk.clear()
        if chunk:
            stream.write(b"".join(chunk))
