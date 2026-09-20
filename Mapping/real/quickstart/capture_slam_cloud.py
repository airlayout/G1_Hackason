"""稼働中の内蔵SLAM が出している地図点群を貯めて PCD に落とす。**PC2 で動かす。**

## 何に使うか

`1801` を投げた直後の SLAM は、**今いる場所を原点にした新しい地図**を作る。
一方こちらは過去の記録から作った地図（OctoMap 掃除済み）を持っている。
この 2 つを重ねる変換が分かれば、**Nav2 を過去の地図の座標系で走らせられる。**

その位置合わせの「動いている側」を取り出すのがこのスクリプト。
`/unitree/slam_mapping/points` は 1 回あたり約 1,000 点しか出ないので、
数十秒ぶん貯めないと ICP の入力にならない。

    ssh g1 'python3 ~/mapping_tools/capture_slam_cloud.py --seconds 40 --out ~/live_slam.pcd'

購読するだけで、ロボットには何も指令しない。
"""
import argparse
import struct
import sys
import threading
import time

sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_  # noqa: E402

TOPIC = "rt/unitree/slam_mapping/points"


def extract_xyz(msg):
    """PointCloud2 の先頭 3 フィールド（x y z, float32 前提）だけ取り出す。"""
    offsets = {f.name: f.offset for f in msg.fields}
    for name in ("x", "y", "z"):
        if name not in offsets:
            return []
    step = msg.point_step
    count = msg.width * msg.height
    data = bytes(msg.data)
    ox, oy, oz = offsets["x"], offsets["y"], offsets["z"]
    points = []
    for i in range(count):
        base = i * step
        if base + step > len(data):
            break
        x = struct.unpack_from("<f", data, base + ox)[0]
        y = struct.unpack_from("<f", data, base + oy)[0]
        z = struct.unpack_from("<f", data, base + oz)[0]
        # inf/nan は ICP を壊すので入口で落とす
        if x != x or y != y or z != z or abs(x) > 1e5 or abs(y) > 1e5 or abs(z) > 1e5:
            continue
        points.append((x, y, z))
    return points


def write_pcd(path, points):
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    ).format(n=len(points))
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        for x, y, z in points:
            handle.write(struct.pack("<fff", x, y, z))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--out", default="/home/unitree/live_slam.pcd")
    parser.add_argument("--iface", default="eth0")
    parser.add_argument("--domain-id", type=int, default=0)
    args = parser.parse_args()

    lock = threading.Lock()
    collected = []
    frames = [0]

    def handle(msg):
        pts = extract_xyz(msg)
        with lock:
            collected.extend(pts)
            frames[0] += 1

    ChannelFactoryInitialize(args.domain_id, args.iface)
    sub = ChannelSubscriber(TOPIC, PointCloud2_)
    sub.Init(handle, 10)

    print("{} を {:.0f} 秒ぶん貯めます ...".format(TOPIC, args.seconds), flush=True)
    began = time.time()
    while time.time() - began < args.seconds:
        time.sleep(2.0)
        with lock:
            print("  {:.0f}s  {} 枚 / {:,} 点".format(
                time.time() - began, frames[0], len(collected)), flush=True)

    with lock:
        points = list(collected)
    if not points:
        print("[NG] 1 点も受信していない。SLAM が動いているか確認すること", file=sys.stderr)
        return 1
    write_pcd(args.out, points)
    print("[OK] {:,} 点を {} に書きました（{} 枚）".format(len(points), args.out, frames[0]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
