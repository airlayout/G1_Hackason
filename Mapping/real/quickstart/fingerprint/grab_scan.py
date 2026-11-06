#!/usr/bin/env python3
"""静止中の生の点群と IMU を数秒ためて 1 つの .npz に落とす（読むだけ。何も publish しない）。

指紋（UpDown-SC）の問い合わせと、合成スキャンの「光線の型」に使う。

    # PC2 で（jammy の python3.10。live/up.sh の姿勢ロガーと同じ起こし方）
    python3.10 grab_scan.py --seconds 3 --out ~/g1_runs/fp_query.npz

## 何を入れるか

| キー | 中身 |
|---|---|
| `points` | LiDAR 座標系の点 (N, 3)。**全フレームを重ねただけ**（静止が前提） |
| `accel` | IMU の加速度の平均 (3,)。単位は g。**上向きの向き**を決めるのに使う |
| `frames` | 重ねた点群の枚数 |

⚠️ **歩行中に録らない。** 重ねるだけなので動くと像が流れる。
⚠️ IMU の加速度は**重力の反作用＝上向き**を指す（静止で約 +1 g）。
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2

CLOUD_TOPIC = "/utlidar/cloud_livox_mid360"
IMU_TOPIC = "/utlidar/imu_livox_mid360"


def fields_xyz(msg: PointCloud2) -> np.ndarray:
    offsets = {f.name: f.offset for f in msg.fields}
    dtype = np.dtype({"names": ["x", "y", "z"],
                      "formats": ["<f4"] * 3,
                      "offsets": [offsets["x"], offsets["y"], offsets["z"]],
                      "itemsize": msg.point_step})
    flat = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    points = np.stack((flat["x"], flat["y"], flat["z"]), axis=1).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]


class Grabber(Node):
    def __init__(self):
        super().__init__("fp_grab_scan")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.clouds = []
        self.accels = []
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self.on_cloud, qos)
        self.create_subscription(Imu, IMU_TOPIC, self.on_imu, qos)

    def on_cloud(self, msg: PointCloud2) -> None:
        self.clouds.append(fields_xyz(msg))

    def on_imu(self, msg: Imu) -> None:
        a = msg.linear_acceleration
        self.accels.append((a.x, a.y, a.z))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Grabber()
    deadline = time.monotonic() + args.seconds
    # 最初の 1 枚が来るまでは数えない（購読の立ち上がりで秒数を食わない）
    wait_until = time.monotonic() + 10.0
    while not node.clouds and time.monotonic() < wait_until:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.clouds:
        print("[NG] 点群が来なかった（%s）" % CLOUD_TOPIC, flush=True)
        return 2
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()

    if len(node.accels) < 10:
        print("[NG] IMU が %d 件しか来なかった（%s）" % (len(node.accels), IMU_TOPIC), flush=True)
        return 2
    accels = np.asarray(node.accels)
    points = np.concatenate(node.clouds, axis=0)
    spread = float(np.linalg.norm(accels.std(axis=0)))
    np.savez_compressed(args.out, points=points, accel=accels.mean(axis=0),
                        frames=len(node.clouds))
    print("点群 %d 枚 / %d 点, IMU %d 件 (平均 %s g, ばらつき %.4f g) -> %s"
          % (len(node.clouds), len(points), len(accels),
             np.round(accels.mean(axis=0), 4).tolist(), spread, args.out), flush=True)
    if spread > 0.05:
        print("⚠️ IMU のばらつきが大きい。静止していないかもしれない", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
