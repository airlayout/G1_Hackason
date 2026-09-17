#!/usr/bin/env python3
"""生 LiDAR から**機体自身の点**を落として出し直す。`octomap_server` の入口。

    jrun $PREFIX/usr/bin/python3.10 filter_self_returns.py [オプション]

## なぜ要るか（2026-09-17 に実機で踏んだ）

`octomap_server` を生 LiDAR に繋ぐと、**頭の LiDAR が見下ろした自分の胴体を
占有として焼き込む**。しかも**自分の体は常に自分を遮るので光線が通らず、永久に消えない**。
40 分立っていた機体では `/projected_map` の最近傍の占有セルが **0.07 m**、
**0.30 m 以内に 8 セル**あり、`robot_radius: 0.30` の足元が致死セルを含んで

    Smac2D: failed to create plan, invalid use: Starting point in lethal space!

となり、**planner が経路を 1 本も作れなかった**（Spin と Wait のリカバリが走り、
global costmap を 2 回全消去しても復帰せず Goal failed）。

| 地図 | 機体までの最短 | 0.30 m 以内 |
|---|---|---|
| `nav_map_run`（検証済み） | 0.66 m | 0 |
| 種の投影（育つ前） | 0.66 m | 0 |
| **ライブで育てた `/projected_map`** | **0.07 m** | **8** |

⇒ **種は綺麗で、自己マーキングはライブで育てる過程で入る。**

⚠️ Nav2 の `voxel_layer` には `obstacle_min_range: 0.30` があり、まさにこれを
防いでいる。**`octomap_server` には半径の下限パラメータが無い**のでここで落とす。

## どう落とすか — センサ軸まわりの**円柱**

LiDAR は**上下逆さま**に付いている（`base_link -> livox_frame` の rpy が
`(-179.401, -1.9437, 0.0321)`）。つまり**センサ系の z 軸はほぼ鉛直**なので、
センサ系で `sqrt(x^2+y^2) < R` を落とせば「機体を囲む鉛直な円柱」を落とせる。
球（`sqrt(x^2+y^2+z^2) < R`）ではいけない —— 機体の胴はセンサから 0.6 m 下まで
続くので、球で消すには R を大きく取るしかなく、その半径の**実在の障害物**も消える。

⚠️ 取付の pitch 1.9° ぶん（1.2 m 先で 4 cm）は無視している。R の 1/8 なので効かない。

⚠️ **円柱の中のセルは「空き」になる。**点を落としてもレイは通るので、
`octomap_server` は円柱を通り抜ける光線で**そこを空きに投票する**。
つまり機体の足元は未知ではなく空きになり、planner の起点として正しく使える。

## レートも落とせる（CPU の手当てを兼ねる）

`octomap_server` は実機で **CPU 71 %** だった（点群 10 Hz で 335k セルを毎回再投影）。
`--max-hz 2` にすると入力が 1/5 になる。オフラインでの実測は 10 Hz 13.5 % → 2 Hz 2.0 %。
幽霊が消えるのに要る「9 回の miss」は 2 Hz なら 4.5 秒なので、
段 6 の合格線（60 秒）には十分間に合う。

## 使い方

    # PC2 で（run_nav2_live.sh が起こす）
    jrun $PREFIX/usr/bin/python3.10 filter_self_returns.py --radius 0.35 --max-hz 2

出力は `/utlidar/cloud_livox_mid360_noself`（frame_id は入力のまま `livox_frame`）。
`octomap_server` の `cloud_in` をこちらに向ける。
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import PointCloud2

IN_TOPIC = "/utlidar/cloud_livox_mid360"
OUT_TOPIC = "/utlidar/cloud_livox_mid360_noself"
# 機体を囲む円柱の半径[m]。G1 の肩幅は約 0.45 m（半幅 0.22 m）で、
# 背中のバックパックとアームの振りを足して 0.35 を既定にした。
# ⚠️ 大きくすると実在の障害物も消える。`voxel_layer` の obstacle_min_range 0.30 と
# 同程度に保つこと（あちらはセンサからの球なので厳密には別物）
DEFAULT_RADIUS = 0.35


def xyz_view(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 の x/y/z を (N,3) float32 で見る（コピーしない）。

    ⚠️ Livox の `point_step` は 22 で `time` が offset 18 に居るため
    **4 バイト境界に乗らない**。構造化 dtype に `itemsize=point_step` を
    与えれば numpy がそのまま読める（`cloud_to_scan.py` と同じ手）。
    """
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [np.float32] * 3,
        "offsets": [0, 4, 8],
        "itemsize": msg.point_step,
    })
    raw = np.frombuffer(msg.data, dtype=dtype)
    return np.stack([raw["x"], raw["y"], raw["z"]], axis=1)


class SelfFilter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("filter_self_returns")
        self._radius2 = args.radius * args.radius
        self._period = (1.0 / args.max_hz) if args.max_hz > 0 else 0.0
        self._last = 0.0
        self._point_step = None
        self._seen = 0
        self._sent = 0
        self._dropped_points = 0
        self._total_points = 0
        # ⚠️ 生 LiDAR は純正 SDK の bare DDS app が BEST_EFFORT で出す。合わせる
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(PointCloud2, args.out_topic, sensor_qos)
        self.create_subscription(PointCloud2, args.in_topic, self._on_cloud, sensor_qos)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            "[filter] {} -> {} / 円柱 半径 {:.2f} m を落とす / {}".format(
                args.in_topic, args.out_topic, args.radius,
                "{:.1f} Hz に間引く".format(args.max_hz) if args.max_hz > 0 else "間引かない"))

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._seen += 1
        now = time.monotonic()
        if self._period and now - self._last < self._period:
            return
        # ⚠️ 記録に混ざる異物を弾く（frame_id='map' / point_step=48 の SLAM 点群が
        # 生 LiDAR のトピックに書かれている例がある。2026-09-07 に 5900 件中 229 件）
        if msg.point_step != 22 or msg.header.frame_id != "livox_frame":
            return
        self._last = now

        points = xyz_view(msg)
        # センサ系の z 軸はほぼ鉛直（逆さ付け）。xy だけで円柱になる
        keep = (points[:, 0] ** 2 + points[:, 1] ** 2) >= self._radius2
        self._total_points += len(points)
        self._dropped_points += int((~keep).sum())

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(keep.sum())
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * out.width
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
        out.data = rows[keep].tobytes()
        out.is_dense = msg.is_dense
        self._pub.publish(out)
        self._sent += 1

    def _report(self) -> None:
        if self._seen == 0:
            self.get_logger().warn("[filter] 入力が来ていない")
            return
        share = 100.0 * self._dropped_points / max(self._total_points, 1)
        self.get_logger().info(
            "[filter] 10 秒: 受け {} 枚 / 出し {} 枚 / 落とした点 {:.1f} %".format(
                self._seen, self._sent, share))
        self._seen = self._sent = 0
        self._dropped_points = self._total_points = 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--in-topic", default=IN_TOPIC)
    p.add_argument("--out-topic", default=OUT_TOPIC)
    p.add_argument("--radius", type=float, default=DEFAULT_RADIUS,
                   help="機体を囲む円柱の半径[m]。この中の点を落とす"
                        f"（既定 {DEFAULT_RADIUS}）")
    p.add_argument("--max-hz", type=float, default=0.0,
                   help="出力のレート上限[Hz]。0 で間引かない。"
                        "octomap_server の CPU を下げるなら 2 前後")
    args = p.parse_args(argv)

    rclpy.init()
    node = SelfFilter(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
