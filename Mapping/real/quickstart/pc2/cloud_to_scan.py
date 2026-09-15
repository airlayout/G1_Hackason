#!/usr/bin/env python3
"""3D LiDAR の点群を **base_link 系の 2D LaserScan** に落とす（AMCL 用）。

    jrun $PREFIX/usr/bin/python3.10 cloud_to_scan.py [オプション]

`/utlidar/cloud_livox_mid360`（`sensor_msgs/PointCloud2`・`livox_frame`・約 10 Hz・
20064 点/枚・`point_step` 22）を購読し、`/scan`（`sensor_msgs/LaserScan`）を出す。

## なぜ自作なのか（2026-09-15 の判断）

`pointcloud_to_laserscan` の deb を入れる道もあるが、PC2 には**インターネットが無い**ので
Mac のコンテナで閉包を解決して tar で配り、さらに **`write_jammy_env.sh` を回して
71 個の ELF ラッパを作り直す**必要がある（`pc2/README.md` の罠 3・4）。
いま動いている MOLA と Nav2 がその env.sh に乗っているので、**動いている環境を
作り直す危険**を、100 行の numpy で置き換えられるなら置き換える方が安い。
加えて自作なら**出力の座標系を base_link にできる**（下の「なぜ base_link 系か」）。

## なぜ base_link 系で出すのか

`pointcloud_to_laserscan` は `target_frame` を tf2 で引くので、スキャンの frame_id を
`livox_frame` にすると AMCL が毎回 `base_link -> livox_frame` を引くことになる。
ここで**取付角（ロール 178°）を掛け忘れると落ちずに数字だけ悪くなる**。
このノードは**取付変換をノードの中で掛けてしまい**、`frame_id` を `base_link` にする。
AMCL 側の laser pose は恒等になり、取り違えようが無くなる。

## 取付値（live）

`base_link -> livox_frame` = `xyz (0, 0, 1.228)` / `rpy (177.93, 3.32, 0) deg`。
⚠️ **LiDAR は上下逆さま**に付いている。MID-360 の縦 FOV は -7°〜+52° なので、
逆さ付けでは**水平より上はおよそ 7° しか見えない**。
センサ高 1.228 m なので、帯 1.30〜1.80 m は「水平よりわずかに上」の細い円錐で拾う。
点数が足りないときは `--min-height` を下げる（地図の帯は床上 0.23〜1.80 m なので
1.80 m を超えない限り地図と矛盾しない。`pcd_to_occupancy.py` の `OBSTACLE_BAND`）。

## 帯（既定 1.30〜1.80 m）

床（z < 0.2）は必ず外す。反射で「外れ」を量産するため（`overlay-metric-counts-floor-as-miss`）。
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2

# live の取付値。run_mola_live.sh / nav_stack.sh と同じ値でなければならない
LIVOX_XYZ = (0.0, 0.0, 1.228)
LIVOX_RPY_DEG = (177.93, 3.32, 0.0)


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RPY[rad] -> 3x3。tf2 と同じ Rz(yaw) @ Ry(pitch) @ Rx(roll) の順。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def xyz_view(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 の生バイトから x,y,z だけを N x 3 で取り出す（コピー 1 回）。

    ⚠️ `point_step` は 22 で、`time` が offset 18 に居るため **4 バイト境界に乗らない**。
    構造化 dtype に `itemsize=point_step` を与えれば numpy がそのまま読める。
    """
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [0, 4, 8],
        "itemsize": msg.point_step,
    })
    n = msg.width * msg.height
    flat = np.frombuffer(msg.data, dtype=dtype, count=n)
    return np.stack((flat["x"], flat["y"], flat["z"]), axis=1).astype(np.float64)


class CloudToScan(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("cloud_to_scan")
        self._frame = args.frame
        self._lo, self._hi = args.min_height, args.max_height
        self._rmin, self._rmax = args.range_min, args.range_max
        self._bins = args.bins
        self._inc = 2.0 * math.pi / self._bins
        self._rot = rotation_from_rpy(*(math.radians(v) for v in args.livox_rpy_deg))
        self._trans = np.asarray(args.livox_xyz, dtype=np.float64)
        self._report_every = args.report_every

        self._count = 0
        # 直近 report_every 枚の統計。報告用（帯が薄すぎないかを親が判断できるように）
        self._stat_in_band = 0
        self._stat_bins = 0

        # ⚠️ AMCL は sensor_data QoS（BEST_EFFORT）で購読する。合わせる
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(LaserScan, args.scan_topic, sensor_qos)
        self.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, sensor_qos)
        self.get_logger().info(
            "[cloud_to_scan] {} -> {} / frame={} / 帯 {:.2f}〜{:.2f} m / "
            "{} bins ({:.2f} deg) / range {:.2f}〜{:.1f} m".format(
                args.cloud_topic, args.scan_topic, self._frame,
                self._lo, self._hi, self._bins, math.degrees(self._inc),
                self._rmin, self._rmax))
        self.get_logger().info(
            "[cloud_to_scan] 取付 xyz={} rpy={} deg（**上下逆さま**）".format(
                tuple(self._trans), tuple(args.livox_rpy_deg)))

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = xyz_view(msg)
        # NaN/Inf と原点そのものを落とす
        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]
        if pts.size == 0:
            return

        # livox_frame -> base_link（取付変換をここで掛ける。AMCL 側は恒等になる）
        base = pts @ self._rot.T + self._trans

        z = base[:, 2]
        band = (z >= self._lo) & (z <= self._hi)
        base = base[band]
        if base.size == 0:
            self._publish(msg, np.full(self._bins, np.inf), 0, 0)
            return

        x, y = base[:, 0], base[:, 1]
        r = np.hypot(x, y)
        ok = (r >= self._rmin) & (r <= self._rmax)
        x, y, r = x[ok], y[ok], r[ok]
        if r.size == 0:
            self._publish(msg, np.full(self._bins, np.inf), 0, 0)
            return

        idx = np.floor((np.arctan2(y, x) + math.pi) / self._inc).astype(np.int64)
        np.clip(idx, 0, self._bins - 1, out=idx)

        # 各ビンの最小距離。`np.minimum.at` は遅いので、距離の降順に書き込んで
        # 「最後に書いた＝最小」にする（20k 点で 1 ms 程度）
        order = np.argsort(r)[::-1]
        ranges = np.full(self._bins, np.inf, dtype=np.float64)
        ranges[idx[order]] = r[order]
        self._publish(msg, ranges, int(r.size), int(np.isfinite(ranges).sum()))

    def _publish(self, msg: PointCloud2, ranges: np.ndarray, n_band: int, n_bins: int) -> None:
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp        # 元の時刻をそのまま使う
        scan.header.frame_id = self._frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - self._inc
        scan.angle_increment = self._inc
        scan.time_increment = 0.0
        scan.scan_time = 0.1                        # LiDAR は 10 Hz
        scan.range_min = float(self._rmin)
        scan.range_max = float(self._rmax)
        # ⚠️ inf のままで良い（AMCL は range_max 以上を「当たり無し」として扱う）
        scan.ranges = ranges.astype(np.float32).tolist()
        self._pub.publish(scan)

        self._count += 1
        self._stat_in_band += n_band
        self._stat_bins += n_bins
        if self._count == 1 or self._count % self._report_every == 0:
            k = 1 if self._count == 1 else self._report_every
            self.get_logger().info(
                "[cloud_to_scan] {} 枚目: 帯の点 {:.0f} 点/枚・埋まったビン {:.0f}/{}".format(
                    self._count, self._stat_in_band / k, self._stat_bins / k, self._bins))
            self._stat_in_band = 0
            self._stat_bins = 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--cloud-topic", default="/utlidar/cloud_livox_mid360")
    p.add_argument("--scan-topic", default="/scan")
    p.add_argument("--frame", default="base_link",
                   help="出力の frame_id。既定 base_link（取付変換はノードが掛ける）")
    p.add_argument("--min-height", type=float, default=1.30,
                   help="帯の下限[m]（base_link 系＝床上）。床の反射を拾わせない")
    p.add_argument("--max-height", type=float, default=1.80,
                   help="帯の上限[m]。地図の OBSTACLE_BAND 上限と同じ 1.80 を超えないこと")
    p.add_argument("--range-min", type=float, default=0.40)
    p.add_argument("--range-max", type=float, default=20.0,
                   help="AMCL の laser_max_range と揃えること")
    p.add_argument("--bins", type=int, default=720, help="360° を何本に割るか（既定 0.5°）")
    p.add_argument("--livox-xyz", nargs=3, type=float, default=list(LIVOX_XYZ))
    p.add_argument("--livox-rpy-deg", nargs=3, type=float, default=list(LIVOX_RPY_DEG))
    p.add_argument("--report-every", type=int, default=100, help="何枚ごとに統計を出すか")
    args = p.parse_args(argv)

    if args.min_height < 0.2:
        p.error("--min-height は 0.2 以上にすること（床の反射を拾う）")
    if args.max_height <= args.min_height:
        p.error("--max-height は --min-height より大きいこと")

    rclpy.init()
    node = CloudToScan(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[cloud_to_scan] 停止します")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
