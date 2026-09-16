#!/usr/bin/env python3
"""姿勢つき PCD を `PointCloud2` ＋ TF として配信する。`octomap_server` の駆動役。

## なぜ要るか

段 2（`/projected_map` が種を保ったまま育つか）を**機体も測位器も無しで**測るため。
コンテナには `fast_lio` も `open3d_loc` も `glim` も無い（`ros2 pkg prefix` で確認）。

09-13 の実機記録には `/tf`（当時の測位結果）が入っているので姿勢源として使えるが、
**その姿勢は信用できない** —— 事前地図との重畳が click2/3/4 で 67.5 / 70.2 / 65.4 %
しかない。MOLA は黙って偽の極大に入ることがあり、そのときの重畳がちょうど 70 % 台に出る。
⇒ 「種が壊れた」のか「姿勢がずれていた」のかを**切り分けられない**。

一方 09-06 の姿勢つき PCD（`benchmark_s5`）は**その地図を作った当のスキャンと姿勢**なので、
定義上ずれていない。octomap_server の振る舞いだけを見たいときはこちらを使う。

## ⚠️ 回転を持たせない理由

`octomap_server` が点群から使うのは 2 つだけである（`insertCloudCallback`）:

- `cloud->header.frame_id` から `frame_id`（world）への変換 → 点を world に持ち上げる
- その変換の**並進**だけ → レイの始点（`sensorOrigin`）

つまり**姿勢の回転は要らない。**そこで
「並進だけの TF（`map -> <sensor_frame>`）＋ 点は原点を引いただけ」で配信する。
こうすると四元数の並び（`pcd_io` は PCL の qw 先頭、ROS は qx 先頭）を
取り違える余地が消える。⚠️ この配信物は**可視化用ではない**（点群の姿勢が寝ている）。

## 使い方（コンテナの中で。Mac に rclpy は無い）

    docker exec -u ubuntu rviz bash -lc '
      source /opt/ros/humble/setup.bash
      python3 /work/.../quickstart/publish_posed_pcd.py \\
        /work/.../runs/<id>/benchmark_s5/pcd --z-offset 1.247803 --stride 5 --rate 10'

`--z-offset` は配備済みの `floor0` 系（床が z≈0）に揃える量。
`octomap_seed_from_scans.py` の `FLOOR0_Z_OFFSET` と同じ値を渡すこと。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.pcd_io import read_pcd  # noqa: E402

# TF をこの前後の時刻にも打って、点群の時刻がちょうど補間の内側に来るようにする。
# ⚠️ 1 点だけだと lookupTransform が「その時刻の標本が無い」で落ちることがある
TF_BRACKET_S = 0.05


def cloud_message(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """(N,3) float64 を x/y/z float32 の PointCloud2 にする。"""
    data = points.astype(np.float32)
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(data)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(data)
    message.data = data.tobytes()
    message.is_dense = True
    return message


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("pcd_dir", type=Path, help="姿勢つき PCD の置き場")
    p.add_argument("--topic", default="/replay/cloud")
    p.add_argument("--world-frame", default="map")
    p.add_argument("--sensor-frame", default="replay_sensor")
    p.add_argument("--stride", type=int, default=1, help="何枚に 1 枚配信するか")
    p.add_argument("--limit", type=int, default=0, help="配信する枚数の上限（0 で全部）")
    p.add_argument("--rate", type=float, default=10.0, help="配信レート[Hz]")
    p.add_argument("--z-offset", type=float, default=0.0,
                   help="点とレイの始点を z にずらす量[m]（floor0 系に揃えるなら 1.247803）")
    a = p.parse_args()

    files = sorted(a.pcd_dir.glob("*.pcd"))[::a.stride]
    if a.limit:
        files = files[:a.limit]
    if not files:
        raise SystemExit(f"PCD がありません: {a.pcd_dir}")

    rclpy.init()
    node = Node("publish_posed_pcd")
    # ⚠️ octomap_server の cloud_in は既定 QoS（RELIABLE / depth 5）。
    # BEST_EFFORT で出すと互換だが、取りこぼすと「消えない」と誤診する
    publisher = node.create_publisher(
        PointCloud2, a.topic,
        QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))
    broadcaster = TransformBroadcaster(node)

    shift = np.array([0.0, 0.0, a.z_offset])
    period = 1.0 / a.rate
    node.get_logger().info(
        f"{len(files)} 枚を {a.rate} Hz で {a.topic} に出す（z を {a.z_offset:+.6f} m ずらす）")

    sent = 0
    for index, path in enumerate(files, start=1):
        data = read_pcd(path)
        if len(data.points) == 0:
            continue
        origin = data.origin + shift
        # 点は world 系のまま。原点を引いて「並進だけの TF」で戻せる形にする
        local = (data.points + shift) - origin

        now = node.get_clock().now()
        stamp = now.to_msg()
        for offset in (-TF_BRACKET_S, TF_BRACKET_S):
            transform = TransformStamped()
            transform.header.stamp = (
                now + rclpy.duration.Duration(seconds=offset)).to_msg()
            transform.header.frame_id = a.world_frame
            transform.child_frame_id = a.sensor_frame
            transform.transform.translation.x = float(origin[0])
            transform.transform.translation.y = float(origin[1])
            transform.transform.translation.z = float(origin[2])
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)

        publisher.publish(cloud_message(local, a.sensor_frame, stamp))
        sent += 1
        rclpy.spin_once(node, timeout_sec=0.0)
        if index % 50 == 0 or index == len(files):
            node.get_logger().info(f"  {index}/{len(files)} 枚")
        time.sleep(period)

    node.get_logger().info(f"{sent} 枚を出しおわった")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
