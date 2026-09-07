#!/usr/bin/env python3
"""/tf を合成して map -> <frame> の姿勢を TUM 形式で書き出す。

段 3（ROS 上で MOLA-LO が出す map->odom）を段 4 と同じ物差しで測るために使う。
オフラインの `--output-tum-path` と同じ形式で出るので、eval_mola_traj.py に
そのまま渡せる。

  ros2 run 相当:
    python3 log_tf_pose.py --target livox_frame --out /tmp/tf_pose.txt --use-sim-time

⚠️ tf2 の lookup は「その時刻の変換」を返すので、**LiDAR のスキャン時刻で引く**。
今の時刻で引くと、map->odom の更新レート（10 Hz）と odom->base_link の更新レート
（10 Hz）のずれが誤差として混ざる。
"""
import argparse
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException


class TfPoseLogger(Node):
    """LiDAR のスキャン時刻で /tf を引き、TUM 形式で落とす。

    ⚠️ **スキャン時刻ちょうどでは引けない。** その時刻の変換は、スキャンを受けた
    ノード（MOLA-LO）が処理を終えてから publish されるので、こちらがスキャンを
    受け取った瞬間には /tf にまだ入っていない（未来側の外挿になって失敗する）。
    2026-09-07 に実機で 249 回連続で失敗して気づいた。
    そこで**スキャン時刻を貯めておき、lag 秒だけ遅れて引く**。
    """

    def __init__(self, target: str, source: str, out_path: str, topic: str, lag: float) -> None:
        super().__init__("tf_pose_logger")
        self.target, self.source = target, source
        self.lag = lag
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.fh = open(out_path, "w", buffering=1)
        self.fh.write("# 記録時刻 tx ty tz qx qy qz qw\n")
        self.n_ok = 0
        self.n_fail = 0
        self.pending: deque = deque(maxlen=200)
        self.create_subscription(PointCloud2, topic, self.on_scan, 10)
        self.create_timer(0.02, self.drain)
        self.create_timer(5.0, self.report)

    def on_scan(self, msg: PointCloud2) -> None:
        self.pending.append(Time.from_msg(msg.header.stamp))

    def drain(self) -> None:
        now = self.get_clock().now()
        lag_ns = int(self.lag * 1e9)
        while self.pending:
            t = self.pending[0]
            if (now - t).nanoseconds < lag_ns:
                break                      # まだ若い。次の周期に回す
            self.pending.popleft()
            try:
                tr = self.buf.lookup_transform(self.target, self.source, t)
            except TransformException:
                self.n_fail += 1
                continue
            v, q = tr.transform.translation, tr.transform.rotation
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            self.fh.write(
                f"{stamp:.6f} {v.x:.6f} {v.y:.6f} {v.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"
            )
            self.n_ok += 1

    def report(self) -> None:
        self.get_logger().info(
            f"{self.target} -> {self.source}: 取れた {self.n_ok} / 取れない {self.n_fail}"
            f" / 待ち {len(self.pending)}"
        )


def main(argv=None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--target", default="map", help="親フレーム")
    p.add_argument("--source", default="livox_frame", help="子フレーム（真値と同じ livox_frame が既定）")
    p.add_argument("--out", required=True)
    p.add_argument("--topic", default="/utlidar/cloud_livox_mid360",
                   help="この topic のヘッダ時刻で /tf を引く")
    p.add_argument("--lag", type=float, default=0.3,
                   help="スキャン時刻から何秒遅れて /tf を引くか[s]。"
                        "publish されるまで待つために要る（既定 0.3）")
    args, ros_args = p.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = TfPoseLogger(args.target, args.source, args.out, args.topic, args.lag)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.fh.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
