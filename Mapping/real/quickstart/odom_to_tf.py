#!/usr/bin/env python3
"""G1 の Odometry を TF に変換して流す。Humble 環境（~/g1_humble）で動かす。

G1 は /tf も /tf_static も配信していないため、Foxglove/Lichtblick の 3D パネルは
座標系どうしの関係を知ることができない。その結果:

  - `map` 座標系の /unitree/slam_mapping/points は表示できるが
  - `livox_frame` 座標系の /utlidar/cloud_livox_mid360 と重ねられない
  - G1 が地図上のどこに居るのかも描けない

一方 /unitree/slam_mapping/odom (nav_msgs/Odometry) は
`frame_id=map` / `child_frame_id=base_link` で自己位置を出しており、
これは TF の transform と同じ情報である。詰め替えて /tf に流せば解決する。

    ssh g1 'bash ~/mapping_tools/start_odom_tf.sh'

## base_link -> livox_frame について

LiDAR の取付オフセットは**実測値が手元に無い**（リポジトリにもPC2にもURDFや
extrinsicの記載が無いことを2026-09-03に確認）。推測値を入れると点群が静かに
ずれた場所に描かれ、誤りに気づけないので、**既定では流さない**。
実測が得られたら --livox-xyz / --livox-rpy で与えること。
なおSimEnv3Dの LIDAR_OFFSET_Z はシミュレータ用の仮定値であり、実機の値ではない。
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    """RPY[rad] -> (x, y, z, w)。静的変換の指定用。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class OdomToTf(Node):
    def __init__(self, topic: str, livox_xyz, livox_rpy,
                 nav2_frames: bool = False, publish_map_odom: bool = True) -> None:
        super().__init__("odom_to_tf")
        # G1側の配信QoSは不明。BEST_EFFORTで購読すればRELIABLEな相手とも繋がる
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.create_subscription(Odometry, topic, self._on_odom, qos)

        self._count = 0
        self._warned_no_child = False
        self.get_logger().info("[odom_to_tf] {} を購読して /tf へ流します".format(topic))

        self._nav2_frames = nav2_frames
        if nav2_frames and publish_map_odom:
            self._publish_static_identity("map", "odom")
            self.get_logger().info(
                "[odom_to_tf] Nav2 用: map -> odom を恒等にし、odom -> base_link を流します")
        elif nav2_frames:
            self.get_logger().info(
                "[odom_to_tf] Nav2 用: odom -> base_link のみ流します"
                "（map -> odom は別途 align_to_map.py の結果を流すこと）")

        if livox_xyz is not None:
            self._publish_static_livox(livox_xyz, livox_rpy)
        else:
            self.get_logger().info(
                "[odom_to_tf] base_link->livox_frame は流しません"
                "（実測値が無いため。--livox-xyz で指定可）")

    def _publish_static_identity(self, parent: str, child: str) -> None:
        """恒等の静的変換。map -> odom のように、ずれが無い関係を繋ぐのに使う。"""
        static_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.rotation.w = 1.0
        pub.publish(TFMessage(transforms=[t]))
        self._identity_pub = pub        # GCで消えないよう保持する

    def _publish_static_livox(self, xyz, rpy) -> None:
        # transient_local にしないと、後から繋いだ購読者が静的変換を受け取れない
        static_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "livox_frame"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        qx, qy, qz, qw = quaternion_from_rpy(*rpy)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        pub.publish(TFMessage(transforms=[t]))
        self._static_pub = pub          # GCで消えないよう保持する
        self.get_logger().info(
            "[odom_to_tf] base_link->livox_frame を静的変換として流しました xyz={} rpy={}"
            .format(xyz, rpy))

    def _on_odom(self, msg: Odometry) -> None:
        child = msg.child_frame_id
        if not child:
            if not self._warned_no_child:
                self.get_logger().warn(
                    "[odom_to_tf] child_frame_id が空です。TFを組めないので捨てます")
                self._warned_no_child = True
            return

        t = TransformStamped()
        t.header.stamp = msg.header.stamp          # 元の時刻をそのまま使う
        # Nav2 は map -> odom -> base_link を要求する。G1 の odom は map -> base_link を
        # 直接出しているので、親を odom に付け替えて REP-105 の形に合わせる
        t.header.frame_id = "odom" if self._nav2_frames else msg.header.frame_id
        t.child_frame_id = child
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_pub.publish(TFMessage(transforms=[t]))

        self._count += 1
        if self._count in (1, 10) or self._count % 200 == 0:
            self.get_logger().info(
                "[odom_to_tf] {} -> {} を {} 件配信 (最新 x={:.3f} y={:.3f} z={:.3f})".format(
                    t.header.frame_id, child, self._count,
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z))


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topic", default="/unitree/slam_mapping/odom")
    p.add_argument("--livox-xyz", nargs=3, type=float, metavar=("X", "Y", "Z"),
                   default=None, help="base_link->livox_frame の並進[m]。**実測値のみ**")
    p.add_argument("--livox-rpy", nargs=3, type=float, metavar=("R", "P", "Y"),
                   default=None, help="同回転[**ラジアン**]。度で渡すなら --livox-rpy-deg")
    p.add_argument("--livox-rpy-deg", nargs=3, type=float, metavar=("R", "P", "Y"),
                   default=None,
                   help="同回転[**度**]。実測値は repo 全体で度で記録されているので"
                        "（例 178.35 -8.41 -0.72）、通常はこちらを使う")
    p.add_argument("--nav2-frames", action="store_true",
                   help="Nav2 が要求する map -> odom -> base_link の形で流す。"
                        "G1 の odom は map -> base_link を直接出すので、"
                        "そのままでは REP-105 に合わない。"
                        "map -> odom を恒等の静的変換とし、odom -> base_link として流す")
    p.add_argument("--no-map-odom", action="store_true",
                   help="map -> odom を流さない。**過去の地図の座標系で走らせるとき**に使う。"
                        "その変換は align_to_map.py が求め、static_transform_publisher が流すので、"
                        "こちらが恒等を流すと衝突する")
    args = p.parse_args(argv)

    if args.livox_rpy is not None and args.livox_rpy_deg is not None:
        p.error("--livox-rpy と --livox-rpy-deg は同時に指定できない")
    if args.livox_rpy_deg is not None:
        rpy = [math.radians(v) for v in args.livox_rpy_deg]
    else:
        rpy = args.livox_rpy if args.livox_rpy is not None else [0.0, 0.0, 0.0]

    rclpy.init()
    node = OdomToTf(args.topic, args.livox_xyz, rpy,
                    args.nav2_frames, not args.no_map_odom)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[odom_to_tf] 停止します")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
