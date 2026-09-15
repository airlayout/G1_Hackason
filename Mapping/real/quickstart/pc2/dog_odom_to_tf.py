#!/usr/bin/env python3
"""純正の脚 odometry `/dog_odom` を **`odom -> base_link` の TF** にして流す（AMCL 用）。

    jrun $PREFIX/usr/bin/python3.10 dog_odom_to_tf.py [オプション]

## なぜ要るのか

AMCL は REP-105 の `map -> odom -> base_link` を前提にしており、
**`odom -> base_link` は外から与えられている**ことを要求する（AMCL が出すのは
`map -> odom` だけ）。G1 は `/tf` も `/tf_static` も一切出さないので、
誰かが `/dog_odom` を TF に詰め替える必要がある。

## `/dog_odom` の実測（2026-09-15・機体は立位で静止）

    型         nav_msgs/Odometry
    頻度       約 1008 Hz、publisher 1
    QoS        RELIABLE / KEEP_LAST(1) / VOLATILE
    frame_id   odom
    child      robot_center          ← **base_link ではない**
    position   x 0.215  y -0.221  z 0.734   ← z は胴体の高さ

## 2 つの詰め替え

1. **`child_frame_id` を `base_link` に付け替える。** 機体が名乗るのは `robot_center`。
2. ⚠️ **2D に潰す（z=0・roll=pitch=0・yaw だけ残す）。**
   `base_link` は**水平・床面**で定義し直す決まりである
   （`g1-base-link-must-be-level-on-floor`）。`cloud_to_scan.py` の取付値
   `xyz (0,0,1.228)` はその定義に乗っているので、ここで胴体の z 0.734 や
   歩行中の roll/pitch をそのまま流すと、LiDAR の高さと傾きが二重に入る。
   AMCL 自体は 2D しか見ないので潰しても測位は劣化しない。
   `--no-flatten` で切れるが、**既定のまま使うこと**。

## 頻度

1008 Hz をそのまま TF に流すと無駄なので既定 100 Hz に間引く。
⚠️ **下げすぎないこと。** tf2 は未来に外挿しないので、LiDAR の打刻が最新の TF より
新しいと AMCL の lookup が落ちる。100 Hz なら最悪 10 ms の遅れで済む。

## ⚠️ 素直に書くと 1 コアの 66% を食う（2026-09-15 実測）。手当ては 2 つ

**(1) `raw=True` で受ける。** 型付きの `nav_msgs/Odometry` は共分散 36 要素 × 2 を
numpy 配列に起こすので 1 件が重い。CDR のバイト列のまま受け、`struct` で要る 9 個だけ
取り出す（同じ 1 件で型付きと**全桁一致**することを確認済み。`--typed` で戻せる）。
実測 66.5% → 50.6%。

**(2) executor を `--rate` で歩調を取る。** これが効く。`rclpy.spin()` は
届いた分だけ take するので、コールバックの中で間引いても **1008 回/秒の take は消えない**。
`spin_once` を 1/rate 秒ごとに 1 回だけ呼べば take も 100 回/秒になる。
落ちた分は購読側の履歴が `KEEP_LAST(1)` なので**常に最新だけが残る**（古いものが捨てられる）。
実測 50.6% → **21.5%**（既定 100 Hz。うち起動の import が数秒ぶん入っているので
定常はおよそ 19%）。受信件数も 1008/秒 → 107/秒 に落ちている。

CDR の並び（XCDR1・先頭 4 バイトは encapsulation ヘッダで、整列はその**後ろ**が基点）:

    off 4    int32  sec          off 8   uint32 nanosec
    off 12   uint32 frame_id の長さ → その分だけ飛ばす → 4 整列
             uint32 child_frame_id の長さ → 同上 → **8 整列**
             float64 x 7（position 3 ＋ orientation 4）
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

# live の取付値。cloud_to_scan.py / run_mola_live.sh と同じ値でなければならない
LIVOX_XYZ = (0.0, 0.0, 1.228)
LIVOX_RPY_DEG = (177.93, 3.32, 0.0)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    """RPY[rad] -> (x, y, z, w)。tf2 と同じ Rz(yaw) Ry(pitch) Rx(roll) の順。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """四元数 -> yaw[rad]。roll/pitch は捨てる。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parse_odom_cdr(buf: bytes):
    """CDR のバイト列から (sec, nanosec, x, y, z, qx, qy, qz, qw) だけ取り出す。

    ⚠️ 整列は encapsulation ヘッダ（先頭 4 バイト）の**後ろ**を基点に数える。
    """
    e = "<" if buf[1] in (1, 3) else ">"          # 1/3 = little endian
    sec, nanosec = struct.unpack_from(e + "iI", buf, 4)
    off = 12
    (n,) = struct.unpack_from(e + "I", buf, off)
    off += 4 + n
    off = 4 + ((off - 4 + 3) & ~3)                # 4 整列（child_frame_id の長さ）
    (n,) = struct.unpack_from(e + "I", buf, off)
    off += 4 + n
    off = 4 + ((off - 4 + 7) & ~7)                # 8 整列（float64 の並び）
    return (sec, nanosec) + struct.unpack_from(e + "7d", buf, off)


class DogOdomToTf(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("dog_odom_to_tf")
        self._odom_frame = args.odom_frame
        self._base_frame = args.base_frame
        self._flatten = not args.no_flatten
        # ⚠️ 歩調は executor 側（`spin_paced`）が取る。ここは念のための保険なので
        # 0.8 倍に緩める。同じ値にすると揺らぎで正当な 1 件を落として頻度が半減する
        self._min_period = 0.8 / args.rate if args.rate > 0 else 0.0
        self._last_sec = -1.0
        self._count = 0
        self._published = 0

        self._tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self._publish_static_livox(args.livox_xyz, [math.radians(v) for v in args.livox_rpy_deg])

        # /dog_odom は RELIABLE で出ている。BEST_EFFORT の購読でも繋がる（互換）
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._raw = not args.typed
        if self._raw:
            self.create_subscription(Odometry, args.topic, self._on_raw, qos, raw=True)
        else:
            self.create_subscription(Odometry, args.topic, self._on_typed, qos)
        self.get_logger().info(
            "[dog_odom_to_tf] {} -> TF {} -> {} / {:.0f} Hz に間引き / 2D に潰す={} / raw={}".format(
                args.topic, self._odom_frame, self._base_frame, args.rate,
                self._flatten, self._raw))

    def _publish_static_livox(self, xyz, rpy) -> None:
        """base_link -> livox_frame。⚠️ transient_local でないと後から来た購読者に届かない。"""
        static_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._base_frame
        t.child_frame_id = "livox_frame"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        qx, qy, qz, qw = quaternion_from_rpy(*rpy)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        pub.publish(TFMessage(transforms=[t]))
        self._static_pub = pub          # GC で消えないよう保持する
        self.get_logger().info(
            "[dog_odom_to_tf] 静的 TF {} -> livox_frame xyz={} rpy(deg)={}".format(
                self._base_frame, tuple(xyz), tuple(math.degrees(v) for v in rpy)))

    def _on_raw(self, buf) -> None:
        """CDR のまま受ける既定の経路。⚠️ 間引きの判定を **parse より前**に置くこと。"""
        self._count += 1
        # 打刻だけ先に読む（残りは通すと決まってから）
        sec_i, nsec = struct.unpack_from("<iI", buf, 4)
        sec = sec_i + nsec * 1e-9
        if self._min_period > 0.0 and (sec - self._last_sec) < self._min_period:
            return
        self._last_sec = sec
        _, _, px, py, pz, qx, qy, qz, qw = parse_odom_cdr(bytes(buf))
        self._emit(sec_i, nsec, px, py, pz, qx, qy, qz, qw)

    def _on_typed(self, msg: Odometry) -> None:
        """`--typed` のときだけ使う予備の経路（1 コアの 66% を食う。冒頭の注記）。"""
        self._count += 1
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._min_period > 0.0 and (sec - self._last_sec) < self._min_period:
            return
        self._last_sec = sec
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._emit(msg.header.stamp.sec, msg.header.stamp.nanosec,
                   p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def _emit(self, sec: int, nanosec: int,
              px: float, py: float, pz: float,
              qx: float, qy: float, qz: float, qw: float) -> None:
        t = TransformStamped()
        t.header.stamp.sec = sec                   # 元の時刻をそのまま使う
        t.header.stamp.nanosec = nanosec
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame        # ⚠️ robot_center から付け替える
        t.transform.translation.x = px
        t.transform.translation.y = py
        if self._flatten:
            t.transform.translation.z = 0.0
            qx, qy, qz, qw = quaternion_from_rpy(
                0.0, 0.0, yaw_from_quaternion(qx, qy, qz, qw))
        else:
            t.transform.translation.z = pz
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self._tf_pub.publish(TFMessage(transforms=[t]))

        self._published += 1
        if self._published in (1, 10) or self._published % 500 == 0:
            self.get_logger().info(
                "[dog_odom_to_tf] {} 件配信（受信 {} 件）最新 x={:.3f} y={:.3f} yaw={:.1f} deg".format(
                    self._published, self._count,
                    t.transform.translation.x, t.transform.translation.y,
                    math.degrees(yaw_from_quaternion(
                        t.transform.rotation.x, t.transform.rotation.y,
                        t.transform.rotation.z, t.transform.rotation.w))))


def spin_paced(node: Node, rate: float) -> None:
    """**1/rate 秒に 1 回だけ** `spin_once` する。

    `rclpy.spin()` だと届いた 1008 件/秒を全部 take してしまい、コールバックの中で
    間引いても CPU は減らない。ここで歩調を取れば take も rate 回/秒になり、
    取りこぼした分は購読側の `KEEP_LAST(1)` が捨てるので**常に最新**が残る。
    """
    if rate <= 0:
        rclpy.spin(node)
        return
    period = 1.0 / rate
    while rclpy.ok():
        started = time.monotonic()
        rclpy.spin_once(node, timeout_sec=period)
        rest = period - (time.monotonic() - started)
        if rest > 0.0:
            time.sleep(rest)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topic", default="/dog_odom")
    p.add_argument("--odom-frame", default="odom")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--rate", type=float, default=100.0, help="TF を出す上限[Hz]。0 で間引かない")
    p.add_argument("--no-flatten", action="store_true",
                   help="2D に潰さず z と roll/pitch をそのまま流す。**通常は使わない**")
    p.add_argument("--typed", action="store_true",
                   help="CDR を自前で読まず型付きで購読する。**1 コアの 66% を食う**ので"
                        "CDR の読み出しを疑うときだけ使う")
    p.add_argument("--livox-xyz", nargs=3, type=float, default=list(LIVOX_XYZ))
    p.add_argument("--livox-rpy-deg", nargs=3, type=float, default=list(LIVOX_RPY_DEG))
    # ⚠️ **記録の再生で測るときは必ず付けること。** TF の打刻は `get_clock().now()` で
    # 打っている（`/dog_odom` の打刻ではない）ので、既定の壁時計のままだと
    # 2026-09-13 の打刻を持つ `/scan` と**2 日ずれる**。AMCL は lookup に失敗し、
    # `map -> odom` を 1 度も出さないまま「動いているように見える」（落ちない）。
    p.add_argument("--use-sim-time", action="store_true",
                   help="`/clock`（`ros2 bag play --clock`）に乗る。再生で測るときは必須")
    args = p.parse_args(argv)

    rclpy.init(args=[sys.argv[0], "--ros-args", "-p", "use_sim_time:=true"]
               if args.use_sim_time else None)
    node = DogOdomToTf(args)
    try:
        spin_paced(node, args.rate)
    except KeyboardInterrupt:
        node.get_logger().info("[dog_odom_to_tf] 停止します")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
