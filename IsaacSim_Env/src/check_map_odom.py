#!/usr/bin/env python3
"""map -> odom が恒等変換になっていることを測定の前に確かめる。

これが無いと、TF の配信が黙って死んでいた場合に「着かなかった」が
測位のせいなのか配信漏れなのか分からなくなる。**測る前に前提を確かめる。**

合格の条件:
    - map -> odom が引ける
    - 並進が 1 mm 未満、回転が 0.1 度未満（＝恒等）
    - map -> base_link と /odom（真値）の差が 1 mm 未満
"""
from __future__ import annotations

import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

TRANS_TOL_M = 0.001
ROT_TOL_DEG = 0.1


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self) -> None:
        super().__init__("check_map_odom", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.odom: Odometry | None = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)

    def _on_odom(self, m) -> None:
        self.odom = m

    def spin_for(self, sec: float) -> None:
        import time
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def lookup(self, parent: str, child: str):
        for _ in range(15):
            try:
                return self.buf.lookup_transform(parent, child,
                                                 rclpy.time.Time())
            except Exception:
                self.spin_for(1.0)
        return None


def main() -> int:
    rclpy.init()
    node = Check()
    node.spin_for(5.0)

    mo = node.lookup("map", "odom")
    if mo is None:
        print("[NG] map -> odom が引けない。publish_map_odom_tf.py が動いているか")
        return 1
    t, q = mo.transform.translation, mo.transform.rotation
    d = math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
    a = abs(math.degrees(yaw_of(q)))
    print(f"[TF] map -> odom = ({t.x:+.6f}, {t.y:+.6f}, {t.z:+.6f}) "
          f"yaw={math.degrees(yaw_of(q)):+.4f} 度")
    ok = d < TRANS_TOL_M and a < ROT_TOL_DEG
    print(f"     恒等か: {'OK' if ok else 'NG'}（並進 {d * 1000:.3f} mm / 回転 {a:.4f} 度）")

    mb = node.lookup("map", "base_link")
    if mb is None or node.odom is None:
        print("[NG] map -> base_link または /odom が取れない")
        return 1
    bx, by = mb.transform.translation.x, mb.transform.translation.y
    ox = node.odom.pose.pose.position.x
    oy = node.odom.pose.pose.position.y
    gap = math.hypot(bx - ox, by - oy)
    print(f"[TF] map -> base_link = ({bx:+.3f}, {by:+.3f})  "
          f"真値 /odom = ({ox:+.3f}, {oy:+.3f})  差 {gap * 1000:.1f} mm")
    # 機体は歩いていなくても漂うので、TF と /odom は同時刻ではない。
    # 恒等が確かめられていれば差は取得タイミングの分だけ。10 mm まで許す。
    ok = ok and gap < 0.010
    print(f"== {'PASS' if ok else 'FAIL'} ==")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
