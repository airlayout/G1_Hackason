#!/usr/bin/env python3
"""Livox IMU の**壊れた姿勢クォータニオン**を直して中継する。

    python3 imu_orientation_fix.py --in /utlidar/imu_livox_mid360 --out /imu_fixed

## なぜ要るか（2026-09-08 実測）

G1 の `/utlidar/imu_livox_mid360` は `orientation` に **(0, 0, 0, 0)** を載せる。
Livox の IMU は姿勢を出さないので当然だが、ROS の慣習では
「姿勢が無いなら `orientation_covariance[0] = -1` を立てる」であって**全ゼロではない**。

MOLA の `InitLocalization::PitchAndRollFromIMU` を使うと、この値がそのまま
MRPT の `CQuaternion` に渡され、

    Initialization data for quaternion is not normalized: 0 0 0 0 -> sqrNorm=0
    at mola::imu::ImuInitialCalibrator::add(...)

で例外になり、**MOLA が致命停止する**（"Discarding incoming observations:
a fatal error ocurred above." 以降、測位が一切出ない）。
`InitLocalization::FixedPose` では姿勢を読まないので、この地雷は踏まない。

## 何を直すか

クォータニオンのノルムが 0 に近いときだけ**単位クォータニオン**に差し替え、
`orientation_covariance[0] = -1`（＝姿勢は使えない、の ROS の印）を立てる。
**加速度と角速度は触らない。** pitch/roll の推定は加速度（重力方向）から行われるので、
差し替えた単位クォータニオンが推定に混ざらないことは実測で確かめること
（`mola.log` の "Initial re-localization done with pose" の pitch/roll を見る）。

⚠️ **これは「値を作る」ものではない。** 姿勢は元から無いので、
単位クォータニオンは「無い」を表す入れ物にすぎない。
ここで実際の姿勢を推定して入れてはいけない（それは MOLA の仕事）。
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

DEGENERATE_NORM = 1e-6


class ImuFix(Node):
    def __init__(self, topic_in: str, topic_out: str, sim_time: bool) -> None:
        super().__init__(
            "imu_orientation_fix",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, sim_time)],
        )
        self.n_seen = 0
        self.n_fixed = 0
        self.pub = self.create_publisher(Imu, topic_out, qos_profile_sensor_data)
        self.create_subscription(Imu, topic_in, self._on_imu, qos_profile_sensor_data)
        self.create_timer(10.0, self._report)
        self.get_logger().info(f"[imu_fix] {topic_in} -> {topic_out}")

    def _on_imu(self, msg: Imu) -> None:
        self.n_seen += 1
        q = msg.orientation
        norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm2 < DEGENERATE_NORM:
            # 元のメッセージを書き換えず、姿勢だけ差し替えた別のメッセージを出す
            out = Imu()
            out.header = msg.header
            out.angular_velocity = msg.angular_velocity
            out.angular_velocity_covariance = msg.angular_velocity_covariance
            out.linear_acceleration = msg.linear_acceleration
            out.linear_acceleration_covariance = msg.linear_acceleration_covariance
            out.orientation.w = 1.0
            out.orientation_covariance = list(msg.orientation_covariance)
            out.orientation_covariance[0] = -1.0   # ROS の「姿勢は使えない」の印
            self.n_fixed += 1
            self.pub.publish(out)
        else:
            self.pub.publish(msg)

    def _report(self) -> None:
        self.get_logger().info(
            f"[imu_fix] 受け {self.n_seen} 件 / 差し替え {self.n_fixed} 件")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--in", dest="topic_in", default="/utlidar/imu_livox_mid360")
    p.add_argument("--out", dest="topic_out", default="/imu_fixed")
    p.add_argument("--no-sim-time", action="store_true", help="実機で使うとき")
    args = p.parse_args(argv)

    rclpy.init()
    node = ImuFix(args.topic_in, args.topic_out, sim_time=not args.no_sim_time)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
