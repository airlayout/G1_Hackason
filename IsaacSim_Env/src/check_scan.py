"""/scan の安定性を確認するツール。

連続するスキャンが「ほぼ同じ」であることを確かめる。ロボットは制御 1 周期で
数センチしか動かないため、同じ方向の距離が大きく変わるのは異常である。

地図が放射状に壊れる不具合の切り分けに使った。原因は LiDAR の
ray_alignment を "base"（胴体の姿勢に完全追従）にしていたことで、
歩行中の pitch/roll がレイに乗って上下を向き、同じ方向の距離が
1 スキャンごとに 10 m 近く暴れていた。"yaw" にすると解消する。

実行方法:
    source env.sh && python3 src/check_scan.py
"""

from __future__ import annotations

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

# 連続スキャン間の差がこれを超えたら異常とみなす [m]
STABLE_THRESHOLD_MEAN: float = 0.5
# 集めるスキャン数
NUM_SCANS: int = 8


class ScanChecker(Node):
    """/scan を数フレーム集めて安定性を見るノード。"""

    def __init__(self) -> None:
        super().__init__("scan_checker")
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.scans: list[np.ndarray] = []
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos)

    def _on_scan(self, msg: LaserScan) -> None:
        """スキャンを保存する。"""
        self.scans.append(np.array(msg.ranges))


def main() -> None:
    """スキャンを集めて安定性を判定する。"""
    rclpy.init()
    node = ScanChecker()
    for _ in range(900):
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.scans) >= NUM_SCANS:
            break

    scans = node.scans
    print(f"[Scan] 取得したスキャン数: {len(scans)}")
    if len(scans) < 2:
        print("[NG] スキャンが足りません。Isaac Sim が動いているか確認してください")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # 各スキャンの概要。前方 (index 180 = 0 度) の距離を見る。
    for i, s in enumerate(scans[:5]):
        finite = np.isfinite(s)
        nearest = np.min(np.where(finite, s, np.inf)) if finite.any() else float("nan")
        print(
            f"[Scan] scan{i}: 有効 {int(finite.sum()):3d}/{s.size} "
            f"最短 {nearest:6.2f} m 前方 {s[len(s) // 2]:6.2f} m"
        )

    # 連続スキャン間の差を見る。ロボットは 1 周期で数センチしか動かないので、
    # 同じ方向の距離が大きく変わるのは LiDAR の取り付け方に問題がある。
    diffs = []
    for a, b in zip(scans, scans[1:]):
        both = np.isfinite(a) & np.isfinite(b)
        if both.any():
            diffs.append(np.abs(a[both] - b[both]))

    if not diffs:
        print("[NG] 比較できる有効ビームがありません")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    all_diff = np.concatenate(diffs)
    mean_diff = float(all_diff.mean())
    max_diff = float(all_diff.max())
    print(f"[Scan] 連続スキャンの差: 平均 {mean_diff:.3f} m / 最大 {max_diff:.3f} m")

    if mean_diff < STABLE_THRESHOLD_MEAN:
        print(f"[OK] スキャンは安定している（平均差 < {STABLE_THRESHOLD_MEAN} m）")
    else:
        print(
            f"[NG] スキャンが不安定（平均差 {mean_diff:.2f} m）。"
            f"LiDAR の ray_alignment が 'yaw' になっているか確認すること"
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
