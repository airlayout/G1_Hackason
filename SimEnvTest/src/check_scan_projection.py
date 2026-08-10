"""スキャン 1 枚を odom 姿勢で世界座標へ投影して画像化する。

地図が壊れる原因の切り分け。SLAM を介さず、自分で
「スキャン + 姿勢 -> 点群」の変換だけを行う。

ここで壁の形が出るなら、スキャンと姿勢は正しく、原因は slam_toolbox 側。
ここでも形が出ないなら、スキャンか姿勢のどちらかが間違っている。

複数フレームを重ねて、フレーム間で点群が一致するかも見る。

実行方法:
    source env.sh && python3 src/check_scan_projection.py
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

# このファイルの位置から maps/ を求める（リポジトリの置き場所に依存しない）
OUT_PATH: str = str(pathlib.Path(__file__).resolve().parent.parent / "maps" / "projection.png")
# 何フレーム重ねるか
NUM_FRAMES: int = 12
# 出力画像の解像度 [m/pixel]
RESOLUTION: float = 0.05


class Collector(Node):
    """scan と odom を対にして集めるノード。"""

    def __init__(self) -> None:
        super().__init__("scan_projection")
        sensor_qos = QoSProfile(
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.latest_odom: tuple[float, float, float] | None = None
        self.frames: list[tuple[np.ndarray, LaserScan, tuple[float, float, float]]] = []
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos)

    def _on_odom(self, msg: Odometry) -> None:
        """直近の姿勢を保持する。"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.latest_odom = (p.x, p.y, yaw)

    def _on_scan(self, msg: LaserScan) -> None:
        """スキャンとそのときの姿勢を対にして保存する。"""
        if self.latest_odom is None:
            return
        self.frames.append((np.array(msg.ranges), msg, self.latest_odom))


def project(ranges: np.ndarray, scan: LaserScan, pose: tuple[float, float, float]):
    """スキャンを世界座標の点群に変換する。

    Args:
        ranges: 各ビームの距離
        scan: LaserScan（角度情報を使う）
        pose: (x, y, yaw) のロボット姿勢

    Returns:
        (N, 2) の世界座標点群
    """
    x, y, yaw = pose
    # 有効な測距だけを使う。0.0（近すぎ）と inf（当たらず）は除く。
    idx = np.where(np.isfinite(ranges) & (ranges > scan.range_min))[0]
    if idx.size == 0:
        return np.zeros((0, 2))
    # yaw は足さない。LiDAR は ray_alignment="yaw" で構築されており、
    # /scan の角度はすでにワールド座標基準になっているため。
    angles = scan.angle_min + idx * scan.angle_increment
    d = ranges[idx]
    return np.stack([x + d * np.cos(angles), y + d * np.sin(angles)], axis=1)


def main() -> None:
    """スキャンを集めて投影し、画像に書き出す。"""
    rclpy.init()
    node = Collector()
    for _ in range(1200):
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.frames) >= NUM_FRAMES:
            break

    if len(node.frames) < 2:
        print(f"[NG] スキャンが足りません（{len(node.frames)} 枚）")
        sys.exit(1)

    print(f"[INFO] {len(node.frames)} フレームを取得しました")

    clouds = []
    for i, (ranges, scan, pose) in enumerate(node.frames):
        cloud = project(ranges, scan, pose)
        clouds.append(cloud)
        if i < 3:
            print(
                f"[INFO] frame{i}: 姿勢=({pose[0]:+.2f}, {pose[1]:+.2f}, "
                f"{math.degrees(pose[2]):+.0f} 度) 有効点={cloud.shape[0]}"
            )

    all_points = np.concatenate([c for c in clouds if c.size], axis=0)
    print(f"[INFO] 合計点数: {all_points.shape[0]}")
    if all_points.shape[0] == 0:
        print("[NG] 有効な点がありません")
        sys.exit(1)

    lo = all_points.min(axis=0)
    hi = all_points.max(axis=0)
    print(
        f"[INFO] 点群の範囲: X {lo[0]:+.1f} 〜 {hi[0]:+.1f} m / "
        f"Y {lo[1]:+.1f} 〜 {hi[1]:+.1f} m"
    )

    # 画像化する
    from PIL import Image

    width = max(1, int((hi[0] - lo[0]) / RESOLUTION) + 1)
    height = max(1, int((hi[1] - lo[1]) / RESOLUTION) + 1)
    img = np.full((height, width), 255, dtype=np.uint8)
    px = ((all_points[:, 0] - lo[0]) / RESOLUTION).astype(int)
    py = ((all_points[:, 1] - lo[1]) / RESOLUTION).astype(int)
    px = np.clip(px, 0, width - 1)
    py = np.clip(py, 0, height - 1)
    # Y を反転して画像の向きを合わせる
    img[height - 1 - py, px] = 0
    Image.fromarray(img).save(OUT_PATH)
    print(f"[OK] 投影図を書き出しました: {OUT_PATH} ({width} x {height})")

    # フレーム間の一致を見る。壁が同じ位置に来るなら姿勢は正しい。
    # 各フレームの点群の重心のばらつきで簡易的に判定する。
    centroids = np.array([c.mean(axis=0) for c in clouds if c.size])
    spread = centroids.std(axis=0)
    print(
        f"[INFO] フレームごとの重心のばらつき: "
        f"X {spread[0]:.2f} m / Y {spread[1]:.2f} m"
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
