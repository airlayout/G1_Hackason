"""octomap_server が /points を受けて 3D 地図を作れるかを確認する。

Isaac Sim を起動せず、合成した点群を流して octomap の設定（パラメータ名、
remap、TF）が正しいかだけを切り分ける。実機や Sim を絡めると
「点群が悪いのか設定が悪いのか」が分からなくなるため。

確認すること:
    1. octomap_server が /points を購読できているか（remap が効いているか）
    2. /octomap_full と /projected_map が出てくるか
    3. 既知の形（壁）を入れて、地図がその形になるか

実行方法:
    cd /home/spacedata/isaac_dev/G1/SimEnv3D
    source env.sh
    python3 src/test_octomap_pipeline.py
"""

from __future__ import annotations

import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from octomap_msgs.msg import Octomap
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

PASS = "[OK]"
FAIL = "[NG]"

FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_BASE = "base_link"
FRAME_LIDAR3D = "lidar3d"

# 合成する壁の位置。センサから 3 m 前方に、幅 4 m・高さ 2 m の壁を置く。
WALL_DISTANCE = 3.0


def make_wall_points() -> np.ndarray:
    """センサ座標系で、前方 3 m にある壁の点群を作る。

    octomap が既知の形を正しく再現できるかを見るための入力。
    """
    ys = np.arange(-2.0, 2.0, 0.05)
    zs = np.arange(-0.9, 1.1, 0.05)  # センサ基準（センサは地上 1.1 m）
    yy, zz = np.meshgrid(ys, zs)
    xx = np.full_like(yy, WALL_DISTANCE)
    return np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=-1).astype(np.float32)


def build_cloud(points: np.ndarray, stamp) -> PointCloud2:
    """PointCloud2 を組む（ros_bridge.publish_points と同じ手順）。"""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME_LIDAR3D
    pts = np.asarray(points, dtype=np.float32)
    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.is_dense = True
    msg.is_bigendian = False
    fields = []
    for i, name in enumerate(("x", "y", "z")):
        f = PointField()
        f.name = name
        f.offset = 4 * i
        f.datatype = PointField.FLOAT32
        f.count = 1
        fields.append(f)
    msg.fields = fields
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = pts.tobytes()
    return msg


class Harness(Node):
    """点群と TF と /clock を流し、octomap の出力を受け取る。"""

    def __init__(self) -> None:
        super().__init__("octomap_test_harness")
        # octomap は use_sim_time:=true なので /clock を流す必要がある
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", value=False)]
        )
        self._clock_pub = self.create_publisher(Clock, "/clock", 10)
        self._points_pub = self.create_publisher(PointCloud2, "/points", 10)
        self._static_tf = StaticTransformBroadcaster(self)

        self.octomap_count = 0
        self.projected: OccupancyGrid | None = None
        self.create_subscription(Octomap, "/octomap_full", self._on_octomap, 10)
        self.create_subscription(
            OccupancyGrid, "/projected_map", self._on_projected, 10
        )
        self._sim_time = 0.0

    def _on_octomap(self, msg: Octomap) -> None:
        self.octomap_count += 1

    def _on_projected(self, msg: OccupancyGrid) -> None:
        self.projected = msg

    def publish_tf(self) -> None:
        """map -> odom -> base_link -> lidar3d を恒等変換で流す。

        octomap は点群を map フレームへ変換するため、この連鎖が必要。
        欠けていると octomap は点を一切取り込まない（無言で）。
        """
        transforms = []
        for parent, child, z in (
            (FRAME_MAP, FRAME_ODOM, 0.0),
            (FRAME_ODOM, FRAME_BASE, 0.0),
            (FRAME_BASE, FRAME_LIDAR3D, 1.1),
        ):
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = parent
            tf.child_frame_id = child
            tf.transform.translation.z = z
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        self._static_tf.sendTransform(transforms)

    def tick(self, points: np.ndarray) -> None:
        """/clock と点群を 1 回流す。"""
        self._sim_time += 0.1
        clock = Clock()
        clock.clock.sec = int(self._sim_time)
        clock.clock.nanosec = int((self._sim_time % 1.0) * 1e9)
        self._clock_pub.publish(clock)
        self._points_pub.publish(build_cloud(points, clock.clock))


def main() -> None:
    failures = 0

    print("=" * 70)
    print("octomap_server パイプラインの検証（Isaac Sim 不要）")
    print("=" * 70)

    # octomap_server を起動する。/cloud_in を /points に remap する。
    print("[INFO] octomap_server を起動します")
    proc = subprocess.Popen(
        [
            "ros2", "run", "octomap_server", "octomap_server_node",
            "--ros-args",
            "--params-file", "config/octomap.yaml",
            "-r", "/cloud_in:=/points",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    rclpy.init()
    node = Harness()
    wall = make_wall_points()
    print(f"[INFO] 合成した壁: {len(wall)} 点、前方 {WALL_DISTANCE} m")

    try:
        time.sleep(6.0)  # ノードの起動待ち
        node.publish_tf()
        time.sleep(1.0)

        # 点群を繰り返し流す（octomap は複数回の観測で確率を更新する）
        print("[INFO] 点群を 30 回流します")
        for _ in range(30):
            node.tick(wall)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)

        # 出力を待つ
        deadline = time.time() + 15.0
        while time.time() < deadline and (
            node.octomap_count == 0 or node.projected is None
        ):
            node.tick(wall)
            rclpy.spin_once(node, timeout_sec=0.2)

        # --- 1. octomap が出たか ---
        print(f"[Test] /octomap_full の配信")
        if node.octomap_count > 0:
            print(f"  {PASS} {node.octomap_count} 回受信した")
        else:
            failures += 1
            print(f"  {FAIL} 受信できなかった（remap か TF か設定の問題）")

        # --- 2. 2D 投影が出たか ---
        print(f"[Test] /projected_map の配信")
        if node.projected is not None:
            grid = node.projected
            occupied = sum(1 for v in grid.data if v > 50)
            print(f"  {PASS} 受信した: {grid.info.width} x {grid.info.height} セル, "
                  f"解像度 {grid.info.resolution:.2f} m")
            print(f"  [INFO] 占有セル: {occupied}")

            # --- 3. 壁の位置が正しいか ---
            print(f"[Test] 壁の位置")
            if occupied > 0:
                # 占有セルのワールド X 座標を出す。壁は x=3.0 付近にあるはず。
                xs = []
                for i, v in enumerate(grid.data):
                    if v > 50:
                        col = i % grid.info.width
                        xs.append(
                            grid.info.origin.position.x
                            + (col + 0.5) * grid.info.resolution
                        )
                mean_x = sum(xs) / len(xs)
                print(f"  占有セルの平均 X: {mean_x:.2f} m（期待 {WALL_DISTANCE:.1f} m）")
                if abs(mean_x - WALL_DISTANCE) < 0.5:
                    print(f"  {PASS} 壁が正しい位置にある")
                else:
                    failures += 1
                    print(f"  {FAIL} 壁の位置がずれている")
            else:
                failures += 1
                print(f"  {FAIL} 占有セルが無い（filter_ground_plane が強すぎる可能性）")
        else:
            failures += 2
            print(f"  {FAIL} 受信できなかった")

    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            out = ""
        # octomap 側のエラーは切り分けに重要なので出す
        for line in (out or "").splitlines():
            if any(k in line.lower() for k in ("error", "warn", "fail")):
                print(f"  [octomap] {line}")

    print("=" * 70)
    if failures:
        print(f"{FAIL} {failures} 件が失敗しました")
        sys.exit(1)
    print(f"{PASS} すべて成功しました")


main()
