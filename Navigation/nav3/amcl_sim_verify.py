#!/usr/bin/env python3
"""MuJoCo上のG1シミュレーションで、AMCLの自己位置推定がどれだけ正確か検証する。

## 何を確かめたいか

`../nav2/g1_nav2.yaml`は`map -> odom`を恒等変換にしており、AMCLは使われていない
（`Navigation/README.md`参照）。これは「G1内蔵SLAMの位置をそのまま信じる」設計だが、
外部で作った地図・別セッションへの再利用には自己位置推定（再測位）が要る。
このスクリプトは、AMCLが実際にLiDARスキャンから正しい位置を求められるかを、
MuJoCoの物理シミュレーション上で検証する。

## 構成

`sim/rooms.py`の部屋(占有格子)から、MuJoCoの物理モデルとROSの静的地図を
**同じ元から**作る（`sim/rooms.py`が地図とMuJoCoの世界の食い違いを防いでいるのと
同じ考え方）。そこにG1を歩かせ、`mujoco_lidar`で撮ったLiDARスキャンを
`sensor_msgs/LaserScan`としてROS2へ流し、実際にAMCLへ処理させる。

**odomは簡略化してground truthをそのまま流す。** 本来のodomは誤差を持つが、
ここで確かめたいのは「AMCLがスキャンマッチングで地図に対する位置を
正しく求められるか」であって、「オドメトリの誤差にどこまで耐えるか」ではない。
後者を確かめたくなったら、odomにノイズを注入する改造をここに足すこと。

## 実行環境について

`rclpy`（システムのROS2 Jazzy）と`mujoco`/`torch`（sim用）を同じプロセスで
使う必要があるが、Navigation本体の`.venv`はPython 3.10固定
（`cyclonedds`のwheel都合、`pyproject.toml`参照）で、ROS2 Jazzyの`rclpy`は
Python 3.12でビルドされているため**同居できない**。そこで本フォルダ専用に
`--system-site-packages`付きの3.12 venv（`.venv_amcl/`）を別途作り、
そこに`mujoco`/`mujoco-lidar`/`torch`（CPU版）だけ追加した。
`Navigation/.venv`（3.10、`uv sync`管理）とは別物であり、依存関係も混ぜない。

  source /opt/ros/jazzy/setup.bash
  cd Navigation/nav3
  .venv_amcl/bin/python amcl_sim_verify.py --map-only     # 地図生成だけ
  .venv_amcl/bin/python amcl_sim_verify.py                # 地図生成 + sim走行 + 記録
                                                            # （map_server/amclは別途起動しておくこと）
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

NAV_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAV_DIR))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from nav.protocol import Pose2D  # noqa: E402
from pcd_to_ros_map import PIXEL_FREE, PIXEL_OCCUPIED, write_pgm, write_yaml  # noqa: E402
from sim.rooms import get_room  # noqa: E402

DEFAULT_ROOM = "test_room"
NUM_BEAMS = 360
RANGE_MAX_M = 10.0
LIDAR_HEIGHT_MIN_M = 0.15  # 床上。障害物とみなす帯（sim/slam_service.pyと同じ考え方）
LIDAR_HEIGHT_MAX_M = 1.80
CONTROL_TICK_S = 0.1


def grid_to_ros_map(room, output_stem: Path) -> None:
    """`sim/rooms.py`の占有格子を、そのままROS map_server形式で書き出す。

    合成の部屋なので「未観測」は存在しない（占有 or 自由の2値で足りる）。
    `OccupancyGrid.blocked`は行0がy最小（`nav/occupancy.py`の`to_cell`参照）だが、
    ROSのPGMは行0がy最大の規約なので上下反転させる。
    """
    grid = room.grid(inflation=0.0)  # AMCL検証では機体半径の膨張は不要（地図そのものを見る）
    arr = np.where(grid.blocked, PIXEL_OCCUPIED, PIXEL_FREE).astype(np.uint8)
    arr = np.flipud(arr)
    write_pgm(output_stem.with_suffix(".pgm"), arr)
    write_yaml(output_stem.with_suffix(".yaml"), output_stem.with_suffix(".pgm").name,
              grid.spec.resolution, grid.spec.origin_x, grid.spec.origin_y)
    print(f"[amcl-verify] 地図出力: {output_stem}.pgm / .yaml "
          f"({grid.spec.width}x{grid.spec.height}セル, {grid.spec.resolution}m/セル)")


def scan_ranges_from_hits(points_world: np.ndarray, pose: Pose2D) -> np.ndarray:
    """world座標のLiDARヒット点群を、ロボット基準のLaserScan距離配列に変換する。

    等角度ビンごとの最短距離を取る（実物のLaserScanと同じ「最初に当たったもの」の扱い）。
    """
    ranges = np.full(NUM_BEAMS, np.inf, dtype=np.float64)
    if len(points_world) == 0:
        return ranges

    z = points_world[:, 2]
    band = (z >= LIDAR_HEIGHT_MIN_M) & (z <= LIDAR_HEIGHT_MAX_M)
    pts = points_world[band]
    if len(pts) == 0:
        return ranges

    dx = pts[:, 0] - pose.x
    dy = pts[:, 1] - pose.y
    cos_yaw, sin_yaw = math.cos(-pose.yaw), math.sin(-pose.yaw)
    rx = dx * cos_yaw - dy * sin_yaw
    ry = dx * sin_yaw + dy * cos_yaw
    r = np.hypot(rx, ry)
    ang = np.arctan2(ry, rx)

    angle_min = -math.pi
    increment = (2 * math.pi) / NUM_BEAMS
    idx = np.clip(np.floor((ang - angle_min) / increment).astype(np.int64), 0, NUM_BEAMS - 1)
    valid = r <= RANGE_MAX_M
    # 同じビンに複数当たったら最短を残す（np.minimum.atで一括処理）
    np.minimum.at(ranges, idx[valid], r[valid])
    return ranges


class Bridge:
    """MuJoCoの1ステップぶんを ROS2 (/scan, /odom, /tf) へ publish する。"""

    def __init__(self, node, walker, lidar) -> None:
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from tf2_ros import TransformBroadcaster

        self._node = node
        self._walker = walker
        self._lidar = lidar
        self._TransformStamped = TransformStamped
        self._Odometry = Odometry
        self._LaserScan = LaserScan
        self._tf = TransformBroadcaster(node)
        self._scan_pub = node.create_publisher(LaserScan, "/scan", 10)
        self._odom_pub = node.create_publisher(Odometry, "/odom", 10)
        self.history: list[tuple[float, Pose2D, "tuple[float,float] | None"]] = []
        self.amcl_estimate: "tuple[float, float] | None" = None

        from geometry_msgs.msg import PoseWithCovarianceStamped
        node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)

    def _on_amcl(self, msg) -> None:
        self.amcl_estimate = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def publish_step(self) -> None:
        pose = self._walker.pose
        stamp = self._node.get_clock().now().to_msg()
        qz, qw = math.sin(pose.yaw / 2.0), math.cos(pose.yaw / 2.0)

        # odom -> base_link。簡略化のためground truthをそのまま流す（モジュール docstring参照）
        t = self._TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = pose.x
        t.transform.translation.y = pose.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf.sendTransform(t)

        # base_link -> livox_frame は恒等（センサ位置の厳密なオフセットはここでは省略）
        t2 = self._TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = "base_link"
        t2.child_frame_id = "livox_frame"
        t2.transform.rotation.w = 1.0
        self._tf.sendTransform(t2)

        odom = self._Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self._odom_pub.publish(odom)

        points = self._lidar.scan(self._walker.data)
        ranges = scan_ranges_from_hits(points, pose)
        scan = self._LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "livox_frame"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - (2 * math.pi / NUM_BEAMS)
        scan.angle_increment = 2 * math.pi / NUM_BEAMS
        scan.range_min = 0.1
        scan.range_max = RANGE_MAX_M
        scan.ranges = [float(r) for r in ranges]
        self._scan_pub.publish(scan)

        self.history.append((self._walker.sim_time, pose, self.amcl_estimate))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--room", default=DEFAULT_ROOM)
    parser.add_argument("--map-only", action="store_true", help="地図(.pgm/.yaml)だけ作って終わる")
    parser.add_argument("--duration", type=float, default=20.0, help="走らせるsim時間[s]")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    room = get_room(args.room)
    out = Path(__file__).resolve().parent / "amcl_test_map"
    grid_to_ros_map(room, out)
    if args.map_only:
        return 0

    from sim.g1_walker import G1Walker, Mid360, build_model
    model = build_model(room)
    walker = G1Walker(model)
    lidar = Mid360(walker.model)

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("amcl_sim_bridge")
    bridge = Bridge(node, walker, lidar)

    # 台本: 前進・旋回・横移動を混ぜて動かす（四隅巡回ほど大きくは動かさない）
    plan = [
        (0.4, 0.0, 0.0, 4.0),
        (0.0, 0.0, 0.5, 3.0),
        (0.3, 0.0, 0.0, 4.0),
        (0.0, 0.2, 0.0, 3.0),
        (0.0, 0.0, -0.6, 3.0),
        (0.3, 0.0, 0.0, 3.0),
    ]
    print(f"[amcl-verify] 出発姿勢: {walker.pose}")
    t_budget = args.duration
    for vx, vy, wz, dur in plan:
        if t_budget <= 0:
            break
        dur = min(dur, t_budget)
        walker.set_command(vx, vy, wz)
        t_end = walker.sim_time + dur
        while walker.sim_time < t_end:
            walker.step(CONTROL_TICK_S)
            bridge.publish_step()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.03)  # AMCLの処理に間を与える（実時間で流す）
        t_budget -= dur
    walker.stop()
    for _ in range(20):
        walker.step(CONTROL_TICK_S)
        bridge.publish_step()
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.03)

    print(f"[amcl-verify] 最終ground truth: {walker.pose}")
    if bridge.amcl_estimate is None:
        print("[amcl-verify] ⚠️ /amcl_poseを一度も受信しなかった。"
              "map_server/amclが起動しているか、initial_poseが設定されているか確認すること")
    else:
        ex, ey = bridge.amcl_estimate
        err = math.hypot(walker.pose.x - ex, walker.pose.y - ey)
        print(f"[amcl-verify] 最終AMCL推定: x={ex:+.3f} y={ey:+.3f}  誤差={err:.3f}m")

    # 誤差の推移（収束したかを見る）
    print("[amcl-verify] 時刻ごとの誤差（AMCL推定が来ていた区間のみ）:")
    for t, gt, amcl in bridge.history[::max(1, len(bridge.history) // 15)]:
        if amcl is None:
            continue
        err = math.hypot(gt.x - amcl[0], gt.y - amcl[1])
        print(f"    t={t:6.2f}s  真値=({gt.x:+.2f},{gt.y:+.2f})  "
              f"AMCL=({amcl[0]:+.2f},{amcl[1]:+.2f})  誤差={err:.3f}m")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
