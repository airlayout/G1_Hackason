#!/usr/bin/env python3
"""G1 内蔵 SLAM の odometry を Nav2 が使える TF/トピックに変換する。

## なぜ必要か

`1801`(建図開始)を送ると内蔵 SLAM が `/unitree/slam_mapping/odom` を 9.1Hz で流す
（実測。詳細: `findings/g1_dds_sensors.md` §8）。しかし:

1. **TF は一切出ない**（`/tf` も `/tf_static` も存在しない）。Nav2 は TF が無いと動かない
2. **frame_id が `map` / child が `base_link` になっているが、原点は 1801 を送った時点の
   ロボット位置**であり、意味的には odometry である。これを `map` と名乗らせたまま使うと、
   保存地図の座標系と衝突する

そこで本ノードが:

- `/unitree/slam_mapping/odom` を購読し、**`odom → base_link` の TF** として発行する
  （`map` → `odom` に改名する）
- 同じ内容を `/odom`（フレーム名を直したもの）として再配信する（Nav2 の
  controller が速度フィードバックに使う）
- **`map → odom`** を静的変換として発行する。既定は恒等変換で、処理済み地図への
  ICP 合わせ（`tools/match_scan_to_map_2d.py`）で求めた値を `--map-to-odom` で渡せる
- **`base_link → livox_frame`** を静的変換として発行する。点群を costmap に入れるために必要

## 姿勢（base_link → livox_frame）の与え方 — 起動時に自動校正する

⚠️ **内蔵 SLAM の odom の姿勢は「重力に対する水平」ではない。**（2026-09-09 実測）
静止した座位で SLAM は **pitch = -7.469° ± 0.027°** を安定して報告する一方、
同時刻の IMU が示すセンサーの傾きは約 3.9° だった。つまり SLAM は
**ロボット胴体の姿勢**を出しており（座位で胴体が前傾している）、センサーの傾きとは別物。

これを無補正で Nav2 に渡すと costmap が傾く。実測した合成後のずれ:

| 経路 | 重力の -Z 軸からのずれ |
|---|---|
| 生の livox_frame | 3.9° |
| `base_link ← livox_frame`（静的補正のみ）| 2.09° |
| **`map ← livox_frame`（SLAM の姿勢込み）** | **6.10°**（＝悪化） |
| `map ← base_link` | 11.29° |

そこで `--auto-level`（既定 ON）では、起動時に IMU の重力と SLAM の姿勢を同時に採り、

    R(base_link←livox) = R(map←base_link)ᵀ · R_level

を計算して静的変換にする（`R_level` は重力を (0,0,-1) に合わせる回転）。
これで `map ← livox_frame` の合成が重力整列になる。

⚠️ **この自動校正は「起動時の姿勢＝運用中の姿勢」でしか正しくない。**
本来 `base_link→livox_frame` は剛体の固定変換であり、歩行中は胴体姿勢が振動するので
**取付角の実測値（U-09）で置き換えるべき**。現状は静止した配線試験のための実用的な近似。

### 並進（base_link の高さ）

**`base_link` は床面に置く**。`base_link → livox_frame` の並進 z は U-09 の実測値
**1.213m**（立位でのセンサー高さ）を既定にしている。こうすると costmap の
`min_obstacle_height`/`max_obstacle_height` を**床基準**で書けるので直感的。

⚠️ **並進を (0,0,0) にすると base_link がセンサー位置（床から1.2m）になり、
costmap の高さ帯が 1.2m ずれて天井付近を拾う。**（2026-09-09 に実際にこの状態で
Nav2 配線試験を走らせてしまった）

x/y のオフセットは未計測（0 のまま）。歩行時の姿勢変化で z も変わるので、
厳密には脚の関節角から求めるべきだが、2D の costmap では影響は小さい。

## 使い方

    python3 g1_slam_odom_tf.py --gravity 0.0918 -0.0443 -0.9948
    python3 g1_slam_odom_tf.py --map-to-odom 3.2 -1.5 0.42   # ICP の結果(x y yaw[rad])
"""
from __future__ import annotations

import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

SLAM_ODOM_TOPIC = "/unitree/slam_mapping/odom"


def leveling_quaternion(ax: float, ay: float, az: float) -> tuple[float, float, float, float]:
    """センサー座標系で測った**加速度計の値(=上向き)**を (0,0,+1) に合わせる回転を返す。

    ⚠️ **符号を取り違えないこと(2026-09-09 に実際にバグを作った)。**
    静止した加速度計が返すのは「重力」ではなく**重力の反作用＝上向き**である。
    当初これを「重力」だと思って (0,0,-1) に合わせており、**z軸が下向きのフレーム**を
    作ってしまっていた(ROS の base_link/odom/map は z 上向きが規約)。

    さらに **MID-360 は逆さ(roll≈180°)に取り付けられている**ため、センサー座標系での
    上向きは概ね (0,0,-1) を指す。したがって正しい変換には約180°の反転が含まれる。
    実測: 本体IMU の上向き (-0.056,+0.018,+0.998) に対し、Livox IMU は (-0.064,-0.002,-0.999)。

    軸角で素直に作る。オイラー角の順序規約に悩まなくて済むのが利点。
    """
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    gx, gy, gz = ax / norm, ay / norm, az / norm
    # 加速度計の値(上向き)を +z へ合わせる ＝ 出来上がるフレームは z 上向き(ROS 規約)
    tx, ty, tz = 0.0, 0.0, 1.0
    # 回転軸 = g × target、回転角 = acos(g・target)
    ax, ay, az = gy * tz - gz * ty, gz * tx - gx * tz, gx * ty - gy * tx
    s = math.sqrt(ax * ax + ay * ay + az * az)
    c = gx * tx + gy * ty + gz * tz
    if s < 1e-9:
        return (0.0, 0.0, 0.0, 1.0) if c > 0 else (1.0, 0.0, 0.0, 0.0)
    ax, ay, az = ax / s, ay / s, az / s
    angle = math.atan2(s, c)
    half = angle / 2.0
    sin_h = math.sin(half)
    return (ax * sin_h, ay * sin_h, az * sin_h, math.cos(half))


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_matrix(q: "tuple[float, float, float, float]") -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(m: np.ndarray) -> "tuple[float, float, float, float]":
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


class SlamOdomTf(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("g1_slam_odom_tf")
        self._args = args
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._count = 0

        # 内蔵SLAMは RELIABLE + VOLATILE で配信する(実測)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, SLAM_ODOM_TOPIC, self._on_odom, qos)

        # 自動校正用。IMU の重力と SLAM の姿勢を同時に貯める
        self._imu_samples: list[list[float]] = []
        self._odom_quats: list[list[float]] = []
        self._static_done = False
        if self._args.auto_level:
            imu_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(Imu, self._args.imu_topic, self._on_imu, imu_qos)
            self.get_logger().info(
                f"自動校正: {self._args.imu_topic} と {SLAM_ODOM_TOPIC} を "
                f"{self._args.calib_samples} サンプル貯めてから静的TFを出す")
        else:
            self._publish_static(leveling_quaternion(*self._args.gravity))
        self.get_logger().info(
            f"{SLAM_ODOM_TOPIC} -> TF(odom->base_link) + /odom に変換する。"
            f"map->odom = x={args.map_to_odom[0]:.3f} y={args.map_to_odom[1]:.3f} "
            f"yaw={args.map_to_odom[2]:.3f}rad")

    def _publish_static(self, lidar_rotation: "tuple[float, float, float, float]") -> None:
        now = self.get_clock().now().to_msg()
        transforms = []

        # map -> odom。既定は恒等(＝配線試験用)。ICP の結果を渡せば地図基準になる
        m2o = TransformStamped()
        m2o.header.stamp = now
        m2o.header.frame_id = "map"
        m2o.child_frame_id = "odom"
        m2o.transform.translation.x = float(self._args.map_to_odom[0])
        m2o.transform.translation.y = float(self._args.map_to_odom[1])
        qx, qy, qz, qw = yaw_quaternion(float(self._args.map_to_odom[2]))
        m2o.transform.rotation.x, m2o.transform.rotation.y = qx, qy
        m2o.transform.rotation.z, m2o.transform.rotation.w = qz, qw
        transforms.append(m2o)

        # base_link -> livox_frame。回転は重力から求める(上の docstring 参照)
        b2l = TransformStamped()
        b2l.header.stamp = now
        b2l.header.frame_id = "base_link"
        b2l.child_frame_id = self._args.lidar_frame
        b2l.transform.translation.x = float(self._args.lidar_xyz[0])
        b2l.transform.translation.y = float(self._args.lidar_xyz[1])
        b2l.transform.translation.z = float(self._args.lidar_xyz[2])
        qx, qy, qz, qw = lidar_rotation
        b2l.transform.rotation.x, b2l.transform.rotation.y = qx, qy
        b2l.transform.rotation.z, b2l.transform.rotation.w = qz, qw
        transforms.append(b2l)

        self._static_tf.sendTransform(transforms)
        self._static_done = True
        self.get_logger().info(
            f"静的TF を発行した: map->odom, base_link->{self._args.lidar_frame}")

    def _on_imu(self, msg: Imu) -> None:
        a = msg.linear_acceleration
        if len(self._imu_samples) < self._args.calib_samples * 4:
            self._imu_samples.append([a.x, a.y, a.z])

    def _try_auto_level(self) -> None:
        """IMU の重力と SLAM の姿勢から base_link->livox_frame を逆算する。

        求めたいのは「合成 R(map<-livox) が重力整列になる」こと。
        R(map<-livox) = R(map<-base_link) * R(base_link<-livox) なので、
            R(base_link<-livox) = R(map<-base_link)^T * R_level
        とすれば R(map<-livox) = R_level となり、重力が (0,0,-1) に落ちる。
        """
        if self._static_done:
            return
        need = self._args.calib_samples
        if len(self._imu_samples) < need or len(self._odom_quats) < need:
            return

        g = np.asarray(self._imu_samples[-need:]).mean(axis=0)
        norm = float(np.linalg.norm(g))
        if norm < 1e-6:
            self.get_logger().error("IMU の重力が 0。自動校正できない")
            return
        g = g / norm
        r_level = quat_to_matrix(leveling_quaternion(float(g[0]), float(g[1]), float(g[2])))

        q = np.asarray(self._odom_quats[-need:]).mean(axis=0)
        q = q / float(np.linalg.norm(q))
        r_map_base = quat_to_matrix((float(q[0]), float(q[1]), float(q[2]), float(q[3])))

        r_needed = r_map_base.T @ r_level
        # 検算: 合成した結果、重力が (0,0,-1) に落ちるか
        # 検算: 加速度計の値(上向き)が map 系で (0,0,+1) に来れば正しい(z上向き=ROS規約)
        gm = (r_map_base @ r_needed) @ g
        residual = math.degrees(math.acos(max(-1.0, min(1.0, float(gm[2])))))
        # 生センサーの傾き。MID-360 は逆さ取付なので上向きは概ね -z を指す
        tilt_raw = math.degrees(math.acos(max(-1.0, min(1.0, float(abs(g[2]))))))
        self.get_logger().info(
            f"自動校正した: 生センサーの傾き {tilt_raw:.2f}° / "
            f"map系で上向きが +z から {residual:.3f}° (0に近ければ成功)")
        self._publish_static(matrix_to_quat(r_needed))

    def _on_odom(self, msg: Odometry) -> None:
        # 内蔵SLAMは frame_id='map' / child='base_link' と名乗るが、原点は1801時点の
        # ロボット位置なので odometry として扱う。ここで改名する
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(t)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link"
        out.pose = msg.pose
        out.twist = msg.twist
        self._odom_pub.publish(out)

        if self._args.auto_level and not self._static_done:
            o = msg.pose.pose.orientation
            self._odom_quats.append([o.x, o.y, o.z, o.w])
            self._try_auto_level()

        self._count += 1
        if self._count % 90 == 1:
            p = msg.pose.pose.position
            self.get_logger().info(
                f"{self._count} 件中継 (最新 x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f})")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gravity", type=float, nargs=3, default=[-0.0642, -0.0016, -0.9994],
                        metavar=("AX", "AY", "AZ"),
                        help="IMU の linear_acceleration(センサー座標系)。**重力ではなく上向き**。"
                             "既定は立位での実測値。MID-360 は逆さ取付なので z が負になる")
    parser.add_argument("--lidar-xyz", type=float, nargs=3, default=[0.0, 0.0, 1.213],
                        metavar=("X", "Y", "Z"),
                        help="base_link から LiDAR までの並進[m]。既定の z=1.213 は U-09 の実測値"
                             "(立位でのセンサー高さ)。**base_link を床面に置く前提**なので、"
                             "costmap の高さ帯(min/max_obstacle_height)を床基準で書ける。"
                             "x/y は未計測のため 0")
    parser.add_argument("--lidar-frame", default="livox_frame",
                        help="点群の frame_id (既定: livox_frame)")
    parser.add_argument("--map-to-odom", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("X", "Y", "YAW"),
                        help="map->odom の静的変換。既定は恒等(配線試験用)。"
                             "ICP(match_scan_to_map_2d.py)の結果を渡すと地図基準になる")
    parser.add_argument("--auto-level", dest="auto_level", action="store_true", default=True,
                        help="起動時に IMU と SLAM 姿勢から base_link->LiDAR を逆算する(既定)")
    parser.add_argument("--no-auto-level", dest="auto_level", action="store_false",
                        help="自動校正せず --gravity の値だけを使う")
    parser.add_argument("--imu-topic", default="/utlidar/imu_livox_mid360")
    parser.add_argument("--calib-samples", type=int, default=50,
                        help="自動校正に使うサンプル数(既定: 50)")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SlamOdomTf(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
