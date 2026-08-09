"""Isaac Sim と ROS 2 をつなぐブリッジ。

SLAM (slam_toolbox) と Nav2 が必要とするトピック・TF を配信し、
Nav2 が出す速度指令を受け取る。

配信するもの:
    /scan       sensor_msgs/LaserScan   2D LiDAR のスキャン
    /odom       nav_msgs/Odometry       オドメトリ（Sim の真値）
    TF          odom -> base_link       ロボットの位置姿勢
    static TF   base_link -> laser      LiDAR の取り付け位置

購読するもの:
    /cmd_vel    geometry_msgs/Twist     Nav2 からの速度指令

map -> odom は slam_toolbox が配信するため、ここでは出さない。

前提:
    rclpy は Isaac Sim の python.sh から import できる（疎通確認済み）。
    どちらも Python 3.12 なので ABI が一致する。env.sh が
    /opt/ros/jazzy/setup.bash を読み込んで PYTHONPATH を通す。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .command import VelocityCommand
from .lidar import ScanData

# TF の frame 名。Nav2 / slam_toolbox の慣習に合わせる。
FRAME_ODOM: str = "odom"
FRAME_BASE: str = "base_link"
FRAME_LASER: str = "laser"

# G1 の root body は "pelvis" で base_link は存在しない（実測で確認済み）。
# ROS 側では base_link という名前が慣習なので、pelvis を base_link として扱う。
# LiDAR は base_link から見て この高さ にある。
LIDAR_OFFSET_Z_FROM_BASE: float = 0.347


@dataclass(frozen=True)
class OdomState:
    """odom として配信するロボットの状態。

    Attributes:
        x: ワールド座標の X [m]
        y: ワールド座標の Y [m]
        yaw: ワールド座標の yaw [rad]
        vx: 胴体座標系の前後速度 [m/s]
        vy: 胴体座標系の左右速度 [m/s]
        yaw_rate: 胴体座標系の角速度 [rad/s]
    """

    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    yaw_rate: float


def quat_wxyz_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """クォータニオン (w,x,y,z) から yaw [rad] を取り出す。

    IsaacLab の root_quat_w は (w, x, y, z) 順である（実測で確認済み）。
    ROS の geometry_msgs は (x, y, z, w) 順なので混同しないこと。
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat_xyzw(yaw: float) -> tuple[float, float, float, float]:
    """yaw [rad] から ROS 順 (x, y, z, w) のクォータニオンを作る。"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class RosBridge:
    """ROS 2 との橋渡しを行うノード。

    Isaac Sim のループから `publish_*` を呼び、`latest_command` で
    Nav2 からの指令を読む。
    """

    def __init__(self, node_name: str = "g1_twin") -> None:
        """ROS 2 を初期化してノードとトピックを作る。"""
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._node: Node = rclpy.create_node(node_name)

        # Isaac Sim は実時間の 0.3〜0.7 倍で動き、その比率も負荷で変動する。
        # 実時間のタイムスタンプを付けると、スキャンの時刻とロボットが実際に
        # そこに居た時刻がずれ、SLAM が誤った姿勢でスキャンを重ねて地図が
        # 放射状に壊れる。そのためシミュレーション内の時刻を /clock として
        # 配信し、SLAM / Nav2 側は use_sim_time:=true で参照させる。
        self._sim_time: float = 0.0
        self._Clock = Clock
        self._clock_pub = self._node.create_publisher(Clock, "/clock", 10)

        # センサデータは欠落を許容する（最新が届けばよい）
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._scan_pub = self._node.create_publisher(LaserScan, "/scan", sensor_qos)
        self._odom_pub = self._node.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self._node)

        # Nav2 からの速度指令を受け取る
        self._latest_command = VelocityCommand()
        self._node.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # メッセージ型を保持しておく（毎回 import しないため）
        self._LaserScan = LaserScan
        self._Odometry = Odometry

        # 静的 TF はシム時刻が動き出してから送る（時刻 0 のまま送ると
        # use_sim_time を使う購読側が受け取れないことがある）。
        # 実際の送信は publish_odom から初回だけ行う。
        self._static_tf_sent = False
        print(
            "[ROS] ノードを起動しました: /clock /scan /odom /tf を配信、"
            "/cmd_vel を購読"
        )

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg) -> None:
        """Nav2 からの速度指令を受け取る。"""
        self._latest_command = VelocityCommand(
            vx=float(msg.linear.x),
            vy=float(msg.linear.y),
            yaw_rate=float(msg.angular.z),
        )

    def _now(self):
        """シミュレーション内の現在時刻を ROS の Time として返す。

        実時間ではなくシム内時刻を使う。実時間だと Sim の進みが遅い分だけ
        スキャンと姿勢の対応がずれ、地図が壊れる。
        """
        from builtin_interfaces.msg import Time

        stamp = Time()
        stamp.sec = int(self._sim_time)
        stamp.nanosec = int((self._sim_time - int(self._sim_time)) * 1e9)
        return stamp

    def publish_clock(self, sim_time: float) -> None:
        """シミュレーション内の経過時刻を /clock として配信する。

        他のノード（slam_toolbox / Nav2）は use_sim_time:=true でこれを
        参照する。配信するトピック類のタイムスタンプもこの時刻に揃う。

        Args:
            sim_time: シミュレーション開始からの経過秒数
        """
        self._sim_time = sim_time
        msg = self._Clock()
        msg.clock = self._now()
        self._clock_pub.publish(msg)

    def _publish_static_tf(self) -> None:
        """base_link -> laser の固定 TF を配信する。"""
        from geometry_msgs.msg import TransformStamped

        tf = TransformStamped()
        tf.header.stamp = self._now()
        tf.header.frame_id = FRAME_BASE
        tf.child_frame_id = FRAME_LASER
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = LIDAR_OFFSET_Z_FROM_BASE
        tf.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(tf)
        print(
            f"[ROS] 固定 TF を配信: {FRAME_BASE} -> {FRAME_LASER} "
            f"(z={LIDAR_OFFSET_Z_FROM_BASE})"
        )

    # ------------------------------------------------------------------
    # 配信
    # ------------------------------------------------------------------
    def publish_scan(self, scan: ScanData) -> None:
        """LaserScan を配信する。"""
        msg = self._LaserScan()
        msg.header.stamp = self._now()
        msg.header.frame_id = FRAME_LASER
        msg.angle_min = scan.angle_min
        msg.angle_max = scan.angle_max
        msg.angle_increment = scan.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = scan.range_min
        msg.range_max = scan.range_max
        # inf はそのまま送ってよい（ROS の仕様で「測距不能」を意味する）
        msg.ranges = [float(r) for r in scan.ranges]
        self._scan_pub.publish(msg)

    def publish_odom(self, state: OdomState) -> None:
        """Odometry と odom -> base_link の TF を配信する。"""
        from geometry_msgs.msg import TransformStamped

        # 静的 TF はシム時刻が進み始めてから 1 度だけ送る
        if not self._static_tf_sent and self._sim_time > 0.0:
            self._publish_static_tf()
            self._static_tf_sent = True

        stamp = self._now()
        qx, qy, qz, qw = yaw_to_quat_xyzw(state.yaw)

        odom = self._Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = FRAME_ODOM
        odom.child_frame_id = FRAME_BASE
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # 速度は child_frame_id（＝胴体座標系）で表す ROS の規約に従う
        odom.twist.twist.linear.x = state.vx
        odom.twist.twist.linear.y = state.vy
        odom.twist.twist.angular.z = state.yaw_rate
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = FRAME_ODOM
        tf.child_frame_id = FRAME_BASE
        tf.transform.translation.x = state.x
        tf.transform.translation.y = state.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

    # ------------------------------------------------------------------
    # 受信
    # ------------------------------------------------------------------
    def spin_once(self) -> None:
        """購読コールバックを 1 回処理する。ループ内で毎周期呼ぶこと。"""
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    @property
    def latest_command(self) -> VelocityCommand:
        """直近に /cmd_vel で受け取った速度指令。"""
        return self._latest_command

    def close(self) -> None:
        """ノードを破棄する。"""
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
        print("[ROS] ノードを終了しました")
