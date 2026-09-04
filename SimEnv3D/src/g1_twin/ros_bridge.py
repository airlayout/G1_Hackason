"""Isaac Sim と ROS 2 をつなぐブリッジ。

SLAM (slam_toolbox) と Nav2 が必要とするトピック・TF を配信し、
Nav2 が出す速度指令を受け取る。

配信するもの:
    /scan       sensor_msgs/LaserScan   2D LiDAR のスキャン
    /points     sensor_msgs/PointCloud2 3D LiDAR の点群（octomap 用）
    /odom       nav_msgs/Odometry       オドメトリ（Sim の真値）
    TF          odom -> base_link       ロボットの位置姿勢
    static TF   base_link -> laser      2D LiDAR の取り付け位置
    static TF   base_link -> lidar3d    3D LiDAR の取り付け位置（前傾を含む）

購読するもの:
    /cmd_vel    geometry_msgs/Twist     Nav2 からの速度指令

map -> odom は slam_toolbox が配信するため、ここでは出さない。

前提:
    rclpy は Isaac Sim の python.sh から import できる（疎通確認済み）。
    どちらも Python 3.12 なので ABI が一致する。env.sh が
    /opt/ros/jazzy/setup.bash を読み込んで PYTHONPATH を通す。
"""

from __future__ import annotations

import copy
import io
import json
import math
import struct
from dataclasses import dataclass

from .command import VelocityCommand
from .lidar import ScanData

# TF の frame 名。Nav2 / slam_toolbox の慣習に合わせる。
FRAME_ODOM: str = "odom"
FRAME_BASE: str = "base_link"
FRAME_LASER: str = "laser"
# 3D LiDAR は 2D とは取り付け姿勢（前傾）が違うため別フレームで持つ。
FRAME_LIDAR3D: str = "lidar3d"
FRAME_LIVOX: str = "livox_frame"
FRAME_CAMERA: str = "camera_color_optical_frame"

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


@dataclass(frozen=True)
class GroundTruthState:
    """評価用の完全な6DoF状態。"""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]


def quat_xyzw_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """クォータニオン (x,y,z,w) から yaw [rad] を取り出す。

    IsaacLab の root_quat_w は **(x, y, z, w) 順**である。
    base_articulation_data.py の docstring に
    "Root link orientation (x, y, z, w)" と明記されている。

    以前 (w,x,y,z) と誤解していた。静止状態 (yaw=30度) での検証では
    たまたま辻褄が合ってしまい、歩行中に初めて破綻した。
    実測: 初期姿勢（無回転）の生値が [-0.0005, -0.0014, 0.0024, 1.0] で、
    単位クォータニオン w=1 が最後に来ることから (x,y,z,w) と確定した。

    ROS の geometry_msgs も (x, y, z, w) 順なので、そのまま渡せる。
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
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, LaserScan, PointCloud2, PointField
        from std_msgs.msg import String
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
        self._points_pub = self._node.create_publisher(
            PointCloud2, "/utlidar/cloud_livox_mid360", sensor_qos
        )
        self._legacy_points_pub = self._node.create_publisher(
            PointCloud2, "/points", sensor_qos
        )
        self._imu_pub = self._node.create_publisher(
            Imu, "/utlidar/imu_livox_mid360", sensor_qos
        )
        self._image_pub = self._node.create_publisher(
            CompressedImage, "/g1_camera/color/image/compressed", sensor_qos
        )
        self._camera_metadata_pub = self._node.create_publisher(
            String, "/g1_camera/frame_metadata", sensor_qos
        )
        camera_info_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._camera_info_pub = self._node.create_publisher(
            CameraInfo, "/g1_camera/color/camera_info", camera_info_qos
        )
        self._odom_pub = self._node.create_publisher(Odometry, "/odom", 10)
        self._ground_truth_odom_pub = self._node.create_publisher(
            Odometry, "/g1_sim/ground_truth/odom", 10
        )
        self._ground_truth_camera_pub = self._node.create_publisher(
            PoseStamped, "/g1_sim/ground_truth/camera_pose", sensor_qos
        )
        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self._node)

        # Nav2 からの速度指令を受け取る
        self._latest_command = VelocityCommand()
        self._node.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # メッセージ型を保持しておく（毎回 import しないため）
        self._LaserScan = LaserScan
        self._Odometry = Odometry
        self._PointCloud2 = PointCloud2
        self._PointField = PointField
        self._Imu = Imu
        self._CompressedImage = CompressedImage
        self._CameraInfo = CameraInfo
        self._PoseStamped = PoseStamped
        self._String = String
        self._camera_sequence = 0
        self._sensor_rig_tf_sent = False

        # 静的 TF はシム時刻が動き出してから送る（時刻 0 のまま送ると
        # use_sim_time を使う購読側が受け取れないことがある）。
        # 実際の送信は publish_odom から初回だけ行う。
        self._static_tf_sent = False
        print(
            "[ROS] ノードを起動しました: /clock、LiDAR、IMU、RGB、真値を配信、"
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

        return self._time_at(self._sim_time)

    @staticmethod
    def _time_at(seconds: float):
        from builtin_interfaces.msg import Time

        seconds = max(0.0, seconds)
        stamp = Time()
        stamp.sec = int(seconds)
        stamp.nanosec = int((seconds - int(seconds)) * 1e9)
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
        from .sensor_rig import (
            TORSO_TO_MID360_RPY,
            TORSO_TO_MID360_XYZ,
            rpy_to_quat_xyzw,
        )

        tf3d = TransformStamped()
        tf3d.header.stamp = self._now()
        tf3d.header.frame_id = FRAME_BASE
        tf3d.child_frame_id = FRAME_LIDAR3D
        tf3d.transform.translation.x = TORSO_TO_MID360_XYZ[0]
        tf3d.transform.translation.y = TORSO_TO_MID360_XYZ[1]
        tf3d.transform.translation.z = TORSO_TO_MID360_XYZ[2]
        qx, qy, qz, qw = rpy_to_quat_xyzw(*TORSO_TO_MID360_RPY)
        tf3d.transform.rotation.x = qx
        tf3d.transform.rotation.y = qy
        tf3d.transform.rotation.z = qz
        tf3d.transform.rotation.w = qw
        self._static_tf_broadcaster.sendTransform([tf, tf3d])
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

    def publish_points(self, points_sensor) -> None:
        """3D LiDAR の点群を PointCloud2 として配信する。

        Args:
            points_sensor: センサ座標系の点群 (N, 3)。torch.Tensor を想定。

        点群はセンサ座標系（frame_id=lidar3d）で送る。ワールド座標へは
        購読側が TF を使って変換するため、こちらでは変換しない。
        """
        import numpy as np

        msg = self._PointCloud2()
        # Livoxのheaderはフレーム先頭時刻、各点timestampはそこからのoffset。
        msg.header.stamp = self._time_at(self._sim_time - 0.1)
        msg.header.frame_id = FRAME_LIVOX

        # torch.Tensor -> numpy float32 (N, 3)
        if hasattr(points_sensor, "detach"):
            pts = points_sensor.detach().cpu().numpy().astype(np.float32)
        else:
            pts = np.asarray(points_sensor, dtype=np.float32)

        num_points = int(pts.shape[0])

        # 順序なし点群として送る（height=1 の「並びに意味が無い」形式）
        msg.height = 1
        msg.width = num_points
        msg.is_dense = True  # 無効な点は除外済みなので NaN は含まれない
        msg.is_bigendian = False

        fields = []
        for i, name in enumerate(("x", "y", "z")):
            field = self._PointField()
            field.name = name
            field.offset = 4 * i
            field.datatype = self._PointField.FLOAT32
            field.count = 1
            fields.append(field)
        fields.extend(
            [
                self._PointField(
                    name="intensity", offset=12, datatype=self._PointField.FLOAT32, count=1
                ),
                self._PointField(
                    name="tag", offset=16, datatype=self._PointField.UINT8, count=1
                ),
                self._PointField(
                    name="line", offset=17, datatype=self._PointField.UINT8, count=1
                ),
                self._PointField(
                    name="timestamp", offset=18, datatype=self._PointField.FLOAT64, count=1
                ),
            ]
        )
        msg.fields = fields

        msg.point_step = 26
        msg.row_step = msg.point_step * num_points
        data = bytearray(msg.row_step)
        denominator = max(1, num_points - 1)
        for index, (x, y, z) in enumerate(pts):
            point_time = 0.1 * index / denominator
            struct.pack_into(
                "<ffffBBd",
                data,
                index * msg.point_step,
                float(x),
                float(y),
                float(z),
                100.0,
                0,
                index % 4,
                point_time,
            )
        msg.data = bytes(data)

        self._points_pub.publish(msg)
        legacy = copy.deepcopy(msg)
        legacy.header.frame_id = FRAME_LIDAR3D
        self._legacy_points_pub.publish(legacy)

    def publish_imu(self, angular_velocity, linear_acceleration) -> None:
        """200HzのLiDAR内蔵IMU相当データを配信する。"""

        message = self._Imu()
        message.header.stamp = self._now()
        message.header.frame_id = FRAME_LIVOX
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.x = float(angular_velocity[0])
        message.angular_velocity.y = float(angular_velocity[1])
        message.angular_velocity.z = float(angular_velocity[2])
        message.linear_acceleration.x = float(linear_acceleration[0])
        message.linear_acceleration.y = float(linear_acceleration[1])
        message.linear_acceleration.z = float(linear_acceleration[2])
        self._imu_pub.publish(message)

    def publish_camera(self, rgb, intrinsic, position, orientation) -> None:
        """RGB JPEG、CameraInfo、metadata、評価用真値姿勢を同時配信する。"""

        import numpy as np
        from PIL import Image

        pixels = rgb.detach().cpu().numpy() if hasattr(rgb, "detach") else np.asarray(rgb)
        if pixels.dtype != np.uint8:
            maximum = float(np.nanmax(pixels)) if pixels.size else 0.0
            if maximum <= 1.0:
                pixels = pixels * 255.0
            pixels = np.clip(pixels, 0.0, 255.0).astype(np.uint8)
        output = io.BytesIO()
        Image.fromarray(pixels, mode="RGB").save(output, format="JPEG", quality=90)

        image = self._CompressedImage()
        image.header.stamp = self._now()
        image.header.frame_id = FRAME_CAMERA
        image.format = "rgb8; jpeg compressed rgb8"
        image.data = output.getvalue()
        self._image_pub.publish(image)

        matrix = intrinsic.detach().cpu().numpy() if hasattr(intrinsic, "detach") else np.asarray(intrinsic)
        info = self._CameraInfo()
        info.header = image.header
        info.width = int(pixels.shape[1])
        info.height = int(pixels.shape[0])
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [float(value) for value in matrix.reshape(-1)]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            info.k[0], 0.0, info.k[2], 0.0,
            0.0, info.k[4], info.k[5], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self._camera_info_pub.publish(info)

        metadata = self._String()
        stamp_ns = image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec
        metadata.data = json.dumps(
            {
                "schema_version": 1,
                "sequence": self._camera_sequence,
                "stamp_ns": stamp_ns,
                "timestamp_source": "simulation_clock",
                "width": info.width,
                "height": info.height,
                "calibration_complete": True,
            },
            separators=(",", ":"),
        )
        self._camera_metadata_pub.publish(metadata)
        self._camera_sequence += 1

        pose = self._PoseStamped()
        pose.header = image.header
        pose.header.frame_id = "sim_world"
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])
        self._ground_truth_camera_pub.publish(pose)

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

    def publish_ground_truth(self, state: GroundTruthState) -> None:
        """アルゴリズム入力から隔離した評価用6DoF odometryを配信する。"""

        message = self._Odometry()
        message.header.stamp = self._now()
        message.header.frame_id = "sim_world"
        message.child_frame_id = "sim_base_link"
        message.pose.pose.position.x = state.position[0]
        message.pose.pose.position.y = state.position[1]
        message.pose.pose.position.z = state.position[2]
        message.pose.pose.orientation.x = state.orientation[0]
        message.pose.pose.orientation.y = state.orientation[1]
        message.pose.pose.orientation.z = state.orientation[2]
        message.pose.pose.orientation.w = state.orientation[3]
        message.twist.twist.linear.x = state.linear_velocity[0]
        message.twist.twist.linear.y = state.linear_velocity[1]
        message.twist.twist.linear.z = state.linear_velocity[2]
        message.twist.twist.angular.x = state.angular_velocity[0]
        message.twist.twist.angular.y = state.angular_velocity[1]
        message.twist.twist.angular.z = state.angular_velocity[2]
        self._ground_truth_odom_pub.publish(message)

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
