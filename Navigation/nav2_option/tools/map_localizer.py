#!/usr/bin/env python3
"""保存地図に対して自己位置を継続的に補正する（`map→odom` を動的に更新する）。

## なぜ要るのか

G1 内蔵 SLAM の odometry は LiDAR+IMU の**実測**で、静止 70 秒のドリフトは 0.9cm と
優秀（`findings/g1_dds_sensors.md` §8）。しかし**保存地図に対する補正が入らない**ので、
誤差は一方的に積み上がり、戻らない。

`g1_slam_odom_tf.py` は `map→odom` を**起動時に決めた固定値**として出す。
つまり「出発時に地図と合わせて、あとは歩数で推測する」状態になる。
本ノードはそこに**地図との照合**を足し、`map→odom` を走行中も更新し続ける。

⚠️ **`g1_slam_odom_tf.py` は `--no-map-to-odom` を付けて起動すること。**
同じ親子関係を静的と動的の両方で出すと、tf2 は静的側を「常に最新」として扱うため
**補正が一切効かなくなる**（しかもエラーは出ない）。

## どうやって照合するか — 相関型スキャンマッチング

現在の推定の**周りだけ**を探す局所探索にしてある。大域探索
（`match_scan_to_map_2d.py` の FFT 相関）は初期合わせ用で、毎周期回すには重い。

1. 保存地図（`/map` の OccupancyGrid）の占有セルから**距離場**を作る（起動時に1回）
2. 点群を高さで絞って 2D に落とす
3. `(dx, dy, dyaw)` の小さな窓を走査し、**距離場の値の合計が最小**になる補正を選ぶ
4. 粗く探してから細かく探す（2 段階）

⚠️ **対応点を取らない**ので、ICP のように初期値が悪いと発散する、ということが無い。
窓の外へは原理的に動かないので、**暴れない**ことが保証できる。

## 安全側の作り

**位置推定が誤ると、ロボットは壁に向かって自信満々に歩く。**
だから「分からないときは動かさない」を徹底する。

- 照合スコアが閾値より悪ければ**採用しない**（前回の補正を保つ）
- 1 回の補正量に上限を設ける（`--max-step-m` / `--max-step-deg`）。
  **大きく飛ぶ補正は、正しくても危ない**（controller が急な経路変更を強いられる）
- 補正は指数移動平均で滑らかにする（`--smoothing`）
- 状態を `/g1/localizer_status`（`diagnostic_msgs/DiagnosticArray`）に出す。
  **採用したか棄却したか、スコアはいくつか**が rosbag から追える

## 使い方

    # g1_slam_odom_tf.py 側で map→odom を止める
    python3 g1_slam_odom_tf.py --no-map-to-odom &
    python3 map_localizer.py --initial 1.2 -3.4 0.15

⚠️ **`--initial` に初期位置を渡すこと。** 局所探索なので、出発点が地図上の
どこかを大まかに知っている必要がある。`match_scan_to_map_2d.py` で求めた値を渡す。
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def cloud_xyz(msg: PointCloud2) -> np.ndarray | None:
    """PointCloud2 から x,y,z だけを numpy 配列で取り出す。

    ⚠️ **`sensor_msgs_py.point_cloud2.read_points_numpy` は使えない。**
    MID-360 の点群は `x,y,z`(float32) と `intensity`/`tag`/`line`(uint8 等)が混在しており、
    あのヘルパは**選択した列だけでなく全フィールドの型が同一であることを要求する**ため
    `AssertionError: All fields need to have the same datatype` で落ちる
    (2026-09-13 に実測で踏んだ)。

    宣言された offset を使って直接読む。間引き前の全点を扱うので、
    構造化配列を経由するより速いという利点もある。
    """
    if msg.is_bigendian:
        return None  # 実機は little endian。来たら扱わない(黙って誤動作させない)
    fields = {f.name: f for f in msg.fields}
    if not all(n in fields for n in ("x", "y", "z")):
        return None
    if not all(fields[n].datatype == PointField.FLOAT32 for n in ("x", "y", "z")):
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    n_pts = len(raw) // msg.point_step
    if n_pts == 0:
        return None
    raw = raw[: n_pts * msg.point_step].reshape(n_pts, msg.point_step)
    out = np.empty((n_pts, 3), dtype=np.float32)
    for i, name in enumerate(("x", "y", "z")):
        off = fields[name].offset
        out[:, i] = raw[:, off:off + 4].copy().view(np.float32).ravel()
    return out[np.isfinite(out).all(axis=1)]


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def distance_field(occupied: np.ndarray, resolution: float) -> np.ndarray:
    """占有セルからの距離(メートル)を返す。

    `scipy.ndimage.distance_transform_edt` が在ればそれを使う。無ければ
    **2 パスのチャンファー距離**で近似する（実機 PC2 に scipy が無くても動くように）。
    """
    try:
        from scipy.ndimage import distance_transform_edt
        return distance_transform_edt(~occupied).astype(np.float32) * resolution
    except ImportError:
        pass
    big = np.float32(1e6)
    d = np.where(occupied, np.float32(0.0), big)
    h, w = d.shape
    # 前向き
    for r in range(h):
        for c in range(w):
            v = d[r, c]
            if r > 0:
                v = min(v, d[r - 1, c] + 1.0)
                if c > 0:
                    v = min(v, d[r - 1, c - 1] + 1.41421356)
                if c + 1 < w:
                    v = min(v, d[r - 1, c + 1] + 1.41421356)
            if c > 0:
                v = min(v, d[r, c - 1] + 1.0)
            d[r, c] = v
    # 後ろ向き
    for r in range(h - 1, -1, -1):
        for c in range(w - 1, -1, -1):
            v = d[r, c]
            if r + 1 < h:
                v = min(v, d[r + 1, c] + 1.0)
                if c > 0:
                    v = min(v, d[r + 1, c - 1] + 1.41421356)
                if c + 1 < w:
                    v = min(v, d[r + 1, c + 1] + 1.41421356)
            if c + 1 < w:
                v = min(v, d[r, c + 1] + 1.0)
            d[r, c] = v
    return d * resolution


class MapLocalizer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("map_localizer")
        self._args = args
        # 現在の map→odom 推定 (x, y, yaw)
        self._m2o = np.array(args.initial, dtype=np.float64)
        self._field: np.ndarray | None = None
        self._origin = (0.0, 0.0)
        self._resolution = 0.05
        self._accepted = 0
        self._rejected = 0
        self._m2o_initial = self._m2o.copy()   # 累積補正量を測る基準
        self._last_score: float | None = None
        self._last_reason = "起動直後"

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        # 地図は transient local で配信される。遅れて起動しても受け取れる
        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, map_qos)
        # 点群は best effort で出る。Reliable にすると**一切届かない**
        cloud_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, cloud_qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/g1/localizer_status", 10)
        # ⚠️ **TF の供給を止めずにモードだけ変える**ための口。プロセスを入れ替えると
        # 必ず TF が途切れ、`tf_stale` で FAULT になる（2026-09-24 にモックで実測）。
        self._mode = args.mode
        self.create_service(SetBool, "/g1/localizer/correct", self._srv_mode)

        # map→odom は**補正が無くても出し続ける**。出さないと TF が途切れ、
        # Nav2 も cmd_router の鮮度監視も止まってしまう(A-10h)。
        self.create_timer(1.0 / args.tf_rate, self._publish_tf)
        self.create_timer(1.0, self._publish_diag)
        self._last_cloud_stamp = None

        self.get_logger().info(
            f"起動: 地図={args.map_topic} 点群={args.cloud_topic} "
            f"初期 map->odom=({self._m2o[0]:.3f}, {self._m2o[1]:.3f}, "
            f"{math.degrees(self._m2o[2]):.1f}°)")

    # --- 地図 ---------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        if self._field is not None:
            return
        grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        # ⚠️ **未知(-1)を占有として扱わない。** 未観測を壁と見なすと、
        # 地図の外周全部に引き寄せられて補正が壊れる。
        occupied = grid >= self._args.occupied_thresh
        if not occupied.any():
            self.get_logger().error("地図に占有セルが1つも無い。地図を確認すること")
            return
        self._resolution = msg.info.resolution
        self._origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.get_logger().info(
            f"地図を受け取った {msg.info.width}x{msg.info.height} "
            f"({self._resolution}m/cell) 占有セル {int(occupied.sum())} → 距離場を作る")
        self._field = distance_field(occupied, self._resolution)
        self.get_logger().info("距離場の作成が終わった")

    # --- 点群 ---------------------------------------------------------------
    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._field is None:
            return
        # 間引き: 毎フレーム照合する必要は無い
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_cloud_stamp is not None and \
                (stamp - self._last_cloud_stamp) < (1.0 / self._args.match_rate):
            return
        self._last_cloud_stamp = stamp

        pts = self._cloud_in_odom(msg)
        if pts is None or len(pts) < self._args.min_points:
            self._last_reason = f"点が少ない({0 if pts is None else len(pts)})"
            self._rejected += 1
            return
        self._match(pts)

    def _cloud_in_odom(self, msg: PointCloud2) -> np.ndarray | None:
        """点群を odom 系の 2D 点に落とす。

        ⚠️ **map 系ではなく odom 系に落とす。** 補正したいのは map→odom そのものなので、
        それを含む変換を使うと自分の推定で自分を評価することになる。
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                "odom", msg.header.frame_id, rclpy.time.Time())
        except Exception as exc:  # TF がまだ揃っていない
            self._last_reason = f"TF 取得できず: {exc}"
            return None
        arr = cloud_xyz(msg)
        if arr is None or arr.size == 0:
            self._last_reason = "点群を解釈できない"
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        # 回転行列(クォータニオン→3x3)
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)
        pts = arr @ rot.T + np.array([t.x, t.y, t.z])
        # 高さで絞る。床と天井を落として壁だけを残す
        m = (pts[:, 2] >= self._args.min_height) & (pts[:, 2] <= self._args.max_height)
        pts = pts[m][:, :2]
        if len(pts) > self._args.max_points:
            idx = np.random.choice(len(pts), self._args.max_points, replace=False)
            pts = pts[idx]
        return pts

    # --- 照合 ---------------------------------------------------------------
    def _score(self, pts_odom: np.ndarray, cand: np.ndarray) -> float:
        """候補の map→odom で点群を map 系に写し、距離場の平均値を返す(小さいほど良い)。"""
        c, s = math.cos(cand[2]), math.sin(cand[2])
        xs = c * pts_odom[:, 0] - s * pts_odom[:, 1] + cand[0]
        ys = s * pts_odom[:, 0] + c * pts_odom[:, 1] + cand[1]
        cols = np.floor((xs - self._origin[0]) / self._resolution).astype(np.int32)
        rows = np.floor((ys - self._origin[1]) / self._resolution).astype(np.int32)
        h, w = self._field.shape
        inside = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
        if inside.sum() < self._args.min_points:
            return float("inf")
        # ⚠️ 地図の外に出た点は**評価から外す**(ペナルティを与えない)。
        # 与えると「地図の外に逃げる」方向へ引っ張られる。
        # 代わりに、外に出た割合が大きすぎる候補は上で inf にしている。
        return float(self._field[rows[inside], cols[inside]].mean())

    def _srv_mode(self, req, res):
        """data=true → correct（補正する） / false → hold（初期値のまま出す）。"""
        self._mode = "correct" if req.data else "hold"
        self._last_reason = f"モードを {self._mode} にした"
        self.get_logger().info(f"モード: **{self._mode}**")
        res.success = True
        res.message = self._mode
        return res

    def _match(self, pts_odom: np.ndarray) -> None:
        if self._mode != "correct":
            self._last_reason = "hold（補正しない）"
            return
        base = self._m2o.copy()
        best = base.copy()
        best_score = self._score(pts_odom, base)

        # 粗→細の2段階。窓の外へは原理的に動かないので暴れない
        for step_m, step_deg, n in ((self._args.coarse_step_m, self._args.coarse_step_deg, 3),
                                    (self._args.fine_step_m, self._args.fine_step_deg, 2)):
            center = best.copy()
            for dx in np.arange(-n, n + 1) * step_m:
                for dy in np.arange(-n, n + 1) * step_m:
                    for dth in np.arange(-n, n + 1) * math.radians(step_deg):
                        cand = center + np.array([dx, dy, dth])
                        sc = self._score(pts_odom, cand)
                        if sc < best_score:
                            best_score, best = sc, cand
        self._last_score = best_score

        if not math.isfinite(best_score) or best_score > self._args.max_score:
            self._rejected += 1
            self._last_reason = f"スコアが悪い({best_score:.3f} > {self._args.max_score})"
            return

        delta = best - self._m2o
        dist = math.hypot(delta[0], delta[1])
        dyaw = abs(math.atan2(math.sin(delta[2]), math.cos(delta[2])))
        if dist > self._args.max_step_m or dyaw > math.radians(self._args.max_step_deg):
            # ⚠️ **大きく飛ぶ補正は、正しくても採用しない。**
            # 誤対応の可能性が高いうえ、正しくても controller が急な経路変更を強いられる。
            self._rejected += 1
            self._last_reason = (f"補正が大きすぎる({dist:.3f}m/"
                                 f"{math.degrees(dyaw):.1f}°)")
            return

        a = self._args.smoothing
        self._m2o = self._m2o + a * delta
        self._accepted += 1
        self._last_reason = "採用"
        self._log_row(best_score, dist, dyaw)

    def _log_row(self, score: float, step_m: float, step_rad: float) -> None:
        """1周期ぶんを CSV に残す。**累積補正量が「ずれの実測値」になる。**"""
        if not self._args.log_csv:
            return
        drift = self._m2o - self._m2o_initial
        new_file = not self._args.log_csv.exists()
        with self._args.log_csv.open("a", encoding="utf-8") as fp:
            if new_file:
                fp.write("t,score,step_m,step_deg,cum_dx,cum_dy,cum_dyaw_deg,cum_dist,"
                         "accepted,rejected\n")
            fp.write(f"{time.time():.3f},{score:.4f},{step_m:.4f},"
                     f"{math.degrees(step_rad):.3f},{drift[0]:.4f},{drift[1]:.4f},"
                     f"{math.degrees(drift[2]):.3f},{math.hypot(drift[0], drift[1]):.4f},"
                     f"{self._accepted},{self._rejected}\n")

    # --- 出力 ---------------------------------------------------------------
    def _publish_tf(self) -> None:
        # ⚠️ 観測専用では**出さない**。出すと §7 の静的 TF と二重になり、
        # どちらが効いているか分からないまま機体が動くことになる。
        if self._args.observe_only:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = float(self._m2o[0])
        t.transform.translation.y = float(self._m2o[1])
        qx, qy, qz, qw = yaw_quaternion(float(self._m2o[2]))
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self._tf_broadcaster.sendTransform(t)

    def _publish_diag(self) -> None:
        st = DiagnosticStatus()
        st.name = "map_localizer"
        st.hardware_id = "g1"
        ready = self._field is not None
        if not ready:
            st.level = DiagnosticStatus.WARN
            st.message = "地図待ち"
        elif self._last_reason == "採用":
            st.level = DiagnosticStatus.OK
            st.message = "localized"
        else:
            # ⚠️ 棄却が続くのは「補正できていない」ということ。**WARN で見えるようにする**
            st.level = DiagnosticStatus.WARN
            st.message = "not_updating"
        for k, v in (
            ("reason", self._last_reason),
            ("score", "-" if self._last_score is None else f"{self._last_score:.4f}"),
            ("accepted", str(self._accepted)),
            ("mode", self._mode),
            ("rejected", str(self._rejected)),
            ("map_to_odom",
             f"{self._m2o[0]:.3f}, {self._m2o[1]:.3f}, {math.degrees(self._m2o[2]):.2f}deg"),
        ):
            st.values.append(KeyValue(key=k, value=v))
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self._diag_pub.publish(arr)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--initial", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                   metavar=("X", "Y", "YAW_RAD"),
                   help="初期の map->odom。局所探索なので**おおよそ合っている必要がある**")
    p.add_argument("--map-topic", default="/map")
    p.add_argument("--cloud-topic", default="/utlidar/cloud_livox_mid360")
    p.add_argument("--occupied-thresh", type=int, default=65,
                   help="OccupancyGrid のこの値以上を占有と見なす(未知=-1 は含めない)")
    p.add_argument("--min-height", type=float, default=0.3,
                   help="この高さ未満の点を捨てる(床を除く)。odom 系の z")
    p.add_argument("--max-height", type=float, default=1.8,
                   help="この高さを超える点を捨てる(天井を除く)")
    p.add_argument("--min-points", type=int, default=200)
    p.add_argument("--max-points", type=int, default=1500,
                   help="これ以上は無作為に間引く(1周期の計算量を抑える)")
    # --- 観測専用（2026-09-24 追加）------------------------------------------
    # ⚠️ **既定の運用は「起動時に1回だけ合わせて以後固定」**（§7 の find_map_offset）。
    # つまり実質の補正頻度は**16〜18分に1回**（内蔵SLAM が落ちるたび）で、
    # AMCL 等の 1〜20Hz と比べて桁違いに薄い。ただし常時補正へいきなり切り替えると、
    # 補正のたびに姿勢が飛んで追従が乱れる恐れがある。
    # そこで **「計算はするが TF を出さない」** モードを用意した。巡回を回しながら
    # これを流せば、**走行に一切影響せずに「何 m ずれていくか」が数字になる。**
    p.add_argument("--mode", choices=("hold", "correct"), default="correct",
                   help="hold=**初期値のまま出すだけ**（従来の静的 map→odom と同じ）/ "
                        "correct=照合して補正する（既定）。"
                        "⚠️ **走行中に切り替えるならこれを使うこと。** 静的 TF と動的 TF の"
                        "入れ替えは tf2 の仕様上うまくいかない（静的はバッファに残り続け、"
                        "止めた瞬間に tf_stale で FAULT になる。2026-09-24 にモックで実測）")
    p.add_argument("--observe-only", action="store_true",
                   help="補正量を計算・記録するだけで **map→odom の TF を出さない**。"
                        "走行には影響しない（ずれの実測用）")
    p.add_argument("--log-csv", type=Path, default=None,
                   help="1周期ごとに時刻・スコア・累積補正量を CSV で残す")
    p.add_argument("--match-rate", type=float, default=2.0, help="照合する頻度[Hz]")
    p.add_argument("--tf-rate", type=float, default=20.0,
                   help="map->odom を出す頻度[Hz]。**照合できなくても出し続ける**")
    p.add_argument("--coarse-step-m", type=float, default=0.10)
    p.add_argument("--coarse-step-deg", type=float, default=2.0)
    p.add_argument("--fine-step-m", type=float, default=0.03)
    p.add_argument("--fine-step-deg", type=float, default=0.5)
    p.add_argument("--max-score", type=float, default=0.35,
                   help="距離場の平均がこれを超えたら採用しない[m]")
    p.add_argument("--max-step-m", type=float, default=0.30,
                   help="1回の補正量の上限[m]。大きく飛ぶ補正は正しくても採用しない")
    p.add_argument("--max-step-deg", type=float, default=5.0)
    p.add_argument("--smoothing", type=float, default=0.5,
                   help="補正の指数移動平均の係数(1.0 で即時反映)")
    return p


def main() -> None:
    parser = build_parser()
    rclpy.init()
    # ROS の引数を取り除いてから解析する
    import sys
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    node = MapLocalizer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
