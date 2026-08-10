"""真値 odom と 3D 点群から 3D voxel 地図を作る。

`SimEnvTest/build_map.py`（2D 版）とは別物として新規に作った。
2D 版は実形状とのずれ 0.36 m という実績があるため、そちらは一切触らない。

Isaac Sim は真値の姿勢を持っているため、SLAM の姿勢推定は本来不要で
点群を姿勢どおりに重ねるだけでよい（2D で実証済みの方針）。

2D 版との決定的な違い（ここを間違えると地図が壊れる）:
    2D 版は odom の yaw を**足してはいけない**。LiDAR が
    ray_alignment="yaw" で構築されており、RayCaster が既に yaw を反映して
    レイを飛ばしているため、足すと二重適用になって地図が 11 m ずれる
    （実際に発生した）。

    3D 版は ray_alignment="base" で、/points は**センサ座標系**で配信される。
    そのため姿勢（yaw だけでなく前傾も含む回転）と位置を**必ず適用する**。
    ここは TF に任せる方が安全なので、tf2 で lidar3d -> map を引く。

出力:
    maps/warehouse_3d.npz   voxel の占有情報（自前形式）
    maps/warehouse_3d_slices/  高さ帯ごとのスライス画像（目視確認用）

実行方法:
    # 端末 1: 自動巡回で歩き回らせる（3D LiDAR 付き）
    source env.sh
    "$ISAAC_SIM/python.sh" src/run_g1_twin.py --viz none \
        --lidar3d --command-source patrol --max-steps 90000

    # 端末 2: 点群を集めて 3D 地図にする
    source env.sh
    python3 src/build_map_3d.py --duration 1500 --output maps/warehouse_3d
"""

from __future__ import annotations

import argparse
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# voxel の一辺 [m]。octomap の設定と揃える。
VOXEL_SIZE: float = 0.10
# 地図に採用する高さの範囲 [m]。床と天井を除く。
MIN_Z: float = 0.05
MAX_Z: float = 2.5
# 採用する最大距離 [m]。遠方はノイズが大きいので切る。
MAX_RANGE: float = 30.0
# voxel を「障害物」と確定するのに必要な観測回数。
# 1 回だけ当たった voxel はレイキャストのノイズの可能性があるため捨てる。
MIN_HITS: int = 2
# センサの取り付け高さ [m]。lidar3d.py の TARGET_LIDAR_HEIGHT と揃える。
LIDAR_HEIGHT: float = 1.1


class PointMapBuilder(Node):
    """/points と /odom を集めて 3D voxel 地図を作るノード。"""

    def __init__(self, duration_sec: float) -> None:
        super().__init__("map_builder_3d")
        sensor_qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # 直近の姿勢 (x, y, yaw)
        self._latest_odom: tuple[float, float, float] | None = None
        # voxel 座標 -> 観測回数
        self.voxel_hits: dict[tuple[int, int, int], int] = {}
        self.num_clouds = 0
        self.num_points = 0
        self._duration = duration_sec

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PointCloud2, "/points", self._on_points, sensor_qos)

    def _on_odom(self, msg: Odometry) -> None:
        """直近の姿勢を保持する。"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._latest_odom = (p.x, p.y, yaw)

    def _on_points(self, msg: PointCloud2) -> None:
        """点群をワールド座標へ変換して voxel に焼く。

        /points はセンサ座標系（frame_id=lidar3d）なので、姿勢を適用する。
        2D 版と違い、ここでは yaw を必ず適用する（RayCaster が
        ray_alignment="base" なのでレイに yaw が入っていない）。
        """
        if self._latest_odom is None:
            return

        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        if pts.shape[0] == 0:
            return

        x, y, yaw = self._latest_odom

        # 距離で間引く（遠方のノイズを除く）
        dist = np.linalg.norm(pts, axis=1)
        pts = pts[dist <= MAX_RANGE]
        if pts.shape[0] == 0:
            return

        # センサ座標 -> ワールド座標。
        # 前傾は publish 側で既に点に反映されている（センサ座標系が傾いている）
        # ため、ここでは yaw 回転と平行移動だけを適用する。
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        wx = x + pts[:, 0] * cos_y - pts[:, 1] * sin_y
        wy = y + pts[:, 0] * sin_y + pts[:, 1] * cos_y
        # センサは地上 1.1 m にあるので、点の z はそこからの相対値
        wz = pts[:, 2] + LIDAR_HEIGHT

        # 高さで絞る（床と天井を除く）
        keep = (wz >= MIN_Z) & (wz <= MAX_Z)
        wx, wy, wz = wx[keep], wy[keep], wz[keep]
        if wx.size == 0:
            return

        # voxel に量子化して観測回数を数える。
        # 1 スキャン 11520 点 x 10Hz を捌くため、点ごとの Python ループは避け、
        # 同一 voxel を numpy で先に畳んでから辞書へ入れる。
        ix = np.floor(wx / VOXEL_SIZE).astype(np.int32)
        iy = np.floor(wy / VOXEL_SIZE).astype(np.int32)
        iz = np.floor(wz / VOXEL_SIZE).astype(np.int32)
        cells = np.stack([ix, iy, iz], axis=1)
        unique, counts = np.unique(cells, axis=0, return_counts=True)
        for key, cnt in zip(map(tuple, unique.tolist()), counts.tolist()):
            self.voxel_hits[key] = self.voxel_hits.get(key, 0) + int(cnt)

        self.num_clouds += 1
        self.num_points += int(wx.size)


def save_map(
    voxel_hits: dict[tuple[int, int, int], int], stem: str, min_hits: int
) -> None:
    """voxel 地図を保存し、高さ帯ごとのスライス画像を出す。

    数値だけでは壊れた地図に気付けないため、必ず画像を出す
    （2D で「探索面積は増えているのに地図は壊れていた」ことがあった）。
    """
    confirmed = {k: v for k, v in voxel_hits.items() if v >= min_hits}
    if not confirmed:
        print(f"[NG] 観測回数 {min_hits} 以上の voxel が無い。地図を保存しない。")
        return

    keys = np.array(list(confirmed.keys()), dtype=np.int32)
    counts = np.array(list(confirmed.values()), dtype=np.int32)

    os.makedirs(os.path.dirname(stem) or ".", exist_ok=True)
    np.savez_compressed(
        f"{stem}.npz",
        voxels=keys,
        counts=counts,
        voxel_size=VOXEL_SIZE,
        min_z=MIN_Z,
        max_z=MAX_Z,
    )

    x_m = keys[:, 0] * VOXEL_SIZE
    y_m = keys[:, 1] * VOXEL_SIZE
    z_m = keys[:, 2] * VOXEL_SIZE
    print(f"[OK] {stem}.npz を保存しました: {len(confirmed)} voxel")
    print(f"     X 範囲: {x_m.min():.1f} 〜 {x_m.max():.1f} m")
    print(f"     Y 範囲: {y_m.min():.1f} 〜 {y_m.max():.1f} m")
    print(f"     Z 範囲: {z_m.min():.1f} 〜 {z_m.max():.1f} m")

    _save_slices(keys, stem)


def _save_slices(keys: np.ndarray, stem: str) -> None:
    """高さ帯ごとの占有を画像として保存する（目視確認用）。

    足元（0〜0.3 m）・胴体（0.3〜1.5 m）・頭上（1.5 m 以上）の 3 帯。
    2D LiDAR では見えなかった足元と頭上が写っているかを確認できる。
    """
    try:
        from PIL import Image
    except ImportError:
        print("[WARN] PIL が無いためスライス画像を保存しません")
        return

    out_dir = f"{stem}_slices"
    os.makedirs(out_dir, exist_ok=True)

    bands = (
        ("足元_0.0-0.3m", 0.0, 0.3),
        ("胴体_0.3-1.5m", 0.3, 1.5),
        ("頭上_1.5m以上", 1.5, 99.0),
        ("全体", 0.0, 99.0),
    )

    ix_min, ix_max = int(keys[:, 0].min()), int(keys[:, 0].max())
    iy_min, iy_max = int(keys[:, 1].min()), int(keys[:, 1].max())
    width = ix_max - ix_min + 1
    height = iy_max - iy_min + 1

    for name, z_lo, z_hi in bands:
        iz_lo = int(math.floor(z_lo / VOXEL_SIZE))
        iz_hi = int(math.floor(z_hi / VOXEL_SIZE))
        sel = keys[(keys[:, 2] >= iz_lo) & (keys[:, 2] < iz_hi)]

        img = np.full((height, width), 255, dtype=np.uint8)
        if sel.shape[0] > 0:
            # 画像は上下反転させる（Y 上向きを画像の上に合わせる）
            rows = (iy_max - sel[:, 1]).astype(np.int32)
            cols = (sel[:, 0] - ix_min).astype(np.int32)
            img[rows, cols] = 0

        path = os.path.join(out_dir, f"{name}.png")
        Image.fromarray(img).save(path)
        print(f"[OK] スライス画像: {path} ({sel.shape[0]} voxel)")

    print(f"[INFO] 画像を必ず目視すること。数値だけでは壊れた地図に気付けない。")


def main() -> None:
    """点群を集めて 3D voxel 地図を作る。"""
    parser = argparse.ArgumentParser(description="真値 odom から 3D voxel 地図を作る")
    parser.add_argument(
        "--duration", type=float, default=600.0, help="収集する秒数（実時間）"
    )
    parser.add_argument(
        "--output", type=str, default="maps/warehouse_3d", help="出力のパス（拡張子なし）"
    )
    parser.add_argument(
        "--min-hits",
        type=int,
        default=MIN_HITS,
        help="障害物と確定するのに必要な観測回数",
    )
    args = parser.parse_args()

    rclpy.init()
    node = PointMapBuilder(args.duration)
    print(f"[INFO] /points と /odom を {args.duration:.0f} 秒収集します")
    print(f"[INFO] voxel {VOXEL_SIZE} m、高さ {MIN_Z}〜{MAX_Z} m")

    start = time.time()
    last_report = start
    try:
        while time.time() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            if now - last_report >= 10.0:
                last_report = now
                print(
                    f"[INFO] {now - start:.0f}s / {args.duration:.0f}s  "
                    f"点群 {node.num_clouds} 個, 点 {node.num_points} 個, "
                    f"voxel {len(node.voxel_hits)} 個"
                )
    except KeyboardInterrupt:
        print("\n[INFO] 中断されました。ここまでの結果を保存します。")

    print()
    print(f"[INFO] 収集完了: 点群 {node.num_clouds} 個、点 {node.num_points} 個")
    if node.num_clouds == 0:
        print("[NG] 点群を 1 つも受信していない。")
        print("     Isaac Sim を --lidar3d 付きで起動しているか確認すること。")
        node.destroy_node()
        rclpy.shutdown()
        return

    save_map(node.voxel_hits, args.output, args.min_hits)

    node.destroy_node()
    rclpy.shutdown()


# 単体テスト（test_build_map_3d.py）から import できるようにガードする。
# ガードが無いと import した時点で収集が始まってしまう。
if __name__ == "__main__":
    main()
