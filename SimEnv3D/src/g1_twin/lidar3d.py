"""G1 に搭載する 3D LiDAR（Livox Mid-360 相当）。

2D 版（lidar.py）との違い:
    - 垂直 FOV を持つ多層スキャン（Mid-360 相当の -7〜+52 度）
    - Unitree公式G1記述のtorso_link -> mid360_link姿勢で搭載する
    - ray_alignment="base"（2D とは逆。下記参照）
    - 出力は距離配列ではなく 3D 点群（PointCloud2 用）

ray_alignment について（2D とは逆にする）:
    2D では "yaw" が必須だった。歩行中の pitch/roll がレイに乗ると同じ方向の
    距離が 1 スキャンごとに 10 m 近く暴れ、SLAM が地図を作れなかった。

    3D では "base" が正しい。3D 点群は各点が 3 次元座標を持つため、センサが
    傾いても点は正しい位置に落ちる（2D は高さ情報を捨てるから壊れた）。
    実機の Mid-360 も胴体に固定されるので、こちらが実挙動に近い。

走査パターンは近似である:
    Mid-360 は非リピート型ロゼッタ走査だが、IsaacLab の LidarPatternCfg は
    等間隔グリッドしか生成できない。FOV と点数を合わせた等間隔グリッドで
    代用している。マッピング用途では実用上問題にならない。

性能について:
    mesh_prim_paths には親パスではなく Mesh を個別に列挙して渡す。
    親パスを渡すと 146 倍遅くなる（lidar.py の expand_mesh_paths 参照）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch

from .lidar import expand_mesh_paths
from .sensor_rig import (
    TORSO_TO_MID360_RPY,
    TORSO_TO_MID360_XYZ,
    rpy_to_quat_wxyz,
)

# Livox Mid-360 の垂直 FOV [deg]。実機 G1 の標準構成に合わせる。
VERTICAL_FOV_DEG: tuple[float, float] = (-7.0, 52.0)
# 垂直方向の層数。コスト実測の結果から決める（レイキャストはビーム数に
# ほぼ依存しないため、多めに取っても実行時間は変わらない）。
CHANNELS: int = 32
# 水平方向のビーム数（360 度を等分する）
HORIZONTAL_BEAMS: int = 625
# 32 x 625 x 10Hz = 200,000 points/s。走査形状は等間隔近似だが、
# 点数・FOV・フレーム周期はMid-360公称仕様へ合わせる。
FORWARD_TILT_DEG: float = math.degrees(TORSO_TO_MID360_RPY[1])

MAX_RANGE: float = 30.0
MIN_RANGE: float = 0.3

# G1 の胴体 (torso_link) は地上 0.753 m にある（実測値）。
LIDAR_OFFSET_Z: float = TORSO_TO_MID360_XYZ[2]
# 旧検証ツールが表示・投影計算に使う値。Mapping本体はセンサ姿勢を直接使う。
TORSO_HEIGHT: float = 0.745
TARGET_LIDAR_HEIGHT: float = TORSO_HEIGHT + LIDAR_OFFSET_Z


def tilt_quat_wxyz(tilt_deg: float) -> tuple[float, float, float, float]:
    """前傾（pitch 方向の回転）を表すクォータニオンを返す。

    IsaacLab の OffsetCfg.rot は `(w, x, y, z)` 順。ROS の geometry_msgs は
    `(x, y, z, w)` 順で異なるので取り違えないこと（過去にこの誤りで /odom が
    実際の向きを反映せず、Nav2 が旋回し続ける不具合を出した）。

    Args:
        tilt_deg: 前傾角 [deg]。正の値でセンサが下を向く。

    Returns:
        クォータニオン `(w, x, y, z)`
    """
    half = math.radians(tilt_deg) / 2.0
    # y 軸まわりの回転が pitch。正で下向き。
    return (math.cos(half), 0.0, math.sin(half), 0.0)


@dataclass(frozen=True)
class PointCloudData:
    """1 スキャン分の 3D 点群。

    Attributes:
        points_sensor: センサ座標系での当たり点 (N, 3)。無効なレイは除外済み。
        num_valid: 有効な点の数
        num_beams: 発射したレイの総数（当たり率の算出用）
    """

    points_sensor: torch.Tensor
    num_valid: int
    num_beams: int

    @property
    def hit_ratio(self) -> float:
        """当たったレイの割合 [0-1]。"""
        return self.num_valid / self.num_beams if self.num_beams else 0.0


class G1Lidar3D:
    """G1 の胴体に取り付ける 3D LiDAR（Mid-360 相当）。

    Isaac Sim のアプリ起動後、かつ sim.reset() より前に生成すること
    （センサの登録が reset 時に行われるため）。
    """

    def __init__(
        self,
        robot_prim_path: str = "/World/G1",
        mesh_prim_paths: list[str] | None = None,
        channels: int = CHANNELS,
        horizontal_beams: int = HORIZONTAL_BEAMS,
        tilt_deg: float = FORWARD_TILT_DEG,
    ) -> None:
        """3D LiDAR を構築する。

        Args:
            robot_prim_path: G1 の prim パス
            mesh_prim_paths: raycast の対象とする prim。既定は Warehouse。
                配下の Mesh は自動で個別に展開される。
            channels: 垂直方向の層数
            horizontal_beams: 水平方向のビーム数（360 度を等分）
            tilt_deg: 前傾角 [deg]
        """
        from isaaclab.sensors import MultiMeshRayCaster, MultiMeshRayCasterCfg, patterns

        # 親パスを渡すと毎スキャンで配下を走査し直して 146 倍遅くなるため、
        # Mesh を個別に列挙して渡す（実測: 467 ms -> 3.2 ms）
        targets = expand_mesh_paths(mesh_prim_paths or ["/World/Warehouse"])

        pattern = patterns.LidarPatternCfg(
            channels=channels,
            vertical_fov_range=VERTICAL_FOV_DEG,
            horizontal_fov_range=(-180.0, 180.0),
            horizontal_res=360.0 / horizontal_beams,
        )

        cfg = MultiMeshRayCasterCfg(
            prim_path=f"{robot_prim_path}/torso_link",
            offset=MultiMeshRayCasterCfg.OffsetCfg(
                pos=TORSO_TO_MID360_XYZ,
                rot=rpy_to_quat_wxyz(
                    TORSO_TO_MID360_RPY[0],
                    math.radians(tilt_deg),
                    TORSO_TO_MID360_RPY[2],
                ),
            ),
            # 3D では "base" が正しい（2D の "yaw" とは逆）。詳細は
            # モジュール冒頭の docstring を参照。
            ray_alignment="base",
            pattern_cfg=pattern,
            mesh_prim_paths=targets,
            max_distance=MAX_RANGE,
            debug_vis=False,
        )
        self._sensor = MultiMeshRayCaster(cfg)
        self._num_beams = channels * horizontal_beams
        self._tilt_deg = tilt_deg

        # 前傾によって実際に見える垂直範囲（下向きがどこまで確保できたか）
        down_deg = VERTICAL_FOV_DEG[0] - tilt_deg
        up_deg = VERTICAL_FOV_DEG[1] - tilt_deg
        # 下向き角から床が見え始める距離を出す（地上 TARGET_LIDAR_HEIGHT から）
        if down_deg < 0.0:
            floor_dist = 1.1 / math.tan(math.radians(-down_deg))
            floor_note = f"{floor_dist:.1f} m 先の床から見える"
        else:
            floor_note = "床は見えない"

        print(
            f"[Lidar3D] 3D LiDAR を構築しました: {channels} 層 x {horizontal_beams} "
            f"= {self._num_beams} ビーム, 最大 {MAX_RANGE} m"
        )
        print(
            f"[Lidar3D] 前傾 {tilt_deg} 度 -> 垂直 {down_deg:+.0f} 〜 {up_deg:+.0f} 度, "
            f"{floor_note}"
        )

    def update(self, dt: float) -> None:
        """センサを更新する。シミュレーションループから呼ぶ。"""
        self._sensor.update(dt, force_recompute=True)

    @property
    def position_w(self) -> torch.Tensor:
        """レイの実際の発射位置（ワールド座標 (3,)）。

        **`data.pos_w` をそのまま返してはいけない。** IsaacLab の RayCaster は
        `cfg.offset.pos` をレイの始点（`ray_starts`）にしか適用せず、
        `data.pos_w` には含めない（`ray_caster.py` の 217 行目と 233 行目）。

        実測（2026-08-10）:
            cfg.offset.pos の z   : +0.347
            ray_starts のローカル z: 0.347   <- 効いている
            data.pos_w の z        : 0.745   <- 含まない（torso_link と同じ）
            真の発射高さ           : 1.092   <- pos_w + offset

        `data.pos_w` を原点として距離を計算すると、足元の点で 0.32 m（28%）
        も過小評価する。実際にこのバグを入れた。
        """
        return self._sensor.data.pos_w[0] + self._offset_w()

    def _offset_w(self) -> torch.Tensor:
        """cfg.offset.pos をワールド座標系へ回した値 (3,) を返す。

        offset はセンサのローカル座標なので、姿勢で回してから足す必要がある。
        `ray_alignment="base"` では胴体の姿勢がそのまま乗る。
        """
        quat_w = self._sensor.data.quat_w[0]
        offset_local = torch.tensor(
            list(TORSO_TO_MID360_XYZ), device=quat_w.device, dtype=torch.float32
        )
        return _rotate_by_quat(offset_local.unsqueeze(0), quat_w)[0]

    @property
    def quat_w(self) -> torch.Tensor:
        """センサのワールド姿勢 (4,)。IsaacLab 規約の `(w, x, y, z)` 順。"""
        return self._sensor.data.quat_w[0]

    @property
    def num_beams(self) -> int:
        """発射するレイの総数。"""
        return self._num_beams

    def read_points_world(self) -> torch.Tensor:
        """ワールド座標系の当たり点 (N, 3) を返す。無効なレイは除外する。

        地図生成（build_map_3d.py）はワールド座標で点を積むため、こちらを使う。
        """
        hits_w = self._sensor.data.ray_hits_w[0]  # (B, 3)
        # position_w（offset を含む真の発射位置）を使う。data.pos_w を直接
        # 使うと距離が 0.32 m 過小になる（position_w の docstring 参照）。
        origin = self.position_w  # (3,)

        valid = self._valid_mask(hits_w, origin)
        return hits_w[valid]

    def read_point_cloud(self) -> PointCloudData:
        """センサ座標系の点群を返す。PointCloud2 の配信に使う。

        ROS の PointCloud2 は frame_id（センサのフレーム）基準の座標を持つため、
        ワールド座標から センサ座標へ変換する。

        重要（点群を使う側への注意）:
            ここでは **センサのワールド姿勢 quat_w の逆回転**を掛けるため、
            返る点群は yaw も前傾も歩行の pitch/roll も**すべて除かれた**
            真のセンサ座標系になる。したがって使う側はワールドへ戻すときに
            それらを適用しなければならない。yaw だけ適用して前傾を忘れると、
            10 m 先の点で高さが 3.42 m ずれる（実際にこのバグを入れた）。

            未解決の近似: base_link -> lidar3d の静的 TF は前傾しか持たない
            ため、歩行中の pitch/roll ぶんは TF に反映されない。octomap は
            この TF を使うので、歩行の揺れが誤差として入る。どの程度効くかは
            ステップ 4（"base" vs "yaw" の比較）で実測して判断する。
        """
        hits_w = self._sensor.data.ray_hits_w[0]  # (B, 3)
        # offset を含む真の発射位置を原点にする（data.pos_w だと 0.32 m ずれる）
        origin = self.position_w  # (3,)
        quat_w = self._sensor.data.quat_w[0]  # (4,) (w, x, y, z)

        valid = self._valid_mask(hits_w, origin)
        points_w = hits_w[valid]

        # ワールド -> センサ座標。共役クォータニオンで逆回転させる。
        offset = points_w - origin.unsqueeze(0)
        points_sensor = _rotate_by_quat_inverse(offset, quat_w)

        return PointCloudData(
            points_sensor=points_sensor,
            num_valid=int(points_sensor.shape[0]),
            num_beams=self._num_beams,
        )

    def _valid_mask(self, hits_w: torch.Tensor, origin: torch.Tensor) -> torch.Tensor:
        """有効な当たり点のマスクを返す。

        当たらなかったレイは inf が入る。近すぎるものは自己の身体を拾っている
        可能性があるため除外する。2D の LaserScan と違い、点群では「無効な点を
        含めない」だけでよい（inf/0.0 の規約問題は起きない）。
        """
        finite = torch.isfinite(hits_w).all(dim=-1)
        distances = torch.full(
            (hits_w.shape[0],), float("inf"), device=hits_w.device, dtype=torch.float32
        )
        if finite.any():
            distances[finite] = torch.linalg.norm(
                hits_w[finite] - origin.unsqueeze(0), dim=-1
            )
        return finite & (distances >= MIN_RANGE) & (distances <= MAX_RANGE)


def _rotate_by_quat_inverse(
    vectors: torch.Tensor, quat_wxyz: torch.Tensor
) -> torch.Tensor:
    """クォータニオンの逆回転をベクトル列に適用する。

    Args:
        vectors: (N, 3) のベクトル列
        quat_wxyz: (4,) のクォータニオン `(w, x, y, z)`

    Returns:
        回転後のベクトル列 (N, 3)
    """
    if vectors.shape[0] == 0:
        return vectors

    w, x, y, z = quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]
    # 逆回転 = 共役（虚部の符号を反転）
    conj = torch.stack([w, -x, -y, -z])
    return _rotate_by_quat(vectors, conj)


def _rotate_by_quat(vectors: torch.Tensor, quat_wxyz: torch.Tensor) -> torch.Tensor:
    """クォータニオン回転をベクトル列に適用する（v' = q v q*）。

    Args:
        vectors: (N, 3) のベクトル列
        quat_wxyz: (4,) のクォータニオン `(w, x, y, z)`

    Returns:
        回転後のベクトル列 (N, 3)
    """
    w = quat_wxyz[0]
    xyz = quat_wxyz[1:]
    # Rodrigues 形式: v' = v + 2w(u x v) + 2(u x (u x v))
    t = 2.0 * torch.linalg.cross(xyz.expand_as(vectors), vectors, dim=-1)
    return vectors + w * t + torch.linalg.cross(xyz.expand_as(vectors), t, dim=-1)
