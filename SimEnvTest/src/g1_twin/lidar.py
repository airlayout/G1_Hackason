"""G1 に搭載する 2D LiDAR。

SLAM (slam_toolbox) 用に 360 度の 2D スキャンを取得する。

実装方式について:
    Isaac Sim 標準の PhysX LiDAR (RangeSensorSchema) は、このビルドでは
    prim の作成に失敗するため使用できない。RangeSensorCreatePrim が
    Imageable でない Lidar prim の "visibility" 属性を設定しようとして
    例外になる（Empty typeName for </World/Lidar.visibility>）。
    加えて isaacsim.util.debug_draw が undefined symbol で読み込めず、
    PhysX LiDAR プラグインがセンサを認識しない。

    そのため IsaacLab の MultiMeshRayCaster を使う。Warehouse の
    Mesh 3473 個を対象にしても構築は 1 秒未満で、実用上の問題は無い。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch

# LiDAR の水平解像度 [deg]。360 / 1.0 = 360 ビーム。
# slam_toolbox は 1 度刻みでも十分な精度で地図を作れる。
HORIZONTAL_RES_DEG: float = 1.0
# 測定可能な最大距離 [m]。Warehouse の対角より長く取る。
MAX_RANGE: float = 30.0
# 測定可能な最小距離 [m]。自己の身体を拾わないための下限。
MIN_RANGE: float = 0.3
# G1 の胴体 (torso_link) は地上 0.753 m にある（実測値）。
# 地上 1.1 m に置きたいので、その差分を offset とする。
# 棚の下部が空洞なため、低すぎると棚の脚だけを拾って地図が穴だらけになる。
TORSO_HEIGHT: float = 0.753
TARGET_LIDAR_HEIGHT: float = 1.1
LIDAR_OFFSET_Z: float = TARGET_LIDAR_HEIGHT - TORSO_HEIGHT  # +0.347


@dataclass(frozen=True)
class ScanData:
    """1 スキャン分の距離データ。

    Attributes:
        ranges: 各ビームの距離 [m]。測距できないビームは inf。
        angle_min: 最初のビームの角度 [rad]
        angle_max: 最後のビームの角度 [rad]
        angle_increment: ビーム間の角度差 [rad]
        range_min: 有効な最小距離 [m]
        range_max: 有効な最大距離 [m]
    """

    ranges: list[float]
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float


class G1Lidar:
    """G1 の胴体に取り付ける 2D LiDAR。

    Isaac Sim のアプリ起動後に生成すること（isaaclab.sensors の import が必要）。
    """

    def __init__(
        self,
        robot_prim_path: str = "/World/G1",
        mesh_prim_paths: list[str] | None = None,
    ) -> None:
        """LiDAR を構築する。

        Args:
            robot_prim_path: G1 の prim パス
            mesh_prim_paths: raycast の対象とする prim。既定は Warehouse。
        """
        from isaaclab.sensors import MultiMeshRayCaster, MultiMeshRayCasterCfg, patterns

        # 2D スキャン: 1 層のみ、水平 360 度
        pattern = patterns.LidarPatternCfg(
            channels=1,
            vertical_fov_range=(0.0, 0.0),
            horizontal_fov_range=(-180.0, 180.0),
            horizontal_res=HORIZONTAL_RES_DEG,
        )

        cfg = MultiMeshRayCasterCfg(
            prim_path=f"{robot_prim_path}/torso_link",
            offset=MultiMeshRayCasterCfg.OffsetCfg(pos=(0.0, 0.0, LIDAR_OFFSET_Z)),
            # "base" は胴体の姿勢に完全に追従する（yaw のみでなく roll/pitch も）。
            # 歩行中は胴体が傾くため、その傾きも反映される。
            ray_alignment="base",
            pattern_cfg=pattern,
            mesh_prim_paths=mesh_prim_paths or ["/World/Warehouse"],
            max_distance=MAX_RANGE,
            debug_vis=False,
        )
        self._sensor = MultiMeshRayCaster(cfg)

        # ビームの並びは実測で確認済み:
        #   360 本、-180 度から +179 度まで 1 度刻み（+180 度は -180 度と重複するため無い）
        #   先頭ビーム dirs[0] = (-1, 0, 0) = センサ後方
        self._num_beams = int(360.0 / HORIZONTAL_RES_DEG)
        self._angle_increment = math.radians(HORIZONTAL_RES_DEG)
        self._angle_min = math.radians(-180.0)
        self._angle_max = self._angle_min + self._angle_increment * (self._num_beams - 1)

        print(
            f"[Lidar] 2D LiDAR を構築しました: {self._num_beams} ビーム, "
            f"最大 {MAX_RANGE} m, 地上 {TARGET_LIDAR_HEIGHT} m"
        )

    def update(self, dt: float) -> None:
        """センサを更新する。シミュレーションループから毎制御周期呼ぶ。"""
        self._sensor.update(dt, force_recompute=True)

    @property
    def position_w(self) -> torch.Tensor:
        """センサのワールド座標 (3,)。"""
        return self._sensor.data.pos_w[0]

    def read_scan(self) -> ScanData:
        """現在のスキャンを LaserScan 相当の形式で返す。

        RayCaster はワールド座標の当たり点を返すため、センサ原点からの
        距離に変換する。当たらなかったビームは inf になる。
        """
        hits_w = self._sensor.data.ray_hits_w[0]  # (B, 3)
        origin = self._sensor.data.pos_w[0]  # (3,)

        # 当たらなかったビームは inf が入るので、有限のものだけ距離を計算する
        finite = torch.isfinite(hits_w).all(dim=-1)
        distances = torch.full(
            (hits_w.shape[0],), float("inf"), device=hits_w.device, dtype=torch.float32
        )
        if finite.any():
            distances[finite] = torch.linalg.norm(
                hits_w[finite] - origin.unsqueeze(0), dim=-1
            )

        # 近すぎるものは自己の身体の可能性が高いので無効にする
        distances[distances < MIN_RANGE] = float("inf")

        return ScanData(
            ranges=distances.cpu().tolist(),
            angle_min=self._angle_min,
            angle_max=self._angle_max,
            angle_increment=self._angle_increment,
            range_min=MIN_RANGE,
            range_max=MAX_RANGE,
        )
