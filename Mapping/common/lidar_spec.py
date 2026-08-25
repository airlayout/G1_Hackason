"""3D LiDAR のビーム配置（どの方向にレーザーを飛ばすか）の定義。

sim と real で共通の「LiDAR とはどういうセンサーか」の記述をここに置く。
`sim/mujoco_lidar.py` はこの方向ベクトル群を MuJoCo のレイキャストに渡し、
実機では同じ spec が「受け取った点群がどのくらいの密度・範囲か」の期待値になる。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LidarSpec:
    """回転式 3D LiDAR のビーム配置。

    既定は「水平 360 度・垂直 -25〜+15 度・32 チャンネル」の汎用的な回転式 LiDAR
    （Velodyne VLP-32 / Ouster OS1 に近い配置）。実機に載っている機種が未確定なので、
    屋内マッピングで素直に扱える上下対称寄りの FOV を既定にしている。
    実機が Livox Mid-360 であることが確認できたら `livox_mid360()` に差し替える。
    """

    n_channels: int = 32
    """垂直方向のビーム本数。"""

    elev_min_deg: float = -25.0
    """垂直 FOV の下端[deg]（センサー座標系。正が上向き）。"""

    elev_max_deg: float = 15.0
    """垂直 FOV の上端[deg]。"""

    n_azimuth: int = 360
    """水平 1 周あたりのビーム本数。既定は 1 度刻み。"""

    max_range_m: float = 40.0
    """最大測距距離[m]。これを超える点は「反射なし」として捨てる。"""

    @classmethod
    def livox_mid360(cls) -> "LidarSpec":
        """Livox Mid-360 相当（水平 360 度・垂直 -7〜+52 度）。

        Mid-360 は「低い位置に置いて上方を広く見る」設計で FOV が上に強く偏っている。
        胴体（地上 1.3m 前後）に水平取り付けすると、下端 -7 度は 10m 先でようやく
        床に届くため、部屋の床がほとんど写らない。かといって下向きに傾けると
        360 度の裏側が同じ角度だけ上を向き、後方が空を向いて何も返らなくなる
        （実際にピッチ 30 度で試して、前方しか点が取れないことを確認した）。
        この機種を使うなら「低い位置に取り付ける」ことが前提になる。

        実機は非反復スキャン（フレームごとにビーム位置が変わる）だが、ここでは
        検証の再現性を優先して等間隔グリッドで近似している。
        """
        return cls(n_channels=40, elev_min_deg=-7.0, elev_max_deg=52.0)

    @property
    def n_rays(self) -> int:
        return self.n_channels * self.n_azimuth

    def ray_directions(self) -> np.ndarray:
        """センサー座標系での単位方向ベクトル `(n_rays, 3)` を返す。

        センサー座標系は +x が正面、+z が上（MuJoCo / ROS と同じ右手系）。
        戻り値の並びは「垂直チャンネル順 → その中で方位角順」。
        """
        elev = np.deg2rad(np.linspace(self.elev_min_deg, self.elev_max_deg, self.n_channels))
        azim = np.deg2rad(np.linspace(0.0, 360.0, self.n_azimuth, endpoint=False))
        e, a = np.meshgrid(elev, azim, indexing="ij")
        cos_e = np.cos(e)
        dirs = np.stack([cos_e * np.cos(a), cos_e * np.sin(a), np.sin(e)], axis=-1)
        return dirs.reshape(-1, 3).astype(np.float64)
