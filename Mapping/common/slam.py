"""LiDAR スキャンの列から、自己位置推定と 3D 地図構築を同時に行う（SLAM）。

方式は scan-to-map ICP。1 フレーム前のスキャンとだけ合わせる scan-to-scan は
実装が軽い代わりに毎フレームの誤差がそのまま積算されるため、蓄積済みの地図全体に
対して合わせている（同じ壁を再訪したときに過去の観測が効いてドリフトが減る）。

ループクロージャ（一周して戻ってきたときに軌跡全体を補正する処理）は入っていない。
そのため長距離・長時間になるほどドリフトは残る。まずは「地図が形になるか」を
確かめる段階のため。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from .icp import IcpResult, icp
from .pointcloud import VoxelMap, transform_points, voxel_downsample


@dataclass
class SlamConfig:
    scan_voxel_size: float = 0.10
    """ICP に渡す前にスキャンを間引くボクセルサイズ[m]。小さいほど精度は上がるが遅い。"""

    map_voxel_size: float = 0.05
    """出力する地図の解像度[m]。"""

    icp_voxel_size: float = 0.15
    """ICP の相手（地図側）を間引くボクセルサイズ[m]。KD木の構築コストを抑えるため
    出力地図より粗くしている。"""

    max_correspondence_dist: float = 1.0
    max_iterations: int = 30

    min_fitness: float = 0.3
    """ICP の対応点率がこれを下回ったら位置合わせ失敗とみなし、等速度モデルの
    予測姿勢をそのまま採用する（明らかに壊れた姿勢を地図に焼き込まないため）。"""

    kdtree_rebuild_interval: int = 5
    """何フレームごとに地図の KD 木を作り直すか。毎フレーム作り直すと地図が
    大きくなるほど支配的なコストになるので間引く。"""


@dataclass
class SlamStats:
    """1 フレーム分の処理結果。あとで追従性能を評価するために残す。"""

    frame: int
    pose: np.ndarray
    fitness: float
    inlier_rmse: float
    iterations: int
    degraded: bool
    """ICP が信用できず等速度モデルにフォールバックしたフレームかどうか。"""


@dataclass
class LidarSlam:
    """スキャンを 1 フレームずつ食わせると、姿勢と地図を更新していく。"""

    config: SlamConfig = field(default_factory=SlamConfig)

    def __post_init__(self) -> None:
        self.map = VoxelMap(self.config.map_voxel_size)
        self.pose: np.ndarray = np.eye(4)
        # 直前フレーム間の相対移動。次フレームの初期値を等速度モデルで作るのに使う。
        self._last_delta: np.ndarray = np.eye(4)
        self._tree: cKDTree | None = None
        self._tree_points: np.ndarray = np.zeros((0, 3))
        self._frames_since_rebuild = 0
        self._frame = 0
        self.stats: list[SlamStats] = []

    def process(self, scan_sensor: np.ndarray) -> np.ndarray:
        """センサー座標系のスキャン `(N, 3)` を 1 フレーム処理し、推定した姿勢を返す。

        ここに渡すのは「LiDAR から見た点群」だけで、真値の姿勢は一切使わない。
        """
        scan = voxel_downsample(np.asarray(scan_sensor, dtype=np.float64), self.config.scan_voxel_size)

        if self._tree is None:
            # 最初のフレームは合わせる相手が無い。原点を地図座標系の基準にする。
            self._commit(scan, self.pose, IcpResult(self.pose.copy(), 1.0, 0.0, 0), degraded=False)
            return self.pose

        predicted = self.pose @ self._last_delta
        result = icp(
            scan,
            self._tree,
            self._tree_points,
            predicted,
            max_correspondence_dist=self.config.max_correspondence_dist,
            max_iterations=self.config.max_iterations,
        )

        degraded = result.fitness < self.config.min_fitness
        pose = predicted if degraded else result.pose
        self._commit(scan, pose, result, degraded)
        return self.pose

    def _commit(self, scan: np.ndarray, pose: np.ndarray, result: IcpResult, degraded: bool) -> None:
        self._last_delta = np.linalg.inv(self.pose) @ pose
        self.pose = pose
        self.map.add(transform_points(scan, pose))

        self._frames_since_rebuild += 1
        if self._tree is None or self._frames_since_rebuild >= self.config.kdtree_rebuild_interval:
            self._tree_points = voxel_downsample(self.map.points(), self.config.icp_voxel_size)
            self._tree = cKDTree(self._tree_points)
            self._frames_since_rebuild = 0

        self.stats.append(
            SlamStats(
                frame=self._frame,
                pose=pose.copy(),
                fitness=result.fitness,
                inlier_rmse=result.inlier_rmse,
                iterations=result.n_iterations,
                degraded=degraded,
            )
        )
        self._frame += 1

    def trajectory(self) -> np.ndarray:
        """推定した軌跡を `(n_frames, 3)` の位置列として返す。"""
        if not self.stats:
            return np.zeros((0, 3))
        return np.array([s.pose[:3, 3] for s in self.stats])
