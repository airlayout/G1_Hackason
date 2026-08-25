"""MuJoCo のレイキャストで 3D LiDAR を模擬する。

MuJoCo には LiDAR センサーが無く、`lerobot/unitree-g1-mujoco` の G1 モデルにも
`rangefinder` の類は 1 つも定義されていない（あるのは関節・IMU・足裏の力センサーと
`head_camera` だけ）。そこで `mj_multiRay`（1 点から多数のレイを飛ばして最初に当たった
geom までの距離を返す C 実装）を使って、LiDAR 相当の点群を毎フレーム自前で生成する。
"""
from __future__ import annotations

import mujoco
import numpy as np

from Mapping.common.lidar_spec import LidarSpec


def _rotation_y(angle_rad: float) -> np.ndarray:
    """y 軸まわりの回転行列（正の角度で機首下げ = ピッチダウン）。"""
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class MujocoLidar:
    """MuJoCo モデル上の指定ボディに取り付けた 3D LiDAR。"""

    def __init__(
        self,
        mj_model: mujoco.MjModel,
        spec: LidarSpec | None = None,
        mount_body: str = "torso_link",
        mount_pos: tuple[float, float, float] = (0.0, 0.0, 0.55),
        mount_pitch_deg: float = 0.0,
        self_body_root: str = "pelvis",
    ) -> None:
        """
        `mount_pos` はボディ座標系での取り付け位置。既定は頭部の少し上
        （立位でワールド座標 z≈1.40m）で、実機で言えば短いマストの上に
        測量用 LiDAR を載せた格好にあたる。

        高さを詰めすぎてはいけない。`torso_link` の頭部 geom はワールド z≈1.12〜1.33m
        を占めており、`head_camera` と同じ高さ（z≈1.27m）に置くと LiDAR が頭の
        内側に入ってしまい、全レイが自分の頭に当たって点が 1 つも返らない
        （実際にこれで最初にハマった）。

        `mount_pitch_deg` は下向きの取り付け角で、既定は 0（水平）。
        360 度スキャンする LiDAR を傾けると、前を向く側が下がるのと引き換えに
        後ろを向く側が同じだけ上を向き、後方が空振りする。既定の LidarSpec は
        上下対称寄りの FOV なので傾ける必要が無い。
        """
        self.spec = spec or LidarSpec()
        self.mount_pos = np.asarray(mount_pos, dtype=np.float64)
        self.mount_rot = _rotation_y(np.deg2rad(mount_pitch_deg))

        self.body_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, mount_body)
        if self.body_id < 0:
            raise ValueError(f"取り付け先のボディが見つからない: {mount_body}")

        # センサー座標系での方向ベクトル。返す点群もこの座標系で表す。
        self.dirs_sensor = self.spec.ray_directions()
        # ボディ座標系での方向ベクトル（レイキャストに渡すのはこちら）。
        self.dirs_body = self.dirs_sensor @ self.mount_rot.T

        # 自己反射（ロボット自身の腕や脚に当たったレイ）を落とすためのマスク。
        # 浮遊ベースのロボットは全ボディが pelvis を根とする 1 本の運動学ツリーに属し、
        # 壁・床のような静的 geom は world(=0) を根とするので、根で見分けられる。
        root_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, self_body_root)
        geom_root = mj_model.body_rootid[mj_model.geom_bodyid]
        self._is_self_geom = geom_root == root_id

        n = self.spec.n_rays
        self._geomid = np.empty(n, dtype=np.int32)
        self._dist = np.empty(n, dtype=np.float64)

        # レイキャスト専用の MjData。シミュレーション本体の mj_data を直接使ってはいけない。
        # `mj_multiRay` は mjData のスタックを一時領域として確保するが、その mjData は
        # 物理演算スレッドが回し、さらにビューアスレッドが `mj_copyDataVisual` で
        # 毎フレーム複製している。別スレッドからレイキャストすると
        # 「attempting to copy mjData while stack is in use」でプロセスごと segfault する
        # （実際にこれで落ちた）。自前の MjData なら誰とも競合しない。
        self._scratch = mujoco.MjData(mj_model)

    def scan(self, mj_model: mujoco.MjModel, mj_data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
        """1 フレーム分のスキャンを撮る。

        戻り値は `(センサー座標系の点群 (N,3), LiDAR のワールド姿勢 (4,4))`。
        点群のほうが SLAM への入力で、ワールド姿勢は真値なので**評価にしか使わない**。
        """
        # レイキャストが参照するのは geom のワールド姿勢だけなので、そこだけ写して使う。
        # コピー中に物理演算が 1〜2 ステップ進む可能性はあるが、250Hz・0.4m/s では
        # 1 ステップ 1.6mm で、地図の解像度(5cm)から見て無視できる。
        self._scratch.geom_xpos[:] = mj_data.geom_xpos
        self._scratch.geom_xmat[:] = mj_data.geom_xmat

        body_rot = mj_data.xmat[self.body_id].reshape(3, 3)
        body_pos = mj_data.xpos[self.body_id]
        origin = np.ascontiguousarray(body_pos + body_rot @ self.mount_pos, dtype=np.float64)
        dirs_world = np.ascontiguousarray(self.dirs_body @ body_rot.T, dtype=np.float64)

        mujoco.mj_multiRay(
            mj_model,
            self._scratch,
            origin,
            dirs_world.reshape(-1),
            None,  # geomgroup=None: 全 geom グループを対象にする
            1,  # flg_static: 壁・床は静的 geom なので必ず含める
            -1,  # bodyexclude: 除外は下の self geom マスクで行う
            self._geomid,
            self._dist,
            None,  # 法線は使わない
            self.spec.n_rays,
            self.spec.max_range_m,
        )

        hit = (self._geomid >= 0) & (self._dist > 0.0) & (self._dist <= self.spec.max_range_m)
        hit &= ~self._is_self_geom[np.where(self._geomid >= 0, self._geomid, 0)]

        points_sensor = self.dirs_sensor[hit] * self._dist[hit, None]

        pose = np.eye(4)
        pose[:3, :3] = body_rot @ self.mount_rot
        pose[:3, 3] = origin
        return points_sensor, pose
