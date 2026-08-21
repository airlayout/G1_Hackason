"""G1 の頭部 (head_link) に搭載する RGB カメラ。

現時点では搭載のみ（ROS への配信や自律走行への利用はしていない）。
将来 ROS の /camera/image_raw に配信したくなったら、read_rgb() の戻り値を
sensor_msgs/Image にエンコードして ros_bridge.py から publish すればよい。
"""

from __future__ import annotations

import torch

# 解像度。高くするほどレンダリング負荷が増え実時間比が落ちるため、
# LiDAR と同様まずは軽量な値にしてある。
IMAGE_WIDTH: int = 640
IMAGE_HEIGHT: int = 480
# 更新周期 [s]。カメラのレンダリングは重いため、物理ステップ毎(dt=0.005)
# ではなく LiDAR と同様に間引く（10Hz）。
UPDATE_PERIOD: float = 0.1


class G1Camera:
    """G1 の頭部に取り付ける RGB カメラ。

    Isaac Sim のアプリ起動後に生成すること（isaaclab.sensors の import が必要）。
    起動時に `--enable_cameras` が必要（run.sh は設定済み）。
    """

    def __init__(self, robot_prim_path: str = "/World/G1") -> None:
        """カメラを構築する。

        Args:
            robot_prim_path: G1 の prim パス
        """
        import isaaclab.sim as sim_utils
        from isaaclab.sensors import Camera, CameraCfg

        cfg = CameraCfg(
            prim_path=f"{robot_prim_path}/head_link/front_cam",
            # head_link の原点から前方 (+X) にわずかにオフセットし、
            # 自分の頭部メッシュが映り込まないようにする。
            # convention="world" は 前方 +X・上 +Z（G1 の body 座標系と同じ）
            # なので、回転を考える必要が無く rot は無回転(単位クォータニオン)でよい。
            offset=CameraCfg.OffsetCfg(pos=(0.1, 0.0, 0.0), rot=(0.0, 0.0, 0.0, 1.0), convention="world"),
            update_period=UPDATE_PERIOD,
            width=IMAGE_WIDTH,
            height=IMAGE_HEIGHT,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0,
                clipping_range=(0.05, 30.0),
            ),
        )
        self._sensor = Camera(cfg)

        print(f"[Camera] 頭部カメラを構築しました: {IMAGE_WIDTH}x{IMAGE_HEIGHT}, {1.0 / UPDATE_PERIOD:.0f} Hz")

    def update(self, dt: float) -> None:
        """センサを更新する。シミュレーションループから毎制御周期呼ぶ。"""
        self._sensor.update(dt, force_recompute=True)

    def read_rgb(self) -> torch.Tensor:
        """直近の RGB フレームを返す。

        Returns:
            形状 (H, W, 3) の uint8 テンソル。
        """
        return self._sensor.data.output["rgb"][0, ..., :3]
