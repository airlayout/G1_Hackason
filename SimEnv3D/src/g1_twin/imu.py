"""Mid-360内蔵IMU相当のIsaacLabセンサ。"""

from __future__ import annotations

import warp as wp

from .sensor_rig import TORSO_TO_MID360_RPY, TORSO_TO_MID360_XYZ, rpy_to_quat_xyzw


def _as_torch(value):
    if hasattr(value, "torch"):
        return value.torch
    return wp.to_torch(value)


class G1LidarImu:
    """LiDARと同じ取付姿勢で角速度・固有加速度を取得する。"""

    def __init__(self, robot_prim_path: str = "/World/G1", update_period: float = 0.005) -> None:
        from isaaclab.sensors import Imu, ImuCfg

        cfg = ImuCfg(
            prim_path=f"{robot_prim_path}/torso_link",
            update_period=update_period,
            offset=ImuCfg.OffsetCfg(
                pos=TORSO_TO_MID360_XYZ,
                rot=rpy_to_quat_xyzw(*TORSO_TO_MID360_RPY),
            ),
        )
        self._sensor = Imu(cfg)
        print(f"[IMU] Mid-360位置に構築しました: {1.0 / update_period:.0f} Hz")

    def reset(self) -> None:
        self._sensor.reset()

    def update(self, dt: float) -> None:
        self._sensor.update(dt, force_recompute=True)

    def read(self):
        angular_velocity = _as_torch(self._sensor.data.ang_vel_b)[0]
        linear_acceleration = _as_torch(self._sensor.data.lin_acc_b)[0]
        return angular_velocity, linear_acceleration
