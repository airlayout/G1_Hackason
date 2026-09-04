"""G1のD435i相当RGBカメラ。"""

from __future__ import annotations

import torch

from .sensor_rig import TORSO_TO_D435_RPY, TORSO_TO_D435_XYZ, rpy_to_quat_xyzw

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360
UPDATE_PERIOD = 0.1

# RealSense D435i RGBの公称水平FOV 69度に合わせる。
FOCAL_LENGTH_MM = 24.0
HORIZONTAL_APERTURE_MM = 2.0 * FOCAL_LENGTH_MM * 0.686242214563  # tan(69deg / 2)


def _torch_view(value):
    if hasattr(value, "torch"):
        return value.torch
    return value


class G1Camera:
    """torso_linkへ固定したRGBカメラを、画像・内部行列・真値姿勢として公開する。"""

    def __init__(
        self,
        robot_prim_path: str = "/World/G1",
        width: int = IMAGE_WIDTH,
        height: int = IMAGE_HEIGHT,
        update_period: float = UPDATE_PERIOD,
    ) -> None:
        import isaaclab.sim as sim_utils
        from isaaclab.sensors import Camera, CameraCfg

        self.width = width
        self.height = height
        self.update_period = update_period
        cfg = CameraCfg(
            prim_path=f"{robot_prim_path}/torso_link/d435_color",
            offset=CameraCfg.OffsetCfg(
                pos=TORSO_TO_D435_XYZ,
                rot=rpy_to_quat_xyzw(*TORSO_TO_D435_RPY),
                convention="world",
            ),
            update_period=update_period,
            width=width,
            height=height,
            data_types=["rgb"],
            update_latest_camera_pose=True,
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=FOCAL_LENGTH_MM,
                horizontal_aperture=HORIZONTAL_APERTURE_MM,
                clipping_range=(0.1, 40.0),
            ),
        )
        self._sensor = Camera(cfg)
        print(
            f"[Camera] D435i profile: {width}x{height}, "
            f"{1.0 / update_period:.1f} Hz"
        )

    def update(self, dt: float) -> None:
        self._sensor.update(dt, force_recompute=True)

    def read_rgb(self) -> torch.Tensor:
        output = _torch_view(self._sensor.data.output["rgb"])
        return output[0, ..., :3]

    def intrinsic_matrix(self) -> torch.Tensor:
        matrices = _torch_view(self._sensor.data.intrinsic_matrices)
        return matrices[0]

    def pose_ros(self) -> tuple[torch.Tensor, torch.Tensor]:
        positions = _torch_view(self._sensor.data.pos_w)
        quaternions = _torch_view(self._sensor.data.quat_w_ros)
        return positions[0], quaternions[0]
