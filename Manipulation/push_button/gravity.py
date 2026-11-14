"""画像・IKとは独立した固定骨盤モデルで、右腕の重力補償を計算する。"""

from pathlib import Path

import numpy as np

from kinematics import ArmKinematics, load_robot
from trajectory import validate_arm_tau


class GravityCompensation:
    def __init__(self, model_path, calibration, scale, *, dynamics=False):
        if not np.isfinite(scale) or not 0 <= scale <= 1:
            raise ValueError("gravity-scaleは0〜1が必要です")
        self.scale = float(scale)
        self.dynamics = dynamics
        model, _ = load_robot(Path(model_path).expanduser().resolve(),
            tuple(calibration["tip_offset_m"]), float(calibration["pelvis_height_m"]),
            float(calibration["tip_radius_m"]))
        self.kin = ArmKinematics(model)

    def __call__(self, configuration, *, velocity=None):
        joints, imu, *_ = configuration
        self.kin.data.qvel[:] = 0.0
        self.kin.update_fixed_base(np.asarray(joints), imu)
        torque = self.kin.data.qfrc_bias[self.kin.dadr].copy()
        if self.dynamics and velocity is not None:
            velocity=np.asarray(velocity,dtype=float)
            if velocity.shape != (7,) or not np.all(np.isfinite(velocity)) or np.max(np.abs(velocity)) > .8+1e-8:
                raise ValueError('動的補償の関節速度が不正です')
            # SDKはdq=0なので、既知のkd*dq_refをtauへ加える。摩擦は公称モデルのみを使う。
            torque += 1.5*velocity+self.kin.model.dof_frictionloss[self.kin.dadr]*np.tanh(velocity/.01)
        torque *= self.scale
        return validate_arm_tau(torque.tolist())
