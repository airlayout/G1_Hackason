"""Navigationの12軸歩行方策を、腕が独立に動く29軸MuJoCoモデルへ接続する。

観測47次元、PD、位相とスケールはunitree_rl_gymの固定版を使用。
上半身17軸を観測へ混ぜず、脚12軸だけを名前で対応付ける。
DDSや実機SDKには接続しない。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np

LEG_JOINTS = tuple(f"{side}_{joint}_joint" for side in ("left", "right")
                   for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"))
DEFAULT_ASSETS = Path(__file__).resolve().parents[3] / "Navigation/sim/assets"


def yaw_of(quaternion):
    w, x, y, z = quaternion
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def support_margin(points, com_xy):
    """接地点の凸包から重心投影までの最小距離。外側は負、退化形状はNone。"""
    points = sorted(set(map(tuple, points)))

    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])

    halves = []
    for sequence in (points, reversed(points)):
        hull = []
        for point in sequence:
            while len(hull) >= 2 and cross(hull[-2], hull[-1], point) <= 0:
                hull.pop()
            hull.append(point)
        halves.append(hull[:-1])
    hull = halves[0]+halves[1]
    if len(hull) < 3:
        return None
    return min(cross(a, b, com_xy)/math.hypot(b[0]-a[0], b[1]-a[1])
               for a, b in zip(hull, hull[1:]+hull[:1]))


@dataclass(frozen=True)
class Waypoint:
    x_m: float
    y_m: float
    yaw_deg: float = 0.0

    def __post_init__(self):
        if not np.all(np.isfinite([self.x_m, self.y_m, self.yaw_deg])):
            raise ValueError("経由点の座標・向きは有限値が必要です")


def velocity_to_waypoint(xy, yaw, goal, max_forward=0.3):
    """世界座標の経由点に向けた、胴体座標の速度指令。位置は模擬オドメトリ。"""
    error = np.array([goal.x_m, goal.y_m]) - np.asarray(xy)
    c, s = math.cos(yaw), math.sin(yaw)
    body = np.array([[c, s], [-s, c]]) @ error
    command = np.array([np.clip(1.5*body[0], -0.15, max_forward),
                        np.clip(1.5*body[1], -0.15, 0.15),
                        np.clip(1.5*wrap_angle(math.radians(goal.yaw_deg)-yaw), -0.4, 0.4)])
    return command


class WalkingPolicy:
    """脚トルク制御。初期姿勢以外でfreejointのqpos/qvelを書き換えない。"""

    def __init__(self, model, data, assets=DEFAULT_ASSETS):
        import torch
        import yaml

        assets = Path(assets)
        for name in ("g1.yaml", "motion.pt"):
            if not (assets/name).is_file():
                raise ValueError(f"歩行資産がありません: {assets/name}。Navigation/sim/fetch_assets.shで取得してください")
        self.config = yaml.safe_load((assets/"g1.yaml").read_text())
        if self.config["num_actions"] != 12 or self.config["num_obs"] != 47:
            raise ValueError("12軸・47次元観測の歩行方策が必要です")
        if (not np.isclose(model.opt.timestep, self.config["simulation_dt"])
                or self.config["control_decimation"] != 10):
            raise ValueError("歩行方策には物理刻み2ms・制御刻み20msが必要です")
        self.model, self.data, self.torch = model, data, torch
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(assets/"motion.pt"), map_location="cpu").eval()
        joints = [model.joint(name) for name in LEG_JOINTS]
        self.qadr = np.array([j.qposadr[0] for j in joints], dtype=int)
        self.dadr = np.array([j.dofadr[0] for j in joints], dtype=int)
        self.actuator = np.array([model.actuator(name).id for name in LEG_JOINTS], dtype=int)
        self.defaults = np.array(self.config["default_angles"], dtype=float)
        self.kp, self.kd = np.array(self.config["kps"]), np.array(self.config["kds"])
        self.command, self.action = np.zeros(3), np.zeros(12, dtype=np.float32)
        self.target = self.defaults.copy()
        self.steps, self.peak_action = 0, 0.0
        self.saved = {name: getattr(model, name)[self.actuator].copy() for name in
                      ("actuator_gainprm", "actuator_biasprm", "actuator_biastype", "actuator_ctrllimited")}
        self.saved_friction = model.dof_frictionloss[self.dadr].copy()
        self.saved_damping = model.dof_damping[self.dadr].copy()
        model.actuator_gainprm[self.actuator] = 0
        model.actuator_gainprm[self.actuator, 0] = 1
        model.actuator_biasprm[self.actuator] = 0
        model.actuator_biastype[self.actuator] = mujoco.mjtBias.mjBIAS_NONE
        model.actuator_ctrllimited[self.actuator] = False
        # 方策の学習元と同じ脚の受動抵抗。元の値は静止制御への引き継ぎ時に戻す。
        model.dof_frictionloss[self.dadr] = 0.1
        model.dof_damping[self.dadr] = 0.001

    def set_command(self, command):
        command = np.asarray(command, dtype=float)
        if command.shape != (3,) or not np.all(np.isfinite(command)):
            raise ValueError("歩行の速度指令が不正です")
        self.command = np.clip(command, [-0.15, -0.15, -0.4], [0.3, 0.15, 0.4])

    def observation(self):
        c, d = self.config, self.data
        w, x, y, z = d.qpos[3:7]
        obs = np.zeros(47, np.float32)
        obs[:3] = d.qvel[3:6]*c["ang_vel_scale"]
        obs[3:6] = [2*(w*y-z*x), -2*(z*y+w*x), 1-2*(w*w+z*z)]
        obs[6:9] = self.command*np.array(c["cmd_scale"])
        obs[9:21] = (d.qpos[self.qadr]-self.defaults)*c["dof_pos_scale"]
        obs[21:33] = d.qvel[self.dadr]*c["dof_vel_scale"]
        obs[33:45] = self.action
        phase = (self.steps*self.model.opt.timestep) % 0.8 / 0.8
        obs[45:] = [math.sin(2*math.pi*phase), math.cos(2*math.pi*phase)]
        return obs

    def step(self):
        d, m = self.data, self.model
        d.ctrl[self.actuator] = (self.target-d.qpos[self.qadr])*self.kp-d.qvel[self.dadr]*self.kd
        mujoco.mj_step(m, d)
        self.steps += 1
        if self.steps % self.config["control_decimation"]:
            return
        with self.torch.inference_mode():
            action = self.policy(self.torch.from_numpy(self.observation()).unsqueeze(0)).numpy().squeeze()
        if action.shape != (12,) or not np.all(np.isfinite(action)) or np.max(np.abs(action)) > 10:
            raise ValueError("歩行方策が発散しました")
        self.action = action
        self.peak_action = max(self.peak_action, float(np.max(np.abs(action))))
        self.target = action*self.config["action_scale"]+self.defaults

    def restore_position_control(self):
        """両足接地時の脚角度を保持し、位置サーボの減衰で足踏みを止める。

        速度に比例する項を目標角へ加えると、静止後にもその角度のずれが
        残って姿勢を崩す。脚の実測角度だけを保持し、胴体状態は変更しない。
        """
        for name, value in self.saved.items():
            getattr(self.model, name)[self.actuator] = value
        self.model.dof_frictionloss[self.dadr] = self.saved_friction
        self.model.dof_damping[self.dadr] = self.saved_damping
        self.data.ctrl[self.actuator] = self.data.qpos[self.qadr]
