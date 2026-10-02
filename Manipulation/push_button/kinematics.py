"""MuJoCo を運動学モデルとして使う右腕の FK / IK と経路衝突検査。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

from trajectory import ARM_JOINTS, validate_arm_q

IK_POSITION_TOLERANCE_M = 0.0015
IK_AXIS_TOLERANCE_DEG = 10.0


@dataclass(frozen=True)
class IkSolution:
    joint_angles: np.ndarray
    position_error_m: float
    axis_error_deg: float
    min_joint_margin_rad: float
    iterations: int


def arm_addresses(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    joints = [model.joint(name) for name in ARM_JOINTS]
    qadr = np.array([joint.qposadr[0] for joint in joints], dtype=int)
    dadr = np.array([joint.dofadr[0] for joint in joints], dtype=int)
    actuator = np.array([model.actuator(name).id for name in ARM_JOINTS], dtype=int)
    return qadr, dadr, actuator


def solve_ik(model: mujoco.MjModel, data: mujoco.MjData, target: np.ndarray,
             seed: np.ndarray, rest: np.ndarray, qadr: np.ndarray,
             dadr: np.ndarray, desired_axis: np.ndarray | None = None,
             position_tolerance_m: float = IK_POSITION_TOLERANCE_M) -> IkSolution:
    """手先の3D位置と押下方向を、右腕7軸のヤコビアンIKで関節角へ変換する。

    姿勢誤差と位置誤差を重み付きで扱い、関節限界を守りつつ現在の解に近い姿勢を選ぶ。
    各目標で収束しない場合は、押下経路を作らずエラーにする。
    """
    site_id = model.site("button_tcp").id
    q = seed.copy()
    joint_limits = model.jnt_range[[model.joint(name).id for name in ARM_JOINTS]]
    lower = joint_limits[:, 0] + 0.025
    upper = joint_limits[:, 1] - 0.025
    if desired_axis is None:
        desired_axis = np.array([1.0, 0.0, 0.0])
    for iteration in range(1, 351):
        data.qpos[qadr] = q
        mujoco.mj_forward(model, data)
        position_error = target - data.site_xpos[site_id]
        axis = data.site_xmat[site_id].reshape(3, 3)[:, 0]
        axis_error_deg = math.degrees(math.acos(float(np.clip(
            np.dot(axis, desired_axis), -1.0, 1.0))))
        position_error_m = float(np.linalg.norm(position_error))
        if (position_error_m < position_tolerance_m
                and axis_error_deg < IK_AXIS_TOLERANCE_DEG):
            margin = float(min(np.min(q - joint_limits[:, 0]),
                               np.min(joint_limits[:, 1] - q)))
            return IkSolution(q.copy(), position_error_m, axis_error_deg,
                              margin, iteration)
        jac_pos = np.zeros((3, model.nv))
        jac_rot = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jac_pos, jac_rot, site_id)
        # d(axis)/dq = angular_velocity × axis
        axis_jac = np.cross(jac_rot[:, dadr].T, axis).T
        regularization = min(0.003, 2 * position_tolerance_m)
        matrix = np.vstack((jac_pos[:, dadr], 0.10 * axis_jac, regularization * np.eye(7)))
        residual = np.concatenate((position_error, 0.10 * (desired_axis - axis),
                                   regularization * (rest - q)))
        step = np.linalg.lstsq(matrix, residual, rcond=None)[0]
        step = np.clip(step, -0.10, 0.10)
        q = np.clip(q + step, lower, upper)
    data.qpos[qadr] = q
    mujoco.mj_forward(model, data)
    error = np.linalg.norm(target - data.site_xpos[site_id])
    axis = data.site_xmat[site_id].reshape(3, 3)[:, 0]
    axis_error = math.degrees(math.acos(float(np.clip(
        np.dot(axis, desired_axis), -1.0, 1.0))))
    raise ValueError(f"手先のIKが収束しません (位置誤差 {error:.3f} m、"
                     f"方向誤差 {axis_error:.1f} 度): {target}")


def load_robot(model_path: Path, tip_offset: tuple[float, float, float],
               pelvis_height: float, tip_radius: float = 0.015) -> tuple[mujoco.MjModel, np.ndarray]:
    """固定骨盤のモデルを読み込む。床基準の高さは校正値を要求する。"""
    if not (np.isfinite(pelvis_height) and 0.65 < pelvis_height < 1.0):
        raise ValueError("実測した骨盤高 (0.65～1.0 m) が必要です")
    if (len(tip_offset) != 3 or not np.all(np.isfinite(tip_offset))
            or not 0.04 <= tip_offset[0] <= 0.16
            or max(abs(tip_offset[1]), abs(tip_offset[2])) > 0.05
            or not 0 < tip_radius < 0.04):
        raise ValueError("手先の校正値が不正です")
    root = ET.parse(model_path).getroot()
    if "29dof" not in root.get("model", ""):
        raise ValueError("29DoF の Menagerie モデルが必要です")
    root.find("compiler").set("meshdir", str((model_path.parent / "assets").resolve()))
    stand = root.find("./keyframe/key[@name='stand']")
    rest = np.fromstring(stand.get("ctrl"), sep=" ")
    if len(rest) != 29:
        raise ValueError("29軸の stand 姿勢が必要です")
    root.remove(root.find("keyframe"))
    pelvis = root.find(".//body[@name='pelvis']")
    pelvis.remove(pelvis.find("freejoint"))
    pelvis.set("pos", f"0 0 {pelvis_height}")
    wrist = root.find(".//body[@name='right_wrist_yaw_link']")
    position = " ".join(map(str, tip_offset))
    ET.SubElement(wrist, "site", name="button_tcp", pos=position, size="0.006")
    ET.SubElement(wrist, "geom", name="button_tip", type="sphere", pos=position,
                  size=str(tip_radius), mass="0.01")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    if model.nq != 29 or model.nu != 29:
        raise ValueError("想定外の関節構成です")
    return model, rest


class ArmKinematics:
    """実際の全関節・骨盤姿勢をコピーし、物理状態を変更せずに計画する。"""

    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.data = mujoco.MjData(model)
        self.qadr, self.dadr, self.actuator = arm_addresses(model)
        self.site_id = model.site("button_tcp").id
        self.tip_id = model.geom("button_tip").id
        shoulder = model.body("right_shoulder_pitch_link").id
        bodies = {shoulder}
        for body in range(shoulder + 1, model.nbody):
            if int(model.body_parentid[body]) in bodies:
                bodies.add(body)
        self.arm_geoms = {g for g in range(model.ngeom)
                          if int(model.geom_bodyid[g]) in bodies
                          and (model.geom_contype[g] or model.geom_conaffinity[g])}

    def update(self, qpos: np.ndarray) -> None:
        qpos = np.asarray(qpos, dtype=float)
        if qpos.shape != (self.model.nq,) or not np.all(np.isfinite(qpos)):
            raise ValueError("運動学に渡す実機状態が不正です")
        self.data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.data)

    def tip_position(self) -> np.ndarray:
        return self.data.site_xpos[self.site_id].copy()

    def update_fixed_base(self, qpos: np.ndarray, imu_rpy: tuple[float, float, float]):
        """停止時の床座標で骨盤のroll/pitchを反映する。yawは基準の前方を0とする。"""
        if self.model.nq != 29:
            raise ValueError("固定骨盤の29軸モデルだけが対象です")
        rpy = np.asarray(imu_rpy, dtype=float)
        if rpy.shape != (3,) or not np.all(np.isfinite(rpy)):
            raise ValueError("骨盤IMU姿勢が不正です")
        roll, pitch = rpy[:2] / 2
        cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
        self.model.body_quat[self.model.body("pelvis").id] = (cr*cp, sr*cp, cr*sp, -sr*sp)
        self.update(qpos)

    def body_transform(self, name: str) -> np.ndarray:
        body = self.model.body(name).id
        transform = np.eye(4)
        transform[:3, :3] = self.data.xmat[body].reshape(3, 3)
        transform[:3, 3] = self.data.xpos[body]
        return transform

    def solve(self, target: np.ndarray, normal: np.ndarray,
              position_tolerance_m: float = 0.0001) -> IkSolution:
        if not np.isfinite(position_tolerance_m) or not 0 < position_tolerance_m <= IK_POSITION_TOLERANCE_M:
            raise ValueError("IKの位置許容誤差が不正です")
        original = self.data.qpos.copy()
        seed = original[self.qadr].copy()
        try:
            solution = solve_ik(self.model, self.data, np.asarray(target), seed,
                                seed, self.qadr, self.dadr, np.asarray(normal),
                                position_tolerance_m=position_tolerance_m)
            validate_arm_q(solution.joint_angles.tolist())
            return solution
        finally:
            self.update(original)

    def validate_path(self, goal: np.ndarray, face: np.ndarray | None = None,
                      normal: np.ndarray | None = None, allow_tip_contact: bool = False,
                      allowed_tip_geoms: set[int] | None = None) -> None:
        """関節補間中の自己衝突とパネル面への侵入を検査する。

        壁は計測面を無限平面として保守的に扱う。実環境の障害物全体は表現しない。
        """
        goal = np.asarray(validate_arm_q(np.asarray(goal).tolist()))
        original = self.data.qpos.copy()
        start = original[self.qadr]
        samples = max(12, int(np.max(np.abs(goal - start)) / 0.015) + 1)
        try:
            for ratio in np.linspace(0, 1, samples):
                self.data.qpos[self.qadr] = start + ratio * (goal - start)
                mujoco.mj_forward(self.model, self.data)
                for contact in self.data.contact:
                    if contact.dist >= -0.0005:
                        continue
                    pair = {int(contact.geom1), int(contact.geom2)}
                    if not pair.intersection(self.arm_geoms):
                        continue
                    other = pair - {self.tip_id}
                    if (allow_tip_contact and self.tip_id in pair
                            and allowed_tip_geoms is not None and other <= allowed_tip_geoms):
                        continue
                    raise ValueError(f"腕の経路に衝突があります (geoms={sorted(pair)})")
                if face is not None:
                    for geom in self.arm_geoms:
                        if allow_tip_contact and geom == self.tip_id:
                            continue
                        # 球とメッシュの頂点で、計測面に対する最前面を求める。
                        center = self.data.geom_xpos[geom]
                        rotation = self.data.geom_xmat[geom].reshape(3, 3)
                        local_normal = rotation.T @ normal
                        kind = self.model.geom_type[geom]
                        if kind == mujoco.mjtGeom.mjGEOM_MESH:
                            mesh = int(self.model.geom_dataid[geom])
                            adr = self.model.mesh_vertadr[mesh]
                            count = self.model.mesh_vertnum[mesh]
                            extent = np.max(self.model.mesh_vert[adr:adr+count] @ local_normal)
                        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
                            extent = self.model.geom_size[geom, 0]
                        else:
                            extent = self.model.geom_rbound[geom]
                        if (center - face) @ normal + extent > 0.0005:
                            raise ValueError("腕の経路が計測したパネル面へ侵入します")
        finally:
            self.update(original)
