"""MuJoCo のロボット（G1 + エレベーター乗り場）。腰（pelvis）は固定。

上半身は、実機の arm_sdk と同じく関節の目標の角度を PD で追う（強さは configs/contest.yaml の gains）。
PD の D の項は MuJoCo の関節の減衰（dof_damping）に入れ、積分器を implicitfast にする。
陽的に計算すると、軽い手首のリンクで発散するため（J1-gen の common/arm/backend_sim.py と同じ対策）。
下半身の速度の指令は、腰を固定しているので無視する（base_enabled = False）。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from common.config import FEATURE_DIR
from common.j1gen_bridge import j1gen_config
from common.scene_spec import build_hall_scene

from ..interface import JOINT_NAMES, NUM_JOINTS, UPPER_BODY_IDX, Observation
from .base import Robot, StepInfo

sys.path.insert(0, str(FEATURE_DIR / "sim" / "mujoco"))

from mujoco_hall import HALL_BODY, HallMujoco, build_hall_model  # noqa: E402

WRIST_IDX = (19, 20, 21, 26, 27, 28)


def joint_gains(gains_cfg: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """configs/contest.yaml の gains を、29 関節の (kp, kd) にする。"""
    kp, kd = np.zeros(NUM_JOINTS), np.zeros(NUM_JOINTS)
    for i in range(NUM_JOINTS):
        if i < 12:
            g = gains_cfg["legs"]
        elif i < 15:
            g = gains_cfg["waist"]
        elif i in WRIST_IDX:
            g = gains_cfg["wrist"]
        else:
            g = gains_cfg["arm"]
        kp[i], kd[i] = float(g["kp"]), float(g["kd"])
    return kp, kd


class MujocoRobot(Robot):
    name = "mujoco"
    base_enabled = False

    def __init__(self, scene_cfg: dict[str, Any], contest_cfg: dict[str, Any], viewer: bool = False):
        import mujoco

        self._mj = mujoco
        self.scene = build_hall_scene(scene_cfg)
        self.robot_cfg = j1gen_config("robot.yaml")
        m = build_hall_model(self.scene, self.robot_cfg, scene_cfg.get("fingertip"))
        m.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.model, self.data = m, mujoco.MjData(m)

        self._qadr = np.zeros(NUM_JOINTS, dtype=int)
        self._vadr = np.zeros(NUM_JOINTS, dtype=int)
        self._act = np.zeros(NUM_JOINTS, dtype=int)
        self._lo, self._hi = np.zeros(NUM_JOINTS), np.zeros(NUM_JOINTS)
        for i, n in enumerate(JOINT_NAMES):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            self._qadr[i], self._vadr[i] = m.jnt_qposadr[jid], m.jnt_dofadr[jid]
            self._act[i] = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            self._lo[i], self._hi[i] = m.jnt_range[jid]
        self._kp, self._kd = joint_gains(contest_cfg["gains"])
        m.dof_damping[self._vadr] = self._kd

        q0 = np.zeros(NUM_JOINTS)
        for idx, deg in scene_cfg.get("initial_pose_deg", {}).items():
            q0[int(idx)] = np.radians(float(deg))
        self.data.qpos[self._qadr] = q0
        mujoco.mj_forward(m, self.data)
        self.q_des = q0.copy()
        self.hall = HallMujoco(m, self.data, self.scene)

        cam = self.robot_cfg["head_camera"]
        self._cam = cam["name"]
        w, h = int(cam["width"]), int(cam["height"])
        fy = (h / 2.0) / np.tan(np.radians(float(cam["fovy_deg"])) / 2.0)
        self.K = np.array([[fy, 0.0, w / 2.0], [0.0, fy, h / 2.0], [0.0, 0.0, 1.0]])
        self._rgb = mujoco.Renderer(m, height=h, width=w)
        self._depth = mujoco.Renderer(m, height=h, width=w)
        self._depth.enable_depth_rendering()

        # 乗り場の固定の箱（壁・扉・盤）の geom と、ロボットの geom（衝突の力を測るため）
        hall_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, HALL_BODY)
        self._hall_geoms = set(np.flatnonzero(m.geom_bodyid == hall_id).tolist())
        # ワールド（床）、乗り場、ボタン以外の body の geom がロボットのもの
        not_robot = {0, hall_id} | {self.hall.body_id(b.name) for b in self.scene.buttons}
        self._robot_geoms = set(np.flatnonzero(~np.isin(m.geom_bodyid, list(not_robot))).tolist())

        self._viewer = None
        if viewer:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(m, self.data)

    # ---- Robot ----------------------------------------------------------------

    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._lo.copy(), self._hi.copy()

    def upper_gains(self) -> tuple[np.ndarray, np.ndarray]:
        idx = list(UPPER_BODY_IDX)
        return self._kp[idx].copy(), self._kd[idx].copy()

    def observe(self, t: float) -> Observation:
        d = self.data
        self._rgb.update_scene(d, camera=self._cam)
        self._depth.update_scene(d, camera=self._cam)
        return Observation(
            rgb=self._rgb.render().copy(),
            depth=self._depth.render().astype(np.float32),
            K=self.K.copy(),
            q=d.qpos[self._qadr].copy(),
            dq=d.qvel[self._vadr].copy(),
            t=float(t),
        )

    def command(self, q_upper: np.ndarray, base_cmd: np.ndarray) -> None:
        self.q_des[list(UPPER_BODY_IDX)] = q_upper

    def advance(self, dt: float) -> StepInfo:
        m, d = self.model, self.data
        n = max(1, int(round(dt / m.opt.timestep)))
        max_f = 0.0
        t0 = time.time()
        for _ in range(n):
            d.ctrl[self._act] = self._kp * (self.q_des - d.qpos[self._qadr])
            self._mj.mj_step(m, d)
            self.hall.update()
            max_f = max(max_f, self._max_hall_contact())
        if self._viewer is not None:
            if not self._viewer.is_running():
                raise KeyboardInterrupt("画面が閉じられた")
            self._viewer.sync()
            time.sleep(max(0.0, dt - (time.time() - t0)))
        return StepInfo(lit={b.name: self.hall.lit(b.name) for b in self.scene.buttons}, max_contact_force=max_f)

    def close(self) -> None:
        self._rgb.close()
        self._depth.close()
        if self._viewer is not None:
            self._viewer.close()

    # ---- 内部 -----------------------------------------------------------------

    def _max_hall_contact(self) -> float:
        """ロボットと乗り場の固定の箱（壁・扉・盤）の接触の、法線方向の力の最大 [N]。"""
        d = self.data
        f6 = np.zeros(6)
        best = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if (g1 in self._hall_geoms and g2 in self._robot_geoms) or (g2 in self._hall_geoms and g1 in self._robot_geoms):
                self._mj.mj_contactForce(self.model, d, i, f6)
                best = max(best, abs(float(f6[0])))
        return best
