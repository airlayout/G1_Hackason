"""Isaac Sim のロボット（G1 + エレベーター乗り場）。腰（pelvis）は固定。

Isaac Sim のアプリの起動後に作ること（contest/evaluate_isaac.py が起動する）。
試行ごとに新しいステージに世界を作り直し、close() で片付ける（アプリは起動したまま）。

上半身は、実機の arm_sdk と同じく関節の目標の角度を PD で追う（IsaacLab の ImplicitActuator = PhysX の関節のドライブ。
強さは configs/contest.yaml の gains）。下半身の速度の指令は、腰を固定しているので無視する（base_enabled = False）。
Isaac Sim の関節の並びは motor 番号の順と違うので、名前で並べ替える。
"""

from __future__ import annotations

import sys
from typing import Any

import numpy as np
import torch

from common.config import FEATURE_DIR

from ..interface import JOINT_NAMES, NUM_JOINTS, UPPER_BODY_IDX, Observation
from .base import Robot, StepInfo, joint_gains

sys.path.insert(0, str(FEATURE_DIR / "sim" / "isaac"))


def _torch(x: Any) -> torch.Tensor:
    """IsaacLab 3.0 のデータ（warp の配列 / ProxyArray）を torch のテンソルにする。"""
    if isinstance(x, torch.Tensor):
        return x
    if hasattr(x, "torch"):
        return x.torch
    import warp as wp

    return wp.to_torch(x)


class IsaacRobot(Robot):
    name = "isaac"
    base_enabled = False

    def __init__(self, scene_cfg: dict[str, Any], contest_cfg: dict[str, Any], device: str = "cuda:0"):
        from isaac_world import PHYSICS_DT, build_world

        self._physics_dt = PHYSICS_DT
        self.w = build_world(scene_cfg, device, gains_cfg=contest_cfg["gains"], cameras=True,
                             head_data_types=("rgb", "distance_to_image_plane"), overview=False, new_stage=True)
        self.scene = self.w.scene
        names = list(self.w.robot.joint_names)
        missing = [n for n in JOINT_NAMES if n not in names]
        if missing:
            raise RuntimeError(f"Isaac Sim の G1 に無い関節: {missing}")
        # motor 番号 i の関節が、Isaac Sim の何番目か
        self._isaac_idx = torch.tensor([names.index(n) for n in JOINT_NAMES], device=self.w.robot.device)
        lim = _torch(self.w.robot.data.joint_pos_limits)[0][self._isaac_idx].cpu().numpy()
        self._lo, self._hi = lim[:, 0].astype(float), lim[:, 1].astype(float)
        self._kp, self._kd = joint_gains(contest_cfg["gains"])
        # 初期の姿勢（build_world が書き込んだもの。joint_pos は次の update まで古いので default を使う）
        self.q_des = _torch(self.w.robot.data.default_joint_pos)[0][self._isaac_idx].cpu().numpy().astype(float)
        self._cam = self.w.cameras[self.w.robot_cfg["head_camera"]["name"]]

    # ---- Robot ----------------------------------------------------------------

    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self._lo.copy(), self._hi.copy()

    def upper_gains(self) -> tuple[np.ndarray, np.ndarray]:
        idx = list(UPPER_BODY_IDX)
        return self._kp[idx].copy(), self._kd[idx].copy()

    def observe(self, t: float) -> Observation:
        out = self._cam.data.output
        rgb = _torch(out["rgb"])[0][..., :3].cpu().numpy().astype(np.uint8)
        depth = _torch(out["distance_to_image_plane"])[0].squeeze(-1).cpu().numpy().astype(np.float32)
        depth[~np.isfinite(depth)] = 0.0  # 何も無い方向（無限遠）は「取れない画素」と同じ 0 にする
        K = _torch(self._cam.data.intrinsic_matrices)[0].cpu().numpy().astype(float)
        dq = _torch(self.w.robot.data.joint_vel)[0][self._isaac_idx].cpu().numpy().astype(float)
        return Observation(rgb=rgb, depth=depth, K=K, q=self._q(), dq=dq, t=float(t))

    def command(self, q_upper: np.ndarray, base_cmd: np.ndarray) -> None:
        self.q_des[list(UPPER_BODY_IDX)] = q_upper
        target = torch.zeros((1, NUM_JOINTS), dtype=torch.float32, device=self.w.robot.device)
        target[0, self._isaac_idx] = torch.as_tensor(self.q_des, dtype=torch.float32, device=self.w.robot.device)
        self.w.robot.set_joint_position_target_index(target=target)

    def advance(self, dt: float) -> StepInfo:
        n = max(1, int(round(dt / self._physics_dt)))
        self.w.step(n)
        # 壁や盤との接触力は、まだ測っていない（PhysX の接触の報告を足すまで None）
        return StepInfo(lit={b.name: self.w.hall.lit(b.name) for b in self.scene.buttons}, max_contact_force=None)

    def close(self) -> None:
        self.w.close()

    # ---- 内部 -----------------------------------------------------------------

    def _q(self) -> np.ndarray:
        return _torch(self.w.robot.data.joint_pos)[0][self._isaac_idx].cpu().numpy().astype(float)
