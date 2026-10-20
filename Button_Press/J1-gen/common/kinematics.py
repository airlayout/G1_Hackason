"""片腕の FK / IK（URDF + Pinocchio）。タスク2。

用語:
- FK（順運動学）: 関節角 → 手先の位置と向き
- IK（逆運動学）: 手先の目標 → 関節角。ここでは「微分 IK」（少しずつ関節角を直して近づける）を使う
- ヤコビアン J: 関節角を少し変えたときに手先がどれだけ動くかを並べた行列（Δx ≈ J Δq）
- 減衰付き擬似逆行列（DLS）: Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ Δx。腕が伸び切った姿勢などで Δq が暴れないよう、
  λ（減衰）を足したもの

座標は pelvis 基準（公式 URDF の根元リンクが pelvis）。関節角は 29 関節・motor 番号順で受け渡す
（Pinocchio のモデルの関節の並びも同じであることを確認済み。tests/test_kinematics.py）。
IK で動かすのは片腕 7 関節だけ。腰と反対の腕は、渡された角度のまま固定する。

向きの扱い: 押すときは「指の向き（手先リンクの x 軸）を押す方向にそろえる」ことだけを求める
（指の軸まわりの回転は自由）。位置だけ合わせると、肩だけで補正して指の向きがずれるため。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .config import resolve_repo_path
from .robot_model import JOINT_NAMES, LEFT_ARM_IDX, NUM_MOTORS, RIGHT_ARM_IDX


@dataclass
class IKResult:
    q: np.ndarray  # 29 関節（motor 番号順）
    success: bool
    pos_err: float  # [m]
    axis_err: float  # 指の向きと目標の向きの角度 [rad]（向きを指定しないときは 0）
    iterations: int
    reason: str = ""
    at_limit: list[str] = field(default_factory=list)


class ArmKinematics:
    def __init__(self, robot_cfg: dict[str, Any], side: str, ik_cfg: dict[str, Any]) -> None:
        import pinocchio as pin

        self._pin = pin
        if side not in ("left", "right"):
            raise ValueError(f"side は left か right: {side}")
        self.side = side
        urdf = resolve_repo_path(robot_cfg["model"]["urdf_path"])
        self.model = pin.buildModelFromUrdf(str(urdf))
        names = [self.model.names[i] for i in range(1, self.model.njoints)]
        if names != list(JOINT_NAMES) or self.model.nq != NUM_MOTORS:
            raise ValueError(f"URDF の関節の並びが motor 番号順と違う: {names}")

        ee = robot_cfg["end_effector"][side]
        link_id = self.model.getFrameId(ee["link"])
        link = self.model.frames[link_id]
        placement = link.placement * pin.SE3(np.eye(3), np.asarray(ee["offset"], dtype=float))
        self.ee_frame = self.model.addFrame(
            pin.Frame(f"{side}_fingertip", link.parentJoint, link_id, placement, pin.FrameType.OP_FRAME)
        )
        self.data = self.model.createData()
        self.arm_idx = np.array(RIGHT_ARM_IDX if side == "right" else LEFT_ARM_IDX)
        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()
        self.cfg = ik_cfg

    # ---- FK -------------------------------------------------------------------

    def fk(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """手先の位置 (3,) と向き (3, 3)（pelvis 座標）。"""
        pin = self._pin
        pin.framesForwardKinematics(self.model, self.data, np.asarray(q, dtype=float))
        t = self.data.oMf[self.ee_frame]
        return t.translation.copy(), t.rotation.copy()

    def fk_pos(self, q: np.ndarray) -> np.ndarray:
        return self.fk(q)[0]

    def finger_axis(self, q: np.ndarray) -> np.ndarray:
        """指の向き（手先リンクの x 軸、pelvis 座標）。"""
        return self.fk(q)[1][:, 0]

    # ---- IK -------------------------------------------------------------------

    def ik(
        self,
        target_pos: np.ndarray,
        q_seed: np.ndarray,
        target_axis: np.ndarray | None = None,
        max_iter: int | None = None,
    ) -> IKResult:
        """手先を target_pos へ（与えれば指の向きを target_axis へ）合わせる片腕 7 関節の角度を求める。

        q_seed: 初期値（29 関節）。腰と反対の腕はこの値のまま固定する。記録した押し込み姿勢を渡すと、
                それに近い解が得られる（余った自由度で q_seed に近づける項を入れている）。
        収束しない、または関節リミット（余裕 limit_margin_rad）に当たる場合は success=False。
        """
        pin = self._pin
        c = self.cfg
        tol_pos = float(c["tol_pos_m"])
        tol_axis = np.radians(float(c["tol_axis_deg"]))
        w_axis = float(c["axis_weight"])
        lam = float(c["damping"])
        max_step = float(c["max_step_rad"])
        k_null = float(c["null_gain"])
        margin = float(c["limit_margin_rad"])
        n_iter = int(max_iter if max_iter is not None else c["max_iter"])

        target_pos = np.asarray(target_pos, dtype=float)
        axis_t = None
        if target_axis is not None:
            axis_t = np.asarray(target_axis, dtype=float)
            axis_t = axis_t / np.linalg.norm(axis_t)
        q = np.asarray(q_seed, dtype=float).copy()
        q_ref = q[self.arm_idx].copy()
        lo = self.lower[self.arm_idx] + margin
        hi = self.upper[self.arm_idx] - margin
        q[self.arm_idx] = np.clip(q[self.arm_idx], lo, hi)

        e_pos = np.inf
        e_axis = 0.0
        it = 0
        for it in range(1, n_iter + 1):
            pin.computeJointJacobians(self.model, self.data, q)
            pin.framesForwardKinematics(self.model, self.data, q)
            t = self.data.oMf[self.ee_frame]
            J = pin.getFrameJacobian(self.model, self.data, self.ee_frame, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J = J[:, self.arm_idx]
            err_p = target_pos - t.translation
            e_pos = float(np.linalg.norm(err_p))
            rows_e = [err_p]
            rows_j = [J[:3]]
            if axis_t is not None:
                a = t.rotation[:, 0]
                # 指の軸の向きのずれ。回転ベクトル a × a_t の方向に回すと a が a_t に近づく
                cross = np.cross(a, axis_t)
                e_axis = float(np.arctan2(np.linalg.norm(cross), np.dot(a, axis_t)))
                # 指の軸まわりの回転（ロール）は自由にするため、角速度から a 方向の成分を除く
                P = np.eye(3) - np.outer(a, a)
                rows_e.append(w_axis * cross)
                rows_j.append(w_axis * (P @ J[3:]))
            if e_pos < tol_pos and (axis_t is None or e_axis < tol_axis):
                break
            e = np.concatenate(rows_e)
            Jt = np.vstack(rows_j)
            JJt = Jt @ Jt.T + (lam**2) * np.eye(Jt.shape[0])
            Jpinv = Jt.T @ np.linalg.inv(JJt)
            dq = Jpinv @ e
            # 余った自由度（腕は 7 関節、条件は 5 つ）で、初期値の姿勢に近づける。
            # 手先を動かさない方向だけに入れるため、射影には減衰なしの擬似逆行列を使う
            # （減衰付きで射影すると、本来の目標とわずかに打ち消し合って収束しなくなる）
            N = np.eye(len(self.arm_idx)) - np.linalg.pinv(Jt, rcond=1e-4) @ Jt
            dq += N @ (k_null * (q_ref - q[self.arm_idx]))
            n = float(np.max(np.abs(dq)))
            if n > max_step:
                dq *= max_step / n
            q[self.arm_idx] = np.clip(q[self.arm_idx] + dq, lo, hi)

        ok = e_pos < tol_pos and (axis_t is None or e_axis < tol_axis)
        at_limit = [
            JOINT_NAMES[i]
            for k, i in enumerate(self.arm_idx)
            if q[i] <= lo[k] + 1e-6 or q[i] >= hi[k] - 1e-6
        ]
        reason = ""
        if at_limit:
            ok = False
            reason = "関節リミットに当たる: " + ", ".join(at_limit)
        elif not ok:
            reason = (
                f"収束しない（位置の誤差 {e_pos * 1000:.1f} mm、"
                f"指の向きの誤差 {np.degrees(e_axis):.1f}°、{it} 回）"
            )
        return IKResult(q=q, success=ok, pos_err=e_pos, axis_err=e_axis, iterations=it,
                        reason=reason, at_limit=at_limit)
