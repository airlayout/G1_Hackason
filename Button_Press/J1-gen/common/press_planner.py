"""押し込みの軌道を作る（タスク2）。

    planner = PressPlanner.from_config(robot_cfg, press_cfg, arm_cfg, side)
    plan = planner.plan(q_now, target, push_dir, q_seed=q_rec)   # 届かない・ぶつかるなら UnreachableError

動作は 2 段階（HANDOFF 6章）:
1. 対象の手前（押す方向に approach_distance_m 戻った点）の姿勢まで、関節空間で補間して移動する
   （補間そのものは ArmCommander.move_to が行う。ここでは途中の姿勢の衝突だけを調べる）
2. そこから押す方向に沿って、手先を直線で表面 + press_depth_m まで押し込み、同じ直線で戻る。
   直線上の点ごとに IK を解く（前の点の解を初期値にする）

送る前（dry-run でも）に、次をすべて確かめる。どれかに引っかかれば UnreachableError で拒否する:
- IK が収束する、関節リミット（余裕付き）に当たらない
- 手先が作業空間の箱の中にある
- 腕が体にぶつからない（MuJoCo、common/collision.py）
- 直線の区間で、1 周期あたりの関節の動きが上限（arm.yaml の max_joint_speed_rad_s / control_hz）以下
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .arm.safety import UnsafeTargetError, WorkspaceBox, interpolate_joint
from .collision import CollisionChecker
from .kinematics import ArmKinematics


class UnreachableError(UnsafeTargetError):
    """届かない、ぶつかる、などの理由で押し込みの軌道が作れない。"""


@dataclass
class PressPlan:
    side: str
    q_approach: np.ndarray  # 手前の姿勢（片腕 7 関節）
    press_in: list[np.ndarray]  # 手前 → 押し込み終わり（片腕 7 関節の点列、1 周期に 1 点）
    press_out: list[np.ndarray]  # 押し込み終わり → 手前
    approach_point: np.ndarray  # 手前の点（pelvis 座標）
    target_point: np.ndarray  # 対象の表面の点
    end_point: np.ndarray  # 押し込み終わりの点
    push_dir: np.ndarray  # 押す方向（単位ベクトル）
    max_ik_err_m: float
    depth: float  # 押し込みの深さ [m]（上限で頭打ちにした後の値）
    # 計算に使った腰の角度（yaw, roll, pitch）[rad]
    waist_q: np.ndarray

    def summary(self) -> str:
        f = lambda v: np.array2string(np.asarray(v), precision=3, suppress_small=True)  # noqa: E731
        return (
            f"手前 {f(self.approach_point)} → 表面 {f(self.target_point)} → 押し込み終わり {f(self.end_point)}"
            f"（押す方向 {f(self.push_dir)}、直線 {len(self.press_in)} 点、IK の最大誤差 "
            f"{self.max_ik_err_m * 1000:.1f} mm）"
        )


class PressPlanner:
    def __init__(
        self,
        kin: ArmKinematics,
        collision: CollisionChecker,
        workspace: WorkspaceBox,
        press_cfg: dict[str, Any],
        control_dt: float,
        max_joint_step: float,
        joint_path_samples: int,
    ) -> None:
        self.kin = kin
        self.collision = collision
        self.workspace = workspace
        self.cfg = press_cfg
        self.dt = control_dt
        self.max_joint_step = max_joint_step
        self.joint_path_samples = joint_path_samples

    @classmethod
    def from_config(
        cls, robot_cfg: dict[str, Any], press_cfg: dict[str, Any], arm_cfg: dict[str, Any], side: str
    ) -> "PressPlanner":
        dt = 1.0 / float(arm_cfg["control_hz"])
        return cls(
            kin=ArmKinematics(robot_cfg, side, press_cfg["ik"]),
            collision=CollisionChecker(robot_cfg, side, press_cfg["collision"]["clearance_m"]),
            workspace=WorkspaceBox.from_config(press_cfg["workspace"][side]),
            press_cfg=press_cfg["press"],
            control_dt=dt,
            max_joint_step=float(arm_cfg["safety"]["max_joint_speed_rad_s"]) * dt,
            joint_path_samples=int(press_cfg["collision"]["joint_path_samples"]),
        )

    @property
    def side(self) -> str:
        return self.kin.side

    # ---- 個別の確認 -------------------------------------------------------------

    def check_pose(self, q: np.ndarray, what: str) -> None:
        """1 つの姿勢（29 関節）について、作業空間と衝突を確かめる。"""
        try:
            self.workspace.check(self.kin.fk_pos(q), f"{what}の手先")
        except UnsafeTargetError as e:
            raise UnreachableError(str(e)) from e
        hits = self.collision.contacts(q)
        if hits:
            raise UnreachableError(f"{what}で腕が体にぶつかる: " + "; ".join(str(h) for h in hits[:3]))

    def solve(self, point: np.ndarray, q_seed: np.ndarray, axis: np.ndarray | None, what: str) -> np.ndarray:
        """IK を解き、姿勢を確かめて 29 関節の角度を返す。"""
        r = self.kin.ik(point, q_seed, axis)
        if not r.success:
            raise UnreachableError(f"{what}に届かない: {r.reason}")
        self.check_pose(r.q, what)
        return r.q

    # ---- 軌道 -------------------------------------------------------------------

    def plan(
        self,
        q_now: np.ndarray,
        target: np.ndarray,
        push_dir: np.ndarray,
        q_seed: np.ndarray | None = None,
        depth: float | None = None,
    ) -> PressPlan:
        """
        q_now: 今の関節角（29、実測）。腰と反対の腕はこの値で固定して計算する
        target: 対象の表面の点（pelvis 座標）
        push_dir: 押す方向（pelvis 座標。長さは問わない）
        q_seed: IK の初期値（29）。ティーチングで記録した押し込み姿勢を渡す。無ければ q_now
        depth: 押し込みの深さ [m]（無ければ設定値）。max_press_depth_m で頭打ちにする
        """
        c = self.cfg
        q_now = np.asarray(q_now, dtype=float)
        target = np.asarray(target, dtype=float)
        n = np.asarray(push_dir, dtype=float)
        if not np.all(np.isfinite(target)) or not np.all(np.isfinite(n)) or np.linalg.norm(n) < 1e-9:
            raise UnreachableError(f"目標または押す方向が不正: target={target}, push_dir={push_dir}")
        n = n / np.linalg.norm(n)
        d = float(c["press_depth_m"] if depth is None else depth)
        d = min(max(d, 0.0), float(c["max_press_depth_m"]))
        approach = target - float(c["approach_distance_m"]) * n
        end = target + d * n
        axis = n if c["align_finger"] else None

        seed = q_now.copy()
        if q_seed is not None:
            seed[self.kin.arm_idx] = np.asarray(q_seed, dtype=float)[self.kin.arm_idx]

        for p, what in ((approach, "手前の点"), (end, "押し込み終わりの点")):
            try:
                self.workspace.check(p, what)
            except UnsafeTargetError as e:
                raise UnreachableError(str(e)) from e

        q_app = self.solve(approach, seed, axis, "手前の姿勢")

        # 今の姿勢 → 手前の姿勢（関節空間の補間）の途中の姿勢もぶつからないか
        arm = self.kin.arm_idx
        path = interpolate_joint(q_now[arm], q_app[arm], 1.0, 1.0 / self.joint_path_samples)
        for k, qa in enumerate(path):
            q = q_app.copy()
            q[arm] = qa
            hits = self.collision.contacts(q)
            if hits:
                raise UnreachableError(
                    f"手前の姿勢へ移動する途中（{k + 1}/{len(path)}）で腕が体にぶつかる: "
                    + "; ".join(str(h) for h in hits[:3])
                )

        # 直線の押し込み。1 周期に進む距離 = 速さ × dt
        length = float(np.linalg.norm(end - approach))
        steps = max(1, int(np.ceil(length / (float(c["press_speed_m_s"]) * self.dt))))
        press_in = []
        q = q_app.copy()
        max_err = 0.0
        for k in range(1, steps + 1):
            p = approach + (end - approach) * (k / steps)
            r = self.kin.ik(p, q, axis, max_iter=50)
            if not r.success:
                raise UnreachableError(f"押し込みの直線の {k}/{steps} 点目に届かない: {r.reason}")
            self.check_pose(r.q, f"押し込みの直線の {k}/{steps} 点目")
            step = float(np.max(np.abs(r.q[arm] - q[arm])))
            if step > self.max_joint_step + 1e-9:
                raise UnreachableError(
                    f"押し込みの直線の {k}/{steps} 点目で関節が 1 周期に {step:.4f} rad 動く"
                    f"（上限 {self.max_joint_step:.4f}）。特異姿勢に近い可能性がある"
                )
            max_err = max(max_err, r.pos_err)
            q = r.q
            press_in.append(q[arm].copy())
        # 戻りは同じ点列を逆にたどり、最後に手前の姿勢へ
        press_out = [w.copy() for w in reversed(press_in[:-1])] + [q_app[arm].copy()]
        return PressPlan(
            side=self.side,
            q_approach=q_app[arm].copy(),
            press_in=press_in,
            press_out=press_out,
            approach_point=approach,
            target_point=target,
            end_point=end,
            push_dir=n,
            max_ik_err_m=max_err,
            depth=d,
            waist_q=q_now[[12, 13, 14]].copy(),
        )

    def replan(self, plan: PressPlan, q_measured: np.ndarray, q_arm_now: np.ndarray) -> PressPlan:
        """手前の姿勢に着いたあと、実測の腰の角度で同じ押し込みを計算し直す。

        腰が倒れると肩の位置が動くので、最初の計算（開始時の腰の角度）のままだと手先がずれる。
        q_measured: lowstate の関節角（29）。腰と反対の腕はこの値を使う
        q_arm_now: 今の指令の片腕 7 関節（手前の姿勢）。IK の初期値にする
        """
        q = np.asarray(q_measured, dtype=float).copy()
        q[self.kin.arm_idx] = q_arm_now
        return self.plan(q, plan.target_point, plan.push_dir, q_seed=q, depth=plan.depth)
