"""見本のエージェント（従来型）: 深度でボタンを見つける → IK → 右手の中指で押す。

各チームは、これをコピーして作り始めてよい。使っているのは観測（深度・内部パラメータ・関節の角度）と、
機体の公式モデル（J1-gen の IK とカメラの取り付け位置）だけ。シミュレーターの正解の値は使っていない。

流れ:
1. look   : 両腕を下ろしたまま、深度からボタンを探す
            （壁より 1.5〜2.6 cm 手前に出ている、大きさ 2〜6 cm の塊。上にある方が ▲）
2. reach  : 右手をボタンの手前 6 cm へ（関節空間で滑らかに補間）
3. settle : 手前で止まり、腕の揺れが収まるのを待つ（kp 60 / kd 1.5 は減衰が小さく、しばらく揺れる）
4. press  : 押す向き（+x）にまっすぐ押し込む（中指の先の球がボタンの面を越える深さまで）
5. hold   : 押し込んだまま少し待つ
6. retract: 手前の位置に戻して終了（done）

腕を弱い PD で動かすときの工夫（実機でも同じ）:
- 重力: 垂れる量（重力のトルク ÷ kp）だけ目標をずらす
- 残りのずれ: settle 以降は、実際の関節の角度から指先の位置を計算（FK）し、目標とのずれを少しずつ足し込む
  （積分の補正）。押す向き（x）は押し込みの力を出すために、押している間は足し込まない。
  ボタンに触れる直前で補正を止め、そのまま押し込む（触れたあとは摩擦で指先が横に動けないため）
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from common.j1gen_bridge import j1gen, j1gen_config
from contest.interface import RIGHT_ARM_IDX, UPPER_BODY_IDX, WAIST_IDX, Action, Agent, Observation, TaskInfo

PRESS_AXIS = np.array([1.0, 0.0, 0.0])
# 中指の先の衝突判定の球の半径（configs/elevator_hall.yaml の fingertip.radius。機体の情報として使う）
TIP_RADIUS = 0.008
APPROACH_M = 0.06
# ボタンの面からさらに押し込む量。腕が弱いので、ボタンの沈む量（4 mm）より深く目標を置いて力を出す
PRESS_BEYOND_M = 0.015
LOOK_AT_S = 0.2
MOVE_SPEED_RAD_S = 0.8  # 関節空間の補間の速さの目安（ランナーの上限 1.0 rad/s より遅く）
SETTLE_S, PRESS_S, HOLD_S, RETRACT_S = 1.0, 2.0, 0.8, 1.0
# 積分の補正の強さ [1/s] と上限 [m]
SERVO_GAIN = 3.0
SERVO_MAX_M = 0.05
# 指先がボタンの面のこの距離 [m] より手前にあるときだけ補正を足し込む。触れたあとは摩擦で指先が横に動けないので、
# 足し込み続けると補正だけが膨らみ、腕の力が押す向きではなく横と上に逃げる（実測: 補正が 4〜5 cm まで膨らんだ）
SERVO_FREEZE_M = 0.01
# 手前の位置の IK を始める右腕の姿勢（motor 22〜28）。肘を外に張って腕を上げた姿勢。
# 腕を下ろした姿勢から始めると、肘を内側にたたむ解になり、肩が胴に当たった（練習用の 20 試行のほぼすべて）。
# この姿勢から始めると、20 試行すべてで当たらない解になった（MuJoCo の衝突判定で確認）
IK_SEED_RIGHT_ARM = np.array([-1.0, -0.5, 0.3, 0.3, 0.0, 0.0, 0.0])


@dataclass
class Button:
    name: str
    center: np.ndarray  # 面の中心（pelvis 座標）
    n_pixels: int


def find_buttons(obs: Observation, cam_tf) -> list[Button]:
    """深度からボタンを探す。上から順に up、down と名前を付ける。見つからなければ空。"""
    from scipy import ndimage

    z = obs.depth.astype(float)
    h, w = z.shape
    v, u = np.mgrid[0:h, 0:w]
    K = obs.K
    valid = (z > 0.05) & (z < 3.0)
    p_opt = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], axis=-1)
    r, p = cam_tf.pelvis_from_optical(obs.q[list(WAIST_IDX)])
    P = p_opt @ r.T + p  # (h, w, 3) pelvis 座標

    # 壁の位置: 前方の面のうち、いちばん奥の大きな面（扉は壁より少し手前にある）
    x = P[..., 0][valid & (P[..., 2] > -0.3)]
    if x.size == 0:
        return []
    hist, edges = np.histogram(x, bins=np.arange(x.min(), x.max() + 0.002, 0.002))
    peaks = np.flatnonzero(hist > 0.1 * hist.max())
    wall_x = float(edges[peaks.max()] + 0.001)

    mask = valid & (P[..., 0] < wall_x - 0.015) & (P[..., 0] > wall_x - 0.026)
    labels, n = ndimage.label(mask)
    found: list[Button] = []
    for k in range(1, n + 1):
        pts = P[labels == k]
        if len(pts) < 30:
            continue
        size = pts[:, 1:].max(axis=0) - pts[:, 1:].min(axis=0)
        if np.all(size > 0.02) and np.all(size < 0.06):  # 細長い扉の枠などは除く
            c = np.array([np.median(pts[:, 0]), pts[:, 1].mean(), pts[:, 2].mean()])
            found.append(Button(name="", center=c, n_pixels=len(pts)))
    found.sort(key=lambda b: -b.center[2])
    for b, name in zip(found, ("up", "down")):
        b.name = name
    return found[:2]


def smoothstep(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


class ExampleAgent(Agent):
    def __init__(self) -> None:
        import pinocchio as pin

        robot_cfg = j1gen_config("robot.yaml")
        self.kin = j1gen("kinematics").ArmKinematics(robot_cfg, "right", j1gen_config("press.yaml")["ik"])
        self.cam_tf = j1gen("camera_geometry").HeadCameraTransform(robot_cfg)
        self._pin = pin

    # ---- Agent ----------------------------------------------------------------

    def reset(self, task: TaskInfo) -> None:
        self.task = task
        self.dt = float(task.control_dt)
        self.kp = np.asarray(task.upper_kp, dtype=float)
        self.phase = "look"
        self.t_phase = 0.0
        self.q_hold: np.ndarray | None = None  # 29 関節。右腕以外はこの角度のまま
        self.q_ik: np.ndarray | None = None  # 直前の IK の解（29 関節。次の IK の初期値）
        self.corr = np.zeros(3)  # 積分の補正 [m]
        self.p_approach = self.p_pressed = self.p_touch = np.zeros(3)

    def act(self, obs: Observation) -> Action:
        if self.q_hold is None:
            self.q_hold = obs.q.copy()
        arm = list(RIGHT_ARM_IDX)
        q_des = self.q_hold.copy()
        done = False
        el = obs.t - self.t_phase

        if self.phase == "look":
            if obs.t >= LOOK_AT_S:
                if self._plan(obs):
                    self._enter("reach", obs.t)
                else:
                    done = True
        elif self.phase == "reach":
            s = smoothstep(el / self.reach_T)
            q_des[arm] = (1 - s) * self.q_hold[arm] + s * self.q_ik[arm]
            if s >= 1.0:
                self._enter("settle", obs.t)
        elif self.phase == "settle":
            q_des[arm] = self._servo(obs, self.p_approach, integrate_x=True)
            if el >= SETTLE_S:
                self._enter("press", obs.t)
        elif self.phase == "press":
            s = smoothstep(el / PRESS_S)
            q_des[arm] = self._servo(obs, (1 - s) * self.p_approach + s * self.p_pressed, integrate_x=False)
            if s >= 1.0:
                self._enter("hold", obs.t)
        elif self.phase == "hold":
            q_des[arm] = self._servo(obs, self.p_pressed, integrate_x=False)
            if el >= HOLD_S:
                self._enter("retract", obs.t)
        elif self.phase == "retract":
            s = smoothstep(el / RETRACT_S)
            q_des[arm] = self._servo(obs, (1 - s) * self.p_pressed + s * self.p_approach, integrate_x=False)
            done = s >= 1.0

        q_cmd = self._gravity_offset(q_des)
        return Action(q_target=q_cmd[list(UPPER_BODY_IDX)], done=done)

    # ---- 内部 -----------------------------------------------------------------

    def _enter(self, phase: str, t: float) -> None:
        self.phase, self.t_phase = phase, t

    def _plan(self, obs: Observation) -> bool:
        buttons = find_buttons(obs, self.cam_tf)
        print(f"[example] 見つけたボタン: {[(b.name, b.center.round(3).tolist()) for b in buttons]}")
        target = next((b for b in buttons if b.name == self.task.target), None)
        if target is None or len(buttons) < 2:
            print("[example] ボタンを 2 つ見つけられなかったので、やめる")
            return False
        # 中指の先の球の中心の目標: 球の表面がボタンの面に触れる位置から、PRESS_BEYOND_M だけ奥
        touch = target.center - PRESS_AXIS * TIP_RADIUS
        self.p_touch = touch
        self.p_approach = touch - PRESS_AXIS * APPROACH_M
        self.p_pressed = touch + PRESS_AXIS * PRESS_BEYOND_M
        seed = self.q_hold.copy()
        seed[list(RIGHT_ARM_IDX)] = IK_SEED_RIGHT_ARM
        r = self.kin.ik(self.p_approach, self.kin.ik(self.p_approach, seed).q, PRESS_AXIS)
        if not r.success:
            print(f"[example] 手前の位置の IK が解けない: {r.reason}")
            return False
        self.q_ik = r.q
        arm = list(RIGHT_ARM_IDX)
        self.reach_T = max(1.5, float(np.abs(r.q[arm] - self.q_hold[arm]).max()) / MOVE_SPEED_RAD_S)
        return True

    def _servo(self, obs: Observation, p_goal: np.ndarray, integrate_x: bool) -> np.ndarray:
        """指先を p_goal に合わせる右腕 7 関節の角度。実際の指先とのずれを積分して目標に足す。"""
        tip = self.kin.fk_pos(obs.q)
        if float((self.p_touch - tip) @ PRESS_AXIS) > SERVO_FREEZE_M:
            mask = np.array([1.0 if integrate_x else 0.0, 1.0, 1.0])
            self.corr = np.clip(self.corr + SERVO_GAIN * self.dt * (p_goal - tip) * mask, -SERVO_MAX_M, SERVO_MAX_M)
        r = self.kin.ik(p_goal + self.corr, self.q_ik, PRESS_AXIS, max_iter=50)
        if np.isfinite(r.pos_err) and r.pos_err < 0.01:  # 収束しきらなくても近ければ使う（毎周期解き直すため）
            self.q_ik = r.q
        return self.q_ik[list(RIGHT_ARM_IDX)]

    def _gravity_offset(self, q_des: np.ndarray) -> np.ndarray:
        """PD が重力で垂れる量（重力のトルク ÷ kp）だけ、上半身の目標をずらす。"""
        tau_g = self._pin.computeGeneralizedGravity(self.kin.model, self.kin.data, q_des)
        out = q_des.copy()
        idx = list(UPPER_BODY_IDX)
        kp = np.where(self.kp > 0, self.kp, np.inf)
        out[idx] = q_des[idx] + tau_g[idx] / kp
        return out


def make_agent() -> Agent:
    return ExampleAgent()
