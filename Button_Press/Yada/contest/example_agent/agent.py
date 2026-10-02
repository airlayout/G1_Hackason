"""見本のエージェント（従来型）: 深度でボタンを見つける → IK → 右手の中指で押す。

各チームは、これをコピーして作り始めてよい。使っているのは観測（深度・内部パラメータ・関節の角度）と、
機体の公式モデル（J1-gen の IK とカメラの取り付け位置）だけ。シミュレーターの正解の値は使っていない。

流れ:
1. look   : 両腕を下ろしたまま、ボタンを探す（find_buttons）
            - カラー画像で、まわりより明るい灰色の小さな丸を探し、深度で 3 次元の位置にする（本番に似た乗り場。
              ボタンはほとんど出っ張っていないので、深度だけでは見つからない）。上から 2 つが一般用の ▲▼
            - 見つからなければ、深度で「壁より 1.5〜2.6 cm 手前に出ている塊」を探す（前の、壁に付いた盤）
            - 押す向きは、ボタンのまわりの柱（壁）の面の向きを深度から推定して決める（正面と決めつけない）
2. reach  : 右手をボタンの手前 6 cm へ（関節空間で滑らかに補間）
3. settle : 手前で止まり、腕の揺れが収まるのを待つ（kp 60 / kd 1.5 は減衰が小さく、しばらく揺れる）
4. press  : 面にまっすぐ押し込む（中指の先の球がボタンの面を越える深さまで）。指は斜め（左・上に傾ける）
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
# ⚠️ 弱点（2026-10-02）: 本番に似た乗り場の ▲ で、手前の位置へ関節の角度で動かす途中に、指先が柱の横の壁を
#    こする（接触力 30〜47 N。点灯はする）。手前の手前（15 cm）を経由させると、柱の正面 0.3 m に立っているので
#    手が胸の前に来て体に当たり、かえって失敗した（成功率 100% → 65%）。通り道を横に回すなどの工夫が要る
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
# 指の向き: 押す向き（面の向き）から、左に 35°・上に 20° 傾ける（右下から斜めに指を伸ばす。人が押すときと同じ）。
# 本番に似た乗り場では、柱の正面に立つのでボタンが体の真ん中（胸の高さ）にある。指を正面に向けると、
# 右腕を体の前に寄せることになり、肘・手首・肩が胴体に当たった（端の 8 通りのうち 0 通りしか当たらずに届かなかった）。
# この傾きなら 8 通りすべてで当たらずに届いた（2026-10-02、MuJoCo の衝突判定で確認）。
# 指先の動きは、面にまっすぐ（押す向き）にする（指の向きのまま押し込むと、指先がボタンの面の上を滑って外れるため）
FINGER_TILT_LEFT_DEG = 35.0
FINGER_TILT_UP_DEG = 20.0


def finger_axis_for(normal: np.ndarray) -> np.ndarray:
    """押す向き normal（面に向かう向き）から、指の向きを決める（左・上に傾ける）。"""
    up = np.array([0.0, 0.0, 1.0])
    left = np.cross(up, normal)
    left /= np.linalg.norm(left)
    up = np.cross(normal, left)
    a, b = np.radians(FINGER_TILT_LEFT_DEG), np.radians(FINGER_TILT_UP_DEG)
    f = np.cos(b) * (np.cos(a) * normal + np.sin(a) * left) + np.sin(b) * up
    return f / np.linalg.norm(f)


@dataclass
class Button:
    name: str
    center: np.ndarray  # 面の中心（pelvis 座標）
    n_pixels: int


def _pelvis_points(obs: Observation, cam_tf) -> np.ndarray:
    """深度の各画素の 3 次元の位置 (H, W, 3)（pelvis 座標）。測れない画素は z が 0 の点になる。"""
    z = obs.depth.astype(float)
    h, w = z.shape
    v, u = np.mgrid[0:h, 0:w]
    K = obs.K
    p_opt = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], axis=-1)
    r, p = cam_tf.pelvis_from_optical(obs.q[list(WAIST_IDX)])
    return p_opt @ r.T + p


def _plane_axis(P: np.ndarray, mask: np.ndarray) -> np.ndarray | None:
    """mask の画素の点に平面 x = a + b·y + c·z を当てはめ、その面に向かう向き（押す向き）を返す。"""
    pts = P[mask]
    if len(pts) < 200:
        return None
    A = np.c_[np.ones(len(pts)), pts[:, 1], pts[:, 2]]
    coef, *_ = np.linalg.lstsq(A, pts[:, 0], rcond=None)
    n = np.array([1.0, -coef[1], -coef[2]])
    n /= np.linalg.norm(n)
    # 正面から大きく外れた推定は使わない（何か別の物に当てはめた）
    return n if n[0] > np.cos(np.radians(20)) else None


def find_buttons_by_color(obs: Observation, cam_tf) -> tuple[list[Button], np.ndarray | None]:
    """カラー画像で、まわりより明るい灰色の小さな丸を探す。(上から順のボタン, 柱の面に向かう向き)。

    まわり（約 120 画素四方の平均）より明るい所を探す。柱は、頭カメラに近い上のほうが照明で明るく写るので、
    画像全体の暗さを基準にすると、上のボタンが明るい柱とつながって見つからなかった（2026-10-02）。
    """
    from scipy import ndimage

    rgb = obs.rgb.astype(float)
    gray = rgb.mean(axis=2)
    chroma = rgb.max(axis=2) - rgb.min(axis=2)
    background = ndimage.uniform_filter(gray, size=121)
    spots = (gray - background > 18) & (chroma < 40)  # 明るい灰色（青い車いすの印や、色のある物は除く）
    spots = ndimage.binary_fill_holes(spots)  # ボタンの中の暗い記号（▲▼）の穴を埋める
    valid = (obs.depth > 0.05) & (obs.depth < 3.0)
    P = _pelvis_points(obs, cam_tf)
    labels, _ = ndimage.label(spots)
    found: list[Button] = []
    for k, sl in enumerate(ndimage.find_objects(labels), start=1):
        if sl is None:
            continue
        comp = labels[sl] == k
        area = int(comp.sum())
        if area < 40 or area > 20000 or area < 0.5 * comp.size:  # 小さすぎる・大きすぎる・丸くない（細長い線など）
            continue
        m = np.zeros_like(spots)
        m[sl] = comp
        m &= valid
        if m.sum() < 20:
            continue
        pts = P[m]
        size = pts[:, 1:].max(axis=0) - pts[:, 1:].min(axis=0)
        if not (np.all(size > 0.015) and np.all(size < 0.07)):  # ボタンの大きさ（直径 3〜4.5 cm）
            continue
        c = np.array([np.median(pts[:, 0]), pts[:, 1].mean(), pts[:, 2].mean()])
        found.append(Button(name="", center=c, n_pixels=int(m.sum())))
    found.sort(key=lambda b: -b.center[2])
    axis = None
    if found:
        # 押す向き: いちばん上の 2 つのボタンのまわり（左右 6 cm、上下 10 cm）の、ボタン以外の点（柱の面）
        top = np.array([b.center for b in found[:2]])
        near = (np.abs(P[..., 1] - top[:, 1].mean()) < 0.06) & (P[..., 2] > top[:, 2].min() - 0.1) & \
               (P[..., 2] < top[:, 2].max() + 0.1) & valid & ~spots
        axis = _plane_axis(P, near)
    return found, axis


def find_buttons(obs: Observation, cam_tf) -> list[Button]:
    """ボタンを探し、上から順に up、down と名前を付ける（一般用の 2 つ）。見つからなければ空。

    押す向き（柱の面に向かう向き）は find_buttons_with_axis で得られる。
    """
    return find_buttons_with_axis(obs, cam_tf)[0]


def find_buttons_with_axis(obs: Observation, cam_tf) -> tuple[list[Button], np.ndarray]:
    found, axis = find_buttons_by_color(obs, cam_tf)
    if len(found) < 2:
        found = find_buttons_by_depth(obs, cam_tf)
        axis = None
    for b, name in zip(found, ("up", "down")):
        b.name = name
    return found[:2], (axis if axis is not None else PRESS_AXIS.copy())


def find_buttons_by_depth(obs: Observation, cam_tf) -> list[Button]:
    """深度からボタンを探す（壁に付いた盤のシーン。ボタンが壁から 2 cm 出ている）。上から順。"""
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
    return found


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
        self.axis = PRESS_AXIS.copy()  # 押す向き（柱の面の向きから推定する）。指先はこの向きに動く
        self.finger = finger_axis_for(self.axis)  # 指の向き（押す向きから傾ける）

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
        buttons, self.axis = find_buttons_with_axis(obs, self.cam_tf)
        print(f"[example] 見つけたボタン: {[(b.name, b.center.round(3).tolist()) for b in buttons]}、"
              f"押す向き {self.axis.round(3).tolist()}")
        target = next((b for b in buttons if b.name == self.task.target), None)
        if target is None or len(buttons) < 2:
            print("[example] ボタンを 2 つ見つけられなかったので、やめる")
            return False
        # 中指の先の球の中心の目標: 球の表面がボタンの面に触れる位置から、PRESS_BEYOND_M だけ奥
        ax = self.axis
        self.finger = finger_axis_for(ax)
        touch = target.center - ax * TIP_RADIUS
        self.p_touch = touch
        self.p_approach = touch - ax * APPROACH_M
        self.p_pressed = touch + ax * PRESS_BEYOND_M
        seed = self.q_hold.copy()
        seed[list(RIGHT_ARM_IDX)] = IK_SEED_RIGHT_ARM
        r = self.kin.ik(self.p_approach, self.kin.ik(self.p_approach, seed).q, self.finger)
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
        if float((self.p_touch - tip) @ self.axis) > SERVO_FREEZE_M:
            e = p_goal - tip
            if not integrate_x:  # 押す向きの成分は足し込まない（押し込みの力を出すため）
                e = e - (e @ self.axis) * self.axis
            self.corr = np.clip(self.corr + SERVO_GAIN * self.dt * e, -SERVO_MAX_M, SERVO_MAX_M)
        r = self.kin.ik(p_goal + self.corr, self.q_ik, self.finger, max_iter=50)
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
