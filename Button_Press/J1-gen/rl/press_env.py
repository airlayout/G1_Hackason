"""強化学習でボトルを押す環境（Gymnasium）。範囲 A: 右腕 7 関節だけを動かし、ボトルの位置は与える（状態を入力）。

設定は configs/rl_press.yaml。物理は common/arm/backend_sim.py（arm_sdk の近似、weight 1）と同じ作り:
胴体固定、Kd 項は関節減衰に入れて implicitfast で解く、重力は qfrc_bias で補償する、腕以外は開始の姿勢で保持する。

1 周期（50 Hz）の流れ:
  行動 a（7、−1〜1）→ 目標角 += a × max_step_rad（関節リミットの内側に収める）→ PD で 0.02 秒進める → 観測と報酬

観測（すべて実機でも得られる値。pelvis 座標）:
  右腕の関節角 7、角速度 7（× 0.1）、目標角 − 関節角 7、指先の位置 3、押す点 − 指先 3、指の向き 3、
  机の縁の x と天板の z 2、前の行動 7（合わせて 39）

終わり方（info["outcome"]）:
  success   指先が押す点の近く（押す向きに直交するずれが lateral_tol_m 以内）で面まで届き（reach_x_tol_m）、
            ボトルとの接触点が指先の近く（contact_tip_tol_m）で、押す力が範囲内のまま hold_s 続いた
  collision 腕・ハンドが机や体に当たった
  overforce ボトルを押す力が force_abort_n を超えた（接触の瞬間の強打。設定が null なら無効）
  timeout   episode_s を過ぎた（truncated）
"""

from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np

from common.collision import add_hand_boxes, hand_boxes
from common.config import deep_merge, load_config
from common.robot_model import JOINT_NAMES, LEFT_ARM_IDX, NUM_MOTORS, RIGHT_ARM_IDX, WAIST_IDX, build_spec
from common.sim_scene import BOTTLE, TABLE, add_scene

# 押す向き（pelvis の前方）
PUSH_DIR = np.array([1.0, 0.0, 0.0])
OBS_DIM = 39
TIP_SITE = "rl_fingertip"
WRIST_IDX = (4, 5, 6)  # 腕 7 関節のうちの手首 3 つ


class PressEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, cfg: dict[str, Any] | None = None, robot_cfg: dict[str, Any] | None = None,
                 overrides: dict[str, Any] | None = None) -> None:
        import mujoco

        self._mj = mujoco
        self.cfg = deep_merge(cfg if cfg is not None else load_config("rl_press.yaml"), overrides or {})
        robot_cfg = robot_cfg if robot_cfg is not None else load_config("robot.yaml")
        if self.cfg["arm"] != "right":
            raise ValueError(f"今は右腕だけ: arm = {self.cfg['arm']}")
        self.arm = np.array(RIGHT_ARM_IDX)
        sc = self.cfg["scene"]

        # 机とボトルは、置き場所だけをエピソードごとに変える（model.body_pos を書き換える）。ここでは仮の位置で作る
        hx, hy = sc["table_half_xy"]
        hz = (2 * sc["table_half_z"] + sc["table_below_m"]) / 2
        scene = load_config("sim_scene.yaml")
        scene["table"].update(top_center=[0.5, sc["table_y"], 0.0], half_size=[hx, hy, hz])
        spec = build_spec(robot_cfg, fixed_base=True)
        add_scene(spec, scene)
        add_hand_boxes(spec, hand_boxes(robot_cfg))
        ee = robot_cfg["end_effector"]["right"]
        spec.body(ee["link"]).add_site(name=TIP_SITE, pos=list(ee["offset"]))
        self.model = m = spec.compile()
        m.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.data = mujoco.MjData(m)
        self._table_hz = hz
        self._bottle_cfg = scene["bottle"]

        self._qadr = np.array([m.joint(n).qposadr[0] for n in JOINT_NAMES])
        self._vadr = np.array([m.joint(n).dofadr[0] for n in JOINT_NAMES])
        self._act = np.array([m.actuator(n).id for n in JOINT_NAMES])
        self._tip = m.site(TIP_SITE).id
        self._hand_body = m.body(ee["link"]).id
        self._pelvis = m.body("pelvis").pos.copy()
        self._table_body = m.body(TABLE).id
        self._bottle_body = m.body(BOTTLE).id
        self._n_sub = max(1, int(round(1.0 / self.cfg["control_hz"] / m.opt.timestep)))
        self.max_steps = int(round(self.cfg["episode_s"] * self.cfg["control_hz"]))
        self._hold_steps = max(1, int(round(self.cfg["success"]["hold_s"] * self.cfg["control_hz"])))

        # 接触の相手の分類: 動かす右腕（ハンドの箱を含む）、机、ボトル、それ以外のロボット（体）、床
        arm_bodies = {m.body(f"right_{n}").id for n in ("shoulder_pitch_link", "shoulder_roll_link", "shoulder_yaw_link",
                                                        "elbow_link", "wrist_roll_link", "wrist_pitch_link",
                                                        "wrist_yaw_link")}
        self._cat = np.full(m.nbody, "body", dtype=object)
        self._cat[0] = "world"
        for b in arm_bodies:
            self._cat[b] = "arm"
        self._cat[self._table_body] = "desk"
        self._cat[self._bottle_body] = "bottle"

        # ゲインと、目標角の範囲（関節リミットの内側）
        g = self.cfg["gains"]
        kp, kd = np.zeros(NUM_MOTORS), np.zeros(NUM_MOTORS)
        kp[:12], kd[:12] = g["leg"]["kp"], g["leg"]["kd"]
        kp[list(WAIST_IDX)], kd[list(WAIST_IDX)] = g["waist"]["kp"], g["waist"]["kd"]
        kp[list(LEFT_ARM_IDX)], kd[list(LEFT_ARM_IDX)] = g["other_arm"]["kp"], g["other_arm"]["kd"]
        kp[self.arm], kd[self.arm] = g["arm"]["kp"], g["arm"]["kd"]
        wrist = self.arm[list(WRIST_IDX)]
        kp[wrist], kd[wrist] = g["wrist"]["kp"], g["wrist"]["kd"]
        self._kp = kp
        m.dof_damping[self._vadr] = kd
        margin = float(self.cfg["action"]["limit_margin_rad"])
        rng = np.array([m.jnt_range[m.joint(JOINT_NAMES[i]).id] for i in self.arm])
        self._q_lo, self._q_hi = rng[:, 0] + margin, rng[:, 1] - margin
        self._start_q = np.asarray(self.cfg["start_q"], dtype=float)
        if self._start_q.shape != (NUM_MOTORS,):
            raise ValueError(f"start_q は {NUM_MOTORS} 個: {self._start_q.shape}")

        # 右肩（肩のピッチ関節）の位置（pelvis 座標）。胴体固定なので、開始の姿勢のものをそのまま使う（配置を球に収めるのに使う）
        self.data.qpos[self._qadr] = self._start_q
        mujoco.mj_forward(m, self.data)
        self.shoulder = self.data.body("right_shoulder_pitch_link").xpos - self._pelvis

        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(7,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(OBS_DIM,), dtype=np.float32)

        self._q_des = self._start_q.copy()
        self._prev_action = np.zeros(7)
        self.target = np.zeros(3)
        self.table_edge_x = 0.0
        self.top_z = 0.0
        self._steps = 0
        self._good_steps = 0
        self._touched = False
        self._prev_dist = 0.0
        self._f6 = np.zeros(6)
        # 開始の姿勢ですでに接している（腕, 相手）の組と、そのときの距離
        self._start_dist: dict[tuple[int, int], float] = {}

    # ---- 配置 ---------------------------------------------------------------------------

    def sample_layout(self) -> dict[str, float]:
        """配置（押す点の x、ボトルの y、縁からの隙間、天板の z）を設定の範囲から引く。縁が近すぎる、または押す点が
        右肩から reach_radius_m の球の外（腕が届かない）なら引き直す。"""
        sc = self.cfg["scene"]
        u = self.np_random.uniform
        # 学習だけ、左右の端の配置を多く引く（edge_focus。評価と動画では prob を 0 にする。0 のときは乱数を余計に使わない）
        # 端か中かと右か左かは、引き直しの前に 1 回だけ決める（引き直しのたびに決めると、球で落とされやすい右が減る）
        focus = sc.get("edge_focus") or {}
        y_range = sc["bottle_y"]
        if focus.get("prob", 0.0) > 0.0 and self.np_random.random() < focus["prob"]:
            y_range = [sc["bottle_y"][0], focus["right_below"]] if self.np_random.random() < 0.5 \
                else [focus["left_above"], sc["bottle_y"][1]]
        for _ in range(100):
            lay = {"target_x": u(*sc["target_x"]), "bottle_y": u(*y_range), "gap": u(*sc["gap"]),
                   "top_z": u(*sc["top_z"])}
            if lay["target_x"] - lay["gap"] < sc["table_edge_min_m"]:
                continue
            r = sc.get("reach_radius_m")
            if r is not None:
                target = np.array([lay["target_x"], lay["bottle_y"], lay["top_z"] + sc["press_height_m"]])
                if np.linalg.norm(target - self.shoulder) > r:
                    continue
            return lay
        raise RuntimeError("配置を引けない（scene の範囲を見直す）")

    def set_layout(self, target_x: float, bottle_y: float, gap: float, top_z: float) -> None:
        """机とボトルを置き、押す点を決める（pelvis 座標）。"""
        sc = self.cfg["scene"]
        m = self.model
        edge = target_x - gap
        hx = sc["table_half_xy"][0]
        m.body_pos[self._table_body] = self._pelvis + [edge + hx, sc["table_y"], top_z - self._table_hz]
        r = float(self._bottle_cfg["radius"])
        m.body_pos[self._bottle_body] = self._pelvis + [target_x + r, bottle_y, top_z]
        self.target = np.array([target_x, bottle_y, top_z + sc["press_height_m"]])
        self.table_edge_x, self.top_z = edge, top_z

    # ---- Gymnasium ----------------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None) -> tuple[np.ndarray, dict[str, Any]]:
        """options["layout"] で配置を指定できる（評価用）。無ければランダムに引く。"""
        super().reset(seed=seed)
        mj, m, d = self._mj, self.model, self.data
        lay = (options or {}).get("layout") or self.sample_layout()
        self.set_layout(**lay)
        mj.mj_resetData(m, d)
        d.qpos[self._qadr] = self._start_q
        mj.mj_forward(m, d)
        self._start_dist = {}
        self._start_dist = self._arm_touches()
        self._q_des = self._start_q.copy()
        self._prev_action = np.zeros(7)
        self._steps = 0
        self._good_steps = 0
        self._touched = False
        self._prev_dist = float(np.linalg.norm(self.target - self.tip_pos()))
        return self._obs(), {"layout": lay}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        mj, m, d = self._mj, self.model, self.data
        cfg_r, cfg_s = self.cfg["reward"], self.cfg["success"]
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self._q_des[self.arm] = np.clip(self._q_des[self.arm] + a * self.cfg["action"]["max_step_rad"],
                                        self._q_lo, self._q_hi)
        for _ in range(self._n_sub):
            q = d.qpos[self._qadr]
            # 重力（と遠心力・コリオリ）は qfrc_bias で打ち消す（backend_sim.py の gravity_compensation と同じ）
            d.ctrl[self._act] = self._kp * (self._q_des - q) + d.qfrc_bias[self._vadr]
            mj.mj_step(m, d)
        self._steps += 1

        hit, force, contact_tip = self._contacts()
        tip = self.tip_pos()
        dist = float(np.linalg.norm(self.target - tip))
        axis = self.finger_axis()
        reward = cfg_r["progress"] * (self._prev_dist - dist)
        reward += cfg_r["align"] * float(axis @ PUSH_DIR)
        reward -= cfg_r["action_rate"] * float(np.sum((a - self._prev_action) ** 2))
        reward -= cfg_r["over_force"] * max(0.0, force - cfg_s["force_max_n"])
        self._prev_dist = dist
        self._prev_action = a

        lateral = float(np.linalg.norm((self.target - tip) - ((self.target - tip) @ PUSH_DIR) * PUSH_DIR))
        # 探す時間への罰（2026-11-06: 押す点の 5 cm 以内に入ってから成功まで中央値 1.07 秒、触れたあとに小さく動いて合わせていた）。
        # (1) 押す点の近くにいる間は毎周期少し減点する（成功して終われば、それ以上かからない）
        near_m = cfg_r.get("search_near_m", 0.0)
        if near_m > 0.0 and dist < near_m:
            reward -= cfg_r.get("search_penalty", 0.0)
        # (2) 最初にボトルに触れた周期の横のずれが free_m を超えた分を減点する（触れる前に位置を合わせさせる）
        if not self._touched and force > 0.5:
            self._touched = True
            reward -= cfg_r.get("touch_lateral_penalty", 0.0) * max(0.0, lateral - cfg_r.get("touch_lateral_free_m", 0.0))
        # 指先が面まで届いている（押す点の x より reach_x_tol_m 以上手前ではない）、ボトルとの接触点が指先の近く
        # （ハンドの当たり判定の箱の角で押す抜け穴をふさぐ。2026-11-01 の v2 は接触点が指先から約 50 mm だった）
        reached = (self.target[0] - tip[0]) <= cfg_s["reach_x_tol_m"]
        near_tip = contact_tip <= cfg_s["contact_tip_tol_m"]
        pressing = (lateral <= cfg_s["lateral_tol_m"] and cfg_s["force_min_n"] <= force <= cfg_s["force_max_n"]
                    and reached and near_tip)
        self._good_steps = self._good_steps + 1 if pressing else 0

        terminated, outcome = False, ""
        abort_n = cfg_s.get("force_abort_n")
        if hit:
            reward += cfg_r["collision"]
            terminated, outcome = True, "collision"
        elif abort_n is not None and force > abort_n:
            reward += cfg_r["overforce"]
            terminated, outcome = True, "overforce"
        elif self._good_steps >= self._hold_steps:
            reward += cfg_r["success"]
            terminated, outcome = True, "success"
        truncated = not terminated and self._steps >= self.max_steps
        if truncated:
            outcome = "timeout"
        info = {"outcome": outcome, "dist": dist, "lateral": lateral, "force": force, "hit": hit,
                "contact_tip": contact_tip, "reach_x": float(self.target[0] - tip[0])}
        return self._obs(), float(reward), terminated, truncated, info

    # ---- 状態 ---------------------------------------------------------------------------

    def tip_pos(self) -> np.ndarray:
        """指先（中指の先）の位置（pelvis 座標）。"""
        return self.data.site_xpos[self._tip] - self._pelvis

    def finger_axis(self) -> np.ndarray:
        """指の向き（手先リンクの x 軸）。common/kinematics.py の finger_axis と同じ。"""
        return self.data.xmat[self._hand_body].reshape(3, 3)[:, 0].copy()

    def _arm_touches(self) -> dict[tuple[int, int], float]:
        """腕（ハンドの箱を含む）とボトル以外（机・体）がめり込んでいる組の、(腕の body, 相手の body) → 最も深い距離 [m]。"""
        m, d = self.model, self.data
        out: dict[tuple[int, int], float] = {}
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist >= 0:
                continue
            b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
            a, b = self._cat[b1], self._cat[b2]
            if (a == "arm") == (b == "arm") or "world" in (a, b) or "bottle" in (a, b):
                continue
            key = (b1, b2) if a == "arm" else (b2, b1)
            out[key] = min(out.get(key, 0.0), float(c.dist))
        return out

    def _contacts(self) -> tuple[str, float, float]:
        """(当たった相手 "desk" / "body" / ""、ボトルを押す力の合計 [N]、ボトルとの接触点から指先までの最短距離 [m]。接触なしは inf)。

        腕と机・体の接触は、めり込んだもの（dist < 0）だけを数える（肩のリンクは設計上 胴体の近くにあるため。common/collision.py）。
        開始の姿勢ですでにめり込んでいる組（実機の立った姿勢では、下ろした手が股関節の当たり判定に −8〜−10 mm 入っている。
        ハンドの当たり判定はメッシュを包む箱で実物より大きい）は、開始より start_extra_m 以上深くなったときだけ数える
        （common/press_planner.py と同じ考え方）。
        """
        m, d = self.model, self.data
        force, contact_tip = 0.0, float("inf")
        tip_w = d.site_xpos[self._tip]
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = self._cat[m.geom_bodyid[c.geom1]], self._cat[m.geom_bodyid[c.geom2]]
            if "bottle" in (a, b) and "arm" in (a, b):
                self._mj.mj_contactForce(m, d, i, self._f6)
                force += abs(float(self._f6[0]))
                contact_tip = min(contact_tip, float(np.linalg.norm(c.pos - tip_w)))
        extra = float(self.cfg["collision"]["start_extra_m"])
        for (arm_b, other_b), dist in self._arm_touches().items():
            start = self._start_dist.get((arm_b, other_b))
            if start is None or dist < start - extra:
                return str(self._cat[other_b]), force, contact_tip
        return "", force, contact_tip

    def _obs(self) -> np.ndarray:
        d = self.data
        q = d.qpos[self._qadr][self.arm]
        dq = d.qvel[self._vadr][self.arm]
        tip = self.tip_pos()
        return np.concatenate([
            q, 0.1 * dq, self._q_des[self.arm] - q, tip, self.target - tip, self.finger_axis(),
            [self.table_edge_x, self.top_z], self._prev_action,
        ]).astype(np.float32)
