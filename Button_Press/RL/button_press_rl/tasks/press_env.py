"""ボタン押しの学習環境（教師）。右腕 7 関節で、一般用の ▼ を押す。

1 周期（50 Hz）の流れ（Isaac Lab の DirectRLEnv）:
  _pre_physics_step（行動 → 右腕の目標の角度）→ 物理 10 ステップ → _get_dones（点灯・力・打ち切りの判定）
  → _get_rewards → 終わった環境の _reset_idx → _get_observations

評価環境（Yada）とそろえていること:
- 乗り場・ボタン・G1 のモデル・PD の強さ・初期の姿勢・物理の刻み・制御の周期（spec.py、hall_usd.py）
- 目標の角度の丸め方（proprio.step_command。ランナーの安全のための処理と同じ）
- 点灯の判定（沈みが press_depth 以上で点灯し、そのまま点灯し続ける）
- 押す力（ボタンが受ける接触力。評価の press_force と同じ量）と、点灯のあと post_press_s 続けること

ボタンの位置の違いは、ロボットの立つ位置をずらして作る（spec.py の offset_ranges）。
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_from_euler_xyz, sample_uniform

from ..hall_usd import BUTTONS_REL, SCENE_NAME, write_hall_usd
from ..paths import use_yada
from ..proprio import RIGHT_ARM_JOINTS, policy_obs, proprio, step_command
from ..rewards import RewardInputs, compute_rewards
from ..spec import TaskSpec, load_task_spec

# 指先以外の接触力を測る右腕のリンク（手より根元）。手（指先の球を含む）はボタンを押すので別に扱う
ARM_LINKS = ("right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link",
             "right_elbow_link", "right_wrist_roll_link", "right_wrist_pitch_link")
from .press_env_cfg import ButtonPressEnvCfg

ENV_NS = "/World/envs/env_.*"
ENV0 = "/World/envs/env_0"


def _t(x) -> torch.Tensor:
    """Isaac Lab 3.0 のデータ（ProxyArray / warp の配列）を torch のテンソルにする。"""
    if isinstance(x, torch.Tensor):
        return x
    if hasattr(x, "torch"):
        return x.torch
    import warp as wp

    return wp.to_torch(x)


class ButtonPressEnv(DirectRLEnv):
    cfg: ButtonPressEnvCfg

    def __init__(self, cfg: ButtonPressEnvCfg, render_mode: str | None = None, **kwargs):
        self.task: TaskSpec = load_task_spec(cfg.target)
        super().__init__(cfg, render_mode, **kwargs)
        dev = self.device
        n = self.num_envs

        # ---- ロボット ----
        self._arm_ids = torch.tensor(self._robot.find_joints(list(RIGHT_ARM_JOINTS), preserve_order=True)[0],
                                     device=dev)
        lim = _t(self._robot.data.joint_pos_limits)[0, self._arm_ids]
        m = self.task.limit_margin
        self._lo, self._hi = lim[:, 0] + m, lim[:, 1] - m
        self._q0 = _t(self._robot.data.default_joint_pos)[0, self._arm_ids].clone()
        self._kp0 = _t(self._robot.data.joint_stiffness)[0, self._arm_ids].clone()
        self._targets = _t(self._robot.data.default_joint_pos).clone()
        self._hand_body = self._robot.find_bodies(self.task.fingertip_link)[0][0]
        self._tip_offset = torch.tensor(self.task.fingertip_offset, device=dev).repeat(n, 1)

        # ---- ボタン ----
        names = [b["name"] for b in self.task.scene_cfg["buttons"]["items"]]
        self._button_names = names
        self._button_joint_ids = torch.tensor(
            [self._hall.find_joints(f"button_{b}_slide")[0][0] for b in names], device=dev)
        self._button_body_ids = torch.tensor([self._hall.find_bodies(f"button_{b}")[0][0] for b in names], device=dev)
        bs = [b.removeprefix("button_") for b in self._button_contact.body_names]
        self._button_sensor_ids = torch.tensor([bs.index(b) for b in names], device=dev)
        self._tgt = names.index(cfg.target)
        self._others = [i for i in range(len(names)) if i != self._tgt]
        # 押す向き（乗り場の +x）
        self._axis_w = torch.tensor([1.0, 0.0, 0.0], device=dev).repeat(n, 1)

        # ---- 試行ごとの状態 ----
        na = len(RIGHT_ARM_JOINTS)
        self._q_cmd = self._q0.repeat(n, 1)
        self._prev_cmd = self._q_cmd.clone()
        self._actions = torch.zeros(n, na, device=dev)
        self._last_actions = torch.zeros(n, na, device=dev)
        self._delay = torch.zeros(n, dtype=torch.bool, device=dev)
        self._lit = torch.zeros(n, len(names), dtype=torch.bool, device=dev)
        self._lit_step = torch.full((n,), -1, dtype=torch.long, device=dev)
        self._max_f_ep = torch.zeros(n, device=dev)
        self._post_steps = int(round(self.task.post_press_s * self.task.control_hz))
        self._ep = {k: torch.zeros(n, device=dev) for k in
                    ("success", "wrong", "released", "in_band", "force_abort", "max_force", "tip_press")}
        # 試行の中での、指先の球の中心とボタンの面の中心の最も近い距離 [m]（学習が進んでいるかの目安）
        self._ep["min_dist"] = torch.full((n,), 10.0, device=dev)
        self._reward_sums: dict[str, torch.Tensor] = {}
        # 終わった試行の記録（scripts/eval_teacher.py が分布を見る。cfg.record_episodes のときだけ）
        self.episode_records: list[dict[str, float]] = []
        # _get_dones で求めて _get_rewards / _get_observations で使う値
        self._cache: dict[str, torch.Tensor] = {}

        o = self.task.offsets
        mg = 1.0 + cfg.rand.offset_margin
        self._off_lo = torch.tensor([_widen(o.x, mg)[0], _widen(o.y, mg)[0], _widen(o.z, mg)[0], _widen(o.yaw, mg)[0]],
                                    device=dev)
        self._off_hi = torch.tensor([_widen(o.x, mg)[1], _widen(o.y, mg)[1], _widen(o.z, mg)[1], _widen(o.yaw, mg)[1]],
                                    device=dev)
        self._pelvis_h = float(self.task.scene_cfg["pelvis_height"])
        print(f"[press_env] 環境 {n}、押すボタン {cfg.target}、立つ位置のずらし幅 x {o.x} y {o.y} z {o.z} "
              f"yaw {tuple(round(math.degrees(v), 1) for v in o.yaw)} deg（さらに {cfg.rand.offset_margin:.0%} 広げる）")

    # ---- シーン ----------------------------------------------------------------

    def _setup_scene(self) -> None:
        use_yada(isaac=True)
        from isaac_hall import hall_articulation_cfg
        from isaac_world import add_fingertips, find_link_path
        from pxr import PhysxSchema

        from common.j1gen_bridge import j1gen_config
        from common.scene_spec import build_hall_scene

        spec = load_task_spec(self.cfg.target)
        # 乗り場（USD に書き出したものを env_0 に読み込み、あとで全環境に複製する）
        usd = write_hall_usd(spec.scene_cfg)
        sim_utils.UsdFileCfg(usd_path=str(usd)).func(f"{ENV0}/{SCENE_NAME}", sim_utils.UsdFileCfg(usd_path=str(usd)))
        self._robot = Articulation(self.cfg.robot)
        add_fingertips(sim_utils.get_current_stage(), j1gen_config("robot.yaml"), spec.fingertip_radius,
                       root=f"{ENV0}/Robot")
        hall_cfg = hall_articulation_cfg(build_hall_scene(spec.scene_cfg)).replace(prim_path=f"{ENV_NS}/{BUTTONS_REL}")
        self._hall = Articulation(hall_cfg)
        h = self.cfg.decimation
        self._button_contact = ContactSensor(ContactSensorCfg(prim_path=f"{ENV_NS}/{BUTTONS_REL}/button_.*",
                                                              history_length=h))
        # G1 のリンクは親子の入れ子（腕の先ほど深い）。Isaac Lab の接触センサは同じ階層に並んだリンクしか
        # まとめて測れないので、リンクごとに 1 つ作る（prim の場所は env_0 で探す）
        stage = sim_utils.get_current_stage()

        def link_sensor(name: str) -> ContactSensor:
            path = find_link_path(stage, f"{ENV0}/Robot", name)
            # Isaac Lab の activate_contact_sensors は最初の剛体（pelvis）より下を探さないので、自分で付ける
            PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.0)
            return ContactSensor(ContactSensorCfg(prim_path=path.replace(ENV0, ENV_NS, 1), history_length=h))

        self._hand_contact = link_sensor(spec.fingertip_link)
        self._arm_contacts = [link_sensor(n) for n in ARM_LINKS]

        self.scene.clone_environments(copy_from_source=False)
        self.scene.articulations["robot"] = self._robot
        self.scene.articulations["hall"] = self._hall
        self.scene.sensors["button_contact"] = self._button_contact
        self.scene.sensors["hand_contact"] = self._hand_contact
        for name, sensor in zip(ARM_LINKS, self._arm_contacts):
            self.scene.sensors[f"contact_{name}"] = sensor
        light = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.95, 0.95, 0.95))
        light.func("/World/Light", light)

    # ---- 行動 ------------------------------------------------------------------

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self._last_actions = self._actions.clone()
        self._actions = torch.clamp(actions, -1.0, 1.0)
        self._prev_cmd = self._q_cmd.clone()
        self._q_cmd = step_command(self._q_cmd, self._actions, self._lo, self._hi, torch.clamp)
        # 指令の遅れ: 遅れる環境は、1 周期前の目標を送る
        cmd = torch.where(self._delay.unsqueeze(-1), self._prev_cmd, self._q_cmd)
        self._targets[:, self._arm_ids] = cmd

    def _apply_action(self) -> None:
        self._robot.set_joint_position_target_index(target=self._targets)

    # ---- 判定 ------------------------------------------------------------------

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        c = self._cache
        # ボタンの沈みと点灯
        depth = _t(self._hall.data.joint_pos)[:, self._button_joint_ids]
        new_lit = (depth >= self.task.press_depth) & ~self._lit
        tgt_before = self._lit[:, self._tgt].clone()
        self._lit |= new_lit
        wrong_event = new_lit[:, self._others].any(-1)
        lit_event = new_lit[:, self._tgt] & ~wrong_event & ~tgt_before
        self._lit_step = torch.where(lit_event, self.episode_length_buf, self._lit_step)
        # 中指の先で押したか（点灯した周期の指先の位置で決める。手の別の部分で押しても点灯はするため）
        tip_press = self._fingertip_on_button()

        # 押す力（周期の中の最大と、最後の値）
        fh = torch.linalg.norm(_t(self._button_contact.data.net_forces_w_history), dim=-1)  # (N, T, B)
        fh = fh[:, :, self._button_sensor_ids]
        f_all = fh.max(1).values
        f_tgt = f_all[:, self._tgt]
        f_now = fh[:, 0, self._tgt]
        self._max_f_ep = torch.maximum(self._max_f_ep, f_tgt)

        # 指先以外の接触力: 右腕のリンクの力 + 手の力のうちボタンを押す力を超えた分
        def peak(sensor: ContactSensor) -> torch.Tensor:
            return torch.linalg.norm(_t(sensor.data.net_forces_w_history), dim=-1).max(1).values.sum(-1)  # (N,)

        f_arm = torch.stack([peak(s) for s in self._arm_contacts], 0).sum(0)
        f_hand = torch.clamp(peak(self._hand_contact) - f_all.sum(-1), min=0.0)
        unwanted = f_arm + f_hand

        lit_tgt = self._lit[:, self._tgt]
        since = self.episode_length_buf - self._lit_step
        in_post = lit_tgt & (since < self._post_steps) & ~lit_event
        post_end = lit_tgt & (since >= self._post_steps)
        released_event = post_end & (f_now < self.task.release_n)
        force_abort = f_tgt > self.cfg.force_abort_scale * self.task.press_max

        terminated = post_end
        if self.cfg.terminate_on_failure:
            terminated = terminated | wrong_event | force_abort
        truncated = self.episode_length_buf >= self.max_episode_length - 1

        c.update(depth=depth, f_tgt=f_tgt, f_now=f_now, unwanted=unwanted, lit_event=lit_event,
                 wrong_event=wrong_event, released_event=released_event, force_abort=force_abort,
                 in_post=in_post, tip_press=tip_press, end_event=post_end)
        # 試行ごとの記録（ログ用）
        lo, hi = self.task.press_ok
        e = self._ep
        e["success"] = torch.maximum(e["success"], lit_event.float())
        e["wrong"] = torch.maximum(e["wrong"], wrong_event.float())
        e["released"] = torch.maximum(e["released"], released_event.float())
        e["force_abort"] = torch.maximum(e["force_abort"], force_abort.float())
        # 適正な範囲か: 試行全体の最大で見る（評価環境の press_force_in_range と同じ）。post_end で確定
        band = (self._max_f_ep >= lo) & (self._max_f_ep <= hi)
        e["in_band"] = torch.where(post_end, band.float(), e["in_band"])
        e["tip_press"] = torch.where(lit_event, tip_press.float(), e["tip_press"])
        e["max_force"] = self._max_f_ep
        return terminated, truncated

    # ---- 報酬 ------------------------------------------------------------------

    def _get_rewards(self) -> torch.Tensor:
        c = self._cache
        tip, tip_vel, face, axis = self._geometry()
        at_limit = ((self._q_cmd <= self._lo + 1e-4) | (self._q_cmd >= self._hi - 1e-4)).float().sum(-1)
        x = RewardInputs(
            tip=tip, tip_vel=tip_vel, face=face, axis=axis, tip_radius=self.task.fingertip_radius,
            depth=c["depth"][:, self._tgt], travel=self.task.travel, force=c["f_tgt"],
            max_force_ep=self._max_f_ep, unwanted_force=c["unwanted"], in_post=c["in_post"],
            lit_event=c["lit_event"], wrong_event=c["wrong_event"], released_event=c["released_event"],
            end_event=c["end_event"],
            force_abort=c["force_abort"], wrong_lit=self._lit[:, self._others].any(-1),
            tip_press=c["tip_press"],
            action=self._actions, last_action=self._last_actions,
            dq=_t(self._robot.data.joint_vel)[:, self._arm_ids], at_limit=at_limit,
            press_ok=self.task.press_ok, release_n=self.task.release_n,
        )
        total, terms = compute_rewards(x, self.cfg.rewards)
        self._ep["min_dist"] = torch.minimum(self._ep["min_dist"], torch.linalg.norm(tip - face, dim=-1))
        for k, v in terms.items():
            self._reward_sums[k] = self._reward_sums.get(k, torch.zeros_like(v)) + v
        return total

    # ---- 観測 ------------------------------------------------------------------

    def _get_observations(self) -> dict[str, torch.Tensor]:
        c = self._cache
        if not getattr(self, "_printed_geometry", False):
            self._printed_geometry = True
            tip, _, face, axis = self._geometry()
            print(f"[press_env] env 0: 指先 {tip[0].tolist()}、▼ の面 {face[0].tolist()}、押す向き {axis[0].tolist()}"
                  "（腰の座標 [m]）")
        r = self.cfg.rand
        q = _t(self._robot.data.joint_pos)[:, self._arm_ids]
        dq = _t(self._robot.data.joint_vel)[:, self._arm_ids]
        q_obs = q + r.q_noise * torch.randn_like(q)
        dq_obs = dq + r.dq_noise * torch.randn_like(dq)
        prop = proprio(q_obs, dq_obs, self._q_cmd, self._q0, self._actions, torch.cat)
        tip, _, face, _ = self._geometry()
        depth = _t(self._hall.data.joint_pos)[:, self._button_joint_ids]
        lit_tgt = self._lit[:, self._tgt].float()
        since = (self.episode_length_buf - self._lit_step).float()
        post = torch.where(self._lit[:, self._tgt], torch.clamp(since / max(self._post_steps, 1), 0.0, 1.0),
                           torch.zeros_like(since))
        # 方策: 体の感覚 + ボタンの面と指先の位置（腰の座標）。評価用の Agent（agent/agent.py）と同じ並び
        policy = policy_obs(prop, face, tip, torch.cat)
        f = c.get("f_tgt", torch.zeros(self.num_envs, device=self.device))
        u = c.get("unwanted", torch.zeros(self.num_envs, device=self.device))
        critic = torch.cat([policy,
                            (depth[:, self._tgt] / self.task.press_depth).unsqueeze(-1),
                            lit_tgt.unsqueeze(-1), post.unsqueeze(-1),
                            (f / 10.0).unsqueeze(-1), torch.clamp(u / 10.0, max=5.0).unsqueeze(-1),
                            self._lit[:, self._others].float()], -1)
        return {"policy": policy, "critic": critic}

    # ---- リセット --------------------------------------------------------------

    def _reset_idx(self, env_ids: Sequence[int] | torch.Tensor | None) -> None:
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        # Isaac Lab の書き込みの関数は、番号を int32 で受け取る
        ids32 = env_ids.to(torch.int32)
        arm32 = self._arm_ids.to(torch.int32)
        self._log_episodes(env_ids)
        super()._reset_idx(env_ids)
        k = len(env_ids)
        dev = self.device
        r = self.cfg.rand

        # ロボットの立つ位置（乗り場に対して）
        off = sample_uniform(self._off_lo, self._off_hi, (k, 4), dev)
        pos = self.scene.env_origins[env_ids] + torch.stack(
            [off[:, 0], off[:, 1], self._pelvis_h + off[:, 2]], -1)
        zero = torch.zeros(k, device=dev)
        quat = quat_from_euler_xyz(zero, zero, off[:, 3])
        self._robot.write_root_pose_to_sim_index(root_pose=torch.cat([pos, quat], -1), env_ids=ids32)

        # 関節（右腕だけ少しずらす）
        q = _t(self._robot.data.default_joint_pos)[env_ids].clone()
        q_arm = q[:, self._arm_ids] + sample_uniform(-r.init_q_noise, r.init_q_noise, (k, len(self._arm_ids)), dev)
        q_arm = torch.clamp(q_arm, self._lo, self._hi)
        q[:, self._arm_ids] = q_arm
        self._robot.write_joint_position_to_sim_index(position=q, env_ids=ids32)
        self._robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q), env_ids=ids32)
        self._targets[env_ids] = q
        self._robot.set_joint_position_target_index(target=q, env_ids=ids32)

        # 右腕の PD の強さと、回転子の見かけの慣性
        kp = self._kp0 * sample_uniform(r.kp_scale[0], r.kp_scale[1], (k, len(self._arm_ids)), dev)
        self._robot.write_joint_stiffness_to_sim_index(stiffness=kp, joint_ids=arm32, env_ids=ids32)
        arm = sample_uniform(r.armature[0], r.armature[1], (k, len(self._arm_ids)), dev)
        self._robot.write_joint_armature_to_sim_index(armature=arm, joint_ids=arm32, env_ids=ids32)

        # ボタン
        zb = torch.zeros(k, self._hall.num_joints, device=dev)
        self._hall.write_joint_position_to_sim_index(position=zb, env_ids=ids32)
        self._hall.write_joint_velocity_to_sim_index(velocity=zb, env_ids=ids32)

        # 試行ごとの状態
        self._q_cmd[env_ids] = q_arm
        self._prev_cmd[env_ids] = q_arm
        self._actions[env_ids] = 0.0
        self._last_actions[env_ids] = 0.0
        self._delay[env_ids] = torch.rand(k, device=dev) < r.command_delay_prob
        self._lit[env_ids] = False
        self._lit_step[env_ids] = -1
        self._max_f_ep[env_ids] = 0.0
        for v in self._ep.values():
            v[env_ids] = 0.0
        self._ep["min_dist"][env_ids] = 10.0

    # ---- 内部 ------------------------------------------------------------------

    def _fingertip_on_button(self) -> torch.Tensor:
        """(N,) bool 中指の先の球がボタンの面に触れる位置にあるか（すき間 tip_gap_m 以内、横のずれが半径 + tip_lateral_m 以内）。"""
        tip, _, face, axis = self._geometry()
        rel = tip - face
        along = (rel * axis).sum(-1)
        lateral = torch.linalg.norm(rel - along.unsqueeze(-1) * axis, dim=-1)
        gap = -along - self.task.fingertip_radius
        return (gap < self.cfg.tip_gap_m) & (lateral < self.task.button_radius + self.cfg.tip_lateral_m)

    def _geometry(self) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """(指先, 指先の速度, 目標のボタンの面（押していないとき）, 押す向き)。どれもロボットの腰の座標。"""
        rd = self._robot.data
        root_p = _t(rd.root_link_pos_w)
        root_q = _t(rd.root_link_quat_w)
        hp = _t(rd.body_link_pos_w)[:, self._hand_body]
        hq = _t(rd.body_link_quat_w)[:, self._hand_body]
        hv = _t(rd.body_link_lin_vel_w)[:, self._hand_body]
        hw = _t(rd.body_link_ang_vel_w)[:, self._hand_body]
        r = quat_apply(hq, self._tip_offset)
        tip_w = hp + r
        vel_w = hv + torch.cross(hw, r, dim=-1)
        depth = _t(self._hall.data.joint_pos)[:, self._button_joint_ids[self._tgt]]
        bp = _t(self._hall.data.body_link_pos_w)[:, self._button_body_ids[self._tgt]]
        face_w = bp - depth.unsqueeze(-1) * self._axis_w
        tip = quat_apply_inverse(root_q, tip_w - root_p)
        vel = quat_apply_inverse(root_q, vel_w)
        face = quat_apply_inverse(root_q, face_w - root_p)
        axis = quat_apply_inverse(root_q, self._axis_w)
        return tip, vel, face, axis

    def _log_episodes(self, env_ids: torch.Tensor) -> None:
        """終わった試行の成功率・押す力・報酬の項を、学習のログに出す。"""
        if len(env_ids) == 0:
            return
        log = self.extras.setdefault("log", {})
        e = self._ep
        if self.cfg.record_episodes:
            keys = list(e)
            cols = torch.stack([e[k][env_ids] for k in keys], -1).cpu().tolist()
            self.episode_records += [dict(zip(keys, row)) for row in cols]
        succ = e["success"][env_ids]
        log["Metrics/success_rate"] = succ.mean().item()
        log["Metrics/wrong_rate"] = e["wrong"][env_ids].mean().item()
        log["Metrics/force_abort_rate"] = e["force_abort"][env_ids].mean().item()
        log["Metrics/min_dist_m"] = e["min_dist"][env_ids].mean().item()
        if succ.sum() > 0:
            m = succ > 0
            log["Metrics/in_band_rate"] = e["in_band"][env_ids][m].mean().item()
            log["Metrics/tip_press_rate"] = e["tip_press"][env_ids][m].mean().item()
            log["Metrics/released_rate"] = e["released"][env_ids][m].mean().item()
            log["Metrics/max_press_force_n"] = e["max_force"][env_ids][m].mean().item()
        for k, v in self._reward_sums.items():
            log[f"Episode_Reward/{k}"] = (v[env_ids].mean() / self.max_episode_length_s).item()
            v[env_ids] = 0.0


def _widen(r: tuple[float, float], scale: float) -> tuple[float, float]:
    """範囲 r を、中心のまわりに scale 倍に広げる。"""
    c, h = (r[0] + r[1]) / 2.0, (r[1] - r[0]) / 2.0 * scale
    return c - h, c + h
