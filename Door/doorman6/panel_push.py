"""3c の押し方の新案をシムで測る口（パッチ 0115、2026-11-16）。

計画 docs/plan/2026-11-16-g1-door-push-line-two-hands-sim.md。台本の教師（script_teacher.py）が持つ。
  PanelPush: 扉の板の接触センサ（両手首・胴を 1 つずつ filter にした 3 つ・当たる点つき。door_open_homie が env.config.panel_contact.enabled
    で作る。体ごとに分けたのは並びの取り違えを避けるため。1 つに並べた形でも値は同じだった）と
    レバー‐手の接触センサから、体が扉から受ける力の合計と、pelvis の鉛直の軸の回りの回す力 Mz を毎周期出す。
    3c・4a の間を台ごとに貯め、回ごとの結果にまとめる。押されて下がる脚（pelvis_replay.cmd_follow.src panel）が読む
  ArmComply: 3c・4a の間だけ腕の関節の硬さを kp_scale 倍に下げ、重力の補償（PhysX の gravity_compensation_forces）を
    力の先回りで足す（柔らかい腕・インピーダンスの案 A のシムの形。行き先を板より先に置いた分のばねで押す）。
    ⚠️ 重力の補償はシムの本当の値（実機では模型の値になり、ずれる）
体を回す力 Mz ＝ Σ（当たる点 − pelvis）×（体が受ける力）の鉛直の成分（世界の座標・左回りが正）。
効く横のずれ e ＝ ΣMz ÷ Σ押し返し（押し返しが 2 N 以上の周期）。押す力の大きさによらない、力の線と体の中心との横のずれ。
"""

from __future__ import annotations

import numpy as np
import torch

PANEL_SENSOR = "door_panel_push_contact_sensor"  # 後ろに _<体の名前> を付けた 3 つ
PANEL_BODIES = ("left_wrist_yaw_link", "right_wrist_yaw_link", "torso_link")
PUSH_SUBS = ("3c", "4a")
PUSH_MIN_N = 2.0     # e を出す周期の押し返しの下限 [N]
TOUCH_N = 2.0        # 「当たっている」とみる力 [N]
# 回ごとに貯める列（1 周期 1 行）
ROW = ("mz", "push", "lat", "f_left", "f_right", "f_torso", "f_lever", "lw_x", "lw_y", "rw_x", "rw_y")  # 手首は pelvis の向きの座標 [m]


def panel_sensor_name(body: str) -> str:
    return f"{PANEL_SENSOR}_{body}"


def tt(x) -> torch.Tensor:
    """Isaac Lab 3.0 の配列（ProxyArray・warp）を torch に（torch の配列はそのまま）"""
    if isinstance(x, torch.Tensor):
        return x
    if hasattr(x, "torch"):
        return x.torch
    import warp as wp  # noqa: PLC0415

    return wp.to_torch(x)


def yaw_moment(f: torch.Tensor, p: torch.Tensor, c: torch.Tensor) -> torch.Tensor:
    """体を回す力 Mz (N,)。f (N,K,3) 体が受ける力・p (N,K,3) 当たる点・c (N,3) pelvis（どれも世界の座標）。
    当たる点か力が nan の所は 0（当たっていない）"""
    ok = torch.isfinite(p).all(-1) & torch.isfinite(f).all(-1)
    f0 = torch.where(ok[..., None], f, torch.zeros_like(f))
    r = torch.where(ok[..., None], p - c[:, None, :], torch.zeros_like(p))
    return (r[..., 0] * f0[..., 1] - r[..., 1] * f0[..., 0]).sum(-1)


def to_body_xy(f_xy: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    """世界の水平の力 (N,2) を pelvis の向きの座標（前・左）へ"""
    c, s = torch.cos(yaw), torch.sin(yaw)
    return torch.stack([c * f_xy[:, 0] + s * f_xy[:, 1], -s * f_xy[:, 0] + c * f_xy[:, 1]], dim=-1)


def yaw_from_quat_xyzw(q: torch.Tensor) -> torch.Tensor:
    x, y, z, w = q.unbind(-1)
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def summarize_rows(rows: np.ndarray) -> dict:
    """回ごとのまとめ。rows (T, len(ROW))＝3c・4a の周期の値"""
    if rows is None or len(rows) == 0:
        return {"n": 0}
    r = np.asarray(rows, dtype=float)
    mz, push, fl, fr, ftor, flev = r[:, 0], r[:, 1], r[:, 3], r[:, 4], r[:, 5], r[:, 6]
    on = push >= PUSH_MIN_N
    hands = fl + fr
    out = {
        "n": int(len(r)),
        "push_frac": round(float(on.mean()), 3),
        "mz_mean": round(float(mz.mean()), 2), "mz_abs_mean": round(float(np.abs(mz).mean()), 2),
        "mz_p90": round(float(np.percentile(np.abs(mz), 90)), 2),
        "push_med": round(float(np.median(push)), 1), "push_p90": round(float(np.percentile(push, 90)), 1),
        "push_max": round(float(push.max()), 1),
        "e": round(float(mz[on].sum() / push[on].sum()), 3) if on.sum() >= 5 else None,
        "push_std_on": round(float(push[on].std()), 1) if on.sum() >= 5 else None,
        "f_left_med": round(float(np.median(fl)), 1), "f_right_med": round(float(np.median(fr)), 1),
        "right_share": round(float(fr.sum() / hands.sum()), 3) if hands.sum() > 1e-6 else None,
        "torso_frac": round(float((ftor >= TOUCH_N).mean()), 3), "lever_frac": round(float((flev >= TOUCH_N).mean()), 3),
        "right_touch_frac": round(float((fr >= TOUCH_N).mean()), 3),
    }
    if r.shape[1] >= 11:
        out["wrist_body_med"] = np.round(np.median(r[:, 7:11], axis=0), 3).tolist()  # 左の前・横・右の前・横
    return out
    return out


class PanelPush:
    """measure(env, sub_code) を台本の教師の apply の始めに毎周期呼ぶ。summary(i)・reset_env(i) は回の終わり"""

    def __init__(self, env, n: int, sub_names) -> None:
        self.sensors = [env.simulator.scene.sensors[panel_sensor_name(b)] for b in PANEL_BODIES]
        self.robot = env.simulator._robot
        ids, _ = self.robot.find_bodies(["left_wrist_yaw_link", "right_wrist_yaw_link"], preserve_order=True)
        self.wrist_ids = list(ids)
        self.lever_idx = (list(env.left_hand_indices_tgt_ct_sensor), list(env.right_hand_indices_tgt_ct_sensor))
        self.codes = torch.as_tensor([list(sub_names).index(s) for s in PUSH_SUBS], device=env.device)
        self.n = n
        self.rows: list = [[] for _ in range(n)]
        print(f"[panel_push] 板の接触センサ {PANEL_SENSOR}（{', '.join(PANEL_BODIES)}）とレバーの接触から Mz を測る（{n} 台）", flush=True)

    def measure(self, env, sub_code: torch.Tensor) -> dict:
        fm = torch.stack([tt(s.data.force_matrix_w)[:, 0, 0] for s in self.sensors], dim=1)  # (N,3,3) 板が受ける力（左手首・右手首・胴から）
        cp = torch.stack([tt(s.data.contact_pos_w)[:, 0, 0] if s.data.contact_pos_w is not None
                          else torch.full_like(fm[:, 0], float("nan")) for s in self.sensors], dim=1)
        f_panel, p_panel = -fm, cp
        lev = env.simulator.object_to_hand_contact_forces[:, 0]  # (N,H,3) レバーが受ける力（手から）
        f_lev = torch.stack([-lev[:, self.lever_idx[0]].sum(1), -lev[:, self.lever_idx[1]].sum(1)], dim=1)
        p_lev = tt(self.robot.data.body_pos_w)[:, self.wrist_ids]  # レバーの当たる点は手首で代える
        pose = tt(self.robot.data.root_link_pose_w)
        yaw = yaw_from_quat_xyzw(pose[:, 3:7])
        f = torch.cat([f_panel, f_lev], dim=1)
        p = torch.cat([p_panel, p_lev], dim=1)
        # 板で当たる点が無い（nan）周期は、その力も 0 と数える（PhysX は当たりが無い時に nan を返す）
        f_ok = torch.where((torch.isfinite(p).all(-1) & torch.isfinite(f).all(-1))[..., None], f, torch.zeros_like(f))
        mz = yaw_moment(f, p, pose[:, :3])
        fb = to_body_xy(f_ok.sum(1)[:, :2], yaw)
        mag = f_ok.norm(dim=-1)
        wb = [to_body_xy(p_lev[:, k, :2] - pose[:, :2], yaw) for k in (0, 1)]  # 手首（pelvis の向きの座標）
        out = {"mz": mz, "push": -fb[:, 0], "lat": fb[:, 1], "f_left": mag[:, 0], "f_right": mag[:, 1],
               "f_torso": mag[:, 2], "f_lever": mag[:, 3] + mag[:, 4],
               "lw_x": wb[0][:, 0], "lw_y": wb[0][:, 1], "rw_x": wb[1][:, 0], "rw_y": wb[1][:, 1]}
        on = torch.isin(sub_code[: self.n], self.codes)
        if bool(on.any()):
            vals = torch.stack([out[k][: self.n] for k in ROW], dim=-1).detach().cpu().numpy()
            for i in torch.nonzero(on).flatten().tolist():
                self.rows[i].append(vals[i])
        return out

    def summary(self, i: int) -> dict:
        return summarize_rows(np.array(self.rows[i]) if self.rows[i] else None)

    def reset_env(self, i: int) -> None:
        self.rows[i] = []


class ArmComply:
    """step(env, sub_code, use_left, use_right) を台本の教師の apply の終わりに毎周期呼ぶ。reset(env_ids) は回の戻し。
    use_left・use_right (N,) bool: その台で左・右の腕を柔らかくしてよいか（把手の側の腕、と添えた手を使う台の反対の腕）"""

    def __init__(self, env, cfg: dict, sim_from_real: np.ndarray, sub_names) -> None:
        self.robot = env.simulator._robot
        dof_ids = list(env.simulator.dof_ids)
        # SDK の並びの左腕 15〜21・右腕 22〜28 → シムの関節の番号 → 物理の関節の番号
        self.j = (
            [int(dof_ids[int(sim_from_real[r])]) for r in range(15, 22)],
            [int(dof_ids[int(sim_from_real[r])]) for r in range(22, 29)],
        )
        self.all_j = self.j[0] + self.j[1]
        self.kp_scale = float(cfg.get("kp_scale", 0.3))
        self.kd_scale = float(cfg.get("kd_scale", 1.0))
        self.gravity = bool(cfg.get("gravity", True))
        assert 0.02 <= self.kp_scale <= 1.0 and 0.1 <= self.kd_scale <= 2.0, f"comply の値: {cfg}"
        self.codes = torch.as_tensor([list(sub_names).index(s) for s in PUSH_SUBS], device=env.device)
        n = env.num_envs
        self.on = torch.zeros(2, n, dtype=torch.bool, device=env.device)
        self.k0 = None
        self.d0 = None
        self.nj = int(self.robot.num_joints)
        print(f"[arm_comply] 3c・4a の腕の硬さ ×{self.kp_scale}・粘り ×{self.kd_scale}・重力の補償 {self.gravity}", flush=True)

    def _write(self, side: int, envs: torch.Tensor, soft: torch.Tensor) -> None:
        ids = self.j[side]
        k = self.k0[envs][:, ids] * torch.where(soft[:, None], self.kp_scale, 1.0)
        d = self.d0[envs][:, ids] * torch.where(soft[:, None], self.kd_scale, 1.0)
        self.robot.write_joint_stiffness_to_sim_index(stiffness=k.contiguous(), joint_ids=ids, env_ids=envs)
        self.robot.write_joint_damping_to_sim_index(damping=d.contiguous(), joint_ids=ids, env_ids=envs)

    def step(self, env, sub_code: torch.Tensor, use_left: torch.Tensor, use_right: torch.Tensor) -> None:
        if self.k0 is None:
            self.k0 = tt(self.robot.data.joint_stiffness).clone()
            self.d0 = tt(self.robot.data.joint_damping).clone()
        push = torch.isin(sub_code, self.codes)
        want = torch.stack([push & use_left, push & use_right])
        for side in (0, 1):
            ch = torch.nonzero(want[side] != self.on[side]).flatten()
            if len(ch):
                self._write(side, ch, want[side][ch])
        self.on = want
        if not self.gravity:
            return
        g = tt(self.robot.data.gravity_compensation_forces)
        off = int(g.shape[1]) - self.nj  # 動く根元の 6 列が前に付く版がある
        assert off in (0, 6), f"gravity_compensation_forces の列 {g.shape[1]} と関節 {self.nj} が合わない"
        cols = torch.as_tensor([off + j for j in self.all_j], device=g.device)
        mask = torch.cat([want[0][:, None].expand(-1, 7), want[1][:, None].expand(-1, 7)], dim=1)
        eff = torch.where(mask, g[:, cols], torch.zeros_like(g[:, cols]))
        self.robot.set_joint_effort_target_index(target=eff.contiguous(), joint_ids=self.all_j)

    def reset(self, env_ids) -> None:
        if self.k0 is None:
            return
        ids = torch.as_tensor(env_ids, device=self.on.device, dtype=torch.long)
        for side in (0, 1):
            was = ids[self.on[side][ids]]
            if len(was):
                self._write(side, was, torch.zeros(len(was), dtype=torch.bool, device=was.device))
            self.on[side][ids] = False
