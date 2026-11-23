"""実機の pelvis の動きをシムで再生する口（パッチ 0113、2026-11-14）。

計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 D。純正の脚はシムで作れないので、脚は歩かせず、
実機の記録の pelvis の動き（3a に入ってからの扉の座標の前後・横・向きと roll・pitch の差。
G1_Hackason/Door/real/make_pelvis_replay_1114.py）をそのまま再生する。台本（script_teacher）が要る。
  hold_from の小段から: pelvis をその時の姿勢に止める（HOMIE の踏み直しを消す。実機の純正は 2a〜2d で 2 cm 内）
  start_sub の小段から: 止めた姿勢 ＋ 記録の差 に沿わせる（記録の終わりより後は最後の差のまま）
方法（method）:
  "vel"  方法 1: 物理 1 回（200 Hz）ごとに、機体の根元の速さを 記録の速さ ＋ kp ×（記録の姿勢 − 今の姿勢）に書く。
         瞬間移動が無いので、手とレバーの間の力が跳ねにくい（根元は無限に重い体のようにふるまう）
  "door" 方法 2: 機体の根元は止めた姿勢に保ち、扉（根元は固定・壁も同じ体）を記録と逆に動かす（根元の姿勢を物理 1 回ごとに書く）。
         扉の座標が動くので、台本の教師は pelvis を扉の根元からの姿勢で読む（script_teacher.py）。
         扉が加速するので扉の重さとクローザーの計算が狂う＝掛け金が外れた後は使えない（計画 6 節）
env.config.pelvis_replay（無い・enabled=False なら元のまま）:
  enabled・file（npz）・method（vel／door）・hold_from（既定 "2b"）・start_sub（既定 "3a"）・kp・kp_rot（既定 20 /s）・
  labels（使う回の番号のリスト。空なら全部）・offset_s（記録のこの秒から再生する。0113c・「歩いている最中」の動きを当てる時）・
  release_sub（この小段に入ったら再生を放して、脚（HOMIE）に任せる。0113d・後半 3c の入りのつなぎ目。既定 None＝放さない）・
  cmd_follow（0113e・⚠️ 推測: release_sub で放す代わりに、pelvis を「純正の脚の応え方」で動かす。台本の歩きの指令を遅れ delay_s・
  倍率 gain・効かない帯 dead（前後と横 [m/s]）で pelvis の速さにし、手の押し返し（台本の見積もりの −前後）が f_th を越えたら back_s 秒
  back_v で下がる。sway_label を渡すと、放した時にその回の記録の最初の sway_s 秒の横・向きの動きを足す（3a を脚を止めて回した時の
  3c の歩き出しの揺れ）。fwd_per_n: 押し返し 1 N あたり前の速さを落とす [m/s/N]・turn_per_n: 押し返し 1 N あたり把手の側へ回る [rad/s/N]
  （11-13 朝の 1 回目: 0.2 m/s の指令で押す力 6〜12 N の間 0.027 m/s しか進まず +36° 回された。15 回目: 3.8 秒で +28°。
  回りの指令を上限 −1.0 rad/s にしても毎秒 29° 回された）。HOMIE の脚だけだと 3c の歩き出しに 4〜9 cm 後ろへ下がり、0.2 m/s では
  ほとんど前へ出なかった（11-14 g_ga_off）。押し返しの効きを入れないと、扉は 3 秒で 30° 開いた（実機は 8°。g2_ga））。
  台 i は回 i % 回の数 を再生する
"""

from __future__ import annotations

import os

import numpy as np
import torch

# door_script.SUBS と同じ並び（script_teacher.sub_code の番号）
SUBS = ("2a", "2b", "2c", "2d", "3a", "3c", "done", "retreat", "failed")
LAST_ACTIVE_SUB = SUBS.index("3c")


def pelvis_replay_config(cfg) -> dict | None:
    """env.config.pelvis_replay を読んで確かめる。無い・enabled=False なら None"""
    if cfg is None or not bool(cfg.get("enabled", False)):
        return None
    f = str(cfg.get("file", ""))
    assert os.path.isfile(f), f"pelvis_replay.file が無い: {f}"
    method = str(cfg.get("method", "vel"))
    assert method in ("vel", "door"), f"pelvis_replay.method は vel か door: {method}"
    hold_from, start_sub = str(cfg.get("hold_from", "2b")), str(cfg.get("start_sub", "3a"))
    assert hold_from in SUBS and start_sub in SUBS and SUBS.index(hold_from) <= SUBS.index(start_sub), \
        f"pelvis_replay.hold_from・start_sub がおかしい: {hold_from} {start_sub}"
    labels = cfg.get("labels", None)
    release = cfg.get("release_sub", None)
    assert release is None or (str(release) in SUBS and SUBS.index(str(release)) >= SUBS.index(start_sub)), \
        f"pelvis_replay.release_sub は start_sub より後の小段: {release}"
    cf = cfg.get("cmd_follow", None) or {}
    cmd = None
    if cf.get("enabled", False):
        assert release is not None, "pelvis_replay.cmd_follow には release_sub が要る（そこから指令に沿わせる）"
        cmd = {"gain": float(cf.get("gain", 0.6)), "delay_s": float(cf.get("delay_s", 0.26)), "dead": float(cf.get("dead", 0.1)),
               "wz_gain": float(cf.get("wz_gain", 0.6)), "f_th": float(cf.get("f_th", 30.0)), "back_v": float(cf.get("back_v", -0.2)),
               "back_s": float(cf.get("back_s", 0.5)), "sway_label": cf.get("sway_label", None), "sway_s": float(cf.get("sway_s", 0.8)),
               "fwd_per_n": float(cf.get("fwd_per_n", 0.0)), "turn_per_n": float(cf.get("turn_per_n", 0.0)),
               "src": str(cf.get("src", "est")),
               # 0115: src panel の時の Mz の効き [rad/s/(N·m)]・一歩ごとの揺れ（正弦波・前後 osc_x・横 osc_y [m]・向き osc_yaw_deg・osc_hz）
               "turn_per_nm": float(cf.get("turn_per_nm", 0.0)), "osc_x": float(cf.get("osc_x", 0.0)),
               "osc_y": float(cf.get("osc_y", 0.0)), "osc_yaw_deg": float(cf.get("osc_yaw_deg", 0.0)),
               "osc_hz": float(cf.get("osc_hz", 1.0))}
        assert cmd["src"] in ("est", "contact", "panel"), f"cmd_follow.src は est・contact・panel のどれか: {cmd['src']}"
        assert cmd["osc_hz"] > 0 and min(cmd["osc_x"], cmd["osc_y"], cmd["osc_yaw_deg"]) >= 0, f"cmd_follow の揺れの値: {cf}"
        assert 0 < cmd["gain"] <= 1.5 and cmd["delay_s"] >= 0 and cmd["back_v"] <= 0 and cmd["f_th"] > 0, f"cmd_follow の値: {cf}"
    return {"cmd_follow": cmd, "release_sub": None if release is None else SUBS.index(str(release)),"file": f, "method": method, "hold_from": SUBS.index(hold_from), "start_sub": SUBS.index(start_sub),
            "kp": float(cfg.get("kp", 20.0)), "kp_rot": float(cfg.get("kp_rot", 20.0)),
            "offset_s": float(cfg.get("offset_s", 0.0)),
            "labels": [int(v) for v in labels] if labels else []}


def _tt(x) -> torch.Tensor:
    """Isaac Lab 3.0 の配列（ProxyArray・warp）を torch に（torch の配列はそのまま）"""
    if isinstance(x, torch.Tensor):
        return x
    if hasattr(x, "torch"):
        return x.torch
    import warp as wp  # noqa: PLC0415

    return wp.to_torch(x)


def euler_from_quat_xyzw(q: torch.Tensor):
    """(yaw, roll, pitch)。ZYX の並び"""
    x, y, z, w = q.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1.0, 1.0))
    return yaw, roll, pitch


def quat_xyzw_from_euler(yaw, roll, pitch) -> torch.Tensor:
    cy, sy = torch.cos(yaw / 2), torch.sin(yaw / 2)
    cr, sr = torch.cos(roll / 2), torch.sin(roll / 2)
    cp, sp = torch.cos(pitch / 2), torch.sin(pitch / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return torch.stack([x, y, z, w], dim=-1)


def rotvec_to_target(q: torch.Tensor, q_t: torch.Tensor) -> torch.Tensor:
    """今の向き q から目標 q_t への回り（世界の座標の回転ベクトル）。四元数は xyzw"""
    x, y, z, w = q.unbind(-1)
    xt, yt, zt, wt = q_t.unbind(-1)
    # q_err = q_t ⊗ conj(q)
    ew = wt * w + xt * x + yt * y + zt * z
    ex = -wt * x + xt * w - yt * z + zt * y
    ey = -wt * y + xt * z + yt * w - zt * x
    ez = -wt * z - xt * y + yt * x + zt * w
    sign = torch.where(ew < 0, -1.0, 1.0)
    ew, ex, ey, ez = ew * sign, ex * sign, ey * sign, ez * sign
    s = torch.sqrt(torch.clamp(ex * ex + ey * ey + ez * ez, min=1e-12))
    ang = 2 * torch.atan2(s, ew)
    return torch.stack([ex, ey, ez], dim=-1) * (ang / s)[..., None]


class PelvisReplay:
    """control_step(env) を台本の教師の apply の直後に、physics_substep(env) を物理 1 回ごと（力を掛ける所）に、
    reset(env, env_ids) を回の戻しで呼ぶ"""

    def __init__(self, env, cfg: dict) -> None:
        z = np.load(cfg["file"])
        labels, d = np.asarray(z["labels"]), np.asarray(z["d"], dtype=np.float32)
        if cfg["labels"]:
            keep = np.isin(labels, cfg["labels"])
            assert keep.any(), f"pelvis_replay.labels {cfg['labels']} が file に無い（{labels.tolist()}）"
            labels, d = labels[keep], d[keep]
        self.cfg, self.method = cfg, cfg["method"]
        self.dev = env.device
        self.labels = labels
        self.R, self.M = d.shape[0], d.shape[1]
        self.dt_traj = float(z["t"][1] - z["t"][0])
        self.d = torch.as_tensor(d, device=self.dev)
        v = np.gradient(d, self.dt_traj, axis=1)
        self.v = torch.as_tensor(v.astype(np.float32), device=self.dev)
        n = env.num_envs
        self.run = torch.arange(n, device=self.dev) % self.R
        self.phase = torch.zeros(n, dtype=torch.long, device=self.dev)  # 0 切・1 止める・2 再生・3 放した（0113d・回の戻しまで）
        self.t0 = torch.zeros(n, device=self.dev)
        self.p0 = torch.zeros(n, 6, device=self.dev)  # x, y, z（台の原点から）・yaw・roll・pitch
        self.err_max = torch.zeros(n, device=self.dev)  # 再生中の pelvis の水平のずれの最大 [m]
        self.t_ctrl = torch.zeros(n, device=self.dev)
        self.sub_k = 0
        self.phys_dt = float(env.dt) / float(env.config.simulator.config.sim.control_decimation)
        # 0113e: 指令に沿わせる（phase 4）の持ち物
        self.cmd = cfg.get("cmd_follow")
        self.tgt = torch.zeros(n, 3, device=self.dev)      # 目標の x, y, yaw（台の原点から）
        self.vtgt = torch.zeros(n, 3, device=self.dev)     # 目標の速さ（世界の x, y, wz）
        self.back_until = torch.full((n,), -1.0, device=self.dev)
        self.t_rel0 = torch.zeros(n, device=self.dev)      # 指令に沿わせ始めた時刻
        self.sway0 = torch.zeros(n, 3, device=self.dev)    # その時の pelvis（揺れの記録の基準）
        self.n_back = torch.zeros(n, dtype=torch.long, device=self.dev)
        if self.cmd is not None:
            self.delay_n = max(0, int(round(self.cmd["delay_s"] / float(env.dt))))
            self.cmd_buf = torch.zeros(self.delay_n + 1, n, 3, device=self.dev)
            self.sway_k = None
            if self.cmd["sway_label"] is not None:
                k = np.where(np.asarray(z["labels"]) == int(self.cmd["sway_label"]))[0]
                assert len(k), f"cmd_follow.sway_label {self.cmd['sway_label']} が file に無い"
                self.sway_d = torch.as_tensor(np.asarray(z["d"], dtype=np.float32)[int(k[0])], device=self.dev)  # (M, 5)
                self.sway_k = int(round(self.cmd["sway_s"] / self.dt_traj))
        self.robot = env.simulator._robot
        self.door = env.simulator.scene.articulations["door"]
        self.door_pose0 = _tt(self.door.data.root_link_pose_w).clone()  # 扉の根元（世界・xyzw）
        print(f"[pelvis_replay] 方法 {self.method}・回 {labels.tolist()}（台 i は回 i % {self.R}）・{self.M} 点・"
              f"止めるのは {SUBS[cfg['hold_from']]} から・再生は {SUBS[cfg['start_sub']]} から・kp {cfg['kp']}", flush=True)

    # --- 毎歩（台本の教師の apply の直後） ---
    def control_step(self, env) -> None:
        sub = env._script_teacher.sub_code
        t = env.episode_length_buf.to(torch.float32) * float(env.dt)
        alive = (sub >= 0) & (sub <= LAST_ACTIVE_SUB)
        new_hold = (self.phase == 0) & alive & (sub >= self.cfg["hold_from"])
        if new_hold.any():
            pos, yaw, roll, pitch = self._robot_pose(env)
            self.p0[new_hold] = torch.stack([pos[:, 0], pos[:, 1], pos[:, 2], yaw, roll, pitch], dim=-1)[new_hold]
            self.phase[new_hold] = 1
        start = (self.phase == 1) & alive & (sub >= self.cfg["start_sub"])
        self.t0[start] = t[start]
        self.phase[start] = 2
        if self.cfg.get("release_sub") is not None:  # 0113d: 放す小段（4a は番号が最後なので、3c 以降はどれでも放す）
            rel = (self.phase == 2) & (sub >= self.cfg["release_sub"]) & (sub != SUBS.index("retreat")) & (sub != SUBS.index("failed"))
            if self.cmd is None:
                self.phase[rel] = 3
            elif bool(rel.any()):  # 0113e: 放す代わりに指令に沿わせる（今の pelvis から）
                pos, yaw, _, _ = self._robot_pose(env)
                cur = torch.stack([pos[:, 0], pos[:, 1], yaw], dim=-1)
                self.tgt[rel] = cur[rel]
                self.sway0[rel] = cur[rel]
                self.t_rel0[rel] = t[rel]
                self.phase[rel] = 4
        if self.cmd is not None:
            self._cmd_step(env, t)
        self.t_ctrl = t
        self.sub_k = 0

    def _cmd_step(self, env, t: torch.Tensor) -> None:
        """0113e: 台本の歩きの指令 → 純正の応え方（遅れ・効かない帯・倍率）→ 目標の速さ。押し返しが線を越えたら下がる"""
        c, dt = self.cmd, float(env.dt)
        sc = float(env.config.homie_command_scale)
        D = env._delta_actions[:, 0:3]
        cmd = torch.stack([D[:, 0] * sc * 2.0, D[:, 1] * sc * 2.0, D[:, 2] * sc * 0.5], dim=-1)  # script_teacher の逆 [m/s, m/s, rad/s]
        self.cmd_buf = torch.roll(self.cmd_buf, 1, dims=0)
        self.cmd_buf[0] = cmd
        late = self.cmd_buf[-1]
        v = torch.where(late[:, :2].abs() >= c["dead"], late[:, :2] * c["gain"], torch.zeros_like(late[:, :2]))
        wz = late[:, 2] * c["wz_gain"]
        # 0113g: 押し返しの力は、台本の見積もり（est）か、レバーと手の接触センサ（contact）。11-14: シムの 3c では見積もりが
        # 本当の力より約 10 N 押し返しの向きへ片寄り（3a の上下も 6 N）、押していないのに体を回した（g3_ga）
        # 0115: src panel は、板とレバーから体が受ける本当の力（体の前後の押し返し）と、pelvis の回りの回す力 Mz（panel_push.py）
        if c["src"] == "panel":
            f_back = torch.nan_to_num(env._script_teacher.body_push_t, nan=0.0)
            mz = torch.nan_to_num(env._script_teacher.body_mz_t, nan=0.0)
        else:
            f_src = env._script_teacher.hand_f_t if c["src"] == "est" else env._script_teacher.hand_c_t
            f_back = torch.nan_to_num(-f_src[:, 0], nan=0.0)
            mz = torch.zeros_like(f_back)
        push = torch.clamp(f_back, min=0.0)
        lr = torch.where(env.door_open_lr > 0, 1.0, -1.0).to(push.dtype)  # 把手が左なら左（+yaw）へ回される
        v[:, 0] = v[:, 0] - c["fwd_per_n"] * push  # 押すほど前へ出ない（11-13 朝の 1 回目）
        wz = wz + c["turn_per_n"] * push * lr     # 押し返しで体が把手の側へ回される
        wz = wz + c["turn_per_nm"] * mz            # 0115: 回す力 Mz の向きへ回される（左回りが正）
        on = self.phase == 4
        start = on & (f_back > c["f_th"]) & (t >= self.back_until)
        self.back_until = torch.where(start, t + c["back_s"], self.back_until)
        self.n_back += start.long()
        back = on & (t < self.back_until)
        v[:, 0] = torch.where(back, torch.full_like(v[:, 0], c["back_v"]), v[:, 0])
        v[:, 1] = torch.where(back, torch.zeros_like(v[:, 1]), v[:, 1])
        yaw = self.tgt[:, 2]
        cy, sy = torch.cos(yaw), torch.sin(yaw)
        vw = torch.stack([cy * v[:, 0] - sy * v[:, 1], sy * v[:, 0] + cy * v[:, 1], wz], dim=-1)
        self.vtgt = torch.where(on[:, None], vw, torch.zeros_like(vw))
        self.tgt = self.tgt + self.vtgt * dt

    def _sway(self, t_now: torch.Tensor):
        """0113e: 指令に沿わせ始めてからの揺れの記録（横・向き。前後は指令の側に任せる）。記録が無ければ 0"""
        if self.cmd is None:
            return torch.zeros(len(t_now), 3, device=self.dev), torch.zeros(len(t_now), 3, device=self.dev)
        osc, oscv = self._osc(t_now)
        if self.sway_k is None:
            return osc, oscv
        rel = torch.clamp(t_now - self.t_rel0, min=0.0) / self.dt_traj
        i0 = torch.clamp(rel.floor().long(), 0, self.sway_k)
        i1 = torch.clamp(i0 + 1, max=self.sway_k)
        w = (rel - rel.floor()).clamp(0.0, 1.0)[:, None]
        d = (1 - w) * self.sway_d[i0] + w * self.sway_d[i1]
        out = torch.stack([torch.zeros_like(d[:, 0]), d[:, 1], d[:, 2]], dim=-1)
        vel = torch.stack([torch.zeros_like(d[:, 0]), (self.sway_d[i1, 1] - self.sway_d[i0, 1]) / self.dt_traj,
                           (self.sway_d[i1, 2] - self.sway_d[i0, 2]) / self.dt_traj], dim=-1)
        vel = torch.where((i0 >= self.sway_k)[:, None], torch.zeros_like(vel), vel)
        return out + osc, vel + oscv

    def _osc(self, t_now: torch.Tensor):
        """0115: 指令に沿わせ始めてからの一歩ごとの揺れ（正弦波）。前後は sin・横と向きは cos（実機の 3c は約 1 Hz・前後 ±1〜1.7 cm・
        横 ±1.5〜2.2 cm・向き ±1.5〜2.8°）。揺れの位置と速さ (n,3)"""
        c = self.cmd
        if c["osc_x"] <= 0.0 and c["osc_y"] <= 0.0 and c["osc_yaw_deg"] <= 0.0:
            z = torch.zeros(len(t_now), 3, device=self.dev)
            return z, z
        w = 2.0 * np.pi * c["osc_hz"]
        tau = torch.clamp(t_now - self.t_rel0, min=0.0)
        amp = torch.tensor([c["osc_x"], c["osc_y"], np.radians(c["osc_yaw_deg"])], device=self.dev, dtype=torch.float32)
        s, co = torch.sin(w * tau), torch.cos(w * tau)
        pos = torch.stack([amp[0] * s, amp[1] * (co - 1.0), amp[2] * (co - 1.0)], dim=-1)  # 始めは 0 から
        vel = torch.stack([amp[0] * w * co, -amp[1] * w * s, -amp[2] * w * s], dim=-1)
        on = (self.phase == 4)[:, None]
        return torch.where(on, pos, torch.zeros_like(pos)), torch.where(on, vel, torch.zeros_like(vel))

    def _robot_pose(self, env):
        pose = _tt(self.robot.data.root_link_pose_w)
        pos = pose[:, :3] - env.env_origins
        yaw, roll, pitch = euler_from_quat_xyzw(pose[:, 3:7])
        return pos, yaw, roll, pitch

    def _delta(self, t_now: torch.Tensor):
        """(差 (n,5), 速さ (n,5))。再生中でない台は 0"""
        rel = (torch.clamp(t_now - self.t0, min=0.0) + self.cfg["offset_s"]) / self.dt_traj  # 0113c: 記録の途中から
        i0 = torch.clamp(rel.floor().long(), 0, self.M - 1)
        i1 = torch.clamp(i0 + 1, max=self.M - 1)
        w = (rel - rel.floor()).clamp(0.0, 1.0)[:, None]
        d = (1 - w) * self.d[self.run, i0] + w * self.d[self.run, i1]
        if self.cfg["offset_s"] > 0.0:  # 0113c: 途中から再生する時は、その点からの差にする
            k0 = min(int(round(self.cfg["offset_s"] / self.dt_traj)), self.M - 1)
            d = d - self.d[self.run, k0]
        v = (1 - w) * self.v[self.run, i0] + w * self.v[self.run, i1]
        v = torch.where((i0 >= self.M - 1)[:, None], torch.zeros_like(v), v)
        on = (self.phase == 2)[:, None]
        return torch.where(on, d, torch.zeros_like(d)), torch.where(on, v, torch.zeros_like(v))

    # --- 物理 1 回ごと ---
    def physics_substep(self, env) -> None:
        active = (self.phase == 1) | (self.phase == 2) | (self.phase == 4)
        if not bool(active.any()):
            return
        t_end = self.t_ctrl + (self.sub_k + 1) * self.phys_dt  # この物理 1 回の終わりの時刻
        self.sub_k += 1
        d, v = self._delta(t_end)
        robot_d = d if self.method == "vel" else torch.zeros_like(d)
        robot_v = v if self.method == "vel" else torch.zeros_like(v)
        pos, yaw, roll, pitch = self._robot_pose(env)
        tx, ty = self.p0[:, 0] + robot_d[:, 0], self.p0[:, 1] + robot_d[:, 1]
        if self.cmd is not None:  # 0113e: 指令に沿わせる台は、目標（指令の積み）＋揺れの記録へ
            c4 = self.phase == 4
            sw, swv = self._sway(t_end)
            frac = float(self.sub_k) / float(env.config.simulator.config.sim.control_decimation)
            tx = torch.where(c4, self.tgt[:, 0] + self.vtgt[:, 0] * self.phys_dt * frac + sw[:, 0], tx)
            ty = torch.where(c4, self.tgt[:, 1] + self.vtgt[:, 1] * self.phys_dt * frac + sw[:, 1], ty)
            robot_v = torch.where(c4[:, None], torch.stack([self.vtgt[:, 0] + swv[:, 0], self.vtgt[:, 1] + swv[:, 1], self.vtgt[:, 2] + swv[:, 2],
                                                             torch.zeros_like(swv[:, 0]), torch.zeros_like(swv[:, 0])], dim=-1), robot_v)
            yaw_t = self.tgt[:, 2] + sw[:, 2] - self.p0[:, 3]  # 下の向きの目標は p0 ＋ robot_d[:, 2]
            robot_d = torch.where(c4[:, None], torch.stack([robot_d[:, 0], robot_d[:, 1], yaw_t, torch.zeros_like(yaw_t),
                                                             torch.zeros_like(yaw_t)], dim=-1), robot_d)
        kp, kr = self.cfg["kp"], self.cfg["kp_rot"]
        lin = torch.stack([robot_v[:, 0] + kp * (tx - pos[:, 0]), robot_v[:, 1] + kp * (ty - pos[:, 1]),
                           kp * (self.p0[:, 2] - pos[:, 2])], dim=-1)
        q = _tt(self.robot.data.root_link_pose_w)[:, 3:7]
        q_t = quat_xyzw_from_euler(self.p0[:, 3] + robot_d[:, 2], self.p0[:, 4] + robot_d[:, 3], self.p0[:, 5] + robot_d[:, 4])
        ang = kr * rotvec_to_target(q, q_t)
        ang[:, 2] = ang[:, 2] + robot_v[:, 2]
        ids = torch.nonzero(active, as_tuple=False).flatten()
        vel = torch.cat([lin, ang], dim=-1)[ids].to(torch.float32).contiguous()
        self.robot.write_root_link_velocity_to_sim_index(root_velocity=vel, env_ids=ids)
        err = torch.hypot(tx - pos[:, 0], ty - pos[:, 1])
        self.err_max = torch.where(self.phase == 2, torch.maximum(self.err_max, err), self.err_max)
        if self.method == "door":
            self._move_door(env, d, ids)

    def _move_door(self, env, d: torch.Tensor, ids: torch.Tensor) -> None:
        """扉の根元を、止めた pelvis から見て pelvis が 止めた姿勢 ＋ 差 に見える所へ置く（平面の剛体変換）。
        扉の根元 T ＝ p0 ⊗ (p0 ⊕ 差)⁻¹（扉の初めの姿勢は台の原点・向き 0）"""
        x0, y0, th0 = self.p0[:, 0], self.p0[:, 1], self.p0[:, 3]
        xq, yq, thq = x0 + d[:, 0], y0 + d[:, 1], th0 + d[:, 2]
        c, s = torch.cos(thq), torch.sin(thq)
        ix, iy = -(c * xq + s * yq), -(-s * xq + c * yq)  # q⁻¹ の位置
        c0, s0 = torch.cos(th0), torch.sin(th0)
        px, py = x0 + c0 * ix - s0 * iy, y0 + s0 * ix + c0 * iy
        th = th0 - thq
        pose = self.door_pose0.clone()
        pose[:, 0] = env.env_origins[:, 0] + px
        pose[:, 1] = env.env_origins[:, 1] + py
        zero = torch.zeros_like(th)
        pose[:, 3:7] = quat_xyzw_from_euler(th, zero, zero)
        self.door.write_root_link_pose_to_sim_index(root_pose=pose[ids].to(torch.float32).contiguous(), env_ids=ids)

    def door_root_planar(self, env, n: int) -> np.ndarray:
        """台本の教師が pelvis を扉の座標で読むための、扉の根元の (x, y, yaw)（台の原点から）。方法 door の時だけ使う"""
        pose = _tt(self.door.data.root_link_pose_w)[:n]
        yaw, _, _ = euler_from_quat_xyzw(pose[:, 3:7])
        xy = pose[:, :2] - env.env_origins[:n, :2]
        return torch.cat([xy, yaw[:, None]], dim=-1).detach().cpu().numpy().astype(float)

    def reset(self, env, env_ids) -> None:
        ids = torch.as_tensor(env_ids, device=self.dev, dtype=torch.long)
        if self.method == "door" and len(ids):
            moved = ids[(self.phase[ids] == 1) | (self.phase[ids] == 2)]
            if len(moved):
                self.door.write_root_link_pose_to_sim_index(root_pose=self.door_pose0[moved].contiguous(), env_ids=moved)
        self.phase[ids] = 0
        self.t0[ids] = 0.0
        self.err_max[ids] = 0.0
        self.back_until[ids] = -1.0
        self.n_back[ids] = 0
        if self.cmd is not None:
            self.cmd_buf[:, ids] = 0.0

    def summary(self, i: int) -> dict:
        """回ごとの結果に足す値（台本の教師の JSON Lines）"""
        return {"replay_label": int(self.labels[int(self.run[i])]), "replay_method": self.method,
                "replay_err_max_cm": round(100 * float(self.err_max[i]), 2), "replay_phase": int(self.phase[i]),
                "cmd_back_n": int(self.n_back[i])}
