"""押されて下がる脚と歩き出しの横揺れ（パッチ 0114、2026-11-14・⚠️ 推測で作った形）。

計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の 13.1。純正の脚はシムで作れないので、純正に似せた脚
（0085・0095 の応え方の層＋HOMIE）に、実機で見えた 2 つの癖を指令の層で足す:
  押されて下がる: 歩いている台で、手が扉から押し返される力（台本の手の力の見積もり＝実機と同じ見積もりの −前後）が f_th を越えたら、
                 t_back 秒、前後の指令を v_back（後ろ向き）に置き換える（応え方の層を通る）。11-13 朝: 25〜40 N の山の直後に
                 pelvis が 9〜14 cm 下がった
  歩き出しの横揺れ: 歩きの指令が 0 から立ち上がった時、sway_s 秒、脚に届く指令（層の後ろ）に横 sway_vy と回り sway_wz を足す。
                 11-13 朝・夕: 足踏みの歩き出しの 0.5〜0.8 秒に pelvis が蝶番の側へ 12〜15 cm・向き +1〜+4°
env.config.loco_push（無い・enabled=False なら元のまま）: enabled・f_th [N]・v_back [m/s]・t_back [s]・refractory [s]・
  sway_vy [m/s・体の座標・把手が左の扉で右が負]・sway_wz [rad/s]・sway_s [s]・walk_on [m/s]・src（"est"＝台本の見積もり／
  "contact"＝接触センサ）。台本の教師（tau=True）と、応え方の層（env.config.loco_resp.enabled=True）が要る
"""

from __future__ import annotations

import torch

ARM_ACCUM_START = 3  # 積み上げの並び [前後, 横, 旋回, 腕 14]（script_teacher.py と同じ）


def loco_push_config(cfg) -> dict | None:
    if cfg is None or not bool(cfg.get("enabled", False)):
        return None
    out = {"f_th": float(cfg.get("f_th", 30.0)), "v_back": float(cfg.get("v_back", -0.3)),
           "t_back": float(cfg.get("t_back", 0.6)), "refractory": float(cfg.get("refractory", 0.6)),
           "sway_vy": float(cfg.get("sway_vy", -0.25)), "sway_wz": float(cfg.get("sway_wz", 0.1)),
           "sway_s": float(cfg.get("sway_s", 0.5)), "walk_on": float(cfg.get("walk_on", 0.08)),
           "src": str(cfg.get("src", "est"))}
    assert out["f_th"] > 0 and out["v_back"] <= 0 and out["t_back"] >= 0 and out["sway_s"] >= 0, f"loco_push の値がおかしい: {cfg}"
    assert out["src"] in ("est", "contact"), f"loco_push.src は est か contact: {out['src']}"
    return out


class LocoPush:
    """control_step(env) を台本の教師の apply の直後に、apply_legs(env) を観測を作る直前に、reset(env_ids) を回の戻しで呼ぶ"""

    def __init__(self, env, cfg: dict) -> None:
        assert getattr(env, "_stock_legs_on", False), "loco_push には応え方の層（env.config.loco_resp.enabled=True）が要る"
        self.cfg, self.dev, n = cfg, env.device, env.num_envs
        self.back_until = torch.full((n,), -1.0, device=self.dev)
        self.back_ready = torch.zeros(n, device=self.dev)
        self.sway_until = torch.full((n,), -1.0, device=self.dev)
        self.walking_prev = torch.zeros(n, dtype=torch.bool, device=self.dev)
        self.n_back = torch.zeros(n, dtype=torch.long, device=self.dev)
        self.n_sway = torch.zeros(n, dtype=torch.long, device=self.dev)
        self.f_peak = torch.zeros(n, device=self.dev)
        self.t = torch.zeros(n, device=self.dev)
        print(f"[loco_push] 押し返し {cfg['f_th']} N を越えたら前後 {cfg['v_back']} m/s を {cfg['t_back']} 秒・"
              f"歩き出しに横 {cfg['sway_vy']} m/s・回り {cfg['sway_wz']} rad/s を {cfg['sway_s']} 秒（力は {cfg['src']}）", flush=True)

    def control_step(self, env) -> None:
        c = self.cfg
        st = env._script_teacher
        f = st.hand_f_t if c["src"] == "est" else st.hand_c_t
        f_back = torch.nan_to_num(-f[:, 0], nan=0.0)
        self.t = env.episode_length_buf.to(torch.float32) * float(env.dt)
        sc = float(env.config.homie_command_scale)
        vx_cmd = env._delta_actions[:, 0] * sc * 2.0  # 台本が書いた前後の指令 [m/s]（script_teacher の逆）
        walking = vx_cmd > c["walk_on"]
        self.f_peak = torch.where(walking, torch.maximum(self.f_peak, f_back), self.f_peak)
        start = walking & (f_back > c["f_th"]) & (self.t >= self.back_ready)
        self.back_until = torch.where(start, self.t + c["t_back"], self.back_until)
        self.back_ready = torch.where(start, self.t + c["t_back"] + c["refractory"], self.back_ready)
        self.n_back += start.long()
        onset = walking & ~self.walking_prev
        self.sway_until = torch.where(onset, self.t + c["sway_s"], self.sway_until)
        self.n_sway += onset.long()
        self.walking_prev = walking
        back = self.t < self.back_until
        if bool(back.any()):  # 前後の指令を後ろ向きに置き換える（応え方の層を通る）
            env._delta_actions[back, 0] = c["v_back"] / 2.0 / sc

    def apply_legs(self, env) -> None:
        sway = self.t < self.sway_until
        if not bool(sway.any()):
            return
        legs = env._homie_commands_legs.clone()
        legs[sway, 1] += self.cfg["sway_vy"] / 2.0  # 指令の単位（get_physical_homie_commands の逆）
        legs[sway, 2] += self.cfg["sway_wz"] / 0.5
        env._homie_commands_legs = legs

    def reset(self, env_ids) -> None:
        ids = torch.as_tensor(env_ids, device=self.dev, dtype=torch.long)
        self.back_until[ids] = -1.0
        self.back_ready[ids] = 0.0
        self.sway_until[ids] = -1.0
        self.walking_prev[ids] = False
        self.n_back[ids] = 0
        self.n_sway[ids] = 0
        self.f_peak[ids] = 0.0

    def summary(self, i: int) -> dict:
        return {"push_back_n": int(self.n_back[i]), "sway_n": int(self.n_sway[i]), "push_f_peak": round(float(self.f_peak[i]), 1)}
