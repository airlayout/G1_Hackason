"""パッチ 0104（11-11・段 2 の途中で胴を鉛直軸のまわりに回す外乱 env.config.yaw_disturb）の試験。
Isaac Sim は起こさない（yaw_disturb.py は torch だけ。扉の環境は関数を取り出して偽の環境で動かす）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 設定が無い・enabled=False なら元のまま（None）。範囲・向き・割合の誤りは止める
- 回りの目標は t0 の前 0・t0+dur の後 amp・真ん中で amp/2（余弦でつなぐ）
- root を回すと、位置と高さ・傾き（roll・pitch）はそのままで向きだけ Δ 変わり、速さ・角速さも Δ 回る
- 向き: lever_side は把手の側（把手が左 open_lr=+1 で +）、away は逆、random は ±。prob=0 は回さない
- 扉の環境: 毎歩、この歩の終わりの目標との差だけ回して書く（xyzw で読み wxyz で書く罠を確かめる）。足した回りは目標に一致する
"""
from __future__ import annotations

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from gr00t.rl.envs.door.yaw_disturb import (
    DEFAULTS,
    ramp_offset,
    rotate_root_state_wxyz,
    sample_disturb,
    yaw_disturb_config,
)

ENV = Path("gr00t/rl/envs/door/door_open_homie.py")


def _method(name: str):
    tree = ast.parse(ENV.read_text(encoding="utf-8"))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    fn.decorator_list = []
    return fn


def _load(names: list[str], extra: dict):
    ns = {"torch": torch, **extra}
    exec(compile(ast.Module(body=[_method(n) for n in names], type_ignores=[]), str(ENV), "exec"), ns)  # noqa: S102
    return ns


def _euler_from_wxyz(q: torch.Tensor):
    w, x, y, z = q.unbind(-1)
    roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1.0, 1.0))
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def _wxyz_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def test_config_off_by_default_and_checked():
    assert yaw_disturb_config(None) is None
    assert yaw_disturb_config({"enabled": False, "amp_deg": [10, 20]}) is None
    cfg = yaw_disturb_config({"enabled": True})
    assert cfg["amp_deg"] == DEFAULTS["amp_deg"] and cfg["start_s"] == DEFAULTS["start_s"]
    assert cfg["dur_s"] == DEFAULTS["dur_s"] and cfg["sign"] == "lever_side" and cfg["prob"] == 1.0
    one = yaw_disturb_config({"enabled": True, "amp_deg": 30})  # 1 つの数は [v, v]
    assert one["amp_deg"] == (30.0, 30.0)
    with pytest.raises(AssertionError):
        yaw_disturb_config({"enabled": True, "amp_deg": [20, 10]})
    with pytest.raises(AssertionError):
        yaw_disturb_config({"enabled": True, "dur_s": [0.0, 0.5]})
    with pytest.raises(AssertionError):
        yaw_disturb_config({"enabled": True, "sign": "left"})
    with pytest.raises(AssertionError):
        yaw_disturb_config({"enabled": True, "prob": 1.5})


def test_ramp_offset_cosine():
    t = torch.tensor([0.0, 1.4, 1.65, 1.9, 3.0])
    out = ramp_offset(t, torch.full((5,), 1.4), torch.full((5,), 0.5), torch.full((5,), 0.4))
    assert torch.allclose(out, torch.tensor([0.0, 0.0, 0.2, 0.4, 0.4]), atol=1e-6)


def test_rotate_keeps_position_tilt_and_rotates_velocity():
    q = torch.tensor(_wxyz_from_euler(0.05, -0.08, -0.8))
    state = torch.cat([torch.tensor([1.0, 2.0, 0.73]), q, torch.tensor([0.3, 0.0, 0.1]), torch.tensor([0.0, 0.2, 0.5])])[None]
    delta = torch.tensor([math.radians(20.0)])
    out = rotate_root_state_wxyz(state, delta)
    assert out is not state and torch.equal(state[0, 3:7], q)  # 元は書き換えない
    assert torch.allclose(out[0, 0:3], state[0, 0:3])
    r0, p0, y0 = _euler_from_wxyz(state[:, 3:7])
    r1, p1, y1 = _euler_from_wxyz(out[:, 3:7])
    assert torch.allclose(y1 - y0, delta, atol=1e-5)
    # 世界の鉛直軸のまわりに回すので、重力に対する傾き（体の z 軸の高さ成分）は変わらない
    up0 = 1 - 2 * (state[0, 4] ** 2 + state[0, 5] ** 2)
    up1 = 1 - 2 * (out[0, 4] ** 2 + out[0, 5] ** 2)
    assert torch.allclose(up0, up1, atol=1e-6)
    c, s = math.cos(delta.item()), math.sin(delta.item())
    assert torch.allclose(out[0, 7:10], torch.tensor([0.3 * c, 0.3 * s, 0.1]), atol=1e-6)
    assert torch.allclose(out[0, 10:13], torch.tensor([-0.2 * s, 0.2 * c, 0.5]), atol=1e-6)
    assert torch.allclose(torch.linalg.norm(out[0, 3:7]), torch.tensor(1.0), atol=1e-6)


def test_sample_sign_and_prob():
    cfg = yaw_disturb_config({"enabled": True, "amp_deg": [10, 30], "start_s": [1.4, 1.6], "dur_s": [0.4, 0.6]})
    open_lr = torch.tensor([1.0, -1.0, 1.0, -1.0])
    u = torch.full((4, 5), 0.5)
    amp, t0, dur = sample_disturb(cfg, open_lr, u)
    assert torch.allclose(amp, torch.deg2rad(torch.tensor([20.0, -20.0, 20.0, -20.0])))  # 把手の側
    assert torch.allclose(t0, torch.full((4,), 1.5)) and torch.allclose(dur, torch.full((4,), 0.5))
    away = yaw_disturb_config({"enabled": True, "amp_deg": 20, "sign": "away"})
    assert torch.allclose(sample_disturb(away, open_lr, u)[0], torch.deg2rad(torch.tensor([-20.0, 20.0, -20.0, 20.0])))
    rnd = yaw_disturb_config({"enabled": True, "amp_deg": 20, "sign": "random"})
    u_sign = u.clone()
    u_sign[:, 3] = torch.tensor([0.1, 0.9, 0.1, 0.9])
    assert torch.allclose(sample_disturb(rnd, open_lr, u_sign)[0], torch.deg2rad(torch.tensor([20.0, -20.0, 20.0, -20.0])))
    half = yaw_disturb_config({"enabled": True, "amp_deg": 20, "prob": 0.5})
    u_p = u.clone()
    u_p[:, 4] = torch.tensor([0.2, 0.2, 0.7, 0.7])
    assert torch.allclose(sample_disturb(half, open_lr, u_p)[0], torch.deg2rad(torch.tensor([20.0, -20.0, 0.0, 0.0])))


class _FakeSim:
    def __init__(self, xyzw_states):
        self._xyzw = xyzw_states
        self.writes = []

    @property
    def robot_root_states(self):
        return self._xyzw.clone()  # 本物と同じく [pos, xyzw, lin_vel, ang_vel] の新しいテンソル

    def set_actor_root_state_tensor(self, ids, wxyz_states):
        self.writes.append((ids.clone(), wxyz_states.clone()))
        new = wxyz_states.clone()
        new[:, 3:7] = wxyz_states[:, [4, 5, 6, 3]]  # 本物と同じく wxyz → xyzw に直して持つ
        self._xyzw[ids] = new[ids]


def _env(n=3):
    ns = _load(["_apply_yaw_disturb"], {"ramp_offset": ramp_offset, "rotate_root_state_wxyz": rotate_root_state_wxyz})
    q_wxyz = torch.tensor(_wxyz_from_euler(0.02, 0.03, -0.8))
    xyzw = torch.zeros(n, 13)
    xyzw[:, 0:3] = torch.tensor([0.5, 0.2, 0.73])
    xyzw[:, 3:7] = q_wxyz[[1, 2, 3, 0]]
    env = SimpleNamespace(
        dt=0.02,
        simulator=_FakeSim(xyzw),
        episode_length_buf=torch.zeros(n, dtype=torch.long),
        _yd_amp=torch.deg2rad(torch.tensor([20.0, -10.0, 0.0])),
        _yd_t0=torch.full((n,), 1.0),
        _yd_dur=torch.full((n,), 0.5),
        _yd_applied=torch.zeros(n),
        log_dict={},
    )
    env._apply_yaw_disturb = ns["_apply_yaw_disturb"].__get__(env)
    return env, q_wxyz


def test_env_applies_difference_each_step_and_converts_quaternion_order():
    env, q0 = _env()
    _, _, yaw0 = _euler_from_wxyz(q0[None])
    for k in range(100):  # 2 s
        env.episode_length_buf = torch.full((3,), k, dtype=torch.long)
        env._apply_yaw_disturb()
    assert len(env.simulator.writes) > 0
    first_ids, _ = env.simulator.writes[0]
    assert 2 not in first_ids.tolist()  # 回りの無い台は書かない
    assert torch.allclose(env._yd_applied, env._yd_amp, atol=1e-6)
    xyzw = env.simulator.robot_root_states
    _, _, yaw = _euler_from_wxyz(xyzw[:, [6, 3, 4, 5]])
    assert torch.allclose(yaw - yaw0, env._yd_amp, atol=1e-4)  # 並びを取り違えると向き以外も壊れて一致しない
    assert torch.allclose(xyzw[:, 0:3], torch.tensor([0.5, 0.2, 0.73]).expand(3, 3))
    assert abs(float(env.log_dict["yaw_disturb_deg"]) - 10.0) < 1e-3  # |20|・|−10|・0 の平均


def test_env_before_start_writes_nothing():
    env, _ = _env()
    for k in range(40):  # 0.8 s < t0
        env.episode_length_buf = torch.full((3,), k, dtype=torch.long)
        env._apply_yaw_disturb()
    assert env.simulator.writes == [] and torch.equal(env._yd_applied, torch.zeros(3))


def test_env_source_wires_config_reset_and_step():
    src = ENV.read_text(encoding="utf-8")
    assert 'yaw_disturb_config(self.config.get("yaw_disturb", None))' in src
    reset = ast.unparse(_method("_reset_buffers_callback"))
    assert "_yd_resample" in reset and "_yd_applied" in reset
    step = ast.unparse(_method("step"))
    assert step.index("_apply_yaw_disturb") < step.index("super().step")


# ---- K6a: 壁と枠の出（door.py の wall_front_out・cover_front_out） ----
DOOR = Path("gr00t/rl/isaac_utils/playground/env_rand/door.py")


def _door_fn():
    tree = ast.parse(DOOR.read_text(encoding="utf-8"))
    body = [n for n in tree.body if (isinstance(n, ast.Assign) and any(getattr(t, "id", "") == "PANEL_HALF_THICKNESS" for t in n.targets))
            or (isinstance(n, ast.FunctionDef) and n.name == "front_out_box")]
    ns = {"Optional": __import__("typing").Optional}
    exec(compile(ast.Module(body=body, type_ignores=[]), str(DOOR), "exec"), ns)  # noqa: S102
    return ns["front_out_box"]


def test_front_out_box_moves_only_the_front_face():
    box = _door_fn()
    assert box(-0.02, 0.05, None) == (-0.02, 0.05)  # 元のまま（前の面 -0.07＝出 5 cm）
    cx, hx = box(-0.02, 0.05, 0.01)  # 壁 1 cm
    assert math.isclose(cx - hx, -0.03) and math.isclose(cx + hx, 0.03)  # 前の面 -0.03・後ろの面はそのまま
    cx, hx = box(-0.02, 0.06, 0.03)  # 枠 3 cm
    assert math.isclose(cx - hx, -0.05) and math.isclose(cx + hx, 0.04)
    with pytest.raises(AssertionError):
        box(-0.02, 0.05, -0.06)  # 前の面が後ろの面より奥


def test_door_source_uses_front_out_for_all_walls_and_covers():
    src = DOOR.read_text(encoding="utf-8")
    assert "wall_front_out: Optional[float] = None" in src and "cover_front_out: Optional[float] = None" in src
    assert src.count("(wall_x, ") == 3 and src.count("(wall_hx, ") == 3  # 脇の壁 2 つと上の壁
    assert src.count("(cover_x, ") == 3 and src.count("(cover_hx, ") == 3  # 枠 3 つ
    assert "(-0.02, half_wall_length" not in src and "(0.06, door_cover_width" not in src
