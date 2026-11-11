"""パッチ 0105（11-11・腰 yaw で体の回りを打ち消す env.config.waist_yaw_comp。計画 2026-11-11 の K3）の試験。
Isaac Sim は起こさない（waist_yaw_comp.py は torch だけ。扉の環境は関数を取り出して偽の環境で動かす）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 設定が無い・enabled=False なら元のまま（None）。誤りは止める
- 打ち消し = −倍率 × 回の始めからの pelvis の回り（上限で切る）。回りは ±π をまたいでも正しく出す
- 扉の値を胴の向きへ: 位置と向きを −打ち消し だけ回す（pelvis が +Δ 回り、腰で −Δ 戻した胴から見た扉は元の位置）
- 扉の環境: 回の始め（episode_length_buf <= 1）で基準の向きを取り直す。目標の腰 yaw に足す・観測から引く・扉の値を回す口がつながっている
"""
from __future__ import annotations

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from gr00t.rl.envs.door.waist_yaw_comp import (
    comp_offset,
    rotate_rel_yaw,
    waist_yaw_comp_config,
    wrap_angle,
    yaw_from_xyzw,
)

ENV = Path("gr00t/rl/envs/door/door_open_homie.py")
FPB = Path("gr00t/rl/envs/base_task/finger_primitive_base.py")


def _method(path: Path, name: str):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    fn.decorator_list = []
    return fn


def _load(names: list[str], extra: dict):
    ns = {"torch": torch, **extra}
    exec(compile(ast.Module(body=[_method(ENV, n) for n in names], type_ignores=[]), str(ENV), "exec"), ns)  # noqa: S102
    return ns


def _qz(a: float) -> list[float]:
    return [0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]  # xyzw


def test_config_off_by_default_and_checked():
    assert waist_yaw_comp_config(None) is None
    assert waist_yaw_comp_config({"enabled": False}) is None
    cfg = waist_yaw_comp_config({"enabled": True})
    assert cfg == {"gain": 1.0, "max_rad": pytest.approx(math.radians(35.0)), "door_obs": "torso", "hide_obs": True}
    with pytest.raises(AssertionError):
        waist_yaw_comp_config({"enabled": True, "gain": -0.5})
    with pytest.raises(AssertionError):
        waist_yaw_comp_config({"enabled": True, "max_deg": 0.0})
    with pytest.raises(AssertionError):
        waist_yaw_comp_config({"enabled": True, "door_obs": "head"})


def test_offset_sign_clamp_and_wrap():
    dyaw = torch.tensor([math.radians(20.0), -math.radians(10.0), math.radians(60.0)])
    out = comp_offset(dyaw, 1.0, math.radians(35.0))
    assert torch.allclose(out, torch.tensor([-math.radians(20.0), math.radians(10.0), -math.radians(35.0)]))
    assert torch.allclose(comp_offset(dyaw[:1], 0.5, 1.0), torch.tensor([-math.radians(10.0)]))
    a = wrap_angle(torch.tensor([math.pi - 0.1]) - torch.tensor([-math.pi + 0.1]))
    assert torch.allclose(a, torch.tensor([-0.2]), atol=1e-6)
    q = torch.tensor([_qz(0.7), _qz(-2.5)])
    assert torch.allclose(yaw_from_xyzw(q), torch.tensor([0.7, -2.5]), atol=1e-6)


def test_rotate_rel_yaw_undoes_pelvis_turn():
    # 扉は pelvis の前 1 m・正対。pelvis が +20° 回ると、pelvis から見た扉は −20° 回った所に見える
    d = math.radians(20.0)
    seen = torch.tensor([[math.cos(-d), math.sin(-d), 0.5]])
    rot = torch.tensor([_qz(-d)])
    off = torch.tensor([-d])  # 腰で −20° 戻す
    pos, q = rotate_rel_yaw(seen, rot, -off)
    assert torch.allclose(pos, torch.tensor([[1.0, 0.0, 0.5]]), atol=1e-6)
    assert torch.allclose(yaw_from_xyzw(q), torch.tensor([0.0]), atol=1e-6)


class _Sim:
    def __init__(self, yaws):
        self._yaws = torch.tensor(yaws)

    @property
    def robot_root_states(self):
        s = torch.zeros(len(self._yaws), 13)
        s[:, 5] = torch.sin(self._yaws / 2)
        s[:, 6] = torch.cos(self._yaws / 2)
        return s


def _env():
    ns = _load(["_wyc_update", "_wyc_hide_obs"], {"comp_offset": comp_offset, "wrap_angle": wrap_angle, "yaw_from_xyzw": yaw_from_xyzw})
    env = SimpleNamespace(
        _wyc={"gain": 1.0, "max_rad": math.radians(35.0), "door_obs": "torso", "hide_obs": True},
        _wyc_idx=12, _wyc_yaw0=torch.zeros(2), _wyc_offset=torch.zeros(2), _wyc_prev=torch.zeros(2),
        simulator=_Sim([0.3, -0.2]), episode_length_buf=torch.tensor([0, 1]), dt=0.02,
        config=SimpleNamespace(obs=SimpleNamespace(obs_scales={"dof_pos": 1.0, "dof_vel": 0.05, "e_homie_dof_pos": 1.0, "f_homie_dof_vel": 0.05})),
        hist_obs_dict={}, obs_buf_dict_raw={},
    )
    env._wyc_update = ns["_wyc_update"].__get__(env)
    env._wyc_hide_obs = ns["_wyc_hide_obs"].__get__(env)
    return env


def test_env_update_takes_reference_at_episode_start():
    env = _env()
    env._wyc_update()
    assert torch.allclose(env._wyc_yaw0, torch.tensor([0.3, -0.2])) and torch.equal(env._wyc_offset, torch.zeros(2))
    env.simulator = _Sim([0.3 + math.radians(15.0), -0.2 - math.radians(50.0)])
    env.episode_length_buf = torch.tensor([30, 30])
    env._wyc_update()
    assert torch.allclose(env._wyc_offset, torch.tensor([-math.radians(15.0), math.radians(35.0)]), atol=1e-5)
    assert torch.equal(env._wyc_prev, torch.zeros(2))


def test_env_hide_obs_subtracts_offset_and_rate_with_scales():
    env = _env()
    env._wyc_offset = torch.tensor([-0.2, 0.1])
    env._wyc_prev = torch.tensor([-0.1, 0.1])
    env.obs_buf_dict_raw = {"actor_obs": {"dof_pos": torch.zeros(2, 29), "dof_vel": torch.zeros(2, 29)},
                            "homie_obs": {"e_homie_dof_pos": torch.zeros(2, 15), "f_homie_dof_vel": torch.zeros(2, 15)}}
    before = env.obs_buf_dict_raw["actor_obs"]["dof_pos"]
    env._wyc_hide_obs()
    a = env.obs_buf_dict_raw["actor_obs"]
    assert a["dof_pos"] is not before  # 写しを書き換える
    assert torch.allclose(a["dof_pos"][:, 12], torch.tensor([0.2, -0.1]))
    assert torch.allclose(a["dof_vel"][:, 12], torch.tensor([5.0 * 0.05, 0.0]))  # (−0.2 − −0.1)/0.02 = −5 rad/s を引く
    assert torch.count_nonzero(a["dof_pos"][:, :12]) == 0 and torch.count_nonzero(a["dof_pos"][:, 13:]) == 0
    assert torch.allclose(env.obs_buf_dict_raw["homie_obs"]["e_homie_dof_pos"][:, 12], torch.tensor([0.2, -0.1]))


def test_sources_wire_target_obs_and_door():
    fpb = FPB.read_text(encoding="utf-8")
    assert 'getattr(self, "_wyc_offset", None)' in fpb and "combined_target[:, self._wyc_idx]" in fpb
    src = ENV.read_text(encoding="utf-8")
    assert 'waist_yaw_comp_config(self.config.get("waist_yaw_comp", None))' in src
    step = ast.unparse(_method(ENV, "step"))
    assert step.index("_wyc_update") < step.index("_apply_yaw_disturb") < step.index("super().step")  # 1 歩遅れ（IMU と同じ）
    door = ast.unparse(_method(ENV, "_update_door_sensor"))
    assert door.index("rotate_rel_yaw") < door.index("self.door_sensor.localize")
    obs = ast.unparse(_method(ENV, "_post_config_observation_callback"))
    assert obs.index("_wyc_hide_obs") < obs.index("_mirror_policy_obs")  # 鏡写しの前に引く
    reset = ast.unparse(_method(ENV, "_reset_buffers_callback"))
    assert "_wyc_offset" in reset
