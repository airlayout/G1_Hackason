"""パッチ 0095（11-07・実機の記録から作る純正の応え方）の試験。
環境は Isaac Sim なしでは import できないので、homie_base.py から関数だけを切り出して偽の環境で動かす（0085 の試験と同じ）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- env.config.loco_resp が無い・enabled=False なら、0085 のまま（何も設定しなければ脚に届く指令＝方策の指令）
- 有効なら: 軸ごとの遅れ → 効かない帯 → 向きごとの倍率の折れ線 × 回ごとの乱択 → どれかの軸が歩いている間だけ流れを足す
- 回ごとの遅れ・乱択は、戻した台だけ引き直す
- env.config.loco_cmd_replay: 台 i は列 i % K を回の始めから流し、尽きたら 0。step の頭打ちの後・脚の層の前で置き換える
- 記録（state_recorder）: homie_cmd と legs_cmd（脚の層が切なら同じ物）
"""
from __future__ import annotations

import ast
from pathlib import Path

import numpy as np
import pytest

HOMIE = Path("gr00t/rl/envs/base_task/homie_base.py")
RECORDER = Path("gr00t/rl/utils/state_recorder.py")
FNS = ("_get_obs_b_homie_commands", "_get_obs_b_homie_commands_legs", "_init_stock_legs", "_reset_stock_legs",
       "_update_stock_legs", "_apply_stock_legs_obs", "_init_loco_resp", "_reset_loco_resp", "_interp",
       "_loco_resp_map", "_init_loco_cmd_replay", "_replay_loco_cmd")


def _class_body(path: Path, name: str) -> list:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name).body


def _method(path: Path, cls: str, name: str) -> ast.FunctionDef:
    for node in _class_body(path, cls):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{cls}.{name} が {path} に無い")


class _Config(dict):
    class _Obs:
        obs_scales = {"b_homie_commands": 1.0}

    obs = _Obs()


def _make_env(cfg: dict, num_envs: int = 3):
    torch = pytest.importorskip("torch")
    ns = {"torch": torch}
    exec(compile(ast.Module(body=[_method(HOMIE, "HomieBase", n) for n in FNS], type_ignores=[]), str(HOMIE), "exec"), ns)  # noqa: S102

    class Env:
        pass

    for n in FNS:
        setattr(Env, n, ns[n])
    env = Env()
    env.config = _Config(cfg)
    env.num_envs = num_envs
    env.device = "cpu"
    env._num_lower_dof = 15
    env._homie_commands = torch.zeros(num_envs, 7)
    env.episode_length_buf = torch.zeros(num_envs, dtype=torch.long)
    env._init_stock_legs()
    return env, torch


def _send(env, torch, values):
    """方策の指令（前後・横・旋回）を 1 歩ぶん入れて、脚に届く指令を返す（step の中と同じ順）"""
    cmd = torch.zeros(env.num_envs, 7)
    cmd[:, :3] = torch.as_tensor(values, dtype=torch.float32)
    env._homie_commands = cmd
    env._replay_loco_cmd()
    env._update_stock_legs()
    return env._homie_commands_legs[:, :3].clone() if env._stock_legs_on else env._homie_commands[:, :3].clone()


def _resp(**kw):
    base = dict(enabled=True, delay_steps=[[0, 0], [0, 0], [0, 0]], dead=[0.0, 0.0, 0.0],
                knots=[[0.0], [0.0], [0.0]], gain_pos=[[1.0], [1.0], [1.0]], gain_neg=[[1.0], [1.0], [1.0]],
                bias=[0.0, 0.0, 0.0], gain_rand=0.0, bias_rand=0.0)
    base.update(kw)
    return base


def test_default_is_unchanged():
    env, torch = _make_env({})
    assert not env._stock_legs_on and not env._loco_resp_on and env._loco_replay is None
    np.testing.assert_allclose(_send(env, torch, [0.2, -0.1, 0.3]).numpy(), [[0.2, -0.1, 0.3]] * 3, atol=1e-7)
    env, torch = _make_env({"loco_resp": _resp(enabled=False)})
    assert not env._stock_legs_on


def test_0085_path_kept_when_resp_off():
    env, torch = _make_env({"loco_cmd_gain_range": [0.5, 0.5]})
    np.testing.assert_allclose(_send(env, torch, [0.2, -0.2, 0.4]).numpy(), [[0.1, -0.1, 0.2]] * 3, atol=1e-7)


def test_per_axis_delay():
    env, torch = _make_env({"loco_resp": _resp(delay_steps=[[2, 2], [5, 5], [0, 0]])}, num_envs=1)
    seen = [_send(env, torch, [0.1 * (t + 1), 0.01 * (t + 1), 1.0 * (t + 1)])[0].numpy() for t in range(8)]
    for t in range(8):
        want_x = 0.1 * (t - 1) if t >= 2 else 0.0
        want_y = 0.01 * (t - 4) if t >= 5 else 0.0
        np.testing.assert_allclose(seen[t], [want_x, want_y, 1.0 * (t + 1)], atol=1e-6)


def test_dead_band_and_gain_curve_by_sign():
    resp = _resp(dead=[0.05, 0.1, 0.0], knots=[[0.1, 0.3], [0.0], [0.0]], gain_pos=[[0.3, 0.6], [0.5], [1.0]],
                 gain_neg=[[0.2, 0.4], [0.5], [0.5]])
    env, torch = _make_env({"loco_resp": resp}, num_envs=1)
    cases = [  # (指令, 脚に届く前後・横・旋回)
        ([0.04, 0.0, 0.0], [0.0, 0.0, 0.0]),          # 前後の帯の中
        ([0.2, 0.0, 0.0], [0.2 * 0.45, 0.0, 0.0]),     # 折れ線の中ほど
        ([-0.2, 0.0, 0.0], [-0.2 * 0.3, 0.0, 0.0]),    # 負の向きは別の倍率
        ([0.5, 0.0, 0.0], [0.5 * 0.6, 0.0, 0.0]),      # 折れ線の外は端の倍率
        ([0.07, 0.0, 0.0], [0.07 * 0.3, 0.0, 0.0]),    # 帯より上・knots より下は端の倍率
        ([0.0, 0.08, 0.0], [0.0, 0.0, 0.0]),           # 横の帯の中
        ([0.0, 0.2, -0.4], [0.0, 0.1, -0.2]),          # 旋回の負は 0.5 倍
    ]
    for cmd, want in cases:
        np.testing.assert_allclose(_send(env, torch, cmd)[0].numpy(), want, atol=1e-6, err_msg=str(cmd))


def test_bias_only_while_walking():
    env, torch = _make_env({"loco_resp": _resp(dead=[0.05, 0.05, 0.05], bias=[0.0, -0.025, 0.01])}, num_envs=1)
    np.testing.assert_allclose(_send(env, torch, [0.0, 0.0, 0.0])[0].numpy(), [0.0, 0.0, 0.0], atol=1e-7)
    np.testing.assert_allclose(_send(env, torch, [0.03, 0.0, 0.0])[0].numpy(), [0.0, 0.0, 0.0], atol=1e-7)  # 帯の中は立つ
    np.testing.assert_allclose(_send(env, torch, [0.2, 0.0, 0.0])[0].numpy(), [0.2, -0.025, 0.01], atol=1e-6)


def test_random_scale_and_delay_redrawn_only_for_reset_envs():
    torch = pytest.importorskip("torch")
    torch.manual_seed(0)
    env, torch = _make_env({"loco_resp": _resp(delay_steps=[[1, 4], [0, 9], [2, 2]], gain_rand=0.2, bias_rand=0.5,
                                                bias=[0.0, -0.02, 0.0])}, num_envs=64)
    assert env._loco_cmd_buf.shape[1] == 10  # 最も長い遅れ＋1
    d, s, b = env._resp_delay.clone(), env._resp_scale.clone(), env._resp_bias_env.clone()
    assert (d[:, 0] >= 1).all() and (d[:, 0] <= 4).all() and (d[:, 1] <= 9).all() and (d[:, 2] == 2).all()
    assert (s >= 0.8 - 1e-6).all() and (s <= 1.2 + 1e-6).all() and s.std() > 0.01
    assert (b[:, 1] <= -0.01 + 1e-7).all() and (b[:, 1] >= -0.03 - 1e-7).all()
    ids = torch.tensor([0, 5])
    env._reset_stock_legs(ids)
    keep = torch.ones(64, dtype=torch.bool)
    keep[ids] = False
    assert torch.equal(env._resp_delay[keep], d[keep]) and torch.equal(env._resp_scale[keep], s[keep])


def test_replay_by_env_and_time(tmp_path):
    seq = np.zeros((2, 3, 3), dtype=np.float32)
    seq[0, :, 0] = [0.1, 0.2, 0.3]
    seq[1, :, 1] = [-0.1, -0.2, -0.3]
    p = tmp_path / "seq.npz"
    np.savez(p, cmd=seq)
    env, torch = _make_env({"loco_cmd_replay": {"path": str(p)}}, num_envs=3)
    for t, want in enumerate([[[0.1, 0, 0], [0, -0.1, 0], [0.1, 0, 0]],
                              [[0.2, 0, 0], [0, -0.2, 0], [0.2, 0, 0]],
                              [[0.3, 0, 0], [0, -0.3, 0], [0.3, 0, 0]],
                              [[0.0, 0, 0], [0, 0.0, 0], [0.0, 0, 0]]]):  # 尽きたら 0
        env.episode_length_buf[:] = t
        np.testing.assert_allclose(_send(env, torch, [0.5, 0.5, 0.5]).numpy(), want, atol=1e-6)
    env.episode_length_buf = torch.tensor([0, 2, 9])
    np.testing.assert_allclose(_send(env, torch, [0.5, 0.5, 0.5]).numpy(), [[0.1, 0, 0], [0, -0.3, 0], [0, 0, 0]], atol=1e-6)


def test_step_replays_before_the_leg_layer():
    step = _method(HOMIE, "HomieBase", "step")
    calls = [n.func.attr for n in ast.walk(step) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    assert "_replay_loco_cmd" in calls and "_update_stock_legs" in calls
    lines = {n.func.attr: n.lineno for n in ast.walk(step) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert lines["_replay_loco_cmd"] < lines["_update_stock_legs"]
    clamp = min(n.lineno for n in ast.walk(step) if isinstance(n, ast.Attribute) and n.attr == "clamp")
    assert clamp < lines["_replay_loco_cmd"]


def test_recorder_writes_commands():
    torch = pytest.importorskip("torch")
    fn = _method(RECORDER, "StateRecorder", "_loco_cmds")
    ns = {}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(RECORDER), "exec"), ns)  # noqa: S102
    loco_cmds = ns["_loco_cmds"].__func__ if hasattr(ns["_loco_cmds"], "__func__") else ns["_loco_cmds"]

    class Env:
        pass

    env = Env()
    assert loco_cmds(env, 2) == {}
    env._homie_commands = torch.tensor([[0.1, 0.2, 0.3, 0.7], [0.4, 0.5, 0.6, 0.7], [9.0, 9.0, 9.0, 9.0]])
    env._homie_commands_legs = torch.zeros(3, 4)
    env._stock_legs_on = False
    out = loco_cmds(env, 2)
    np.testing.assert_allclose(out["legs_cmd"], out["homie_cmd"])
    np.testing.assert_allclose(out["homie_cmd"], [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]], atol=1e-7)
    env._stock_legs_on = True
    np.testing.assert_allclose(loco_cmds(env, 2)["legs_cmd"], np.zeros((2, 3)))
