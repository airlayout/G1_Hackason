"""パッチ 0085（10-30・純正に似せた脚）の試験。
環境は Isaac Sim なしでは import できないので、homie_base.py から関数だけを切り出して偽の環境で動かす。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 既定（設定なし）では何も変えない（脚に届く指令＝生徒の指令・観測は同じ物）
- 遅れ: 脚に届く指令は、台ごとの遅れ d 歩前の生徒の指令。回の始めの d 歩は 0
- 倍率は前後・横・旋回の 3 つにだけ掛かり、高さ・姿勢の指令は変えない
- 遅れと倍率は HOMIE の観測とその履歴にだけ入り、生徒（actor_obs）の指令は変えない
- 脚と腰の値を隠すのは生徒の dof_pos・dof_vel・actions の先頭 15 だけ。critic と履歴の値は元のまま
- 呼ぶ場所: __init__ の終わり・回の戻し・step の頭打ちの後（物理の前）・観測の連結の前
- 0086（11-01）: 隠す回の割合 p。台の戻しごとに引き直し、隠す回の台だけ脚と腰を 0 にする。True は p=1 と同じ
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

HOMIE = Path("gr00t/rl/envs/base_task/homie_base.py")
NEW_FNS = ("_get_obs_b_homie_commands", "_get_obs_b_homie_commands_legs", "_init_stock_legs", "_reset_stock_legs",
           "_update_stock_legs", "_apply_stock_legs_obs")
N_LOWER = 15


def _class(name: str) -> ast.ClassDef:
    tree = ast.parse(HOMIE.read_text(encoding="utf-8"))
    return next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)


def _method(name: str) -> ast.FunctionDef:
    for node in _class("HomieBase").body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"HomieBase.{name} が {HOMIE} に無い")


def _calls(fn: ast.FunctionDef) -> list[str]:
    """関数の中で self.<名前>(...) を呼ぶ順"""
    out = []
    for node in ast.walk(fn):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            out.append((node.lineno, node.col_offset, node.func.attr))
    return [name for _, _, name in sorted(out)]


class _Config(dict):
    """env.config の代わり（get と obs.obs_scales だけ使う）"""

    class _Obs:
        obs_scales = {"b_homie_commands": 1.0}

    obs = _Obs()


def _make_env(cfg: dict, num_envs: int = 3):
    torch = pytest.importorskip("torch")
    ns = {"torch": torch}
    fns = [_method(n) for n in NEW_FNS]
    exec(compile(ast.Module(body=fns, type_ignores=[]), str(HOMIE), "exec"), ns)  # noqa: S102

    class Env:
        pass

    for n in NEW_FNS:
        setattr(Env, n, ns[n])
    env = Env()
    env.config = _Config(cfg)
    env.num_envs = num_envs
    env.device = "cpu"
    env._num_lower_dof = N_LOWER
    env._homie_commands = torch.zeros(num_envs, 7)
    env._init_stock_legs()
    return env, torch


def _send(env, torch, values):
    """生徒の指令（前後・横・旋回）を 1 歩ぶん入れて、脚に届く指令を返す（step の中と同じ順）"""
    cmd = torch.zeros(env.num_envs, 7)
    cmd[:, :3] = torch.as_tensor(values, dtype=torch.float32)
    cmd[:, 3] = 0.7  # 高さ
    env._homie_commands = cmd
    env._update_stock_legs()
    return env._homie_commands_legs


def test_off_by_default_changes_nothing():
    env, torch = _make_env({})
    assert env._stock_legs_on is False and env._mask_lower_obs is False
    legs_before = env._homie_commands_legs
    _send(env, torch, [[0.3, 0.1, 0.2]] * 3)
    assert env._homie_commands_legs is legs_before  # 更新しない（obs も元の _get_obs_b_homie_commands のまま）
    actor = {"dof_pos": torch.ones(3, 29), "b_homie_commands": torch.ones(3, 7)}
    homie = {"b_homie_commands": torch.ones(3, 7)}
    env.obs_buf_dict_raw = {"actor_obs": dict(actor), "homie_obs": dict(homie)}
    env.hist_obs_dict = {"b_homie_commands": actor["b_homie_commands"]}
    env._apply_stock_legs_obs()
    assert env.obs_buf_dict_raw["actor_obs"]["dof_pos"] is actor["dof_pos"]
    assert env.obs_buf_dict_raw["homie_obs"]["b_homie_commands"] is homie["b_homie_commands"]
    assert env.hist_obs_dict["b_homie_commands"] is actor["b_homie_commands"]


def test_delay_is_per_env_and_starts_from_zero():
    env, torch = _make_env({"loco_cmd_delay_steps_range": [0, 3]})
    env._loco_cmd_delay = torch.tensor([0, 1, 3])
    got = [_send(env, torch, [[t, 0.0, 0.0]] * 3)[:, 0].tolist() for t in range(1, 7)]
    # 台 0 は遅れなし・台 1 は 1 歩・台 2 は 3 歩（始めの 3 歩は 0）
    assert [g[0] for g in got] == [1, 2, 3, 4, 5, 6]
    assert [g[1] for g in got] == [0, 1, 2, 3, 4, 5]
    assert [g[2] for g in got] == [0, 0, 0, 1, 2, 3]
    assert _send(env, torch, [[9.0, 0.0, 0.0]] * 3)[:, 3].tolist() == pytest.approx([0.7] * 3)  # 高さは遅らせない


def test_reset_clears_only_that_env_and_samples_within_range():
    env, torch = _make_env({"loco_cmd_delay_steps_range": [2, 5], "loco_cmd_gain_range": [0.8, 1.2]}, num_envs=64)
    assert int(env._loco_cmd_delay.min()) >= 2 and int(env._loco_cmd_delay.max()) <= 5
    assert float(env._loco_cmd_gain.min()) >= 0.8 and float(env._loco_cmd_gain.max()) <= 1.2
    env._loco_cmd_delay[:] = 2
    env._loco_cmd_gain[:] = 1.0
    for _ in range(4):
        _send(env, torch, [[0.4, 0.0, 0.0]] * 64)
    env._reset_stock_legs(torch.tensor([5]))
    env._loco_cmd_delay[5] = 2
    env._loco_cmd_gain[5] = 1.0
    legs = _send(env, torch, [[0.4, 0.0, 0.0]] * 64)
    assert legs[5, 0].item() == 0.0  # 戻した台は始めからやり直す
    assert legs[4, 0].item() == pytest.approx(0.4)  # ほかの台はそのまま


def test_gain_scales_only_walk_commands():
    env, torch = _make_env({"loco_cmd_gain_range": [0.5, 0.5]})
    legs = _send(env, torch, [[0.4, -0.2, 0.3]] * 3)
    assert legs[0, :3].tolist() == pytest.approx([0.2, -0.1, 0.15])
    assert legs[0, 3].item() == pytest.approx(0.7)
    assert env._homie_commands[0, :3].tolist() == pytest.approx([0.4, -0.2, 0.3])  # 生徒の指令は変えない


def test_legs_commands_go_to_homie_obs_and_history_only():
    env, torch = _make_env({"loco_cmd_delay_steps_range": [1, 1]})
    _send(env, torch, [[0.4, 0.0, 0.0]] * 3)  # 1 歩遅れなので脚にはまだ 0
    actor_cmd = env._get_obs_b_homie_commands()
    env.obs_buf_dict_raw = {"actor_obs": {"b_homie_commands": actor_cmd}, "homie_obs": {"b_homie_commands": actor_cmd},
                            "critic_obs": {"b_homie_commands": actor_cmd}}
    env.hist_obs_dict = {"b_homie_commands": actor_cmd}
    env._apply_stock_legs_obs()
    raw = env.obs_buf_dict_raw
    assert raw["actor_obs"]["b_homie_commands"][0, 0].item() == pytest.approx(0.8)  # 観測の尺度は前後 ×2
    assert raw["critic_obs"]["b_homie_commands"][0, 0].item() == pytest.approx(0.8)
    assert raw["homie_obs"]["b_homie_commands"][0, 0].item() == 0.0
    assert env.hist_obs_dict["b_homie_commands"][0, 0].item() == 0.0
    _send(env, torch, [[0.4, 0.0, 0.0]] * 3)
    assert env._get_obs_b_homie_commands_legs()[0, 0].item() == pytest.approx(0.8)


def test_legs_obs_keeps_the_dead_band():
    env, torch = _make_env({"loco_cmd_delay_steps_range": [0, 0], "loco_cmd_gain_range": [0.9, 0.9]})
    _send(env, torch, [[0.04, 0.0, 0.0]] * 3)  # 観測では 0.072 < 0.1 なので 0（元の関数と同じ止まりの帯）
    assert env._get_obs_b_homie_commands_legs()[0, :3].tolist() == [0.0, 0.0, 0.0]


def test_mask_hides_lower_body_from_the_student_only():
    env, torch = _make_env({"actor_obs_mask_lower_body": True})
    assert env._stock_legs_on is False and env._mask_lower_obs is True
    dof = torch.arange(29.0).repeat(3, 1)
    ang = torch.ones(3, 3)
    env.obs_buf_dict_raw = {"actor_obs": {"dof_pos": dof, "dof_vel": dof * 2, "actions": dof * 3, "base_ang_vel": ang},
                            "critic_obs": {"dof_pos": dof}}
    env.hist_obs_dict = {"dof_pos": dof}
    env._apply_stock_legs_obs()
    actor = env.obs_buf_dict_raw["actor_obs"]
    for key, k in (("dof_pos", 1), ("dof_vel", 2), ("actions", 3)):
        assert actor[key][:, :N_LOWER].abs().sum().item() == 0.0
        assert actor[key][0, N_LOWER:].tolist() == [k * float(i) for i in range(N_LOWER, 29)]
    assert actor["base_ang_vel"] is ang  # ほかの観測には触らない
    assert env.obs_buf_dict_raw["critic_obs"]["dof_pos"] is dof and dof[0, 1].item() == 1.0  # 元の値は書き換えない
    assert env.hist_obs_dict["dof_pos"] is dof


def test_bad_ranges_are_rejected():
    with pytest.raises(AssertionError):
        _make_env({"loco_cmd_delay_steps_range": [3, 1]})
    with pytest.raises(AssertionError):
        _make_env({"loco_cmd_gain_range": [0.0, 1.0]})


# ---- パッチ 0086（11-01）: 脚と腰を隠す回の割合 actor_obs_mask_lower_body_prob ----


def test_mask_all_equals_prob_one():
    env, torch = _make_env({"actor_obs_mask_lower_body": True}, num_envs=8)
    assert env._mask_lower_prob == 1.0 and bool(env._mask_lower_env.all())
    env._reset_stock_legs(torch.arange(8))
    assert bool(env._mask_lower_env.all())  # 全部隠すときは引き直さない


def test_mask_prob_hides_only_the_sampled_envs():
    torch = pytest.importorskip("torch")
    torch.manual_seed(0)
    env, torch = _make_env({"actor_obs_mask_lower_body_prob": 0.5}, num_envs=4000)
    assert 0.45 < env._mask_lower_env.float().mean().item() < 0.55
    dof = torch.arange(29.0).repeat(4000, 1)
    env.obs_buf_dict_raw = {"actor_obs": {"dof_pos": dof}}
    env.hist_obs_dict = {}
    env._apply_stock_legs_obs()
    out = env.obs_buf_dict_raw["actor_obs"]["dof_pos"]
    hidden = env._mask_lower_env
    assert out[hidden, :N_LOWER].abs().sum().item() == 0.0
    assert torch.equal(out[~hidden], dof[~hidden])  # 隠さない回はそのまま
    assert torch.equal(out[:, N_LOWER:], dof[:, N_LOWER:])  # 腕は隠さない
    assert dof[0, 1].item() == 1.0  # 元の値は書き換えない


def test_mask_prob_is_resampled_only_for_reset_envs():
    torch = pytest.importorskip("torch")
    torch.manual_seed(1)
    env, torch = _make_env({"actor_obs_mask_lower_body_prob": 0.5}, num_envs=64)
    before = env._mask_lower_env.clone()
    changed = False
    for _ in range(20):
        env._reset_stock_legs(torch.arange(0, 64, 2))
        assert torch.equal(env._mask_lower_env[1::2], before[1::2])  # 戻していない台はそのまま
        changed |= not torch.equal(env._mask_lower_env[::2], before[::2])
    assert changed


def test_mask_prob_zero_is_off_and_bad_values_are_rejected():
    env, _ = _make_env({"actor_obs_mask_lower_body_prob": 0.0})
    assert env._mask_lower_obs is False and not bool(env._mask_lower_env.any())
    with pytest.raises(AssertionError):
        _make_env({"actor_obs_mask_lower_body_prob": 1.5})


def test_wiring():
    init_calls = _calls(_method("__init__"))
    assert init_calls[-1] == "_init_stock_legs"  # バッファ（_homie_commands）を作った後
    assert "_reset_stock_legs" in _calls(_method("_reset_buffers_callback"))
    step = _calls(_method("step"))
    assert step.index("clamp") < step.index("_update_stock_legs") < step.index("_pre_physics_step")
    post = _calls(_method("_post_config_observation_callback"))
    assert post.index("_apply_stock_legs_obs") < post.index("_post_config_observation_callback")
