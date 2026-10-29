"""パッチ 0081（10-29・G1: 腕の出力の 1 歩ごとの変化への罰 penalty_arm_action_change）の試験。
Isaac Sim は起こさない（環境は omni.usd を読むので import せず、コードの組み立てを読んで確かめる）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 1 歩前の出力 _prev_delta_actions を持ち、step で _last_delta_actions を書き換える前に写す。回の始めに 0 に戻す
- 罰は腕 14（積み上げの並びの 3〜16 番）の (a_t − a_{t−1})² の和。歩く指令の 3 つは入れない
- 重みを設定（rewards.reward_scales.penalty_arm_action_change）に書かなければ呼ばれない（元の学習は変わらない）
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

BASE = Path("gr00t/rl/envs/base_task/delta_action_base.py")


def _class() -> ast.ClassDef:
    tree = ast.parse(BASE.read_text(encoding="utf-8"))
    return next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "DeltaActionBase")


def _method(name: str) -> ast.FunctionDef:
    for node in _class().body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} が {BASE} に無い")


def test_prev_buffer_is_made_like_last():
    src = ast.unparse(_method("_init_buffers"))
    assert "self._prev_delta_actions = torch.zeros(self.num_envs, len(self._delta_action_indices)" in src


def test_step_copies_last_before_overwriting_it():
    lines = [ast.unparse(s) for s in _method("step").body]
    i_prev = lines.index("self._prev_delta_actions[:] = self._last_delta_actions")
    i_last = lines.index("self._last_delta_actions[:] = actions[:, self._delta_action_indices]")
    assert i_prev == i_last - 1


def test_reset_clears_prev():
    assert "self._prev_delta_actions[env_ids] = 0.0" in ast.unparse(_method("_reset_buffers_callback"))


def test_penalty_is_the_squared_change_of_the_14_arm_actions():
    torch = pytest.importorskip("torch")
    fn = _method("_reward_penalty_arm_action_change")
    ns = {"torch": torch, "ARM_ACCUM_START": 3, "NUM_ARM": 14}
    # 自分のリポジトリの関数 1 つだけを切り出して動かす（環境は Isaac Sim なしでは import できない）
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(BASE), "exec"), ns)  # noqa: S102

    class Env:
        pass

    env = Env()
    gen = torch.Generator().manual_seed(0)
    env._last_delta_actions = torch.randn(5, 17, generator=gen)
    env._prev_delta_actions = torch.randn(5, 17, generator=gen)
    got = ns["_reward_penalty_arm_action_change"](env)
    want = ((env._last_delta_actions[:, 3:] - env._prev_delta_actions[:, 3:]) ** 2).sum(-1)
    assert torch.allclose(got, want)
    # 歩く指令の 3 つは変えても罰は変わらない
    env._last_delta_actions[:, :3] += 10.0
    assert torch.allclose(ns["_reward_penalty_arm_action_change"](env), want)
    # 一定の向きに動かし続ける（出力が毎歩同じ）なら罰は 0
    env._prev_delta_actions = env._last_delta_actions.clone()
    assert torch.all(ns["_reward_penalty_arm_action_change"](env) == 0)
