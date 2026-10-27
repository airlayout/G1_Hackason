"""パッチ 0079（10-27・F3 の型 B: 押し開けの区間のまま扉の先へ歩き去る近道の罰）の試験。torch だけ（Isaac Sim は起こさない。GPU も使わない）。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 罰の量（walk_away.walk_away_amount）: 扉の面から start_m より手前は 0、start_m + span_m より先は 1、間は直線
- 扉の環境（door_open_homie.py）の _reward_walk_away_in_swing が、段階 4（STAGE_SWING）だけに効き、
  設定の walk_away_start_m（既定 1.2）・walk_away_span_m（既定 0.5）で walk_away_amount を呼ぶ
  （環境は omni.usd を読むので import せず、コードの組み立てを読んで確かめる）
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest
import torch
from gr00t.rl.envs.door.walk_away import walk_away_amount

DOOR_ENV = Path("gr00t/rl/envs/door/door_open_homie.py")


def test_zero_before_start_and_one_after_span():
    x = torch.tensor([-1.0, 0.0, 0.58, 1.15, 1.2, 1.45, 1.7, 2.0])
    got = walk_away_amount(x, 1.2, 0.5)
    want = torch.tensor([0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0])
    assert torch.allclose(got, want, atol=1e-6)


def test_successful_runs_are_not_penalized():
    """成功の回が扉 70° に届いた時の位置（評価の記録: 中央 +0.58〜0.74 m・最大 +1.15 m）では 0"""
    assert float(walk_away_amount(torch.tensor([1.15]), 1.2, 0.5)) == 0.0


def test_span_must_be_positive():
    with pytest.raises(ValueError):
        walk_away_amount(torch.tensor([1.5]), 1.2, 0.0)


def _method(name: str) -> ast.FunctionDef:
    tree = ast.parse(DOOR_ENV.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} が {DOOR_ENV} に無い")


def test_env_reward_is_swing_stage_only():
    fn = _method("_reward_walk_away_in_swing")
    decos = [ast.unparse(d) for d in fn.decorator_list]
    assert decos == ["StagedTaskBase.effective_in_stage([STAGE_SWING])"]


def test_env_reward_reads_config_with_defaults():
    src = ast.unparse(_method("_reward_walk_away_in_swing"))
    assert "walk_away_amount(" in src
    assert "self.config.get('walk_away_start_m', 1.2)" in src
    assert "self.config.get('walk_away_span_m', 0.5)" in src
    assert "self.simulator.robot_root_states[:, 0] - self.env_origins[:, 0]" in src
