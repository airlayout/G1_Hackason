"""パッチ 0083（10-29・G2 の作り直し (a): 構える・つかむの段階で歩く HOMIE に入ったら罰 penalty_walk_in_hold）の試験。
Isaac Sim は起こさない（環境は omni.usd を読むので import せず、コードの組み立てを読んで確かめる）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 効く段階は構える（STAGE_PREGRASP）とつかむ（STAGE_GRASP）だけ。レバーの段階は数えない（踏み込みは要る動き）
- 数える量は学習器の切り替えと同じ: HOMIE の観測の指令（_get_obs_b_homie_commands の前後・横・旋回）の大きさ > 0.5
- 重みを設定（rewards.reward_scales.penalty_walk_in_hold）に書かなければ呼ばれない
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

DOOR_ENV = Path("gr00t/rl/envs/door/door_open_homie.py")
HOMIE = Path("gr00t/rl/envs/base_task/homie_base.py")


def _method(path: Path, cls: str, name: str) -> ast.FunctionDef:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    c = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
    for node in c.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{cls}.{name} が {path} に無い")


def test_counts_only_in_pregrasp_and_grasp():
    fn = _method(DOOR_ENV, "DoorPregrasp", "_reward_penalty_walk_in_hold")
    assert [ast.unparse(d) for d in fn.decorator_list] == ["StagedTaskBase.effective_in_stage([STAGE_PREGRASP, STAGE_GRASP])"]


def test_uses_the_same_command_norm_as_the_trainer_switch():
    src = ast.unparse(_method(DOOR_ENV, "DoorPregrasp", "_reward_penalty_walk_in_hold"))
    assert "cmd = self._get_obs_b_homie_commands()[:, :3]" in src
    assert "(torch.norm(cmd, dim=-1) > float(self.config.get('walk_in_hold_threshold', 0.5))).float()" in src
    # 学習器は HOMIE の観測の指令の大きさを homie_switch_threshold（既定 0.5）と比べる
    trainer = Path("gr00t/rl/trl/trainer/ppo_trainer_homie_api.py").read_text(encoding="utf-8")
    assert "walk_mask = torch.norm(commands, dim=-1, keepdim=True) > self.homie_switch_threshold" in trainer
    assert 'self.config.get("homie_switch_threshold", 0.5)' in trainer


def test_penalty_is_one_only_when_the_walking_policy_would_be_used():
    torch = pytest.importorskip("torch")
    obs_fn = _method(HOMIE, "HomieBase", "_get_obs_b_homie_commands")
    fn = _method(DOOR_ENV, "DoorPregrasp", "_reward_penalty_walk_in_hold")
    fn.decorator_list = []
    ns = {"torch": torch}
    # 自分のリポジトリの関数 2 つだけを切り出して動かす（環境は Isaac Sim なしでは import できない）
    exec(compile(ast.Module(body=[obs_fn, fn], type_ignores=[]), str(DOOR_ENV), "exec"), ns)  # noqa: S102

    class Env:
        def __init__(self) -> None:
            self.config: dict = {}

        def _get_obs_b_homie_commands(self):
            return ns["_get_obs_b_homie_commands"](self)

    env = Env()
    # HOMIE の指令（前後・横・旋回・高さ）。観測では前後・横 ×2・旋回 ×0.5
    env._homie_commands = torch.tensor([[0.20, 0.0, 0.0, 0.7], [0.30, 0.0, 0.0, 0.7], [0.0, 0.0, 0.9, 0.7],
                                        [0.0, 0.0, 1.1, 0.7]])
    got = ns["_reward_penalty_walk_in_hold"](env)
    assert got.tolist() == [0.0, 1.0, 0.0, 1.0]  # 0.4・0.6・0.45・0.55 と 0.5 を比べる
