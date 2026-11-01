"""パッチ 0087（11-01・切り替えの線の写し直し）の試験。
Isaac Sim は起こさない（学習器の _init_trl から、包みを作って切り替えの線を決める数行だけを取り出し、偽の包みで動かす）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 試行（rollout）と評価は学習器の self.homie_switch_threshold で歩くと立つを切り替える
- 元の公開版は、学習器の線を包みの線（既定 0.5）から写した後で、設定の値を包みに書いていた。
  そのため algo.config.homie_switch_threshold に何を書いても、試行と評価の線は 0.5 のままだった
  （10-30 の評価 A1「切り替えの線だけ 0.1」が元の評価と失敗の内訳まで同じだったことで気づいた）
- 直した後は、学習器の線と包みの線がどちらも設定の値になる。設定に書かなければ 0.5（元のまま）
"""
from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

TRAINER = Path("gr00t/rl/trl/trainer/ppo_trainer_homie_api.py")


def _threshold_block() -> ast.Module:
    """_init_trl の「包みを作る」行から、最適化器を作る行の手前までの文"""
    tree = ast.parse(TRAINER.read_text(encoding="utf-8"))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_init_trl")
    src = [ast.unparse(s) for s in fn.body]
    start = next(i for i, s in enumerate(src) if s.startswith("self.model = PolicyAndValueWrapper("))
    end = next(i for i, s in enumerate(src) if i > start and "create_optimizer_and_scheduler" in s)
    return ast.Module(body=fn.body[start:end], type_ignores=[])


class _FakeWrapper:
    """PolicyAndValueWrapper の代わり。切り替えの線の既定値は本物と同じ 0.5"""

    def __init__(self, policy, value_model, homie_walk_model, homie_stand_model):
        self.opt_homie = False
        self.homie_switch_threshold = 0.5


def _run(config: dict) -> SimpleNamespace:
    trainer = SimpleNamespace(config=config, policy_model=None, value_model=None, homie_walk_model=None, homie_stand_model=None)
    exec(compile(_threshold_block(), str(TRAINER), "exec"), {"PolicyAndValueWrapper": _FakeWrapper, "self": trainer})
    return trainer


@pytest.mark.parametrize("value", [0.1, 0.3])
def test_rollout_and_eval_use_the_configured_switch_threshold(value):
    trainer = _run({"homie_switch_threshold": value})
    assert trainer.model.homie_switch_threshold == value
    assert trainer.homie_switch_threshold == value  # 試行と評価が使う線


def test_default_switch_threshold_is_unchanged():
    trainer = _run({})
    assert trainer.model.homie_switch_threshold == 0.5
    assert trainer.homie_switch_threshold == 0.5


def test_rollout_and_eval_compare_against_the_trainer_threshold():
    # 試行（_rollout 側）と評価の walk_mask は、学習器の線と比べる（この試験が守っている先）
    src = TRAINER.read_text(encoding="utf-8")
    assert src.count("torch.norm(commands, dim=-1, keepdim=True) > self.homie_switch_threshold") >= 3
