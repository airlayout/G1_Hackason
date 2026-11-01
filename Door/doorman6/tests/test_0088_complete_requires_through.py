"""パッチ 0088（11-01・完了に段階 5〔通り抜け〕を要する）の試験。
Isaac Sim は起こさない（扉の環境から _stage_5_to_complete_condition だけを取り出し、偽の環境で動かす）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 元の完了の判定は「扉の先 1.5 m」だけで段階を見ない。扉を door_through_angle_deg（70°）未満しか開けずに抜けても完了になり、
  打ち切りの罰が無く、完了の報酬と早く終えた分の報酬が入っていた（評価の成功 count_success.py は段階 5 を要する）
- env.config.complete_requires_through_stage=True で、段階 5 にいて扉の先 1.5 m を越えたときだけ完了にする
- 設定に書かなければ元のまま（段階を見ない）
"""
from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

ENV = Path("gr00t/rl/envs/door/door_open_homie.py")
STAGE_SWING = 4
STAGE_THROUGH = 5


def _complete_condition():
    """扉の環境の _stage_5_to_complete_condition を、クラスの外の関数として取り出す"""
    tree = ast.parse(ENV.read_text(encoding="utf-8"))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_stage_5_to_complete_condition")
    fn.decorator_list = []
    namespace = {"torch": torch}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(ENV), "exec"), namespace)
    return namespace["_stage_5_to_complete_condition"]


def _env(config: dict, x: list[float], stage: list[int]) -> SimpleNamespace:
    n = len(x)
    root = torch.zeros(n, 13)
    root[:, 0] = torch.tensor(x) + 10.0  # 台の原点（x=10）からの位置が x
    return SimpleNamespace(
        config=config,
        simulator=SimpleNamespace(robot_root_states=root),
        env_origins=torch.full((n, 3), 10.0),
        stage_buf=torch.tensor(stage),
        STAGE_SWING=STAGE_SWING,
        STAGE_THROUGH=STAGE_THROUGH,
    )


# 回ごとの位置（扉の先からの x）と段階: 抜けた押し開け・抜けた通り抜け・手前の通り抜け・手前の押し開け
X = [2.0, 2.0, 1.0, 1.0]
STAGE = [STAGE_SWING, STAGE_THROUGH, STAGE_THROUGH, STAGE_SWING]


def test_default_completes_by_distance_only():
    done = _complete_condition()(_env({}, X, STAGE))
    assert done.tolist() == [True, True, False, False]


def test_requires_through_stage_when_configured():
    done = _complete_condition()(_env({"complete_requires_through_stage": True}, X, STAGE))
    assert done.tolist() == [False, True, False, False]


@pytest.mark.parametrize("value", [False, None])
def test_off_values_keep_the_original(value):
    done = _complete_condition()(_env({"complete_requires_through_stage": value}, X, STAGE))
    assert done.tolist() == [True, True, False, False]
