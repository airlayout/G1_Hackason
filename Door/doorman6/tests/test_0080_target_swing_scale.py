"""パッチ 0080（10-28・F3 の型 B の候補 2: 押し開けの間は扉の先の目標へ近づく報酬を止められるようにする）の試験。
Isaac Sim は起こさない（環境は omni.usd を読むので import せず、コードの組み立てを読んで確かめる）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- _reward_target_root_distance の、押し開けの区間（段階 4・STAGE_SWING）に掛ける倍率を、設定 target_root_distance_swing_scale から読む
  （既定は元と同じ 0.5。段 E は 0.0 にして、押し開けの間は先へ歩いても点が入らないようにする）
- 効く段階（押し開けと通り抜け）と、報酬の中身（速さと位置の 2 つの和を 1 で頭打ち）は変えない
"""
from __future__ import annotations

import ast
from pathlib import Path

DOOR_ENV = Path("gr00t/rl/envs/door/door_open_homie.py")


def _method(name: str) -> ast.FunctionDef:
    tree = ast.parse(DOOR_ENV.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} が {DOOR_ENV} に無い")


def test_swing_scale_is_read_from_config_with_the_old_default():
    src = ast.unparse(_method("_reward_target_root_distance"))
    assert "reward[self.stage_buf == DoorPregrasp.STAGE_SWING] *= float(self.config.get('target_root_distance_swing_scale', 0.5))" in src
    assert "*= 0.5" not in src  # 決め打ちの 0.5 は残さない


def test_stages_and_reward_shape_are_unchanged():
    fn = _method("_reward_target_root_distance")
    assert [ast.unparse(d) for d in fn.decorator_list] == ["StagedTaskBase.effective_in_stage([STAGE_SWING, STAGE_THROUGH])"]
    src = ast.unparse(fn)
    assert "reward = (root_vel_reward + root_pos_reward).clamp(max=1.0)" in src
    assert "self.config.get('target_root_vel', 0.3)" in src
