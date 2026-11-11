"""パッチ 0082（10-29・G2: 止まるはずの場面では脚を立つ HOMIE に固定する掛け金）の試験。
Isaac Sim は起こさない（環境は omni.usd を読むので import せず、コードの組み立てを読んで確かめる）。GPU も使わない。
掛け金そのもの（stand_latch.py）の振る舞いは、実機側の元 G1_Hackason/Door/real/tests/test_stand_latch.py で確かめる。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- DeltaActionBase.step: 腕の積み上げを切った後・積み上げを行動に書く前に _stand_latch_mask() を呼び、止める台の歩く指令の
  積み上げ（並びの 0〜2 番）を 0 にする。基の _stand_latch_mask は None（掛け金を使わない環境は何も変わらない）
- 扉の環境: 設定 stand_latch が無ければ None。あれば胴と把手の水平距離・扉の角度で update_stand_latch を進める。
  段階（stage_buf）は使わない（実機に無い）。線の既定は 0.3 m・0.5 m・10°・一度開いたら止めない
- 回の始めに掛け金と「一度開いた」を消す
"""
from __future__ import annotations

import ast
from pathlib import Path

BASE = Path("gr00t/rl/envs/base_task/delta_action_base.py")
DOOR_ENV = Path("gr00t/rl/envs/door/door_open_homie.py")
LATCH = Path("gr00t/rl/envs/base_task/stand_latch.py")


def _method(path: Path, cls: str, name: str) -> ast.FunctionDef:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    c = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
    for node in c.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{cls}.{name} が {path} に無い")


def test_step_zeroes_the_walking_command_of_latched_envs_before_writing_actions():
    lines = [ast.unparse(s) for s in _method(BASE, "DeltaActionBase", "step").body]
    i_guard = next(i for i, s in enumerate(lines) if s.startswith("guard_delta = "))
    i_latch = lines.index("latch = self._stand_latch_mask()")
    i_write = lines.index("actor_state['actions'][:, self._delta_action_indices] = self._delta_actions")
    assert i_guard < i_latch < i_write
    block = lines[i_latch + 1]
    assert block.startswith("if latch is not None:")
    assert "self._delta_actions[latch, :ARM_ACCUM_START] = 0.0" in block


def test_base_latch_is_off():
    fn = _method(BASE, "DeltaActionBase", "_stand_latch_mask")
    assert [ast.unparse(s) for s in fn.body if not isinstance(s, ast.Expr)] == ["return None"]


def test_door_latch_uses_only_what_the_real_robot_can_measure():
    fn = _method(DOOR_ENV, "DoorPregrasp", "_stand_latch_mask")
    src = ast.unparse(fn)
    body = [ast.unparse(s) for s in fn.body]
    # 0097（11-09）: 腕だけの段（env.config.arm_stage）なら先に「毎歩すべての台を止める」で返る。その次が元の入口
    if body[0] == "arm = getattr(self, '_arm_stage', None)":  # 0099 から、腕だけの段の判定は _arm_stage_legs_mask に分けた
        assert body[1] == "if arm is not None:\n    return self._arm_stage_legs_mask(arm)"
        body = body[2:]
    assert body[0] == "if not self.config.get('stand_latch', False):\n    return None"
    assert "torch.norm(self.simulator.robot_root_states[:, :2] - self._compute_grasp_target()[:, :2], dim=-1)" in src
    assert "torch.rad2deg(torch.abs(self.simulator.get_task_dof_pos('door')[:, 0]))" in src
    assert "stage_buf" not in src
    for key, default in (("stand_latch_enter_m", "0.3"), ("stand_latch_exit_m", "0.5"), ("stand_latch_door_deg", "10.0"),
                         ("stand_latch_once_opened", "True")):
        assert f"self.config.get('{key}', {default})" in src
    assert "update_stand_latch(self._stand_latched, self._door_opened_once, dist, door_deg" in src


def test_reset_clears_the_latch_and_the_opened_flag():
    src = ast.unparse(_method(DOOR_ENV, "DoorPregrasp", "_reset_buffers_callback"))
    assert "self._stand_latched[env_ids] = False" in src
    assert "self._door_opened_once[env_ids] = False" in src


def test_shared_function_is_plain_python():
    tree = ast.parse(LATCH.read_text(encoding="utf-8"), feature_version=(3, 8))
    imports = [n for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom))]
    assert imports == []  # torch も numpy も import しない（PC2 の実機でも同じファイルを使う）
