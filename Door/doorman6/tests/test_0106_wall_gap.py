"""パッチ 0106（11-11・枠と壁へのすき間の罰 env.config.wall_gap。計画 2026-11-11-g1-door-stage2-decision.md 5 節 3）の試験。
Isaac Sim は起こさない（wall_gap.py は torch だけ。扉の環境は関数を取り出して偽の環境で動かす）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 設定が無い・enabled=False なら元のまま（None）。誤りは止める
- 上から見た箱までの符号つき距離（外は正・中は負）。枠（縁から枠の幅）と壁（その外）の 2 つの箱・両側の縁
- 腕の点: 指先＝手のひら＋tip_m·（手首→手のひらの向き）・前腕の中ほど＝（手首＋肘）/2
- 費用 = Σ max(0, X − 距離) / X（触れると 1 点あたり 1）
- 重みの自動調整: 違反の割合が目標より多い間は λ を上げ、少なければ下げる（0〜上限）
- 扉の環境: 回の終わりに「回の中で一度でも X 以内に入ったか」を数えて λ を更新（評価の間は更新しない）。扉の customData に枠・壁の寸法を書く
"""
from __future__ import annotations

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from gr00t.rl.envs.door.wall_gap import (
    arm_points,
    box_dist_2d,
    gap_cost,
    gap_to_walls,
    lagrange_step,
    to_door_frame,
    wall_gap_config,
)

ENV = Path("gr00t/rl/envs/door/door_open_homie.py")
DOOR = Path("gr00t/rl/isaac_utils/playground/env_rand/door.py")


def _method(path: Path, name: str):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    fn.decorator_list = []
    return fn


def test_config_off_by_default_and_checked():
    assert wall_gap_config(None) is None
    assert wall_gap_config({"enabled": False}) is None
    cfg = wall_gap_config({"enabled": True})
    assert cfg["x_m"] == 0.06 and cfg["tip_m"] == 0.10 and cfg["target_rate"] == 0.1
    with pytest.raises(AssertionError):
        wall_gap_config({"enabled": True, "x_m": 0.0})
    with pytest.raises(AssertionError):
        wall_gap_config({"enabled": True, "target_rate": 1.5})
    with pytest.raises(AssertionError):
        wall_gap_config({"enabled": True, "lambda_init": 5.0, "lambda_max": 1.0})


def test_box_dist_signed():
    px = torch.tensor([-0.10, -0.04, 0.00, -0.10])
    py = torch.tensor([0.50, 0.50, 0.50, 0.40])
    d = box_dist_2d(px, py, -0.05, 0.25, 0.47, 0.52)
    assert torch.allclose(d, torch.tensor([0.05, -0.01, -0.02, math.hypot(0.05, 0.07)]), atol=1e-6)  # 中は一番近い辺まで


def test_gap_to_walls_frame_wall_and_both_sides():
    edge, fx, wx, cw = (torch.tensor([v]) for v in (0.47, -0.05, -0.03, 0.04))
    P = torch.tensor([[[-0.08, 0.49, 1.0],    # 枠の前 3 cm
                       [-0.08, 0.60, 1.0],    # 壁の前 5 cm
                       [-0.08, -0.60, 1.0],   # 蝶番の側の壁の前 5 cm
                       [-0.08, 0.00, 1.0],    # 開口の真ん中（枠の前の角まで斜めに）
                       [-0.02, 0.44, 1.0]]])  # 縁の内・枠の横の面から 3 cm
    d = gap_to_walls(P, edge, fx, wx, cw)
    assert torch.allclose(d, torch.tensor([[0.03, 0.05, 0.05, math.hypot(0.03, 0.47), 0.03]]), atol=1e-6)


def test_arm_points_and_cost():
    wrist = torch.tensor([[[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]]])
    palm = torch.tensor([[[0.04, 0.0, 1.0], [0.0, 0.04, 1.0]]])
    elbow = torch.tensor([[[-0.2, 0.0, 1.0], [0.0, -0.2, 1.0]]])
    P = arm_points(wrist, palm, elbow, 0.10)
    assert P.shape == (1, 10, 3)
    assert torch.allclose(P[0, 0], torch.tensor([0.14, 0.0, 1.0]))  # 左の指先
    assert torch.allclose(P[0, 3], torch.tensor([-0.1, 0.0, 1.0]))  # 左の前腕の中ほど
    assert torch.allclose(P[0, 5], torch.tensor([0.0, 0.14, 1.0]))  # 右の指先
    d = torch.tensor([[0.10, 0.06, 0.03, 0.0, -0.01]])
    assert torch.allclose(gap_cost(d, 0.06), torch.tensor([0.0 + 0.0 + 0.5 + 1.0 + 7.0 / 6.0]))


def test_to_door_frame_undoes_translation_and_yaw():
    a = math.radians(30.0)
    door_pos = torch.tensor([[1.0, 2.0, 0.0]])
    door_wxyz = torch.tensor([[math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)]])
    local = torch.tensor([[[0.3, -0.1, 1.0]]])
    c, s = math.cos(a), math.sin(a)
    world = torch.tensor([[[1.0 + c * 0.3 + s * 0.1, 2.0 + s * 0.3 - c * 0.1, 1.0]]])
    assert torch.allclose(to_door_frame(world, door_pos, door_wxyz), local, atol=1e-6)


def test_lagrange_step_up_down_and_clip():
    cfg = wall_gap_config({"enabled": True, "lambda_init": 2.0, "lambda_max": 3.0, "lr": 0.01, "target_rate": 0.1, "rate_ema": 0.5})
    lam, rate = lagrange_step(2.0, 0.1, torch.ones(10), cfg)  # 全部違反 → 割合が上がり λ が上がる
    assert rate > 0.1 and lam > 2.0
    lam2, _ = lagrange_step(2.9, 0.9, torch.ones(100), cfg)
    assert lam2 == 3.0  # 上限
    lam3, rate3 = lagrange_step(0.01, 0.0, torch.zeros(100), cfg)
    assert lam3 == 0.0 and rate3 == 0.0  # 下限 0


class _Sim:
    body_names = ["pelvis", "left_elbow_link", "right_elbow_link", "left_wrist_yaw_link", "right_wrist_yaw_link"]


def _env(n=2):
    ns = {"torch": torch, "lagrange_step": lagrange_step}
    exec(compile(ast.Module(body=[_method(ENV, "_wg_episode_end")], type_ignores=[]), str(ENV), "exec"), ns)  # noqa: S102
    env = SimpleNamespace(
        _wg={**wall_gap_config({"enabled": True, "lr": 0.01, "rate_ema": 0.5}), "state_path": None},
        _wg_state={"lambda": 1.0, "rate": 0.0, "events": 0, "violations": 0},
        _wg_min=torch.tensor([0.03, 0.20]), _wg_steps=torch.tensor([50, 50]), is_evaluating=False,
        common_step_counter=0,
    )
    env._wg_episode_end = ns["_wg_episode_end"].__get__(env)
    return env


def test_env_episode_end_counts_and_resets():
    env = _env()
    env._wg_episode_end(torch.tensor([0, 1]))
    assert env._wg_state["events"] == 2 and env._wg_state["violations"] == 1
    assert env._wg_state["lambda"] > 1.0
    assert torch.isinf(env._wg_min).all() and torch.equal(env._wg_steps, torch.zeros(2, dtype=env._wg_steps.dtype))


def test_env_episode_end_skips_eval_and_empty_episodes():
    env = _env()
    env.is_evaluating = True
    env._wg_episode_end(torch.tensor([0, 1]))
    assert env._wg_state["events"] == 0 and env._wg_state["lambda"] == 1.0
    env = _env()
    env._wg_steps = torch.tensor([0, 0])  # 始めのリセット（1 歩も進んでいない）は数えない
    env._wg_episode_end(torch.tensor([0, 1]))
    assert env._wg_state["events"] == 0


def test_sources_wire_reward_reset_and_door_metadata():
    src = ENV.read_text(encoding="utf-8")
    assert 'wall_gap_config(self.config.get("wall_gap", None))' in src
    assert "def _reward_wall_gap_lagrange" in src
    reset = ast.unparse(_method(ENV, "_reset_buffers_callback"))
    assert "_wg_episode_end" in reset
    door = DOOR.read_text(encoding="utf-8")
    for key in ('"coverWidth"', '"coverFrontX"', '"wallFrontX"', '"gapWidth"'):
        assert key in door
    for key in ('"coverWidth"', '"coverFrontX"', '"wallFrontX"'):
        assert key in src
