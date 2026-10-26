"""パッチ 0069（直し B・2・8、10-26）の試験。torch だけ（Isaac Sim は起こさない。GPU も使わない）。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6 && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- B: 学習の環境の腕の積み上げの切り方（DeltaActionBase._guard_arm_accumulator）が、実機の door_core.guard_arm_D と
     同じ並び（積み上げ 3〜16 ＝ 関節 15〜28）・同じ答え（差 1e-6 以下）になる
- 2: 1 制御ステップの目標の変化の上限（FingerPrimitiveBase._limit_target_step）が物理の小ステップの間は同じ目標を使い、
     回の最初の制御ステップには掛からない
- 記録（golden_trace.py）に直し 8・B の設定が書かれる
"""
from __future__ import annotations

import types

import numpy as np
import pytest
import torch
from omegaconf import OmegaConf

from gr00t.rl.envs.base_task import target_guard as tg
from gr00t.rl.envs.base_task.delta_action_base import DeltaActionBase
from gr00t.rl.envs.base_task.finger_primitive_base import FingerPrimitiveBase

N_ENV = 16
NUM_DOF = 29
NUM_LOWER = 15
ACTION_SCALE = 0.25


def _robot(seed: int):
    g = torch.Generator().manual_seed(seed)
    lower = -(1.5 + 1.5 * torch.rand(NUM_DOF, generator=g))
    upper = 1.5 + 1.5 * torch.rand(NUM_DOF, generator=g)
    q_default = 0.6 * torch.rand(NUM_DOF, generator=g) - 0.3
    q = lower + (upper - lower) * torch.rand(N_ENV, NUM_DOF, generator=g)
    D = 30.0 * torch.rand(N_ENV, 17, generator=g) - 15.0
    return lower, upper, q_default, q, D


def _door_core_guard_arm_D(D, q, q_default, lower, upper, delta, margin):
    """実機の door_core.guard_arm_D と同じ並び（numpy・1 台ずつ）"""
    arm = slice(NUM_LOWER, NUM_DOF)
    out = []
    for i in range(D.shape[0]):
        D_arm = tg.guard_arm_accumulator(D[i, 3:], q[i, arm], q_default[arm], lower[arm], upper[arm],
                                         ACTION_SCALE, delta, margin)
        out.append(np.concatenate([D[i, :3], D_arm]))
    return np.stack(out)


def _fake_delta_env(lower, upper, q_default, q, D):
    cfg = OmegaConf.create({"robot": {"lower_body_actions_dim": NUM_LOWER, "control": {"action_scale": ACTION_SCALE}},
                            "arm_accum_guard_delta": 0.3, "arm_accum_guard_margin": 0.05})
    return types.SimpleNamespace(
        _delta_action_indices=torch.arange(17),
        _delta_actions=D.clone(),
        config=cfg,
        simulator=types.SimpleNamespace(hard_dof_pos_limits=torch.stack([lower, upper], dim=1), dof_pos=q),
        default_dof_pos=q_default[None, :].repeat(N_ENV, 1),
    )


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_sim_arm_guard_matches_door_core(seed):
    # Arrange
    lower, upper, qd, q, D = _robot(seed)
    env = _fake_delta_env(lower, upper, qd, q, D.double())
    env.simulator.dof_pos = q.double()
    env.default_dof_pos = env.default_dof_pos.double()
    env.simulator.hard_dof_pos_limits = env.simulator.hard_dof_pos_limits.double()

    # Act
    DeltaActionBase._guard_arm_accumulator(env, 0.3, 0.05)

    # Assert
    want = _door_core_guard_arm_D(D.double().numpy(), q.double().numpy(), qd.double().numpy(),
                                  lower.double().numpy(), upper.double().numpy(), 0.3, 0.05)
    assert np.abs(env._delta_actions.numpy() - want).max() <= 1e-6
    assert torch.equal(env._delta_actions[:, :3], D.double()[:, :3])  # 歩く指令の積み上げは触らない


def test_sim_arm_guard_float32_targets_within_delta():
    lower, upper, qd, q, D = _robot(5)
    env = _fake_delta_env(lower, upper, qd, q, D)
    DeltaActionBase._guard_arm_accumulator(env, 0.3, 0.05)
    target = ACTION_SCALE * env._delta_actions[:, 3:] + qd[NUM_LOWER:]
    arm_q = q[:, NUM_LOWER:]
    lo, hi = lower[NUM_LOWER:] + 0.05, upper[NUM_LOWER:] - 0.05
    inside = (arm_q >= lo + 0.3) & (arm_q <= hi - 0.3)
    assert ((target - arm_q).abs()[inside] <= 0.3 + 1e-5).all()
    assert (target >= lo - 1e-5).all() and (target <= hi + 1e-5).all()


def test_sim_arm_guard_refuses_other_layouts():
    lower, upper, qd, q, D = _robot(6)
    env = _fake_delta_env(lower, upper, qd, q, D)
    env._delta_action_indices = torch.arange(19)  # 指の 2 つが残った並び（43 関節の版）
    with pytest.raises(AssertionError):
        DeltaActionBase._guard_arm_accumulator(env, 0.3, 0.05)


def _fake_finger_env():
    return types.SimpleNamespace(common_step_counter=0, episode_length_buf=torch.zeros(4, dtype=torch.long))


def test_target_step_limit_holds_within_control_step_and_skips_first_step():
    env = _fake_finger_env()
    t0 = torch.zeros(4, NUM_DOF)
    # 回の最初（episode_length_buf == 0）: 前の目標が無いので掛けない
    out = FingerPrimitiveBase._limit_target_step(env, t0, 0.7)
    assert torch.equal(out, t0)
    # 次の制御ステップ: 2 rad 先を指しても 0.7 まで
    env.common_step_counter, env.episode_length_buf = 1, torch.ones(4, dtype=torch.long)
    far = torch.full((4, NUM_DOF), 2.0)
    out = FingerPrimitiveBase._limit_target_step(env, far, 0.7)
    assert torch.allclose(out, torch.full_like(far, 0.7))
    # 同じ制御ステップの物理の小ステップ 2〜4 回目: 入力が変わっても同じ目標（制御ステップの間は 1 つ）
    for _ in range(3):
        again = FingerPrimitiveBase._limit_target_step(env, torch.full((4, NUM_DOF), -3.0), 0.7)
        assert torch.allclose(again, torch.full_like(far, 0.7))
    # 次の制御ステップ: 台 0 だけリセットの直後（episode_length_buf == 0）＝掛けない、ほかは 0.7 からさらに 0.7
    env.common_step_counter = 2
    env.episode_length_buf = torch.tensor([0, 2, 2, 2])
    out = FingerPrimitiveBase._limit_target_step(env, far, 0.7)
    assert torch.allclose(out[0], far[0])
    assert torch.allclose(out[1:], torch.full((3, NUM_DOF), 1.4))


def test_target_step_limit_matches_real_loop_rule():
    """実機（door_run_real.py）: target = last + clip(target − last, ±0.7)"""
    env = _fake_finger_env()
    g = torch.Generator().manual_seed(3)
    last = None
    for k in range(20):
        env.common_step_counter = k
        env.episode_length_buf = torch.full((4,), k, dtype=torch.long)
        target = 4.0 * torch.rand(4, NUM_DOF, generator=g) - 2.0
        out = FingerPrimitiveBase._limit_target_step(env, target, 0.7)
        want = target if last is None else last + (target - last).clamp(-0.7, 0.7)
        assert torch.allclose(out, want)
        last = out


def test_golden_trace_records_fix_settings(tmp_path):
    from gr00t.rl.utils.golden_trace import GoldenTrace

    k = 2
    cfg = OmegaConf.create({"robot": {"dof_names": [f"j{i}" for i in range(NUM_DOF)]},
                            "arm_accum_guard_delta": 0.3, "arm_accum_guard_margin": 0.05})
    z = lambda *s: torch.zeros(*s)  # noqa: E731
    env = types.SimpleNamespace(
        num_envs=k, config=cfg, default_dof_pos=z(k, NUM_DOF),
        simulator=types.SimpleNamespace(dof_pos=z(k, NUM_DOF), dof_vel=z(k, NUM_DOF)),
        base_ang_vel=z(k, 3), projected_gravity=z(k, 3), base_quat=z(k, 4),
        _get_obs_door_angle_lidar=lambda: z(k, 3), _get_obs_door_frame_loc=lambda: z(k, 9),
        _get_obs_door_geometry=lambda: z(k, 7), _get_obs_complete=lambda: z(k, 1),
        _mirror_env=torch.zeros(k, dtype=torch.bool), episode_length_buf=z(k),
        _delta_actions=z(k, 17), _homie_commands=z(k, 7), actions=z(k, NUM_DOF),
    )
    path = tmp_path / "g.npz"
    gt = GoldenTrace(str(path), k, 10, zero_stand_cmd=True)
    obs = {"actor_obs": z(k, 151), "homie_obs": z(k, 516)}
    gt.before_step(env, obs, z(k, 17), z(k, 15), torch.zeros(k, 1, dtype=torch.bool))
    gt.after_step(env, torch.zeros(k))
    gt.save()
    with np.load(path) as g:
        assert float(g["zero_stand_cmd"]) == 1.0
        assert float(g["arm_accum_guard_delta"]) == pytest.approx(0.3)
        assert float(g["arm_accum_guard_margin"]) == pytest.approx(0.05)


def test_sim_arm_guard_per_joint_delta_list():
    """δ を腕 14 関節の一覧で渡せる（関節ごと。V3 の判断材料。実機の door_core は配列をそのまま broadcast する）"""
    lower, upper, qd, q, D = _robot(7)
    env = _fake_delta_env(lower, upper, qd, q, D.double())
    env.simulator.dof_pos = q.double()
    env.default_dof_pos = env.default_dof_pos.double()
    env.simulator.hard_dof_pos_limits = env.simulator.hard_dof_pos_limits.double()
    env.device = "cpu"
    per = [0.2, 0.2, 0.3, 0.3, 0.3, 0.2, 0.2] * 2
    DeltaActionBase._guard_arm_accumulator(env, DeltaActionBase._arm_guard_delta(env, per).double(), 0.05)
    want = _door_core_guard_arm_D(D.double().numpy(), q.double().numpy(), qd.double().numpy(),
                                  lower.double().numpy(), upper.double().numpy(), np.array(per), 0.05)
    assert np.abs(env._delta_actions.numpy() - want).max() <= 1e-6


def test_golden_trace_records_per_joint_delta(tmp_path):
    """10-26: δ が関節ごとの一覧なら 14 個そのまま書く（check_golden が door_core に同じ δ を渡す）"""
    from gr00t.rl.utils.golden_trace import _delta_or_nan

    per = [0.2, 0.2, 0.3, 0.3, 0.3, 0.2, 0.2] * 2
    assert np.array_equal(_delta_or_nan(per), np.array(per))
    assert float(_delta_or_nan(0.3)) == pytest.approx(0.3)
    assert np.isnan(_delta_or_nan(None))
