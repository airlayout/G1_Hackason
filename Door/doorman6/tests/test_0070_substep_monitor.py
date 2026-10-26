"""パッチ 0070（直し 3・C、10-26）の試験。torch だけ（Isaac Sim は起こさない。GPU も使わない）。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 物理 1 回ごとに数えるので、制御の 1 ステップ（物理 4 回）の途中だけ張り付いた・当たった山も残る
- 罰の項: 力の張り付きは腕の関節ごとの割合の和、当たりは 1 N を越えた部品の数
"""
from __future__ import annotations

import types

import torch
from omegaconf import OmegaConf

from gr00t.rl.envs.base_task.substep_monitor import SubstepMonitor
from gr00t.rl.envs.legged_base_task.legged_robot_base import LeggedRobotBase

NAMES = [f"left_hip_pitch_joint{i}" for i in range(15)] + [
    f"{s}_{j}_joint" for s in ("left", "right") for j in
    ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
]
LIMIT = torch.tensor([88.0] * 15 + [25.0] * 5 + [5.0] * 2 + [25.0] * 5 + [5.0] * 2)


def test_monitor_counts_saturation_per_substep_and_keeps_contact_peak():
    mon = SubstepMonitor(2, 14, 3, "cpu")
    mon.begin()
    lim = LIMIT[15:]
    for k in range(4):
        tau = torch.zeros(2, 14)
        if k == 1:
            tau[0, 0] = 23.0  # 25 の 9 割（22.5）以上は 4 回のうち 1 回だけ
        contact = torch.zeros(2, 3)
        if k == 2:
            contact[1, 2] = 7.5  # 3 回目だけ当たった
        mon.after_substep(tau, lim, contact)
    frac = mon.saturated_fraction()
    assert torch.isclose(frac[0, 0], torch.tensor(0.25)) and frac.sum() == 0.25
    assert mon.contact_peak[1, 2] == 7.5 and mon.contact_peak.sum() == 7.5
    mon.begin()
    assert mon.saturated_fraction().sum() == 0 and mon.contact_peak.sum() == 0


def test_monitor_without_contact_sensor():
    mon = SubstepMonitor(2, 14, 0, "cpu")
    mon.begin()
    mon.after_substep(torch.full((2, 14), 30.0), LIMIT[15:], None)
    assert (mon.saturated_fraction() == 1).all()


def _fake_env(contact_names=("left_elbow_link", "right_elbow_link")):
    sim = types.SimpleNamespace(
        arm_body_contact_names=list(contact_names),
        torque_limits=LIMIT,
        applied_dof_torque=torch.zeros(3, 29),
        arm_body_contact_force=lambda: torch.zeros(3, len(contact_names)),
    )
    cfg = OmegaConf.create({"substep_monitor": True, "robot": {"dof_names": NAMES}})
    return types.SimpleNamespace(config=cfg, simulator=sim, num_envs=3, device="cpu", log_dict={})


def test_env_hooks_and_rewards():
    env = _fake_env()
    LeggedRobotBase._substep_monitor_begin(env)
    assert env._monitor_arm_idx.tolist() == list(range(15, 29))
    for k in range(4):
        tau = torch.zeros(3, 29)
        tau[0, 15] = -24.0  # 左肩 pitch が 4 回とも張り付き
        tau[1, 20] = 4.6 if k < 2 else 0.0  # 左手首 pitch（上限 5）が 2 回
        env.simulator.applied_dof_torque = tau
        hit = torch.zeros(3, 2)
        hit[2, 1] = 3.0 if k == 3 else 0.0
        env.simulator.arm_body_contact_force = lambda h=hit: h
        LeggedRobotBase._after_physics_substep(env)
    sat = LeggedRobotBase._reward_penalty_arm_torque_saturation(env)
    assert torch.allclose(sat, torch.tensor([1.0, 0.5, 0.0]))
    con = LeggedRobotBase._reward_penalty_arm_body_contact(env)
    assert torch.equal(con, torch.tensor([0.0, 0.0, 1.0]))
    assert "arm_torque_sat_frac" in env.log_dict and "arm_body_contact_frac" in env.log_dict


def test_monitor_off_returns_zero_rewards():
    env = _fake_env()
    env.config = OmegaConf.create({"robot": {"dof_names": NAMES}})
    LeggedRobotBase._substep_monitor_begin(env)
    LeggedRobotBase._after_physics_substep(env)
    assert torch.equal(LeggedRobotBase._reward_penalty_arm_torque_saturation(env), torch.zeros(3))
    assert torch.equal(LeggedRobotBase._reward_penalty_arm_body_contact(env), torch.zeros(3))
