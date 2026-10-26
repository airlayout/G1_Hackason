"""パッチ 0067（door_carry.py・枠を最初に測って odometry で運ぶ模型）の試験。torch だけ（Isaac Sim は要らない）。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6 && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>
"""
from __future__ import annotations

import math

import pytest
import torch

from gr00t.rl.envs.door.door_carry import CarriedDoorPose

DT = 0.02


def walk(n_steps: int, v_body: tuple[float, float, float], start: tuple[float, float, float]) -> list[torch.Tensor]:
    """体の座標で一定の速さ（前・左・向き）で動く本当の姿勢の列（1 台）"""
    x, y, yaw = start
    out = []
    for _ in range(n_steps):
        out.append(torch.tensor([[x, y, yaw]]))
        vx, vy, w = v_body
        x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * DT
        y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * DT
        yaw += w * DT
    return out


def run(cfg: dict, poses: list[torch.Tensor]) -> list[torch.Tensor]:
    c = CarriedDoorPose(1, "cpu", DT, {"enabled": True, **cfg})
    c.reset(torch.tensor([0]))
    c.step(poses[0], env_ids=torch.tensor([0]))  # リセット直後の観測
    return [c.step(p) for p in poses]


def test_zero_error_reproduces_truth_on_curved_walk():
    # Arrange: 前へ歩きながら曲がる・横にも動く
    poses = walk(400, (0.3, -0.1, 0.4), (-1.0, 0.05, 0.03))

    # Act
    got = run({}, poses)

    # Assert: 誤差 0 なら運んだ値は本当の値と一致する（向きは 1 周を越えるので差を ±π に丸めて比べる）
    diff = torch.cat([g - p for g, p in zip(got, poses)])
    diff[:, 2] = torch.atan2(torch.sin(diff[:, 2]), torch.cos(diff[:, 2]))
    assert diff.abs().max().item() < 1e-5


def test_lateral_scale_shortens_sideways_walk():
    # Arrange: 横（左）へ 1 m、横の倍率 0.9
    poses = walk(101, (0.0, 0.5, 0.0), (-1.0, 0.0, 0.0))

    # Act
    got = run({"scale_lat_mean": 0.9}, poses)

    # Assert: 運んだ値は 0.9 m しか動かない（本当は 1.0 m）
    moved_true = (poses[-1] - poses[0])[0, 1].item()
    moved = (got[-1] - got[0])[0, 1].item()
    assert moved_true == pytest.approx(1.0, abs=1e-4)
    assert moved == pytest.approx(0.9, abs=1e-4)


def test_start_bias_stays_constant_without_motion_error():
    # Arrange: 最初の測定だけ片寄る（σ 5 cm で 1 回引く）。動きの誤差は 0
    poses = walk(200, (0.2, 0.0, 0.3), (-1.0, 0.0, 0.0))
    c = CarriedDoorPose(1, "cpu", DT, {"enabled": True, "start_pos_bias_m": 0.05})
    c.reset(torch.tensor([0]))
    c.step(poses[0], env_ids=torch.tensor([0]))

    # Act
    first = c.step(poses[0])
    last = None
    for p in poses[1:]:
        last = c.step(p)

    # Assert: 片寄りは扉の座標で一定（回っても変わらない）
    b0 = (first - poses[0])[0]
    b1 = (last - poses[-1])[0]
    assert b0[:2].abs().max().item() > 1e-3
    assert torch.allclose(b0, b1, atol=1e-5)


def test_reset_draws_new_errors_and_reinitialises():
    # Arrange: 2 台。片方だけリセットする
    c = CarriedDoorPose(2, "cpu", DT, {"enabled": True, "scale_sigma": 0.1, "start_pos_bias_m": 0.02})
    c.reset(torch.tensor([0, 1]))
    p0 = torch.tensor([[-1.0, 0.0, 0.0], [-1.2, 0.1, 0.2]])
    c.step(p0, env_ids=torch.tensor([0, 1]))
    c.step(p0)
    scale_before = c.scale.clone()

    # Act
    c.reset(torch.tensor([1]))

    # Assert
    assert torch.equal(c.scale[0], scale_before[0])
    assert not torch.equal(c.scale[1], scale_before[1])
    assert bool(c.needs_init[1]) and not bool(c.needs_init[0])


def test_needs_init_reinitialises_on_next_full_step():
    # Arrange: リセット直後の観測では古い姿勢が来て、次の step で正しい姿勢が来る（物理に書き戻される前）
    c = CarriedDoorPose(1, "cpu", DT, {"enabled": True})
    c.reset(torch.tensor([0]))
    stale = torch.tensor([[5.0, 5.0, 1.0]])
    real = torch.tensor([[-1.0, 0.0, 0.0]])
    c.step(stale, env_ids=torch.tensor([0]))

    # Act
    got = c.step(real)

    # Assert: 古い姿勢からの動きを運ばず、正しい姿勢で初め直している
    assert torch.allclose(got, real, atol=1e-6)


def test_disabled_by_default():
    c = CarriedDoorPose(1, "cpu", DT, {})
    assert not c.enabled
