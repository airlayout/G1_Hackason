"""パッチ 0115（11-16・3c の押し方の新案。計画 2026-11-16-g1-door-push-line-two-hands-sim.md）の試験。Isaac Sim は起こさない
（torch だけ）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 体を回す力 Mz: 左の手が押し返されると左回り（正）・力の線が pelvis を通れば 0・当たっていない所（nan）は数えない
- 世界の力を pelvis の向きの座標へ・回ごとのまとめ（効く横のずれ e＝ΣMz÷Σ押し返し）
- cmd_follow の src panel と一歩ごとの揺れの設定・揺れは 0 から始まり、速さは位置の微分と合う
- 柔らかい腕: 3c・4a に入った台だけ硬さを書き、出たら戻す・重力の補償は柔らかい台の腕の列だけ
"""
from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from gr00t.rl.envs.door.panel_push import ROW, ArmComply, summarize_rows, to_body_xy, yaw_moment
from gr00t.rl.envs.door.pelvis_replay import PelvisReplay, pelvis_replay_config

SUBS = ("2a", "2b", "2c", "2d", "3a", "3c", "done", "retreat", "failed", "4a")


def test_left_hand_pushed_back_turns_the_body_left():
    c = torch.zeros(1, 3)
    f = torch.tensor([[[-10.0, 0.0, 0.0]]])        # 手が後ろへ押される
    p = torch.tensor([[[0.35, 0.25, 1.0]]])        # pelvis の左前
    assert yaw_moment(f, p, c).item() == pytest.approx(2.5)  # 0.25 m × 10 N・左回り


def test_force_line_through_the_pelvis_gives_zero_and_two_hands_cancel():
    c = torch.tensor([[1.0, 2.0, 0.8]])
    f = torch.tensor([[[-10.0, 0.0, 0.0]]])
    p = torch.tensor([[[1.4, 2.0, 1.0]]])          # 真正面
    assert yaw_moment(f, p, c).item() == pytest.approx(0.0, abs=1e-6)
    f2 = torch.tensor([[[-10.0, 0.0, 0.0], [-10.0, 0.0, 0.0]]])
    p2 = torch.tensor([[[1.4, 2.25, 1.0], [1.4, 1.75, 1.0]]])  # 左右に 25 cm ずつ
    assert yaw_moment(f2, p2, c).item() == pytest.approx(0.0, abs=1e-5)


def test_tilted_board_force_adds_moment_through_the_forward_arm():
    """扉が θ 開くと板の押し返しは横の成分を持つ（把手の側＝左へ）。手が正面でも r_x × F_y の分だけ左回り"""
    th = math.radians(30.0)
    F = 10.0
    f = torch.tensor([[[-F * math.cos(th), F * math.sin(th), 0.0]]])
    p = torch.tensor([[[0.35, 0.0, 1.0]]])
    assert yaw_moment(f, p, torch.zeros(1, 3)).item() == pytest.approx(0.35 * F * math.sin(th), rel=1e-5)


def test_nan_contact_points_are_not_counted():
    f = torch.tensor([[[-10.0, 0.0, 0.0], [-5.0, 0.0, 0.0]]])
    p = torch.tensor([[[0.3, 0.2, 1.0], [float("nan")] * 3]])
    assert yaw_moment(f, p, torch.zeros(1, 3)).item() == pytest.approx(2.0)


def test_to_body_rotates_by_minus_yaw():
    fb = to_body_xy(torch.tensor([[0.0, -10.0]]), torch.tensor([-math.pi / 2]))  # 右（世界の −y）を向いた体に −y の力＝体の前
    assert fb[0].tolist() == pytest.approx([10.0, 0.0], abs=1e-5)
    fb2 = to_body_xy(torch.tensor([[-10.0, 0.0]]), torch.tensor([0.0]))
    assert fb2[0].tolist() == pytest.approx([-10.0, 0.0])


def test_summary_effective_lateral_offset():
    k = {name: i for i, name in enumerate(ROW)}
    rows = np.zeros((20, len(ROW)))
    rows[:, k["push"]] = 10.0
    rows[:, k["mz"]] = 2.5
    rows[:, k["f_left"]] = 10.0
    rows[:5, k["push"]] = 0.0  # 押していない周期は e に入れない
    rows[:5, k["mz"]] = 0.0
    s = summarize_rows(rows)
    assert s["n"] == 20 and s["e"] == pytest.approx(0.25) and s["push_frac"] == pytest.approx(0.75)
    assert s["right_share"] == pytest.approx(0.0) and summarize_rows(None) == {"n": 0}


def _replay_cfg(tmp_path, **cf):
    f = tmp_path / "r.npz"
    np.savez(f, labels=np.array([1]), t=np.arange(3) * 0.02, d=np.zeros((1, 3, 5)))
    return pelvis_replay_config({"enabled": True, "file": str(f), "release_sub": "3c",
                                 "cmd_follow": {"enabled": True, **cf}})


def test_cmd_follow_accepts_panel_and_oscillation(tmp_path):
    c = _replay_cfg(tmp_path, src="panel", turn_per_nm=0.1, osc_x=0.015, osc_y=0.02, osc_yaw_deg=2.0, osc_hz=1.0)["cmd_follow"]
    assert c["src"] == "panel" and c["turn_per_nm"] == 0.1 and c["osc_hz"] == 1.0
    assert _replay_cfg(tmp_path)["cmd_follow"]["turn_per_nm"] == 0.0  # 既定は今までどおり
    with pytest.raises(AssertionError):
        _replay_cfg(tmp_path, src="lidar")
    with pytest.raises(AssertionError):
        _replay_cfg(tmp_path, osc_x=-0.01)


def test_oscillation_starts_at_zero_and_velocity_is_its_derivative(tmp_path):
    cfg = _replay_cfg(tmp_path, osc_x=0.015, osc_y=0.02, osc_yaw_deg=2.0, osc_hz=1.0)
    r = PelvisReplay.__new__(PelvisReplay)
    r.cmd, r.dev = cfg["cmd_follow"], "cpu"
    r.t_rel0 = torch.tensor([1.0, 1.0])
    r.phase = torch.tensor([4, 3])
    pos0, _ = r._osc(torch.tensor([1.0, 1.0]))
    assert torch.allclose(pos0, torch.zeros(2, 3), atol=1e-7)
    t, h = 1.3, 1e-4
    p1, v1 = r._osc(torch.tensor([t, t]))
    p2, _ = r._osc(torch.tensor([t + h, t + h]))
    assert torch.allclose((p2[0] - p1[0]) / h, v1[0], atol=2e-3)
    assert torch.allclose(p1[1], torch.zeros(3)) and torch.allclose(v1[1], torch.zeros(3))  # 指令に沿わせていない台は 0
    assert p1[0, 0].abs().max() <= 0.015 + 1e-6


class FakeRobot:
    def __init__(self, n, nj):
        self.num_joints = nj
        self.data = SimpleNamespace(joint_stiffness=torch.full((n, nj), 100.0), joint_damping=torch.full((n, nj), 5.0),
                                    gravity_compensation_forces=torch.arange(nj + 6, dtype=torch.float32).repeat(n, 1))
        self.k_writes, self.efforts = [], []

    def write_joint_stiffness_to_sim_index(self, stiffness, joint_ids, env_ids):
        self.k_writes.append((list(joint_ids), env_ids.tolist(), stiffness.clone()))

    def write_joint_damping_to_sim_index(self, damping, joint_ids, env_ids):
        pass

    def set_joint_effort_target_index(self, target, joint_ids):
        self.efforts.append((list(joint_ids), target.clone()))


def test_arm_comply_softens_only_while_pushing_and_restores():
    n, nj = 3, 29
    robot = FakeRobot(n, nj)
    env = SimpleNamespace(simulator=SimpleNamespace(_robot=robot, dof_ids=list(range(nj))), num_envs=n, device="cpu")
    ac = ArmComply(env, {"kp_scale": 0.3}, np.arange(nj), SUBS)
    code = torch.tensor([SUBS.index("3a"), SUBS.index("3c"), SUBS.index("4a")])
    left = torch.tensor([True, True, True])
    right = torch.tensor([False, False, True])
    ac.step(env, code, left, right)
    soft_l = [w for w in robot.k_writes if w[0] == list(range(15, 22))]
    assert soft_l and soft_l[0][1] == [1, 2] and torch.allclose(soft_l[0][2], torch.full((2, 7), 30.0))
    ids, eff = robot.efforts[-1]
    assert ids == list(range(15, 29))
    assert torch.all(eff[0] == 0) and torch.all(eff[1, 7:] == 0)        # 3a の台・右腕を使わない台の右腕は 0
    assert eff[1, 0].item() == pytest.approx(6 + 15)                     # 動く根元の 6 列を飛ばした重力の補償
    assert torch.all(eff[2, 7:] > 0)
    robot.k_writes.clear()
    ac.step(env, torch.tensor([SUBS.index("3c"), SUBS.index("done"), SUBS.index("4a")]), left, right)
    back = [w for w in robot.k_writes if w[0] == list(range(15, 22))]
    # 台 0 は 3a → 3c で柔らかく、台 1 は 3c → done で戻す（1 回の書き込みにまとまる）
    assert back and back[0][1] == [0, 1]
    assert torch.allclose(back[0][2][0], torch.full((7,), 30.0)) and torch.allclose(back[0][2][1], torch.full((7,), 100.0))
    robot.k_writes.clear()
    ac.reset([2])
    assert any(w[1] == [2] and torch.allclose(w[2], torch.full((1, 7), 100.0)) for w in robot.k_writes)
