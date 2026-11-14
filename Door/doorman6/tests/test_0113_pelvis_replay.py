"""パッチ 0113・0114（11-14・実機の pelvis の動きの再生 env.config.pelvis_replay・押されて下がる脚 env.config.loco_push。
計画 2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 D・G）の試験。Isaac Sim は起こさない（torch だけ）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 設定が無い・enabled=False なら None（元のまま）。誤りは止める
- 四元数と (yaw, roll, pitch) の行き来・今の向きから目標への回転ベクトル
- 方法 door の扉の置き方: 止めた pelvis を扉の座標で見ると、止めた姿勢＋記録の差に見える（平面の剛体変換）
- 記録の差の読み方（線形の内挿・途中から読む offset_s・再生していない台は 0・記録の終わりより後は最後の値で速さ 0）
- 押されて下がる脚: 歩いていて押し返しが線を越えた台だけ、t_back 秒 前後の指令を後ろ向きに置き換える・歩き出しに横揺れを足す
"""
from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from gr00t.rl.envs.door.loco_push import LocoPush, loco_push_config
from gr00t.rl.envs.door.pelvis_replay import (
    PelvisReplay,
    euler_from_quat_xyzw,
    pelvis_replay_config,
    quat_xyzw_from_euler,
    rotvec_to_target,
)


def test_configs_are_none_when_absent_or_disabled_and_reject_bad_values(tmp_path):
    assert pelvis_replay_config(None) is None and pelvis_replay_config({"enabled": False}) is None
    assert loco_push_config(None) is None and loco_push_config({"enabled": False}) is None
    f = tmp_path / "r.npz"
    np.savez(f, labels=np.array([1]), t=np.arange(3) * 0.02, d=np.zeros((1, 3, 5)))
    c = pelvis_replay_config({"enabled": True, "file": str(f)})
    assert c["method"] == "vel" and c["hold_from"] == 1 and c["start_sub"] == 4 and c["offset_s"] == 0.0
    with pytest.raises(AssertionError):
        pelvis_replay_config({"enabled": True, "file": str(f), "method": "teleport"})
    with pytest.raises(AssertionError):
        pelvis_replay_config({"enabled": True, "file": str(f), "hold_from": "3a", "start_sub": "2b"})
    with pytest.raises(AssertionError):
        loco_push_config({"enabled": True, "v_back": 0.2})


def test_euler_and_quaternion_round_trip_and_rotvec():
    yaw, roll, pitch = torch.tensor([0.3, -1.0]), torch.tensor([0.05, -0.02]), torch.tensor([-0.04, 0.1])
    q = quat_xyzw_from_euler(yaw, roll, pitch)
    y2, r2, p2 = euler_from_quat_xyzw(q)
    assert torch.allclose(y2, yaw, atol=1e-5) and torch.allclose(r2, roll, atol=1e-5) and torch.allclose(p2, pitch, atol=1e-5)
    q0 = quat_xyzw_from_euler(torch.tensor([0.1]), torch.tensor([0.0]), torch.tensor([0.0]))
    q1 = quat_xyzw_from_euler(torch.tensor([0.4]), torch.tensor([0.0]), torch.tensor([0.0]))
    rv = rotvec_to_target(q0, q1)
    assert torch.allclose(rv, torch.tensor([[0.0, 0.0, 0.3]]), atol=1e-5)
    assert torch.allclose(rotvec_to_target(q1, q1), torch.zeros(1, 3), atol=1e-6)


class FakeArt:
    def __init__(self, pose):
        self.data = SimpleNamespace(root_link_pose_w=pose)
        self.written = []

    def write_root_link_velocity_to_sim_index(self, root_velocity, env_ids):
        self.written.append(("vel", root_velocity.clone(), env_ids.clone()))

    def write_root_link_pose_to_sim_index(self, root_pose, env_ids):
        self.written.append(("pose", root_pose.clone(), env_ids.clone()))
        self.data.root_link_pose_w[env_ids] = root_pose


def make_env(n=2, file=None, method="vel", offset=0.0, release=None):
    robot = FakeArt(torch.tensor([[-0.4, 0.08, 0.75, 0.0, 0.0, 0.0, 1.0]] * n))
    door = FakeArt(torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]] * n))
    sim = SimpleNamespace(_robot=robot, scene=SimpleNamespace(articulations={"door": door}))
    env = SimpleNamespace(device="cpu", num_envs=n, dt=0.02, simulator=sim, env_origins=torch.zeros(n, 3),
                          config=SimpleNamespace(simulator=SimpleNamespace(config=SimpleNamespace(sim=SimpleNamespace(control_decimation=4)))),
                          episode_length_buf=torch.zeros(n, dtype=torch.long),
                          _script_teacher=SimpleNamespace(sub_code=torch.full((n,), -1, dtype=torch.long)))
    cfg = {"file": file, "method": method, "hold_from": 0, "start_sub": 4, "kp": 20.0, "kp_rot": 20.0, "labels": [], "offset_s": offset,
           "release_sub": release}
    return env, PelvisReplay(env, cfg)


def write_traj(tmp_path, d):
    f = tmp_path / "r.npz"
    np.savez(f, labels=np.arange(1, d.shape[0] + 1), t=np.arange(d.shape[1]) * 0.02, d=d)
    return str(f)


def test_delta_interpolates_only_while_replaying_and_holds_the_end(tmp_path):
    d = np.zeros((1, 11, 5))
    d[0, :, 1] = np.linspace(0.0, -0.10, 11)  # 0.2 秒で横へ −10 cm
    env, rep = make_env(file=write_traj(tmp_path, d))
    rep.phase[:] = torch.tensor([2, 1])
    rep.t0[:] = 1.0
    dd, v = rep._delta(torch.tensor([1.05, 1.05]))
    assert dd[0, 1].item() == pytest.approx(-0.025, abs=1e-5) and v[0, 1].item() == pytest.approx(-0.5, abs=1e-4)
    assert torch.all(dd[1] == 0) and torch.all(v[1] == 0)  # 止めているだけの台
    dd, v = rep._delta(torch.tensor([5.0, 5.0]))
    assert dd[0, 1].item() == pytest.approx(-0.10, abs=1e-6) and v[0, 1].item() == 0.0


def test_offset_starts_from_the_middle_of_the_record(tmp_path):
    d = np.zeros((1, 11, 5))
    d[0, :, 0] = np.linspace(0.0, 0.10, 11)
    env, rep = make_env(file=write_traj(tmp_path, d), offset=0.1)
    rep.phase[:] = 2
    rep.t0[:] = 0.0
    dd, _ = rep._delta(torch.tensor([0.0, 0.05]))
    assert dd[0, 0].item() == pytest.approx(0.0, abs=1e-6) and dd[1, 0].item() == pytest.approx(0.025, abs=1e-5)


def test_control_step_holds_then_starts_and_vel_method_writes_velocities(tmp_path):
    d = np.zeros((1, 11, 5))
    env, rep = make_env(file=write_traj(tmp_path, d))
    env._script_teacher.sub_code[:] = torch.tensor([1, -1])  # 台 0 は 2b（止める）・台 1 はまだ
    rep.control_step(env)
    assert rep.phase.tolist() == [1, 0]
    assert rep.p0[0, 0].item() == pytest.approx(-0.4) and rep.p0[0, 2].item() == pytest.approx(0.75)
    env._script_teacher.sub_code[:] = torch.tensor([4, 4])  # 3a（台 1 は同じ周期に止めて再生を始める＝その時の姿勢が基準）
    rep.control_step(env)
    assert rep.phase.tolist() == [2, 2]
    rep.physics_substep(env)
    kind, vel, ids = env.simulator._robot.written[-1]
    assert kind == "vel" and ids.tolist() == [0, 1] and torch.allclose(vel, torch.zeros(2, 6), atol=1e-6)
    env._script_teacher.sub_code[:] = torch.tensor([8, 8])  # failed でも再生は続ける（体の動きは台本と関わらない）
    rep.control_step(env)
    assert rep.phase.tolist() == [2, 2]
    rep.reset(env, [0])
    assert rep.phase.tolist() == [0, 2]


def test_door_method_places_the_door_so_the_held_pelvis_reads_as_the_record(tmp_path):
    d = np.zeros((1, 11, 5))
    d[0, :, 0], d[0, :, 1], d[0, :, 2] = 0.05, -0.12, math.radians(4.0)
    env, rep = make_env(file=write_traj(tmp_path, d), method="door")
    rep.phase[:] = 2
    rep.p0[:, 0], rep.p0[:, 1], rep.p0[:, 3] = -0.4, 0.08, math.radians(-5.0)
    rep.t0[:] = 0.0
    rep.t_ctrl[:] = 1.0
    rep.physics_substep(env)
    door = env.simulator.scene.articulations["door"].data.root_link_pose_w[0]
    dyaw = euler_from_quat_xyzw(door[None, 3:7])[0].item()
    # 止めた pelvis（世界 = 台の原点）を扉の根元から見た姿勢
    px, py = -0.4 - door[0].item(), 0.08 - door[1].item()
    c, s = math.cos(dyaw), math.sin(dyaw)
    x_d, y_d = c * px + s * py, -s * px + c * py
    assert x_d == pytest.approx(-0.4 + 0.05, abs=1e-5) and y_d == pytest.approx(0.08 - 0.12, abs=1e-5)
    assert math.radians(-5.0) - dyaw == pytest.approx(math.radians(-5.0 + 4.0), abs=1e-5)


def make_push_env(n=3):
    env = SimpleNamespace(device="cpu", num_envs=n, dt=0.02, _stock_legs_on=True,
                          config=SimpleNamespace(homie_command_scale=0.25),
                          episode_length_buf=torch.zeros(n, dtype=torch.long),
                          _delta_actions=torch.zeros(n, 17), _homie_commands_legs=torch.zeros(n, 4),
                          _script_teacher=SimpleNamespace(hand_f_t=torch.zeros(n, 3), hand_c_t=torch.zeros(n, 3)))
    return env


def test_push_back_only_for_walking_envs_over_the_line_then_sway_on_onset():
    env = make_push_env()
    cfg = loco_push_config({"enabled": True, "f_th": 30.0, "v_back": -0.3, "t_back": 0.6, "sway_vy": -0.25, "sway_wz": 0.1})
    lp = LocoPush(env, cfg)
    sc = 0.25
    env._delta_actions[:, 0] = torch.tensor([0.2, 0.2, 0.0]) / 2.0 / sc  # 台 0・1 は前へ 0.2 m/s で歩く・台 2 は立つ
    env._script_teacher.hand_f_t[:, 0] = torch.tensor([-35.0, -20.0, -50.0])  # 押し返し 35・20・50 N
    lp.control_step(env)
    vx = env._delta_actions[:, 0] * sc * 2.0
    assert vx.tolist() == pytest.approx([-0.3, 0.2, 0.0])  # 線を越えて歩いている台 0 だけ下がる
    assert lp.n_back.tolist() == [1, 0, 0] and lp.n_sway.tolist() == [1, 1, 0]  # 歩き出しは台 0・1
    lp.apply_legs(env)
    assert env._homie_commands_legs[0, 1].item() == pytest.approx(-0.125) and env._homie_commands_legs[2, 1].item() == 0.0
    env.episode_length_buf[:] = 40  # 0.8 秒後: 下がり終え、横揺れも終わる
    env._delta_actions[:, 0] = torch.tensor([0.2, 0.2, 0.0]) / 2.0 / sc
    env._script_teacher.hand_f_t[:, 0] = 0.0
    env._homie_commands_legs[:] = 0.0
    lp.control_step(env)
    lp.apply_legs(env)
    assert (env._delta_actions[:, 0] * sc * 2.0).tolist() == pytest.approx([0.2, 0.2, 0.0])
    assert torch.all(env._homie_commands_legs == 0)


def test_release_sub_hands_the_pelvis_to_the_legs_at_3c(tmp_path):
    d = np.zeros((1, 11, 5))
    env, rep = make_env(file=write_traj(tmp_path, d), release=5)  # 3c で放す
    env._script_teacher.sub_code[:] = torch.tensor([4, 4])
    rep.control_step(env)
    assert rep.phase.tolist() == [2, 2]
    env._script_teacher.sub_code[:] = torch.tensor([5, 4])  # 台 0 だけ 3c に入った
    rep.control_step(env)
    assert rep.phase.tolist() == [3, 2]
    n_before = len(env.simulator._robot.written)
    rep.physics_substep(env)
    kind, vel, ids = env.simulator._robot.written[-1]
    assert len(env.simulator._robot.written) == n_before + 1 and ids.tolist() == [1]  # 放した台 0 には書かない
    env._script_teacher.sub_code[:] = torch.tensor([9, 9])  # 4a（番号は最後）でも放したまま・台 1 も放す
    rep.control_step(env)
    assert rep.phase.tolist() == [3, 3]
    with pytest.raises(AssertionError):
        pelvis_replay_config({"enabled": True, "file": write_traj(tmp_path, d), "start_sub": "3a", "release_sub": "2d"})


def test_cmd_follow_moves_the_target_with_the_stock_like_response_and_backs_off_when_pushed(tmp_path):
    d = np.zeros((1, 11, 5))
    d[0, :, 1] = np.linspace(0.0, -0.10, 11)  # 揺れの記録（横 −10 cm）
    f = write_traj(tmp_path, d)
    env, rep = make_env(file=f, release=5)
    rep.cfg["cmd_follow"] = None
    cfg = pelvis_replay_config({"enabled": True, "file": f, "start_sub": "3a", "release_sub": "3c",
                                "cmd_follow": {"enabled": True, "gain": 0.5, "delay_s": 0.04, "dead": 0.1, "f_th": 30.0,
                                               "back_v": -0.2, "back_s": 0.1, "sway_label": 1, "sway_s": 0.2}})
    cfg["hold_from"] = 0
    env.config.homie_command_scale = 0.25
    env.door_open_lr = torch.ones(2)
    env._delta_actions = torch.zeros(2, 17)
    env._script_teacher.hand_f_t = torch.zeros(2, 3)
    rep = PelvisReplay(env, cfg)
    env._script_teacher.sub_code[:] = torch.tensor([4, 4])
    rep.control_step(env)
    env._script_teacher.sub_code[:] = torch.tensor([5, 4])  # 台 0 が 3c へ: 指令に沿わせる
    env._delta_actions[:, 0] = 0.2 / 2.0 / 0.25  # 前へ 0.2 m/s
    rep.control_step(env)
    assert rep.phase.tolist() == [4, 2]
    for _ in range(3):  # 遅れ 2 歩の後に 0.2 × 0.5 = 0.1 m/s
        rep.control_step(env)
    assert rep.vtgt[0, 0].item() == pytest.approx(0.1, abs=1e-6) and rep.vtgt[1, 0].item() == 0.0
    env._script_teacher.hand_f_t[0, 0] = -35.0  # 押し返し 35 N
    rep.control_step(env)
    assert rep.vtgt[0, 0].item() == pytest.approx(-0.2) and rep.n_back[0].item() == 1
    sw, _ = rep._sway(rep.t_rel0 + 0.1)
    assert sw[0, 1].item() == pytest.approx(-0.05, abs=1e-5) and sw[0, 0].item() == 0.0
    rep.physics_substep(env)
    kind, vel, ids = env.simulator._robot.written[-1]
    assert ids.tolist() == [0, 1]  # 指令に沿わせる台も書く


def test_cmd_follow_push_slows_forward_and_turns_toward_the_lever(tmp_path):
    d = np.zeros((1, 11, 5))
    f = write_traj(tmp_path, d)
    env, _ = make_env(file=f, release=5)
    cfg = pelvis_replay_config({"enabled": True, "file": f, "start_sub": "3a", "release_sub": "3c",
                                "cmd_follow": {"enabled": True, "gain": 0.6, "delay_s": 0.0, "dead": 0.1, "f_th": 30.0,
                                               "fwd_per_n": 0.01, "turn_per_n": 0.014}})
    cfg["hold_from"] = 0
    env.config.homie_command_scale = 0.25
    env.door_open_lr = torch.tensor([1.0, -1.0])
    env._delta_actions = torch.zeros(2, 17)
    env._script_teacher.hand_f_t = torch.zeros(2, 3)
    rep = PelvisReplay(env, cfg)
    env._script_teacher.sub_code[:] = 4
    rep.control_step(env)
    env._script_teacher.sub_code[:] = 5
    env._delta_actions[:, 0] = 0.2 / 2.0 / 0.25
    env._script_teacher.hand_f_t[:, 0] = -8.0  # 8 N 押している（線 30 N の下）
    rep.control_step(env)
    assert rep.vtgt[0, 0].item() == pytest.approx(0.12 - 0.08, abs=1e-5)
    assert rep.vtgt[0, 2].item() == pytest.approx(0.014 * 8.0, abs=1e-5) and rep.vtgt[1, 2].item() == pytest.approx(-0.014 * 8.0, abs=1e-5)


def test_cmd_follow_can_use_the_contact_sensor_instead_of_the_estimate(tmp_path):
    d = np.zeros((1, 11, 5))
    f = write_traj(tmp_path, d)
    env, _ = make_env(file=f, release=5)
    cfg = pelvis_replay_config({"enabled": True, "file": f, "start_sub": "3a", "release_sub": "3c",
                                "cmd_follow": {"enabled": True, "gain": 0.6, "delay_s": 0.0, "dead": 0.1, "f_th": 30.0,
                                               "fwd_per_n": 0.01, "turn_per_n": 0.014, "src": "contact"}})
    cfg["hold_from"] = 0
    env.config.homie_command_scale = 0.25
    env.door_open_lr = torch.ones(2)
    env._delta_actions = torch.zeros(2, 17)
    env._script_teacher.hand_f_t = torch.full((2, 3), -10.0)  # 見積もりは 10 N の偽の押し返し
    env._script_teacher.hand_c_t = torch.zeros(2, 3)          # 本当は触れていない
    rep = PelvisReplay(env, cfg)
    env._script_teacher.sub_code[:] = 4
    rep.control_step(env)
    env._script_teacher.sub_code[:] = 5
    env._delta_actions[:, 0] = 0.2 / 2.0 / 0.25
    rep.control_step(env)
    assert rep.vtgt[0, 0].item() == pytest.approx(0.12, abs=1e-5) and rep.vtgt[0, 2].item() == pytest.approx(0.0, abs=1e-6)
