"""パッチ 0097（11-09・段 2 レバーを腕だけで学ぶ口 env.config.arm_stage）の試験。
Isaac Sim は起こさない（arm_stage.py は torch だけ。扉の環境は関数を取り出して偽の環境で動かす）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>
count_success.py の試験は、OMEN の ~/work_inoue/doorman に置いた count_success.py を COUNT_SUCCESS で渡す（既定はその場所）

- 設定が無い・enabled=False なら元のまま（arm_stage_config が None）。範囲・段の組の誤りは止める
- レバーの付け根の y は door.py と同じ (幅/2 − バックセット) × 左右
- 構えの位置: 前は扉の面から、横はレバーの付け根から（レバーの側が正）、向きは度。把手が右の扉（左右 −1）は横と向きを映す
- 段 0 から始める回を構える段 1 から始める（途中の段から始める回はそのまま）。段 4 以上で完了
- 扉の環境: 腕だけの段なら、掛け金は毎歩すべての台・完了は段だけで決める・回の始めは構えの位置に立たせて胴の速さを小さくする
- count_success.py の arm10: 区間 1 から始めた回を数え、区間 4 に入った回を成功にする
"""
from __future__ import annotations

import ast
import importlib.util
import math
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from gr00t.rl.envs.door.arm_stage import (
    DEFAULT_START_RANGE,
    PRESS_DEFAULTS,
    arm_stage_complete,
    arm_stage_config,
    lever_pivot_y,
    palm_press,
    sample_stance,
    start_stages,
    update_legs_free,
    update_press_count,
)

ENV = Path("gr00t/rl/envs/door/door_open_homie.py")
COUNT_SUCCESS = Path(os.environ.get("COUNT_SUCCESS", os.path.expanduser("~/work_inoue/doorman/count_success.py")))


def _method(name: str):
    tree = ast.parse(ENV.read_text(encoding="utf-8"))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    fn.decorator_list = []
    return fn


def _load(names: list[str], extra: dict):
    ns = {"torch": torch, **extra}
    exec(compile(ast.Module(body=[_method(n) for n in names], type_ignores=[]), str(ENV), "exec"), ns)  # noqa: S102
    return ns


def test_config_off_by_default_and_checked():
    assert arm_stage_config(None) is None
    assert arm_stage_config({"enabled": False, "start_range": [0, 1, 0, 1, 0, 1]}) is None
    cfg = arm_stage_config({"enabled": True})
    assert cfg == {"start_range": DEFAULT_START_RANGE, "start_stage": 1, "complete_stage": 4, "start_vel": 0.1,
                   "complete_door_deg": None, "free_legs_when_unlatched": False, "release": "lever_true",
                   **{k: float(v) for k, v in PRESS_DEFAULTS.items()}, "handover_door_deg": None, "complete_through": False}
    with pytest.raises(AssertionError):
        arm_stage_config({"enabled": True, "release": "torque"})  # 0100: 引き金は 2 つだけ
    assert arm_stage_config({"enabled": True, "complete_door_deg": 3})["complete_door_deg"] == 3.0  # 0098
    with pytest.raises(AssertionError):
        arm_stage_config({"enabled": True, "complete_door_deg": 0.0})
    with pytest.raises(AssertionError):
        arm_stage_config({"enabled": True, "start_range": [-0.3, -0.4, 0, 0, 0, 0]})  # x0 > x1
    with pytest.raises(AssertionError):
        arm_stage_config({"enabled": True, "start_stage": 4, "complete_stage": 4})


def test_lever_pivot_y_matches_door_generator():
    # door.py: door_handle の置き場所 (0, (half_door_width - door_handle_width) * door_open_lr, 高さ)
    w, bs = torch.tensor([0.94, 0.94]), torch.tensor([0.064, 0.064])
    y = lever_pivot_y(w, bs, torch.tensor([1.0, -1.0]))
    assert torch.allclose(y, torch.tensor([0.406, -0.406]))


def test_sample_stance_corners_and_mirror():
    rng = (-0.48, -0.32, -0.15, -0.01, -65.0, -25.0)
    pivot = torch.tensor([0.41, -0.41])
    lr = torch.tensor([1.0, -1.0])
    x, y, yaw = sample_stance(pivot, lr, rng, torch.zeros(2, 3))
    assert torch.allclose(x, torch.tensor([-0.48, -0.48]))
    assert torch.allclose(y, torch.tensor([0.41 - 0.15, -0.41 + 0.15]))  # レバーの側の反対へ 15 cm（映すと左右が逆）
    assert torch.allclose(yaw, torch.tensor([math.radians(-65.0), math.radians(65.0)]))
    x, y, yaw = sample_stance(pivot, lr, rng, torch.ones(2, 3))
    assert torch.allclose(x, torch.tensor([-0.32, -0.32]))
    assert torch.allclose(y, torch.tensor([0.41 - 0.01, -0.41 + 0.01]))
    assert torch.allclose(yaw, torch.tensor([math.radians(-25.0), math.radians(25.0)]))


def test_start_stages_and_complete():
    assert start_stages(torch.tensor([0, 2, 3, 0]), 1).tolist() == [1, 2, 3, 1]
    assert arm_stage_complete(torch.tensor([1, 3, 4, 5]), 4).tolist() == [False, False, True, True]
    # 0098: 扉の角度で（段は見ない）。開く向きは問わない
    hinge = torch.deg2rad(torch.tensor([0.1, 2.9, 3.0, -3.5]))
    assert arm_stage_complete(torch.tensor([3, 3, 1, 2]), 4, hinge, 3.0).tolist() == [False, False, True, True]


class _Env:
    pass


def test_env_latch_all_envs_when_arm_stage():
    ns = _load(["_stand_latch_mask", "_arm_stage_legs_mask"],
               {"update_stand_latch": None, "update_legs_free": update_legs_free})
    env = _Env()
    env.config = {"stand_latch": False}
    env._arm_stage = {"complete_stage": 4, "free_legs_when_unlatched": False}
    env._arm_stage_latched = torch.ones(3, dtype=torch.bool)
    env._arm_stage_legs_mask = lambda arm: ns["_arm_stage_legs_mask"](env, arm)
    assert ns["_stand_latch_mask"](env) is env._arm_stage_latched
    env._arm_stage = None
    assert ns["_stand_latch_mask"](env) is None  # 元のまま（stand_latch も切なら掛けない）


def test_env_frees_legs_once_unlatched():
    """0099: 掛け金が外れた台は放し、レバーが戻っても回の終わりまで放したまま"""
    ns = _load(["_arm_stage_legs_mask"], {"update_legs_free": update_legs_free})
    env = _Env()
    env._arm_stage_latched = torch.ones(3, dtype=torch.bool)
    env._arm_legs_free = torch.zeros(3, dtype=torch.bool)
    env.log_dict = {}
    arm = {"free_legs_when_unlatched": True, "release": "lever_true"}
    unlatched = torch.tensor([False, True, False])
    env._lever_unlatched = lambda: unlatched
    assert ns["_arm_stage_legs_mask"](env, arm).tolist() == [True, False, True]
    unlatched = torch.tensor([False, False, False])  # レバーが戻った
    assert ns["_arm_stage_legs_mask"](env, arm).tolist() == [True, False, True]
    assert float(env.log_dict["arm_legs_free_frac"]) == pytest.approx(1 / 3)
    assert update_legs_free(torch.tensor([True, False]), torch.tensor([False, False])).tolist() == [True, False]


def test_env_complete_by_stage_only_when_arm_stage():
    ns = _load(["_stage_5_to_complete_condition"], {"arm_stage_complete": arm_stage_complete})
    env = _Env()
    env._arm_stage = {"complete_stage": 4, "complete_door_deg": None, "complete_through": False}
    env.stage_buf = torch.tensor([3, 4, 5])

    class _Arr:
        torch = torch.deg2rad(torch.tensor([[5.0, 0, 0], [0.0, 0, 0], [0.0, 0, 0]]))

    door = SimpleNamespace(data=SimpleNamespace(joint_pos=_Arr()))
    env.simulator = SimpleNamespace(scene=SimpleNamespace(articulations={"door": door}))
    assert ns["_stage_5_to_complete_condition"](env).tolist() == [False, True, True]
    env._arm_stage["complete_door_deg"] = 3.0  # 0098: 扉の角度で（台 0 は段 3 でも 5° 開いているので完了）
    assert ns["_stage_5_to_complete_condition"](env).tolist() == [True, False, False]


def test_env_reset_root_states_places_stance():
    def quat(r, p, y):  # 偽の四元数: 渡された roll・pitch・yaw を並べて返す
        return torch.stack([r, p, y, torch.zeros_like(y)], dim=-1)

    ns = _load(["_reset_root_states_arm_stage"], {"sample_stance": sample_stance, "quat_from_euler_xyz": quat})
    env = _Env()
    env.device = "cpu"
    n = 4
    env.target_robot_root_states = torch.zeros(n, 13)
    env.env_origins = torch.tensor([[10.0, 20.0, 0.0]] * n)
    env._lever_pivot_y = torch.full((n,), 0.41)
    env.door_open_lr = torch.ones(n)
    arm = {"start_range": (-0.48, -0.32, -0.15, -0.01, -65.0, -25.0), "start_vel": 0.1}
    ids = torch.tensor([1, 3])
    torch.manual_seed(0)
    ns["_reset_root_states_arm_stage"](env, ids, torch.zeros(2), torch.zeros(2), arm)
    s = env.target_robot_root_states
    assert ((s[ids, 0] >= 10 - 0.48) & (s[ids, 0] <= 10 - 0.32)).all()
    assert ((s[ids, 1] >= 20 + 0.41 - 0.15) & (s[ids, 1] <= 20 + 0.41 - 0.01)).all()
    assert ((s[ids, 5] >= math.radians(-65)) & (s[ids, 5] <= math.radians(-25))).all()
    assert (s[ids, 7:13].abs() <= 0.1).all() and (s[ids, 7:13].abs() > 0).any()
    assert (s[[0, 2]] == 0).all()  # 戻さない台はそのまま


def test_env_wiring():
    """呼ぶ順: _reset_root_states の最後に構えの位置へ・set_to_stage は段を替えてから元へ"""
    reset = [ast.unparse(s) for s in _method("_reset_root_states").body]
    assert reset[-2] == "arm = getattr(self, '_arm_stage', None)"
    assert "self._reset_root_states_arm_stage(env_ids, r, p, arm)" in reset[-1]
    stage = [ast.unparse(s) for s in _method("set_to_stage").body]
    assert "stage = start_stages(stage, arm['start_stage'])" in stage[1]
    assert stage[-1] == "super().set_to_stage(env_ids, stage)"


def _count_success():
    if not COUNT_SUCCESS.exists():
        pytest.skip(f"count_success.py が無い: {COUNT_SUCCESS}")
    spec = importlib.util.spec_from_file_location("count_success", COUNT_SUCCESS)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_count_success_arm10_counts_from_stage1():
    cs = _count_success()
    T, N = 6, 3
    stage = np.array([[1, 1, 0], [2, 1, 0], [3, 2, 0], [4, 3, 1], [4, 3, 1], [1, 1, 1]], dtype=np.float32)
    ep = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2], [3, 3, 3], [4, 4, 4], [0, 0, 5]], dtype=np.float32)
    rec = {"stage": stage, "episode_length": ep, "root_pos": np.zeros((T, N, 3)), "door_dof_pos": np.zeros((T, N, 3))}
    s = cs.summarize(cs.split_episodes(rec), "arm10")
    assert s["start_stage"] == 1
    assert s["episodes_from_stage0"] == 2  # 台 2 は区間 0 から始まり、終わってもいないので数えない
    assert s["success"] == 1 and s["success_rate"] == 0.5
    assert cs.summarize(cs.split_episodes(rec), "through")["start_stage"] == 0


def test_count_success_arm3_counts_door_angle():
    """0098: arm3 は区間 1 から始めた回を数え、扉が 3° 以上開いた回を成功にする"""
    cs = _count_success()
    stage = np.array([[1, 1], [2, 2], [3, 3], [3, 3], [1, 1]], dtype=np.float32)
    ep = np.array([[0, 0], [1, 1], [2, 2], [3, 3], [0, 0]], dtype=np.float32)
    door = np.zeros((5, 2, 3))
    door[3, 0, 0] = np.deg2rad(3.2)
    door[3, 1, 0] = np.deg2rad(2.5)
    rec = {"stage": stage, "episode_length": ep, "root_pos": np.zeros((5, 2, 3)), "door_dof_pos": door}
    s = cs.summarize(cs.split_episodes(rec), "arm3")
    assert s["start_stage"] == 1 and s["episodes_from_stage0"] == 2 and s["success"] == 1


def test_palm_press_and_hold():
    """0100: 水平 R 以内・当てる点より DROP 以上低い・力が下限以上。どれか欠けたら立たない。続いた歩を数え、途切れたら 0"""
    ref = torch.tensor([[0.0, 0.3, 1.0]] * 5)
    palm = torch.tensor([[0.05, 0.3, 0.97],   # 立つ（水平 5 cm・3 cm 低い）
                         [0.12, 0.3, 0.97],   # 水平が遠い
                         [0.05, 0.3, 0.99],   # 下がりが足りない
                         [0.05, 0.3, 0.97],   # 力が足りない
                         [0.05, 0.3, 0.97]])  # 当てる点がまだ無い（nan）
    ref[4] = float("nan")
    force = torch.tensor([6.0, 6.0, 6.0, 4.0, 6.0])
    assert palm_press(palm, ref, force, 0.10, 0.02, 5.0).tolist() == [True, False, False, False, False]
    c = torch.zeros(3, dtype=torch.long)
    for cond in ([True, True, False], [True, False, False], [True, True, True]):
        c = update_press_count(c, torch.tensor(cond))
    assert c.tolist() == [3, 1, 1]


def test_env_palm_press_tracks_closed_lever_and_fires_after_hold():
    """0100: 当てる点はレバーと扉が閉じている間だけ追う。押し下げが press_hold 歩つづいたら立つ"""
    class _Arr:
        torch = torch.zeros(2, 3)

    ns = _load(["_arm_stage_palm_press"], {"math": math, "palm_press": palm_press, "update_press_count": update_press_count})
    env = _Env()
    env.simulator = SimpleNamespace(scene=SimpleNamespace(articulations={"door": SimpleNamespace(data=SimpleNamespace(joint_pos=_Arr()))}))
    env._press_ref = torch.full((2, 3), float("nan"))
    env._press_bias = torch.zeros(2, 3)
    env._press_count = torch.zeros(2, dtype=torch.long)
    env._arm_legs_free = torch.zeros(2, dtype=torch.bool)
    env.log_dict = {}
    env.use_left_hand = torch.tensor([True, True])
    gt = torch.tensor([[0.0, 0.3, 1.0], [0.0, 0.3, 1.0]])
    env._compute_grasp_target = lambda: gt.clone()
    palm = torch.tensor([[0.0, 0.3, 1.05], [0.0, 0.3, 1.05]])  # まだ上
    env._palm_pos = lambda side: palm.clone()
    env._hand_lever_force_n = lambda: torch.tensor([10.0, 10.0])
    arm = {**{k: float(v) for k, v in PRESS_DEFAULTS.items()}, "press_hold": 3}
    assert ns["_arm_stage_palm_press"](env, arm).tolist() == [False, False]
    assert torch.allclose(env._press_ref, gt)  # 閉じているので当てる点を追った
    # レバーが下がる（台 0 だけ 30°）。当てる点は台 0 では動かさず、台 1 は追い続ける
    _Arr.torch = torch.tensor([[0.0, math.radians(30.0), 0.0], [0.0, 0.0, 0.0]])
    gt = torch.tensor([[0.0, 0.3, 0.93], [0.0, 0.3, 1.0]])
    palm = torch.tensor([[0.0, 0.3, 0.95], [0.0, 0.3, 1.05]])
    for _ in range(2):
        assert ns["_arm_stage_palm_press"](env, arm).tolist() == [False, False]
    assert env._press_ref[0, 2] == pytest.approx(1.0)
    assert ns["_arm_stage_palm_press"](env, arm).tolist() == [True, False]  # 3 歩目で立つ
    assert float(env.log_dict["arm_press_early_frac"]) == 0.0  # 真のレバーは 30°＝早すぎでない


def test_env_release_switch():
    """0100: release="palm_press" なら手のひらの判定、既定は真のレバーの角度（0099 のまま）"""
    ns = _load(["_arm_stage_legs_mask"], {"update_legs_free": update_legs_free})
    env = _Env()
    env._arm_stage_latched = torch.ones(2, dtype=torch.bool)
    env._arm_legs_free = torch.zeros(2, dtype=torch.bool)
    env.log_dict = {}
    env._lever_unlatched = lambda: torch.tensor([True, False])
    env._arm_stage_palm_press = lambda arm: torch.tensor([False, True])
    assert ns["_arm_stage_legs_mask"](env, {"free_legs_when_unlatched": True, "release": "lever_true"}).tolist() == [False, True]
    env._arm_legs_free = torch.zeros(2, dtype=torch.bool)
    assert ns["_arm_stage_legs_mask"](env, {"free_legs_when_unlatched": True, "release": "palm_press"}).tolist() == [True, False]


TRAINER = Path("gr00t/rl/trl/trainer/ppo_trainer_homie_api.py")


def test_env_handover_mark_and_complete_through():
    """0101: 扉が handover_door_deg に届いた台に印（回の終わりまで残す）。complete_through なら完了は元の通り抜けの判定"""
    check = [ast.unparse(x) for x in _method("_check_termination").body]
    i = next(i for i, x in enumerate(check) if x.startswith("if arm is not None and arm['handover_door_deg'] is not None:"))
    assert "self.arm_handover_buf |= torch.rad2deg(torch.abs(hinge)) >= arm['handover_door_deg']" in check[i]
    assert check[0] == "super()._check_termination()"
    body = [ast.unparse(x) for x in _method("_stage_5_to_complete_condition").body]
    assert body[1].startswith("if arm is not None and (not arm['complete_through']):")
    assert "self.arm_handover_buf[env_ids] = False" in ast.unparse(_method("_reset_buffers_callback"))
    cfg = arm_stage_config({"enabled": True, "handover_door_deg": 3, "complete_through": True})
    assert cfg["handover_door_deg"] == 3.0 and cfg["complete_through"] is True


def test_trainer_eval_switches_to_handover_policy():
    """0101: 評価は引き継ぐ先の方策を毎歩回し、印の立った台の行動だけ替える（shadow は記憶を温め、zero は替える歩で 0 に）"""
    tree = ast.parse(TRAINER.read_text(encoding="utf-8"))
    ev = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "eval")
    src = ast.unparse(ev)
    assert "handover = self._load_handover_policy() if self.config.get('eval', {}).get('handover_policy_path', None) else None" in src
    assert "action_mean = torch.where(now[:, None], handover.action_mean.detach(), action_mean)" in src
    assert "handover.reset((now & ~handed).float())" in src
    i_mean = src.index("action_mean = policy_model.action_mean.detach()")
    i_switch = src.index("action_mean = torch.where(now[:, None]")
    i_cat = src.index("step_actions = torch.cat([policy_actions, homie_actions], dim=-1)")
    assert i_mean < i_switch < i_cat  # 替えた行動が脚の HOMIE と組まれて送られる
    load = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_load_handover_policy")
    lsrc = ast.unparse(load)
    assert "copy.deepcopy(self.accelerator.unwrap_model(self.model).policy)" in lsrc and "policy.eval_mode()" in lsrc


def test_count_success_armthrough():
    cs = _count_success()
    stage = np.array([[1, 1], [3, 3], [5, 4], [5, 4], [1, 1]], dtype=np.float32)
    ep = np.array([[0, 0], [1, 1], [2, 2], [3, 3], [0, 0]], dtype=np.float32)
    root = np.zeros((5, 2, 3))
    root[3, 0, 0] = 1.6
    rec = {"stage": stage, "episode_length": ep, "root_pos": root, "door_dof_pos": np.zeros((5, 2, 3))}
    s = cs.summarize(cs.split_episodes(rec), "armthrough")
    assert s["start_stage"] == 1 and s["episodes_from_stage0"] == 2 and s["success"] == 1
