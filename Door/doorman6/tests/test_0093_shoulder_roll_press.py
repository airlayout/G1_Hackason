"""パッチ 0093（11-06・T1 肩 roll の胴への押し当て）の試験。
Isaac Sim は起こさない（純粋な関数と、扉の環境から取り出した関数を偽の環境で動かす）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- env.config.shoulder_roll_press_max_nm=6.0 で、肩 roll の送る目標を「胴の側へ押す力 kp·(目標 − 角度) が 6 N·m まで」に切る
  （実機の door_run_stock.cap_roll_press と同じ。左は負・右は正が胴の側）。胴から離す側・他の関節は切らない
- rewards.reward_scales.penalty_shoulder_roll_press で、肩 roll が胴の側へ出した力の 2 乗の和（左右・物理の回の平均）を罰する
- 設定に書かなければ元のまま
"""
from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch

from gr00t.rl.envs.base_task.roll_press import ROLL_INWARD, cap_toward, press_toward
from gr00t.rl.envs.base_task.substep_monitor import SubstepMonitor

FPB = Path("gr00t/rl/envs/base_task/finger_primitive_base.py")
INWARD = np.array(ROLL_INWARD)
KP = np.array([100.0, 100.0])


def _real_cap_roll_press(target, q, kp, max_nm):
    """実機の door_run_stock.cap_roll_press（腕 14 の 1・8 番）を肩 roll の 2 個だけに写したもの"""
    out = np.array(target, dtype=float)
    for j, inward in enumerate(ROLL_INWARD):
        lim = q[j] + inward * max_nm / kp[j]
        out[j] = max(out[j], lim) if inward < 0 else min(out[j], lim)
    return out


def test_cap_pulls_back_only_the_inward_side():
    q = np.array([-0.05, 0.05])
    # 左は胴の側（負）へ 0.2 rad 先・右も胴の側（正）へ 0.2 rad 先 → どちらも 6 N·m（0.06 rad）まで戻る
    np.testing.assert_allclose(cap_toward(np.array([-0.25, 0.25]), q, KP, INWARD, 6.0), [-0.11, 0.11])
    # 胴から離す側は切らない
    np.testing.assert_allclose(cap_toward(np.array([0.3, -0.3]), q, KP, INWARD, 6.0), [0.3, -0.3])
    # 胴の側でも 6 N·m より手前は切らない
    np.testing.assert_allclose(cap_toward(np.array([-0.09, 0.09]), q, KP, INWARD, 6.0), [-0.09, 0.09])


def test_cap_matches_real_robot_and_torch():
    rng = np.random.default_rng(0)
    for _ in range(200):
        q = rng.uniform(-0.5, 0.5, 2)
        target = q + rng.uniform(-0.4, 0.4, 2)
        want = _real_cap_roll_press(target, q, KP, 6.0)
        np.testing.assert_allclose(cap_toward(target, q, KP, INWARD, 6.0), want, atol=1e-12)
        got_t = cap_toward(torch.tensor(target), torch.tensor(q), torch.tensor(KP), torch.tensor(INWARD), 6.0)
        np.testing.assert_allclose(got_t.numpy(), want, atol=1e-12)


def test_cap_does_not_write_inputs():
    target = np.array([-0.25, 0.25])
    q = np.array([-0.05, 0.05])
    cap_toward(target, q, KP, INWARD, 6.0)
    np.testing.assert_array_equal(target, [-0.25, 0.25])


def test_press_counts_only_toward_torso():
    tau = torch.tensor([[-20.0, 16.0], [5.0, -3.0], [-2.0, -1.0]])
    np.testing.assert_allclose(press_toward(tau, torch.tensor(ROLL_INWARD)).numpy(), [[20, 16], [0, 0], [2, 0]])


def test_monitor_averages_press_over_substeps():
    mon = SubstepMonitor(2, 14, 0, "cpu")
    mon.begin()
    arm_tau = torch.zeros(2, 14)
    lim = torch.full((14,), 25.0)
    for p in ([[6.0, 0.0], [0.0, 0.0]], [[6.0, 0.0], [0.0, 2.0]], [[6.0, 0.0], [0.0, 0.0]], [[6.0, 0.0], [0.0, 0.0]]):
        mon.add_roll_press(torch.tensor(p))
        mon.after_substep(arm_tau, lim)
    np.testing.assert_allclose(mon.roll_press_sq_mean().numpy(), [36.0, 1.0])
    np.testing.assert_allclose(mon.roll_press_peak.numpy(), [6.0, 2.0])
    mon.begin()
    np.testing.assert_array_equal(mon.roll_press_sq_mean().numpy(), [0.0, 0.0])


def _env_method():
    """FingerPrimitiveBase._cap_shoulder_roll_press をクラスの外の関数として取り出す"""
    tree = ast.parse(FPB.read_text(encoding="utf-8"))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_cap_shoulder_roll_press")
    namespace = {"torch": torch}
    exec(
        "from gr00t.rl.envs.base_task.roll_press import LEFT_SHOULDER_ROLL, RIGHT_SHOULDER_ROLL, ROLL_INWARD, cap_toward",
        namespace,
    )
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(FPB), "exec"), namespace)
    return namespace["_cap_shoulder_roll_press"]


def test_env_caps_only_shoulder_roll_columns():
    names = ["a", "left_shoulder_roll_joint", "b", "right_shoulder_roll_joint"]
    q = torch.tensor([[0.0, -0.05, 0.0, 0.05]])
    env = SimpleNamespace(
        config=SimpleNamespace(robot=SimpleNamespace(dof_names=names)),
        device="cpu",
        simulator=SimpleNamespace(dof_pos=q),
        p_gains=torch.tensor([150.0, 100.0, 40.0, 100.0]),
    )
    target = torch.tensor([[1.0, -0.25, -1.0, 0.25]])
    out = _env_method()(env, target, 6.0)
    np.testing.assert_allclose(out.numpy(), [[1.0, -0.11, -1.0, 0.11]], atol=1e-6)
    np.testing.assert_allclose(target.numpy(), [[1.0, -0.25, -1.0, 0.25]])  # 入力は書き換えない


def test_off_by_default():
    src = FPB.read_text(encoding="utf-8")
    assert 'self.config.get("shoulder_roll_press_max_nm", None)' in src
