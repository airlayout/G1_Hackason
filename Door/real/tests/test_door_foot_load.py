"""door_foot_load.py（HOLD で両足に体重が乗っているかを静力学で逆算する）の試験（2026-10-26、段 1）。

  cd G1_Hackason && Navigation/.venv/bin/python -m pytest -q Door/real/tests/test_door_foot_load.py

確かめること:
  1. numpy だけの逆算が MuJoCo（公式の g1_29dof_rev_1_0.urdf を読んだもの）と 0.5 N 未満で一致する（mujoco が無ければ飛ばす）
  2. 作り物の立ち姿（MuJoCo の静止シム・綱が体重の 0・40・85% を受ける）で、真値との差が体重の ±2%
  3. 実機の記録: 10-23 の HOLD（A を押す前）は拒み、理由に右足の力が出る。10-16 の 2 回目（約 50%）は線 85% なら拒む
  4. 体重が全部乗った姿勢は通す。脚の関節が 0.2 rad/s 以上で動いている間は判定しない
  5. PC2 の Python 3.8 の文法で読める
作り物の立ち姿の npz は tests/data/make_foot_load_stand.py で作る（Mac・mujoco）。
"""
from __future__ import annotations

import ast
import dataclasses
import json
import re
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import door_foot_load as F  # noqa: E402

MODEL_NPZ = HERE / "leg_model_g1_29dof_rev_1_0.npz"
URDF = HERE.parent / "doorman6" / "usd_fix" / "urdf_src" / "g1_29dof_rev_1_0.urdf"
STAND_SIM = HERE / "tests" / "data" / "foot_load_stand_sim.npz"
LOGS = HERE / "logs_real"
REC_1016_1 = LOGS / "door_real_20261016_173324.npz"  # 10-16 の 1 回目（傾き 15.6° で A）
REC_1016_2 = LOGS / "door_real_20261016_175033.npz"  # 10-16 の 2 回目（144 秒立てた）
REC_1023 = LOGS / "door_real_20261023_165924_damp.npz"  # 10-23（右足が浮いたまま A・6.2 秒で脱力）
PHASE_HOLD, PHASE_STAND = 2, 3  # door_run_real.PHASES
DT = 0.02

try:
    import mujoco  # noqa: F401

    HAS_MUJOCO = True
except ImportError:
    HAS_MUJOCO = False

needs_mujoco = pytest.mark.skipif(not (HAS_MUJOCO and URDF.exists()), reason="mujoco か公式の URDF が無い")
needs_records = pytest.mark.skipif(not (REC_1016_2.exists() and REC_1023.exists()), reason="10-16・10-23 の実機の記録が無い")


@pytest.fixture(scope="module")
def model() -> F.LegModel:
    return F.load_leg_model(MODEL_NPZ)


def _load(path: Path) -> dict:
    with np.load(path) as z:
        return {k: z[k] for k in ("t", "phase", "q", "dq", "tau", "quat")}


def _before_stand(rec: dict, seconds: float) -> np.ndarray:
    """最初に STAND へ入る直前（A を押す前）の seconds 秒の HOLD の周期の番号"""
    ph = rec["phase"].astype(int)
    k = int(np.flatnonzero((ph[1:] == PHASE_STAND) & (ph[:-1] == PHASE_HOLD))[0]) + 1
    t = rec["t"]
    return np.flatnonzero((t >= t[k] - seconds) & (t < t[k]) & (ph == PHASE_HOLD))


def _replay_gate(model: F.LegModel, rec: dict, idx: np.ndarray, **lines) -> F.GateVerdict:
    gate = F.HoldLoadGate(weight=model.weight, **lines)
    for i in idx:
        forces = F.foot_forces(model, rec["q"][i], rec["tau"][i], rec["quat"][i])
        gate = gate.push(float(rec["t"][i]), forces, rec["dq"][i])
    return gate.verdict()


def _stand_sim() -> dict:
    with np.load(STAND_SIM) as z:
        return {k: z[k] for k in z.files}


def _still_gate(model: F.LegModel, q, tau, quat, dq, seconds: float = 1.2, **lines) -> F.HoldLoadGate:
    """同じ姿勢・同じ力を seconds 秒（50 Hz）入れた判定器"""
    forces = F.foot_forces(model, q, tau, quat)
    gate = F.HoldLoadGate(weight=model.weight, **lines)
    for k in range(int(round(seconds / DT)) + 1):
        gate = gate.push(k * DT, forces, dq)
    return gate


# --- モデルの npz ---


def test_model_has_both_legs_and_source(model):
    assert model.joint_names[0] == "left_hip_pitch_joint" and model.joint_names[11] == "right_ankle_roll_joint"
    assert model.total_mass == pytest.approx(33.341, abs=0.01)  # g1_29dof_rev_1_0.urdf の質量の合計
    meta = model.meta
    assert "g1_29dof_rev_1_0" in meta["robot"] and meta["source_commit"] and meta["urdf_sha256"]


# --- 1. numpy と MuJoCo の一致 ---


@needs_mujoco
@needs_records
def test_numpy_matches_mujoco_on_real_records(model):
    """実機の記録の 200 時点（RAMP・HOLD・STAND・POST）で、6 次元の力が MuJoCo の逆算と 0.5 N 未満で一致する"""
    import export_leg_model as E

    m = E.build_model(URDF)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(1026)
    worst = 0.0
    n_checked = 0
    for path, n in ((REC_1016_1, 60), (REC_1016_2, 70), (REC_1023, 70)):
        rec = _load(path)
        pool = np.flatnonzero(np.isin(rec["phase"].astype(int), (1, 2, 3, 5)))
        for i in rng.choice(pool, size=n, replace=False):
            want_l, want_r = E.mujoco_foot_wrenches(m, d, rec["q"][i], rec["tau"][i], rec["quat"][i])
            got = F.foot_forces(model, rec["q"][i], rec["tau"][i], rec["quat"][i])
            worst = max(worst, float(np.abs(got.left - want_l).max()), float(np.abs(got.right - want_r).max()))
            n_checked += 1
    assert n_checked == 200
    assert worst < 0.5


# --- 2. 作り物の立ち姿 ---


def test_synthetic_stand_matches_truth_within_2_percent(model):
    sim = _stand_sim()
    weight = float(sim["weight"])
    assert weight == pytest.approx(model.weight, rel=1e-6)  # 同じモデルで作った
    assert set(np.round(sim["rope_share"], 2)) >= {0.0, 0.4, 0.85}
    for k in range(len(sim["rope_share"])):
        got = F.foot_forces(model, sim["q"][k], sim["tau"][k], sim["quat"][k])
        true_l, true_r = sim["fz_true"][k]
        # 作り物そのものの確かめ: 床の力の和 = 体重 − 綱
        assert (true_l + true_r) / weight == pytest.approx(1.0 - sim["rope_share"][k], abs=0.01)
        assert abs(got.fz_left - true_l) <= 0.02 * weight
        assert abs(got.fz_right - true_r) <= 0.02 * weight
        assert abs(got.fz_left + got.fz_right - true_l - true_r) <= 0.02 * weight


# --- 3. 実機の記録 ---


@needs_records
def test_hold_1023_before_a_is_rejected_for_right_foot(model):
    """10-23: 綱が体重の 8 割以上を受け、右足がほぼ浮いたまま A を押した（右足 約 5 N）"""
    v = _replay_gate(model, _load(REC_1023), _before_stand(_load(REC_1023), 2.0))
    assert v.decision == F.REJECT and not v.passed
    m = re.search(r"右足 (-?\d+) N（体重の", v.reason)
    assert m is not None, v.reason
    assert 0 <= int(m.group(1)) <= 10
    assert v.fz_right == pytest.approx(4.5, abs=3.0)
    assert v.total_ratio < 0.25


@needs_records
def test_hold_1016_second_is_rejected_at_85_percent(model):
    """10-16 の 2 回目: A の前は両足で体重の約 5 割。線 85% なら拒み、線を下げれば通る（線は引数で変えられる）"""
    rec = _load(REC_1016_2)
    idx = _before_stand(rec, 2.0)
    v = _replay_gate(model, rec, idx, min_total=0.85, min_each=0.35)
    assert v.decision == F.REJECT
    assert "両足の合計" in v.reason and "85%" in v.reason
    assert 0.45 < v.total_ratio < 0.60
    loose = _replay_gate(model, rec, idx, min_total=0.45, min_each=0.20)
    assert loose.decision == F.PASS, loose.reason


# --- 4. 通す・判定しない ---


def test_full_load_passes(model):
    sim = _stand_sim()
    k = int(np.argmin(np.abs(sim["rope_share"])))  # 綱なし
    v = _still_gate(model, sim["q"][k], sim["tau"][k], sim["quat"][k], np.zeros(29)).verdict()
    assert v.decision == F.PASS and v.passed, v.reason
    assert v.total_ratio == pytest.approx(1.0, abs=0.02)
    assert "足 L" in v.reason


def test_rope_85_percent_is_rejected(model):
    sim = _stand_sim()
    k = int(np.argmin(np.abs(sim["rope_share"] - 0.85)))
    v = _still_gate(model, sim["q"][k], sim["tau"][k], sim["quat"][k], np.zeros(29)).verdict()
    assert v.decision == F.REJECT and "両足の合計" in v.reason


def test_one_light_foot_is_rejected_and_named(model):
    """作り物の pelvis を 3° 傾けた場合（綱 40%）: 片足が軽い。合計の線を下げても、軽い方の足で拒み、その足を名指す"""
    sim = _stand_sim()
    k = int(np.argmax(np.abs(sim["roll_deg"])))
    true_l, true_r = sim["fz_true"][k]
    light = "左足" if true_l < true_r else "右足"
    assert min(true_l, true_r) < 0.35 * float(sim["weight"]) < max(true_l, true_r)
    v = _still_gate(model, sim["q"][k], sim["tau"][k], sim["quat"][k], np.zeros(29), min_total=0.5).verdict()
    assert v.decision == F.REJECT
    assert light in v.reason and "両足の合計" not in v.reason


def test_display_text_shows_both_feet_and_ratio():
    assert F.format_load(163.4, 163.6, 327.0) == "足 L 163 N・R 164 N（体重の 100%）"


def test_moving_leg_is_not_judged_until_still_for_a_window(model):
    sim = _stand_sim()
    k = int(np.argmin(np.abs(sim["rope_share"])))
    forces = F.foot_forces(model, sim["q"][k], sim["tau"][k], sim["quat"][k])
    still = np.zeros(29)
    moving = np.zeros(29)
    moving[3] = 0.25  # 左膝
    gate = F.HoldLoadGate(weight=model.weight)
    for i in range(60):
        gate = gate.push(i * DT, forces, still)
    assert gate.verdict().passed
    gate = gate.push(60 * DT, forces, moving)
    v = gate.verdict()
    assert v.decision == F.SKIP and not v.passed
    assert "動いている" in v.reason and "関節 3" in v.reason
    for i in range(61, 61 + 49):  # 動いた周期が窓（1 秒）に残っている間は判定しない
        gate = gate.push(i * DT, forces, still)
        assert gate.verdict().decision == F.SKIP
    for i in range(110, 115):  # 1 秒より前に抜けたら判定する
        gate = gate.push(i * DT, forces, still)
    assert gate.verdict().passed


def test_speed_line_is_strict_and_arms_are_ignored(model):
    sim = _stand_sim()
    k = int(np.argmin(np.abs(sim["rope_share"])))
    args = (model, sim["q"][k], sim["tau"][k], sim["quat"][k])
    at_line = np.zeros(29)
    at_line[10] = 0.2  # 右足首 pitch がちょうど 0.2 rad/s（「0.2 未満のときだけ判定」なので判定しない）
    assert _still_gate(*args, at_line).verdict().decision == F.SKIP
    below = np.zeros(29)
    below[10] = 0.19
    assert _still_gate(*args, below).verdict().passed
    arms = np.zeros(29)
    arms[12:] = 2.0  # 腰と腕は見ない
    assert _still_gate(*args, arms).verdict().passed


def test_window_not_yet_full_is_not_judged(model):
    sim = _stand_sim()
    k = int(np.argmin(np.abs(sim["rope_share"])))
    v = _still_gate(model, sim["q"][k], sim["tau"][k], sim["quat"][k], np.zeros(29), seconds=0.5).verdict()
    assert v.decision == F.SKIP and "1.0 秒" in v.reason


def test_nan_force_is_not_judged(model):
    sim = _stand_sim()
    k = int(np.argmin(np.abs(sim["rope_share"])))
    tau = sim["tau"][k].copy()
    tau[4] = np.nan
    v = _still_gate(model, sim["q"][k], tau, sim["quat"][k], np.zeros(29)).verdict()
    assert v.decision == F.SKIP and not v.passed


def test_push_returns_new_gate_and_keeps_old(model):
    sim = _stand_sim()
    forces = F.foot_forces(model, sim["q"][0], sim["tau"][0], sim["quat"][0])
    g0 = F.HoldLoadGate(weight=model.weight)
    g1 = g0.push(0.0, forces, np.zeros(29))
    assert g0.samples == () and len(g1.samples) == 1
    with pytest.raises(dataclasses.FrozenInstanceError):
        g1.weight = 1.0


def test_bad_lines_are_refused():
    with pytest.raises(ValueError):
        F.HoldLoadGate(weight=0.0)
    with pytest.raises(ValueError):
        F.HoldLoadGate(weight=300.0, window_s=0.0)
    with pytest.raises(ValueError):
        F.HoldLoadGate(weight=300.0, min_total=-0.1)


# --- 5. PC2 の Python 3.8 ---


def test_source_parses_as_python38():
    src = (HERE / "door_foot_load.py").read_text(encoding="utf-8")
    ast.parse(src, feature_version=(3, 8))


def test_model_npz_loads_without_pickle():
    with np.load(MODEL_NPZ, allow_pickle=False) as z:
        meta = json.loads(str(z["meta"]))
    assert meta["foot_point"]


# --- 逆向き（足の力 → 脚のトルク）。段 2 の作り物の機体と段 3 のシムの作り物の吊り具に使う ---


def test_torques_for_wrenches_round_trip(model):
    """足の力を決めて脚のトルクを作り、foot_forces に戻すと元の力になる（傾いた姿勢・片足だけ・モーメントあり）"""
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = np.zeros(29)
        q[:12] = rng.uniform(-0.3, 0.3, 12) + np.array([-0.1, 0, 0, 0.3, -0.2, 0] * 2)
        ang = rng.uniform(-0.15, 0.15, 3)
        h = np.linalg.norm(ang)
        quat = np.r_[np.cos(h / 2), np.sin(h / 2) * ang / h]
        w_left = rng.uniform(-20, 20, 6) + np.array([0, 0, 150, 0, 0, 0])
        w_right = rng.uniform(-20, 20, 6) + np.array([0, 0, 50, 0, 0, 0])
        tau = F.leg_torques_for_wrenches(model, q, quat, w_left, w_right)
        assert tau.shape == (12,)
        got = F.foot_forces(model, q, tau, quat)
        np.testing.assert_allclose(got.left, w_left, atol=1e-8)
        np.testing.assert_allclose(got.right, w_right, atol=1e-8)


def test_full_load_torques_pass_gate(model):
    """既定の姿勢・まっすぐで体重を両足に半分ずつ: そのトルクを入れた判定器は通す"""
    q = np.zeros(29)
    q[:12] = [-0.1, 0, 0, 0.3, -0.2, 0] * 2
    quat = np.array([1.0, 0, 0, 0])
    half = np.array([0, 0, model.weight / 2, 0, 0, 0])
    tau = np.zeros(29)
    tau[:12] = F.leg_torques_for_wrenches(model, q, quat, half, half)
    gate = _still_gate(model, q, tau, quat, np.zeros(29))
    assert gate.verdict().passed


def test_too_large_load_is_not_judged(model):
    """逆算が体重より大きすぎる（トルクの読みが壊れた・特異な姿勢）ときは通さず「判定しない」（10-26 の見直し）"""
    q = np.zeros(29)
    q[:12] = [-0.1, 0, 0, 0.3, -0.2, 0] * 2
    quat = np.array([1.0, 0, 0, 0])
    big = np.array([0, 0, model.weight, 0, 0, 0])  # 片足ずつ体重ぶん＝合計 200%
    tau = np.zeros(29)
    tau[:12] = F.leg_torques_for_wrenches(model, q, quat, big, big)
    v = _still_gate(model, q, tau, quat, np.zeros(29)).verdict()
    assert v.decision == F.SKIP and "体重" in v.reason
