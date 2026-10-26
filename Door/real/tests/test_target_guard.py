"""直し B・2・8（10-26）の共通の関数 target_guard.py の試験（機体なし・Mac）。

  python -m pytest G1_Hackason/Door/real/tests/test_target_guard.py -q

- B: 腕の積み上げ D を「今の角度 ±δ」と「可動域の端から margin 内側」に収める（止められた腕の前で目標がたまらない）
- 2: 送る目標を可動域の内側に切り、1 周期の変化を max_step までにする（door_run_real.py と同じ切り方）
- 8: 立つ HOMIE の観測から指令（各フレームの先頭 3 つ）を消す
- numpy と torch で同じ入力に同じ出力（差 1e-6 以下）。学習（VIRAL6）に置く写しはパッチ 0069 の新しいファイルと 1 字も違わない
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import target_guard as tg  # noqa: E402

PATCH_DIR = HERE.parent / "doorman6"
ACTION_SCALE = 0.25
N_ARM = 14


def _arm_case(seed: int, n_env: int = 64):
    """腕 14 関節のでたらめな状態。可動域は ±2.5 rad 前後、今の角度は可動域の中（端の近くも含む）"""
    rng = np.random.default_rng(seed)
    lower = -rng.uniform(1.5, 3.0, N_ARM)
    upper = rng.uniform(1.5, 3.0, N_ARM)
    q_default = rng.uniform(-0.3, 0.3, N_ARM)
    q = rng.uniform(lower, upper, (n_env, N_ARM))
    D = rng.uniform(-15.0, 15.0, (n_env, N_ARM))
    return D, q, q_default, lower, upper


def _target(D, q_default):
    return ACTION_SCALE * D + q_default


# --- B: 腕の積み上げ ---------------------------------------------------------------------------------


def test_guard_keeps_target_within_delta_of_current_angle():
    D, q, qd, lo, hi = _arm_case(0)
    out = tg.guard_arm_accumulator(D, q, qd, lo, hi, ACTION_SCALE, delta=0.3, margin=0.05)
    t = _target(out, qd)
    inside = (q >= lo + 0.05 + 0.3) & (q <= hi - 0.05 - 0.3)  # 可動域の端が効かない所
    assert np.all(np.abs(t - q)[inside] <= 0.3 + 1e-12)
    assert np.all(t >= lo + 0.05 - 1e-12) and np.all(t <= hi - 0.05 + 1e-12)


def test_guard_leaves_unclipped_entries_bit_identical():
    D, q, qd, lo, hi = _arm_case(1)
    q = np.clip(q, lo + 0.1, hi - 0.1)  # 可動域の端から離す（端の近くでは 0.025 rad 先でも切られる）
    D = (q - qd) / ACTION_SCALE + 0.1  # 目標が今の角度の 0.025 rad 先＝切られない
    D[:, 0] = 0.0  # 1 列目は 0（既定の姿勢）。今の角度から離れていれば切られる
    out = tg.guard_arm_accumulator(D, q, qd, lo, hi, ACTION_SCALE, delta=0.3, margin=0.05)
    keep = np.abs(_target(D, qd) - q) <= 0.3
    keep &= (_target(D, qd) >= lo + 0.05) & (_target(D, qd) <= hi - 0.05)
    assert np.array_equal(out[keep], D[keep])
    assert keep[:, 1:].all()


def test_stalled_joint_does_not_accumulate():
    """止められた関節（角度が動かない）の前で、正の増分を 500 周期入れ続けても目標は q+δ で止まる"""
    _, _, qd, lo, hi = _arm_case(2, n_env=1)
    q = np.zeros((1, N_ARM))  # 腕は胴に当たって 0 で止まっている
    D = np.zeros((1, N_ARM))
    for _ in range(500):
        D = np.clip(D + 0.3 * 1.0, -15.0, 15.0)  # door_core.step と同じ積算（出力 1.0 を出し続ける）
        D = tg.guard_arm_accumulator(D, q, qd, lo, hi, ACTION_SCALE, delta=0.3, margin=0.05)
    t = _target(D, qd)
    assert np.all(t <= q + 0.3 + 1e-12)
    assert np.allclose(t, np.minimum(q + 0.3, hi - 0.05))


def test_joint_outside_margin_band_goes_to_the_limit():
    """今の角度が可動域の端（margin の外）にあるときは可動域の側が勝つ（目標は端の margin 内側）"""
    lo = np.full(N_ARM, -1.0)
    hi = np.full(N_ARM, 1.0)
    qd = np.zeros(N_ARM)
    q = np.full((1, N_ARM), 1.6)  # 上の端の 0.6 rad 外（δ 0.3 を足しても届かない）
    D = np.full((1, N_ARM), 10.0)
    out = tg.guard_arm_accumulator(D, q, qd, lo, hi, ACTION_SCALE, delta=0.3, margin=0.05)
    assert np.allclose(_target(out, qd), 0.95)
    q = np.full((1, N_ARM), -1.6)
    D = np.full((1, N_ARM), -10.0)
    out = tg.guard_arm_accumulator(D, q, qd, lo, hi, ACTION_SCALE, delta=0.3, margin=0.05)
    assert np.allclose(_target(out, qd), -0.95)


def test_guard_does_not_change_its_input():
    D, q, qd, lo, hi = _arm_case(3)
    D0 = D.copy()
    tg.guard_arm_accumulator(D, q, qd, lo, hi, ACTION_SCALE, delta=0.3, margin=0.05)
    assert np.array_equal(D, D0)


def test_guard_works_on_one_robot_vector():
    """実機（door_core）は台の次元の無い [14] で呼ぶ"""
    D, q, qd, lo, hi = _arm_case(4, n_env=1)
    batched = tg.guard_arm_accumulator(D, q, qd, lo, hi, ACTION_SCALE, delta=0.3, margin=0.05)
    single = tg.guard_arm_accumulator(D[0], q[0], qd, lo, hi, ACTION_SCALE, delta=0.3, margin=0.05)
    assert np.array_equal(batched[0], single)


# --- 2: 送る目標 --------------------------------------------------------------------------------------


def test_limit_sent_target_matches_door_run_real():
    """door_run_real.py の元の 2 行（可動域の内側に切る → 前の目標から 0.7 rad まで）と同じ"""
    rng = np.random.default_rng(5)
    lo = -rng.uniform(1.0, 3.0, 29)
    hi = rng.uniform(1.0, 3.0, 29)
    for _ in range(200):
        target = rng.uniform(-4.0, 4.0, 29)
        last = rng.uniform(lo, hi)
        want = np.clip(target, lo + 0.05, hi - 0.05)
        want = last + np.clip(want - last, -0.7, 0.7)
        got = tg.limit_sent_target(target, last, lo, hi, margin=0.05, max_step=0.7)
        assert np.array_equal(got, want)


def test_limit_sent_target_without_last_or_margin():
    rng = np.random.default_rng(6)
    lo, hi = -np.ones(29), np.ones(29)
    target = rng.uniform(-3.0, 3.0, 29)
    assert np.array_equal(tg.limit_sent_target(target, None, lo, hi, margin=0.05, max_step=0.7),
                          np.clip(target, -0.95, 0.95))
    last = np.zeros(29)
    assert np.array_equal(tg.limit_sent_target(target, last, None, None, margin=None, max_step=0.7),
                          np.clip(target, -0.7, 0.7))
    assert np.array_equal(tg.limit_sent_target(target, last, lo, hi, margin=None, max_step=None), target)


# --- 8: 立つ HOMIE の指令 -----------------------------------------------------------------------------


def test_zero_stand_command_clears_first_three_of_every_frame():
    rng = np.random.default_rng(7)
    obs = rng.normal(size=(4, 6 * 86))
    out = tg.zero_stand_command(obs, n_frames=6)
    frames_in = obs.reshape(4, 6, 86)
    frames_out = out.reshape(4, 6, 86)
    assert np.all(frames_out[..., :3] == 0.0)
    assert np.array_equal(frames_out[..., 3:], frames_in[..., 3:])
    assert out.shape == obs.shape
    assert not np.all(obs.reshape(4, 6, 86)[..., :3] == 0.0)  # 入力は書き換えない


def test_zero_stand_command_on_trajectory_shaped_obs():
    """PPO の学習は [台, 歩, 6×86] の形で渡す"""
    rng = np.random.default_rng(8)
    obs = rng.normal(size=(3, 5, 6 * 86))
    out = tg.zero_stand_command(obs, n_frames=6)
    assert np.all(out.reshape(3, 5, 6, 86)[..., :3] == 0.0)
    assert np.array_equal(out.reshape(3, 5, 6, 86)[..., 3:], obs.reshape(3, 5, 6, 86)[..., 3:])


# --- numpy と torch -----------------------------------------------------------------------------------


def test_numpy_and_torch_give_the_same_answer():
    torch = pytest.importorskip("torch")
    for seed in range(5):
        D, q, qd, lo, hi = _arm_case(10 + seed)
        want = tg.guard_arm_accumulator(D, q, qd, lo, hi, ACTION_SCALE, delta=0.3, margin=0.05)
        T = [torch.tensor(x, dtype=torch.float64) for x in (D, q, qd, lo, hi)]
        got = tg.guard_arm_accumulator(*T, ACTION_SCALE, delta=0.3, margin=0.05).numpy()
        assert np.abs(got - want).max() <= 1e-6
        T32 = [torch.tensor(x, dtype=torch.float32) for x in (D, q, qd, lo, hi)]
        got32 = tg.guard_arm_accumulator(*T32, ACTION_SCALE, delta=0.3, margin=0.05).double().numpy()
        assert np.abs(_target(got32, qd) - _target(want, qd)).max() <= 1e-5  # 学習は float32
    rng = np.random.default_rng(20)
    target, last = rng.uniform(-4, 4, (8, 29)), rng.uniform(-1, 1, (8, 29))
    lo, hi = -np.ones(29) * 2, np.ones(29) * 2
    want = tg.limit_sent_target(target, last, lo, hi, margin=0.05, max_step=0.7)
    got = tg.limit_sent_target(*(torch.tensor(x, dtype=torch.float64) for x in (target, last, lo, hi)),
                               margin=0.05, max_step=0.7).numpy()
    assert np.abs(got - want).max() <= 1e-6
    obs = rng.normal(size=(4, 6 * 86))
    t_obs = torch.tensor(obs)
    got = tg.zero_stand_command(t_obs, n_frames=6).numpy()
    assert np.array_equal(got, tg.zero_stand_command(obs, n_frames=6))
    assert not torch.all(t_obs.reshape(4, 6, 86)[..., :3] == 0.0)  # 入力の tensor も書き換えない


def test_python38_syntax():
    import ast

    ast.parse((HERE / "target_guard.py").read_text(encoding="utf-8"), feature_version=(3, 8))


# --- 学習側の写し --------------------------------------------------------------------------------------


def _new_file_from_patch(patch: Path, path_in_repo: str) -> str:
    """git format-patch の「新しいファイル」の中身を取り出す"""
    text = patch.read_text(encoding="utf-8")
    m = re.search(r"^diff --git a/" + re.escape(path_in_repo) + r" b/.*?\n@@ [^\n]*\n(.*?)(?=^diff --git |^-- \n|\Z)",
                  text, flags=re.S | re.M)
    assert m, f"{patch.name} に {path_in_repo} の新しいファイルが無い"
    lines = m.group(1).split("\n")
    body = [ln[1:] for ln in lines if ln.startswith("+")]
    return "\n".join(body) + "\n"


def test_training_copy_is_identical():
    patches = sorted(PATCH_DIR.glob("0069-*.patch"))
    if not patches:
        pytest.skip("パッチ 0069 がまだ無い")
    got = _new_file_from_patch(patches[0], "gr00t/rl/envs/base_task/target_guard.py")
    assert got == (HERE / "target_guard.py").read_text(encoding="utf-8")
