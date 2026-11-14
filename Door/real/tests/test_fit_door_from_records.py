"""11-14 fit_door_from_records: 実機の記録から扉の値（レバーのばね・掛け金・手とレバーの滑り・クローザー）を幅付きで出す。
値が分かっている作り物の系列から正しく戻せるかを確かめる。機体も記録も要らない（順運動学を通る取り出しは記録で確かめる）。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import door_script as dscr  # noqa: E402
import fit_door_from_records as fdr  # noqa: E402

DT = 0.02
S2C, S2D, S3A, S3C = (dscr.SUBS.index(s) for s in ("2c", "2d", "3a", "3c"))


# ── 幾何 ───────────────────────────────────────────────────
def test_lever_angle_is_atan_of_depth_over_horizontal_distance():
    assert fdr.lever_angle(0.0, 0.08) == pytest.approx(0.0)
    assert fdr.lever_angle(0.08, 0.08) == pytest.approx(math.pi / 4)
    assert fdr.lever_angle(-0.01, 0.08) == 0.0  # 触れた高さより上はレバーが水平
    np.testing.assert_allclose(fdr.lever_angle(np.array([0.0, 0.08]), np.array([0.08, 0.08])), [0.0, math.pi / 4])


def test_lever_torque_about_the_axis_from_hand_force():
    # 手が受ける力（上向きが正）。水平のレバーを付け根から 10 cm で 10 N 押し下げる → 1 N·m
    assert fdr.lever_torque(0.0, 10.0, 0.10, 0.0) == pytest.approx(1.0)
    # 下がったレバー（付け根から 8 cm 下）で、手が蝶番の側（−y）へ押される分は回りを足す
    assert fdr.lever_torque(-3.0, 10.0, 0.10, 0.08) == pytest.approx(1.0 + 0.24)


def test_slide_ratio_recovers_the_friction_ratio_on_the_slope():
    # 坂 θ のレバーの上で、法線の力 N と棒に沿う（付け根へ戻す向きの）摩擦 μN を手が受ける
    th, n_force, mu = math.radians(40.0), 10.0, 0.4
    normal = np.array([-math.sin(th), math.cos(th)])  # (y, z)
    up_slope = np.array([math.cos(th), math.sin(th)])  # 付け根の側へ（+y）・上へ
    fy, fz = n_force * normal + mu * n_force * up_slope
    ratio, fn = fdr.slide_ratio(fy, fz, th)
    assert ratio == pytest.approx(mu)
    assert fn == pytest.approx(n_force)


# ── 触れた所・ばね ─────────────────────────────────────────
def test_contact_onset_needs_the_force_to_stay_above_the_line():
    fz = np.array([0.0, 4.0, 0.0, 1.0, 3.5, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0])
    assert fdr.contact_onset(fz, line=3.0, hold=5) == 4  # 1 番目の 4 N は 1 周期だけなので数えない
    assert fdr.contact_onset(np.zeros(10), line=3.0, hold=5) is None


def test_fit_spring_recovers_preload_and_slope():
    rng = np.random.default_rng(0)
    th = np.linspace(0.05, 0.75, 80)
    tau = 0.45 + 0.8 * th + rng.normal(0, 0.03, th.size)
    tau0, k, r2 = fdr.fit_spring(th, tau)
    assert tau0 == pytest.approx(0.45, abs=0.05)
    assert k == pytest.approx(0.8, abs=0.08)
    assert r2 > 0.9


def test_fit_spring_rejects_too_few_points():
    with pytest.raises(ValueError):
        fdr.fit_spring(np.array([0.1, 0.2]), np.array([0.5, 0.6]))


# ── 幅・回の分け方 ─────────────────────────────────────────
def test_band_is_low_median_high_and_ignores_nan():
    lo, mid, hi = fdr.band([1.0, 2.0, np.nan, 3.0, 4.0, 5.0])
    assert (lo, mid, hi) == (pytest.approx(1.4), pytest.approx(3.0), pytest.approx(4.6))
    assert all(math.isnan(v) for v in fdr.band([np.nan]))


def test_split_runs_is_deterministic_and_holds_out_every_third():
    fit, check = fdr.split_runs([1, 3, 7, 8, 9, 11, 12, 13, 14, 15, 16, 19, 21])
    assert check == [7, 11, 14, 19]
    assert fit == [1, 3, 8, 9, 12, 13, 15, 16, 21]
    assert sorted(fit + check) == [1, 3, 7, 8, 9, 11, 12, 13, 14, 15, 16, 19, 21]


def test_latch_bounds_upper_from_released_lower_from_not_released():
    b = fdr.latch_bounds([(True, 42.0), (True, 44.0), (False, 20.0), (False, 25.0)])
    assert b["upper_deg"] == pytest.approx(42.0)
    assert b["lower_deg"] == pytest.approx(25.0)
    b2 = fdr.latch_bounds([(True, 42.0)])
    assert b2["upper_deg"] == pytest.approx(42.0) and b2["lower_deg"] is None


# ── 作り物の系列で通し ─────────────────────────────────────
def synth_series(label: int, tau0: float = 0.5, k: float = 0.9, mu: float = 0.45, slip_from: float = 0.6,
                 door_open: bool = False, closer_nm: float = 6.0) -> fdr.RunSeries:
    """2c で触れて 2d で押し下げ、3a・3c へ。手は坂で mu の比で滑る（slip_from rad から）。3c で扉を押す"""
    n2c, n2d, n3a, n3c = 60, 100, 60, 150
    sub = np.r_[np.full(n2c, S2C), np.full(n2d, S2D), np.full(n3a, S3A), np.full(n3c, S3C)].astype(float)
    n = sub.size
    t = np.arange(n) * DT
    y_root, z0 = 0.406, 1.028
    s = np.full(n, 0.075)
    depth = np.r_[np.linspace(-0.04, 0.0, n2c), np.linspace(0.0, 0.07, n2d), np.full(n3a + n3c, 0.07)]
    th = np.zeros(n)
    for i in range(n):
        if i:
            s[i] = s[i - 1]
        th[i] = fdr.lever_angle(depth[i], s[i])
        if th[i] > slip_from and sub[i] in (S2D, S3A):
            s[i] = s[i - 1] + 0.0004  # 坂で棒の先へ流れる
            th[i] = fdr.lever_angle(depth[i], s[i])
    tau = np.where(depth > 0, tau0 + k * th, 0.0)
    sliding = np.r_[0.0, np.diff(s)] > 0
    F = np.zeros((n, 3))
    for i in range(n):
        if tau[i] <= 0:
            continue
        normal = np.array([-math.sin(th[i]), math.cos(th[i])])
        up = np.array([math.cos(th[i]), math.sin(th[i])])
        ratio = mu if sliding[i] else 0.6 * math.tan(th[i])
        # 回り = s·Fz − depth·Fy を tau に合わせる大きさ
        f_unit = normal + ratio * up
        mag = tau[i] / (s[i] * f_unit[1] - depth[i] * f_unit[0])
        F[i, 1:] = mag * f_unit
    r_hinge = 0.77
    if door_open:
        F[sub == S3C, 0] = -closer_nm / r_hinge
    tip = np.c_[np.full(n, -0.075), y_root - s, z0 - depth]
    ang = np.zeros(n)
    if door_open:
        ang[sub == S3C] = np.linspace(3.0, 8.0, n3c)
    return fdr.RunSeries(label=label, t=t, sub=sub, tip=tip, palm=tip + np.array([-0.1, 0.0, 0.0]), force=F,
                         pose=np.zeros((n, 3)), angle_deg=ang, y_root=y_root, r_hinge=r_hinge, stop_reason="", press_cm=7.0)


def test_lever_samples_recover_the_spring_from_a_synthetic_run():
    sr = synth_series(1)
    smp = fdr.lever_samples(sr)
    tau0, k, _ = fdr.fit_spring(smp["theta"], smp["tau"])
    assert tau0 == pytest.approx(0.5, abs=0.05)
    assert k == pytest.approx(0.9, abs=0.1)


def test_run_summary_finds_slide_and_angle():
    sr = synth_series(3, mu=0.45)
    row = fdr.run_summary(sr)
    assert row["label"] == 3
    assert row["slide_cm"] > 1.0
    assert row["slide_ratio"] == pytest.approx(0.45, abs=0.03)
    assert 35.0 < row["theta_3a_deg"] < 45.0
    assert not row["fell_under"]


def test_closer_moment_from_opening_runs_only():
    runs = [synth_series(1, door_open=True, closer_nm=6.0), synth_series(7, door_open=True, closer_nm=9.0),
            synth_series(9, door_open=False)]
    lo, mid, hi = fdr.closer_moment(runs, opened={1, 7})
    assert 6.0 - 1e-6 <= lo <= mid <= hi <= 9.0 + 1e-6
    assert mid == pytest.approx(7.5, abs=0.5)


def test_fit_all_writes_bands_and_keeps_check_runs_out():
    runs = [synth_series(lb, tau0=0.5 + 0.02 * i, k=0.9, door_open=lb in (1, 7, 8)) for i, lb in
            enumerate([1, 3, 7, 8, 9, 11, 12, 13, 14, 15, 16, 19, 21])]
    out = fdr.fit_all(runs, opened={1, 7, 8}, released={1, 3, 7, 8, 9, 11, 13, 15, 16, 19, 21})
    assert out["runs_check"] == [7, 11, 14, 19]
    lo, mid, hi = out["lever"]["tau0_nm"]
    assert lo <= mid <= hi and 0.45 < mid < 0.75
    assert out["latch_release_deg"]["upper_deg"] is not None
    assert len(out["closer"]["open_moment_nm"]) == 3
    assert all(r["label"] not in (7, 11, 14, 19) for r in out["rows_fit"])


def test_fit_all_skips_turned_hands_for_spring_and_unknown_runs_for_latch():
    labels = [1, 3, 7, 8, 9, 11, 12, 13, 14, 15, 16, 19, 21]
    runs = [synth_series(lb, tau0=(3.0 if lb == 21 else 0.5), k=0.9) for lb in labels]
    out = fdr.fit_all(runs, opened=set(), released=set(labels) - {12, 14}, unknown={12, 14}, turned={19, 20, 21})
    assert all(s["label"] != 21 for s in out["lever"]["per_run"])
    assert out["lever"]["tau0_nm"][2] < 1.0  # 21 回目の 3.0 が混ざらない
    assert out["latch_release_deg"]["n_not_released"] == 0
    assert out["excluded"] == {"latch_unknown": [12, 14], "hand_turned": [19, 20, 21]}


def test_closer_moment_pushing_only_ignores_samples_off_the_door():
    sr = synth_series(1, door_open=True, closer_nm=8.0)
    m = np.where(sr.sub == S3C)[0]
    sr.force[m[::2], 0] = 0.0  # 半分の時間は手が扉から離れている
    _, mid_push, _ = fdr.closer_moment([sr], opened={1})
    _, mid_all, _ = fdr.closer_moment([sr], opened={1}, pushing_only=False)
    assert mid_push == pytest.approx(8.0, abs=0.2)
    assert mid_all < mid_push
