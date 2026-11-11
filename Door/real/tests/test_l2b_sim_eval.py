"""l2b_sim_eval_1110 の試験: 蝶番の軸の当てはめ・最初の回の切り出し・扉の片寄り・FAST-LIO の受け取りの刻み。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import hand_door_angle as hda  # noqa: E402
import l2b_sim_eval_1110 as l2b  # noqa: E402


def synthetic_episode(open_sign: float, n: int = 60) -> l2b.Episode:
    """蝶番 (2.0, −0.47)・板の中心 (2.0, 0.0)。20 歩目から 0.4°/歩で開く"""
    hinge_xy = np.array([2.0, -0.47])
    c0 = np.array([2.0, 0.0])
    ang = np.radians(np.clip(np.arange(n) - 20, 0, None) * 0.4)
    panel = np.array([[*(hinge_xy + l2b.rot2(open_sign * a) @ (c0 - hinge_xy)), 1.0] for a in ang])
    z3 = np.zeros((n, 3))
    return l2b.Episode(t=np.arange(n) * 0.02, hinge=ang, lever=np.zeros(n), root_pos=z3, root_quat=np.tile([0, 0, 0, 1.0], (n, 1)),
                       q=np.zeros((n, 29)), panel=panel, grasp=z3, force=np.zeros(n))


@pytest.mark.parametrize("open_sign", [1.0, -1.0])
def test_fit_hinge_recovers_axis_and_push(open_sign: float) -> None:
    # Arrange
    ep = synthetic_episode(open_sign)
    # Act
    line, res = l2b.fit_hinge(ep)
    # Assert
    np.testing.assert_allclose(line.hinge_xy, [2.0, -0.47], atol=1e-9)
    np.testing.assert_allclose(line.along, [0.0, 1.0], atol=1e-9)
    assert line.open_sign == open_sign
    assert res < 1e-9
    # 開く向きに回すと板の中心は push の向きへ出る
    moved = ep.panel[-1, :2] - ep.panel[0, :2]
    assert float(moved @ line.push) > 0


def test_fit_hinge_needs_two_degrees() -> None:
    ep = synthetic_episode(1.0, n=24)  # 最大 1.2°
    assert l2b.fit_hinge(ep) is None


def test_first_episode_stops_at_reset() -> None:
    # Arrange: 台 1 は 5 歩目で回が替わる
    T, E = 8, 2
    el = np.tile(np.arange(T, dtype=np.float32)[:, None], (1, E))
    el[5:, 1] = np.arange(3)
    data = {"episode_length": el, "door_dof_pos": np.zeros((T, E, 3)), "root_pos": np.zeros((T, E, 3)),
            "root_quat_xyzw": np.zeros((T, E, 4)), "dof_pos": np.zeros((T, E, 29)), "door_body_pos": np.zeros((T, E, 5, 3)),
            "grasp_target": np.zeros((T, E, 3)), "hand_lever_force": np.zeros((T, E, 3))}
    # Act / Assert
    assert len(l2b.first_episode(data, 0, 0.02).t) == T
    assert len(l2b.first_episode(data, 1, 0.02).t) == 5


def test_perturb_line_is_rigid() -> None:
    line = hda.door_line(0.94, True)
    rng = np.random.default_rng(1)
    p = l2b.perturb_line(line, np.zeros(2), 0.06, 2.0, rng)
    assert np.linalg.norm(p.along) == pytest.approx(1.0)
    assert float(p.along @ p.push) == pytest.approx(0.0, abs=1e-12)
    assert p.open_sign == line.open_sign


def test_fastlio_pose_is_held_and_lagged() -> None:
    ep = synthetic_episode(1.0)
    idx, perr, yerr = l2b.fastlio_pose(ep, 20, np.random.default_rng(0))
    # 0.1 s（5 歩）ごとに、0.1 s（5 歩）前の値を持つ
    assert list(idx[:12]) == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5]
    assert np.all(perr[:, 2] == 0.0)
    assert yerr[0] == 0.0 and math.isfinite(yerr[-1])


def test_compose_palms_without_error_equals_true_palms() -> None:
    # 誤差 0・今の歩の姿勢と順運動学なら、本当の手のひらと同じ
    import g1_arm_fk as fk
    chain = fk.load_chain()
    n = 6
    rng = np.random.default_rng(2)
    q = 0.3 * rng.standard_normal((n, 29))
    yaw = np.linspace(0, 0.5, n)
    quat = np.stack([np.zeros(n), np.zeros(n), np.sin(yaw / 2), np.cos(yaw / 2)], axis=1)
    ep = l2b.Episode(t=np.arange(n) * 0.02, hinge=np.zeros(n), lever=np.zeros(n), root_pos=rng.standard_normal((n, 3)),
                     root_quat=quat, q=q, panel=np.zeros((n, 3)), grasp=np.zeros((n, 3)), force=np.zeros(n))
    now = np.arange(n)
    got = l2b.compose_palms(ep, l2b.palms_pelvis(chain, ep), ep.root_pos, now, now, np.zeros(n))
    np.testing.assert_allclose(got, l2b.palms(chain, ep), atol=1e-12)


def with_latch(ep: l2b.Episode) -> l2b.Episode:
    """synthetic_episode に掛け金の部品（閉じた板の中心から戸先の側へ 0.45 m）を足す"""
    from dataclasses import replace
    latch = ep.panel + np.array([0.0, 0.45, 0.0])
    return replace(ep, latch=latch)


def test_nominal_line_agrees_with_fit_for_lever_left_door() -> None:
    # 把手が左の扉は右回りに開く（door_line の open_sign = −1）。名目の線は当てはめと同じ蝶番・向き・回り
    ep = with_latch(synthetic_episode(-1.0))
    fit, _ = l2b.fit_hinge(ep)
    nom = l2b.nominal_line(ep, 0.94, lever_left=True)
    chk = l2b.line_check(fit, nom)
    assert chk["hinge_m"] < 1e-9 and chk["along_deg"] < 1e-6 and chk["sign_ok"]


def test_lever_refs_first_is_fixed_and_closed_follows_truth() -> None:
    ep = synthetic_episode(1.0)
    bias = np.array([0.01, 0.0, 0.0])
    first = l2b.lever_refs(ep, bias, "first")
    assert first[0] is not None and all(r is None for r in first[1:])
    closed = l2b.lever_refs(ep, bias, "closed")
    # 扉が 1° を越えるまで（20 歩目から 0.4°/歩 → 22 歩目まで）だけ渡す
    assert [r is not None for r in closed[:25]] == [True] * 23 + [False] * 2


def test_score_and_summarize_count_early_and_false_fires() -> None:
    # Arrange: 開いた台（推定が本当より 1° 先行）と、開かなかった台（推定だけ 3° を越える）
    ep = synthetic_episode(1.0)
    est_open = ep.hinge + np.radians(np.where(np.arange(len(ep.t)) >= 20, 1.0, 0.0))
    shut = l2b.Episode(**{**ep.__dict__, "hinge": np.zeros(len(ep.t))})
    est_shut = np.radians(np.linspace(0, 4, len(ep.t)))
    # Act
    rows = [l2b.score(ep, 20, est_open), l2b.score(shut, 10, est_shut)]
    s = l2b.summarize(rows)
    # Assert
    assert rows[0]["true_at_est3"] == pytest.approx(2.0)  # 推定 3° の時に本当は 2°
    assert s["early_true_lt2"] == 0 and s["fired"] == 1 and s["episodes"] == 1
    assert s["band_err_deg_median"] == pytest.approx(1.0)
    assert s["unopened"] == 1 and s["unopened_fired"] == 1 and s["unopened_false3"] == 1


def test_eval_env_rng_does_not_depend_on_variant_list() -> None:
    # 場合を絞って回しても、同じ台・同じ場合の値は変わらない
    import g1_arm_fk as fk
    chain = fk.load_chain()
    ep = synthetic_episode(1.0)
    line, _ = l2b.fit_hinge(ep)
    both = l2b.eval_env(chain, ep, line, ep.panel[0, :2], 0, 3, ["ideal", "fastlio"], ["hinge"])
    one = l2b.eval_env(chain, ep, line, ep.panel[0, :2], 0, 3, ["fastlio"], ["hinge"])
    assert both["fastlio/hinge"] == one["fastlio/hinge"]
