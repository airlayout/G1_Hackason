"""stereo_core・scene_calib の部品の試し（合成のデータ。GPU・スマホ・COLMAP は要らない）"""

from __future__ import annotations

import pathlib
import sys

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from run_stereo_slam import droid_size  # noqa: E402
from scene_calib import SceneCollector  # noqa: E402
from stereo_core import (  # noqa: E402
    epipolar_dy,
    estimate_offset,
    estimate_offset_xcorr,
    load_calib,
    rectified_intrinsics,
    rectify,
    robust_relative_pose,
    save_calib,
)

SIZE = (1280, 720)
K1 = np.array([[820.0, 0, 640.0], [0, 820.0, 362.0], [0, 0, 1]])
K2 = np.array([[905.0, 0, 633.0], [0, 905.0, 355.0], [0, 0, 1]])  # 右は別の機種（画角が違う）
D0 = np.zeros(5)
BASELINE = 0.12
R_TRUE = Rotation.from_euler("xyz", [0.4, -1.2, 0.3], degrees=True).as_matrix()  # 2 台の小さな向きの違い
T_TRUE = np.array([-BASELINE, 0.003, -0.002])  # 右は左の +x（X_r = R X_l + T）


def _project_pairs(n: int = 400, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """部屋の点を 2 台に写した、同じ点の画素の組"""
    rng = np.random.default_rng(seed)
    X = np.column_stack([rng.uniform(-1.5, 1.5, n), rng.uniform(-1.0, 1.0, n), rng.uniform(1.0, 4.0, n)])
    pl, _ = cv2.projectPoints(X, np.zeros(3), np.zeros(3), K1, D0)
    pr, _ = cv2.projectPoints(X, cv2.Rodrigues(R_TRUE)[0], T_TRUE, K2, D0)
    return pl.reshape(-1, 2), pr.reshape(-1, 2)


class TestRectify:
    def test_true_extrinsics_give_zero_vertical_disparity(self) -> None:
        c = rectify(K1, D0, K2, D0, R_TRUE, T_TRUE, SIZE)
        lp, rp = _project_pairs()
        assert c["swapped"] is False
        assert np.median(epipolar_dy(c, lp, rp)) < 1e-3
        assert c["baseline_m"] == pytest.approx(np.linalg.norm(T_TRUE))

    def test_rectified_right_projection_encodes_baseline(self) -> None:
        c = rectify(K1, D0, K2, D0, R_TRUE, T_TRUE, SIZE)
        fx, _, _, _ = rectified_intrinsics(c)
        assert -c["P2"][0, 3] / fx == pytest.approx(np.linalg.norm(T_TRUE), rel=1e-6)

    def test_swapped_sides_are_detected_and_still_rectify(self) -> None:
        # 右に置いたつもりの方を左として渡す（X_l = R^T X_r - R^T T）
        Rs, Ts = R_TRUE.T, -R_TRUE.T @ T_TRUE
        c = rectify(K2, D0, K1, D0, Rs, Ts, SIZE)
        lp, rp = _project_pairs()
        assert c["swapped"] is True
        assert c["T"][0] < 0
        assert np.median(epipolar_dy(c, lp, rp)) < 1e-3  # 入れ替えた後は元の左右で合う

    def test_small_rotation_error_shows_up_as_vertical_disparity(self) -> None:
        R_bad = Rotation.from_euler("x", 0.5, degrees=True).as_matrix() @ R_TRUE
        c = rectify(K1, D0, K2, D0, R_bad, T_TRUE, SIZE)
        lp, rp = _project_pairs()
        assert np.median(epipolar_dy(c, lp, rp)) > 2.0  # 0.5 度の狂いで数画素ずれる＝計器が効く

    def test_save_and_load_roundtrip(self, tmp_path: pathlib.Path) -> None:
        c = rectify(K1, D0, K2, D0, R_TRUE, T_TRUE, SIZE)
        save_calib(tmp_path / "c.json", c, {"left_stream": "cam1"})
        back = load_calib(tmp_path / "c.json")
        assert np.allclose(back["P1"], c["P1"])
        assert back["left_stream"] == "cam1"


class TestRelativePose:
    def test_average_ignores_outliers(self) -> None:
        rng = np.random.default_rng(2)
        Rs, ts = [], []
        for _ in range(20):
            noise = Rotation.from_rotvec(rng.normal(0, np.radians(0.05), 3)).as_matrix()
            Rs.append(noise @ R_TRUE)
            ts.append(T_TRUE * 7.3 + rng.normal(0, 0.002, 3))  # COLMAP の大きさ（7.3 倍）
        Rs.append(Rotation.from_euler("y", 10, degrees=True).as_matrix())  # 外れ
        ts.append(np.array([3.0, 0, 0]))
        R, t, n, spread = robust_relative_pose(Rs, ts)
        assert n == 20
        assert spread["rot_spread_deg"] < 0.2  # 固定された 2 台
        assert np.degrees(Rotation.from_matrix(R.T @ R_TRUE).magnitude()) < 0.05
        assert t / np.linalg.norm(t) == pytest.approx(T_TRUE / np.linalg.norm(T_TRUE), abs=2e-3)

    def test_loose_rig_is_reported_not_hidden(self) -> None:
        # 2 台が互いに 5〜10 度動いた（11-17 の実機と同じ形）。全部の組に逃がして平均を壊さない
        rng = np.random.default_rng(3)
        Rs = [Rotation.from_rotvec(rng.normal(0, np.radians(6), 3)).as_matrix() @ R_TRUE for _ in range(28)]
        ts = [T_TRUE * 7.3 + rng.normal(0, 0.3, 3) for _ in range(28)]
        Rs.append(R_TRUE)
        ts.append(np.array([2000.0, 0, 0]))  # 登録に失敗した 1 組
        try:
            _, t, n, spread = robust_relative_pose(Rs, ts)
        except ValueError:
            return  # 残りが少なすぎると止まるのも正しい
        assert spread["rot_spread_deg"] > 1.0  # 固定されていないと分かる
        assert np.linalg.norm(t) < 100  # 外れた 1 組に引きずられていない

    def test_no_pairs_is_an_error(self) -> None:
        with pytest.raises(ValueError):
            robust_relative_pose([], [])


class TestSceneCollector:
    def _scene(self, seed: int) -> np.ndarray:
        rng = np.random.default_rng(seed)
        img = (rng.random((360, 640)) * 255).astype(np.uint8)
        img = cv2.GaussianBlur(cv2.resize(img, (1280, 720)), (5, 5), 0)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    def test_takes_only_when_still_textured_and_new(self) -> None:
        col = SceneCollector(want=3)
        a, b = self._scene(1), self._scene(2)
        assert col.offer(a, a, 0.0) == "hold still"  # 1 回目は比べる物が無い
        assert col.offer(a, a, 0.3) == "taken"
        assert col.offer(a, a, 1.5) == "move to a new view"  # 同じ景色
        assert col.offer(b, b, 2.0) == "hold still"  # 動いた直後
        assert col.offer(b, b, 2.3) == "taken"

    def test_blank_wall_is_rejected(self) -> None:
        col = SceneCollector(want=3)
        blank = np.full((720, 1280, 3), 180, np.uint8)
        col.offer(blank, blank, 0.0)
        assert col.offer(blank, blank, 0.3).startswith("not enough texture")


class TestDroidSize:
    def test_landscape_and_portrait_are_multiples_of_8(self) -> None:
        assert droid_size(1280, 720, 512) == (512, 288)
        assert droid_size(720, 1280, 512) == (288, 512)

    def test_odd_aspect_rounds_to_8(self) -> None:
        w, h = droid_size(1000, 333, 512)
        assert w % 8 == 0 and h % 8 == 0 and w == 512


class TestSync:
    def test_estimates_known_offset_from_flashes(self) -> None:
        rng = np.random.default_rng(1)
        edges = np.cumsum(rng.uniform(0.7, 1.5, 30))
        state = lambda t: (np.searchsorted(edges, t) % 2).astype(float) * 200 + 20  # noqa: E731

        def stream(latency: float, phase: float) -> tuple[np.ndarray, np.ndarray]:
            cap = np.arange(phase, edges[-1], 1 / 30) + rng.normal(0, 0.002, 1)
            arrive = cap + latency + rng.normal(0, 0.004, cap.shape)
            return arrive, state(cap)

        tl, vl = stream(0.120, 0.0)
        tr, vr = stream(0.203, 0.011)
        r = estimate_offset(tl, vl, tr, vr)
        assert r["n_events"] >= 25
        assert r["offset_s"] == pytest.approx(0.083, abs=0.012)

    def test_no_flash_gives_no_events(self) -> None:
        t = np.arange(0, 5, 1 / 30)
        v = np.full_like(t, 100.0)
        assert estimate_offset(t, v, t, v)["n_events"] == 0


class TestSyncByMotion:
    def _motion(self, seed: int = 4) -> tuple:
        rng = np.random.default_rng(seed)
        t = np.arange(0, 25, 0.001)
        energy = np.convolve((rng.random(len(t)) < 0.0015).astype(float), np.hanning(400), "same")  # ときどき振る
        return t, energy, rng

    def test_estimates_known_offset_from_motion(self) -> None:
        t, energy, rng = self._motion()

        def stream(latency: float, phase: float):
            cap = np.arange(phase, 24.5, 1 / 30)
            return cap + latency + rng.normal(0, 0.004, cap.shape), np.interp(cap, t, energy) + rng.normal(0, 0.02, cap.shape)

        tl, vl = stream(0.110, 0.0)
        tr, vr = stream(0.193, 0.013)
        r = estimate_offset_xcorr(tl, vl, tr, vr)
        assert r["ok"]
        assert r["offset_s"] == pytest.approx(0.083, abs=0.012)

    def test_unrelated_signals_are_not_ok(self) -> None:
        rng = np.random.default_rng(5)
        tl = np.arange(0, 20, 1 / 30)
        r = estimate_offset_xcorr(tl, rng.random(len(tl)), tl, rng.random(len(tl)))
        assert not r["ok"]
