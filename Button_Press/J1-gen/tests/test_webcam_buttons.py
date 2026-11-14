"""前方の Web カメラでボタンの位置を推定する（common/webcam_buttons.py）と、寄る流れへのつなぎ（common/approach.py の WebcamAssist）。

合成のピンホールカメラ（胸の高さ、少し下向き）でボタンを写した画素で確かめる。実機の Web カメラ・評価環境は使わない
（評価環境には Web カメラが無い）。
"""

from __future__ import annotations

import copy
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from common.approach import Measurement, WebcamAssist, decide
from common.column_face import ColumnFace
from common.config import FEATURE_DIR, load_config
from common.elevator_buttons import Candidate2D
from common.loco import LocoMover
from common.webcam_buttons import (
    PairPx,
    WebcamModel,
    WebcamNotReady,
    find_pair,
    fit_camera,
    from_plane,
    from_size,
    looming_range,
    triangulate,
)

CFG = load_config("webcam.yaml")
APP = load_config("approach.yaml")
CAM = WebcamModel(fx=500.0, fy=500.0, cx=239.5, cy=319.5, xyz=np.array([0.10, 0.02, 0.30]),
                  rpy=np.radians([0.0, 8.0, 0.0]))
UP_Z, DOWN_Z = 0.21, 0.14  # 車いす用の ▲▼ の高さ（pelvis から）
STAND_R = {"min_dist_m": 0.30, "max_dist_m": 0.50, "target_dist_m": 0.40, "center_side_m": -0.14, "max_side_m": 0.02}


def view(cam: WebcamModel, d: float, y: float, noise_px: float = 0.0, seed: int = 0) -> PairPx:
    """柱の面（正対、pelvis から d）の左右 y にある ▲▼ を写した画素の組。直径は「焦点距離 × 3.5 cm ÷ 距離」。"""
    rng = np.random.default_rng(seed)
    out = []
    for z in (UP_Z, DOWN_Z):
        p = np.array([d, y, z])
        uv = cam.project(p) + noise_px * rng.standard_normal(2)
        diam = cam.fx * 0.035 / np.linalg.norm(p - cam.xyz) + noise_px * rng.standard_normal()
        out.append((uv, diam))
    return PairPx(out[0][0], out[1][0], out[0][1], out[1][1])


def wall(d: float) -> tuple[np.ndarray, np.ndarray]:
    return np.array([-1.0, 0.0, 0.0]), np.array([d, 0.0, 0.0])


class TestModel(unittest.TestCase):
    def test_project_and_ray(self) -> None:
        p = np.array([0.9, -0.12, 0.17])
        o, d = CAM.ray(CAM.project(p))
        np.testing.assert_allclose(o + d * np.linalg.norm(p - o), p, atol=1e-9)

    def test_uncalibrated(self) -> None:
        with self.assertRaises(WebcamNotReady):
            WebcamModel.from_config(CFG)  # fx・xyz が null のうちは使わない
        cfg = copy.deepcopy(CFG)
        cfg["camera"].update(fx=500.0, xyz=[0.1, 0.0, 0.3])
        cam = WebcamModel.from_config(cfg)
        self.assertEqual((cam.fy, cam.cx, cam.cy), (500.0, 239.5, 319.5))


class TestEstimates(unittest.TestCase):
    def test_plane(self) -> None:
        pair = view(CAM, 1.0, -0.10)
        p = from_plane(CAM, pair.mid_px, *wall(1.0))
        np.testing.assert_allclose(p, [1.0, -0.10, (UP_Z + DOWN_Z) / 2], atol=1e-3)  # 画素の中点と 3 次元の中点の差

    def test_plane_with_mount_error(self) -> None:
        """取り付けの向きが 1° ずれていても、柱の面との交点の左右のずれは 1 m 先で約 2 cm。"""
        pair = view(CAM, 1.0, -0.10)
        wrong = WebcamModel(CAM.fx, CAM.fy, CAM.cx, CAM.cy, CAM.xyz, CAM.rpy + np.radians([0, 0, 1.0]))
        p = from_plane(wrong, pair.mid_px, *wall(1.0))
        self.assertAlmostEqual(p[1], -0.10, delta=0.02)
        self.assertAlmostEqual(p[0], 1.0, delta=1e-6)  # 距離は頭カメラの面が決める

    def test_size(self) -> None:
        pair = view(CAM, 1.0, -0.10)
        est = from_size(CAM, pair, CFG)
        for k in ("by_diameter", "by_spacing"):
            with self.subTest(k=k):
                self.assertAlmostEqual(est[k][0], 1.0, delta=0.03)  # 斜めから見るので、間隔はわずかに縮んで見える
        # 中心と直径に ±2 画素の雑音（1 m 先で直径は約 17 画素、間隔は約 34 画素）: どちらも距離の誤差の中央値は 4〜5%
        errs = {"by_diameter": [], "by_spacing": []}
        for seed in range(50):
            e = from_size(CAM, view(CAM, 1.0, -0.10, noise_px=2.0, seed=seed), CFG)
            for k in errs:
                errs[k].append(abs(e[k][0] - 1.0))
        for k in errs:
            self.assertLess(np.median(errs[k]), 0.08, k)

    def test_parallax_sideways(self) -> None:
        """横に 20 cm 動いた前後で見ると、2 本の線の交点で位置が出る（動いた量が正しければ数 mm）。"""
        before = view(CAM, 1.0, -0.10)
        after = view(CAM, 1.0, -0.30)  # 体が左へ 20 cm → ボタンは体から見て右へ
        p, gap = triangulate(CAM, before.mid_px, after.mid_px, (0.0, 0.20, 0.0))
        # ▲▼ の画素の中点は、遠近のため 3 次元の中点を写した点と 1 mm ほど違う
        np.testing.assert_allclose(p, [1.0, -0.30, (UP_Z + DOWN_Z) / 2], atol=1e-3)
        self.assertLess(gap, 1e-6)
        # 動いた量を 2 cm 間違えると、距離が約 9 cm ずれる（動いた量は頭カメラで測るので、その精度が効く）
        p2, _ = triangulate(CAM, before.mid_px, after.mid_px, (0.0, 0.22, 0.0))
        self.assertGreater(abs(p2[0] - 1.0), 0.05)

    def test_parallax_forward_is_weak(self) -> None:
        """前にまっすぐ 20 cm 動いただけだと、ほぼ同じ線の上を見るので弱い: 画素が縦に 2 画素ずれると距離が約 6 cm ずれる
        （柱の面との交点なら、2 画素のずれは 1 m 先で約 4 mm）。"""
        before = view(CAM, 1.0, -0.02)
        after = view(CAM, 0.8, -0.02)
        p, _ = triangulate(CAM, before.mid_px, after.mid_px, (0.20, 0.0, 0.0))
        self.assertAlmostEqual(p[0], 0.8, delta=0.01)
        res = triangulate(CAM, before.mid_px, after.mid_px + np.array([0.0, 2.0]), (0.20, 0.0, 0.0))
        self.assertTrue(res is None or abs(res[0][0] - 0.8) > 0.04)
        q = from_plane(CAM, after.mid_px + np.array([0.0, 2.0]), *wall(0.8))
        self.assertLess(np.linalg.norm(q[:2] - [0.8, -0.02]), 0.01)

    def test_looming(self) -> None:
        r0 = np.linalg.norm(np.array([1.0, -0.02, 0.175]) - CAM.xyz)
        r1 = np.linalg.norm(np.array([0.8, -0.02, 0.175]) - CAM.xyz)
        d0, d1 = CAM.fx * 0.035 / r0, CAM.fx * 0.035 / r1
        self.assertAlmostEqual(looming_range(d0, d1, r0 - r1), r1, places=6)
        self.assertIsNone(looming_range(20.0, 20.0, 0.2))


def cand(uv: np.ndarray, diam: float, arrow: str | None) -> Candidate2D:
    r = diam / 2
    return Candidate2D((int(uv[0] - r), int(uv[1] - r), int(uv[0] + r), int(uv[1] + r)), np.zeros((1, 1), bool), arrow)


class TestFindPair(unittest.TestCase):
    def test_picks_lowest_pair_for_wheelchair(self) -> None:
        """柱に ▲▼（一般用）、車いすの印、▲▼（車いす用）が縦に並ぶ。車いす用は、条件に合う組のうち画像でいちばん下。"""
        pr = view(CAM, 0.9, -0.10)
        lift = np.array([0.0, -120.0])  # 一般用は画像でずっと上
        cands = [cand(pr.up_px + lift, pr.up_diam_px, "up"), cand(pr.down_px + lift, pr.down_diam_px, "down"),
                 cand(pr.up_px, pr.up_diam_px, "up"), cand(pr.down_px, pr.down_diam_px, "down")]
        got = find_pair(cands, CFG)
        np.testing.assert_allclose(got.mid_px, pr.mid_px, atol=1.0)
        general = copy.deepcopy(CFG)
        general["buttons"]["button_set"] = "general"
        np.testing.assert_allclose(find_pair(cands, general).mid_px, pr.mid_px + lift, atol=1.0)

    def test_rejects(self) -> None:
        pr = view(CAM, 0.9, -0.10)
        far = PairPx(pr.up_px, pr.up_px + 3 * (pr.down_px - pr.up_px), pr.up_diam_px, pr.down_diam_px)
        self.assertIsNone(find_pair([cand(far.up_px, far.up_diam_px, "up"), cand(far.down_px, far.down_diam_px, "down")], CFG))
        self.assertIsNone(find_pair([cand(pr.up_px, pr.up_diam_px, "down"), cand(pr.down_px, pr.down_diam_px, "up")], CFG))


class TestFit(unittest.TestCase):
    def test_recovers_focal_and_angles(self) -> None:
        """頭カメラで測ったボタンの位置と Web カメラの画素の組から、焦点距離と向きを求める（位置は巻き尺の値のまま）。"""
        rng = np.random.default_rng(0)
        pairs = []
        for _ in range(12):
            p = np.array([rng.uniform(0.35, 0.5), rng.uniform(-0.25, 0.1), rng.uniform(0.12, 0.23)])
            pairs.append((p, CAM.project(p) + 0.5 * rng.standard_normal(2)))
        guess = WebcamModel(420.0, 420.0, CAM.cx, CAM.cy, CAM.xyz, np.radians([1.0, 3.0, -4.0]))
        fit, rms = fit_camera(pairs, guess)
        self.assertAlmostEqual(fit.fx, CAM.fx, delta=CAM.fx * 0.02)
        np.testing.assert_allclose(np.degrees(fit.rpy), np.degrees(CAM.rpy), atol=0.5)
        self.assertLess(rms, 1.0)


class TestFitCommand(unittest.TestCase):
    def test_fit_prints_config(self) -> None:
        """real/webcam_check.py --fit: 集めた組のファイルから、webcam.yaml に書く値を出す（設定は書き換えない）。"""
        rng = np.random.default_rng(1)
        rows = []
        for _ in range(10):
            p = np.array([rng.uniform(0.35, 0.5), rng.uniform(-0.25, 0.1), rng.uniform(0.12, 0.23)])
            rows.append({"p_pelvis": p.tolist(), "uv": (CAM.project(p) + 0.3 * rng.standard_normal(2)).tolist()})
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp) / "pairs.json"
            f.write_text(json.dumps({"xyz": CAM.xyz.tolist(), "pairs": rows}))
            r = subprocess.run([sys.executable, str(FEATURE_DIR / "real" / "webcam_check.py"), "--fit", str(f), "--fx", "420"],
                               capture_output=True, text=True, timeout=120)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        fx = float(r.stdout.split("fx: ")[1].split()[0])
        self.assertAlmostEqual(fx, CAM.fx, delta=CAM.fx * 0.02)
        self.assertIn("rpy_deg:", r.stdout)


class FakeDetector:
    def __init__(self, pair: PairPx | None) -> None:
        self.pair = pair

    def detect(self, rgb: np.ndarray) -> list[Candidate2D]:
        if self.pair is None:
            return []
        return [cand(self.pair.up_px, self.pair.up_diam_px, "up"), cand(self.pair.down_px, self.pair.down_diam_px, "down")]


def face(d: float) -> ColumnFace:
    n, pt = wall(d)
    return ColumnFace(normal=n, point=pt, distance_m=d, yaw_rad=0.0, left_edge_m=None, right_edge_m=None, n_points=1000,
                      floor_found=True, pitch_correction_deg=0.0, floor_z_m=-0.73)


class TestAssist(unittest.TestCase):
    def assist(self, pair: PairPx | None, age: float = 0.1, cfg: dict | None = None) -> WebcamAssist:
        return WebcamAssist(lambda: (np.zeros((640, 480, 3), np.uint8), age), FakeDetector(pair), CAM, cfg or CFG)

    def test_side_hint_moves_sideways_while_far(self) -> None:
        """面しか見えない 90 cm でも、Web カメラでボタンが体の左 5 cm と分かれば、左へ寄る（目安は右 14 cm）。"""
        est = self.assist(view(CAM, 0.9, 0.05)).estimate(face(0.9))
        self.assertAlmostEqual(est["use"][1], 0.05, delta=3e-3)
        m = Measurement("face", 0.9, 0.0, None, face=face(0.9), side_hint_m=est["use"][1], side_hint_source="webcam",
                        webcam=est)
        mover = LocoMover(None, None, APP["loco"], go=False, log=lambda s: None)
        d = decide(m, None, {}, {"right": STAND_R}, APP["approach"], mover)
        self.assertEqual(d.pulse.axis, "left")
        self.assertIn("Web カメラ", d.reason)

    def test_parallax_is_recorded(self) -> None:
        a = self.assist(view(CAM, 1.0, 0.05))
        f0 = ColumnFace(normal=np.array([-1.0, 0, 0]), point=np.array([1.0, 0, 0]), distance_m=1.0, yaw_rad=0.0,
                        left_edge_m=0.30, right_edge_m=-0.20, n_points=1000, floor_found=True, pitch_correction_deg=0.0,
                        floor_z_m=-0.73)
        a.estimate(f0)
        a.detector = FakeDetector(view(CAM, 1.0, -0.15))  # 体が左へ 20 cm
        f1 = ColumnFace(normal=np.array([-1.0, 0, 0]), point=np.array([1.0, 0, 0]), distance_m=1.0, yaw_rad=0.0,
                        left_edge_m=0.10, right_edge_m=-0.40, n_points=1000, floor_found=True, pitch_correction_deg=0.0,
                        floor_z_m=-0.73)
        est = a.estimate(f1)
        np.testing.assert_allclose(est["parallax"][:2], [1.0, -0.15], atol=3e-3)

    def test_rejects_disagreement_and_old_frames(self) -> None:
        cfg = copy.deepcopy(CFG)
        cfg["buttons"]["spacing_m"] = 0.14  # 間隔を 2 倍と思っている → 距離が 2 倍に出て、交点と食い違う
        cfg["buttons"]["diameter_m"] = 0.07
        est = self.assist(view(CAM, 0.9, 0.05), cfg=cfg).estimate(face(0.9))
        self.assertIsNone(est["use"])
        self.assertIn("食い違う", est["reason"])
        a = self.assist(view(CAM, 0.9, 0.05), age=2.0)
        self.assertIsNone(a.estimate(face(0.9)))
        self.assertIn("古い", a.last_reason)
        b = self.assist(None)
        self.assertIsNone(b.estimate(face(0.9)))


if __name__ == "__main__":
    unittest.main()
