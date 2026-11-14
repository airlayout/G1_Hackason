"""遠くから柱の面を求める（common/column_face.py）。

床・柱の面・後ろの壁を平面で置き、頭カメラ（床から約 1.2 m、下に 47.6°）から光線を飛ばして作った深度で確かめる
（雑音 1〜3 mm と、測れない画素 5% を入れる）。評価環境（MuJoCo）の画像はまだ使わない。
"""

from __future__ import annotations

import copy
import unittest
from typing import Any

import numpy as np

from _env import needs
from common.camera_geometry import R_LINK_OPTICAL, rpy_to_matrix
from common.column_face import ColumnFace, FaceNotFound, FaceTracker, find_face
from common.config import load_config

W, H, F = 640, 480, 605.0
K = np.array([[F, 0, 318.6], [0, F, 254.8], [0, 0, 1.0]])
CAM_POS = np.array([0.06, 0.02, 0.47])  # pelvis 座標。床（z −0.73）から 1.2 m
FLOOR_Z = -0.73
URDF_PITCH = 47.6
UP = np.array([0.0, 0.0, 1.0])


def camera(pitch_deg: float) -> np.ndarray:
    """光学座標 → pelvis 座標の回転（下に pitch_deg 傾いた頭カメラ）。"""
    return rpy_to_matrix(0.0, np.radians(pitch_deg), 0.0) @ R_LINK_OPTICAL


def render(dist: float, yaw_deg: float = 0.0, pitch_deg: float = URDF_PITCH, edges: tuple[float, float] = (-0.20, 0.25),
           wall_behind: float | None = 0.15, floor: bool = True, seed: int = 0) -> np.ndarray:
    """柱の面（pelvis から dist、向き yaw_deg、面に沿って edges [右, 左] の幅）と後ろの壁・床の深度 [m]。"""
    rot = camera(pitch_deg)
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - K[0, 2]) / F, (v - K[1, 2]) / F, np.ones((H, W))], axis=-1) @ rot.T  # 奥行き 1 あたり
    t_best = np.full((H, W), np.inf)
    f = np.array([np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg)), 0.0])  # 面に向かう向き
    side = np.cross(UP, f)

    def plane(n: np.ndarray, p0: np.ndarray, keep: Any = None) -> None:
        den = rays @ n
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((p0 - CAM_POS) @ n) / den
        hit = CAM_POS + rays * t[..., None]
        ok = (t > 0) & np.isfinite(t)
        if keep is not None:
            ok &= keep(hit)
        np.copyto(t_best, np.where(ok & (t < t_best), t, t_best))

    if floor:
        plane(UP, np.array([0, 0, FLOOR_Z]))
    foot = dist * f
    plane(-f, foot, lambda p: ((p - foot) @ side >= edges[0]) & ((p - foot) @ side <= edges[1]))
    if wall_behind is not None:
        plane(-f, foot + wall_behind * f)
    depth = np.where(np.isfinite(t_best), t_best, 0.0)
    rng = np.random.default_rng(seed)
    depth = depth + rng.normal(0, 0.001 + 0.002 * depth ** 2)
    depth[rng.random(depth.shape) < 0.05] = 0.0
    return depth.astype(np.float32)


SHIPPED = load_config("approach.yaml")["face"]
# 床で首の向きを直す仕組みのテストは、直す設定で行う（2026-11-14 から設定の既定は直さない。理由は configs/approach.yaml）
CFG = copy.deepcopy(SHIPPED)
CFG["floor"]["correct_pitch"] = True


def find(depth: np.ndarray, cfg: dict | None = None, pitch_deg: float = URDF_PITCH) -> ColumnFace:
    return find_face(depth, K, camera(pitch_deg), CAM_POS, UP, cfg or CFG)


class TestFindFace(unittest.TestCase):
    def test_straight_1m(self) -> None:
        face = find(render(1.0))
        self.assertAlmostEqual(face.distance_m, 1.0, delta=0.01)
        self.assertAlmostEqual(np.degrees(face.yaw_rad), 0.0, delta=0.5)
        self.assertTrue(face.floor_found)
        self.assertAlmostEqual(face.floor_z_m, FLOOR_Z, delta=0.01)
        self.assertAlmostEqual(face.left_edge_m, 0.25, delta=0.03)
        self.assertAlmostEqual(face.right_edge_m, -0.20, delta=0.03)
        # 後ろの壁は選ばない
        self.assertTrue(any(abs(d - 1.15) < 0.02 for d in face.others_m), face.others_m)

    def test_turned(self) -> None:
        for yaw in (-15.0, 10.0):
            with self.subTest(yaw=yaw):
                face = find(render(0.8, yaw_deg=yaw))
                self.assertAlmostEqual(face.distance_m, 0.8, delta=0.01)
                # 面が左を向いている（+）なら、左に回れば正対する
                self.assertAlmostEqual(np.degrees(face.yaw_rad), yaw, delta=0.7)

    def test_neck_pitch_corrected_by_floor(self) -> None:
        """実機の首は URDF より下を向いていた（11/3: 51〜54°）。床で直せば距離が合う。"""
        depth = render(1.0, pitch_deg=53.0)
        face = find(depth)
        self.assertAlmostEqual(face.pitch_correction_deg, 53.0 - URDF_PITCH, delta=0.5)
        self.assertAlmostEqual(face.distance_m, 1.0, delta=0.015)
        self.assertAlmostEqual(np.degrees(face.yaw_rad), 0.0, delta=0.5)
        # 直さなければ数 cm ずれる（直す意味がある）
        cfg = copy.deepcopy(CFG)
        cfg["floor"]["correct_pitch"] = False
        raw = find(depth, cfg)
        self.assertGreater(abs(raw.distance_m - 1.0), 0.03)

    def test_too_large_pitch_error_stops(self) -> None:
        with self.assertRaises(FaceNotFound):
            find(render(1.0, pitch_deg=URDF_PITCH + 20))

    def test_near_without_floor(self) -> None:
        """近いと床が写らない。直さずに求める。"""
        face = find(render(0.45, edges=(-3.0, 3.0), wall_behind=None))  # 柱が画像いっぱい（横から床も見えない）
        self.assertFalse(face.floor_found)
        self.assertAlmostEqual(face.distance_m, 0.45, delta=0.01)

    def test_near_uses_previous_floor_correction(self) -> None:
        """首が 53° のとき、近く（床なし）では直せず 2〜4 cm 遠く出る。遠くで床から求めた直し方を渡せば合う。"""
        far = find(render(0.8, pitch_deg=53.0))
        self.assertEqual(far.correction_from, "floor")
        near_depth = render(0.45, pitch_deg=53.0, edges=(-3.0, 3.0), wall_behind=None)  # 柱が画像いっぱい（横から床も見えない）
        raw = find(near_depth)
        self.assertEqual(raw.correction_from, "none")
        self.assertGreater(raw.distance_m - 0.45, 0.015)
        fixed = find_face(near_depth, K, camera(URDF_PITCH), CAM_POS, UP, CFG, far.correction)
        self.assertEqual(fixed.correction_from, "previous")
        self.assertAlmostEqual(fixed.distance_m, 0.45, delta=0.01)
        self.assertAlmostEqual(fixed.pitch_correction_deg, 53.0 - URDF_PITCH, delta=0.5)

    def test_narrow_floor_is_not_used(self) -> None:
        """近いと、柱の横に見える床は狭い。狭い床からは直さず（法線が数度ずれる）、遠くで広い床から求めた直し方を使う。"""
        near_depth = render(0.45, pitch_deg=53.0)
        self.assertNotEqual(find(near_depth).correction_from, "floor")
        far = find(render(0.9, pitch_deg=53.0))
        self.assertEqual(far.correction_from, "floor")
        face = find_face(near_depth, K, camera(URDF_PITCH), CAM_POS, UP, CFG, far.correction)
        self.assertEqual(face.correction_from, "previous")
        self.assertAlmostEqual(face.distance_m, 0.45, delta=0.01)

    def test_floor_strip_and_wall_bottom(self) -> None:
        """床が画像の下の細い帯にしか写らなくても、床の帯と壁の下をまたぐ傾いた平面を床と取り違えない（評価環境で起きた）。"""
        for pitch in (URDF_PITCH, 53.0):
            with self.subTest(pitch=pitch):
                face = find(render(0.62, pitch_deg=pitch, edges=(-3.0, 3.0), wall_behind=None))
                self.assertAlmostEqual(face.distance_m, 0.62, delta=0.01)
                self.assertAlmostEqual(np.degrees(face.yaw_rad), 0.0, delta=0.7)
                if face.floor_found:
                    self.assertAlmostEqual(face.pitch_correction_deg, pitch - URDF_PITCH, delta=0.7)

    def test_far(self) -> None:
        face = find(render(1.5, yaw_deg=5.0))
        self.assertAlmostEqual(face.distance_m, 1.5, delta=0.02)
        self.assertAlmostEqual(np.degrees(face.yaw_rad), 5.0, delta=1.0)

    def test_wide_face_has_no_edges(self) -> None:
        """面が画像の左右に切れていれば、端は分からない。"""
        face = find(render(0.8, edges=(-3.0, 3.0), wall_behind=None))
        self.assertAlmostEqual(face.distance_m, 0.8, delta=0.01)
        self.assertIsNone(face.left_edge_m)
        self.assertIsNone(face.right_edge_m)

    def test_flush_wall_edges_unknown_but_distance_ok(self) -> None:
        """柱が壁と同じ面（出っ張らない）なら、端は分からないが距離は合う。"""
        face = find(render(0.9, wall_behind=0.0))
        self.assertAlmostEqual(face.distance_m, 0.9, delta=0.01)
        self.assertIsNone(face.left_edge_m)

    def test_only_floor(self) -> None:
        with self.assertRaises(FaceNotFound):
            find(render(1.0, edges=(5.0, 6.0), wall_behind=None))

    def test_no_depth(self) -> None:
        with self.assertRaises(FaceNotFound):
            find(np.zeros((H, W), np.float32))


class TestFaceTracker(unittest.TestCase):
    def face(self, d: float, yaw_deg: float = 0.0, left: float | None = 0.25) -> ColumnFace:
        return ColumnFace(normal=np.array([-1.0, 0, 0]), point=np.array([d, 0, 0]), distance_m=d,
                          yaw_rad=np.radians(yaw_deg), left_edge_m=left, right_edge_m=-0.2, n_points=1000,
                          floor_found=True, pitch_correction_deg=0.0, floor_z_m=FLOOR_Z)

    def test_median_when_steady(self) -> None:
        tr = FaceTracker(CFG["tracking"])
        self.assertIsNone(tr.add(self.face(1.000)))
        self.assertIsNone(tr.add(self.face(1.004, 0.5, None)))
        out = tr.add(self.face(1.002, 0.2))
        self.assertIsNotNone(out)
        self.assertAlmostEqual(out.distance_m, 1.002)
        self.assertAlmostEqual(out.left_edge_m, 0.25)  # 3 枚中 2 枚で分かった

    def test_outlier_is_tolerated(self) -> None:
        """直近 5 枚のうち 1 枚だけ外れていても、残りがそろえばまとめる（乱しありの評価環境で、そろわずに止まった）。"""
        tr = FaceTracker(CFG["tracking"])
        self.assertIsNone(tr.add(self.face(0.744, -4.7)))
        self.assertIsNone(tr.add(self.face(0.727, 0.2)))
        out = tr.add(self.face(0.727, 0.1))
        self.assertIsNone(out)  # 3 枚のうち 2 枚しかそろっていない
        self.assertIn("ばらつく", tr.last_reject)
        out = tr.add(self.face(0.728, 0.1))
        self.assertIsNotNone(out)
        self.assertAlmostEqual(out.distance_m, 0.727, places=3)

    def test_waits_when_moving(self) -> None:
        tr = FaceTracker(CFG["tracking"])
        tr.add(self.face(1.00))
        tr.add(self.face(0.97))
        self.assertIsNone(tr.add(self.face(0.97)))  # 1 cm より大きくばらつく
        self.assertIsNotNone(tr.add(self.face(0.971)))  # 古いものを捨てて、そろった


@needs("pin", "models")
class TestFaceFinder(unittest.TestCase):
    def test_from_config(self) -> None:
        """URDF の頭カメラ（腰 0、補正なし）でも、合成の深度で距離が合う。"""
        from common.column_face import FaceFinder

        finder = FaceFinder.from_config(load_config("robot.yaml"), CFG, offset=np.zeros(3))
        rot, pos = finder.cam_tf.pelvis_from_optical(np.zeros(3))
        global CAM_POS
        saved = CAM_POS
        try:
            CAM_POS = pos
            pitch = float(np.degrees(np.arctan2(-rot[2, 2], rot[0, 2])))  # 光軸の下向きの角度
            depth = render(1.0, pitch_deg=pitch)
        finally:
            CAM_POS = saved
        face = finder.find(depth, K, np.zeros(3))
        self.assertAlmostEqual(face.distance_m, 1.0, delta=0.01)


class TestShippedConfig(unittest.TestCase):
    def test_default_does_not_correct_pitch(self) -> None:
        """既定は床で直さない（ボタンと同じ URDF の向き）。首が 53° でも直さず、床は探すだけ。"""
        self.assertFalse(SHIPPED["floor"]["correct_pitch"])
        face = find(render(1.0, pitch_deg=53.0), SHIPPED)
        self.assertEqual(face.correction_from, "none")
        self.assertEqual(face.pitch_correction_deg, 0.0)
        self.assertTrue(face.floor_found)


if __name__ == "__main__":
    unittest.main()
