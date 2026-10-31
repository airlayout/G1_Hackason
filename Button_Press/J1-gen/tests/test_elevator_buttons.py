"""エレベーターの呼びボタンを見つける（common/elevator_buttons.py）と、押す計画（common/elevator_press.py）。

見つける部分は、正面の平らな柱に灰色の丸と黒い三角を描いた作り物の画像と深度で確かめる
（カメラは柱にまっすぐ向いている。1 cm ≈ 18.75 画素）。
"""

from __future__ import annotations

import unittest

import numpy as np

from _env import needs
from common.config import load_config
from common.elevator_buttons import (
    Button3D,
    ButtonFinder,
    ButtonNotFound,
    ButtonTracker,
    CallButtons,
    ClassicButtonDetector,
    read_arrow,
    select_call_pair,
)

W, H, F = 640, 480, 600.0
WALL_X = 0.32  # 柱の面までの距離 [m]
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])
PX_PER_M = F / WALL_X


class FacingCamera:
    """柱にまっすぐ向いたカメラ（光学座標: x 右・y 下・z 前 → pelvis: x 前・y 左・z 上）。"""

    def pelvis_from_optical(self, q_waist: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return np.array([[0.0, 0, 1], [-1, 0, 0], [0, -1, 0]]), np.zeros(3)


def scene(buttons: list[tuple[float, float, str]], diameter: float = 0.035) -> tuple[np.ndarray, np.ndarray]:
    """buttons: (左右 y [m], 上下 z [m], "up" / "down") の並び。黒い柱に灰色の丸と黒い三角を描く。"""
    import cv2

    rgb = np.full((H, W, 3), 15, np.uint8)
    depth = np.full((H, W), WALL_X, np.float32)
    r = int(round(diameter / 2 * PX_PER_M))
    for y, z, arrow in buttons:
        u, v = int(round(W / 2 - y * PX_PER_M)), int(round(H / 2 - z * PX_PER_M))
        cv2.circle(rgb, (u, v), r, (90, 90, 90), -1)
        cv2.circle(depth, (u, v), r, WALL_X - 0.002, -1)  # 2 mm 出っ張っている
        s = int(r * 0.45)
        tri = [(u - s, v + s // 2), (u + s, v + s // 2), (u, v - s)] if arrow == "up" else \
              [(u - s, v - s // 2), (u + s, v - s // 2), (u, v + s)]
        cv2.fillPoly(rgb, [np.array(tri, np.int32)], (20, 20, 20))
    return rgb, depth


class TestFindButtons(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.cfg = load_config("elevator.yaml")
        cls.finder = ButtonFinder(FacingCamera(), ClassicButtonDetector(cls.cfg["classic"]), cls.cfg)

    def find(self, buttons: list[tuple[float, float, str]]) -> CallButtons:
        rgb, depth = scene(buttons)
        return self.finder.find(rgb, depth, K, np.zeros(3))

    def test_reads_arrows(self) -> None:
        rgb, _ = scene([(0.0, 0.0, "up"), (0.1, 0.0, "down")])
        gray = rgb.astype(float).mean(axis=2)
        cands = sorted(ClassicButtonDetector(self.cfg["classic"]).detect(rgb), key=lambda c: c.bbox[0])
        self.assertEqual(len(cands), 2)
        # 画像の左が pelvis の +y（▲ を y = 0.1 の左側に置いた）
        self.assertEqual([c.arrow for c in cands], ["down", "up"])
        self.assertEqual(read_arrow(gray, cands[0].bbox), "down")

    def test_finds_call_pair_and_normal(self) -> None:
        call = self.find([(0.0, 0.10, "up"), (0.0, 0.02, "down"), (0.0, -0.12, "up")])
        self.assertAlmostEqual(call.up.center[2], 0.10, delta=0.004)
        self.assertAlmostEqual(call.down.center[2], 0.02, delta=0.004)
        self.assertAlmostEqual(call.up.center[0], WALL_X - 0.002, delta=0.002)
        self.assertAlmostEqual(call.up.diameter_m, 0.035, delta=0.005)
        self.assertGreater(call.normal[0], 0.99)
        self.assertEqual((call.up.arrow, call.down.arrow), ("up", "down"))
        self.assertEqual(len(call.others), 1)

    def test_top_cut_off_is_rejected(self) -> None:
        """▲ が画面の外: ▼ と車いす用の ▲ は 20 cm 離れているので、一般用の組とはみなさない。"""
        with self.assertRaises(ButtonNotFound):
            self.find([(0.0, 0.18, "down"), (0.0, -0.02, "up")])

    def test_wrong_arrow_order_is_rejected(self) -> None:
        with self.assertRaises(ButtonNotFound):
            self.find([(0.0, 0.10, "down"), (0.0, 0.02, "up")])

    def test_single_button_is_rejected(self) -> None:
        with self.assertRaises(ButtonNotFound):
            self.find([(0.0, 0.05, "up")])

    def test_calibration_offset_shifts_buttons(self) -> None:
        """段階4の較正の補正（pelvis 座標で足す）が、ボタンの位置にも入る。押す向きは変わらない。"""
        from common.elevator_buttons import calibration_offset

        cam = FacingCamera()
        cam.offset = np.array([0.01, -0.02, 0.005])
        finder = ButtonFinder(cam, ClassicButtonDetector(self.cfg["classic"]), self.cfg)
        rgb, depth = scene([(0.0, 0.10, "up"), (0.0, 0.02, "down")])
        call = finder.find(rgb, depth, K, np.zeros(3))
        base = self.find([(0.0, 0.10, "up"), (0.0, 0.02, "down")])
        np.testing.assert_allclose(call.up.center - base.up.center, cam.offset, atol=1e-9)
        np.testing.assert_allclose(call.normal, base.normal, atol=1e-9)
        # 設定が null なら localize.yaml（ボトルと同じ較正）の値
        self.assertIsNone(self.cfg["calibration"]["offset_pelvis_m"])
        np.testing.assert_allclose(calibration_offset(self.cfg),
                                   load_config("localize.yaml")["calibration"]["offset_pelvis_m"])
        np.testing.assert_allclose(calibration_offset({"calibration": {"offset_pelvis_m": [0.0, 0.01, 0.0]}}),
                                   [0.0, 0.01, 0.0])


class TestYoloCandidates(unittest.TestCase):
    """YOLO の検出（クラス up / down）を、ボタンの候補（枠に内接する楕円の面）に変える。"""

    def test_converts_detections(self) -> None:
        from types import SimpleNamespace

        from common.elevator_buttons import YoloButtonDetector

        class FakeYolo:
            def __init__(self) -> None:
                self.frames: list[np.ndarray] = []

            def detect(self, frame: np.ndarray) -> list[SimpleNamespace]:
                self.frames.append(frame)
                return [SimpleNamespace(class_name="up", confidence=0.9, bbox=(100.0, 50.0, 140.0, 80.0)),
                        SimpleNamespace(class_name="down", confidence=0.8, bbox=(100.5, 150.0, 139.5, 180.0)),
                        SimpleNamespace(class_name="other", confidence=0.7, bbox=(0.0, 0.0, 10.0, 10.0))]

        det = YoloButtonDetector.__new__(YoloButtonDetector)
        det._det = FakeYolo()
        rgb = np.zeros((H, W, 3), np.uint8)
        rgb[..., 0] = 200  # R
        cands = det.detect(rgb)
        self.assertEqual([c.arrow for c in cands], ["up", "down", None])
        self.assertEqual(cands[1].bbox, (100, 150, 140, 180))
        self.assertEqual(cands[0].source, "yolo")
        # 面は枠に内接する楕円（面積 ≒ π × 20 × 15）
        self.assertAlmostEqual(int(cands[0].mask.sum()), np.pi * 20 * 15, delta=40)
        self.assertTrue(cands[0].mask[65, 120] and not cands[0].mask[50, 100])
        # Perception の YoloDetector には BGR で渡す
        self.assertEqual(int(det._det.frames[0][0, 0, 2]), 200)

    def test_backend_follows_weights_suffix(self) -> None:
        """重みが .onnx なら onnxruntime（OnnxYoloDetector）、.pt なら ultralytics（YoloDetector）で作る。"""
        import tempfile
        from pathlib import Path
        from types import SimpleNamespace
        from unittest import mock

        from common.elevator_buttons import YoloButtonDetector

        made: list[tuple[str, dict]] = []

        class Fake:
            def __init__(self, kind: str, **kw: object) -> None:
                made.append((kind, kw))

            def detect(self, frame: np.ndarray) -> list:
                return []

        modules = {"detector.onnx_yolo": SimpleNamespace(OnnxYoloDetector=lambda **kw: Fake("onnx", **kw)),
                   "detector.yolo_detector": SimpleNamespace(YoloDetector=lambda **kw: Fake("pt", **kw))}
        with tempfile.TemporaryDirectory() as d, \
                mock.patch("common.perception_bridge.perception", side_effect=lambda name: modules[name]):
            for suffix in (".onnx", ".pt"):
                (Path(d) / f"best{suffix}").write_bytes(b"")
            YoloButtonDetector({"weights": str(Path(d) / "best.onnx"), "confidence": 0.5, "threads": 4})
            YoloButtonDetector({"weights": str(Path(d) / "best.pt"), "confidence": 0.5, "device": "cpu"})
            YoloButtonDetector({"weights": str(Path(d) / "best.onnx"), "threads": None})
            with self.assertRaisesRegex(FileNotFoundError, "export_button_onnx"):
                YoloButtonDetector({"weights": str(Path(d) / "none.onnx")})
        self.assertEqual([k for k, _ in made], ["onnx", "pt", "onnx"])
        self.assertEqual((made[0][1]["num_threads"], made[0][1]["confidence_threshold"]), (4, 0.5))
        self.assertEqual(made[1][1]["device"], "cpu")
        self.assertIsNone(made[2][1]["num_threads"])
        self.assertEqual(made[2][1]["confidence_threshold"], 0.4)

    def test_val_split_has_both_sets(self) -> None:
        """学習データの検証用に、basic（偶数の種）と realistic（奇数の種）が半分ずつ入る。"""
        import sys
        from pathlib import Path

        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
        from make_button_dataset import choose_val

        val = choose_val(list(range(1000, 1300)), 10)
        self.assertEqual(len(val), 30)
        self.assertEqual(sum(s % 2 for s in val), 15)


class TestCheckPhotos(unittest.TestCase):
    """real/check_button_photos.py（撮った写真で確かめるツール）の、読み込みと数え方。"""

    @classmethod
    def setUpClass(cls) -> None:
        import sys
        from pathlib import Path

        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "real"))
        import check_button_photos

        cls.m = check_button_photos

    def test_score_against_labels(self) -> None:
        from common.elevator_buttons import Candidate2D

        def cand(box: tuple[int, int, int, int], arrow: str | None) -> Candidate2D:
            return Candidate2D(bbox=box, mask=np.zeros((1, 1), bool), arrow=arrow)

        truth = [((10, 10, 30, 30), "up"), ((10, 50, 30, 70), "down"), ((100, 100, 120, 120), "up")]
        cands = [cand((11, 10, 31, 30), "up"), cand((10, 51, 30, 71), None), cand((300, 300, 320, 320), "up")]
        s = self.m.score_against(cands, truth)
        self.assertEqual((s.correct, s.wrong_arrow, s.missed, s.extra), (1, 1, 1, 1))

    def test_reads_yolo_labels_next_to_images(self) -> None:
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as d:
            img = Path(d) / "images" / "val" / "a.png"
            img.parent.mkdir(parents=True)
            (Path(d) / "labels" / "val").mkdir(parents=True)
            (Path(d) / "labels" / "val" / "a.txt").write_text("1 0.5 0.25 0.1 0.2\n")
            labels = self.m.read_yolo_labels(img, 200, 100)
            self.assertEqual(len(labels), 1)
            np.testing.assert_allclose(labels[0][0], (90.0, 15.0, 110.0, 35.0))
            self.assertEqual(labels[0][1], "down")
            self.assertIsNone(self.m.read_yolo_labels(Path(d) / "other" / "a.png", 200, 100))

    def test_recording_with_depth_finds_pair(self) -> None:
        """real/record.py の形式の収録から、深度・内部パラメータ・腰の角度を読み、▲▼ の 1 組まで選べる。"""
        import tempfile
        from pathlib import Path

        from common.arm.types import JointState
        from common.recording import RecordingWriter
        from common.rgbd_protocol import Intrinsics, RgbdFrame

        rgb, depth = scene([(0.0, 0.10, "up"), (0.0, 0.02, "down")])
        bgr = np.ascontiguousarray(rgb[:, :, ::-1])
        depth_u16 = np.round(depth / 0.001).astype(np.uint16)
        frame = RgbdFrame(color_bgr=bgr, depth=depth_u16, depth_scale=0.001,
                          intrinsics=Intrinsics(width=W, height=H, fx=F, fy=F, cx=W / 2, cy=H / 2),
                          timestamp=1.0, frame_id=1)
        z = np.zeros(29)
        state = JointState(q=z, dq=z, motor_mode=np.ones(29, int), mode_machine=5, stamp=0.0)
        with tempfile.TemporaryDirectory() as d:
            w = RecordingWriter(Path(d), "button", {})
            w.add_frame(bgr, depth_u16, 1.0, frame, state)
            w.close()
            photos = self.m.load_photos([w.dir], every=1)
        self.assertEqual(len(photos), 1)
        ph = photos[0]
        self.assertAlmostEqual(float(ph.depth_m[0, 0]), WALL_X, delta=0.001)
        self.assertEqual(ph.K[0, 0], F)
        cfg = load_config("elevator.yaml")
        finder = ButtonFinder(FacingCamera(), ClassicButtonDetector(cfg["classic"]), cfg)
        self.assertTrue(self.m.try_pair(finder, ph).startswith("OK"))
        ph.waist_q = None
        self.assertIn("腰の角度が無い", self.m.try_pair(finder, ph))


class TestRealPhotoLabels(unittest.TestCase):
    """real/label_button_photos.py（枠を付ける）と sim/merge_button_datasets.py（混ぜる）の、画面を使わない部分。"""

    @classmethod
    def setUpClass(cls) -> None:
        import sys
        from pathlib import Path

        root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(root / "real"))
        sys.path.insert(0, str(root / "sim"))
        import label_button_photos
        import merge_button_datasets

        cls.label = label_button_photos
        cls.merge = merge_button_datasets

    def test_edit_save_and_reload(self) -> None:
        import tempfile
        from pathlib import Path

        rgb, _ = scene([(0.0, 0.10, "up"), (0.0, 0.02, "down")])
        sheet = self.label.Sheet("a", np.ascontiguousarray(rgb[:, :, ::-1]))
        self.label.prelabel(sheet, ClassicButtonDetector(load_config("elevator.yaml")["classic"]))
        self.assertEqual(sorted(b.cls for b in sheet.boxes), ["down", "up"])
        up = next(b for b in sheet.boxes if b.cls == "up")
        cx, cy = (up.x1 + up.x2) / 2, (up.y1 + up.y2) / 2
        self.assertTrue(sheet.toggle_at(cx, cy))
        self.assertEqual(up.cls, "down")
        self.assertTrue(sheet.delete_at(cx, cy))
        self.assertFalse(sheet.add(10, 10, 12, 12, "up"))  # 小さすぎる枠は足さない
        self.assertTrue(sheet.add(700, 500, 600, 400, "up"))  # 逆向きのドラッグ・画面の外も直す
        self.assertEqual((sheet.boxes[-1].x2, sheet.boxes[-1].y2), (W, H))
        with tempfile.TemporaryDirectory() as d:
            self.label.save(sheet, Path(d))
            again = self.label.Sheet("a", sheet.bgr)
            self.assertTrue(self.label.load_saved(again, Path(d)))
            self.assertEqual([b.cls for b in again.boxes], [b.cls for b in sheet.boxes])
            np.testing.assert_allclose([(b.x1, b.y1, b.x2, b.y2) for b in again.boxes],
                                       [(b.x1, b.y1, b.x2, b.y2) for b in sheet.boxes], atol=0.01)
            self.label.remove(again, Path(d))
            self.assertFalse((Path(d) / "labels" / "a.txt").exists())

    def test_merge_splits_and_repeats_real(self) -> None:
        import tempfile
        from pathlib import Path

        def put(folder: Path, split: str | None, names: list[str]) -> None:
            img_dir = folder / "images" / split if split else folder / "images"
            lab_dir = folder / "labels" / split if split else folder / "labels"
            img_dir.mkdir(parents=True, exist_ok=True)
            lab_dir.mkdir(parents=True, exist_ok=True)
            for n in names:
                (img_dir / f"{n}.png").write_bytes(b"x")
                (lab_dir / f"{n}.txt").write_text("0 0.5 0.5 0.1 0.1\n")

        with tempfile.TemporaryDirectory() as d:
            sim, real, out = Path(d) / "sim", Path(d) / "hall", Path(d) / "mixed"
            put(sim, "train", ["s1", "s2"])
            put(sim, "val", ["s3"])
            put(real, None, [f"f{i}" for i in range(10)])
            (real / "images" / "nolabel.png").write_bytes(b"x")  # ラベルの無い写真は使わない
            c = self.merge.merge(sim, [real], out, real_val_every=5, real_repeat=3)
            self.assertEqual(c, {"sim_train": 2, "sim_val": 1, "real_train": 8, "real_val": 2})
            self.assertEqual(len(list((out / "images" / "train").iterdir())), 2 + 8 * 3)
            self.assertEqual(len(list((out / "labels" / "val").iterdir())), 1 + 2)
            self.assertEqual(len(list((out / "images" / "val_real").iterdir())), 2)
            self.assertIn(f"path: {out}", (out / "data.yaml").read_text())


class TestSelectAndTrack(unittest.TestCase):
    def b(self, y: float, z: float, arrow: str | None = None) -> Button3D:
        return Button3D(np.array([0.3, y, z]), 0.035, arrow, (0, 0, 1, 1), 100)

    def test_two_columns_are_ambiguous(self) -> None:
        g = load_config("elevator.yaml")["geometry"]
        with self.assertRaises(ButtonNotFound):
            select_call_pair([self.b(0.0, 0.3), self.b(0.0, 0.22), self.b(0.3, 0.3), self.b(0.3, 0.22)], g)

    def test_unknown_arrows_use_order(self) -> None:
        g = load_config("elevator.yaml")["geometry"]
        up, down = select_call_pair([self.b(0.0, 0.22), self.b(0.0, 0.3)], g)
        self.assertEqual((up.center[2], down.center[2]), (0.3, 0.22))

    def lab_column(self) -> list[Button3D]:
        """ラボのエレベーターの柱（2026-10-22 の測定、pelvis 座標の z）: 一般用 ▲▼、車いすの印（YOLO が up と囲む）、車いす用 ▲▼。"""
        return [self.b(0, 0.51, "up"), self.b(0, 0.44, "down"), self.b(0, 0.30, "up"),
                self.b(0, 0.21, "up"), self.b(0, 0.14, "down")]

    def test_wheelchair_set_picks_the_bottom_pair(self) -> None:
        g = dict(load_config("elevator.yaml")["geometry"])
        up, down = select_call_pair(self.lab_column(), g)  # 既定は一般用（柱の上の 2 つ）
        self.assertEqual((up.center[2], down.center[2]), (0.51, 0.44))
        g["button_set"] = "wheelchair"
        up, down = select_call_pair(self.lab_column(), g)
        self.assertEqual((up.center[2], down.center[2]), (0.21, 0.14))  # 車いすの印（0.30）は選ばない
        self.assertEqual((up.name, down.name), ("up", "down"))

    def test_wheelchair_set_works_when_only_that_pair_is_in_view(self) -> None:
        """頭カメラには、ラボでは車いす用の組（と印）しか写らない（一般用は画像の上に切れる）。"""
        g = dict(load_config("elevator.yaml")["geometry"])
        g["button_set"] = "wheelchair"
        for col in (self.lab_column()[3:], self.lab_column()[2:]):  # 2 つだけ / 印も up として囲まれた 3 つ
            up, down = select_call_pair(col, g)
            self.assertEqual((up.center[2], down.center[2]), (0.21, 0.14))

    def test_wheelchair_set_refuses_the_general_pair(self) -> None:
        """一般用の 2 つしか見えていないとき（車いす用が画面の外）に、一般用を車いす用と取り違えない（高さで見る）。"""
        g = dict(load_config("elevator.yaml")["geometry"])
        g["button_set"] = "wheelchair"
        with self.assertRaises(ButtonNotFound) as cm:
            select_call_pair(self.lab_column()[:2], g)
        self.assertIn("高すぎる", str(cm.exception))

    def test_wheelchair_set_checks_spacing_and_order(self) -> None:
        g = dict(load_config("elevator.yaml")["geometry"])
        g["button_set"] = "wheelchair"
        col = self.lab_column()
        col[-1] = self.b(0, -0.10, "down")  # 下の間隔 31 cm
        with self.assertRaises(ButtonNotFound):
            select_call_pair(col, g)
        col = self.lab_column()
        col[-2], col[-1] = self.b(0, 0.21, "down"), self.b(0, 0.14, "up")  # 順番が逆
        with self.assertRaises(ButtonNotFound):
            select_call_pair(col, g)

    def test_tracker(self) -> None:
        cfg = {"frames": 3, "max_spread_m": 0.005, "max_normal_spread_deg": 5.0}
        tr = ButtonTracker(cfg)

        def call(dz: float) -> CallButtons:
            return CallButtons(self.b(0, 0.3 + dz, "up"), self.b(0, 0.22 + dz, "down"), np.array([1.0, 0, 0]))

        self.assertIsNone(tr.add(call(0.0)))
        self.assertIsNone(tr.add(call(0.02)))  # 2 cm 動いた
        self.assertIsNone(tr.add(call(0.0)))  # ばらつきが大きいので、まだ返さない
        self.assertIsNone(tr.add(call(0.001)))  # 2 cm 動いたフレームがまだ 3 つの中にある
        merged = tr.add(call(0.0))  # 続けて 3 つそろった
        self.assertIsNotNone(merged)
        self.assertAlmostEqual(merged.up.center[2], 0.30, delta=1e-6)


@needs("mujoco", "pin")
class TestElevatorPlan(unittest.TestCase):
    """評価環境の seed 1（▼、柱まで 0.31 m）の値で、計画が立つこと。"""

    @classmethod
    def setUpClass(cls) -> None:
        from common.elevator_press import ElevatorPressController

        cls.ctl = ElevatorPressController.from_config(
            load_config("robot.yaml"), load_config("press.yaml"), load_config("arm.yaml"),
            load_config("elevator.yaml"))
        # 評価環境の始めの姿勢（両腕を下ろしている）
        cls.q0 = np.zeros(29)
        cls.q0[[15, 22]] = 0.2
        cls.q0[16], cls.q0[23] = 0.25, -0.25
        cls.q0[[18, 25]] = 1.0

    def call(self, x: float = 0.309) -> CallButtons:
        return CallButtons(Button3D(np.array([x, 0.027, 0.267]), 0.033, "up", (0, 0, 1, 1), 100, "up"),
                           Button3D(np.array([x, 0.027, 0.187]), 0.033, "down", (0, 0, 1, 1), 100, "down"),
                           np.array([1.0, 0.0, 0.0]))

    def test_plans_both_buttons(self) -> None:
        from common.elevator_press import wall_obstacle

        for target in ("up", "down"):
            self.ctl.reset(target, 0.02, np.full(17, 60.0))
            self.ctl._plan(self.q0, self.call())
            p = self.ctl.plan_
            face = self.call().button(target).center
            # 球の表面がボタンに触れる点、押す向き、押し込み終わり
            np.testing.assert_allclose(p.target_point, face - [0.008, 0, 0], atol=1e-9)
            self.assertAlmostEqual(p.end_point[0] - p.target_point[0], 0.015, places=6)
            # 指は左・上に傾いている（押す向きとは別）
            self.assertLess(p.finger_axis @ p.push_dir, np.cos(np.radians(30)))
            # 手前の姿勢は壁の箱の手前（箱の前面はボタンの面の位置）
            wall = wall_obstacle(self.call(), self.ctl.cfg["wall"])
            front = wall["center"][0] - wall["half_size"][0]
            self.assertAlmostEqual(front, face[0] + self.ctl.cfg["wall"]["margin_m"], places=9)
            self.assertLess(p.approach_point[0], front)

    def test_plans_from_the_real_standing_pose(self) -> None:
        """実機の立った姿勢（2026-10-21 の lowstate）は、下ろした手が股関節の当たり判定に入っている（−7 mm）。
        開始姿勢ですでに近い組は、最初の区間で開始より近づかない間は許す（PressPlanner.plan と同じ）。許さないと、
        どの位置でも「手前の姿勢へ動く途中で柱・壁に当たる」と断られた（sim/elevator_stand_map.py で気づいた、2026-10-29）。"""
        q = np.array([-0.290485, -0.004960, -0.002531, 0.593250, -0.318594, 0.008501, -0.271123, 0.003017,
                      -0.002223, 0.601994, -0.338352, -0.005524, -0.000576, 0.000470, -0.002349, 0.291049,
                      0.129657, 0.015711, 0.981556, 0.087161, 0.076807, 0.015232, 0.286950, -0.125894,
                      0.024520, 0.982035, -0.096293, 0.058938, -0.020649])
        for target in ("up", "down"):
            self.ctl.reset(target, 0.02, np.full(17, 60.0))
            self.assertTrue(any("hip" in h.body1 + h.body2 for h in self.ctl.planner.collision.contacts(q)))  # 開始で当たり判定に入っている
            self.ctl._plan(q, self.call())
            self.assertIsNotNone(self.ctl.plan_)

    def test_plans_low_wheelchair_buttons(self) -> None:
        """車いす用の低いボタン（ボトルの押す点とほぼ同じ高さ、pelvis から z +0.04〜+0.15）にも手前の姿勢まで計画が立つ（2026-10-22）。"""
        for z in (0.15, 0.10, 0.04):
            for y in (0.027, -0.05):
                c = CallButtons(Button3D(np.array([0.309, y, z + 0.08]), 0.03, "up", (0, 0, 1, 1), 100, "up"),
                                Button3D(np.array([0.309, y, z]), 0.03, "down", (0, 0, 1, 1), 100, "down"),
                                np.array([1.0, 0.0, 0.0]))
                self.ctl.reset("down", 0.02, np.full(17, 60.0))
                self.ctl._plan(self.q0, c)
                np.testing.assert_allclose(self.ctl.plan_.target_point[2], z, atol=1e-9)

    def test_tilted_wall_box_follows_the_face(self) -> None:
        """面が斜め（3°）でも、箱の前面はボタンの面に沿う（手前に出っ張らない）。"""
        from common.collision import CollisionChecker
        from common.elevator_press import wall_obstacle

        n = np.array([np.cos(np.radians(3)), np.sin(np.radians(3)), 0.0])
        call = self.call()
        call.normal = n
        ob = wall_obstacle(call, self.ctl.cfg["wall"])
        np.testing.assert_allclose(ob["x_axis"], n)
        chk = CollisionChecker(self.ctl.robot_cfg, "right", 0.0, [ob])
        m, d = chk.model, chk.data
        import mujoco

        mujoco.mj_forward(m, d)
        gid = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("obstacle_0")][0]
        x_axis_world = d.geom_xmat[gid].reshape(3, 3)[:, 0]
        np.testing.assert_allclose(x_axis_world, n, atol=1e-9)

    def test_unreachable_far_wall(self) -> None:
        from common.press_planner import UnreachableError

        self.ctl.reset("up", 0.02, np.full(17, 60.0))
        with self.assertRaises(UnreachableError):
            self.ctl._plan(self.q0, self.call(x=0.9))

    def test_fresh_images_without_capture_time(self) -> None:
        """撮った時刻が 0 のまま（評価環境の実機用の口）でも、一定の間隔ごとに新しい画像とみなす。"""
        self.ctl.reset("up", 0.02, np.full(17, 60.0))
        interval = self.ctl.cfg["press"]["image_interval_s"]
        fresh = [self.ctl._is_fresh(k * 0.02, 0.0) for k in range(int(round(1.0 / 0.02)))]
        self.assertEqual(sum(fresh), int(round(1.0 / interval)))
        # 撮った時刻があれば、それが進んだときだけ新しい
        self.ctl.reset("up", 0.02, np.full(17, 60.0))
        self.assertEqual([self.ctl._is_fresh(t, it) for t, it in [(0.1, 0.05), (0.12, 0.05), (0.14, 0.09)]],
                         [True, False, True])

    def test_lit_check_retries_once(self) -> None:
        self.ctl.reset("down", 0.02, np.full(17, 60.0))
        self.ctl.call = self.call()
        self.ctl._plan(self.q0, self.call())
        self.ctl.q_hold = self.q0.copy()
        dark = np.zeros((480, 640, 3), np.uint8)
        self.ctl.baseline_brightness = self.ctl.baseline_level = 0.0
        depth0 = self.ctl.plan_.depth
        self.assertFalse(self.ctl._check(dark, 1.0))  # 点灯していない → 深く押し直す
        self.assertEqual(self.ctl.phase, "press")
        self.assertAlmostEqual(self.ctl.plan_.depth, depth0 + 0.01)
        self.assertTrue(self.ctl._check(dark, 2.0))  # 押し直しは 1 回まで
        self.assertIn("確かめられない", self.ctl.outcome)
        self.assertEqual(self.ctl.phase, "home")  # 確かめられなくても、腕を戻してから終わる

    def test_lit_level_sees_a_thin_ring(self) -> None:
        """縁の細い輪だけが光る点灯（ラボのエレベーター）でも、明るい部分の上がり方で点灯と分かる。枠全体の平均では薄まる。"""
        import cv2

        img = np.full((480, 640, 3), 30, np.uint8)  # 黒い盤
        cv2.circle(img, (320, 240), 25, (150, 150, 150), -1)  # 灰色のボタン
        lit = img.copy()
        cv2.circle(lit, (320, 240), 27, (240, 240, 255), 2)  # 縁の細い輪だけが光る
        box = (295, 215, 346, 266)
        rise = self.ctl._lit_level(lit, box) - self.ctl._lit_level(img, box)
        whole = self.ctl._brightness(lit, box) - self.ctl._brightness(img, box)
        self.assertGreaterEqual(rise, self.ctl.cfg["press"]["lit_check"]["min_rise"])
        self.assertLess(whole, rise / 2)
        self.assertAlmostEqual(self.ctl._lit_level(img, box), self.ctl._lit_level(img.copy(), box))

    def test_returns_home_by_the_way_it_came(self) -> None:
        """点灯を確かめたら、来た経路（手前の姿勢 → 経由の姿勢 → 始め）を逆にたどって戻してから終わる。"""
        ctl = self.ctl
        ctl.reset("down", 0.02, np.full(17, 60.0))
        ctl.call = self.call()
        ctl._plan(self.q0, self.call())
        ctl.q_hold = self.q0.copy()
        self.assertIsNotNone(ctl.plan_.q_via)  # この配置は経由の姿勢を通る（seed 1 の ▼）
        arm = ctl.kin.arm_idx
        ctl.q_ik[arm] = ctl.plan_.q_approach + 0.01  # 指先のずれの補正で、手前の姿勢から少しずれている
        ctl.baseline_brightness = ctl.baseline_level = 0.0
        self.assertTrue(ctl._check(np.full((480, 640, 3), 255, np.uint8), 0.0))
        self.assertEqual((ctl.outcome, ctl.phase), ("点灯した", "home"))
        # 今の姿勢 → 手前の姿勢 → 経由の姿勢 → 始め
        pts = [ctl.home_legs[0][0]] + [b for _, b, _ in ctl.home_legs]
        for got, want in zip(pts, [ctl.plan_.q_approach + 0.01, ctl.plan_.q_approach, ctl.plan_.q_via, self.q0[arm]]):
            np.testing.assert_allclose(got, want, atol=1e-9)
        self.assertFalse(ctl._path_hits(self.q0, pts))
        img, depth, K0 = np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640)), np.eye(3)
        q, t, done, phases = self.q0.copy(), 0.0, False, []
        while not done and t < 30.0:
            t += 0.02
            _, done = ctl.step(img, depth, K0, q, t, 0.0)
            phases.append(ctl.phase)
        self.assertTrue(done)
        self.assertEqual(list(dict.fromkeys(phases)), ["home", "done"])
        np.testing.assert_allclose(ctl.q_ik[arm], self.q0[arm], atol=1e-9)
        self.assertAlmostEqual(t, sum(T for _, _, T in ctl.home_legs), delta=0.05)

    def test_stand_advice(self) -> None:
        """立つ位置が範囲の外なら、どちらへ何 cm 動けばよいかを出して、計画せずに終わる。"""
        ctl = self.ctl
        st = ctl.cfg["press"]["stand"]
        ctl.reset("up", 0.02, np.full(17, 60.0))
        self.assertEqual(ctl._stand_advice(self.call(x=0.33)), "")  # 範囲の中
        near = ctl._stand_advice(self.call(x=float(st["min_dist_m"]) - 0.025))
        self.assertIn("近すぎる。あと 3 cm 以上下がる", near)
        far = ctl._stand_advice(self.call(x=float(st["max_dist_m"]) + 0.002))
        self.assertIn("遠すぎる。あと 1 cm 以上近づく", far)  # 切り上げ（「0 cm」と出さない）
        side = self.call(x=0.33)
        for b in (side.up, side.down):
            b.center = b.center + [0.0, -0.08, 0.0]
        self.assertEqual(ctl._stand_advice(side), "")  # 左右 5.3 cm は範囲の中
        for b in (side.up, side.down):
            b.center = b.center + [0.0, -0.03, 0.0]
        self.assertIn("右に 8.3 cm。右へ 3 cm 以上寄る", ctl._stand_advice(side))
        # 計画できなければ、中止の理由に動く向きと量を添える
        from unittest import mock

        def look(call: CallButtons) -> None:
            ctl.reset("up", 0.02, np.full(17, 60.0))
            with mock.patch.object(ctl.finder, "find", return_value=call), \
                    mock.patch.object(ctl.tracker, "add", return_value=call):
                ctl._look(np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640)), np.eye(3), self.q0, 1.0)

        look(self.call(x=0.6))
        self.assertEqual(ctl.phase, "done")
        self.assertRegex(ctl.outcome, "遠すぎる。あと 2[01] cm 以上近づく.*）。計画できない")
        # 範囲の外でも計画が立てば、注意だけ出して押す（推定の距離は数 cm ずれることがあるため）
        look(self.call(x=float(st["min_dist_m"]) - 0.01))
        self.assertEqual(ctl.phase, "reach")
        self.assertTrue(any("⚠️" in l.message and "近すぎる" in l.message for l in ctl.log))

    def test_press_stops_when_finger_is_blocked(self) -> None:
        """押し込みの途中で指先が止まったら（ボタンが底に着いた）、それ以上奥へ押さずに保持へ移る。"""
        ctl = self.ctl
        ctl.reset("down", 0.02, np.full(17, 60.0))
        ctl.call = self.call()
        ctl._plan(self.q0, self.call())
        ctl.q_hold = self.q0.copy()
        arm, p = ctl.kin.arm_idx, ctl.plan_
        ctl.q_ik[arm] = p.q_approach
        ctl._enter("press", 0.0)
        img, depth, K0 = np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640)), np.eye(3)
        cs = ctl.cfg["press"]["contact_stop"]
        q, t, blocked = self.q0.copy(), 0.0, None
        while ctl.phase == "press" and t < 5.0:
            if blocked is None:
                q[arm] = ctl.q_ik[arm]  # 遅れなく付いてくる
                if float((ctl.kin.fk_pos(q) - p.target_point) @ p.push_dir) > 0.003:
                    blocked = q.copy()  # 面の 3 mm 奥で止まる
            t += 0.02
            ctl.step(img, depth, K0, blocked if blocked is not None else q, t, 0.0)
        self.assertIsNotNone(blocked)
        self.assertEqual(ctl.phase, "hold")
        self.assertLess(t, ctl.cfg["press"]["press_s"])  # 押し込みの終わりより前に止まる
        n, tip = p.push_dir, ctl.kin.fk_pos(blocked)
        # 押し込み終わりは、止まった指先の keep_m 先（計画の 15 mm 奥ではない）
        self.assertAlmostEqual(float((ctl.plan_.end_point - tip) @ n), cs["keep_m"], delta=0.0015)
        self.assertLess(float((ctl.plan_.end_point - p.target_point) @ n), p.depth - 0.005)
        # 保持の間、指令した指先は止まった指先の少し先のまま
        for _ in range(10):
            t += 0.02
            ctl.step(img, depth, K0, blocked, t, 0.0)
        self.assertLess(ctl._push_lag(blocked), cs["lag_m"])
        # 押し直しのときは、強めに押してから止める
        ctl.retries = 1
        self.assertEqual(ctl._stop_lag(), cs["retry_lag_m"])

    def _lagging_press(self, lag0: float, block_after: float | None) -> tuple[bool, float]:
        """押す前から指先が lag0 [m] 遅れている（実機の重力補償 0.5）状態で押し込みを回す。
        block_after を与えると、面の block_after [m] 奥で指先が止まる。(保持へ移ったか, 経過時間) を返す。"""
        ctl = self.ctl
        ctl.reset("down", 0.02, np.full(17, 60.0))
        ctl.call = self.call()
        ctl._plan(self.q0, self.call())
        ctl.q_hold = self.q0.copy()
        arm, p = ctl.kin.arm_idx, ctl.plan_
        ctl.q_ik[arm] = p.q_approach
        ctl._enter("press", 0.0)
        img, depth, K0 = np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640)), np.eye(3)
        axis, n = p.finger_axis, p.push_dir
        t, frozen = 0.0, None
        while ctl.phase == "press" and t < 5.0:
            tip_cmd = ctl.kin.fk_pos(ctl.q_ik)
            if frozen is None:
                q = ctl.kin.ik(tip_cmd - lag0 * n, ctl.q_ik, axis, max_iter=50).q  # 指令より lag0 手前に付いてくる
                if block_after is not None and float((ctl.kin.fk_pos(q) - p.target_point) @ n) > block_after:
                    frozen = q.copy()
            else:
                q = frozen
            t += 0.02
            ctl.step(img, depth, K0, q, t, 0.0)
        return ctl.phase == "hold", t

    def test_constant_lag_before_pushing_does_not_stop(self) -> None:
        """押す前から 6 mm 遅れていても（実機は 4.5 mm に、押し始めの遅れが足されて 5 mm を超えた）、増えなければ止めず、押し込みの終わりまで押す。"""
        held, t = self._lagging_press(0.006, None)
        self.assertTrue(held)
        self.assertGreaterEqual(t, self.ctl.cfg["press"]["press_s"] - 0.05)  # 途中で止めていない
        self.assertAlmostEqual(self.ctl.lag0, 0.006, delta=0.0015)

    def test_lagging_press_still_stops_when_blocked(self) -> None:
        """押す前の遅れがあっても、指先が止まって遅れが増えたら止める。"""
        held, t = self._lagging_press(0.006, 0.003)
        self.assertTrue(held)
        self.assertLess(t, self.ctl.cfg["press"]["press_s"])


if __name__ == "__main__":
    unittest.main()
