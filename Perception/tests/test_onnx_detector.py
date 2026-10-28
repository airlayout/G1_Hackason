"""ONNXの検出器(common/detector/onnx_yolo.py)。G1のPC2で torch/ultralytics を使わずにYOLOを動かすためのもの。

前処理・後処理は合成した出力で確かめる(モデル不要)。ultralytics と onnx が入っていれば、書き出したONNXを
ultralytics で動かした結果と一致するかも確かめる(入っていなければ飛ばす)。
"""
from __future__ import annotations

import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

PERCEPTION_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PERCEPTION_DIR))

from common.detector.onnx_yolo import letterbox, nms, postprocess, postprocess_end2end  # noqa: E402

NAMES = {0: "person", 1: "bicycle", 2: "car"}


class LetterboxTest(unittest.TestCase):
    def test_wide_image_is_padded_top_and_bottom(self) -> None:
        """G1の頭カメラの配信(640x360)は、640x640の上下に灰色を足す。"""
        img = np.zeros((360, 640, 3), dtype=np.uint8)
        out, r, (left, top) = letterbox(img, 640)
        self.assertEqual(out.shape, (640, 640, 3))
        self.assertAlmostEqual(r, 1.0)
        self.assertEqual((left, top), (0, 140))
        self.assertTrue(np.all(out[:140] == 114))


class PostprocessTest(unittest.TestCase):
    def test_end2end_filters_class_and_maps_back(self) -> None:
        # 入力 640x640 の画素で [x1, y1, x2, y2, 確からしさ, クラス]。元の画像は 1280x720(倍率 0.5、上に余白 140)
        out = np.array([[[100, 200, 300, 500, 0.9, 0], [10, 150, 50, 190, 0.8, 2], [0, 140, 10, 150, 0.3, 0]]], dtype=np.float32)
        dets = postprocess_end2end(out, NAMES, [0], 0.5, 0.5, (0, 140), (720, 1280))
        self.assertEqual(len(dets), 1)
        self.assertEqual(dets[0].class_name, "person")
        np.testing.assert_allclose(dets[0].bbox, (200, 120, 600, 720))

    def test_classic_output_runs_nms(self) -> None:
        # (1, 4 + クラス数, 候補数)。同じ人に重なった枠が 2 つと、離れた人が 1 つ
        cand = np.zeros((4 + 3, 3), dtype=np.float32)
        cand[:4, 0], cand[4, 0] = [320, 320, 100, 200], 0.9
        cand[:4, 1], cand[4, 1] = [322, 321, 100, 200], 0.7
        cand[:4, 2], cand[4, 2] = [100, 100, 40, 80], 0.6
        dets = postprocess(cand[None], NAMES, [0], 0.5, 0.7, 1.0, (0, 0), (640, 640))
        self.assertEqual([round(d.confidence, 1) for d in dets], [0.9, 0.6])
        np.testing.assert_allclose(dets[0].bbox, (270, 220, 370, 420))

    def test_nms_keeps_separate_boxes(self) -> None:
        boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11], [50, 50, 60, 60]], dtype=np.float32)
        self.assertEqual(nms(boxes, np.array([0.9, 0.8, 0.7]), 0.5), [0, 2])


class TorchFreeImportTest(unittest.TestCase):
    def test_detector_package_does_not_import_torch(self) -> None:
        """PC2(torchなし)で使えるように、common.detector と common.camera は torch/ultralytics を読み込まない。"""
        code = ("import sys; sys.path.insert(0, %r); import common.detector, common.camera; "
                "print('torch' in sys.modules, 'ultralytics' in sys.modules)") % str(PERCEPTION_DIR)
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True).stdout.split()
        self.assertEqual(out, ["False", "False"])


@unittest.skipUnless(all(importlib.util.find_spec(m) for m in ("ultralytics", "onnx", "onnxruntime", "cv2")),
                     "ultralytics・onnx・onnxruntime が無い")
class SameAsUltralyticsTest(unittest.TestCase):
    def test_matches_ultralytics_on_same_onnx(self) -> None:
        import os

        os.environ.setdefault("YOLO_AUTOINSTALL", "False")
        import cv2
        import ultralytics
        from ultralytics import YOLO

        from common.detector.onnx_yolo import OnnxYoloDetector

        assets = Path(ultralytics.__file__).parent / "assets"
        with tempfile.TemporaryDirectory() as tmp:
            pt = Path(tmp) / "yolo26n.pt"
            repo_pt = PERCEPTION_DIR.parent / "yolo26n.pt"
            if repo_pt.exists():
                pt.write_bytes(repo_pt.read_bytes())
            onnx = YOLO(str(pt) if pt.exists() else "yolo26n.pt").export(format="onnx", imgsz=640, simplify=False,
                                                                         opset=17, nms=False)
            mine = OnnxYoloDetector(onnx, classes=["person"], confidence_threshold=0.25)
            ref = YOLO(onnx, task="detect")
            for name in ("bus.jpg", "zidane.jpg"):
                img = cv2.imread(str(assets / name))
                a = sorted((d.confidence for d in mine.detect(img)), reverse=True)
                b = sorted((float(c) for c in ref.predict(img, classes=[0], conf=0.25, verbose=False,
                                                           device="cpu")[0].boxes.conf), reverse=True)
                self.assertEqual(len(a), len(b), name)
                np.testing.assert_allclose(a, b, atol=0.01, err_msg=name)


if __name__ == "__main__":
    unittest.main()
