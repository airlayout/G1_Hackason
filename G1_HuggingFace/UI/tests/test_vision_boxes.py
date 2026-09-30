"""検出枠の表示切り替え（枠あり/枠なしの2系統）のテスト。

⚠️ **YOLO は使わない。** 偽の検出器と偽の映像源を差し込んで、`VisionWorker` が
同じフレームから2種類の JPEG を正しく作り分けるかだけを見る。

⚠️ ただし `vision.py` の import は `ultralytics` を引き込む（Perception の
`yolo_detector` に `Detection` と `YoloDetector` が同居しているため）。入っていない
環境では飛ばす。
"""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

import numpy as np

MAIN = Path(__file__).resolve().parents[1] / "Main"
sys.path.insert(0, str(MAIN))

try:
    from vision import VisionWorker                                  # noqa: E402
    from common.detector.yolo_detector import Detection              # noqa: E402
    _READY = True
except ImportError as exc:      # ultralytics/torch が無い環境
    _READY = False
    _WHY = str(exc)


class _StubSource:
    """毎回まったく同じ絵を返す映像源。フレーム差で結果がぶれないようにする。"""

    def __init__(self) -> None:
        rng = np.random.RandomState(0)
        # 真っ黒だと JPEG が潰れて差が出にくいので、模様のある絵にする
        self._frame = rng.randint(0, 255, (240, 320, 3), dtype=np.uint8)

    def open(self) -> None: ...
    def close(self) -> None: ...

    def read(self) -> np.ndarray:
        return self._frame.copy()


class _StubDetector:
    """与えられた検出結果を返すだけの検出器。"""

    device = "cpu"

    def __init__(self, detections: list) -> None:
        self._detections = detections

    def detect(self, frame: np.ndarray) -> list:
        return list(self._detections)


@unittest.skipUnless(_READY, "ultralytics が無いので飛ばす" if not _READY else "")
class BoxesToggleTest(unittest.TestCase):

    def _run_worker(self, detections: list) -> tuple[bytes, bytes]:
        """1フレーム処理させて、枠あり・枠なしの JPEG を取り出す。"""
        w = VisionWorker(_StubSource(), _StubDetector(detections), target_fps=30.0)
        w.start()
        self.addCleanup(w.stop)
        deadline = time.monotonic() + 5.0
        while True:
            boxed, seq = w.wait_for_frame(-1, boxes=True, timeout=1.0)
            if boxed is not None:
                break
            if time.monotonic() > deadline:
                self.fail("フレームが1枚も出てきませんでした")
        # ⚠️ **同じ seq の絵どうしを比べる。** 別フレーム同士を比べると、
        #    動いている映像では必ず違って見えて何も検証できない
        plain, seq2 = w.wait_for_frame(seq - 1, boxes=False, timeout=1.0)
        self.assertEqual(seq, seq2, "比較が同一フレームで行われていません")
        return boxed, plain

    def test_differs_when_something_is_detected(self) -> None:
        det = [Detection(class_name="person", confidence=0.9, bbox=(40.0, 30.0, 140.0, 200.0))]
        boxed, plain = self._run_worker(det)
        self.assertNotEqual(boxed, plain, "枠を焼いたのに枠なしと同じ絵になっている")

    def test_same_bytes_when_nothing_is_detected(self) -> None:
        """検出が無ければ2枚は同じ絵。二度エンコードしていないことの確認でもある。"""
        boxed, plain = self._run_worker([])
        self.assertEqual(boxed, plain, "検出が無いのに別々にエンコードしている")


if __name__ == "__main__":
    unittest.main()
