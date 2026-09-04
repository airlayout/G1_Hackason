"""サンプル画像・サンプル動画でYOLO検出パイプラインが動くことを確認する簡単なテスト。

ZMQ接続(sim/real)は実機・シムが無いとテストできないため対象外。ここでは
FrameSource -> YoloDetector -> ResultWriter の一連の流れが例外なく動くことだけを
確認する(検出結果の中身は厳密にチェックしない。環境によって検出数が変わりうるため)。
"""
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from common.camera.video_file import VideoFileSource  # noqa: E402
from common.detector.yolo_detector import YoloDetector  # noqa: E402
from common.output.result_writer import ResultWriter  # noqa: E402
from common.pipeline import run_pipeline  # noqa: E402


def make_sample_image() -> np.ndarray:
    """簡単な図形を描いた640x480のサンプル画像を生成する(実物のカメラ映像の代用)。"""
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (50, 50), (300, 400), (200, 200, 200), thickness=-1)
    cv2.circle(image, (450, 200), 80, (100, 150, 200), thickness=-1)
    return image


class YoloDetectorSampleImageTest(unittest.TestCase):
    """サンプル画像に対してYOLO検出が例外なく動くことを確認する。"""

    def test_detect_runs_on_sample_image(self) -> None:
        detector = YoloDetector(model_name="yolo26n.pt", classes=["person"], device="cpu")
        frame = make_sample_image()

        detections = detector.detect(frame)

        self.assertIsInstance(detections, list)


class PipelineSampleVideoTest(unittest.TestCase):
    """サンプル動画(生成物)に対してパイプライン全体(FrameSource->Detector->Writer)が動くことを確認する。"""

    def test_run_pipeline_on_sample_video(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            video_path = Path(tmp_dir) / "sample.mp4"
            self._write_sample_video(video_path, num_frames=3)

            output_dir = Path(tmp_dir) / "outputs"
            config = {
                "source": {
                    "type": "video",
                    "video": {"path": str(video_path), "loop": False},
                },
                "detector": {
                    "model": "yolo26n.pt",
                    "classes": ["person"],
                    "confidence_threshold": 0.5,
                    "device": "cpu",
                },
                "output": {
                    "console": False,
                    "save_json": True,
                    "save_csv": True,
                    "output_dir": str(output_dir),
                },
            }

            run_pipeline(config, max_frames=3)

            self.assertTrue((output_dir / "run.jsonl").exists())
            self.assertTrue((output_dir / "run.csv").exists())

    @staticmethod
    def _write_sample_video(path: Path, num_frames: int) -> None:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(path), fourcc, 10.0, (640, 480))
        try:
            for _ in range(num_frames):
                writer.write(make_sample_image())
        finally:
            writer.release()


if __name__ == "__main__":
    unittest.main()
