"""動画ファイルが実時間で流れることのテスト。

⚠️ **サンプル動画は git に入っていない**ので、ここでは既知の fps の動画をその場で
作って使う。YOLO は要らない。

なぜこのテストが要るか: 素の `VideoFileSource` には時間の概念が無く、24fps の動画を
10fps で読むと 2.4 倍遅くなる。見た目が間延びするだけでなく、**検知ログの継続時間が
同じ倍率で水増しされる**（2026-09-20 に実測して判明）。
"""
from __future__ import annotations

import sys
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

MAIN = Path(__file__).resolve().parents[1] / "Main"
sys.path.insert(0, str(MAIN))
sys.path.insert(0, str(MAIN.parents[2] / "Perception"))

from common.camera.video_file import VideoFileSource     # noqa: E402
from video_realtime import RealtimeVideoSource           # noqa: E402

VIDEO_FPS = 30.0
VIDEO_FRAMES = 150          # 5 秒ぶん
READ_HZ = 10.0              # UI の target_fps を模す
MEASURE_S = 3.0


def make_video(path: Path) -> None:
    """既知の fps・長さの動画を作る。"""
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"),
                             VIDEO_FPS, (160, 120))
    for i in range(VIDEO_FRAMES):
        frame = np.full((120, 160, 3), i % 256, dtype=np.uint8)
        writer.write(frame)
    writer.release()


def consume(source, seconds: float) -> tuple[int, float]:
    """`READ_HZ` で read し続け、(動画を進めた枚数, かかった秒数) を返す。"""
    source.open()
    interval = 1.0 / READ_HZ
    reads, t0 = 0, time.monotonic()
    while time.monotonic() - t0 < seconds:
        start = time.monotonic()
        if source.read() is None:
            break
        reads += 1
        rest = interval - (time.monotonic() - start)
        if rest > 0:
            time.sleep(rest)
    dt = time.monotonic() - t0
    advanced = getattr(source, "_index", reads)   # 捨てた分も含む
    source.close()
    return advanced, dt


class RealtimeVideoTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls._dir = tempfile.TemporaryDirectory()
        cls.path = Path(cls._dir.name) / "sample.mp4"
        make_video(cls.path)
        if not cls.path.exists() or cls.path.stat().st_size == 0:
            raise unittest.SkipTest("この環境では mp4 を書けない（コーデック不足）")

    @classmethod
    def tearDownClass(cls) -> None:
        cls._dir.cleanup()

    def test_probes_fps(self) -> None:
        src = RealtimeVideoSource(str(self.path), loop=True)
        self.assertAlmostEqual(src._fps, VIDEO_FPS, delta=1.0)

    def test_plays_at_real_time(self) -> None:
        src = RealtimeVideoSource(str(self.path), loop=True)
        advanced, dt = consume(src, MEASURE_S)
        rate = (advanced / VIDEO_FPS) / dt
        self.assertGreater(rate, 0.85, f"再生が遅すぎる（{rate:.2f} 倍）")
        self.assertLess(rate, 1.15, f"再生が速すぎる（{rate:.2f} 倍）")
        self.assertGreater(src.dropped, 0, "フレームを1枚も捨てていない")

    def test_naive_source_is_slower(self) -> None:
        """包まないと遅いこと。**この包みが要る理由そのもの。**"""
        advanced, dt = consume(VideoFileSource(str(self.path), loop=True), MEASURE_S)
        rate = (advanced / VIDEO_FPS) / dt
        expected = READ_HZ / VIDEO_FPS          # 10/30 = 0.33 倍
        self.assertAlmostEqual(rate, expected, delta=0.1,
                               msg=f"素の読み方の速度が想定と違う（{rate:.2f} 倍）")


if __name__ == "__main__":
    unittest.main()
