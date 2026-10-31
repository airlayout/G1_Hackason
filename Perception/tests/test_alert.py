import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from common.alert import AlertStore, EventJudge, draw_detections, make_record
from common.detector.types import Detection


class TestEventJudge(unittest.TestCase):
    def test_fires_after_min_frames(self) -> None:
        judge = EventJudge(min_frames=3, cooldown_s=10.0)
        self.assertEqual([judge.update(True, t) for t in (0.0, 0.1, 0.2)], [False, False, True])

    def test_streak_resets_on_missing_frame(self) -> None:
        judge = EventJudge(min_frames=3, cooldown_s=10.0)
        results = [judge.update(p, i * 0.1) for i, p in enumerate([True, True, False, True, True, True])]
        self.assertEqual(results, [False, False, False, False, False, True])

    def test_cooldown_then_repeat_while_present(self) -> None:
        judge = EventJudge(min_frames=1, cooldown_s=10.0)
        self.assertTrue(judge.update(True, 0.0))
        self.assertFalse(judge.update(True, 5.0))
        self.assertTrue(judge.update(True, 10.0))


class TestAlertStore(unittest.TestCase):
    def test_save_and_prune(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            store = AlertStore(Path(tmp), max_photos=3)
            dets = [Detection(class_name="person", confidence=0.9, bbox=(1.0, 2.0, 3.0, 4.0))]
            names = [store.save(b"jpeg", make_record(dets, 1000.0 + i, i)).name for i in range(5)]
            left = sorted(p.name for p in Path(tmp).glob("*.jpg"))
            self.assertEqual(left, sorted(names)[-3:])  # 古い 2 枚が消える
            lines = (Path(tmp) / "events.jsonl").read_text().splitlines()
            self.assertEqual(len(lines), 5)  # 記録は消さない
            self.assertEqual(json.loads(lines[-1])["photo"], names[-1])

    def test_draw_does_not_change_input(self) -> None:
        img = np.zeros((20, 20, 3), dtype=np.uint8)
        out = draw_detections(img, [Detection(class_name="person", confidence=0.5, bbox=(2.0, 2.0, 15.0, 15.0))])
        self.assertEqual(int(img.sum()), 0)
        self.assertGreater(int(out.sum()), 0)


if __name__ == "__main__":
    unittest.main()
