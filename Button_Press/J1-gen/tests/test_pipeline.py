"""全体をつなぐ流れ（タスク6）を MuJoCo（同じプロセス）で確かめる。"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import tempfile
import unittest

import numpy as np

from common.config import FEATURE_DIR
from common.pipeline import EXIT_OK, EXIT_TARGET
from common.post_press import PostPressContext, PostPressResult
from common.robot_model import RIGHT_ARM_IDX

sys.path.insert(0, str(FEATURE_DIR / "sim"))
from press_bottle_sim import run_sim  # noqa: E402

# 記録は本番のフォルダ（_local/button_press/runs）ではなく、一時フォルダに書く
LOG_DIR = tempfile.TemporaryDirectory()


def args(**kw: object) -> argparse.Namespace:
    base = dict(label="test", arm=None, target=None, point=None, taught_pose=None, offset=None, seed_pose="",
                depth_mm=None, detector=None, gravity_scale=None, sim_detector="segmentation", confirm=False,
                log_dir=LOG_DIR.name)
    base.update(kw)
    return argparse.Namespace(**base)


def events(logger: object) -> list[dict]:
    with open(logger.dir / "events.jsonl", encoding="utf-8") as f:  # type: ignore[attr-defined]
        return [json.loads(line) for line in f]


def csv_rows(path: object) -> int:
    with open(path, encoding="utf-8") as f:  # type: ignore[arg-type]
        return sum(1 for _ in csv.reader(f)) - 1


class TestPipelineSim(unittest.TestCase):
    def test_depth_target_full_run(self) -> None:
        res, backend, logger = run_sim(args())
        self.assertEqual(res.code, EXIT_OK, res.message)
        ev = events(logger)
        names = [e["event"] for e in ev]
        stages = [e["stage"] for e in ev if e["event"] == "stage"]
        # 実測の腰の角度で計算し直す段階が必ずある
        self.assertIn("approach_replanned", stages)
        self.assertEqual(stages[:2], ["approach", "approach_replanned"])
        for n in ("start", "target_depth", "plan", "post_check", "result"):
            self.assertIn(n, names)
        # 記録が残っている
        d = logger.dir
        for f in ("meta.json", "plan_initial.json", "plan_approach_replanned.json"):
            self.assertTrue((d / f).exists(), f)
        self.assertGreater(csv_rows(d / "commands.csv"), 500)
        self.assertGreater(csv_rows(d / "lowstate.csv"), 500)
        self.assertTrue(list((d / "frames").glob("*_locate0_depth.png")))
        self.assertTrue(list((d / "frames").glob("*_end_color.png")))
        # 開始姿勢に戻っている
        q0 = np.array(json.loads((d / "meta.json").read_text())["configs"]["arm"]["sim"]["initial_q"])
        np.testing.assert_allclose(backend.read_state().q[list(RIGHT_ARM_IDX)], q0[list(RIGHT_ARM_IDX)], atol=0.02)
        # 対象はボトルの胴の前面の近く（x: 前面 0.387、y: 中心 -0.20）
        assert res.target is not None
        self.assertLess(abs(res.target[0] - 0.387), 0.005)
        self.assertLess(abs(res.target[1] + 0.20), 0.01)

    def test_calibration_offset_is_applied(self) -> None:
        """較正値（localize.yaml）は、depth の対象の位置に必ず足される。"""
        import common.pipeline_cli as cli

        orig = cli.load_pipeline_config

        def with_offset(a: argparse.Namespace):  # type: ignore[no-untyped-def]
            cfg = orig(a)
            cfg.localize["calibration"]["offset_pelvis_m"] = [0.004, -0.003, 0.002]
            return cfg

        base, _, _ = run_sim(args(label="calib0"))
        cli.load_pipeline_config = with_offset
        sys.modules["press_bottle_sim"].load_pipeline_config = with_offset  # type: ignore[attr-defined]
        try:
            shifted, _, logger = run_sim(args(label="calib1"))
        finally:
            cli.load_pipeline_config = orig
            sys.modules["press_bottle_sim"].load_pipeline_config = orig  # type: ignore[attr-defined]
        assert base.target is not None and shifted.target is not None
        np.testing.assert_allclose(shifted.target - base.target, [0.004, -0.003, 0.002], atol=1e-6)
        ev = [e for e in events(logger) if e["event"] == "target_depth"][0]
        self.assertEqual(ev["calibration_offset"], [0.004, -0.003, 0.002])

    def test_manual_target(self) -> None:
        res, _, logger = run_sim(args(target="manual", point=[0.388, -0.20, 0.04]))
        self.assertEqual(res.code, EXIT_OK, res.message)
        self.assertIn("target_manual", [e["event"] for e in events(logger)])

    def test_post_check_retry_deeper(self) -> None:
        """押したあとの確認が NG なら、深くして押し直す（ボタン版の差し込み口）。"""
        seen: list[float] = []

        def check(ctx: PostPressContext) -> PostPressResult:
            seen.append(ctx.depth_m)
            return PostPressResult(ok=ctx.attempt >= 2, deeper_m=0.005, message=f"{ctx.attempt} 回目")

        res, _, logger = run_sim(args(label="retry"), post_check=check)
        self.assertEqual(res.code, EXIT_OK, res.message)
        self.assertEqual(len(seen), 2)
        self.assertAlmostEqual(seen[1] - seen[0], 0.005)
        stages = [e["stage"] for e in events(logger) if e["event"] == "stage"]
        self.assertIn("retry_2_replanned", stages)  # 押し直しの前にも、実測の腰で計算し直す

    def test_post_check_retry_limited(self) -> None:
        calls: list[int] = []

        def never_ok(ctx: PostPressContext) -> PostPressResult:
            calls.append(ctx.attempt)
            return PostPressResult(ok=False, deeper_m=0.005)

        res, _, _ = run_sim(args(label="retry_limit"), post_check=never_ok)
        self.assertEqual(res.code, EXIT_OK)
        self.assertEqual(calls, [1, 2])  # press.yaml の post_check.max_retries = 1

    def test_unreachable_sends_nothing(self) -> None:
        res, _, logger = run_sim(args(target="manual", point=[0.9, -0.2, 0.0]))
        self.assertEqual(res.code, EXIT_TARGET)
        self.assertIn("何も送っていない", res.message)
        self.assertEqual(csv_rows(logger.dir / "commands.csv"), 0)


if __name__ == "__main__":
    unittest.main()
