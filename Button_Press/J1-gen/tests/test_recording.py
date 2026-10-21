"""収録と再生（タスク5）。実機・DDS を使わず、作り物のフレームと lowstate で確かめる。"""

from __future__ import annotations

import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from _env import needs
from common.arm.types import JointState
from common.config import FEATURE_DIR, load_config
from common.localize import Locator
from common.recorder import Recorder
from common.recording import Recording, RecordingWriter
from common.rgbd_protocol import Intrinsics, RgbdFrame
from common.robot_model import NUM_MOTORS

sys.path.insert(0, str(FEATURE_DIR / "sim"))

INTR = Intrinsics(64, 48, 60.0, 60.0, 31.5, 23.5)


def make_frame(i: int) -> RgbdFrame:
    color = np.full((48, 64, 3), i % 255, np.uint8)
    depth = (np.arange(48 * 64).reshape(48, 64) + i).astype(np.uint16)
    return RgbdFrame(color, depth, 0.001, INTR, time.time(), i)


class FakeLowState:
    """lowstate の代わり。腰の角度が時間とともに変わる。"""

    def __init__(self) -> None:
        self.t0 = time.monotonic()

    def latest(self) -> JointState:
        t = time.monotonic() - self.t0
        q = np.zeros(NUM_MOTORS)
        q[12:15] = [0.1 * t, 0.0, -0.05]
        return JointState(q, np.zeros(NUM_MOTORS), np.ones(NUM_MOTORS, int), 5, time.monotonic())


class TestRecording(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def _getter(self, period: float = 0.01):  # type: ignore[no-untyped-def]
        n = [0]

        def get():  # type: ignore[no-untyped-def]
            time.sleep(period)
            n[0] += 1
            f = make_frame(n[0])
            return f.color_bgr, f
        return get

    def test_record_and_replay_all(self) -> None:
        w = RecordingWriter(self.root, "t", {"note": "テスト"})
        rec = Recorder(w, self._getter(), FakeLowState(), mode="all", lowstate_hz=200)
        n = rec.run(max_frames=10)
        w.close()
        self.assertEqual(n, 10)
        r = Recording(w.dir)
        self.assertEqual(len(r), 10)
        self.assertGreater(r.lowstate_count, 5)
        self.assertEqual(r.meta["note"], "テスト")
        self.assertEqual(r.meta["n_frames"], 10)
        frames = list(r)
        for k, fr in enumerate(frames):
            assert fr.rgbd is not None
            np.testing.assert_array_equal(fr.rgbd.depth, make_frame(k + 1).depth)  # 深度は完全に一致
            self.assertEqual(fr.rgbd.intrinsics, INTR)
            assert fr.waist_q is not None
            self.assertAlmostEqual(fr.waist_q[2], -0.05)
        # 腰の角度は時間とともに増えている（保存したときの値が入っている）
        self.assertGreater(frames[-1].waist_q[0], frames[0].waist_q[0])  # type: ignore[index]

    def test_interval_mode_thins_frames(self) -> None:
        w = RecordingWriter(self.root, "i", {})
        rec = Recorder(w, self._getter(0.01), None, mode="interval", interval_s=0.1)
        rec.run(duration_s=0.55)
        w.close()
        self.assertGreater(rec.n_received, 30)
        self.assertTrue(4 <= w.n_frames <= 7, w.n_frames)

    def test_enter_mode_saves_only_on_trigger(self) -> None:
        w = RecordingWriter(self.root, "e", {})
        flags = [False] * 20 + [True] + [False] * 10 + [True] + [False] * 100
        it = iter(flags)
        rec = Recorder(w, self._getter(0.001), None, mode="enter", enter_trigger=lambda: next(it, False))
        rec.run(duration_s=0.3)
        w.close()
        self.assertEqual(w.n_frames, 2)

    def test_rgb_only(self) -> None:
        w = RecordingWriter(self.root, "rgb", {}, color_format="jpg")
        rec = Recorder(w, lambda: (np.zeros((48, 64, 3), np.uint8), None), None)
        rec.run(max_frames=3)
        w.close()
        r = Recording(w.dir)
        fr = r.load(0)
        self.assertIsNone(fr.rgbd)
        self.assertEqual(fr.color_bgr.shape, (48, 64, 3))

    def test_stop_from_other_thread(self) -> None:
        w = RecordingWriter(self.root, "s", {})
        rec = Recorder(w, self._getter(), None)
        threading.Timer(0.2, rec.stop).start()
        rec.run()
        w.close()
        self.assertGreater(w.n_frames, 3)

    def test_truncated_lowstate_row_is_ignored(self) -> None:
        w = RecordingWriter(self.root, "c", {})
        Recorder(w, self._getter(), FakeLowState(), lowstate_hz=200).run(max_frames=5)
        w.close()
        with open(w.dir / "lowstate.csv", "a") as f:
            f.write("123.0,5,0.1,0.2")  # 落ちたときの書きかけの行
        r = Recording(w.dir)
        self.assertGreater(r.lowstate_count, 0)
        self.assertIsNotNone(r.q_at(0.0))


@needs('render', 'pin')
class TestOfflineLocate(unittest.TestCase):
    def test_locate_from_recorded_sim_frames(self) -> None:
        """MuJoCo のフレームを収録し、再生したフレームでタスク4（位置）を求めると、収録前と同じ結果になる。"""
        import mujoco

        from common.sim_camera import SimHeadCamera
        from common.sim_scene import BOTTLE, build_scene_model, detection_pose

        robot = load_config("robot.yaml")
        scene = load_config("sim_scene.yaml")
        m = build_scene_model(robot, scene)
        d = mujoco.MjData(m)
        d.qpos[:29] = detection_pose(scene, np.zeros(29))
        d.qpos[12:15] = [0.02, 0.0, 0.03]
        mujoco.mj_forward(m, d)
        cam = SimHeadCamera(m, d, robot)
        frame = cam.render()
        bbox = cam.segmentation_bbox(BOTTLE)
        assert bbox is not None
        cam.close()

        class SimState:
            def latest(self) -> JointState:
                return JointState(np.array(d.qpos[:29]), np.zeros(29), np.ones(29, int), 5, time.monotonic())

        with tempfile.TemporaryDirectory() as tmp:
            w = RecordingWriter(Path(tmp), "sim", {})
            Recorder(w, lambda: (frame.color_bgr, frame), SimState()).run(max_frames=2)
            w.close()
            fr = Recording(w.dir).load(0)
        assert fr.rgbd is not None and fr.waist_q is not None
        np.testing.assert_allclose(fr.waist_q, [0.02, 0.0, 0.03])
        loc = Locator(robot, load_config("localize.yaml"), None)
        a = loc.locate_bbox(frame, bbox, np.array([0.02, 0.0, 0.03]))
        b = loc.locate_bbox(fr.rgbd, bbox, fr.waist_q)
        assert a.p_pelvis is not None and b.p_pelvis is not None
        np.testing.assert_allclose(a.p_pelvis, b.p_pelvis, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
