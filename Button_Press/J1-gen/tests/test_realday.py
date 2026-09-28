"""実機日用のツールの部品（タスク7）。実機を使わずに確かめられる部分。"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from common.camera_geometry import HeadCameraTransform
from common.config import FEATURE_DIR, load_config
from common.localize import deproject
from common.realday import FingertipFK, load_taught_poses, project_to_image, save_taught_pose, taught_q_seed
from common.rgbd_protocol import Intrinsics

sys.path.insert(0, str(FEATURE_DIR / "real"))
import calibrate  # noqa: E402

INTR = Intrinsics(640, 480, 615.0, 615.0, 319.5, 239.5)


class TestProjection(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.robot = load_config("robot.yaml")

    def test_projection_inverts_deprojection(self) -> None:
        """画素と深度 → pelvis 座標 → 画素 で元に戻る（較正の補正と腰の角度があっても）。"""
        t = HeadCameraTransform(self.robot, np.array([0.01, -0.02, 0.005]))
        for qw in (np.zeros(3), np.array([0.1, -0.05, 0.08])):
            for u, v, z in [(100, 50, 0.4), (320, 240, 0.6), (600, 400, 0.9)]:
                p = t.to_pelvis(deproject(u, v, z, INTR), qw)
                uv = project_to_image(p, qw, t, INTR)
                assert uv is not None
                np.testing.assert_allclose(uv, (u, v), atol=1e-6)

    def test_point_behind_camera(self) -> None:
        t = HeadCameraTransform(self.robot)
        self.assertIsNone(project_to_image(np.array([-1.0, 0.0, 0.5]), np.zeros(3), t, INTR))

    def test_fingertip_projects_onto_hand_in_sim(self) -> None:
        """MuJoCo で、FK の指先を頭カメラに投影した画素に、右手（wrist_yaw_link の体）が写っている。"""
        import mujoco

        from common.robot_model import load_model
        from common.sim_camera import SimHeadCamera

        m = load_model(self.robot)
        d = mujoco.MjData(m)
        q = np.zeros(29)
        q[22], q[25] = -0.5, 0.3  # 右手を前に出す
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        cam = SimHeadCamera(m, d, self.robot)
        fk = FingertipFK(self.robot, load_config("press.yaml")["ik"])
        uv = project_to_image(fk.positions(q)["right"], np.zeros(3), HeadCameraTransform(self.robot), cam.intrinsics)
        assert uv is not None
        # 投影した画素のまわり（±6 画素）に、右手（wrist_yaw_link に付いたハンド）が写っているか
        r = cam._renderer
        r.enable_segmentation_rendering()
        r.update_scene(d, camera=cam.name)
        seg = r.render()
        r.disable_segmentation_rendering()
        u, v = int(round(uv[0])), int(round(uv[1]))
        self.assertTrue(0 <= u < cam.width and 0 <= v < cam.height, uv)
        win = seg[max(0, v - 6):v + 7, max(0, u - 6):u + 7]
        hand = m.body("right_wrist_yaw_link").id
        bodies = {int(m.geom_bodyid[g]) for g in win[:, :, 0].ravel() if g >= 0}
        cam.close()
        self.assertIn(hand, bodies)


class TestTaughtPoses(unittest.TestCase):
    def test_save_load_and_seed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "poses.yaml"
            q = np.zeros(29)
            q[22:29] = np.arange(7) * 0.1
            q[12:15] = [0.01, 0.02, 0.03]
            save_taught_pose("press_bottle", "right", q, np.array([0.4, -0.2, 0.05]), "メモ", path)
            save_taught_pose("other", "left", q, np.array([0.3, 0.2, 0.0]), path=path)
            poses = load_taught_poses(path)
            self.assertEqual(set(poses), {"press_bottle", "other"})
            self.assertEqual(poses["press_bottle"]["note"], "メモ")
            np.testing.assert_allclose(poses["press_bottle"]["arm_q_rad"], q[22:29])
            side, seed = taught_q_seed("press_bottle", np.ones(29), path)
            self.assertEqual(side, "right")
            np.testing.assert_allclose(seed[22:29], q[22:29])
            np.testing.assert_allclose(seed[:22], 1.0)  # 腕以外は今の値のまま
            with self.assertRaises(KeyError):
                taught_q_seed("none", np.zeros(29), path)
            self.assertTrue(path.read_text(encoding="utf-8").startswith("# Button_Press"))


class TestCalibrateHelpers(unittest.TestCase):
    def test_write_offset_keeps_comments(self) -> None:
        src = (FEATURE_DIR / "configs" / "localize.yaml").read_text(encoding="utf-8")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "localize.yaml"
            path.write_text(src, encoding="utf-8")
            calibrate.write_offset(np.array([0.0123, -0.0045, 0.0067]), path)
            text = path.read_text(encoding="utf-8")
            import yaml

            cfg = yaml.safe_load(text)
            np.testing.assert_allclose(cfg["calibration"]["offset_pelvis_m"], [0.0123, -0.0045, 0.0067])
            self.assertEqual(text.count("#"), src.count("#"))  # コメントは消えていない

    def test_place_table(self) -> None:
        pairs = [{"camera_raw": [0.4, -0.2, 0.05], "diff": [0.01, 0.0, 0.0]},
                 {"camera_raw": [0.45, -0.1, 0.05], "diff": [0.03, 0.0, 0.0]}]
        rows = calibrate.place_table(pairs)
        self.assertEqual(len(rows), 3)
        self.assertIn("+10.0", rows[1])
        self.assertTrue(rows[1].rstrip().endswith("10.0"))  # 平均（20 mm）からのずれ

    def test_confirm_support(self) -> None:
        from common.realday import confirm_support

        self.assertTrue(confirm_support(lambda _: ""))
        self.assertFalse(confirm_support(lambda _: "q"))

    def test_summarize(self) -> None:
        mean, std = calibrate.summarize([np.array([0.01, 0, 0]), np.array([0.03, 0, 0])])
        np.testing.assert_allclose(mean, [0.02, 0, 0])
        np.testing.assert_allclose(std, [0.01, 0, 0])


if __name__ == "__main__":
    unittest.main()
