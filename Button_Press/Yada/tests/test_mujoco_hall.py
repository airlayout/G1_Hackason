"""MuJoCo のシーン（sim/mujoco/mujoco_hall.py）のテスト。mujoco と公式モデルが無ければスキップする。"""

from __future__ import annotations

import importlib.util
import sys
import unittest

import numpy as np

from common.config import FEATURE_DIR, REPO_ROOT, load_config
from common.scene_spec import build_hall_scene

sys.path.insert(0, str(FEATURE_DIR / "sim" / "mujoco"))

MODELS = REPO_ROOT / "_local" / "button_press" / "models" / "g1_description" / "g1_29dof_rev_1_0.xml"
HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None
HAS_PIN = importlib.util.find_spec("pinocchio") is not None
SKIP_MODEL = "公式モデルが無い（bash Button_Press/J1-gen/sim/fetch_models.sh を実行していない）"


@unittest.skipUnless(HAS_MUJOCO, "mujoco が無い")
@unittest.skipUnless(MODELS.exists(), SKIP_MODEL)
class TestMujocoHall(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from view_hall import make

        cls.make = staticmethod(make)

    def _push(self, name: str, force: float, seconds: float = 0.3) -> tuple[float, bool]:
        import mujoco

        scene, _, model, data, hall = self.make("elevator_hall.yaml")
        idx = hall.robot_qpos_slice()
        q0 = data.qpos[idx].copy()
        bid = hall.body_id(name)
        max_depth = 0.0
        for _ in range(int(seconds / model.opt.timestep)):
            data.xfrc_applied[bid, :3] = [force, 0.0, 0.0]
            mujoco.mj_step(model, data)
            data.qpos[idx] = q0
            data.qvel[idx] = 0.0
            hall.update()
            max_depth = max(max_depth, hall.depth(name))
        return max_depth, hall.lit(name)

    def test_strong_push_lights(self) -> None:
        """6 N で押すと底（travel）まで沈み、押したと判定されて点灯する。"""
        depth, lit = self._push("up", 6.0)
        self.assertGreater(depth, 0.0035)
        self.assertTrue(lit)

    def test_weak_push_does_not_light(self) -> None:
        """ばね 1000 N/m で押したと判定する深さ 2.5 mm には 2.5 N 要る。1.5 N では点灯しない。"""
        depth, lit = self._push("down", 1.5)
        self.assertLess(depth, 0.0025)
        self.assertFalse(lit)

    def test_only_pushed_button_lights(self) -> None:
        import mujoco

        _, _, model, data, hall = self.make("elevator_hall.yaml")
        bid = hall.body_id("up")
        for _ in range(100):
            data.xfrc_applied[bid, :3] = [6.0, 0.0, 0.0]
            mujoco.mj_step(model, data)
            hall.update()
        self.assertTrue(hall.lit("up"))
        self.assertFalse(hall.lit("down"))
        hall.reset_lights()
        self.assertFalse(hall.lit("up"))

    def test_button_positions_match_spec(self) -> None:
        """MuJoCo のボタンの位置 = pelvis の位置 + scene_spec の位置（Isaac Sim と同じ決め方）。"""
        import mujoco

        scene, _, model, data, hall = self.make("elevator_hall.yaml")
        pelvis = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")]
        for b in scene.buttons:
            np.testing.assert_allclose(data.xpos[hall.body_id(b.name)] - pelvis, b.face_center, atol=1e-9)


@unittest.skipUnless(HAS_PIN, "pinocchio が無い")
@unittest.skipUnless(MODELS.exists(), SKIP_MODEL)
class TestReachable(unittest.TestCase):
    def test_right_fingertip_reaches_buttons(self) -> None:
        """右手の中指の先を、指を押す向き（+x）にそろえて、各ボタンの手前 5 cm と押し込んだ位置へ動かせる。"""
        from common.j1gen_bridge import j1gen, j1gen_config

        cfg = load_config("elevator_hall.yaml")
        scene = build_hall_scene(cfg)
        kin = j1gen("kinematics").ArmKinematics(j1gen_config("robot.yaml"), "right", j1gen_config("press.yaml")["ik"])
        q0 = np.zeros(29)
        for idx, deg in cfg["initial_pose_deg"].items():
            q0[int(idx)] = np.radians(float(deg))
        axis = np.array([1.0, 0.0, 0.0])
        for b in scene.buttons:
            approach = b.face_center - axis * 0.05
            r = kin.ik(approach, kin.ik(approach, q0).q, axis)
            self.assertTrue(r.success, f"{b.name} の手前: {r.reason}")
            r = kin.ik(b.face_center + axis * b.travel, r.q, axis)
            self.assertTrue(r.success, f"{b.name} の押し込み: {r.reason}")


if __name__ == "__main__":
    unittest.main()
