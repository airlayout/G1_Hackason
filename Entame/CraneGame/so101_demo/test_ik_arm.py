import math
import random
import unittest

import ik_arm

try:
    import mujoco
    import sim_replay
except ImportError:          # mujoco が無い環境では MuJoCo との照合だけ飛ばす
    mujoco = None


class RoundTripTests(unittest.TestCase):
    def test_inverse_undoes_forward(self):
        rng = random.Random(3)
        for _ in range(200):
            q = (rng.uniform(-0.6, 1.2), rng.uniform(-1.0, 0.8), rng.uniform(-1.2, 1.2))
            (x, z), pitch = ik_arm.forward(*q)
            lift, elbow, wrist, reached = ik_arm.inverse(x, z, pitch)
            if not reached:
                continue
            (x2, z2), pitch2 = ik_arm.forward(lift, elbow, wrist)
            self.assertAlmostEqual(x, x2, places=6)
            self.assertAlmostEqual(z, z2, places=6)
            self.assertAlmostEqual(pitch, pitch2, places=4)

    def test_zero_angles_is_the_horizontal_forward_pose(self):
        (x, z), pitch = ik_arm.forward(0, 0, 0)
        self.assertAlmostEqual(x, 0.3914, places=3)
        self.assertLess(abs(pitch), 5)

    def test_unreachable_target_is_reported(self):
        self.assertFalse(ik_arm.inverse(0.9, 0.1, 0)[3])
        self.assertTrue(ik_arm.inverse(0.30, 0.15, -15)[3])

    def test_elbow_up_solution_stays_near_the_zero_pose(self):
        lift, elbow, wrist, reached = ik_arm.inverse(0.39, 0.2265, -2.8)
        self.assertTrue(reached)
        for angle in (lift, elbow, wrist):
            self.assertLess(abs(math.degrees(angle)), 3)


@unittest.skipIf(mujoco is None, "mujoco が無い")
class MujocoAgreementTests(unittest.TestCase):
    def test_forward_matches_mujoco(self):
        model = mujoco.MjModel.from_xml_path(str(sim_replay.MODEL_XML))
        data = mujoco.MjData(model)
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        names = ("shoulder_lift", "elbow_flex", "wrist_flex")
        rng = random.Random(5)
        for _ in range(50):
            q = [rng.uniform(-1.5, 1.5) for _ in names]
            data.qpos[:] = 0
            for name, value in zip(names, q):
                data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]] = value
            mujoco.mj_forward(model, data)
            (x, z), _ = ik_arm.forward(*q)
            self.assertAlmostEqual(x, data.site_xpos[site][0], places=5)
            self.assertAlmostEqual(z, data.site_xpos[site][2], places=5)


if __name__ == "__main__":
    unittest.main()
