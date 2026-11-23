import json
import unittest
from pathlib import Path

import reach_map

HERE = Path(__file__).resolve().parent
try:
    import crane_sim
    import sim_replay
except ImportError:          # mujoco / rerun が無い環境では、シミュレーションの試験だけ飛ばす
    crane_sim = None

try:
    CALIBRATION = sim_replay.load_calibration()          # 実機の較正（lerobot のキャッシュ）
except Exception:                                        # 無い環境では、近い値の代用
    CALIBRATION = {j: {"range_min": 800, "range_max": 3300} for j in ("shoulder_lift", "elbow_flex", "wrist_flex")}
UNITS = reach_map.joint_units(CALIBRATION)
HOME = json.loads((HERE / "home.json").read_text())


class EnvelopeTests(unittest.TestCase):
    def test_home_pose_is_inside(self):
        self.assertTrue(reach_map.inside_envelope(HOME, UNITS))

    def test_pose_above_height_limit_is_outside(self):
        pose = {"shoulder_lift": -30.0, "elbow_flex": 0.0, "wrist_flex": 0.0}      # 腕をほぼ水平に伸ばした高い姿勢
        self.assertGreater(reach_map.height_above_desk(pose, UNITS), reach_map.HEIGHT_MAX)
        self.assertFalse(reach_map.inside_envelope(pose, UNITS))

    def test_shoulder_back_swing_is_outside(self):
        self.assertFalse(reach_map.inside_envelope({**HOME, "shoulder_lift": -60.0}, UNITS))

    def test_clamp_stops_before_the_outside(self):
        want = {"shoulder_lift": -30.0, "elbow_flex": 0.0, "wrist_flex": 0.0}
        got = reach_map.clamp_to_envelope(want, HOME, UNITS)
        self.assertTrue(reach_map.inside_envelope(got, UNITS))
        self.assertNotEqual(got, want)

    def test_clamp_keeps_inside_target(self):
        self.assertEqual(reach_map.clamp_to_envelope(HOME, HOME, UNITS), HOME)


class TargetTests(unittest.TestCase):
    def test_every_hand_input_gives_a_pose_inside_the_envelope(self):
        anchor = reach_map.tip_of(HOME, UNITS)[0]
        safe = {j: float(HOME[j]) for j in reach_map.USABLE}
        steps = [i / 4 for i in range(-4, 5)]
        for reach in steps:
            for height in steps:
                _, pose = reach_map.target(reach, height, anchor, UNITS, safe)
                self.assertTrue(reach_map.inside_envelope(pose, UNITS), (reach, height, pose))

    def test_fully_extended_hand_straightens_the_elbow_and_reaches_far(self):
        # 利用者の指摘（2026-11-22）: 腕を伸ばしても肘が浅く、遠くへ届かない。伸ばし切ると腕がほぼまっすぐ（まっすぐ＝肘 約 -75）になること
        anchor = reach_map.tip_of(HOME, UNITS)[0]
        safe = {j: float(HOME[j]) for j in reach_map.USABLE}
        (x, z), pose = reach_map.target(1.0, 0.0, anchor, UNITS, safe)
        self.assertGreaterEqual(x, 0.43)
        self.assertLessEqual(pose["elbow_flex"], -55.0)
        self.assertLessEqual(z - reach_map.Z_DESK, 0.02)             # 机の高さで届いている（宙に上がって稼いでいない）

    def test_extending_the_hand_moves_the_tip_forward_not_up(self):
        anchor = reach_map.tip_of(HOME, UNITS)[0]
        safe = {j: float(HOME[j]) for j in reach_map.USABLE}
        (x0, z0), _ = reach_map.target(0.0, 0.0, anchor, UNITS, safe)
        (x1, z1), _ = reach_map.target(1.0, 0.0, anchor, UNITS, safe)
        self.assertGreater(x1 - x0, 0.10)
        self.assertLess(z1 - z0, 0.05)

    def test_neutral_maps_input_range_around_the_base(self):
        self.assertEqual(reach_map.neutral(0, 0.0, 1.0, 0.4), 0.4)
        self.assertEqual(reach_map.neutral(1, 0.0, 1.0, 0.4), 1.0)
        self.assertEqual(reach_map.neutral(-1, 0.0, 1.0, 0.4), 0.0)


@unittest.skipIf(crane_sim is None, "mujoco / rerun が無い")
class CraneSimTests(unittest.TestCase):
    def setUp(self):
        self.units = reach_map.joint_units(sim_replay.load_calibration())
        self.limits = json.loads((HERE / "limits.json").read_text())
        self.anchor = reach_map.tip_of(HOME, self.units)[0]

    def run_script(self, x):
        snack = (x, 0.0, reach_map.Z_DESK + crane_sim.SNACK_HALF[2])
        inputs = crane_sim.scripted_inputs(crane_sim.script_for(snack, self.anchor))
        frames = crane_sim.run(inputs, self.units, {**HOME, "gripper": 100.0}, self.limits, snack)
        return crane_sim.judge(frames, snack)

    def test_pan_and_roll_inputs_move_the_joints_and_without_them_they_stay_at_home(self):
        inputs = [(0, 0.0, 0.0, 1.0)] * 60
        home = {**HOME, "gripper": 100.0}
        still = crane_sim.run(inputs, self.units, home, self.limits, None)
        moved = crane_sim.run(inputs, self.units, home, self.limits, None,
                              free={"shoulder_pan": [1.0] * 60, "wrist_roll": [1.0] * 60})
        for joint in ("shoulder_pan", "wrist_roll"):
            self.assertAlmostEqual(still[-1]["pose"][joint], HOME[joint], delta=0.5)
            self.assertGreater(abs(moved[-1]["pose"][joint] - HOME[joint]), 10, joint)

    def test_free_target_maps_like_the_real_server(self):
        limits = {"shoulder_pan": {"min": 20.0, "max": 63.0}}
        self.assertAlmostEqual(crane_sim.free_target("shoulder_pan", 1.0, -2.0, limits), 26.0)
        self.assertAlmostEqual(crane_sim.free_target("shoulder_pan", -1.0, -2.0, limits), -60.0)
        self.assertAlmostEqual(crane_sim.free_target("shoulder_pan", 0.0, -2.0, limits), -2.0)

    def test_gripper_reaches_snack_between_18_and_30_cm(self):
        for x in (0.18, 0.22, 0.26, 0.30):
            result = self.run_script(x)
            self.assertTrue(result["reached"], (x, result))

    def test_never_leaves_the_envelope(self):
        for x in (0.18, 0.30):
            result = self.run_script(x)
            self.assertGreaterEqual(result["shoulder_min"], crane_sim.SHOULDER_BACK_LIMIT)
            self.assertLessEqual(result["height_max"], reach_map.HEIGHT_MAX)
            self.assertGreaterEqual(result["pitch"][0], reach_map.PITCH_RANGE[0])

    def test_input_for_inverts_neutral(self):
        for value in (0.19, 0.25, 0.4):
            hand = crane_sim.input_for(value, reach_map.X_MIN, reach_map.X_MAX, 0.19)
            self.assertAlmostEqual(reach_map.neutral(hand, reach_map.X_MIN, reach_map.X_MAX, 0.19), value, places=6)


if __name__ == "__main__":
    unittest.main()
