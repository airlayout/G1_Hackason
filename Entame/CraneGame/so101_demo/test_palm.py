import json
import tempfile
import unittest
from pathlib import Path

from home import HOME_JOINTS
from limits import default_limits
from palm import load_palm_up, parse_palm_up, save_palm_up
from server import Desk, MockArm
from skeleton import FEATURES, POSE_NAMES

BASE = {"shoulder_pan": 0.0, "shoulder_lift": -10.0, "elbow_flex": 10.0, "wrist_flex": 0.0, "wrist_roll": 0.0}
UP = 60.0


def human_for(robot):
    return {key: 0.9*robot[joint] + 10.0 for joint, key in FEATURES.items()}


class PalmFileTests(unittest.TestCase):
    def test_round_trip_and_bad_files(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "palm.json"
            self.assertIsNone(load_palm_up(path))
            save_palm_up(UP, path)
            self.assertEqual(load_palm_up(path), UP)
            path.write_text("{")
            self.assertIsNone(load_palm_up(path))
            path.write_text(json.dumps({"wrist_roll": 500}))
            self.assertIsNone(load_palm_up(path))

    def test_parse_rejects_bad_values(self):
        for bad in (None, {}, {"wrist_roll": "a"}, {"wrist_roll": float("nan")}, {"wrist_roll": 101}):
            with self.assertRaises(ValueError):
                parse_palm_up(bad)


class PalmDeskTests(unittest.TestCase):
    def setUp(self):
        self.time = [10.0]
        limits = default_limits()
        for j in HOME_JOINTS:
            limits[j].update(speed=150.0, step=15.0)
        self.folder = tempfile.TemporaryDirectory()
        self.palm_file = Path(self.folder.name) / "palm.json"
        self.desk = Desk(MockArm(), clock=lambda: self.time[0], limits=limits, home=dict(BASE), palm_file=self.palm_file)
        self.desk.arm.pose.update(BASE)

    def tearDown(self):
        self.folder.cleanup()

    def run_command(self, **data):
        self.time[0] += 0.05
        return self.desk.command(data)

    def register_up(self):
        self.desk.arm.pose["wrist_roll"] = UP
        self.run_command(op="palm_set")
        self.desk.arm.pose["wrist_roll"] = BASE["wrist_roll"]

    def follow(self):
        for i in range(len(POSE_NAMES)):
            self.run_command(op="skeleton_pose", index=i)
            for _ in range(400):
                self.time[0] += 0.03
                self.desk.tick()
                if not self.desk.mover.active:
                    break
            self.run_command(op="skeleton_capture", index=i, features=human_for(self.desk.arm.read()))
        self.desk.arm.pose["wrist_roll"] = BASE["wrist_roll"]
        return self.run_command(op="skeleton_start")["token"]

    def sample(self, token, palm, times=60):
        for _ in range(times):
            self.run_command(op="skeleton_sample", token=token, features=human_for(BASE), palm=palm)

    def test_palm_set_saves_the_current_roll(self):
        self.register_up()
        self.assertEqual(self.desk.palm_up, UP)
        self.assertEqual(load_palm_up(self.palm_file), UP)
        self.assertEqual(self.desk.state()["skeleton"]["palm_up"], UP)

    def test_palm_set_refuses_a_roll_close_to_the_home(self):
        with self.assertRaises(ValueError):
            self.run_command(op="palm_set")
        self.assertIsNone(self.desk.palm_up)
        self.assertFalse(self.palm_file.exists())

    def test_default_is_palm_down_and_up_needs_registration(self):
        token = self.follow()
        self.sample(token, "down", 10)
        self.assertEqual(self.desk.arm.read()["wrist_roll"], BASE["wrist_roll"])
        with self.assertRaises(ValueError):
            self.sample(token, "up", 1)

    def test_up_then_down_moves_the_wrist_roll(self):
        self.register_up()
        token = self.follow()
        self.sample(token, "up")
        self.assertAlmostEqual(self.desk.arm.read()["wrist_roll"], UP, delta=1.0)
        self.sample(token, "down")
        self.assertAlmostEqual(self.desk.arm.read()["wrist_roll"], BASE["wrist_roll"], delta=1.0)

    def test_unknown_palm_is_rejected(self):
        token = self.follow()
        with self.assertRaises(ValueError):
            self.sample(token, "left", 1)

    def test_cannot_register_while_following(self):
        self.register_up()
        self.follow()
        with self.assertRaises(ValueError):
            self.run_command(op="palm_set")

    def test_start_refused_when_up_and_down_are_too_close(self):
        self.desk.palm_up = BASE["wrist_roll"] + 5.0
        with self.assertRaises(ValueError):
            self.follow()
        self.assertFalse(self.desk.state()["skeleton"]["following"])


if __name__ == "__main__":
    unittest.main()
