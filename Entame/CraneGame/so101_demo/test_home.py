import tempfile
import unittest
from pathlib import Path

from home import HOME_JOINTS, load_home, neutral_map, parse_home, save_home
from limits import default_limits
from server import Desk, MockArm

HOME = {"shoulder_pan": 5.0, "shoulder_lift": -80.0, "elbow_flex": 70.0, "wrist_flex": 10.0, "wrist_roll": 0.0}


class NeutralMapTests(unittest.TestCase):
    def test_zero_is_home_and_ends_are_limits(self):
        self.assertEqual(neutral_map(0, -60, 40, -20), -20)
        self.assertEqual(neutral_map(1, -60, 40, -20), 40)
        self.assertEqual(neutral_map(-1, -60, 40, -20), -60)
        self.assertEqual(neutral_map(0.5, -60, 40, -20), 10)

    def test_without_home_uses_middle_of_range(self):
        self.assertEqual(neutral_map(0, -60, 40), -10)

    def test_home_outside_range_is_pulled_to_the_edge(self):
        self.assertEqual(neutral_map(0, -60, 40, -100), -60)
        self.assertEqual(neutral_map(-1, -60, 40, -100), -60)     # 下側は動かない
        self.assertEqual(neutral_map(1, -60, 40, -100), 40)

    def test_home_at_edge_has_no_room_on_that_side(self):
        self.assertEqual(neutral_map(-1, -100, 100, -100), -100)


class HomeFileTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.path = Path(self.dir.name) / "home.json"

    def test_round_trip(self):
        save_home(HOME, self.path)
        self.assertEqual(load_home(self.path), HOME)

    def test_missing_file_is_unregistered(self):
        self.assertIsNone(load_home(self.path))

    def test_broken_file_is_unregistered(self):
        for text in ("{", "[]", '{"shoulder_pan": 1}', '{"shoulder_pan":1,"shoulder_lift":1,"elbow_flex":1,'
                     '"wrist_flex":1,"wrist_roll":"x"}'):
            self.path.write_text(text)
            self.assertIsNone(load_home(self.path))

    def test_out_of_range_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_home({**HOME, "elbow_flex": 120})


class DeskHomeTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.file = Path(self.dir.name) / "home.json"
        self.time = [10.0]
        limits = default_limits()
        for j in HOME_JOINTS:
            limits[j].update(speed=150.0, step=15.0)          # 速度制限に邪魔されず、目標へ着かせる
        self.desk = Desk(MockArm(), clock=lambda: self.time[0], limits=limits, home_file=self.file)
        self.desk.arm.pose.update(HOME)

    def follow(self, token, **inputs):
        for _ in range(400):
            self.time[0] += 0.065
            self.desk.command(dict(op="sample", token=token, height=0.5, opening=0.5, **inputs))
        return self.desk.arm.read()

    def test_records_current_pose_and_saves_it(self):
        self.desk.command(dict(op="home"))
        self.assertEqual(self.desk.state()["home"], HOME)
        self.assertEqual(load_home(self.file), HOME)

    def test_neutral_hand_keeps_the_home_pose_and_ends_reach_the_limits(self):
        self.desk.command(dict(op="home"))
        token = self.desk.command(dict(op="start", gripper_only=True))["token"]
        neutral = self.follow(token, lift=0, reach=0, pan=0, flex=0, roll=0)
        for j in HOME_JOINTS:
            self.assertAlmostEqual(neutral[j], HOME[j], places=3)
        up = self.follow(token, lift=1, reach=-1)
        self.assertAlmostEqual(up["shoulder_lift"], 100.0, places=3)
        self.assertAlmostEqual(up["elbow_flex"], -100.0, places=3)

    def test_start_is_allowed_from_any_pose(self):
        self.desk.command(dict(op="home"))
        self.desk.arm.pose["shoulder_lift"] += 30          # 初期姿勢から離れていても、基準を取るので開始できる
        self.assertTrue(self.desk.command(dict(op="start", gripper_only=True))["token"])

    def test_start_without_home_is_allowed(self):
        self.desk.arm.pose["shoulder_lift"] = 90.0
        self.assertTrue(self.desk.command(dict(op="start", gripper_only=True))["token"])

    def test_cannot_record_while_following(self):
        self.desk.command(dict(op="start", gripper_only=True))
        with self.assertRaises(ValueError):
            self.desk.command(dict(op="home"))

    def test_unwritable_file_keeps_the_old_home(self):
        self.desk.home_file = Path(self.dir.name) / "missing" / "home.json"
        with self.assertRaises(ValueError):
            self.desk.command(dict(op="home"))
        self.assertIsNone(self.desk.state()["home"])


if __name__ == "__main__":
    unittest.main()
