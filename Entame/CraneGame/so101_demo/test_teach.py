import json
import tempfile
import unittest
from pathlib import Path

from limits import JOINTS
from server import Desk, MockArm
from teach import HUMAN_IDS, HUMAN_POSES, load_teach, parse_pose, parse_teach, save_teach

POSE = {"shoulder_pan": -5.0, "shoulder_lift": -40.0, "elbow_flex": 30.0, "wrist_flex": 60.0, "wrist_roll": 3.0, "gripper": 20.0}


class TeachFileTests(unittest.TestCase):
    def test_human_poses_are_ten_with_unique_ids(self):
        self.assertEqual(len(HUMAN_POSES), 10)
        self.assertEqual(len(set(HUMAN_IDS)), 10)

    def test_round_trip_and_bad_files(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "teach.json"
            self.assertEqual(load_teach(path), {})
            data = {"H3": {"robot": POSE, "time": "2026-11-22 10:00:00"}}
            save_teach(data, path)
            self.assertEqual(load_teach(path), data)
            self.assertFalse(path.with_name("teach.json.tmp").exists())
            path.write_text("{")
            self.assertEqual(load_teach(path), {})
            path.write_text(json.dumps({"H99": {"robot": POSE}}))
            self.assertEqual(load_teach(path), {})

    def test_parse_pose_rejects_bad_values(self):
        for bad in (None, {}, {**POSE, "elbow_flex": 101}, {**POSE, "gripper": -1}, {**POSE, "wrist_roll": float("nan")},
                    {**POSE, "wrist_roll": "a"}, {k: v for k, v in POSE.items() if k != "gripper"}):
            with self.assertRaises(ValueError):
                parse_pose(bad)
        self.assertEqual(set(parse_pose(POSE)), set(JOINTS))

    def test_parse_teach_rejects_unknown_id(self):
        with self.assertRaises(ValueError):
            parse_teach({"H11": {"robot": POSE}})


class TeachDeskTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.file = Path(self.folder.name) / "teach.json"
        self.desk = Desk(MockArm(), teach_file=self.file)

    def tearDown(self):
        self.folder.cleanup()

    def run_command(self, **data):
        return self.desk.command(data)

    def move_to(self, pose):
        self.desk.arm.pose.update(pose)

    def test_save_records_the_current_joints_and_persists(self):
        self.move_to(POSE)
        state = self.run_command(op="teach_save", id="H4")
        self.assertEqual(state["teach"]["saved"]["H4"]["robot"], POSE)
        self.assertEqual(load_teach(self.file)["H4"]["robot"], POSE)
        self.assertEqual(len(state["teach"]["poses"]), 10)

    def test_save_overwrites_the_same_id_and_keeps_others(self):
        self.move_to(POSE)
        self.run_command(op="teach_save", id="H1")
        self.move_to({"shoulder_lift": 10.0})
        state = self.run_command(op="teach_save", id="H1")
        self.assertEqual(state["teach"]["saved"]["H1"]["robot"]["shoulder_lift"], 10.0)
        self.move_to({"shoulder_lift": -80.0})
        state = self.run_command(op="teach_save", id="H2")
        self.assertEqual(set(state["teach"]["saved"]), {"H1", "H2"})

    def test_unknown_id_is_rejected_and_nothing_saved(self):
        for bad in ("H0", "h1", None, ["H1"], 3):
            with self.assertRaises(ValueError):
                self.run_command(op="teach_save", id=bad)
        self.assertFalse(self.file.exists())
        self.assertEqual(self.desk.teach, {})

    def test_warns_when_the_pose_is_the_same_as_another(self):
        self.move_to(POSE)
        self.run_command(op="teach_save", id="H1")
        state = self.run_command(op="teach_save", id="H2")
        self.assertIn("ほぼ同じ", state["note"])

    def test_save_works_with_motors_off(self):
        self.run_command(op="drive", mode="off")
        self.move_to(POSE)
        state = self.run_command(op="teach_save", id="H5")
        self.assertIn("H5", state["teach"]["saved"])

    def test_delete(self):
        self.move_to(POSE)
        self.run_command(op="teach_save", id="H1")
        state = self.run_command(op="teach_delete", id="H1")
        self.assertEqual(state["teach"]["saved"], {})
        self.assertEqual(load_teach(self.file), {})
        with self.assertRaises(ValueError):
            self.run_command(op="teach_delete", id="H1")

    def test_failed_write_changes_nothing(self):
        self.desk.teach_file = Path(self.folder.name) / "missing" / "teach.json"
        with self.assertRaises(ValueError):
            self.run_command(op="teach_save", id="H1")
        self.assertEqual(self.desk.teach, {})

    def test_cannot_save_while_following(self):
        self.desk.token, self.desk.last = "t", self.desk.clock()      # 入力途絶の時間切れで止まらないように
        with self.assertRaises(ValueError):
            self.run_command(op="teach_save", id="H1")


if __name__ == "__main__":
    unittest.main()
