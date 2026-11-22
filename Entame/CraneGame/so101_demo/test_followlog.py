import json
import tempfile
import unittest
from pathlib import Path

from followlog import FollowLog
from server import Desk, MockArm


def read(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


class FollowLogTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.folder = Path(self.dir.name)
        self.addCleanup(self.dir.cleanup)

    def test_writes_events_as_json_lines_and_closes_on_end(self):
        log = FollowLog(self.folder)
        log.begin(mode="x", pose={"a": 1.23456})
        log.write("sample", inp={"v": 0.5})
        log.end("理由")
        events = read(log.path)
        self.assertEqual([e["ev"] for e in events], ["start", "sample", "stop"])
        self.assertEqual(events[0]["pose"]["a"], 1.23)           # 小数は 2 桁
        self.assertEqual(events[2]["reason"], "理由")
        log.write("sample")                                       # 閉じた後は何も書かない
        self.assertEqual(len(read(log.path)), 3)

    def test_end_without_begin_does_nothing(self):
        FollowLog(self.folder).end("x")
        self.assertEqual(list(self.folder.glob("follow-*")), [])

    def test_keeps_only_newest_logs(self):
        times = iter(range(1000, 1100))
        log = FollowLog(self.folder, now=lambda: next(times), keep=3)
        for _ in range(6):
            log.begin(mode="x")
        log.close()
        self.assertEqual(len(list(self.folder.glob("follow-*.jsonl"))), 3)

    def test_prune_leaves_other_files(self):
        (self.folder / "snap-1.jpg").write_bytes(b"x")
        log = FollowLog(self.folder, keep=1)
        log.begin(mode="x")
        log.begin(mode="x")
        log.close()
        self.assertTrue((self.folder / "snap-1.jpg").exists())

    def test_unwritable_folder_does_not_raise(self):
        blocker = self.folder / "file"
        blocker.write_text("")
        log = FollowLog(blocker / "debug")                       # フォルダを作れない
        log.begin(mode="x")
        log.write("sample")
        log.end("x")


class DeskFollowLogTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.time = [10.0]
        self.desk = Desk(MockArm(), clock=lambda: self.time[0], log=FollowLog(self.dir.name))
        self.addCleanup(self.desk.log.close)                      # 先に閉じてからフォルダを消す（後に登録＝先に実行）
        self.token = self.desk.command(dict(op="start", gripper_only=True))["token"]

    def sample(self, **extra):
        self.time[0] += 0.065
        self.desk.command(dict(op="sample", token=self.token, height=0.5, opening=0.5, **extra))

    def events(self):
        return read(self.desk.log.path)

    def test_records_start_samples_and_stop_reason(self):
        self.sample(pan=0.5, roll=-0.2, dbg=dict(ratio=0.4, tilt=-8.0))
        self.desk.command(dict(op="stop", why="手を見失った"))
        start, sample, stop = self.events()
        self.assertEqual((start["ev"], start["mode"]), ("start", "gripper_only"))
        self.assertIn("limits", start)
        self.assertEqual(sample["inp"], dict(height=0.5, opening=0.5, pan=0.5, roll=-0.2))
        self.assertEqual(sample["dbg"], dict(ratio=0.4, tilt=-8.0))
        self.assertEqual(set(sample["goal"]), {"gripper", "shoulder_pan", "wrist_roll"})
        self.assertEqual(set(sample["want"]), set(sample["goal"]))
        self.assertEqual(stop["reason"], "停止（手を見失った）")

    def test_bad_debug_values_are_dropped_not_fatal(self):
        self.sample(dbg=dict(ok=1, bad="x", nan=float("nan"), flag=True))
        self.assertEqual(self.events()[1]["dbg"], dict(ok=1.0))
        self.sample(dbg="not a dict")
        self.assertEqual(self.events()[2]["dbg"], {})

    def test_timeout_and_drive_change_close_the_log(self):
        self.time[0] += 100
        self.desk.expired()
        self.assertEqual(self.events()[-1]["ev"], "stop")
        self.assertIn("入力途絶", self.events()[-1]["reason"])
        self.token = self.desk.command(dict(op="start", gripper_only=True))["token"]
        self.desk.command(dict(op="drive", mode="on"))
        self.assertEqual(self.events()[-1]["reason"], "モーターの状態を切り替え")

    def test_each_follow_gets_its_own_file(self):
        first = self.desk.log.path
        self.desk.command(dict(op="stop"))
        self.time[0] += 1
        self.desk.command(dict(op="start", gripper_only=True))
        self.assertNotEqual(self.desk.log.path, first)


class ShoulderElbowTests(unittest.TestCase):
    """肩（lift）と肘（reach）は、送られた関節だけが入力の向きへ動く。"""

    def move(self, **inputs):
        time = [10.0]
        desk = Desk(MockArm(), clock=lambda: time[0])
        token = desk.command(dict(op="start", gripper_only=True))["token"]
        before = desk.arm.read()
        time[0] += 0.065
        desk.command(dict(op="sample", token=token, height=0.5, opening=0.5, **inputs))
        return before, desk.arm.read()

    def test_each_input_moves_only_its_joint(self):
        for key, joint in (("lift", "shoulder_lift"), ("reach", "elbow_flex")):
            for value, sign in ((1.0, 1), (-1.0, -1)):
                before, after = self.move(**{key: value})
                self.assertGreater((after[joint]-before[joint])*sign, 0)
                for other in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
                    if other != joint:
                        self.assertEqual(after[other], before[other])

    def test_out_of_range_is_rejected(self):
        for key in ("lift", "reach"):
            with self.assertRaises(ValueError):
                self.move(**{key: 1.2})

    def test_all_arm_inputs_together(self):
        before, after = self.move(pan=1, lift=1, reach=1, flex=1, roll=1)
        for joint in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
            self.assertGreater(after[joint], before[joint])


class PanTests(unittest.TestCase):
    def test_pan_moves_only_shoulder_pan_toward_its_side(self):
        for pan, sign in ((1.0, 1), (-1.0, -1)):
            time = [10.0]
            desk = Desk(MockArm(), clock=lambda: time[0])
            token = desk.command(dict(op="start", gripper_only=True))["token"]
            before = desk.arm.read()
            time[0] += 0.065
            desk.command(dict(op="sample", token=token, height=0.5, opening=0.5, pan=pan))
            after = desk.arm.read()
            self.assertGreater((after["shoulder_pan"]-before["shoulder_pan"])*sign, 0)
            for joint in ("shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
                self.assertEqual(after[joint], before[joint])

    def test_pan_out_of_range_is_rejected(self):
        desk = Desk(MockArm(), clock=lambda: 10.0)
        token = desk.command(dict(op="start", gripper_only=True))["token"]
        with self.assertRaises(ValueError):
            desk.command(dict(op="sample", token=token, height=0.5, opening=0.5, pan=2))


if __name__ == "__main__":
    unittest.main()
