import json
import tempfile
import unittest
import unittest.mock
from pathlib import Path

from followlog import FollowLog
from limits import default_limits
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

    def test_records_the_body_skeleton_in_the_sample(self):
        self.sample(body=dict(ls_x=0.61, ls_y=0.42, rs_x=0.38, rs_y=0.43, bad="x"))
        self.assertEqual(self.events()[1]["body"], dict(ls_x=0.61, ls_y=0.42, rs_x=0.38, rs_y=0.43))

    def test_records_the_input_after_baseline_and_flip(self):
        self.sample(lift=0.5, pan=-0.25)
        self.assertEqual(self.events()[1]["rel"], dict(shoulder_pan=-0.25, shoulder_lift=0.5))

    def test_saves_a_log_image_next_to_the_log_and_notes_it(self):
        import base64
        jpeg = "data:image/jpeg;base64," + base64.b64encode(b"\xff\xd8fake").decode()
        name = self.desk.log_image(dict(image=jpeg, body=dict(ls_x=0.6)))
        folder = self.desk.log.image_folder()
        self.assertEqual((folder / name).read_bytes(), b"\xff\xd8fake")
        image = [e for e in self.events() if e["ev"] == "image"][0]
        self.assertEqual((image["file"], image["body"]), (f"{folder.name}/{name}", dict(ls_x=0.6)))

    def test_log_image_is_ignored_when_not_following_and_bad_image_is_rejected(self):
        with self.assertRaises(ValueError):
            self.desk.log_image(dict(image="nope"))
        self.desk.command(dict(op="stop"))
        self.assertIsNone(self.desk.log_image(dict(image="data:image/jpeg;base64,AA==")))

    def test_old_logs_take_their_image_folders_with_them(self):
        log = FollowLog(self.dir.name, keep=2)
        for _ in range(3):
            log.begin(mode="x")
            log.image_folder().mkdir()
            log.end("t")
        logs = {p.stem for p in Path(self.dir.name).glob("follow-*.jsonl")}
        folders = {p.name.removesuffix("-img") for p in Path(self.dir.name).glob("follow-*-img")}
        self.assertEqual(len(logs), 2)
        self.assertLessEqual(folders, logs)                  # 残っている画像フォルダは、残っているログのものだけ

    def test_wait_keeps_following_alive_without_a_hand_and_moves_nothing(self):
        before = self.desk.arm.read()
        for _ in range(5):                       # TIMEOUT(0.65 秒)を超える時間、手が映らないまま待つ
            self.time[0] += 0.3
            self.desk.command(dict(op="wait", token=self.token))
        self.assertEqual(self.desk.arm.read(), before)
        self.assertEqual(self.desk.token, self.token)           # 入力途絶で止まっていない
        self.sample(pan=0.2)                                    # 手が映ったら、そのまま追従に入れる
        self.assertEqual(self.desk.token, self.token)

    def test_wait_is_refused_when_not_following_or_with_a_wrong_token(self):
        with self.assertRaises(ValueError):
            self.desk.command(dict(op="wait", token="wrong"))
        self.desk.command(dict(op="stop"))
        with self.assertRaises(ValueError):
            self.desk.command(dict(op="wait", token=self.token))

    def test_appends_camera_video_chunks_next_to_the_log_and_notes_it_once(self):
        self.assertEqual(self.desk.log_video(b"head"), 4)
        self.assertEqual(self.desk.log_video(b"tail"), 8)
        path = self.desk.log.video_path()
        self.assertEqual((path.read_bytes(), path.stem), (b"headtail", self.desk.log.path.stem))
        self.assertEqual([e["file"] for e in self.events() if e["ev"] == "video"], [path.name])

    def test_last_video_chunk_arriving_after_stop_still_goes_to_that_log(self):
        self.desk.log_video(b"a")
        self.desk.command(dict(op="stop"))
        self.assertEqual(self.desk.log_video(b"b"), 2)
        self.assertEqual(self.desk.log.video_path().read_bytes(), b"ab")

    def test_video_is_capped_and_empty_chunks_are_rejected(self):
        import server
        self.desk.log_video(b"abc")
        with unittest.mock.patch.object(server, "LOG_VIDEO_MAX", 4):
            self.assertIsNone(self.desk.log_video(b"de"))
        self.assertEqual(self.desk.log.video_path().read_bytes(), b"abc")
        with self.assertRaises(ValueError):
            self.desk.log_video(b"")

    def test_old_logs_take_their_videos_with_them(self):
        log = FollowLog(self.dir.name, keep=2)
        for _ in range(3):
            log.begin(mode="x")
            log.video_path().write_bytes(b"v")
            log.end("t")
        logs = {p.stem for p in Path(self.dir.name).glob("follow-*.jsonl")}
        videos = {p.stem for p in Path(self.dir.name).glob("follow-*.webm")}
        self.assertEqual(len(logs), 2)
        self.assertLessEqual(videos, logs)

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


class LimitRangeTests(unittest.TestCase):
    """手の入力 -1〜1 は、関節の「動かせる下限〜上限」へ写る（範囲を絞っても手の全域が使える）。"""

    def reach(self, limits, **inputs):
        time = [10.0]
        desk = Desk(MockArm(), clock=lambda: time[0], limits=limits)
        token = desk.command(dict(op="start", gripper_only=True))["token"]
        desk.arm.pose["shoulder_lift"] = 0.0
        for _ in range(400):                       # 速度制限があるので、十分な回数を送って目標へ着かせる
            time[0] += 0.065
            desk.command(dict(op="sample", token=token, height=0.5, opening=0.5, **inputs))
        return desk.arm.read()["shoulder_lift"]

    def test_input_ends_map_to_limit_ends(self):
        limits = default_limits()
        limits["shoulder_lift"].update(min=30.0, max=70.0, speed=150.0, step=15.0)
        self.assertAlmostEqual(self.reach(limits, lift=1), 2*70-100, places=3)    # 上限 70%
        self.assertAlmostEqual(self.reach(limits, lift=-1), 2*30-100, places=3)   # 下限 30%
        self.assertAlmostEqual(self.reach(limits, lift=0), 0.0, places=3)         # 中央＝中間

    def test_default_limits_use_the_full_range(self):
        limits = default_limits()
        limits["shoulder_lift"].update(speed=150.0, step=15.0)
        self.assertAlmostEqual(self.reach(limits, lift=1), 100.0, places=3)
        self.assertAlmostEqual(self.reach(limits, lift=-1), -100.0, places=3)


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
