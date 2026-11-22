import json
import subprocess
import sys
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from server import GRIPPER_CLOSED, GRIPPER_OPEN, Desk, MockArm


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.time = 10.0
        self.arm = MockArm()
        self.desk = Desk(self.arm, clock=lambda: self.time)

    def teach(self):
        self.desk.command(dict(op="mark", label="low"))
        self.arm.pose.update(shoulder_lift=16)
        self.desk.command(dict(op="mark", label="high"))

    def test_requires_teaching(self):
        with self.assertRaises(ValueError):
            self.desk.command({"op":"start"})

    def test_lift_and_grip_from_webcam_values(self):
        self.teach()
        token = self.desk.command({"op":"start"})["token"]
        for height, opening in [(0, 1), (0, 0), (1, 0)]:
            for _ in range(35):
                self.time += .08
                previous = self.arm.read()
                self.desk.command(dict(op="sample", token=token, height=height, opening=opening))
                self.assertLessEqual(abs(self.arm.pose["shoulder_lift"]-previous["shoulder_lift"]), 1.77)
            self.assertEqual(self.arm.pose["shoulder_lift"], 16*height)
            self.assertEqual(self.arm.pose["gripper"], GRIPPER_CLOSED + (GRIPPER_OPEN-GRIPPER_CLOSED)*opening)

    def test_stale_packets_and_timeout(self):
        self.teach()
        token = self.desk.command({"op":"start"})["token"]
        self.time += .8
        self.desk.expired()
        new_token = self.desk.command({"op":"start"})["token"]
        for bad in (token, None):
            with self.assertRaises(ValueError):
                self.desk.command(dict(op="sample", token=bad, height=1, opening=0))
        self.assertNotEqual(new_token, token)
        self.desk.command({"op":"stop"})
        with self.assertRaises(ValueError):
            self.desk.command(dict(op="sample", token=new_token, height=1, opening=0))

    def test_invalid_sample_and_teaching_while_active(self):
        self.teach()
        token = self.desk.command({"op":"start"})["token"]
        before = self.arm.read()
        for data in [dict(op="sample", token=token, height=float("nan"), opening=1),
                     dict(op="sample", token=token, height=1.5, opening=1),
                     dict(op="move", joint="shoulder_pan", value=10), dict(op="mark", label="low")]:
            with self.assertRaises(ValueError):
                self.desk.command(data)
        self.assertEqual(before, self.arm.read())

    def test_start_off_path(self):
        self.teach()
        self.arm.pose["wrist_roll"] = 20
        with self.assertRaises(ValueError):
            self.desk.command({"op":"start"})
        self.assertIsNone(self.desk.token)

    def test_gripper_needs_no_marks(self):
        self.assertEqual(self.desk.state()["gripper"], dict(closed=GRIPPER_CLOSED, open=GRIPPER_OPEN))
        with self.assertRaises(ValueError) as caught:
            self.desk.command({"op":"start"})
        self.assertNotIn("幅", str(caught.exception))


class GripperOnlyTests(unittest.TestCase):
    def test_starts_without_marks_and_moves_only_gripper(self):
        from server import JOINTS
        time = [10.0]
        desk = Desk(MockArm(), clock=lambda: time[0])
        result = desk.command(dict(op="start", gripper_only=True))
        self.assertEqual(desk.marks, {})
        before = desk.pose.copy()
        time[0] += 0.065
        desk.command(dict(op="sample", token=result["token"], height=1, opening=0.0))
        after = desk.arm.read()
        self.assertNotEqual(after["gripper"], before["gripper"])
        for joint in JOINTS[:-1]:
            self.assertEqual(after[joint], before[joint])


class FingerCalibrationTests(unittest.TestCase):
    def test_save_merges_and_reloads(self):
        import tempfile
        from pathlib import Path
        from server import load_finger, save_finger
        with tempfile.TemporaryDirectory() as folder:
            file = Path(folder) / "finger.json"
            self.assertEqual(load_finger(file), {})
            save_finger(dict(closed=0.15), file)
            self.assertEqual(save_finger(dict(open=1.2), file), dict(closed=0.15, open=1.2))
            self.assertEqual(load_finger(file), dict(closed=0.15, open=1.2))

    def test_rejects_bad_values_and_ignores_broken_file(self):
        import tempfile
        from pathlib import Path
        from server import load_finger, save_finger
        with tempfile.TemporaryDirectory() as folder:
            file = Path(folder) / "finger.json"
            for bad in (float("nan"), -1, 99, "x"):
                with self.assertRaises(ValueError):
                    save_finger(dict(open=bad), file)
            file.write_text("{broken")
            self.assertEqual(load_finger(file), {})


class SnapshotTests(unittest.TestCase):
    def test_saves_image_and_numbers(self):
        import base64
        import tempfile
        from pathlib import Path
        from server import save_snapshot
        jpeg = b"\xff\xd8\xff\xd9"
        data = dict(image="data:image/jpeg;base64," + base64.b64encode(jpeg).decode(), info=dict(ratio=0.5))
        with tempfile.TemporaryDirectory() as folder:
            name = save_snapshot(data, Path(folder) / "debug", now=lambda: 1000.0)
            self.assertEqual((Path(folder) / "debug" / (name + ".jpg")).read_bytes(), jpeg)
            self.assertEqual(json.loads((Path(folder) / "debug" / (name + ".json")).read_text())["ratio"], 0.5)

    def test_rejects_bad_input(self):
        import tempfile
        from pathlib import Path
        from server import save_snapshot
        with tempfile.TemporaryDirectory() as folder:
            for bad in [dict(), dict(image="x", info={}), dict(image="data:image/jpeg;base64,!!!", info={}),
                        dict(image="data:image/jpeg;base64,AAAA", info=[])]:
                with self.assertRaises(ValueError):
                    save_snapshot(bad, Path(folder))


class MarkFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.arm = MockArm()
        self.desk = Desk(self.arm, clock=lambda: 10.0)

    def test_state_has_values_and_history(self):
        self.desk.command(dict(op="mark", label="low"))
        self.arm.pose.update(shoulder_lift=6)
        state = self.desk.command(dict(op="mark", label="high"))
        self.assertEqual(state["marks"]["high"]["shoulder_lift"], 6)
        self.assertEqual([h["label"] for h in state["history"]], ["low", "high"])
        self.assertEqual(state["history"][1]["pose"]["shoulder_lift"], 6)
        state["marks"]["low"]["shoulder_lift"] = 99      # 返した値を書き換えても内部は変わらない
        self.assertEqual(self.desk.marks["low"]["shoulder_lift"], 0)

    def test_start_lists_missing_marks(self):
        self.desk.command(dict(op="mark", label="low"))
        with self.assertRaises(ValueError) as caught:
            self.desk.command({"op": "start"})
        self.assertIn("持ち上げた位置", str(caught.exception))
        self.assertNotIn("つかむ位置", str(caught.exception))

    def test_move_goes_toward_target_in_small_steps(self):
        steps = []
        for _ in range(40):
            before = self.arm.read()["shoulder_lift"]
            after = self.desk.command(dict(op="move", joint="shoulder_lift", value=20))["pose"]["shoulder_lift"]
            steps.append(abs(after - before))
        self.assertLessEqual(max(steps), 1.0)
        self.assertEqual(self.arm.read()["shoulder_lift"], 20)

    def test_move_goal_runs_ahead_but_stays_within_lag_when_joint_does_not_follow(self):
        class StuckArm(MockArm):
            def __init__(self):
                super().__init__()
                self.goals = []

            def write(self, pose):
                self.goals.append(pose["elbow_flex"])      # 摩擦などで動かない関節（位置は変わらない）

        time, arm = [10.0], StuckArm()
        desk = Desk(arm, clock=lambda: time[0])
        for _ in range(6):
            desk.command(dict(op="move", joint="elbow_flex", value=60))
        self.assertEqual(arm.goals, [1.0, 2.0, 3.0, 3.0, 3.0, 3.0])   # 先へ進み、実際の位置+MOVE_LAG で頭打ち
        time[0] += 1.0                                                # 指令が途切れたら実際の位置から取り直す
        desk.command(dict(op="move", joint="elbow_flex", value=60))
        self.assertEqual(arm.goals[-1], 1.0)

    def test_move_holds_current_position_when_target_is_the_current_position(self):
        self.desk.command(dict(op="move", joint="elbow_flex", value=10))
        actual = self.arm.read()["elbow_flex"]
        after = self.desk.command(dict(op="move", joint="elbow_flex", value=actual))["pose"]["elbow_flex"]
        self.assertEqual(after, actual)
        self.assertNotIn("elbow_flex", self.desk.cmd)

    def test_move_rejects_out_of_range(self):
        for data in [dict(op="move", joint="gripper", value=-5), dict(op="move", joint="elbow_flex", value=101),
                     dict(op="move", joint="nope", value=0), dict(op="move", joint="elbow_flex", value=float("nan"))]:
            with self.assertRaises(ValueError):
                self.desk.command(data)

    def test_percent_scale(self):
        from server import pct
        self.assertEqual([pct("elbow_flex", v) for v in (-100, 0, 100)], [0, 50, 100])
        self.assertEqual([pct("gripper", v) for v in (0, 55, 100)], [0, 55, 100])

    def test_history_is_capped(self):
        for _ in range(60):
            self.desk.command(dict(op="mark", label="low"))
        self.assertEqual(len(self.desk.state()["history"]), 50)


class CalibrationTests(unittest.TestCase):
    def test_mock_needs_no_calibration(self):
        calibration = Desk(MockArm()).state()["calibration"]
        self.assertFalse(calibration["required"])
        self.assertTrue(calibration["done"])

    def test_uncalibrated_real_arm_refuses_until_checked(self):
        results = [ValueError("校正データがありません")]
        def retry():
            if results:
                raise results.pop()
            return MockArm()
        desk = Desk(None, True, retry=retry, why="校正データがありません", command="lerobot-calibrate ...")
        calibration = desk.state()["calibration"]
        self.assertTrue(calibration["required"])
        self.assertFalse(calibration["done"])
        self.assertIn("lerobot-calibrate", calibration["command"])
        for data in [dict(op="mark", label="low"), dict(op="start"), dict(op="move", joint="elbow_flex", value=0)]:
            with self.assertRaises(ValueError) as caught:
                desk.command(data)
            self.assertIn("校正", str(caught.exception))
        self.assertFalse(desk.command({"op": "stop"})["calibration"]["done"])
        with self.assertRaises(ValueError):                    # 1回目の確認はまだ未校正
            desk.command({"op": "calibration_check"})
        self.assertFalse(desk.state()["calibration"]["done"])
        self.assertTrue(desk.command({"op": "calibration_check"})["calibration"]["done"])   # 2回目で校正済み
        self.assertIn("low", desk.command(dict(op="mark", label="low"))["marks"])


class DriveTests(unittest.TestCase):
    def setUp(self):
        self.arm = MockArm()
        self.desk = Desk(self.arm, clock=lambda: 10.0)

    def test_state_has_drive_and_switches(self):
        self.assertEqual(self.desk.state()["drive"], "on")
        for mode in ("off", "soft", "on"):
            self.assertEqual(self.desk.command(dict(op="drive", mode=mode))["drive"], mode)
            self.assertEqual(self.arm.drive, mode)

    def test_soft_level_is_kept_and_validated(self):
        self.assertEqual(self.desk.command(dict(op="drive", mode="soft", level=60))["soft_level"], 60)
        self.assertEqual(self.desk.command(dict(op="drive", mode="on"))["soft_level"], 60)   # level 省略時は前回の値
        for bad in (-1, 101, "x", float("nan")):
            with self.assertRaises(ValueError):
                self.desk.command(dict(op="drive", mode="soft", level=bad))

    def test_invalid_drive_mode(self):
        for data in [dict(op="drive", mode="loose"), dict(op="drive")]:
            with self.assertRaises((ValueError, KeyError)):
                self.desk.command(data)
        self.assertEqual(self.desk.state()["drive"], "on")

    def test_move_and_start_refused_unless_on_but_marking_allowed(self):
        for mode in ("off", "soft"):
            self.desk.command(dict(op="drive", mode=mode))
            for data in [dict(op="move", joint="elbow_flex", value=5), dict(op="start")]:
                with self.assertRaises(ValueError) as caught:
                    self.desk.command(data)
                self.assertIn("ON", str(caught.exception))
            self.assertIn("low", self.desk.command(dict(op="mark", label="low"))["marks"])   # 手で姿勢を作って登録できる
        self.desk.command(dict(op="drive", mode="on"))
        self.desk.command(dict(op="move", joint="elbow_flex", value=5))

    def test_switching_stops_following(self):
        time = [10.0]
        arm = MockArm()
        desk = Desk(arm, clock=lambda: time[0])
        desk.command(dict(op="mark", label="low"))
        arm.pose.update(shoulder_lift=16)
        desk.command(dict(op="mark", label="high"))
        token = desk.command({"op": "start"})["token"]
        state = desk.command(dict(op="drive", mode="soft"))
        self.assertFalse(state["active"])
        with self.assertRaises(ValueError):
            desk.command(dict(op="sample", token=token, height=1, opening=0))

    def test_refresh_follows_hand_moved_arm_only_when_not_on(self):
        self.arm.pose["elbow_flex"] = 30                     # 手で動かした
        self.desk.refresh()
        self.assertEqual(self.desk.pose["elbow_flex"], 0)    # ON のときは、画面の値を勝手に読み替えない
        self.desk.command(dict(op="drive", mode="off"))
        self.arm.pose["elbow_flex"] = 42
        self.desk.refresh()
        self.assertEqual(self.desk.state()["pose"]["elbow_flex"], 42)


class LimitsTests(unittest.TestCase):
    def setUp(self):
        import tempfile
        from pathlib import Path
        self.folder = tempfile.TemporaryDirectory()
        self.file = Path(self.folder.name) / "limits.json"
        self.time = 10.0
        self.arm = MockArm()
        self.desk = Desk(self.arm, clock=lambda: self.time, limits_file=self.file)

    def tearDown(self):
        self.folder.cleanup()

    def test_defaults_match_the_old_fixed_values(self):
        from limits import default_limits, speed_raw, step_raw
        limits = default_limits()
        self.assertEqual(speed_raw("elbow_flex", limits), 22)
        self.assertEqual(speed_raw("gripper", limits), 45)
        self.assertEqual(step_raw("shoulder_lift", limits), 3.0)
        self.assertEqual(step_raw("gripper", limits), 3.0)

    def test_rejects_bad_input(self):
        from limits import default_limits, parse_limits
        base = default_limits()
        for bad in ({"nothing": {"speed": 5}}, {"gripper": {"color": 5}}, {"gripper": {"speed": 0}},
                    {"gripper": {"speed": float("nan")}}, {"gripper": {"step": 99}}, {"gripper": {"min": "0"}},
                    {"gripper": {"min": 98}}, {"gripper": {"max": 3}}, [1]):
            with self.assertRaises(ValueError, msg=bad):
                parse_limits(bad, base)
        self.assertEqual(base, default_limits())     # 元は変えない

    def test_save_and_auto_load(self):
        from limits import default_limits, load_limits
        self.assertEqual(load_limits(self.file), default_limits())      # ファイルが無ければ既定値
        self.desk.command(dict(op="limits", limits={"gripper": {"speed": 90, "max": 80}}))
        loaded = load_limits(self.file)
        self.assertEqual(loaded["gripper"]["speed"], 90.0)
        self.assertEqual(loaded["gripper"]["max"], 80.0)
        self.assertEqual(loaded["elbow_flex"], default_limits()["elbow_flex"])
        restarted = Desk(MockArm(), limits=loaded, limits_file=self.file)
        self.assertEqual(restarted.arm.limits["gripper"]["speed"], 90.0)

    def test_broken_file_falls_back_to_defaults(self):
        from limits import default_limits, load_limits
        self.file.write_text("{broken")
        self.assertEqual(load_limits(self.file), default_limits())
        self.file.write_text(json.dumps({"gripper": {"speed": 9999}}))
        self.assertEqual(load_limits(self.file), default_limits())

    def test_reset_restores_defaults_and_saves(self):
        from limits import default_limits, load_limits
        self.desk.command(dict(op="limits", limits={"gripper": {"speed": 90}}))
        state = self.desk.command(dict(op="limits", reset=True))
        self.assertEqual(state["limits"], default_limits())
        self.assertEqual(load_limits(self.file), default_limits())

    def test_cannot_change_while_following(self):
        token = self.desk.command(dict(op="start", gripper_only=True))["token"]
        with self.assertRaises(ValueError):
            self.desk.command(dict(op="limits", limits={"gripper": {"speed": 90}}))
        self.assertEqual(self.desk.limits["gripper"]["speed"], 45.0)
        self.assertTrue(token)

    def test_save_failure_changes_nothing(self):
        from pathlib import Path
        desk = Desk(MockArm(), limits_file=Path(self.folder.name) / "missing" / "limits.json")
        with self.assertRaises(ValueError):
            desk.command(dict(op="limits", limits={"gripper": {"speed": 90}}))
        self.assertEqual(desk.limits["gripper"]["speed"], 45.0)

    def test_follow_speed_is_per_joint(self):
        self.desk.command(dict(op="limits", limits={"gripper": {"speed": 20}}))
        token = self.desk.command(dict(op="start", gripper_only=True))["token"]
        before = self.arm.read()["gripper"]
        self.time += .1
        self.desk.command(dict(op="sample", token=token, height=0, opening=1))
        self.assertAlmostEqual(self.arm.read()["gripper"] - before, 20 * .1)

    def test_range_limit_clips_every_command(self):
        self.desk.command(dict(op="limits", limits={"gripper": {"min": 20, "max": 70}, "elbow_flex": {"max": 60}}))
        self.arm.write(dict(gripper=0, elbow_flex=100))
        self.assertEqual(self.arm.read()["gripper"], 20)
        self.assertEqual(self.arm.read()["elbow_flex"], 20)     # 60% → 内部の値 20
        self.arm.write(dict(gripper=100))
        self.assertEqual(self.arm.read()["gripper"], 70)

    def test_slider_move_respects_step_limit(self):
        self.desk.command(dict(op="limits", limits={"shoulder_lift": {"step": 0.25}}))   # 内部の 0.5
        for _ in range(10):
            self.time += .03
            self.desk.command(dict(op="move", joint="shoulder_lift", value=50))
        self.assertLessEqual(self.arm.read()["shoulder_lift"], 10 * .5)
        self.assertGreater(self.arm.read()["shoulder_lift"], 0)

    def test_real_arm_passes_per_joint_step_for_written_joints_only(self):
        from limits import parse_limits
        arm, _ = RealArmDriveTests.make(RealArmDriveTests())
        arm.set_limits(parse_limits({"gripper": {"step": 6, "max": 90}}, arm.limits))
        arm.write(dict(gripper=100))
        self.assertEqual(arm.robot.config.max_relative_target, {"gripper": 6.0})
        arm.write(dict(gripper=10, shoulder_lift=0))
        self.assertEqual(arm.robot.config.max_relative_target, {"gripper": 6.0, "shoulder_lift": 3.0})


class FakeBus:
    def __init__(self):
        self.motors = {"shoulder_lift": 2, "gripper": 6}
        self.calls = []

    def write(self, name, motor, value):
        self.calls.append(("write", name, motor, value))

    def enable_torque(self):
        self.calls.append(("enable",))

    def disable_torque(self):
        self.calls.append(("disable",))


class RealArmDriveTests(unittest.TestCase):
    def make(self):
        from types import SimpleNamespace
        from server import RealArm, JOINTS
        bus, self.sent, self.positions = FakeBus(), [], {j: 5.0 for j in JOINTS}
        from limits import default_limits
        arm = RealArm.__new__(RealArm)
        arm.limits = default_limits()
        arm.robot = SimpleNamespace(
            config=SimpleNamespace(max_relative_target=3.0), bus=bus, send_action=lambda action: (bus.calls.append(("goal",)), self.sent.append(action)),
            get_observation=lambda: {j + ".pos": v for j, v in self.positions.items()})
        arm.normal = {"shoulder_lift": (1000, 32), "gripper": (500, 32)}
        arm.drive, arm.soft_goal, arm.deadband = "on", None, 1.0
        return arm, bus

    def test_off_only_releases_torque(self):
        arm, bus = self.make()
        arm.set_drive("off")
        self.assertEqual(bus.calls, [("disable",)])
        self.assertEqual(arm.drive, "off")

    def test_goal_is_set_to_current_position_before_torque_comes_back(self):
        arm, bus = self.make()
        arm.set_drive("off")
        bus.calls.clear()
        arm.set_drive("on")
        self.assertEqual(bus.calls[0], ("goal",))
        self.assertEqual(bus.calls[-1], ("enable",))

    def test_soft_lowers_torque_and_damping_and_on_restores_each_motor(self):
        from server import SOFT_TORQUE
        arm, bus = self.make()
        arm.set_drive("soft")
        self.assertIn(("write", "Torque_Limit", "shoulder_lift", SOFT_TORQUE), bus.calls)
        self.assertIn(("write", "Torque_Limit", "gripper", min(500, SOFT_TORQUE)), bus.calls)
        self.assertIn(("write", "D_Coefficient", "shoulder_lift", 0), bus.calls)        # 押し返す速度抵抗を無くす
        self.assertFalse([c for c in bus.calls if c[:2] == ("write", "P_Coefficient")])   # P は触らない
        bus.calls.clear()
        arm.set_drive("on")
        self.assertIn(("write", "Torque_Limit", "shoulder_lift", 1000), bus.calls)      # 元の上限・D に戻る
        self.assertIn(("write", "Torque_Limit", "gripper", 500), bus.calls)
        self.assertIn(("write", "D_Coefficient", "shoulder_lift", 32), bus.calls)

    def test_soft_deadband_maps_level_to_range_and_clamps(self):
        from server import soft_deadband, SOFT_DEADBAND
        self.assertEqual(soft_deadband(0), SOFT_DEADBAND[0])
        self.assertEqual(soft_deadband(100), SOFT_DEADBAND[1])
        self.assertEqual(soft_deadband(-5), SOFT_DEADBAND[0])
        self.assertEqual(soft_deadband(500), SOFT_DEADBAND[1])

    def test_soft_holds_posture_without_external_force_and_follows_when_pushed(self):
        arm, bus = self.make()
        arm.set_drive("soft", 0)                                    # 不感帯 0.5
        self.sent.clear()
        self.positions["shoulder_lift"] = 5.3                       # 不感帯の内側のずれ（重力や揺れ）→ 目標は動かさない
        arm.soft_track()
        self.assertEqual(self.sent, [])
        self.positions["shoulder_lift"] = 8.0                       # 押された → 目標を今の位置へ移し、押し返さない
        arm.soft_track()
        self.assertEqual(self.sent, [{"shoulder_lift.pos": 8.0}])   # 動いた関節だけ書く
        arm.soft_track()
        self.assertEqual(len(self.sent), 1)                         # そのまま保持（同じ値を書き続けない）

    def test_soft_track_does_nothing_unless_soft(self):
        arm, bus = self.make()
        for mode in ("on", "off"):
            arm.set_drive(mode)
            self.sent.clear()
            self.positions["elbow_flex"] = 50.0
            arm.soft_track()
            self.assertEqual(self.sent, [])


class LocalServerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from pathlib import Path
        cls.proc = subprocess.Popen([sys.executable,"-u",str(Path(__file__).with_name("server.py")),
                                     "--http-port","0"], stdout=subprocess.PIPE, text=True)
        cls.url = cls.proc.stdout.readline().split()[0]

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(timeout=5)
        cls.proc.stdout.close()

    def test_serves_desk_and_mock(self):
        self.assertIn("SO-101 撮影卓", urlopen(self.url).read().decode())
        self.assertFalse(json.load(urlopen(self.url+"/state"))["live"])
        self.assertIn("javascript", urlopen(self.url+"/app.js").headers["Content-Type"])

    def test_commands_local_only(self):
        for origin, expected in [(self.url, 200), ("https://example.org", 403)]:
            req = Request(self.url+"/command",data=b'{"op":"stop"}',
                          headers={"Origin":origin,"Content-Type":"application/json"})
            try:
                status = urlopen(req).status
            except HTTPError as exc:
                status = exc.code
                exc.close()
            self.assertEqual(status, expected)


if __name__ == "__main__":
    unittest.main()
