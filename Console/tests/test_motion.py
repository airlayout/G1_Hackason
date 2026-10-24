"""運動制御（段階 3）のテスト。実機も SDK も要らない（偽の時計・送信側・子プロセスで検証する）。

T3-1…T3-13 は docs/TEST_SPEC.md の番号に対応する。
"""
import ast
import json
import sys
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from g1console import motion  # noqa: E402
from g1console.motion import (DURATION_S, LIMITS, LIVENESS_S, HelperSender, MockSender, MotionController,  # noqa: E402
                              MotionError, UnavailableSender, parse_velocity)
from server import MockHelper, Monitor, fsm_source, make_handler  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class Fsm:
    """運動の可否判定に渡す FSM の偽。"""

    def __init__(self, fsm_id=501, age=0.1):
        self.fsm_id, self.age = fsm_id, age

    def __call__(self):
        return self.fsm_id, self.age


def make(fsm=None):
    clock, sender = Clock(), MockSender()
    fsm = fsm or Fsm()
    return MotionController(sender, fsm, clock), sender, clock, fsm


def claimed(ctl):
    return ctl.claim()["session"]


class ControllerTest(unittest.TestCase):
    def test_t3_1_ctl_is_sent_as_duration_limited_velocity(self):  # T3-1
        ctl, sender, clock, _ = make()
        sid = claimed(ctl)
        ctl.ctl(sid, 1, 0.2, -0.1, 0.3)
        ctl.tick()
        self.assertEqual(sender.calls, [("move", 0.2, -0.1, 0.3, DURATION_S)])
        self.assertLess(motion.SEND_PERIOD_S, DURATION_S)  # 送信の周期は有効時間より短い（途切れない）
        self.assertTrue(ctl.status()["moving"])

    def test_t3_2_velocity_is_clamped_and_bad_values_rejected(self):  # T3-2
        self.assertEqual(parse_velocity({"vx": 9, "vy": -9, "wz": 9}), (LIMITS["vx"][1], LIMITS["vy"][0], LIMITS["wz"][1]))
        self.assertEqual(parse_velocity({"vx": 0.005, "vy": 0, "wz": -0.005}), (0.0, 0.0, 0.0))  # 不感帯
        for bad in ({"vx": float("nan"), "vy": 0, "wz": 0}, {"vx": float("inf"), "vy": 0, "wz": 0},
                    {"vx": True, "vy": 0, "wz": 0}, {"vx": "0.1", "vy": 0, "wz": 0}, {"vx": None, "vy": 0, "wz": 0}, {"vy": 0, "wz": 0}):
            with self.assertRaises(ValueError, msg=str(bad)):
                parse_velocity(bad)
        ctl, sender, _, _ = make()
        sid = claimed(ctl)
        ctl.ctl(sid, 1, 5, 5, 5)  # コントローラ自身も丸める（HTTP を通らなくても上限を超えない）
        ctl.tick()
        self.assertEqual(sender.calls[0][1:4], (LIMITS["vx"][1], LIMITS["vy"][1], LIMITS["wz"][1]))

    def test_t3_3_silence_stops_and_does_not_resume_by_itself(self):  # T3-3
        ctl, sender, clock, _ = make()
        sid = claimed(ctl)
        ctl.ctl(sid, 1, 0.2, 0, 0)
        ctl.tick()
        clock.advance(LIVENESS_S + 0.01)
        ctl.tick()
        self.assertEqual(sender.calls[-1], ("stop",))
        self.assertFalse(ctl.status()["operator"])
        self.assertEqual(ctl.status()["last_event"]["kind"], "lost")
        n = len(sender.calls)
        with self.assertRaises(MotionError) as cm:  # 同じ session では再開できない
            ctl.ctl(sid, 2, 0.2, 0, 0)
        self.assertEqual(cm.exception.status, 409)
        ctl.tick()
        self.assertEqual(len(sender.calls), n)  # 何も送らない

    def test_t3_3b_liveness_uses_receive_time_not_connection(self):
        ctl, sender, clock, _ = make()
        sid = claimed(ctl)
        for seq in range(1, 8):  # 0.3 秒おきに保持している間は止まらない
            ctl.ctl(sid, seq, 0.2, 0, 0)
            clock.advance(0.3)
            ctl.tick()
        self.assertTrue(ctl.status()["operator"])
        self.assertNotIn(("stop",), sender.calls)

    def test_t3_4_estop_always_sends_stop_and_clears_operator(self):  # T3-4
        ctl, sender, _, _ = make()
        sid = claimed(ctl)  # 動いていなくても止める
        ctl.estop()
        self.assertEqual(sender.calls, [("stop",)])
        self.assertFalse(ctl.status()["operator"])
        self.assertEqual(ctl.status()["last_event"]["kind"], "estop")
        with self.assertRaises(MotionError):
            ctl.ctl(sid, 1, 0.2, 0, 0)
        ctl.estop()  # 操作権が無くても押せる
        self.assertEqual(sender.calls, [("stop",), ("stop",)])

    def test_t3_5_single_operator_and_takeover(self):  # T3-5
        ctl, sender, _, _ = make()
        first = claimed(ctl)
        with self.assertRaises(MotionError) as cm:
            ctl.claim()
        self.assertEqual(cm.exception.status, 409)
        ctl.ctl(first, 1, 0.2, 0, 0)
        ctl.tick()
        second = ctl.claim(takeover=True)["session"]
        self.assertNotEqual(first, second)
        self.assertEqual(sender.calls[-1], ("stop",))  # 奪ったとき、前の人の動きは止まる
        with self.assertRaises(MotionError):
            ctl.ctl(first, 2, 0.2, 0, 0)
        ctl.ctl(second, 1, 0.1, 0, 0)

    def test_t3_6_moves_only_in_walking_fsm_with_fresh_state(self):  # T3-6
        ctl, sender, _, fsm = make(Fsm(1))
        sid = claimed(ctl)
        ctl.ctl(sid, 1, 0.2, 0, 0)
        ctl.tick()
        self.assertEqual(sender.calls, [])
        self.assertIn("FSM", ctl.status()["blocked"])
        fsm.fsm_id = 500
        ctl.ctl(sid, 2, 0.2, 0, 0)
        ctl.tick()
        self.assertEqual(sender.calls[-1][0], "move")
        fsm.age = 6.0  # 状態確認が古い（監視停止・Jetson 断）
        ctl.tick()
        self.assertEqual(sender.calls[-1], ("stop",))
        self.assertIn("古い", ctl.status()["blocked"])
        n = len(sender.calls)
        ctl.tick()
        self.assertEqual(len(sender.calls), n)  # 止めたあとは繰り返さない
        for bad in (None, 0, 3, 702):
            fsm.fsm_id, fsm.age = bad, 0.1
            self.assertIsNotNone(ctl.status()["blocked"], bad)

    def test_t3_7_disable_reason_blocks_claim_and_drops_operator(self):  # T3-7
        ctl, sender, _, _ = make()
        sid = claimed(ctl)
        ctl.ctl(sid, 1, 0.2, 0, 0)
        ctl.tick()
        ctl.set_disable_reason("Nav2 稼働中")
        self.assertFalse(ctl.status()["operator"])
        self.assertEqual(sender.calls[-1], ("stop",))
        with self.assertRaises(MotionError) as cm:
            ctl.claim()
        self.assertIn("Nav2", cm.exception.message)
        ctl.set_disable_reason(None)
        claimed(ctl)

    def test_t3_8_zero_target_sends_one_stop_not_a_stream(self):  # T3-8
        ctl, sender, _, _ = make()
        sid = claimed(ctl)
        ctl.tick()
        self.assertEqual(sender.calls, [])  # 動かしていない間は何も送らない（物理リモコンと競合させない）
        ctl.ctl(sid, 1, 0.2, 0, 0)
        ctl.tick()
        ctl.ctl(sid, 2, 0, 0, 0)
        for _ in range(5):
            ctl.tick()
        self.assertEqual([c[0] for c in sender.calls], ["move", "stop"])

    def test_t3_9_out_of_order_ctl_is_ignored_and_not_counted_as_alive(self):  # T3-9
        ctl, sender, clock, _ = make()
        sid = claimed(ctl)
        ctl.ctl(sid, 5, 0.2, 0, 0)
        clock.advance(0.4)
        ctl.ctl(sid, 5, -0.2, 0, 0)  # 同じ seq（重複・遅延）
        ctl.ctl(sid, 3, -0.2, 0, 0)
        clock.advance(0.2)  # 最初の受信から 0.6 秒。古い指令は生存の根拠にならない
        ctl.tick()
        self.assertFalse(ctl.status()["operator"])
        with self.assertRaises(ValueError):
            c2, _, _, _ = make()
            c2.ctl(claimed(c2), "1", 0, 0, 0)

    def test_t3_10_repeated_send_failure_drops_operator(self):  # T3-10
        ctl, sender, _, _ = make()
        sid = claimed(ctl)
        ctl.ctl(sid, 1, 0.2, 0, 0)
        sender.fail = True
        for _ in range(motion.SEND_FAIL_LIMIT):
            ctl.tick()
        st = ctl.status()
        self.assertFalse(st["operator"])
        self.assertEqual(st["last_event"]["kind"], "sender_error")
        self.assertTrue(st["sender"]["error"])

    def test_t3_11_stale_decision_is_not_sent_after_a_stop(self):  # T3-11
        ctl, sender, _, _ = make()
        sid = claimed(ctl)
        ctl.ctl(sid, 1, 0.2, 0, 0)
        with ctl._lock:
            gen = ctl._gen
        ctl.estop()  # 判断のあとに緊急停止が入った
        self.assertFalse(ctl._emit(gen, ("move", (0.2, 0.0, 0.0))))
        self.assertEqual(sender.calls, [("stop",)])

    def test_t3_12_actions_are_allowlisted_and_need_operator_and_walking_fsm(self):  # T3-12
        ctl, sender, _, fsm = make()
        with self.assertRaises(ValueError):
            ctl.action("x", "SetFsmId")
        with self.assertRaises(MotionError) as cm:
            ctl.action("nobody", "wave_above_head")
        self.assertEqual(cm.exception.status, 409)
        sid = claimed(ctl)
        ctl.ctl(sid, 1, 0.2, 0, 0)
        ctl.tick()
        ctl.action(sid, "wave_above_head")
        self.assertEqual(sender.calls[-1], ("arm", 26))
        ctl.tick()
        self.assertEqual(sender.calls[-1], ("stop",))  # アクション中は歩行指令を止める
        fsm.fsm_id = 1
        with self.assertRaises(MotionError):
            ctl.action(sid, "wave_above_head")
        self.assertEqual({c[1] for c in sender.calls if c[0] == "arm"}, {26})

    def test_unavailable_sender_refuses_claim(self):
        ctl = MotionController(UnavailableSender(), Fsm())
        with self.assertRaises(MotionError) as cm:
            ctl.claim()
        self.assertEqual(cm.exception.status, 503)

    def test_shutdown_stops(self):
        ctl, sender, _, _ = make()
        claimed(ctl)
        ctl.shutdown()
        self.assertEqual(sender.calls, [("stop",)])


class HelperTest(unittest.TestCase):
    def test_motion_helper_source_is_valid_and_limited(self):
        src = (Path(__file__).resolve().parent.parent / "jetson" / "motion_helper.py").read_text(encoding="utf-8")
        ast.parse(src.replace("__LIMITS__", "{}"))
        for forbidden in ("SetFsmId", "SetStandHeight", "Damp", "ZeroTorque", "lowcmd", "LowCmd", "arm_sdk"):
            self.assertNotIn(forbidden, src)
        self.assertEqual(sum(src.count(op) for op in ('op == "move"', 'op == "stop"')), 2)
        self.assertNotIn("SetTaskId", src)
        self.assertIn("MAX_DURATION_S", src)

    def test_helper_sender_substitutes_limits(self):
        src = HelperSender()._source()
        self.assertNotIn("__LIMITS__", src)
        self.assertNotIn("__TASKS__", src)

    def test_arm_accepts_real_banner_output_and_rejects_error_text(self):
        import os, stat, tempfile
        ok = " --- Unitree Robotics --- \n     G1 Arm Action Example      \n\n"
        for out, rc, good in ((ok, 0, True), (ok + "INVALID_FSM_ID\n", 0, False), (ok, 1, False)):
            with tempfile.TemporaryDirectory() as d:
                exe = Path(d) / "arm"
                exe.write_text("#!/bin/sh\nprintf '%s' '" + out + "'\nexit " + str(rc) + "\n")
                exe.chmod(exe.stat().st_mode | stat.S_IXUSR)
                os.environ["G1_ARM_ACTION_BIN"] = str(exe)
                try:
                    if good:
                        HelperSender().arm(99)
                    else:
                        with self.assertRaises(MotionError):
                            HelperSender().arm(99)
                finally:
                    del os.environ["G1_ARM_ACTION_BIN"]

    def test_t3_14_helper_sender_with_fake_child(self):  # T3-14
        fake = ("import sys, json\nprint(json.dumps({'ready': True}), flush=True)\n"
                "for line in sys.stdin:\n    r = json.loads(line)\n    print(json.dumps({'ok': True, 'code': 0, 'echo': r}), flush=True)\n")
        sender = HelperSender()
        sender._command = lambda src: ([sys.executable, "-u", "-c", fake], None)
        sender.stop()  # 動いていないヘルパーを止めるために起動しない
        self.assertIsNone(sender._proc)
        sender.prepare()
        sender.move(0.1, 0.0, 0.0, 0.2)
        sender.stop()
        sender.stop()
        sender.close()
        self.assertIsNone(sender._proc)

    def test_t3_14b_silent_child_times_out_and_is_discarded(self):
        fake = "import sys, json, time\nprint(json.dumps({'ready': True}), flush=True)\nfor line in sys.stdin:\n    time.sleep(30)\n"
        sender = HelperSender()
        sender._command = lambda src: ([sys.executable, "-u", "-c", fake], None)
        old = motion.HELPER_TIMEOUT_S
        motion.HELPER_TIMEOUT_S = 0.3
        try:
            sender.prepare()
            with self.assertRaises(MotionError) as cm:
                sender.move(0.1, 0.0, 0.0, 0.2)
            self.assertEqual(cm.exception.status, 502)
            self.assertIsNone(sender._proc)  # 遅れて届く古い応答と取り違えないよう捨てる
        finally:
            motion.HELPER_TIMEOUT_S = old
            sender.close()

    def test_t3_14c_rejected_command_is_an_error(self):
        fake = ("import sys, json\nprint(json.dumps({'ready': True}), flush=True)\n"
                "for line in sys.stdin:\n    print(json.dumps({'ok': True, 'code': 3104}), flush=True)\n")
        sender = HelperSender()
        sender._command = lambda src: ([sys.executable, "-u", "-c", fake], None)
        sender.prepare()
        try:
            with self.assertRaises(MotionError):
                sender.move(0.1, 0.0, 0.0, 0.2)
        finally:
            sender.close()


class HttpMotionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.helper = MockHelper()
        cls.monitor = Monitor(cls.helper)
        cls.sender = MockSender()
        cls.fsm = Fsm()
        cls.motion = MotionController(cls.sender, cls.fsm)
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(cls.helper, cls.monitor, motion=cls.motion))
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def setUp(self):
        self.motion.estop()
        self.sender.calls.clear()

    def _call(self, method, path, body=None):
        data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
        req = urllib.request.Request("http://127.0.0.1:%d%s" % (self.port, path), data=data, method=method)
        try:
            r = urllib.request.urlopen(req, timeout=5)
            return r.status, json.load(r)
        except urllib.error.HTTPError as e:
            return e.code, json.load(e)

    def test_t3_13_claim_ctl_status_estop_over_http(self):  # T3-13
        code, j = self._call("POST", "/api/motion/claim", {})
        self.assertEqual(code, 200)
        sid = j["session"]
        self.assertEqual(self._call("POST", "/api/motion/claim", {})[0], 409)  # 2 人目
        code, j = self._call("POST", "/api/motion/ctl", {"session": sid, "seq": 1, "vx": 9, "vy": 0, "wz": 0})
        self.assertEqual((code, j["target"]["vx"]), (200, LIMITS["vx"][1]))  # サーバーで丸める
        self.motion.tick()
        self.assertEqual(self.sender.calls[-1][0], "move")
        self.assertEqual(self._call("GET", "/api/motion/status")[1]["state"], "active")
        code, j = self._call("POST", "/api/motion/estop")  # 本文なしでも止まる
        self.assertEqual((code, j["operator"]), (200, False))
        self.assertEqual(self.sender.calls[-1], ("stop",))
        self.assertEqual(self._call("POST", "/api/motion/ctl", {"session": sid, "seq": 2, "vx": 0, "vy": 0, "wz": 0})[0], 409)

    def test_bad_requests_are_400_and_wrong_session_is_409(self):
        self.assertEqual(self._call("POST", "/api/motion/claim", b"not json")[0], 400)
        sid = self._call("POST", "/api/motion/claim", {})[1]["session"]
        self.assertEqual(self._call("POST", "/api/motion/ctl", b"not json")[0], 400)
        self.assertEqual(self._call("POST", "/api/motion/ctl", {"session": sid, "seq": 1, "vx": "fast", "vy": 0, "wz": 0})[0], 400)
        self.assertEqual(self._call("POST", "/api/motion/ctl", {"session": "other", "seq": 1, "vx": 0, "vy": 0, "wz": 0})[0], 409)
        self.assertEqual(self._call("POST", "/api/motion/action", {"session": sid, "name": "SetFsmId"})[0], 400)
        self.assertEqual(self._call("POST", "/api/motion/release", {"session": sid})[0], 200)
        self.assertEqual(self._call("POST", "/api/motion/nope", {})[0], 404)

    def test_blocked_reason_shows_in_status_when_not_walking(self):
        self.fsm.fsm_id = 3
        try:
            self.assertIn("FSM", self._call("GET", "/api/motion/status")[1]["blocked"])
        finally:
            self.fsm.fsm_id = 501

    def test_fsm_source_reads_the_monitor(self):
        monitor = Monitor(MockHelper())
        self.assertEqual(fsm_source(monitor)(), (None, None))  # まだ確認していない
        monitor.poll_once()
        fsm_id, age = fsm_source(monitor)()
        self.assertEqual(fsm_id, 1)
        self.assertLess(age, 2)


if __name__ == "__main__":
    unittest.main()
