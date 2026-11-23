"""Nav2 表示・排他・heartbeat・ロケーション（段階 5、R-16〜R-18）。ロボットにも ROS にも触れない。
T5-1…T5-12 は docs/TEST_SPEC.md の番号に対応する。"""
import json
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from g1console import heartbeat as hb_mod  # noqa: E402
from g1console.heartbeat import OperatorHeartbeat, parse_target  # noqa: E402
from g1console.locations import LocationError, LocationStore  # noqa: E402
from g1console.nav import (REASON_RUNNING, REASON_UNKNOWN, NavError, NavProxy, NavWatcher,  # noqa: E402
                           parse_adapter_url, sanitize_map_info, sanitize_state)
from g1console.navapi import NavApi  # noqa: E402
from g1console.tasks import TaskError, TaskRunner, load_definitions  # noqa: E402
from server import MockHelper, Monitor, make_handler  # noqa: E402

PNG = b"\x89PNG\r\n\x1a\n" + b"\x00" * 16
GOOD_STATE = {
    "connected": True, "pose": {"x": 1.5, "y": -2.0, "yaw_deg": 90.0}, "goal": None,
    "plan": [[0.0, 0.0], [1.0, 1.0]], "route": [], "bridge_state": "READY",
    "patrol": {"state": "IDLE", "index": 0, "total": 0}, "message": "ok",
}


class FakeAdapter:
    """アダプタの代わりの HTTP サーバ。受けたメソッドを記録する（POST が来ないことの確認用）。"""

    def __init__(self):
        outer = self
        self.methods = []
        self.responses = {"/api/state": (200, json.dumps(GOOD_STATE).encode()),
                          "/api/map.json": (200, json.dumps({"resolution": 0.05, "origin": [-10.0, -5.0],
                                                             "width": 200, "height": 100}).encode()),
                          "/api/map.png": (200, PNG)}

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                outer.methods.append("GET")
                code, body = outer.responses.get(self.path, (404, b"{}"))
                self.send_response(code)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                outer.methods.append("POST")
                self.send_response(500)
                self.send_header("Content-Length", "0")
                self.end_headers()

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = "http://127.0.0.1:%d" % self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


class UrlAndSanitizeTest(unittest.TestCase):
    def test_t5_1_adapter_url_is_loopback_http_only(self):
        self.assertEqual(parse_adapter_url("http://127.0.0.1:8081"), "http://127.0.0.1:8081")
        self.assertEqual(parse_adapter_url("http://localhost:8081/"), "http://localhost:8081")
        self.assertEqual(parse_adapter_url("http://[::1]:8081"), "http://[::1]:8081")
        for bad in ("https://127.0.0.1:8081", "http://0.0.0.0:8081", "http://192.168.123.164:8081",
                    "http://127.0.0.1", "http://127.0.0.1:8081/api", "http://u:p@127.0.0.1:8081",
                    "http://127.0.0.1:8081?x=1", "ftp://127.0.0.1:1", "", "127.0.0.1:8081"):
            with self.assertRaises(ValueError, msg=bad):
                parse_adapter_url(bad)

    def test_t5_2_state_is_whitelisted_and_clamped(self):
        raw = dict(GOOD_STATE, secret="x", bridge_state="WHATEVER", message="あ" * 1000)
        out = sanitize_state(raw)
        self.assertNotIn("secret", out)
        self.assertEqual(out["bridge_state"], "UNKNOWN")
        self.assertEqual(len(out["message"]), 300)
        self.assertEqual(out["pose"], {"x": 1.5, "y": -2.0, "yaw_deg": 90.0})

    def test_t5_2b_bad_state_is_rejected(self):
        bad = [[], None, dict(GOOD_STATE, pose={"x": float("nan"), "y": 0, "yaw_deg": 0}),
               dict(GOOD_STATE, pose={"x": True, "y": 0, "yaw_deg": 0}),
               dict(GOOD_STATE, pose={"x": 1e9, "y": 0, "yaw_deg": 0}),
               dict(GOOD_STATE, plan=[[0, 0]] * 2001), dict(GOOD_STATE, plan=[[0, 0, 0]]),
               dict(GOOD_STATE, plan="abc"), dict(GOOD_STATE, message=5),
               dict(GOOD_STATE, patrol={"state": "ru n", "index": 0, "total": 0}),
               dict(GOOD_STATE, patrol={"state": "RUNNING", "index": -1, "total": 0})]
        for raw in bad:
            with self.assertRaises(ValueError, msg=repr(raw)[:60]):
                sanitize_state(raw)
        self.assertFalse(sanitize_state(dict(GOOD_STATE, connected="yes"))["connected"])  # True だけを真とする

    def test_t5_2c_map_info(self):
        good = {"resolution": 0.05, "origin": [-1, 2], "width": 10, "height": 20}
        self.assertEqual(sanitize_map_info(good)["height"], 20)
        for bad in (dict(good, resolution=0), dict(good, resolution=-1), dict(good, origin=[1]),
                    dict(good, width=0.5), dict(good, width=10 ** 7), dict(good, height=True), "x"):
            with self.assertRaises(ValueError, msg=repr(bad)):
                sanitize_map_info(bad)


class ProxyTest(unittest.TestCase):
    def setUp(self):
        self.adapter = FakeAdapter()
        self.proxy = NavProxy(self.adapter.url)

    def tearDown(self):
        self.adapter.close()

    def test_t5_3_reads_state_map_and_never_posts(self):
        self.assertTrue(self.proxy.state()["connected"])
        self.assertEqual(self.proxy.map_info()["width"], 200)
        self.assertTrue(self.proxy.map_png().startswith(b"\x89PNG"))
        self.assertEqual(set(self.adapter.methods), {"GET"})

    def test_t5_3b_adapter_misbehaviour_becomes_nav_error(self):
        cases = {"/api/state": [(404, b"{}"), (200, b"not json"), (200, b"[1]"), (200, b"\xff\xfe"),
                                (200, b"x" * (600 * 1024))],
                 "/api/map.png": [(200, b"GIF89a"), (404, b"{}"), (200, b"\x89PNG\r\n\x1a\n" + b"0" * (9 * 1024 * 1024))]}
        for path, variants in cases.items():
            for resp in variants:
                self.adapter.responses[path] = resp
                call = self.proxy.map_png if path.endswith("png") else self.proxy.state
                with self.assertRaises(NavError, msg="%s %r" % (path, resp[0])):
                    call()

    def test_t5_3c_unreachable_adapter(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        with self.assertRaises(NavError):
            NavProxy("http://127.0.0.1:%d" % port).state()


class FakeProxy:
    def __init__(self):
        self.next = None

    def state(self):
        if isinstance(self.next, Exception):
            raise self.next
        return dict(GOOD_STATE, connected=self.next)


class FakeMotion:
    def __init__(self):
        self.calls = []

    def set_disable_reason(self, reason):
        self.calls.append(reason)


class WatcherTest(unittest.TestCase):
    def setUp(self):
        self.proxy, self.motion = FakeProxy(), FakeMotion()
        self.now = [100.0]
        self.watcher = NavWatcher(self.proxy, self.motion, clock=lambda: self.now[0])

    def poll(self, value):
        self.proxy.next = value
        self.watcher.poll_once()

    def test_t5_4_running_nav2_disables_manual_control_once(self):
        self.poll(True)
        self.poll(True)
        self.assertEqual(self.motion.calls, [REASON_RUNNING])  # 同じ理由を繰り返し設定しない

    def test_t5_5_release_only_when_adapter_says_nav2_is_gone(self):
        self.poll(True)
        self.poll(NavError("down"))
        self.assertEqual(self.motion.calls[-1], REASON_UNKNOWN)  # 黙っても解除しない（fail-safe）
        self.poll(NavError("down"))
        self.assertEqual(self.motion.calls, [REASON_RUNNING, REASON_UNKNOWN])
        self.poll(False)
        self.assertIsNone(self.motion.calls[-1])
        self.poll(NavError("down"))
        self.assertIsNone(self.motion.calls[-1])  # 稼働を見ていない状態に戻ったので、また黙っても無効化しない

    def test_t5_5b_never_seen_means_no_exclusion(self):
        self.poll(NavError("down"))
        self.poll(False)
        self.assertEqual(self.motion.calls, [])

    def test_t5_6_latest_reports_age_and_reason(self):
        self.assertFalse(self.watcher.latest()["available"])
        self.poll(True)
        self.now[0] += 2.0
        latest = self.watcher.latest()
        self.assertEqual((latest["available"], latest["age_s"], latest["exclusive_reason"]), (True, 2.0, REASON_RUNNING))
        self.poll(NavError("x"))
        self.assertFalse(self.watcher.latest()["available"])

    def test_t5_6b_real_motion_controller_drops_the_operator(self):
        from g1console.motion import MockSender, MotionController
        motion = MotionController(MockSender(), lambda: (500, 0.1))
        watcher = NavWatcher(self.proxy, motion)
        session = motion.claim()["session"]
        self.proxy.next = True
        watcher.poll_once()
        self.assertFalse(motion.status()["operator"])
        with self.assertRaises(Exception):
            motion.claim()
        self.assertIsNotNone(session)
        motion.shutdown()


class HeartbeatTest(unittest.TestCase):
    def setUp(self):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", 0))
        self.rx.settimeout(0.5)
        self.now = [10.0]
        self.hb = OperatorHeartbeat("127.0.0.1", self.rx.getsockname()[1], clock=lambda: self.now[0])

    def tearDown(self):
        self.hb.stop()
        self.rx.close()

    def recv(self):
        try:
            return self.rx.recvfrom(64)[0]
        except socket.timeout:
            return None

    def test_t5_7_nothing_is_sent_until_a_human_is_present(self):
        self.assertFalse(self.hb.tick())
        self.assertIsNone(self.recv())

    def test_t5_7b_wire_format_matches_the_receiver(self):
        self.hb.touch()
        self.assertTrue(self.hb.tick())
        self.assertTrue(self.hb.tick())
        first, second = self.recv(), self.recv()
        self.assertEqual(len(first), 29)
        magic, version, session, seq, ns = struct.unpack("<IBQQQ", first)
        self.assertEqual((magic, version, seq), (0x47314842, 1, 1))
        self.assertEqual(first[:4], b"BH1G")  # リトルエンディアンで "G1HB" の並び
        self.assertEqual(struct.unpack("<IBQQQ", second)[2:4], (session, 2))
        self.assertGreater(ns, 0)

    def test_t5_8_stops_when_presence_goes_stale(self):
        self.hb.touch()
        self.assertTrue(self.hb.tick())
        self.now[0] += hb_mod.PRESENCE_MAX_AGE_S + 0.01
        self.assertFalse(self.hb.tick())
        self.assertFalse(self.hb.status()["present"])
        self.recv()
        self.assertIsNone(self.recv())
        self.hb.touch()  # 戻ったら再開（seq は続く）
        self.assertTrue(self.hb.tick())
        self.assertEqual(struct.unpack("<IBQQQ", self.recv())[3], 2)

    def test_t5_8b_new_process_gets_new_session_id(self):
        other = OperatorHeartbeat("127.0.0.1", 9)
        self.assertNotEqual(other._session, self.hb._session)
        other.stop()

    def test_t5_8c_send_failure_is_reported_not_raised(self):
        bad = mock.Mock()
        bad.sendto.side_effect = OSError("network down")
        hb = OperatorHeartbeat("127.0.0.1", 9, sock=bad)
        hb.touch()
        self.assertFalse(hb.tick())

    def test_t5_8d_target_parsing(self):
        self.assertEqual(parse_target("127.0.0.1:47311"), ("127.0.0.1", 47311))
        for bad in ("127.0.0.1", ":47311", "h:0", "h:70000", "h:x", ""):
            with self.assertRaises(ValueError, msg=bad):
                parse_target(bad)

    def test_t5_8e_background_thread_sends_only_while_touched(self):
        self.hb.start()
        self.hb.touch()
        self.assertIsNotNone(self.recv())
        self.hb.stop()


class LocationsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "data" / "locations.json"
        self.store = LocationStore(self.path)

    def tearDown(self):
        self.tmp.cleanup()

    def test_t5_9_crud_and_persistence(self):
        self.assertEqual(self.store.list(), {"locations": []})
        saved = self.store.upsert({"name": "door", "x": 1.23456, "y": -2, "yaw_deg": 179.96, "map_id": "room_a"})
        self.assertEqual(saved, {"name": "door", "x": 1.235, "y": -2.0, "yaw_deg": 180.0, "map_id": "room_a"})
        self.store.upsert({"name": "desk", "x": 0, "y": 0, "yaw_deg": 0})
        self.store.upsert({"name": "door", "x": 5, "y": 5, "yaw_deg": -90})  # 同名は更新
        again = LocationStore(self.path)
        self.assertEqual([i["name"] for i in again.list()["locations"]], ["desk", "door"])
        self.assertEqual(again.list()["locations"][1]["x"], 5.0)
        self.assertEqual(again.delete("desk"), {"deleted": "desk"})
        self.assertEqual([p.name for p in self.path.parent.iterdir()], ["locations.json"])  # 一時ファイルが残らない
        with self.assertRaises(LocationError) as cm:
            again.delete("desk")
        self.assertEqual(cm.exception.status, 404)

    def test_t5_9b_validation(self):
        ok = {"name": "a", "x": 0, "y": 0, "yaw_deg": 0}
        for bad in (dict(ok, name=""), dict(ok, name="a/b"), dict(ok, name="a b"), dict(ok, name="x" * 41),
                    dict(ok, name=5), dict(ok, x=float("nan")), dict(ok, x=float("inf")), dict(ok, x=True),
                    dict(ok, x="1"), dict(ok, x=10001), dict(ok, yaw_deg=181), dict(ok, map_id="../x"),
                    dict(ok, map_id=3), [], None):
            with self.assertRaises(LocationError, msg=repr(bad)) as cm:
                self.store.upsert(bad)
            self.assertEqual(cm.exception.status, 400)
        self.assertEqual(self.store.upsert(dict(ok, name="玄関"))["name"], "玄関")
        self.assertFalse(self.path.with_name("locations.json.tmp").exists())

    def test_t5_9c_corrupt_file_stops_startup_and_is_not_overwritten(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        for text in ("{not json", json.dumps({"version": 2, "locations": []}),
                     json.dumps({"version": 1, "locations": [{"name": "a"}]}),
                     json.dumps({"version": 1, "locations": [dict(name="a", x=0, y=0, yaw_deg=0)] * 2})):
            self.path.write_text(text, encoding="utf-8")
            with self.assertRaises(ValueError, msg=text):
                LocationStore(self.path)
            self.assertEqual(self.path.read_text(encoding="utf-8"), text)

    def test_t5_9d_failed_save_does_not_change_memory(self):
        self.store.upsert({"name": "a", "x": 1, "y": 1, "yaw_deg": 0})
        with mock.patch("os.replace", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.store.upsert({"name": "a", "x": 9, "y": 9, "yaw_deg": 0})
            with self.assertRaises(OSError):
                self.store.delete("a")
        self.assertEqual(self.store.list()["locations"][0]["x"], 1.0)

    def test_t5_9e_limit(self):
        for i in range(200):
            self.store.upsert({"name": "p%d" % i, "x": 0, "y": 0, "yaw_deg": 0})
        with self.assertRaises(LocationError) as cm:
            self.store.upsert({"name": "extra", "x": 0, "y": 0, "yaw_deg": 0})
        self.assertEqual(cm.exception.status, 409)
        self.store.upsert({"name": "p3", "x": 1, "y": 1, "yaw_deg": 0})  # 既存の更新は上限でも通る

    def test_t5_9f_resolve(self):
        self.store.upsert({"name": "door", "x": 1.5, "y": -2, "yaw_deg": 90})
        self.assertEqual([self.store.resolve("door", f) for f in ("x", "y", "yaw_deg")], ["1.500", "-2.000", "90.0"])
        with self.assertRaises(LocationError) as cm:
            self.store.resolve("nope", "x")
        self.assertEqual(cm.exception.status, 409)


class TaskLocationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.store = LocationStore(self.dir / "loc.json")
        code = "import sys;print(sys.argv[1:])"
        defs = {"go": {"argv": [sys.executable, "-u", "-c", code, "--goal={loc:door.x},{loc:door.y}", "{loc:door.yaw_deg}"]}}
        for name, d in defs.items():
            (self.dir / (name + ".json")).write_text(json.dumps(d), encoding="utf-8")
        self.runner = TaskRunner(load_definitions(self.dir, self.dir), self.dir, locations=self.store)

    def tearDown(self):
        self.runner.shutdown()
        self.tmp.cleanup()

    def test_t5_10_placeholders_expand_at_start(self):
        self.store.upsert({"name": "door", "x": 1.5, "y": -2, "yaw_deg": 90})
        self.runner.start("go")
        end = time.monotonic() + 8
        while time.monotonic() < end and not self.runner.logs("go")["lines"]:
            time.sleep(0.02)
        self.assertEqual(self.runner.logs("go")["lines"][0]["text"], "['--goal=1.500,-2.000', '90.0']")

    def test_t5_10b_unregistered_location_refuses_to_start(self):
        with self.assertRaises(TaskError) as cm:
            self.runner.start("go")
        self.assertEqual(cm.exception.status, 409)
        self.assertEqual(self.runner.list()["tasks"][0]["state"], "idle")

    def test_t5_10c_malformed_placeholder_is_rejected_at_load(self):
        for bad in ("{loc:door}", "{loc:door.z}", "{loc:a b.x}", "{loc:door.x", "{loc:}"):
            d = Path(tempfile.mkdtemp(dir=self.tmp.name))
            (d / "t.json").write_text(json.dumps({"argv": ["echo", bad]}), encoding="utf-8")
            with self.assertRaises(ValueError, msg=bad):
                load_definitions(d, d)

    def test_t5_10d_without_locations_a_placeholder_task_fails_cleanly(self):
        runner = TaskRunner(load_definitions(self.dir, self.dir), self.dir)
        with self.assertRaises(TaskError) as cm:
            runner.start("go")
        self.assertEqual(cm.exception.status, 500)


class HttpNavTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.adapter = FakeAdapter()
        cls.proxy = NavProxy(cls.adapter.url)
        cls.motion = FakeMotion()
        cls.watcher = NavWatcher(cls.proxy, cls.motion)
        cls.hb_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cls.hb_rx.bind(("127.0.0.1", 0))
        cls.hb_rx.settimeout(0.5)
        cls.heartbeat = OperatorHeartbeat("127.0.0.1", cls.hb_rx.getsockname()[1])
        cls.api = NavApi(LocationStore(Path(cls.tmp.name) / "l.json"), cls.watcher, cls.proxy, cls.heartbeat)
        helper = MockHelper()
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(helper, Monitor(helper), nav=cls.api))
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.adapter.close()
        cls.heartbeat.stop()
        cls.hb_rx.close()
        cls.tmp.cleanup()

    def req(self, method, path, body=None, raw=False):
        data = json.dumps(body).encode() if body is not None and not isinstance(body, bytes) else body
        r = urllib.request.Request("http://127.0.0.1:%d%s" % (self.port, path), data=data, method=method)
        try:
            resp = urllib.request.urlopen(r, timeout=5)
            return resp.status, (resp.read() if raw else json.load(resp))
        except urllib.error.HTTPError as exc:
            return exc.code, json.load(exc)

    def test_t5_11_state_map_and_exclusion_over_http(self):
        self.assertEqual(self.req("GET", "/api/nav/state")[1]["available"], False)  # 最初の取得前
        self.watcher.poll_once()
        code, state = self.req("GET", "/api/nav/state")
        self.assertEqual((code, state["available"], state["pose"]["x"]), (200, True, 1.5))
        self.assertEqual(state["exclusive_reason"], REASON_RUNNING)
        self.assertTrue(state["heartbeat"]["enabled"])
        self.assertEqual(self.req("GET", "/api/nav/map.json")[1]["resolution"], 0.05)
        code, png = self.req("GET", "/api/nav/map.png", raw=True)
        self.assertEqual((code, png[:4]), (200, b"\x89PNG"))
        self.adapter.responses["/api/map.png"] = (404, b"{}")
        self.assertEqual(self.req("GET", "/api/nav/map.png")[0], 502)
        self.adapter.responses["/api/map.png"] = (200, PNG)

    def test_t5_11b_presence_drives_heartbeat(self):
        self.assertFalse(self.heartbeat.tick())
        self.assertEqual(self.req("POST", "/api/nav/presence", {}), (200, {"ok": True, "heartbeat": True}))
        self.assertTrue(self.heartbeat.tick())
        self.assertEqual(len(self.hb_rx.recvfrom(64)[0]), 29)

    def test_t5_12_locations_over_http(self):
        body = {"name": "door", "x": 1, "y": 2, "yaw_deg": 3}
        self.assertEqual(self.req("POST", "/api/locations/save", body)[0], 200)
        self.assertEqual(self.req("GET", "/api/locations")[1]["locations"][0]["name"], "door")
        self.assertEqual(self.req("POST", "/api/locations/save", dict(body, x="a"))[0], 400)
        self.assertEqual(self.req("POST", "/api/locations/save", b"{bad")[0], 400)
        self.assertEqual(self.req("POST", "/api/locations/delete", {"name": 5})[0], 400)
        self.assertEqual(self.req("POST", "/api/locations/delete", {"name": "zzz"})[0], 404)
        self.assertEqual(self.req("POST", "/api/locations/delete", {"name": "door"})[0], 200)

    def test_t5_12b_no_route_reaches_nav2(self):
        before = list(self.adapter.methods)
        for path in ("/api/nav/goal", "/api/nav/command/enable_navigation", "/api/nav/start", "/api/locations/go"):
            self.assertEqual(self.req("POST", path, {})[0], 404, path)
        self.assertEqual(self.adapter.methods, before)
        self.assertNotIn("POST", self.adapter.methods)

    def test_t5_12c_without_adapter_it_says_so(self):
        api = NavApi(LocationStore(Path(self.tmp.name) / "none.json"))
        self.assertEqual(json.loads(api.get("/api/nav/state")[2])["available"], False)
        self.assertEqual(api.get("/api/nav/map.png")[0], 404)
        self.assertEqual(json.loads(api.post("/api/nav/presence", {})[2])["heartbeat"], False)


if __name__ == "__main__":
    unittest.main()
