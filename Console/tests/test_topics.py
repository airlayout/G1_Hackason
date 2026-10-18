"""トピック通信（段階 6、R-19）。ロボットにも DDS にも触れない。
T6-1…T6-8 は docs/TEST_SPEC.md の番号に対応する。"""
import ast
import json
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from g1console import topic_backend  # noqa: E402
from g1console.topic_backend import HelperTopicBackend  # noqa: E402
from g1console.topics import (DENY_PATTERNS, MAX_SUBSCRIPTIONS, MockTopicBackend, TopicError, TopicService,  # noqa: E402
                              is_denied, load_allowlist, validate_topic)
from server import MockHelper, Monitor, make_handler  # noqa: E402

ALLOW_TOPIC = "rt/console_demo/say"
HELPER_SRC = Path(__file__).resolve().parent.parent / "jetson" / "topic_helper.py"


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def rule(topic=ALLOW_TOPIC, max_bytes=16, interval=1.0):
    return {topic: {"topic": topic, "type": "String_", "max_bytes": max_bytes, "min_interval_s": interval, "note": ""}}


def service(allow=None, clock=None):
    backend = MockTopicBackend()
    return TopicService(backend, rule() if allow is None else allow, clock or Clock()), backend


def status_of(call, *args):
    try:
        call(*args)
    except TopicError as exc:
        return exc.status
    return 200


class PolicyTest(unittest.TestCase):
    def test_t6_1_deny_list_beats_allow_list(self):  # T6-1
        for topic in ("rt/lowcmd", "rt/lf/lowcmd", "rt/arm_sdk", "rt/api/sport/request", "rt/api/loco/request",
                      "rt/api/motion_switcher/request", "rt/dex3/left/cmd", "rt/wirelesscontroller", "rt/lf/emergency_stop"):
            self.assertTrue(is_denied(topic), topic)
            svc, backend = service(rule(topic))  # 許可リストに載せても、実行時に拒否する
            self.assertEqual(status_of(svc.publish, topic, "x"), 403, topic)
            self.assertEqual(backend.published, [])

    def test_t6_1b_allowlist_file_with_denied_topic_stops_startup(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "topics.json"
            path.write_text(json.dumps({"version": 1, "publish": [{"topic": "rt/lowcmd"}]}), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_allowlist(path)

    def test_t6_1c_deny_is_case_insensitive_and_names_are_validated(self):
        self.assertTrue(is_denied("rt/LowCmd"))
        for bad in ("lowcmd", "rt/", "rt//x", "rt/../x", "rt/a b", "rt/" + "a" * 130, None, 5):
            with self.assertRaises(TopicError) as cm:
                validate_topic(bad)
            self.assertEqual(cm.exception.status, 400)

    def test_t6_2_not_allowlisted_is_denied_and_default_is_empty(self):  # T6-2
        svc, backend = service(allow={})
        self.assertEqual(status_of(svc.publish, ALLOW_TOPIC, "hi"), 403)
        self.assertEqual(backend.published, [])
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(load_allowlist(Path(tmp) / "none.json"), {})  # ファイルが無ければ何も送れない

    def test_t6_2b_allowed_topic_publishes_and_checks_size_and_type(self):
        svc, backend = service()
        self.assertEqual(svc.publish(ALLOW_TOPIC, "hello"), {"published": ALLOW_TOPIC, "bytes": 5})
        self.assertEqual(backend.published, [(ALLOW_TOPIC, "hello")])
        clock = svc._clock
        clock.now += 5
        self.assertEqual(status_of(svc.publish, ALLOW_TOPIC, "x" * 17), 413)
        self.assertEqual(status_of(svc.publish, ALLOW_TOPIC, "あ" * 6), 413)  # バイト数で数える（18 バイト）
        for bad in ("", None, 5, ["a"]):
            self.assertEqual(status_of(svc.publish, ALLOW_TOPIC, bad), 400)
        self.assertEqual(len(backend.published), 1)

    def test_t6_2c_rate_limit_uses_min_interval(self):
        clock = Clock()
        svc, backend = service(clock=clock)
        svc.publish(ALLOW_TOPIC, "a")
        clock.now += 0.5
        self.assertEqual(status_of(svc.publish, ALLOW_TOPIC, "b"), 429)
        clock.now += 0.6
        svc.publish(ALLOW_TOPIC, "c")
        self.assertEqual([d for _, d in backend.published], ["a", "c"])

    def test_t6_2d_failed_publish_does_not_consume_the_interval(self):
        clock = Clock()
        svc, backend = service(clock=clock)
        backend.publish = lambda topic, data: (_ for _ in ()).throw(TopicError(502, "down"))
        self.assertEqual(status_of(svc.publish, ALLOW_TOPIC, "a"), 502)
        backend.publish = lambda topic, data: None
        self.assertEqual(status_of(svc.publish, ALLOW_TOPIC, "a"), 200)

    def test_t6_3_unsupported_type_is_refused_not_faked(self):  # T6-3
        svc, backend = service()
        self.assertEqual(status_of(svc.subscribe, "rt/utlidar/cloud_livox_mid360", "PointCloud2_"), 422)
        self.assertEqual(status_of(svc.subscribe, "rt/rtc/state", None), 422)
        self.assertEqual(svc.info()["subscribed"], [])
        self.assertEqual(backend.poll(), [])

    def test_t6_4_subscribe_receive_unsubscribe_and_limit(self):  # T6-4
        svc, _ = service()
        self.assertEqual(svc.received(), {"items": []})
        svc.subscribe("rt/rtc/state", "String_")
        items = svc.received()["items"]
        self.assertEqual((items[0]["topic"], items[0]["message"]), ("rt/rtc/state", {"data": "mock 1"}))
        for n in range(MAX_SUBSCRIPTIONS - 1):
            svc.subscribe("rt/t%d" % n, "String_")
        self.assertEqual(status_of(svc.subscribe, "rt/overflow", "String_"), 409)
        svc.subscribe("rt/rtc/state", "String_")  # 既存の再購読は上限に数えない
        svc.unsubscribe("rt/rtc/state")
        self.assertEqual(status_of(svc.unsubscribe, "rt/rtc/state"), 404)
        svc.subscribe("rt/overflow", "String_")

    def test_t6_5_subscribing_to_denied_topic_is_allowed_read_only(self):  # T6-5 受信は自由
        svc, _ = service()
        svc.subscribe("rt/lowcmd", "String_")
        self.assertIn("rt/lowcmd", svc.info()["subscribed"])
        self.assertEqual(status_of(svc.publish, "rt/lowcmd", "x"), 403)

    def test_allowlist_file_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "topics.json"
            good = {"version": 1, "publish": [{"topic": ALLOW_TOPIC, "max_bytes": 64, "min_interval_s": 0.5, "note": "デモ"}]}
            path.write_text(json.dumps(good), encoding="utf-8")
            self.assertEqual(load_allowlist(path)[ALLOW_TOPIC]["max_bytes"], 64)
            for bad in ({"version": 2, "publish": []}, {"version": 1, "publish": "x"}, {"publish": []},
                        {"version": 1, "publish": [{"topic": ALLOW_TOPIC, "type": "LowCmd_"}]},
                        {"version": 1, "publish": [{"topic": ALLOW_TOPIC, "max_bytes": 99999}]},
                        {"version": 1, "publish": [{"topic": ALLOW_TOPIC, "min_interval_s": 0.0}]},
                        {"version": 1, "publish": [{"topic": "bad name"}]}):
                path.write_text(json.dumps(bad), encoding="utf-8")
                with self.assertRaises(ValueError, msg=str(bad)):
                    load_allowlist(path)
            path.write_text("not json", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_allowlist(path)


class HelperTest(unittest.TestCase):
    def test_t6_6_helper_source_is_valid_and_has_no_motion_clients(self):  # T6-6
        src = HELPER_SRC.read_text(encoding="utf-8")
        ast.parse(src.replace("__PUBLISH__", "{}").replace("__DENY__", "[]"))
        for forbidden in ("LocoClient", "SetVelocity", "SetFsmId", "SportClient", "MotionSwitcher", "LowCmd_", "ArmSdk"):
            self.assertNotIn(forbidden, src)
        self.assertIn("denied(topic)", src)  # ハブとは別に、ヘルパーでも拒否リストを検査する
        self.assertEqual(sum(src.count(op) for op in ('op == "sub"', 'op == "unsub"', 'op == "poll"', 'op == "pub"')), 4)

    def test_t6_6b_backend_substitutes_allowlist_and_deny(self):
        src = HelperTopicBackend(rule())._source()
        self.assertNotIn("__PUBLISH__", src)
        self.assertNotIn("__DENY__", src)
        self.assertIn(ALLOW_TOPIC, src)
        for pattern in DENY_PATTERNS:
            self.assertIn(pattern, src)
        ast.parse(src)

    def _fake(self, body):
        backend = HelperTopicBackend(rule())
        backend._command = lambda src: ([sys.executable, "-u", "-c", body], None)
        return backend

    FAKE_OK = ("import sys, json\nprint(json.dumps({'ready': True}), flush=True)\n"
               "for line in sys.stdin:\n    r = json.loads(line)\n"
               "    out = {'ok': True}\n    if r['op'] == 'poll':\n        out['items'] = [{'topic': 't', 'type': 'String_', 'count': 1, 'age_s': 0.1, 'message': {'data': 'x'}}]\n"
               "    print(json.dumps(out), flush=True)\n")

    def test_t6_7_helper_backend_with_fake_child(self):  # T6-7
        backend = self._fake(self.FAKE_OK)
        try:
            self.assertIsNone(backend._proc)  # 使うまで起動しない
            backend.subscribe("rt/rtc/state", "String_")
            self.assertEqual(backend.poll()[0]["message"], {"data": "x"})
            backend.publish(ALLOW_TOPIC, "hi")
            backend.unsubscribe("rt/rtc/state")
        finally:
            backend.close()
        self.assertIsNone(backend._proc)

    def test_t6_7b_silent_child_times_out_and_subscriptions_are_restored(self):
        old = topic_backend.HELPER_TIMEOUT_S
        topic_backend.HELPER_TIMEOUT_S = 0.3
        try:
            hang = "import sys, json, time\nprint(json.dumps({'ready': True}), flush=True)\nfor line in sys.stdin:\n    time.sleep(30)\n"
            backend = self._fake(hang)
            with self.assertRaises(TopicError) as cm:
                backend.subscribe("rt/rtc/state", "String_")
            self.assertEqual(cm.exception.status, 502)
            self.assertIsNone(backend._proc)
            backend._subs["rt/rtc/state"] = "String_"  # 以前の購読が残っていれば、次に起動したとき復元する
            backend._command = lambda src: ([sys.executable, "-u", "-c", self.FAKE_OK], None)
            self.assertEqual(backend.poll()[0]["topic"], "t")
        finally:
            topic_backend.HELPER_TIMEOUT_S = old
            backend.close()

    def test_t6_7c_helper_rejection_is_an_error(self):
        refuse = ("import sys, json\nprint(json.dumps({'ready': True}), flush=True)\n"
                  "for line in sys.stdin:\n    print(json.dumps({'ok': False, 'error': 'not allowed'}), flush=True)\n")
        backend = self._fake(refuse)
        try:
            with self.assertRaises(TopicError) as cm:
                backend.publish(ALLOW_TOPIC, "hi")
            self.assertEqual(cm.exception.status, 502)
        finally:
            backend.close()

    def test_t6_7d_child_that_never_becomes_ready_is_502(self):
        backend = self._fake("import sys\nsys.exit(1)\n")
        with self.assertRaises(TopicError) as cm:
            backend.poll()
        self.assertEqual(cm.exception.status, 502)


class HttpTopicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        helper = MockHelper()
        cls.backend = MockTopicBackend()
        cls.svc = TopicService(cls.backend, rule(max_bytes=32, interval=0.1))
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(helper, Monitor(helper), topics=cls.svc))
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()
        cls.off = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(helper, Monitor(helper)))
        cls.off_port = cls.off.server_address[1]
        threading.Thread(target=cls.off.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.off.shutdown()
        cls.off.server_close()

    def _req(self, path, body=None, port=None):
        data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
        req = urllib.request.Request("http://127.0.0.1:%d%s" % (port or self.port, path), data=data, method="GET" if data is None else "POST")
        try:
            r = urllib.request.urlopen(req, timeout=5)
            return r.status, json.load(r)
        except urllib.error.HTTPError as exc:
            return exc.code, json.load(exc)

    def test_t6_8_http_roundtrip_and_error_codes(self):  # T6-8
        code, info = self._req("/api/topics")
        self.assertEqual((code, info["backend"], info["publish"][0]["topic"]), (200, "mock", ALLOW_TOPIC))
        self.assertEqual(self._req("/api/topics/subscribe", {"topic": "rt/rtc/state", "type": "String_"})[0], 200)
        self.assertEqual(self._req("/api/topics/received")[1]["items"][0]["topic"], "rt/rtc/state")
        self.assertEqual(self._req("/api/topics/subscribe", {"topic": "rt/x", "type": "PointCloud2_"})[0], 422)
        self.assertEqual(self._req("/api/topics/subscribe", {"topic": "bad", "type": "String_"})[0], 400)
        self.assertEqual(self._req("/api/topics/publish", {"topic": "rt/lowcmd", "data": "x"})[0], 403)
        self.assertEqual(self._req("/api/topics/publish", {"topic": "rt/other/topic", "data": "x"})[0], 403)
        self.assertEqual(self._req("/api/topics/publish", {"topic": ALLOW_TOPIC, "data": "x" * 33})[0], 413)
        self.assertEqual(self._req("/api/topics/publish", {"topic": ALLOW_TOPIC, "data": "hi"})[0], 200)
        self.assertEqual(self._req("/api/topics/publish", {"topic": ALLOW_TOPIC, "data": "hi"})[0], 429)
        self.assertEqual(self._req("/api/topics/publish", b"not json")[0], 400)
        self.assertEqual(self._req("/api/topics/publish", [1, 2])[0], 400)
        self.assertEqual(self._req("/api/topics/nope", {})[0], 404)
        self.assertEqual(self._req("/api/topics/unsubscribe", {"topic": "rt/rtc/state"})[0], 200)
        self.assertEqual(self.backend.published, [(ALLOW_TOPIC, "hi")])

    def test_topics_are_absent_when_not_configured(self):
        self.assertEqual(self._req("/api/topics", port=self.off_port)[0], 404)
        self.assertEqual(self._req("/api/topics/publish", {"topic": ALLOW_TOPIC, "data": "x"}, port=self.off_port)[0], 404)


if __name__ == "__main__":
    unittest.main()
