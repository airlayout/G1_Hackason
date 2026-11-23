"""ハブ自体の停止（POST /api/hub/stop）。"""
import http.client
import json
import os
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import server  # noqa: E402
from g1console.auth import Auth  # noqa: E402
from g1console.motion import MockSender, MotionController  # noqa: E402
from g1console.settings import Settings  # noqa: E402
from server import MockHelper, Monitor, make_handler  # noqa: E402


class FakeTasks:
    def __init__(self, state="idle"):
        self.state = state

    def list(self):
        return {"tasks": [{"name": "t", "state": self.state}]}


class HubStopTest(unittest.TestCase):
    def setUp(self):
        self.stopped = threading.Event()
        self.calls = []
        self._delay = server.HUB_STOP_DELAY_S
        server.HUB_STOP_DELAY_S = 0.01
        self.servers = []

    def tearDown(self):
        server.HUB_STOP_DELAY_S = self._delay
        for srv in self.servers:
            srv.shutdown()
            srv.server_close()

    def start(self, hub_stop=True, **kw):
        def stop():
            self.calls.append(1)
            self.stopped.set()
        helper = MockHelper()
        handler = make_handler(helper, Monitor(helper), None, Settings(None), None, None,
                               hub_stop=stop if hub_stop else None, **kw)
        srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.servers.append(srv)
        return srv.server_address[1]

    def post(self, port, path="/api/hub/stop", body=b'{"confirm": true}', headers=None):
        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
        conn.request("POST", path, body=body, headers={"Host": "127.0.0.1:%d" % port, **(headers or {})})
        res = conn.getresponse()
        data = res.read()
        conn.close()
        return res.status, json.loads(data or b"{}")

    def test_confirm_true_stops_after_responding(self):
        port = self.start()
        status, body = self.post(port)
        self.assertEqual(status, 200)
        self.assertTrue(body["ok"])
        self.assertTrue(self.stopped.wait(2))
        self.assertEqual(len(self.calls), 1)

    def test_without_confirm_or_with_bad_body_is_400_and_does_not_stop(self):
        port = self.start()
        for raw in (b"{}", b'{"confirm": "yes"}', b'{"confirm": 1}', b'{"confirm": false}', b"not json", b"[]"):
            self.assertEqual(self.post(port, body=raw)[0], 400, raw)
        self.assertFalse(self.stopped.wait(0.3))

    def test_get_is_not_a_stop(self):
        port = self.start()
        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
        conn.request("GET", "/api/hub/stop")
        self.assertEqual(conn.getresponse().status, 404)
        conn.close()
        self.assertFalse(self.stopped.wait(0.3))

    def test_not_available_without_hub_stop(self):
        port = self.start(hub_stop=False)
        self.assertEqual(self.post(port)[0], 404)

    def test_running_task_blocks_stop(self):
        for state in ("running", "stopping"):
            port = self.start(tasks=FakeTasks(state))
            status, body = self.post(port)
            self.assertEqual(status, 409, state)
            self.assertIn("タスク", body["error"])
        self.assertFalse(self.stopped.wait(0.3))

    def test_idle_task_does_not_block(self):
        port = self.start(tasks=FakeTasks("idle"))
        self.assertEqual(self.post(port)[0], 200)
        self.assertTrue(self.stopped.wait(2))

    def test_held_operator_session_blocks_stop(self):
        motion = MotionController(MockSender(), lambda: (501, 0.1))
        motion.status = lambda: {"operator": True}  # 操作権が取られている状態
        port = self.start(motion=motion)
        status, body = self.post(port)
        self.assertEqual(status, 409)
        self.assertIn("操作権", body["error"])
        self.assertFalse(self.stopped.wait(0.3))

    def test_requires_auth_when_enabled(self):
        auth = Auth("tok-for-hub-stop", [])
        port = self.start(auth=auth)
        self.assertEqual(self.post(port)[0], 401)
        self.assertFalse(self.stopped.wait(0.3))
        status, _ = self.post(port, headers={"Authorization": "Bearer tok-for-hub-stop"})
        self.assertEqual(status, 200)

    def test_cross_origin_post_is_rejected_when_auth_enabled(self):
        auth = Auth("tok-for-hub-stop", [])
        port = self.start(auth=auth)
        status, _ = self.post(port, headers={"Authorization": "Bearer tok-for-hub-stop", "Origin": "http://evil.example"})
        self.assertEqual(status, 403)
        self.assertFalse(self.stopped.wait(0.3))


if __name__ == "__main__":
    unittest.main()
