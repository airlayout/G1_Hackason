"""Console のテスト。実機も ssh も要らない（MockHelper と標準ライブラリのみ）。"""
import json
import sys
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from modes import ALLOWED_IDS, BUTTONS, FSM_LABELS  # noqa: E402
from server import MockHelper, describe, make_handler  # noqa: E402


class DescribeTest(unittest.TestCase):
    def test_known_fsm_id_is_labeled(self):
        raw = {"ok": True, "service": "ai", "fsm_id": 501}
        out = describe(raw)
        self.assertEqual(out["state"], "ok")
        self.assertEqual(out["label"], FSM_LABELS[501])

    def test_unknown_fsm_id_is_not_hidden(self):
        out = describe({"ok": True, "service": "ai", "fsm_id": 12345})
        self.assertEqual(out["state"], "ok")
        self.assertEqual(out["label"], "不明")
        self.assertEqual(out["fsm_id"], 12345)

    def test_empty_service_name_is_debug_mode(self):
        self.assertEqual(describe({"ok": True, "service": "", "fsm_id": None})["state"], "debug")

    def test_helper_failure_is_offline(self):
        self.assertEqual(describe({"ok": False, "offline": True, "error": "x"})["state"], "offline")

    def test_missing_fsm_id_is_unknown_not_ok(self):
        self.assertEqual(describe({"ok": True, "service": "ai", "fsm_id": None})["state"], "unknown")


class ButtonsTest(unittest.TestCase):
    def test_every_button_id_is_allowed_and_labeled(self):
        for b in BUTTONS:
            self.assertIn(b["id"], ALLOWED_IDS)
            self.assertIn(b["id"], FSM_LABELS)


class HttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(MockHelper()))
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def _post(self, body: bytes):
        req = urllib.request.Request("http://127.0.0.1:%d/api/mode" % self.port, data=body, method="POST")
        try:
            r = urllib.request.urlopen(req, timeout=5)
            return r.status, json.load(r)
        except urllib.error.HTTPError as e:
            return e.code, json.load(e)

    def _status(self):
        return json.load(urllib.request.urlopen("http://127.0.0.1:%d/api/status" % self.port, timeout=5))

    def test_allowed_mode_is_applied(self):
        code, _ = self._post(json.dumps({"id": 3}).encode())
        self.assertEqual(code, 200)
        self.assertEqual(self._status()["fsm_id"], 3)

    def test_disallowed_id_is_rejected_and_state_unchanged(self):
        self._post(json.dumps({"id": 1}).encode())
        code, _ = self._post(json.dumps({"id": 999}).encode())
        self.assertEqual(code, 400)
        self.assertEqual(self._status()["fsm_id"], 1)

    def test_malformed_body_is_rejected(self):
        self.assertEqual(self._post(b"not json")[0], 400)
        self.assertEqual(self._post(json.dumps({"x": 1}).encode())[0], 400)

    def test_index_html_is_served(self):
        html = urllib.request.urlopen("http://127.0.0.1:%d/" % self.port, timeout=5).read().decode()
        self.assertIn("G1 開発コンソール", html)


if __name__ == "__main__":
    unittest.main()
