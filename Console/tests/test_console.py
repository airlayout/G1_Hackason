"""Console のテスト。実機も ssh も要らない（MockHelper と標準ライブラリのみ）。"""
import json
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from modes import ALLOWED_IDS, BUTTONS, FEATURES, FSM_LABELS, PLANNED_MODES  # noqa: E402
from server import MockHelper, Monitor, describe, make_handler  # noqa: E402

G1_TIMEOUT = {"ok": True, "checkmode_code": 3102, "service": None, "fsm_code": 3102, "fsm_id": None}


class DescribeTest(unittest.TestCase):
    def test_known_fsm_id_is_labeled(self):
        out = describe({"ok": True, "checkmode_code": 0, "service": "ai", "fsm_code": 0, "fsm_id": 501})
        self.assertEqual(out["jetson"]["state"], "ok")
        self.assertEqual(out["g1"]["state"], "ok")
        self.assertEqual(out["mode"]["state"], "ok")
        self.assertEqual(out["mode"]["label"], FSM_LABELS[501])

    def test_unknown_fsm_id_is_not_hidden(self):
        out = describe({"ok": True, "checkmode_code": 0, "service": "ai", "fsm_code": 0, "fsm_id": 12345})
        self.assertEqual(out["mode"]["state"], "ok")
        self.assertEqual(out["mode"]["label"], "不明")
        self.assertEqual(out["mode"]["fsm_id"], 12345)

    def test_empty_service_name_is_debug_mode_and_g1_reachable(self):
        out = describe({"ok": True, "checkmode_code": 0, "service": "", "fsm_code": 3102, "fsm_id": None})
        self.assertEqual(out["g1"]["state"], "ok")
        self.assertEqual(out["mode"]["state"], "debug")

    def test_ssh_failure_means_jetson_down_and_g1_unverified(self):
        out = describe({"ok": False, "offline": True, "ssh_exit": 255, "error": "x"})
        self.assertEqual(out["jetson"]["state"], "down")
        self.assertIn("255", out["jetson"]["detail"])
        self.assertEqual(out["g1"]["state"], "unknown")
        self.assertEqual(out["mode"]["state"], "none")

    def test_dds_timeout_means_g1_down_while_jetson_ok(self):
        out = describe(G1_TIMEOUT)
        self.assertEqual(out["jetson"]["state"], "ok")
        self.assertEqual(out["g1"]["state"], "down")
        self.assertEqual(out["mode"]["state"], "none")

    def test_reachable_but_no_fsm_id_is_unknown_mode_not_ok(self):
        out = describe({"ok": True, "checkmode_code": 0, "service": "ai", "fsm_code": 7, "fsm_id": None})
        self.assertEqual(out["g1"]["state"], "ok")
        self.assertEqual(out["mode"]["state"], "unknown")


class ButtonsTest(unittest.TestCase):
    def test_every_button_id_is_allowed_and_labeled(self):
        for b in BUTTONS:
            self.assertIn(b["id"], ALLOWED_IDS)
            self.assertIn(b["id"], FSM_LABELS)


class FeaturesTest(unittest.TestCase):
    def test_unverified_or_unimplemented_is_never_marked_verified(self):
        for f in FEATURES:
            if not f["implemented"]:
                self.assertFalse(f["verified"], f["id"])

    def test_nothing_is_marked_verified_before_a_real_robot_run(self):
        # 実機（G1 電源オン・コンソール経由）で試すまでは、全機能を「実機未確認」に保つ。
        self.assertFalse(any(f["verified"] for f in FEATURES))

    def test_planned_modes_are_not_switchable(self):
        planned = {p["label"] for p in PLANNED_MODES}
        self.assertFalse(planned & {b["label"] for b in BUTTONS})

    def test_ids_are_unique(self):
        ids = [f["id"] for f in FEATURES]
        self.assertEqual(len(ids), len(set(ids)))


class MonitorTest(unittest.TestCase):
    def test_before_first_poll_nothing_is_claimed_connected(self):
        snap = Monitor(MockHelper()).snapshot()
        self.assertIsNone(snap["sampled_at"])
        self.assertIsNone(snap["age_s"])
        self.assertEqual(snap["g1"]["state"], "unknown")

    def test_snapshot_reports_age_and_latency(self):
        mon = Monitor(MockHelper())
        mon.poll_once()
        time.sleep(0.05)
        snap = mon.snapshot()
        self.assertGreaterEqual(snap["age_s"], 0.05)
        self.assertIsNotNone(snap["latency_ms"])
        self.assertTrue(snap["server"]["mock"])

    def test_last_known_mode_survives_disconnect(self):
        helper = MockHelper()
        mon = Monitor(helper)
        mon.poll_once()
        helper.scenario = "g1_off"
        snap = mon.poll_once()
        self.assertEqual(snap["g1"]["state"], "down")
        self.assertEqual(snap["mode"]["state"], "none")
        self.assertEqual(snap["last_mode"]["fsm_id"], 1)

    def test_snapshots_are_replaced_not_mutated(self):
        mon = Monitor(MockHelper())
        first = mon.poll_once()
        second = mon.poll_once()
        self.assertIsNot(first, second)
        self.assertLessEqual(first["sampled_at"], second["sampled_at"])


class HttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.helper = MockHelper()
        cls.monitor = Monitor(cls.helper)
        cls.monitor.poll_once()
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(cls.helper, cls.monitor))
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def setUp(self):
        self.helper.scenario = "ok"

    def _post(self, path: str, body: bytes):
        req = urllib.request.Request("http://127.0.0.1:%d%s" % (self.port, path), data=body, method="POST")
        try:
            r = urllib.request.urlopen(req, timeout=5)
            return r.status, json.load(r)
        except urllib.error.HTTPError as e:
            return e.code, json.load(e)

    def _status(self):
        return json.load(urllib.request.urlopen("http://127.0.0.1:%d/api/status" % self.port, timeout=5))

    def test_status_carries_links_and_freshness(self):
        s = self._status()
        for key in ("jetson", "g1", "mode", "age_s", "latency_ms", "poll_interval_s", "server"):
            self.assertIn(key, s)

    def test_allowed_mode_is_sent_and_reached_after_transition(self):
        code, _ = self._post("/api/mode", json.dumps({"id": 3}).encode())
        self.assertEqual(code, 200)
        time.sleep(MockHelper.TRANSITION_S + 0.2)
        self.monitor.poll_once()
        self.assertEqual(self._status()["mode"]["fsm_id"], 3)

    def test_disallowed_id_is_rejected_before_reaching_helper(self):
        code, _ = self._post("/api/mode", json.dumps({"id": 999}).encode())
        self.assertEqual(code, 400)

    def test_malformed_body_is_rejected(self):
        self.assertEqual(self._post("/api/mode", b"not json")[0], 400)
        self.assertEqual(self._post("/api/mode", json.dumps({"x": 1}).encode())[0], 400)

    def test_set_is_reported_as_failure_when_g1_unreachable(self):
        self.helper.scenario = "g1_off"
        code, _ = self._post("/api/mode", json.dumps({"id": 3}).encode())
        self.assertEqual(code, 502)

    def test_scenario_switch_is_reflected_immediately(self):
        code, _ = self._post("/api/scenario", json.dumps({"scenario": "jetson_off"}).encode())
        self.assertEqual(code, 200)
        self.assertEqual(self._status()["jetson"]["state"], "down")
        self.assertEqual(self._post("/api/scenario", json.dumps({"scenario": "nope"}).encode())[0], 400)

    def test_features_endpoint_lists_planned_items(self):
        j = json.load(urllib.request.urlopen("http://127.0.0.1:%d/api/features" % self.port, timeout=5))
        self.assertEqual({f["id"] for f in j["features"]},
                         {"connection", "reconnect", "mode", "action", "move", "battery"})
        self.assertTrue(j["planned_modes"] and j["planned_actions"])

    def test_index_html_is_served(self):
        html = urllib.request.urlopen("http://127.0.0.1:%d/" % self.port, timeout=5).read().decode()
        self.assertIn("G1 開発コンソール", html)


if __name__ == "__main__":
    unittest.main()
