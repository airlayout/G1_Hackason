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

from dds_catalog import DISPLAYED, SERVICES, TOPICS  # noqa: E402
from modes import validate_audio, ALLOWED_IDS, BUTTONS, FEATURES, FSM_LABELS, PLANNED_MODES  # noqa: E402
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


class ReplayTest(unittest.TestCase):
    def test_telemetry_has_helper_shape_and_loops(self):
        from replay import Replay
        r = Replay()
        for now in (r._t0, r._t0 + 100, r._t0 + r.length_s + 30):
            t = r.telemetry(now)
            self.assertEqual(len(t["joints"]["items"]), 29)
            self.assertEqual(len(t["lowcmd"]["items"]), 29)
            self.assertIn("soc", t["battery"])
            self.assertEqual(t["odom"]["error_code"], 0)
        self.assertGreater(r.telemetry(r._t0)["battery"]["soc"], r.telemetry(r._t0 + r.length_s - 2)["battery"]["soc"])

    def test_replay_scenario_reports_debug_mode_with_real_telemetry(self):
        m = MockHelper()
        m.scenario = "replay"
        raw = m.call({"op": "get"})
        self.assertEqual((raw["checkmode_code"], raw["service"]), (0, ""))
        self.assertEqual(raw["telemetry"]["joints"]["mode_machine"], 5)


class DdsCatalogTest(unittest.TestCase):
    def test_topics_are_unique_and_status_known(self):
        names = [t[0] for t in TOPICS]
        self.assertEqual(len(names), len(set(names)))
        self.assertTrue({t[2] for t in TOPICS} <= {"displayed", "todo", "excluded"})
        self.assertEqual(len(TOPICS), 65)  # 発見した 104 件からサービスの request/response 39 件を除いた数

    def test_displayed_topics_match_helper_subscriptions(self):
        # remote_helper.py が購読しているトピックと、表示済みの一覧は一致していなければならない
        src = (Path(__file__).resolve().parent.parent / "remote_helper.py").read_text(encoding="utf-8")
        for name in (t[0] for t in TOPICS if t[2] == DISPLAYED):
            self.assertIn('"%s"' % name, src)
        self.assertEqual(len([t for t in TOPICS if t[2] == DISPLAYED]), 18)

    def test_services_have_names(self):
        self.assertEqual(len({v[0] for v in SERVICES}), len(SERVICES))
        self.assertEqual(len(SERVICES), 25)  # 発見した rt/api/<名前> の種類数


class AudioValidationTest(unittest.TestCase):
    def test_valid_requests_become_helper_ops(self):
        self.assertEqual(validate_audio("volume", {"volume": 60}), {"op": "audio_volume", "volume": 60})
        self.assertEqual(validate_audio("led", {"r": 0, "g": 255, "b": 3})["op"], "audio_led")
        self.assertEqual(validate_audio("tts", {"text": "こんにちは", "speaker_id": 0})["op"], "audio_tts")

    def test_out_of_range_or_wrong_type_is_rejected(self):
        bad = [("volume", {"volume": 101}), ("volume", {"volume": -1}), ("volume", {"volume": "50"}),
               ("volume", {"volume": True}), ("volume", {}), ("led", {"r": 256, "g": 0, "b": 0}),
               ("led", {"r": 0, "g": 0}), ("tts", {"text": "", "speaker_id": 0}),
               ("tts", {"text": "a" * 101, "speaker_id": 0}), ("tts", {"text": "a", "speaker_id": 7}),
               ("nope", {})]
        for kind, body in bad:
            with self.assertRaises(ValueError, msg=(kind, body)):
                validate_audio(kind, body)


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
        d = json.load(urllib.request.urlopen("http://127.0.0.1:%d/api/dds" % self.port, timeout=5))
        self.assertEqual(len(d["topics"]), 65)
        self.assertEqual(d["topics"][0]["status"], "displayed")
        self.assertTrue(d["services"])
        j = json.load(urllib.request.urlopen("http://127.0.0.1:%d/api/features" % self.port, timeout=5))
        self.assertEqual({f["id"] for f in j["features"]},
                         {"connection", "reconnect", "mode", "action", "move", "battery", "audio", "sensors"})
        self.assertTrue(j["planned_modes"] and j["planned_actions"])

    def test_volume_set_is_read_back_and_verified(self):
        self.assertEqual(json.load(urllib.request.urlopen("http://127.0.0.1:%d/api/audio/volume" % self.port))["volume"], 85)
        code, body = self._post("/api/audio/volume", json.dumps({"volume": 40}).encode())
        self.assertEqual((code, body["volume_after"]), (200, 40))
        self._post("/api/audio/volume", json.dumps({"volume": 85}).encode())

    def test_led_and_tts_are_accepted_but_bad_input_is_400(self):
        self.assertEqual(self._post("/api/audio/led", json.dumps({"r": 1, "g": 2, "b": 3}).encode())[0], 200)
        self.assertEqual(self._post("/api/audio/tts", json.dumps({"text": "テスト", "speaker_id": 0}).encode())[0], 200)
        self.assertEqual(self._post("/api/audio/led", json.dumps({"r": 999, "g": 0, "b": 0}).encode())[0], 400)
        self.assertEqual(self._post("/api/audio/volume", b"not json")[0], 400)

    def test_audio_is_reported_as_failure_when_g1_unreachable(self):
        self.helper.scenario = "g1_off"
        self.assertEqual(self._post("/api/audio/led", json.dumps({"r": 1, "g": 2, "b": 3}).encode())[0], 502)

    def test_index_html_is_served(self):
        html = urllib.request.urlopen("http://127.0.0.1:%d/" % self.port, timeout=5).read().decode()
        self.assertIn("G1 開発コンソール", html)


if __name__ == "__main__":
    unittest.main()


class CameraProxyTest(unittest.TestCase):
    """camera_stream.py の代わりに偽の上流を立て、/camera/<名前> の中継を確かめる。"""

    @classmethod
    def setUpClass(cls):
        from http.server import BaseHTTPRequestHandler

        class Upstream(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                self.wfile.write(b"--frame\r\nFRAMEDATA")

            def log_message(self, *args):
                pass

        cls.upstream = ThreadingHTTPServer(("127.0.0.1", 0), Upstream)
        helper = MockHelper()
        cls.server = ThreadingHTTPServer(
            ("127.0.0.1", 0),
            make_handler(helper, Monitor(helper), "http://127.0.0.1:%d" % cls.upstream.server_address[1]))
        cls.off = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(helper, Monitor(helper)))
        for srv in (cls.upstream, cls.server, cls.off):
            threading.Thread(target=srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        for srv in (cls.upstream, cls.server, cls.off):
            srv.shutdown()

    def test_known_camera_is_relayed_with_upstream_content_type(self):
        with urllib.request.urlopen("http://127.0.0.1:%d/camera/std" % self.server.server_address[1]) as r:
            self.assertIn("multipart/x-mixed-replace", r.headers["Content-Type"])
            self.assertIn(b"FRAMEDATA", r.read())

    def test_unknown_camera_is_404(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen("http://127.0.0.1:%d/camera/nope" % self.server.server_address[1])
        self.assertEqual(ctx.exception.code, 404)

    def test_without_camera_url_it_is_disabled(self):
        url = "http://127.0.0.1:%d/api/cameras" % self.off.server_address[1]
        with urllib.request.urlopen(url) as r:
            self.assertFalse(json.loads(r.read())["enabled"])
