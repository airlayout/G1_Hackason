import json
import struct
import sys
import threading
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "jetson"))

from g1console.camera_proc import SEP  # noqa: E402
from g1console.lidar_proc import MOCK_POINTS, MockLidarCtl, SshLidarCtl, forward_query  # noqa: E402
from g1console.settings import Settings, validate  # noqa: E402
from server import MockHelper, Monitor, make_handler  # noqa: E402

try:
    import numpy as np  # noqa: F401
    import lidar_stream
except ImportError:  # numpy の無い環境では間引きのテストだけ飛ばす
    lidar_stream = None


def fake_msg(points, step=22):
    """x,y,z,intensity (float32) @0/4/8/12 と、末尾に余白を持つ PointCloud2 もどき。"""
    data = b"".join(struct.pack("<ffff", *p) + b"\0" * (step - 16) for p in points)
    fields = [SimpleNamespace(name=n, offset=o) for n, o in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12), ("ring", 16))]
    return SimpleNamespace(fields=fields, point_step=step, data=list(data))


@unittest.skipIf(lidar_stream is None, "numpy が無い")
class DecimateTest(unittest.TestCase):
    def test_same_voxel_collapses_and_invalid_points_are_dropped(self):
        nan = float("nan")
        msg = fake_msg([(1.00, 1.00, 1.00, 5), (1.01, 1.02, 1.03, 6),  # 同じ 0.1 m ボクセル
                        (2.0, 0.0, 0.0, 7), (0.0, 0.0, 0.0, 8), (nan, 1.0, 1.0, 9)])
        arr, raw = lidar_stream.decimate(msg, 0.1, 100)
        self.assertEqual(raw, 5)
        self.assertEqual(arr.shape, (2, 4))
        self.assertEqual(arr.dtype.str, "<f4")

    def test_cap_keeps_at_most_max_points_in_16_bytes_each(self):
        msg = fake_msg([(i * 0.5, 0.0, 0.0, 1) for i in range(1, 200)])
        arr, _ = lidar_stream.decimate(msg, 0.1, 50)
        self.assertEqual(len(arr), 50)
        self.assertEqual(len(arr.tobytes()), 50 * 16)

    def test_missing_field_is_rejected(self):
        msg = fake_msg([(1, 1, 1, 1)])
        msg.fields = msg.fields[:2]
        with self.assertRaises(ValueError):
            lidar_stream.decimate(msg, 0.1, 10)

    def test_latest_reports_hz_and_age(self):
        latest = lidar_stream.Latest()
        self.assertIsNone(latest.status()["age_s"])
        arr = np.ones((1, 4), dtype="<f4")
        latest.put(arr, 10)
        latest.put(arr, 10)
        st = latest.status()
        self.assertEqual((st["frames"], st["points"], st["raw_points"]), (2, 1, 10))
        self.assertIsNotNone(st["age_s"])

    def test_thin_zero_voxel_keeps_all_points(self):
        arr, _ = lidar_stream.extract(fake_msg([(1.00, 1.00, 1.00, 5), (1.01, 1.02, 1.03, 6), (2, 0, 0, 7)]))
        self.assertEqual(len(lidar_stream.thin(arr, 0.0, 100)), 3)
        self.assertEqual(len(lidar_stream.thin(arr, 0.1, 100)), 2)

    def test_latest_frame_thins_per_request_and_caches(self):
        latest = lidar_stream.Latest()
        self.assertIsNone(latest.frame(0.0, 100)[0])
        latest.put(np.array([(i * 0.5, 0, 0, 1) for i in range(1, 200)], dtype="<f4"), 199)
        coarse = latest.frame(0.0, 100)
        full = latest.frame(0.0, 30000)
        self.assertEqual((coarse[2], full[2]), (100, 199))
        self.assertEqual(len(full[0]), 199 * 16)
        self.assertIs(latest.frame(0.0, 30000)[0], full[0])  # 同じ条件ならキャッシュ

    def test_parse_query_defaults_and_clamps(self):
        pq = lidar_stream.parse_query
        self.assertEqual(pq("/lidar"), (lidar_stream.DEFAULT_VOXEL_M, lidar_stream.DEFAULT_MAX_POINTS))
        self.assertEqual(pq("/lidar?voxel=0.05&max=10000"), (0.05, 10000))
        self.assertEqual(pq("/lidar?voxel=-3&max=99999999"), (0.0, lidar_stream.MAX_POINTS))
        self.assertEqual(pq("/lidar?voxel=abc&max=x"), (lidar_stream.DEFAULT_VOXEL_M, lidar_stream.DEFAULT_MAX_POINTS))


class ForwardQueryTest(unittest.TestCase):
    def test_only_valid_numbers_pass_through(self):
        self.assertEqual(forward_query("/lidar"), "")
        self.assertEqual(forward_query("/lidar?voxel=0.05&max=10000"), "?voxel=0.05&max=10000")
        self.assertEqual(forward_query("/lidar?voxel=0&max=30000"), "?voxel=0&max=30000")

    def test_unknown_invalid_and_out_of_range_are_dropped(self):
        self.assertEqual(forward_query("/lidar?x=../../etc&voxel=abc&max=5"), "")
        self.assertEqual(forward_query("/lidar?voxel=9&max=999999"), "")
        self.assertEqual(forward_query("/lidar?voxel=0.1&evil=1%0d%0a"), "?voxel=0.1")


class LidarCtlTest(unittest.TestCase):
    def test_mock_start_stop_and_frame(self):
        m = MockLidarCtl()
        self.assertIsNone(m.frame())
        self.assertTrue(m.call("start")["running"])
        body, n = m.frame()
        self.assertEqual((n, len(body)), (MOCK_POINTS, MOCK_POINTS * 16))
        self.assertFalse(m.call("stop")["running"])
        with self.assertRaises(ValueError):
            m.call("restart")

    def _ssh(self, out, port=8082):
        ctl = SshLidarCtl(lambda: ("u@h", ""), lambda: port)
        ctl.scripts = []
        ctl._run = lambda script: (ctl.scripts.append(script), out)[1]
        return ctl

    def test_start_is_one_ssh_with_port_and_status(self):
        ctl = self._ssh("[lidar] 起動しました (pid=7, port=8082)\n%s\n[lidar] 起動中 (pid=7)\n" % SEP)
        r = ctl.call("start")
        self.assertEqual(len(ctl.scripts), 1)
        self.assertIn("LIDAR_PORT=8082 ~/g1_console_lidar/lidar_ctl.sh start", ctl.scripts[0])
        self.assertTrue(r["running"] and r["pid"] == 7 and "起動しました" in r["message"])

    def test_status_does_not_start_or_stop(self):
        ctl = self._ssh("\n%s\n[lidar] 停止中\n" % SEP)
        r = ctl.call("status")
        self.assertFalse(r["running"])
        self.assertNotIn(" start", ctl.scripts[0])
        self.assertNotIn(" stop", ctl.scripts[0])

    def test_unknown_action_and_non_numeric_port_never_reach_ssh(self):
        ctl = self._ssh("")
        with self.assertRaises(ValueError):
            ctl.call("start; rm -rf ~")
        bad = self._ssh("", port="8082; reboot")
        with self.assertRaises(ValueError):
            bad.call("start")
        self.assertEqual(ctl.scripts + bad.scripts, [])


class LidarSettingsTest(unittest.TestCase):
    def test_default_and_range(self):
        self.assertEqual(validate({})["lidar_port"], 8082)
        for bad in ("0", "70000", "abc"):
            with self.assertRaises(ValueError):
                validate({"lidar_port": bad})

    def test_lidar_base_follows_host_and_port(self):
        s = Settings(None, {"jetson_host": "jetson.local"})
        self.assertEqual(s.lidar_base(), "http://jetson.local:8082")
        s.update({"lidar_port": 9000})
        self.assertEqual(s.lidar_base(), "http://jetson.local:9000")


class LidarServerTest(unittest.TestCase):
    """偽の配信元で /lidar の中継、模擬モードで起動・停止を確かめる。"""

    BODY = struct.pack("<ffff", 1, 2, 3, 4) * 3

    @classmethod
    def setUpClass(cls):
        body = cls.BODY

        seen = cls.seen_paths = []

        class Source(BaseHTTPRequestHandler):
            def do_GET(self):
                seen.append(self.path)
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("X-Lidar-Seq", "9")
                self.send_header("X-Lidar-Points", "3")
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        helper = MockHelper()
        cls.source = ThreadingHTTPServer(("127.0.0.1", 0), Source)
        cls.relay = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(
            helper, Monitor(helper), lidar_ctl=SimpleNamespace(call=lambda a: {"ok": True, "running": True, "pid": 1}),
            lidar_base="http://127.0.0.1:%d" % cls.source.server_address[1]))
        cls.dead = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(  # 誰も居ないポートを指す
            helper, Monitor(helper), lidar_ctl=SimpleNamespace(call=lambda a: {}), lidar_base="http://127.0.0.1:1"))
        cls.unset = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(
            helper, Monitor(helper), lidar_ctl=SimpleNamespace(call=lambda a: {})))
        cls.mock = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(helper, Monitor(helper), lidar_ctl=MockLidarCtl()))
        cls.servers = (cls.source, cls.relay, cls.dead, cls.unset, cls.mock)
        for srv in cls.servers:
            threading.Thread(target=srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        for srv in cls.servers:
            srv.shutdown()

    @staticmethod
    def url(srv, path):
        return "http://127.0.0.1:%d%s" % (srv.server_address[1], path)

    def test_frame_is_relayed_with_headers(self):
        with urllib.request.urlopen(self.url(self.relay, "/lidar")) as r:
            self.assertEqual(r.read(), self.BODY)
            self.assertEqual(r.headers["Content-Type"], "application/octet-stream")
            self.assertEqual((r.headers["X-Lidar-Seq"], r.headers["X-Lidar-Points"]), ("9", "3"))

    def test_query_is_validated_before_reaching_the_source(self):
        with urllib.request.urlopen(self.url(self.relay, "/lidar?voxel=0&max=30000&x=1")) as r:
            self.assertEqual(r.read(), self.BODY)
        self.assertEqual(self.seen_paths[-1], "/lidar?voxel=0&max=30000")
        urllib.request.urlopen(self.url(self.relay, "/lidar?voxel=zzz")).read()
        self.assertEqual(self.seen_paths[-1], "/lidar")

    def test_unreachable_source_is_502_with_reason(self):
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(self.url(self.dead, "/lidar"))
        self.assertEqual(cm.exception.code, 502)
        self.assertIn("lidar", json.loads(cm.exception.read())["error"])

    def test_unset_host_is_404(self):
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(self.url(self.unset, "/lidar"))
        self.assertEqual(cm.exception.code, 404)

    def test_mock_start_frame_stop(self):
        def call(method, path):
            req = urllib.request.Request(self.url(self.mock, path), method=method, data=b"" if method == "POST" else None)
            with urllib.request.urlopen(req) as r:
                return r.read(), r.headers

        with self.assertRaises(urllib.error.HTTPError) as cm:  # 停止中は点群を返さない
            call("GET", "/lidar")
        self.assertEqual(cm.exception.code, 502)
        self.assertFalse(json.loads(call("GET", "/api/lidar/status")[0])["running"])
        self.assertTrue(json.loads(call("POST", "/api/lidar/start")[0])["running"])
        body, heads = call("GET", "/lidar")
        self.assertEqual((len(body), heads["X-Lidar-Points"]), (MOCK_POINTS * 16, str(MOCK_POINTS)))
        self.assertFalse(json.loads(call("POST", "/api/lidar/stop")[0])["running"])


if __name__ == "__main__":
    unittest.main()
