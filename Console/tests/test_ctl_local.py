"""カメラ・lidar 配信の起動・停止をコンソールから呼ぶ（--local）。既定は停止、画面で起動したときだけポートを使う。
T8-1…T8-5 は docs/TEST_SPEC.md の番号に対応する。実機にも DDS にも触れない（偽の python と setsid を PATH に置く）。"""
import json
import os
import stat
import subprocess
import sys
import tempfile
import threading
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from g1console.camera_proc import LOCAL_CTL_DIR, LocalCameraCtl, SshCameraCtl  # noqa: E402
from g1console.lidar_proc import LocalLidarCtl, SshLidarCtl  # noqa: E402
from server import MockHelper, Monitor, make_handler  # noqa: E402

JETSON = Path(__file__).resolve().parent.parent / "jetson"


def recorded(ctl_cls, action, *args):
    seen = []
    ctl = ctl_cls(*args)
    ctl._run = lambda script: (seen.append(script), "")[1]
    ctl.call(action)
    return seen[0]


class CommandLineTest(unittest.TestCase):
    def test_t8_1_local_calls_the_repo_scripts_not_a_copy_in_home(self):
        for cls, args, script in ((LocalCameraCtl, (lambda: 8081,), "camera_ctl.sh"), (LocalLidarCtl, (lambda: 8082,), "lidar_ctl.sh")):
            with self.subTest(script):
                cmd = recorded(cls, "start", *args)
                self.assertIn("%s/%s start" % (LOCAL_CTL_DIR, script), cmd)
                self.assertNotIn("g1_console_", cmd)
                self.assertTrue((JETSON / script).is_file())

    def test_t8_1_local_binds_loopback_and_keeps_state_outside_the_repo(self):
        for cls, args, env in ((LocalCameraCtl, (lambda: 8090,), "CAMERA"), (LocalLidarCtl, (lambda: 8091,), "LIDAR")):
            with self.subTest(env):
                cmd = recorded(cls, "start", *args)
                self.assertIn("%s_BIND=127.0.0.1" % env, cmd)
                self.assertIn("%s_PORT=%d" % (env, args[0]()), cmd)
                self.assertIn(".g1console/run", cmd)

    def test_t8_1_ssh_mode_keeps_its_own_directory_and_opens_to_the_network(self):
        cam = recorded(SshCameraCtl, "start", lambda: ("u@h", ""))
        self.assertIn("~/g1_console_camera/camera_ctl.sh start", cam)
        self.assertNotIn("BIND", cam)
        lid = recorded(SshLidarCtl, "start", lambda: ("u@h", ""), lambda: 8082)
        self.assertIn("LIDAR_PORT=8082 ~/g1_console_lidar/lidar_ctl.sh start", lid)
        self.assertNotIn("BIND", lid)

    def test_t8_1_camera_port_comes_from_the_settings_value(self):
        self.assertIn("CAMERA_PORT=8123", recorded(SshCameraCtl, "start", lambda: ("u@h", ""), lambda: 8123))
        self.assertNotIn("CAMERA_PORT", recorded(SshCameraCtl, "start", lambda: ("u@h", "")))

    def test_t8_1_only_fixed_words_and_integers_reach_the_shell(self):
        with self.assertRaises(ValueError):
            LocalCameraCtl(lambda: 8081).call("start; rm -rf /")
        with self.assertRaises(ValueError):
            LocalLidarCtl(lambda: 8082).call("x")


class ExecutableTest(unittest.TestCase):
    def test_t8_1_ctl_scripts_are_executable(self):
        # ハブは <dir>/<script> を直接実行する。git に実行権限が無いと clone した Jetson で黙って失敗する（実機で発生）
        for script in ("camera_ctl.sh", "lidar_ctl.sh"):
            with self.subTest(script):
                self.assertTrue(os.access(JETSON / script, os.X_OK))


class StreamBindTest(unittest.TestCase):
    def test_t8_2_streams_take_a_bind_address_and_use_it(self):
        for name in ("camera_stream.py", "lidar_stream.py"):
            src = (JETSON / name).read_text(encoding="utf-8")
            with self.subTest(name):
                self.assertIn('"--bind", default="0.0.0.0"', src)  # ssh 方式（別 PC のハブ）の既存の使い方は変えない
                self.assertIn("ThreadingHTTPServer((args.bind, args.port)", src)
                self.assertNotIn('ThreadingHTTPServer(("0.0.0.0"', src)


class ScriptRunTest(unittest.TestCase):
    """本物の ctl スクリプトを、偽の python（引数を記録して終わる）と偽の setsid で動かす。"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        d = Path(self.tmp.name)
        self.state, self.bin, self.args_file = d / "state", d / "bin", d / "args.txt"
        self.bin.mkdir()
        for name, body in (("fakepython", 'echo "$@" >> %s' % self.args_file), ("setsid", 'exec "$@"')):
            p = self.bin / name
            p.write_text("#!/bin/bash\n%s\n" % body, encoding="utf-8")
            p.chmod(p.stat().st_mode | stat.S_IXUSR)

    def tearDown(self):
        self.tmp.cleanup()

    def run_ctl(self, script, action, **env):
        e = {**os.environ, "PATH": "%s:%s" % (self.bin, os.environ["PATH"]), "G1_CTL_STATE_DIR": str(self.state),
             "CAMERA_PYTHON": str(self.bin / "fakepython"), "LIDAR_PYTHON": str(self.bin / "fakepython"),
             "UNITREE_SDK_DIR": self.tmp.name, **env}
        before = sorted(p.name for p in JETSON.iterdir())
        p = subprocess.run(["bash", str(JETSON / script), action], env=e, capture_output=True, text=True, timeout=30)
        self.assertEqual(sorted(p.name for p in JETSON.iterdir()), before)  # pid・ログをリポジトリに残さない
        return p.stdout + p.stderr

    def started_args(self):
        return self.args_file.read_text(encoding="utf-8") if self.args_file.exists() else ""

    def test_t8_3_lidar_start_passes_port_and_bind(self):
        self.run_ctl("lidar_ctl.sh", "start", LIDAR_PORT="8099", LIDAR_BIND="127.0.0.1")
        args = self.started_args()
        self.assertIn("--port 8099", args)
        self.assertIn("--bind 127.0.0.1", args)

    def test_t8_3_camera_start_passes_port_and_bind(self):
        self.run_ctl("camera_ctl.sh", "start", CAMERA_PORT="8098", CAMERA_BIND="127.0.0.1")
        args = self.started_args()
        self.assertIn("--port 8098", args)
        self.assertIn("--bind 127.0.0.1", args)

    def test_t8_3_defaults_keep_the_old_network_wide_behaviour(self):
        self.run_ctl("lidar_ctl.sh", "start")
        self.assertIn("--port 8082 --bind 0.0.0.0", self.started_args())

    def test_t8_3_state_goes_to_the_given_directory(self):
        self.run_ctl("lidar_ctl.sh", "start")
        self.assertTrue(self.state.is_dir())
        self.assertTrue((self.state / "lidar_stream.log").exists())

    def test_t8_3_status_and_stop_start_nothing(self):
        for script in ("camera_ctl.sh", "lidar_ctl.sh"):
            for action in ("status", "stop"):
                with self.subTest((script, action)):
                    out = self.run_ctl(script, action)
                    self.assertIn("停止中" if action == "status" else "起動していません", out)
        self.assertEqual(self.started_args(), "")


class DefaultStoppedTest(unittest.TestCase):
    """ハブは自分からは起動しない。起動は画面の POST /api/{camera,lidar}/start だけ。"""

    @classmethod
    def setUpClass(cls):
        cls.calls = {"camera": [], "lidar": []}

        def ctl(name):
            def call(action):
                cls.calls[name].append(action)
                return {"ok": True, "running": action == "start", "pid": None, "holders": [], "foreign": [], "message": ""}
            return type("Ctl", (), {"call": staticmethod(call), "backend": "test"})()

        helper = MockHelper()
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(
            helper, Monitor(helper), camera_ctl=ctl("camera"), lidar_ctl=ctl("lidar")))
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        cls.base = "http://127.0.0.1:%d" % cls.srv.server_address[1]

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()

    def get(self, path):
        return urllib.request.urlopen(self.base + path, timeout=5).read()

    def post(self, path):
        return json.load(urllib.request.urlopen(urllib.request.Request(self.base + path, data=b"{}", method="POST"), timeout=5))

    def test_t8_4_nothing_is_called_until_asked(self):
        self.assertEqual(self.calls, {"camera": [], "lidar": []})

    def test_t8_4_reading_pages_and_status_never_starts(self):
        for path in ("/", "/api/state", "/api/cameras", "/api/camera/status", "/api/lidar/status"):
            self.get(path)
        self.assertEqual(set(self.calls["camera"] + self.calls["lidar"]), {"status"})

    def test_t8_4_start_and_stop_happen_only_by_post(self):
        self.post("/api/lidar/start")
        self.post("/api/lidar/stop")
        self.post("/api/camera/start")
        self.post("/api/camera/stop")
        self.assertEqual(self.calls["lidar"][-2:], ["start", "stop"])
        self.assertEqual(self.calls["camera"][-2:], ["start", "stop"])

    def test_t8_5_start_buttons_ask_first_in_the_page(self):
        from web_source import source
        html = source()
        self.assertIn("guard('camera'", html)
        self.assertIn("guard('lidar'", html)


if __name__ == "__main__":
    unittest.main()
