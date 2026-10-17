"""タスクの定義と実行（段階 4、R-15）。実プロセスを起動して確かめる（ロボットには触れない）。
T4-1…T4-8 は docs/TEST_SPEC.md の番号に対応する。"""
import json
import os
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from g1console import tasks as tasks_mod  # noqa: E402
from g1console.tasks import DEF_KEYS, TaskError, TaskRunner, load_definitions  # noqa: E402
from server import MockHelper, Monitor, make_handler  # noqa: E402

CONSOLE = Path(__file__).resolve().parent.parent


def py(code):
    return [sys.executable, "-u", "-c", code]


def wait_until(cond, timeout=8.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


class Env:
    """一時ディレクトリに定義ファイルを置いて TaskRunner を作る。"""

    def __init__(self, defs, grace=0.3):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        for name, d in defs.items():
            (self.dir / (name + ".json")).write_text(json.dumps(d), encoding="utf-8")
        self.runner = TaskRunner(load_definitions(self.dir, self.dir), self.dir, term_grace_s=grace)

    def close(self):
        self.runner.shutdown()
        self.tmp.cleanup()

    def state(self, name):
        return next(t for t in self.runner.list()["tasks"] if t["name"] == name)

    def finished(self, name):
        return wait_until(lambda: self.state(name)["state"] == "idle" and self.state(name)["result"] is not None)

    def text(self, name):
        return [ln["text"] for ln in self.runner.logs(name)["lines"]]


class DefinitionTest(unittest.TestCase):
    def _load(self, name, raw):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / (name + ".json")).write_text(json.dumps(raw), encoding="utf-8")
            return load_definitions(d, d)

    def test_t4_1_shipped_definitions_load_and_use_argv_arrays(self):
        defs = load_definitions(CONSOLE / "tasks", CONSOLE)
        self.assertTrue(defs)
        for d in defs.values():
            self.assertIsInstance(d.argv, tuple)
            self.assertTrue(all(isinstance(a, str) for a in d.argv))
            self.assertTrue((CONSOLE / d.cwd).resolve().is_dir())

    def test_t4_1_invalid_definitions_are_rejected(self):
        bad = [
            ("shell-string", {"argv": "echo hi; rm -rf /"}),
            ("empty-argv", {"argv": []}),
            ("non-str-argv", {"argv": ["echo", 1]}),
            ("empty-arg", {"argv": ["echo", ""]}),
            ("unknown-key", {"argv": ["echo"], "shell": True}),
            ("abs-cwd", {"argv": ["echo"], "cwd": "/etc"}),
            ("escape-cwd", {"argv": ["echo"], "cwd": ".."}),
            ("escape-cwd2", {"argv": ["echo"], "cwd": "a/../../x"}),
            ("zero-timeout", {"argv": ["echo"], "timeout_s": 0}),
            ("huge-timeout", {"argv": ["echo"], "timeout_s": 99999}),
            ("bool-timeout", {"argv": ["echo"], "timeout_s": True}),
            ("bad-group", {"argv": ["echo"], "group": "A B"}),
            ("Bad_Name", {"argv": ["echo"]}),
        ]
        for name, raw in bad:
            with self.subTest(name):
                with self.assertRaises(ValueError):
                    self._load(name, raw)

    def test_broken_json_fails_fast(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "x.json").write_text("{not json", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_definitions(d, d)

    def test_defaults(self):
        d = self._load("x", {"argv": ["echo"]})["x"]
        self.assertEqual((d.cwd, d.group, d.title), (".", None, "x"))
        self.assertIn("argv", DEF_KEYS)


class RunnerTest(unittest.TestCase):
    def setUp(self):
        self.envs = []

    def tearDown(self):
        for e in self.envs:
            e.close()

    def env(self, defs, **kw):
        e = Env(defs, **kw)
        self.envs.append(e)
        return e

    def test_t4_1_unknown_name_is_refused(self):
        e = self.env({"a": {"argv": py("print(1)")}})
        for bad in ("nope", None, 5, ["a"], "../a", "A"):
            with self.assertRaises(TaskError) as cm:
                e.runner.start(bad)
            self.assertEqual(cm.exception.status, 404)
        with self.assertRaises(TaskError):
            e.runner.stop("nope")
        with self.assertRaises(TaskError):
            e.runner.logs("nope")

    def test_t4_2_shell_metacharacters_are_passed_literally(self):
        nasty = "; echo pwned $(id) `id` && touch should_not_exist | cat"
        e = self.env({"a": {"argv": py("import sys\nprint(sys.argv[1])") + [nasty]}})
        e.runner.start("a")
        self.assertTrue(e.finished("a"))
        self.assertEqual(e.text("a"), [nasty])
        self.assertFalse((e.dir / "should_not_exist").exists())
        self.assertEqual(e.state("a")["result"], "ok")

    def test_t4_3_same_group_is_exclusive(self):
        sleeper = py("import time\nprint('up')\ntime.sleep(30)")
        e = self.env({"a": {"argv": sleeper, "group": "g"}, "b": {"argv": sleeper, "group": "g"},
                      "c": {"argv": sleeper, "group": "other"}, "d": {"argv": sleeper}})
        e.runner.start("a")
        with self.assertRaises(TaskError) as cm:
            e.runner.start("b")
        self.assertEqual(cm.exception.status, 409)
        with self.assertRaises(TaskError) as cm:
            e.runner.start("a")  # 同じタスクの二重起動も拒否
        self.assertEqual(cm.exception.status, 409)
        e.runner.start("c")
        e.runner.start("d")  # グループなしは自分自身とだけ排他
        e.runner.stop("a")
        self.assertTrue(e.finished("a"))
        e.runner.start("b")  # 解放後は取れる
        self.assertEqual(e.state("b")["state"], "running")

    def test_t4_4_stop_sends_sigterm_to_the_group(self):
        e = self.env({"a": {"argv": py("import time\nprint('up')\ntime.sleep(30)")}})
        e.runner.start("a")
        self.assertTrue(wait_until(lambda: e.text("a") == ["up"]))
        self.assertEqual(e.runner.stop("a")["state"], "stopping")
        self.assertTrue(e.finished("a"))
        st = e.state("a")
        self.assertEqual((st["result"], st["exit_code"]), ("stopped", -15))
        with self.assertRaises(TaskError) as cm:
            e.runner.stop("a")  # 終わったものは止められない
        self.assertEqual(cm.exception.status, 409)

    def test_t4_4b_sigkill_when_sigterm_is_ignored(self):
        code = "import signal, time\nsignal.signal(signal.SIGTERM, signal.SIG_IGN)\nprint('ready')\ntime.sleep(30)"
        e = self.env({"a": {"argv": py(code)}}, grace=0.3)
        e.runner.start("a")
        self.assertTrue(wait_until(lambda: e.text("a") == ["ready"]))
        e.runner.stop("a")
        self.assertTrue(e.finished("a"))
        self.assertEqual((e.state("a")["result"], e.state("a")["exit_code"]), ("stopped", -9))

    def test_t4_4c_descendants_are_killed_too(self):
        code = ("import subprocess, sys, time\n"
                "p = subprocess.Popen(['sleep', '60'])\n"
                "print(p.pid)\n"
                "time.sleep(60)")
        e = self.env({"a": {"argv": py(code)}})
        e.runner.start("a")
        self.assertTrue(wait_until(lambda: len(e.text("a")) == 1))
        child = int(e.text("a")[0])
        os.kill(child, 0)  # 起動直後は生きている
        e.runner.stop("a")
        self.assertTrue(e.finished("a"))

        def gone():
            try:
                os.kill(child, 0)
            except ProcessLookupError:
                return True
            except PermissionError:
                return False
            return False
        self.assertTrue(wait_until(gone), "孫プロセスが残っている")

    def test_t4_5_timeout_stops_the_task(self):
        e = self.env({"a": {"argv": py("import time\ntime.sleep(30)"), "timeout_s": 1}})
        e.runner.start("a")
        self.assertTrue(e.finished("a"))
        st = e.state("a")
        self.assertEqual(st["result"], "timeout")
        self.assertIsNotNone(st["exit_code"])
        self.assertIsNotNone(st["ended_at"])

    def test_t4_5b_exit_code_and_result(self):
        e = self.env({"ok": {"argv": py("print('fine')")}, "ng": {"argv": py("import sys\nsys.exit(3)")}})
        e.runner.start("ok")
        e.runner.start("ng")
        self.assertTrue(e.finished("ok") and e.finished("ng"))
        self.assertEqual((e.state("ok")["result"], e.state("ok")["exit_code"]), ("ok", 0))
        self.assertEqual((e.state("ng")["result"], e.state("ng")["exit_code"]), ("failed", 3))

    def test_t4_6_ring_buffer_and_since(self):
        with mock.patch.object(tasks_mod, "RING_LINES", 5):
            e = self.env({"a": {"argv": py("for i in range(12): print('line', i)")}})
            e.runner.start("a")
            self.assertTrue(e.finished("a"))
        full = e.runner.logs("a", 0)
        self.assertEqual([ln["seq"] for ln in full["lines"]], [8, 9, 10, 11, 12])
        self.assertEqual((full["next"], full["truncated"]), (12, True))
        part = e.runner.logs("a", 10)
        self.assertEqual([ln["text"] for ln in part["lines"]], ["line 10", "line 11"])
        self.assertFalse(part["truncated"])
        self.assertEqual(e.runner.logs("a", 12)["lines"], [])  # 欠落も重複もなく、続きだけ

    def test_t4_6b_since_follows_a_running_task_without_gaps(self):
        e = self.env({"a": {"argv": py("import time\nfor i in range(6):\n    print(i)\n    time.sleep(0.05)")}})
        e.runner.start("a")
        seen, since = [], 0
        end = time.monotonic() + 8
        while time.monotonic() < end and len(seen) < 6:
            got = e.runner.logs("a", since)
            seen += [ln["text"] for ln in got["lines"]]
            since = got["next"]
            time.sleep(0.03)
        self.assertEqual(seen, [str(i) for i in range(6)])

    def test_long_lines_are_truncated(self):
        e = self.env({"a": {"argv": py("print('x' * 100000)")}})
        e.runner.start("a")
        self.assertTrue(e.finished("a"))
        self.assertEqual(len(e.text("a")[0]), tasks_mod.LINE_MAX_CHARS)

    def test_t4_7_start_failure_is_reported_and_does_not_wedge_the_task(self):
        e = self.env({"a": {"argv": ["/nonexistent/binary"]}})
        with self.assertRaises(TaskError) as cm:
            e.runner.start("a")
        self.assertEqual(cm.exception.status, 500)
        self.assertEqual(e.state("a")["state"], "idle")
        with self.assertRaises(TaskError) as cm:
            e.runner.start("a")  # 「実行中です」で固まらない
        self.assertEqual(cm.exception.status, 500)

    def test_t4_7b_rerun_replaces_the_previous_log(self):
        e = self.env({"a": {"argv": py("print('hello')")}})
        for _ in range(2):
            e.runner.start("a")
            self.assertTrue(e.finished("a"))
            self.assertEqual(e.text("a"), ["hello"])

    def test_t4_7c_shutdown_kills_everything_running(self):
        e = self.env({"a": {"argv": py("import time\nprint('up')\ntime.sleep(30)")}})
        e.runner.start("a")
        self.assertTrue(wait_until(lambda: e.text("a") == ["up"]))
        proc = e.runner._runs["a"].proc
        e.runner.shutdown()
        self.assertTrue(wait_until(lambda: proc.poll() is not None))


class HttpTaskTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.env = Env({"slow": {"argv": py("import time\nprint('tick')\ntime.sleep(30)"), "group": "g"},
                       "rival": {"argv": py("print('r')"), "group": "g"},
                       "quick": {"argv": py("print('q')")}})
        cls.marker = cls.env.dir / "pwned"
        helper = MockHelper()
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(helper, Monitor(helper), tasks=cls.env.runner))
        cls.port = cls.server.server_address[1]
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.env.close()
        cls.server.shutdown()
        cls.server.server_close()

    def req(self, method, path, body=None):
        data = json.dumps(body).encode() if body is not None and not isinstance(body, bytes) else body
        r = urllib.request.Request("http://127.0.0.1:%d%s" % (self.port, path), data=data, method=method)
        try:
            resp = urllib.request.urlopen(r, timeout=5)
            return resp.status, json.load(resp)
        except urllib.error.HTTPError as exc:
            return exc.code, json.load(exc)

    def test_t4_8_list_start_logs_stop_over_http(self):
        code, body = self.req("GET", "/api/tasks")
        self.assertEqual(code, 200)
        self.assertEqual({t["name"] for t in body["tasks"]}, {"slow", "rival", "quick"})
        self.assertTrue(all(t["state"] == "idle" for t in body["tasks"]))

        code, body = self.req("POST", "/api/tasks/start", {"name": "slow"})
        self.assertEqual((code, body["state"]), (200, "running"))
        self.assertEqual(self.req("POST", "/api/tasks/start", {"name": "slow"})[0], 409)
        self.assertEqual(self.req("POST", "/api/tasks/start", {"name": "rival"})[0], 409)  # 同じ排他グループ
        self.assertTrue(wait_until(lambda: self.req("GET", "/api/tasks/logs?name=slow")[1]["next"] >= 1))
        code, logs = self.req("GET", "/api/tasks/logs?name=slow&since=0")
        self.assertEqual((code, logs["lines"][0]["text"]), (200, "tick"))
        code, body = self.req("POST", "/api/tasks/stop", {"name": "slow"})
        self.assertEqual((code, body["state"]), (200, "stopping"))
        self.assertTrue(wait_until(lambda: self.env.state("slow")["result"] == "stopped"))
        self.assertEqual(self.req("POST", "/api/tasks/stop", {"name": "slow"})[0], 409)

    def test_t4_8b_bad_requests(self):
        self.assertEqual(self.req("POST", "/api/tasks/start", {"name": "nope"})[0], 404)
        self.assertEqual(self.req("POST", "/api/tasks/start", {})[0], 400)
        self.assertEqual(self.req("POST", "/api/tasks/start", {"name": 5})[0], 400)
        self.assertEqual(self.req("POST", "/api/tasks/start", b"not json")[0], 400)
        self.assertEqual(self.req("POST", "/api/tasks/start", b"[1]")[0], 400)
        self.assertEqual(self.req("GET", "/api/tasks/logs?name=nope")[0], 404)
        self.assertEqual(self.req("GET", "/api/tasks/logs")[0], 404)
        self.assertEqual(self.req("GET", "/api/tasks/logs?name=quick&since=x")[0], 400)
        self.assertEqual(self.req("GET", "/api/tasks/logs?name=quick&since=-1")[0], 400)

    def test_t4_1b_client_cannot_supply_argv_or_cwd(self):
        body = {"name": "quick", "argv": ["touch", str(self.marker)], "cwd": "/", "env": {"X": "1"}}
        self.assertEqual(self.req("POST", "/api/tasks/start", body)[0], 200)
        self.assertTrue(wait_until(lambda: self.env.state("quick")["result"] == "ok"))
        self.assertEqual(self.env.text("quick"), ["q"])
        self.assertFalse(self.marker.exists())


class HubShutdownTest(unittest.TestCase):
    def test_t4_9_sigterm_to_the_hub_stops_running_tasks(self):
        """systemd の停止（SIGTERM）でも、実行中のタスクが孤児として残らない。"""
        import signal
        import socket
        import subprocess
        env = Env({"a": {"argv": py("import os, time\nprint(os.getpid())\ntime.sleep(60)")}})
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        hub = subprocess.Popen([sys.executable, str(CONSOLE / "server.py"), "--mock", "--port", str(port),
                                "--tasks-dir", str(env.dir)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        base = "http://127.0.0.1:%d" % port
        try:
            def up():
                try:
                    return urllib.request.urlopen(base + "/api/tasks", timeout=1).status == 200
                except OSError:
                    return False
            self.assertTrue(wait_until(up, 15), "ハブが起動しない")
            urllib.request.urlopen(urllib.request.Request(base + "/api/tasks/start", b'{"name":"a"}', method="POST"), timeout=5)

            def pid():
                lines = json.load(urllib.request.urlopen(base + "/api/tasks/logs?name=a", timeout=2))["lines"]
                return int(lines[0]["text"]) if lines else None
            self.assertTrue(wait_until(lambda: pid() is not None))
            child = pid()
            os.kill(child, 0)
            hub.send_signal(signal.SIGTERM)
            hub.wait(timeout=15)

            def gone():
                try:
                    os.kill(child, 0)
                except ProcessLookupError:
                    return True
                return False
            self.assertTrue(wait_until(gone), "タスクが残っている")
        finally:
            if hub.poll() is None:
                hub.kill()
            env.close()


if __name__ == "__main__":
    unittest.main()
