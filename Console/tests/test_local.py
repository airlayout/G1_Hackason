"""Jetson 上でのローカル実行（--local）。ssh を使わないことと、既存の ssh 版が壊れていないことを確かめる。"""
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import server  # noqa: E402
from g1console import camera_proc  # noqa: E402
from g1console.camera_proc import LocalCameraCtl, SshCameraCtl, resolve_host  # noqa: E402
from g1console.lidar_proc import LocalLidarCtl, SshLidarCtl  # noqa: E402

ECHO_HELPER = (
    "import json,sys\n"
    "print(json.dumps({'ready': True}), flush=True)\n"
    "for line in sys.stdin:\n"
    "    print(json.dumps({'ok': True, 'echo': json.loads(line), 'cwd': __import__('os').getcwd()}), flush=True)\n"
)


class ResolveHostTest(unittest.TestCase):
    def test_ip_literal_does_not_call_ssh(self):
        with mock.patch.object(camera_proc.subprocess, "run") as run:
            self.assertEqual(resolve_host("127.0.0.1"), "127.0.0.1")
            self.assertEqual(resolve_host("192.168.123.164"), "192.168.123.164")
            self.assertEqual(resolve_host("::1"), "::1")
        run.assert_not_called()

    def test_alias_still_asks_ssh(self):
        done = mock.Mock(stdout="hostname 10.0.0.9\n")
        with mock.patch.object(camera_proc.subprocess, "run", return_value=done) as run:
            self.assertEqual(resolve_host("local-test-alias"), "10.0.0.9")
        run.assert_called_once()


class LocalCtlTest(unittest.TestCase):
    def test_backend_names(self):
        self.assertEqual(LocalCameraCtl.backend, "local")
        self.assertEqual(LocalLidarCtl.backend, "local")
        self.assertEqual(SshCameraCtl.backend, "ssh")

    def test_run_executes_locally_without_ssh(self):
        self.assertEqual(LocalCameraCtl()._run("echo hi").strip(), "hi")

    def test_run_never_invokes_ssh(self):
        with mock.patch.object(camera_proc.subprocess, "run") as run:
            run.return_value = mock.Mock(stdout="")
            LocalCameraCtl()._run("true")
        self.assertEqual(run.call_args[0][0][0], "bash")

    def test_camera_call_splits_three_parts(self):
        ctl = LocalCameraCtl()
        script_out = "msg\n%s\n[camera] 起動中 (pid=77)\n%s\n/dev/video0\t77\tu\tpython -u camera_stream.py\n" % (
            camera_proc.SEP, camera_proc.SEP)
        with mock.patch.object(LocalCameraCtl, "_run", return_value=script_out):
            res = ctl.call("status")
        self.assertTrue(res["running"])
        self.assertEqual(res["pid"], 77)
        self.assertEqual(len(res["holders"]), 1)

    def test_lidar_uses_port(self):
        seen = []
        ctl = LocalLidarCtl(lambda: 8082)
        with mock.patch.object(LocalLidarCtl, "_run", side_effect=lambda s: seen.append(s) or ""):
            ctl.call("start")
        self.assertIn("LIDAR_PORT=8082", seen[0])

    def test_unknown_action_rejected(self):
        with self.assertRaises(ValueError):
            LocalCameraCtl().call("rm -rf /")
        with self.assertRaises(ValueError):
            LocalLidarCtl(lambda: 8082).call("x")

    def test_lidar_is_ssh_subclass_so_existing_paths_keep_working(self):
        self.assertTrue(issubclass(LocalLidarCtl, SshLidarCtl))


class LocalHelperTest(unittest.TestCase):
    def test_backend_and_configured(self):
        helper = server.LocalHelper(sdk_dir="/nonexistent")
        self.assertEqual(helper.backend, "local")
        self.assertEqual(helper._host, server.LOCAL_HOST)

    def test_reconfigure_keeps_local(self):
        helper = server.LocalHelper(sdk_dir="/nonexistent")
        helper.reconfigure("example.invalid", "key")
        self.assertEqual(helper.backend, "local")
        self.assertEqual(helper._host, server.LOCAL_HOST)

    def test_command_is_local_python_in_sdk_dir(self):
        helper = server.LocalHelper(sdk_dir="/opt/sdk")
        cmd, cwd = helper._command("print(1)")
        self.assertEqual(cmd, [sys.executable, "-u", "-c", "print(1)"])
        self.assertEqual(cwd, "/opt/sdk")
        self.assertNotIn("ssh", cmd[0])

    def test_sdk_dir_from_env(self):
        with mock.patch.dict(os.environ, {"UNITREE_SDK_DIR": "/env/sdk"}):
            self.assertEqual(server.LocalHelper()._sdk_dir.as_posix(), "/env/sdk")

    def test_call_round_trip_through_child_process(self):
        with tempfile.TemporaryDirectory() as tmp:
            helper = server.LocalHelper(sdk_dir=tmp)
            with mock.patch.object(server.LocalHelper, "_command",
                                   lambda self, src: ([sys.executable, "-u", "-c", ECHO_HELPER], tmp)):
                res = helper.call({"op": "status"})
                res2 = helper.call({"op": "status"})
            helper._kill()
        self.assertTrue(res["ok"])
        self.assertEqual(res["echo"], {"op": "status"})
        self.assertEqual(os.path.realpath(res["cwd"]), os.path.realpath(tmp))
        self.assertTrue(res2["ok"])

    def test_missing_sdk_dir_reports_offline_not_crash(self):
        helper = server.LocalHelper(sdk_dir="/definitely/not/here")
        res = helper.call({"op": "status"})
        self.assertFalse(res["ok"])
        self.assertTrue(res["offline"])


class DetailTextTest(unittest.TestCase):
    def test_ok_detail_has_no_ssh_wording(self):
        links = server._describe_links({"ok": True, "checkmode_code": 0, "fsm_code": 0, "fsm_id": 500, "service": "sport_mode"})
        self.assertNotIn("ssh", links["jetson"]["detail"])


if __name__ == "__main__":
    unittest.main()
