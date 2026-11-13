from __future__ import annotations

import stat
import json
import subprocess
import sys
import struct
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from Voice.supervision import HealthLease, HeartbeatWire
from Voice.tool_client import ToolClient
from Voice.tool_runtime import ToolResult, ToolRuntime, ToolSpec
from Voice.tool_server import ToolServer


class ToolIPCTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "tools.sock"
        self.backend = Mock(return_value=ToolResult("example", True, "completed", "ok"))
        item = ToolSpec("example", "example", "fake", {}, {"type": "object", "additionalProperties": False})
        self.runtime = ToolRuntime((item,), {"fake": self.backend})
        self.server = ToolServer(self.path, self.runtime, execute=True)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.02})
        self.thread.start()
        self.addCleanup(self.close_server)
        self.client = ToolClient(self.path)

    def close_server(self) -> None:
        self.server.shutdown()
        self.thread.join(2)
        self.server.server_close()

    def test_local_ipc_preserves_structured_result_and_socket_permissions(self) -> None:
        self.assertEqual(stat.S_IMODE(self.path.stat().st_mode), 0o600)
        self.assertEqual(self.client.request("list")["tools"][0]["name"], "example")
        self.assertEqual(self.client.call("example", {}, execute=True)["status"], "completed")
        self.backend.assert_called_once()

    def test_invalid_arguments_and_expired_request_never_execute(self) -> None:
        self.assertFalse(self.client.call("example", {"extra": 1}, execute=True)["success"])
        result = self.client.request("call", tool="example", arguments={}, execute=True, expires_at=time.monotonic() - 1)
        self.assertEqual(result["status"], "expired")
        self.backend.assert_not_called()

    def test_both_client_and_server_must_enable_execution(self) -> None:
        self.client.call("example", {})
        self.server.execute_enabled = False
        self.assertEqual(self.client.call("example", {}, execute=True)["status"], "execution_disabled")
        self.backend.assert_not_called()

    def test_onboard_execution_requires_live_lease(self) -> None:
        self.server.onboard_heartbeat = True
        self.server.execute_enabled = False  # UDP を送らず lease 判定を確認する。
        self.assertTrue(self.client.health("session", True)["success"])
        self.client.health("session", False)
        self.server.execute_enabled = True
        self.assertEqual(self.client.call("example", {}, execute=True)["status"], "unhealthy")
        self.backend.assert_not_called()

    def test_failed_bind_preserves_original_error_and_other_socket(self) -> None:
        other = self.path.parent / "failed.sock"
        with patch.object(ToolServer, "server_bind", side_effect=OSError("bind failed")):
            with self.assertRaisesRegex(OSError, "bind failed"):
                ToolServer(other, self.runtime)
        self.assertTrue(self.path.exists())


class SupervisionTests(unittest.TestCase):
    def test_lease_expires_and_stale_report_does_not_restore_it(self) -> None:
        now = [10.0]
        lease = HealthLease(clock=lambda: now[0])
        self.assertFalse(lease.alive())
        self.assertTrue(lease.update("one", True, 10))
        self.assertTrue(lease.alive())
        self.assertFalse(lease.update("other", True, 10))
        now[0] = 12
        self.assertFalse(lease.alive())
        self.assertFalse(lease.update("one", True, 10))
        self.assertTrue(lease.update("two", True, 12))
        self.assertTrue(lease.update("two", False, 12))
        self.assertFalse(lease.alive())

    def test_wire_matches_existing_cpp_protocol(self) -> None:
        wire = HeartbeatWire()
        first, second = wire.next(), wire.next()
        self.assertEqual(len(first), 29)
        magic, version, session, sequence, timestamp = struct.unpack("<IBQQQ", first)
        self.assertEqual((magic, version, session, sequence), (0x47314842, 1, wire.session, 1))
        self.assertEqual(struct.unpack("<IBQQQ", second)[3], 2)
        self.assertGreater(timestamp, 0)


class ToolServerProcessTests(unittest.TestCase):
    def test_separate_process_can_execute_and_cleans_socket_on_sigterm(self) -> None:
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            path = directory / "tools.sock"
            config = directory / "tools.json"
            config.write_text(json.dumps({"tools": [{
                "name": "echo", "backend": "command",
                "config": {"argv": [sys.executable, "-c", "print('ok')"]},
                "input_schema": {"type": "object", "additionalProperties": False},
            }]}))
            process = subprocess.Popen([sys.executable, str(root / "Voice/tool_server.py"), "--config", str(config),
                                        "--socket", str(path), "--execute"], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            try:
                deadline = time.monotonic() + 3
                while not path.exists() and process.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(path.exists())
                result = ToolClient(path).call("echo", {}, execute=True)
                self.assertTrue(result["success"])
                self.assertEqual(result["data"]["output"].strip(), "ok")
                process.terminate()
                process.wait(timeout=4)
                self.assertEqual(process.returncode, 130)
                self.assertFalse(path.exists())
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                process.stderr.close()
