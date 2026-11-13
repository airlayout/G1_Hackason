from __future__ import annotations

import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from Voice.tool_runtime import CommandBackend, ROSServiceBackend, ToolResult, ToolRuntime, ToolSpec, command_backend


def spec(name: str = "test", backend: str = "fake", **kwargs) -> ToolSpec:
    return ToolSpec(name, "test tool", backend, kwargs.pop("config", {}),
                    kwargs.pop("schema", {"type": "object", "additionalProperties": False}), **kwargs)


class ToolRuntimeTests(unittest.TestCase):
    def test_dry_run_and_invalid_requests_never_reach_backend(self) -> None:
        backend = Mock()
        runtime = ToolRuntime((spec(),), {"fake": backend})
        self.assertEqual(runtime.call("test", {}).status, "dry_run")
        self.assertFalse(runtime.call("missing", {}, execute=True).success)
        self.assertFalse(runtime.call("test", {"surprise": 1}, execute=True).success)
        backend.assert_not_called()

    def test_new_backend_and_tool_require_no_voice_specific_code(self) -> None:
        schema = {"type": "object", "properties": {"count": {"type": "integer", "minimum": 1}},
                  "required": ["count"], "additionalProperties": False}
        backend = lambda item, args: ToolResult(item.name, True, "completed", "ok", args)
        runtime = ToolRuntime((spec("new_tool", "custom", schema=schema),), {"custom": backend})
        self.assertEqual(runtime.call("new_tool", {"count": 2}, execute=True).data, {"count": 2})
        self.assertFalse(runtime.call("new_tool", {"count": "2"}, execute=True).success)

    def test_exclusive_group_returns_busy_instead_of_queueing(self) -> None:
        started, release = threading.Event(), threading.Event()
        def backend(item, args):
            started.set()
            release.wait(2)
            return ToolResult(item.name, True, "completed", "ok")
        runtime = ToolRuntime((spec("one"), spec("two")), {"fake": backend})
        worker = threading.Thread(target=runtime.call, args=("one", {}), kwargs={"execute": True})
        worker.start()
        try:
            self.assertTrue(started.wait(1))
            self.assertEqual(runtime.call("two", {}, execute=True).status, "busy")
        finally:
            release.set()
            worker.join(2)

    def test_command_arguments_are_not_interpreted_by_shell(self) -> None:
        item = spec(backend="command", config={"argv": [sys.executable, "-c", "import sys; print(sys.argv[1])", "{text}"]})
        text = "$(touch should-not-exist); literal"
        result = command_backend(item, {"text": text})
        self.assertTrue(result.success)
        self.assertEqual(result.data["output"].strip(), text)

    def test_command_timeout_is_failure(self) -> None:
        item = spec(backend="command", timeout_s=0.05, config={"argv": [sys.executable, "-c", "import time; time.sleep(10)"]})
        result = command_backend(item, {})
        self.assertFalse(result.success)
        self.assertEqual(result.status, "timeout")

    def test_server_shutdown_stops_active_command(self) -> None:
        backend = CommandBackend()
        item = spec(backend="command", config={"argv": [sys.executable, "-c", "import time; time.sleep(10)"]})
        result = []
        worker = threading.Thread(target=lambda: result.append(backend(item, {})))
        worker.start()
        try:
            deadline = time.monotonic() + 2
            while not backend.processes and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(backend.processes)
            backend.close()
            worker.join(2)
            self.assertFalse(worker.is_alive())
            self.assertFalse(result[0].success)
        finally:
            backend.close()
            worker.join(2)


class ROSBackendTests(unittest.TestCase):
    def setUp(self) -> None:
        self.backend = ROSServiceBackend()
        self.node = Mock()
        self.backend.node = self.node
        self.client = self.node.create_client.return_value
        self.client.wait_for_service.return_value = True
        self.future = self.client.call_async.return_value
        self.future.done.return_value = True
        self.response = {"success": False, "message": "bridge=READY"}
        self.rclpy = Mock()
        utility = SimpleNamespace(get_service=lambda _: SimpleNamespace(Request=lambda: SimpleNamespace()), get_message=lambda _: object)
        modules = {
            "rclpy": self.rclpy,
            "rosidl_runtime_py": Mock(),
            "rosidl_runtime_py.utilities": utility,
            "rosidl_runtime_py.convert": SimpleNamespace(message_to_ordereddict=lambda _: dict(self.response)),
            "rosidl_runtime_py.set_message": SimpleNamespace(set_message_fields=lambda request, fields: None),
        }
        self.patcher = patch.dict(sys.modules, modules)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.item = spec(backend="ros_service", config={"service": "/g1/patrol/start", "service_type": "std_srvs/srv/Trigger", "success_field": "success"})

    def test_service_success_false_is_rejection(self) -> None:
        result = self.backend(self.item, {})
        self.assertFalse(result.success)
        self.assertEqual(result.status, "rejected")
        self.assertIn("READY", result.message)

    def test_missing_service_is_unavailable(self) -> None:
        self.client.wait_for_service.return_value = False
        self.assertEqual(self.backend(self.item, {}).status, "unavailable")

    def test_uncompleted_future_is_timeout(self) -> None:
        self.future.done.return_value = False
        self.assertEqual(self.backend(self.item, {}).status, "timeout")
        self.future.cancel.assert_called_once()

    def test_acknowledgment_and_confirmed_state_are_both_required(self) -> None:
        self.response["success"] = True
        self.item.config["confirm"] = {"topic": "/status", "message_type": "std_msgs/msg/String", "json_field": "state", "allowed_values": ["RUNNING"]}
        def spin(node, **kwargs):
            callback = self.node.create_subscription.call_args.args[2]
            callback(SimpleNamespace(data='{"state":"RUNNING"}'))
        self.rclpy.spin_once.side_effect = spin
        result = self.backend(self.item, {})
        self.assertTrue(result.success)
        self.assertEqual(result.data["confirmed_state"], "RUNNING")
        self.node.destroy_subscription.assert_called_once()
