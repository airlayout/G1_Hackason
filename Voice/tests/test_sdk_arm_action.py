from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

from Entame.real import arm_action_tool

ROOT = Path(__file__).resolve().parents[2]


class SDKArmActionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        lock = patch.object(arm_action_tool, "LOCK_DIRECTORY", Path(self.tmp.name))
        lock.start()
        self.addCleanup(lock.stop)
        self.loco = Mock(spec=["GetFsmId"])
        self.loco.GetFsmId.return_value = (0, 500)
        self.arm = Mock(spec=["ExecuteAction"])
        self.arm.ExecuteAction.return_value = 0
        self.connect = Mock(return_value=(self.loco, self.arm))
        self.wait = Mock()

    def run_preset(self, **kwargs):
        return arm_action_tool.run_preset("high wave", "eth0", connect=self.connect, wait=self.wait, **kwargs)

    def test_dry_run_never_connects_or_waits(self) -> None:
        result = self.run_preset()
        self.assertEqual(result["status"], "dry_run")
        self.assertEqual(result["action_id"], 26)
        self.connect.assert_not_called()
        self.wait.assert_not_called()

    def test_execute_uses_fixed_action_and_preserves_control_mode(self) -> None:
        result = self.run_preset(execute=True)
        self.connect.assert_called_once_with("eth0")
        self.loco.GetFsmId.assert_called_once_with()
        self.arm.ExecuteAction.assert_called_once_with(26)
        self.wait.assert_called_once_with(5.0)
        self.assertEqual(result["status"], "accepted")
        self.assertFalse(result["motion_completed"])

    def test_unregistered_preset_never_connects(self) -> None:
        with self.assertRaises(ValueError):
            arm_action_tool.run_preset("clap", "eth0", execute=True, connect=self.connect)
        self.connect.assert_not_called()

    def test_unavailable_or_unsupported_mode_never_executes(self) -> None:
        for response in ((1, None), (0, 0), (0, 3), (0, True), (0, 500.0)):
            with self.subTest(response=response):
                self.loco.GetFsmId.return_value = response
                with self.assertRaises(RuntimeError):
                    self.run_preset(execute=True)
        self.arm.ExecuteAction.assert_not_called()
        self.wait.assert_not_called()

    def test_sdk_rejection_does_not_wait_and_releases_lock(self) -> None:
        self.arm.ExecuteAction.return_value = 7404
        with self.assertRaisesRegex(RuntimeError, "7404"):
            self.run_preset(execute=True)
        self.wait.assert_not_called()
        self.arm.ExecuteAction.return_value = 0
        self.assertEqual(self.run_preset(execute=True)["status"], "accepted")

    def test_overlapping_invocation_is_rejected_before_dds(self) -> None:
        with arm_action_tool.arm_lock():
            with self.assertRaisesRegex(RuntimeError, "実行中"):
                self.run_preset(execute=True)
        self.connect.assert_not_called()

    def test_interrupt_returns_failure_and_releases_lock(self) -> None:
        with patch.object(arm_action_tool, "connect_sdk", self.connect), \
                patch.object(arm_action_tool.time, "sleep", side_effect=KeyboardInterrupt), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(arm_action_tool.main(["--execute"]), 130)
        self.assertEqual(json.loads(output.getvalue())["status"], "interrupted")
        with arm_action_tool.arm_lock():
            pass

    def test_cli_default_requires_no_sdk(self) -> None:
        process = subprocess.run([sys.executable, str(ROOT / "Entame/real/arm_action_tool.py")],
                                 capture_output=True, text=True, timeout=3)
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertEqual(json.loads(process.stdout)["status"], "dry_run")
