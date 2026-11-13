from __future__ import annotations

import fcntl
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from Voice.task_actions import build_command, run_action


class TaskActionsTests(unittest.TestCase):
    def test_command_uses_registered_target_and_explicit_execute(self) -> None:
        command = build_command("press_down", Path(sys.executable), "test nic", execute=True)
        self.assertEqual(command[2:], ("--target", "down", "--button-set", "wheelchair", "--detector", "yolo",
                                       "--network-interface", "test nic", "--execute"))
        self.assertTrue(command[1].endswith("/Button_Press/J1-gen/real/auto_press.py"))

    def test_dry_run_does_not_spawn_or_require_robot_python(self) -> None:
        with patch("Voice.task_actions.subprocess.Popen") as spawn:
            result = run_action("press_bottle", Path("/does/not/exist"))
        spawn.assert_not_called()
        self.assertEqual(result.status, "dry_run")
        self.assertNotIn("--execute", result.command)

    def test_unknown_action_cannot_be_a_command(self) -> None:
        with self.assertRaises(ValueError):
            build_command("echo hello")

    def test_control_venv_python_symlink_is_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            python = Path(tmp) / "python"
            python.symlink_to(sys.executable)
            self.assertEqual(build_command("press_bottle", python)[0], str(python))

    def test_execute_waits_and_preserves_child_exit_code(self) -> None:
        with tempfile.TemporaryDirectory() as tmp, patch("Voice.task_actions.REPO_ROOT", Path(tmp)), patch("Voice.task_actions.subprocess.Popen") as spawn:
            spawn.return_value.wait.return_value = 7
            result = run_action("press_bottle", Path(sys.executable), execute=True)
            spawn.assert_called_once()
            self.assertIn("--execute", spawn.call_args.args[0])
        self.assertEqual(result.returncode, 7)
        self.assertEqual(result.status, "process_exited")

    def test_ctrl_c_waits_for_existing_task_cleanup(self) -> None:
        with tempfile.TemporaryDirectory() as tmp, patch("Voice.task_actions.REPO_ROOT", Path(tmp)), patch("Voice.task_actions.subprocess.Popen") as spawn:
            spawn.return_value.wait.side_effect = [KeyboardInterrupt, 0]
            result = run_action("press_bottle", Path(sys.executable), execute=True)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(spawn.return_value.wait.call_count, 2)

    def test_concurrent_execution_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp, patch("Voice.task_actions.REPO_ROOT", Path(tmp)), patch("Voice.task_actions.subprocess.Popen") as spawn:
            lock_path = Path(tmp) / "_local/voice/task.lock"
            lock_path.parent.mkdir(parents=True)
            with lock_path.open("a") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with self.assertRaisesRegex(RuntimeError, "実行中"):
                    run_action("press_bottle", Path(sys.executable), execute=True)
            spawn.assert_not_called()
