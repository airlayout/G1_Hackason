from __future__ import annotations

import ipaddress
import json
import os
import signal
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from g1_mapping.doctor import _host_checks, diagnose
from g1_mapping.native import NativeRuntime, _identity
from g1_mapping.runtime import get_runtime
from g1_mapping.session import SessionManager
from test_mapping import make_settings


class RuntimeDiagnosticsTest(unittest.TestCase):
    def test_missing_docker_is_a_diagnostic_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = make_settings(Path(directory))
            with patch("g1_mapping.doctor.shutil.which", return_value=None):
                report = diagnose(settings, requested_backend="sim")
            self.assertFalse(report.ready)
            self.assertTrue(
                any(
                    check.name == "host.docker" and check.status == "FAIL"
                    for check in report.checks
                )
            )

    def test_mock_does_not_call_external_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch(
                "g1_mapping.doctor._run",
                side_effect=AssertionError("unexpected subprocess"),
            ):
                report = diagnose(
                    make_settings(Path(directory)), requested_backend="auto", mock=True
                )
            self.assertTrue(report.ready)

    def test_native_selection_preserves_the_physical_interface(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = replace(
                make_settings(Path(directory)), runtime="native", g1_iface="wired0"
            )
            runtime = get_runtime(settings)
            self.assertIsInstance(runtime, NativeRuntime)
            self.assertIn('name="wired0"', runtime.environment("raw")["CYCLONEDDS_URI"])
            self.assertIn('name="lo"', runtime.environment("sim")["CYCLONEDDS_URI"])

    def test_ssh_transport_keeps_mapping_dds_on_loopback(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = replace(
                make_settings(Path(directory)),
                runtime="native",
                transport="ssh",
                g1_iface="wifi0",
                g1_host="192.168.0.82",
                ros_domain_id=46,
            )
            runtime = get_runtime(settings)
            env = runtime.environment("raw")
            self.assertIn('name="lo"', env["CYCLONEDDS_URI"])
            self.assertNotIn('name="wifi0"', env["CYCLONEDDS_URI"])
            self.assertEqual(env["ROS_DOMAIN_ID"], "46")
            self.assertIn("unitree@192.168.0.82", runtime.sensor_command())
            tailscale = get_runtime(replace(settings, ssh_host="100.78.135.14"))
            self.assertIn("unitree@100.78.135.14", tailscale.sensor_command())
            self.assertEqual(tailscale.settings.camera_host, settings.camera_host)

    def test_ssh_diagnosis_accepts_wifi_subnet_and_rejects_a_wired_route(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = replace(
                make_settings(Path(directory)),
                runtime="native",
                transport="ssh",
                g1_iface="lo",
                g1_host="192.168.0.82",
                camera_enabled=False,
                camera_required=False,
            )
            for device, expected in (("lo", "PASS"), ("wired0", "FAIL")):
                route = subprocess.CompletedProcess(
                    [], 0, json.dumps([{"dev": device, "prefsrc": "192.168.0.167"}]), ""
                )
                with (
                    self.subTest(device=device),
                    patch(
                        "g1_mapping.doctor._interface_addresses",
                        return_value=[ipaddress.ip_interface("192.168.0.167/24")],
                    ),
                    patch("g1_mapping.doctor._run", return_value=route),
                    patch("g1_mapping.doctor.get_runtime") as runtime,
                ):
                    runtime.return_value.run.return_value = subprocess.CompletedProcess(
                        [], 0, "CycloneDDS", ""
                    )
                    checks = _host_checks(settings, mock=False)
                self.assertEqual(
                    next(
                        check.status for check in checks if check.name == "host.network"
                    ),
                    expected,
                )


class NativeShutdownTest(unittest.TestCase):
    def test_save_failure_still_gracefully_closes_recording_process(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = replace(make_settings(Path(directory)), runtime="native")
            session = SessionManager(settings).create(
                name="shutdown", backend="raw", diagnostic={}
            )
            marker = session.directory / "closed"
            child = subprocess.Popen(
                [
                    sys.executable,
                    "-u",
                    "-c",
                    (
                        "import signal,sys,pathlib; "
                        "signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); "
                        "print('ready', flush=True); "
                        "\ntry: signal.pause()\nfinally: pathlib.Path(sys.argv[1]).touch()"
                    ),
                    str(marker),
                ],
                stdout=subprocess.PIPE,
                text=True,
                start_new_session=True,
            )
            try:
                self.assertEqual(child.stdout.readline().strip(), "ready")
                NativeRuntime._process_file(session).write_text(
                    json.dumps(
                        {
                            "recorder": {
                                "pid": child.pid,
                                "identity": _identity(child.pid),
                            },
                        }
                    )
                )
                runtime = NativeRuntime(settings)
                with (
                    patch.object(
                        runtime, "run", side_effect=RuntimeError("save failed")
                    ),
                    self.assertRaisesRegex(RuntimeError, "save failed"),
                ):
                    runtime.save_and_stop(session)
                self.assertEqual(child.wait(timeout=5), 0)
                self.assertTrue(marker.exists())
            finally:
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    child.wait(timeout=5)
                child.stdout.close()

    def test_mismatched_process_identity_is_never_signalled(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = replace(make_settings(Path(directory)), runtime="native")
            session = SessionManager(settings).create(
                name="reused", backend="raw", diagnostic={}
            )
            NativeRuntime._process_file(session).write_text(
                json.dumps(
                    {
                        "recorder": {
                            "pid": os.getpid(),
                            "identity": "not-this-process",
                        },
                    }
                )
            )
            with patch("g1_mapping.native.os.killpg") as kill:
                NativeRuntime(settings)._stop_processes(session)
            kill.assert_not_called()


if __name__ == "__main__":
    unittest.main()
