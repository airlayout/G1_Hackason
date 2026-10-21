from __future__ import annotations

import json
import io
from contextlib import redirect_stdout
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import subprocess

from g1_mapping.cli import _parser, _rebuild, _stop
from g1_mapping.compose import ComposeRuntime
from g1_mapping.session import SessionManager
from g1_mapping.pcd import validate_session
from test_mapping import make_settings


class OccupancyWorkflowTest(unittest.TestCase):
    def test_raw_rebuild_defaults_to_occupancy_and_passes_overrides(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = make_settings(Path(directory))
            session = SessionManager(settings).create(name="raw", backend="raw", diagnostic={})
            args = _parser().parse_args(["rebuild", session.session_id, "--voxel", "0.02"])
            with patch.object(ComposeRuntime, "rebuild_occupancy") as rebuild, \
                    patch("g1_mapping.cli.rebuild_map") as accumulate:
                self.assertEqual(_rebuild(settings, args), 0)
            rebuild.assert_called_once_with(session, session.map_path, voxel_size=0.02,
                                            bag_dir=None, cloud_topic="/g1_mapping/cloud_registered")
            accumulate.assert_not_called()

    def test_rebuild_uses_recorded_configuration_and_read_only_bag_mount(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            settings = replace(make_settings(root), octomap_max_range=2.0)
            session = SessionManager(settings).create(name="raw", backend="raw", diagnostic={})
            (session.directory / "raw" / "rosbag2").mkdir()
            self.assertEqual(json.loads(session.manifest_path.read_text())["occupancy_map"]["max_range"], 2.0)
            runtime = ComposeRuntime(replace(settings, octomap_max_range=10.0))
            with patch.object(runtime, "run", return_value=subprocess.CompletedProcess([], 0, "", "")) as run:
                runtime.rebuild_occupancy(session, session.map_path)
            command = run.call_args.args
            self.assertEqual(command[command.index("--max-range") + 1], "2.0")
            self.assertIn(f"{session.directory}/raw/rosbag2:/bag-input:ro", command)
            self.assertIn("/g1_mapping/cloud_registered", command)

    def test_stop_never_falls_back_to_uncorrected_accumulation_for_raw(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = make_settings(Path(directory))
            SessionManager(settings).create(name="raw", backend="raw", diagnostic={})
            args = _parser().parse_args(["stop"])
            with patch.object(ComposeRuntime, "save_and_stop"), \
                    patch.object(ComposeRuntime, "collect_logs"), \
                    patch.object(ComposeRuntime, "rebuild_occupancy", side_effect=RuntimeError("no sensor poses")) as rebuild, \
                    patch("g1_mapping.cli.rebuild_map") as accumulate, redirect_stdout(io.StringIO()):
                self.assertEqual(_stop(settings, args), 1)
            rebuild.assert_called_once()
            accumulate.assert_not_called()

    def test_save_calls_canonical_service_with_error_checking_client(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = make_settings(Path(directory))
            session = SessionManager(settings).create(name="sim", backend="sim", diagnostic={})
            runtime = ComposeRuntime(settings)
            with patch.object(runtime, "run", return_value=subprocess.CompletedProcess([], 0, "", "")) as run:
                runtime.save_and_stop(session)
            self.assertIn("save_occupancy_map", " ".join(run.call_args_list[0].args))

    def test_environment_carries_the_same_sensor_model_as_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = replace(make_settings(Path(directory)), octomap_prob_miss=0.35)
            self.assertEqual(settings.compose_environment()["OCTOMAP_PROB_MISS"], "0.35")
            self.assertEqual(settings.occupancy_parameters()["prob_miss"], 0.35)

    def test_replayed_bag_uses_its_own_occupancy_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = make_settings(Path(directory))
            session = SessionManager(settings).create(name="raw", backend="raw", diagnostic={})
            source = session.directory / "derived" / "replay" / "rosbag2"
            source.mkdir(parents=True)
            parameters = settings.occupancy_parameters()
            parameters["max_range"] = 1.5
            (source.parent / "occupancy.json").write_text(json.dumps({"parameters": parameters}))
            runtime = ComposeRuntime(settings)
            with patch.object(runtime, "run", return_value=subprocess.CompletedProcess([], 0, "", "")) as run:
                runtime.rebuild_occupancy(session, session.map_path, bag_dir=source)
            command = run.call_args.args
            self.assertEqual(command[command.index("--max-range") + 1], "1.5")

    def test_validation_requires_pose_records_and_occupancy_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = make_settings(Path(directory))
            session = SessionManager(settings).create(name="raw", backend="raw", diagnostic={})
            valid, report = validate_session(session.directory, min_map_points=1, allow_partial=False)
            self.assertFalse(valid)
            statuses = {check["name"]: check["status"] for check in report["checks"]}
            self.assertEqual(statuses["map.occupancy"], "FAIL")
            self.assertEqual(statuses["topic:/g1_mapping/sensor_pose"], "FAIL")


if __name__ == "__main__":
    unittest.main()
