from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from g1_mapping.config import Settings, slugify_session_name
from g1_mapping.cli import _parser
from g1_mapping.doctor import diagnose
from g1_mapping.mock import write_mock_artifacts
from g1_mapping.pcd import inspect_pcd, validate_session
from g1_mapping.session import SessionManager


def make_settings(root: Path) -> Settings:
    return Settings(
        project_dir=root,
        compose_file=root / "compose.yaml",
        runs_dir=root / "runs",
        env_file=root / ".env",
        g1_iface="mock0",
        g1_host="192.168.123.164",
        ros_domain_id=0,
        raw_points_topic="/utlidar/cloud_livox_mid360",
        raw_imu_topic="/utlidar/imu_livox_mid360",
        onboard_points_topic="/unitree/slam_mapping/points",
        onboard_odom_topic="/unitree/slam_mapping/odom",
        onboard_remote_map_dir="/home/unitree/maps",
        camera_enabled=True,
        camera_required=True,
        camera_host="192.168.123.164",
        camera_zmq_port=5555,
        camera_name="head_camera",
        camera_frame_id="camera_color_optical_frame",
        camera_width=640,
        camera_height=360,
        camera_fx=450.0,
        camera_fy=450.0,
        camera_cx=320.0,
        camera_cy=180.0,
        map_voxel_size=0.05,
        density_target_scans=10,
        min_free_gib=0.01,
        min_map_points=1000,
    )


class ConfigurationTest(unittest.TestCase):
    def test_slugify_removes_unsafe_characters(self) -> None:
        self.assertEqual(slugify_session_name(" Room A / 入口 "), "Room_A")
        self.assertEqual(slugify_session_name("***"), "mapping")

    def test_view_command_parses_live_and_saved_modes(self) -> None:
        live = _parser().parse_args(["view", "--live"])
        self.assertTrue(live.live)
        self.assertIsNone(live.session_id)

        saved = _parser().parse_args(
            ["view", "session_01", "--publish-only", "--fixed-frame", "map"]
        )
        self.assertEqual(saved.session_id, "session_01")
        self.assertTrue(saved.publish_only)
        self.assertEqual(saved.fixed_frame, "map")

    def test_sim_backend_uses_loopback_and_sim_time(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            settings = make_settings(Path(temporary))
            environment = settings.compose_environment(backend="sim")
            self.assertEqual(environment["G1_IFACE"], "lo")
            self.assertEqual(environment["USE_SIM_TIME"], "true")

        arguments = _parser().parse_args(["start", "--backend", "sim"])
        self.assertEqual(arguments.backend, "sim")


class SessionLifecycleTest(unittest.TestCase):
    def test_mock_session_produces_valid_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            settings = make_settings(Path(temporary))
            report = diagnose(settings, requested_backend="onboard", mock=True)
            manager = SessionManager(settings)
            session = manager.create(
                name="room A", backend="mock", diagnostic=report.to_dict()
            )
            manager.update(session, "running", "test")
            self.assertEqual(manager.active(), session)

            write_mock_artifacts(session)
            success, quality = validate_session(
                session.directory,
                min_map_points=settings.min_map_points,
                allow_partial=False,
            )
            self.assertTrue(success)
            self.assertTrue(quality["success"])
            self.assertEqual(inspect_pcd(session.map_path)["points"], 1200)
            self.assertTrue(quality["topic_counts"]["/g1_mapping/odom"] > 0)
            self.assertTrue(
                quality["topic_counts"]["/g1_camera/color/image/compressed"] > 0
            )

            manager.finish(session, success=True, message="done")
            self.assertIsNone(manager.active())
            self.assertEqual(manager.state(session)["status"], "completed")

    def test_second_active_session_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            settings = make_settings(Path(temporary))
            manager = SessionManager(settings)
            manager.create(name="first", backend="mock", diagnostic={})
            with self.assertRaises(RuntimeError):
                manager.create(name="second", backend="mock", diagnostic={})


class ValidationTest(unittest.TestCase):
    def test_missing_map_fails_unless_partial_is_allowed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "report").mkdir()
            success, _ = validate_session(
                directory, min_map_points=1000, allow_partial=False
            )
            self.assertFalse(success)
            partial_success, _ = validate_session(
                directory, min_map_points=1000, allow_partial=True
            )
            self.assertTrue(partial_success)


if __name__ == "__main__":
    unittest.main()
