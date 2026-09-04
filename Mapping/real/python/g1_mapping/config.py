"""ホスト側設定の読み込み。外部Pythonパッケージには依存しない。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import os
import re


SESSION_NAME_PATTERN = re.compile(r"[^a-zA-Z0-9_-]+")


def _parse_env_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.exists():
        return values

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
            value = value[1:-1]
        values[key] = value
    return values


def slugify_session_name(name: str) -> str:
    """セッション名をパスに安全なASCII文字へ正規化する。"""

    slug = SESSION_NAME_PATTERN.sub("_", name.strip()).strip("_-")
    return slug[:64] or "mapping"


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class Settings:
    project_dir: Path
    compose_file: Path
    runs_dir: Path
    env_file: Path
    g1_iface: str
    g1_host: str
    ros_domain_id: int
    raw_points_topic: str
    raw_imu_topic: str
    onboard_points_topic: str
    onboard_odom_topic: str
    onboard_remote_map_dir: str
    camera_enabled: bool
    camera_required: bool
    camera_host: str
    camera_zmq_port: int
    camera_name: str
    camera_frame_id: str
    camera_width: int
    camera_height: int
    camera_fx: float
    camera_fy: float
    camera_cx: float
    camera_cy: float
    min_free_gib: float
    min_map_points: int

    @classmethod
    def load(cls, project_dir: Path | None = None) -> "Settings":
        root = (project_dir or Path(__file__).resolve().parents[2]).resolve()
        env_file = root / ".env"
        file_values = _parse_env_file(env_file)

        def value(key: str, default: str) -> str:
            return os.environ.get(key, file_values.get(key, default))

        runs_value = value("MAPPING_RUNS_DIR", str(root / "runs"))
        runs_dir = Path(runs_value).expanduser()
        if not runs_dir.is_absolute():
            runs_dir = (root / runs_dir).resolve()

        return cls(
            project_dir=root,
            compose_file=root / "compose.yaml",
            runs_dir=runs_dir,
            env_file=env_file,
            g1_iface=value("G1_IFACE", "enp3s0"),
            g1_host=value("G1_HOST", "192.168.123.164"),
            ros_domain_id=int(value("ROS_DOMAIN_ID", "0")),
            raw_points_topic=value(
                "RAW_POINTS_TOPIC", "/utlidar/cloud_livox_mid360"
            ),
            raw_imu_topic=value("RAW_IMU_TOPIC", "/utlidar/imu_livox_mid360"),
            onboard_points_topic=value(
                "ONBOARD_POINTS_TOPIC", "/unitree/slam_mapping/points"
            ),
            onboard_odom_topic=value(
                "ONBOARD_ODOM_TOPIC", "/unitree/slam_mapping/odom"
            ),
            onboard_remote_map_dir=value(
                "ONBOARD_REMOTE_MAP_DIR", "/home/unitree/maps"
            ).rstrip("/"),
            camera_enabled=_as_bool(value("CAMERA_ENABLED", "true")),
            camera_required=_as_bool(value("CAMERA_REQUIRED", "true")),
            camera_host=value("CAMERA_HOST", value("G1_HOST", "192.168.123.164")),
            camera_zmq_port=int(value("CAMERA_ZMQ_PORT", "5555")),
            camera_name=value("CAMERA_NAME", "head_camera"),
            camera_frame_id=value(
                "CAMERA_FRAME_ID", "camera_color_optical_frame"
            ),
            camera_width=int(value("CAMERA_WIDTH", "0")),
            camera_height=int(value("CAMERA_HEIGHT", "0")),
            camera_fx=float(value("CAMERA_FX", "0")),
            camera_fy=float(value("CAMERA_FY", "0")),
            camera_cx=float(value("CAMERA_CX", "0")),
            camera_cy=float(value("CAMERA_CY", "0")),
            min_free_gib=float(value("MIN_FREE_GIB", "5")),
            min_map_points=int(value("MIN_MAP_POINTS", "1000")),
        )

    def compose_environment(
        self, session_id: str | None = None, backend: str | None = None
    ) -> dict[str, str]:
        env = os.environ.copy()
        simulation = backend == "sim"
        env.update(
            {
                "MAPPING_RUNS_DIR": str(self.runs_dir),
                "G1_IFACE": "lo" if simulation else self.g1_iface,
                "G1_HOST": self.g1_host,
                "ROS_DOMAIN_ID": str(self.ros_domain_id),
                "RAW_POINTS_TOPIC": self.raw_points_topic,
                "RAW_IMU_TOPIC": self.raw_imu_topic,
                "ONBOARD_POINTS_TOPIC": self.onboard_points_topic,
                "ONBOARD_ODOM_TOPIC": self.onboard_odom_topic,
                "USE_SIM_TIME": "true" if simulation else "false",
                "CAMERA_ENABLED": str(self.camera_enabled).lower(),
                "CAMERA_HOST": self.camera_host,
                "CAMERA_ZMQ_PORT": str(self.camera_zmq_port),
                "CAMERA_NAME": self.camera_name,
                "CAMERA_FRAME_ID": self.camera_frame_id,
                "CAMERA_WIDTH": str(self.camera_width),
                "CAMERA_HEIGHT": str(self.camera_height),
                "CAMERA_FX": str(self.camera_fx),
                "CAMERA_FY": str(self.camera_fy),
                "CAMERA_CX": str(self.camera_cx),
                "CAMERA_CY": str(self.camera_cy),
            }
        )
        if session_id:
            env["MAPPING_SESSION_ID"] = session_id
        return env
