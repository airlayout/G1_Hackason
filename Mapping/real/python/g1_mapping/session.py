"""Mappingセッションの生成と状態管理。"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import json
import subprocess

from .config import Settings, slugify_session_name


STATE_NAMES = {"created", "running", "stopping", "completed", "failed"}


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def _read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _git_revision(project_dir: Path) -> str:
    repository = project_dir.parents[1]
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=repository,
        text=True,
        capture_output=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


@dataclass(frozen=True)
class MappingSession:
    session_id: str
    directory: Path
    backend: str
    remote_map_path: str

    @property
    def state_path(self) -> Path:
        return self.directory / "state.json"

    @property
    def manifest_path(self) -> Path:
        return self.directory / "manifest.json"

    @property
    def map_path(self) -> Path:
        return self.directory / "map" / "map_raw.pcd"


class SessionManager:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.active_path = settings.project_dir / ".mapping-state.json"

    def create(
        self,
        *,
        name: str,
        backend: str,
        diagnostic: dict[str, object],
    ) -> MappingSession:
        if self.active_path.exists():
            active = _read_json(self.active_path)
            raise RuntimeError(
                f"実行中セッションがあります: {active.get('session_id', 'unknown')}"
            )

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_id = f"{timestamp}_{slugify_session_name(name)}"
        directory = self.settings.runs_dir / session_id
        if directory.exists():
            raise RuntimeError(f"セッションディレクトリが既に存在します: {directory}")

        for child in (
            "raw",
            "calibration",
            "map",
            "trajectory",
            "derived",
            "logs",
            "report",
        ):
            (directory / child).mkdir(parents=True, exist_ok=False)

        remote_map_path = (
            f"{self.settings.onboard_remote_map_dir}/{session_id}.pcd"
            if backend == "onboard"
            else ""
        )
        now = datetime.now(timezone.utc).isoformat()
        manifest: dict[str, object] = {
            "schema_version": 2,
            "session_id": session_id,
            "name": name,
            "backend": backend,
            "created_at": now,
            "git_revision": _git_revision(self.settings.project_dir),
            "robot": {
                "host": None if backend == "sim" else self.settings.g1_host,
                "network_interface": "lo" if backend == "sim" else self.settings.g1_iface,
                "ros_domain_id": self.settings.ros_domain_id,
            },
            "topics": {
                "raw_points": self.settings.raw_points_topic,
                "raw_imu": self.settings.raw_imu_topic,
                "onboard_points": self.settings.onboard_points_topic,
                "onboard_odom": self.settings.onboard_odom_topic,
                "canonical_odom": "/g1_mapping/odom",
                "canonical_registered_cloud": "/g1_mapping/cloud_registered",
                "canonical_map": "/g1_mapping/map",
                "camera_image": "/g1_camera/color/image/compressed",
                "camera_info": "/g1_camera/color/camera_info",
                "camera_metadata": "/g1_camera/frame_metadata",
            },
            "camera": {
                "required": self.settings.camera_required,
                "source": "isaac_sim" if backend == "sim" else "lerobot_zmq",
                "host": None if backend == "sim" else self.settings.camera_host,
                "port": None if backend == "sim" else self.settings.camera_zmq_port,
                "name": self.settings.camera_name,
                "frame_id": self.settings.camera_frame_id,
                "intrinsics": {
                    "width": self.settings.camera_width,
                    "height": self.settings.camera_height,
                    "fx": self.settings.camera_fx,
                    "fy": self.settings.camera_fy,
                    "cx": self.settings.camera_cx,
                    "cy": self.settings.camera_cy,
                },
                "calibration_complete": (
                    backend == "sim"
                    or (self.settings.camera_fx > 0.0 and self.settings.camera_fy > 0.0)
                ),
            },
            "remote_map_path": remote_map_path,
            "diagnostic": diagnostic,
        }
        state: dict[str, object] = {
            "session_id": session_id,
            "backend": backend,
            "status": "created",
            "updated_at": now,
            "message": "セッションを作成しました",
        }
        _write_json(directory / "manifest.json", manifest)
        _write_json(directory / "state.json", state)
        _write_json(
            self.active_path,
            {
                "session_id": session_id,
                "directory": str(directory),
                "backend": backend,
                "remote_map_path": remote_map_path,
            },
        )
        return MappingSession(session_id, directory, backend, remote_map_path)

    def active(self) -> MappingSession | None:
        if not self.active_path.exists():
            return None
        data = _read_json(self.active_path)
        return MappingSession(
            session_id=str(data["session_id"]),
            directory=Path(str(data["directory"])),
            backend=str(data["backend"]),
            remote_map_path=str(data["remote_map_path"]),
        )

    def resolve(self, session_id: str | None = None) -> MappingSession:
        if session_id is None:
            active = self.active()
            if active is not None:
                return active
            candidates = sorted(
                (path for path in self.settings.runs_dir.glob("*") if path.is_dir()),
                reverse=True,
            )
            if not candidates:
                raise RuntimeError("Mappingセッションがまだありません")
            directory = candidates[0]
        else:
            directory = self.settings.runs_dir / session_id

        manifest = _read_json(directory / "manifest.json")
        return MappingSession(
            session_id=str(manifest["session_id"]),
            directory=directory,
            backend=str(manifest["backend"]),
            remote_map_path=str(manifest.get("remote_map_path", "")),
        )

    def update(
        self, session: MappingSession, status: str, message: str
    ) -> dict[str, object]:
        if status not in STATE_NAMES:
            raise ValueError(f"不正な状態です: {status}")
        state: dict[str, object] = {
            "session_id": session.session_id,
            "backend": session.backend,
            "status": status,
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "message": message,
        }
        _write_json(session.state_path, state)
        return state

    def finish(self, session: MappingSession, *, success: bool, message: str) -> None:
        self.update(session, "completed" if success else "failed", message)
        if self.active_path.exists():
            active = _read_json(self.active_path)
            if active.get("session_id") == session.session_id:
                self.active_path.unlink()

    def state(self, session: MappingSession) -> dict[str, object]:
        return _read_json(session.state_path)
