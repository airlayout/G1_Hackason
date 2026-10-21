"""Run the existing raw/sim ROS nodes in a local overlay, without Docker."""

from __future__ import annotations

import json
import os
import shlex
import signal
import subprocess
import time
from contextlib import contextmanager
from pathlib import Path

from .config import Settings
from .session import MappingSession


def _identity(pid: int) -> str | None:
    """Linux process start time guards against signalling a reused PID."""
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return None if fields[0] == "Z" else fields[19]
    except (FileNotFoundError, ProcessLookupError):
        return None


class NativeRuntime:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.root = (
            settings.native_root
            or settings.project_dir.parents[1] / "_local" / "mapping"
        )
        self.scripts = settings.project_dir / "native"

    def environment(
        self, backend: str | None = None, session: MappingSession | None = None
    ) -> dict[str, str]:
        env = self.settings.compose_environment(
            session.session_id if session else None, backend
        )
        interface = (
            "lo"
            if backend == "sim" or self.settings.transport == "ssh"
            else self.settings.g1_iface
        )
        env.update(
            {
                "MAPPING_NATIVE_ROOT": str(self.root),
                "MAPPING_RUNS_DIR": str(self.settings.runs_dir),
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "CYCLONEDDS_URI": "<CycloneDDS><Domain><General><Interfaces>"
                f'<NetworkInterface name="{interface}" />'
                "</Interfaces></General></Domain></CycloneDDS>",
                "ROS_LOG_DIR": str(
                    (session.directory if session else self.root) / "logs" / "ros"
                ),
            }
        )
        return env

    def command(self, *arguments: str) -> list[str]:
        return ["bash", str(self.scripts / "exec.sh"), *arguments]

    def run(
        self,
        *arguments: str,
        backend: str | None = None,
        session: MappingSession | None = None,
        timeout: float | None = None,
        check: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        result = subprocess.run(
            self.command(*arguments),
            env=self.environment(backend, session),
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False,
        )
        if check and result.returncode:
            raise RuntimeError((result.stderr or result.stdout).strip())
        return result

    def build(self) -> None:
        subprocess.run(
            ["bash", str(self.scripts / "setup.sh")], env=self.environment(), check=True
        )
        if self.settings.transport == "ssh":
            self.deploy_sensors()

    def deploy_sensors(self) -> None:
        target = f"{self.settings.ssh_user}@{self.settings.ssh_host or self.settings.g1_host}"
        destination = self.settings.sensor_remote_dir
        subprocess.run(
            [
                "ssh",
                "-o",
                "IPQoS=none",
                "-o",
                "BatchMode=yes",
                target,
                shlex.join(["mkdir", "-p", destination]),
            ],
            check=True,
        )
        subprocess.run(
            [
                "scp",
                "-o",
                "IPQoS=none",
                "-o",
                "BatchMode=yes",
                str(
                    self.settings.project_dir
                    / "ros2_ws/src/g1_mapping_tools/g1_mapping_tools/sensor_stream.py"
                ),
                str(self.settings.project_dir / "quickstart/idl_imu.py"),
                f"{target}:{destination}/",
            ],
            check=True,
        )

    def sensor_command(self) -> list[str]:
        return [
            "python3",
            "-m",
            "g1_mapping_tools.sensor_stream",
            "--host",
            f"{self.settings.ssh_user}@{self.settings.ssh_host or self.settings.g1_host}",
            "--remote-script",
            f"{self.settings.sensor_remote_dir}/sensor_stream.py",
            "--iface",
            self.settings.sensor_iface,
            "--sdk-path",
            self.settings.sensor_sdk_path,
            "--points-topic",
            self.settings.raw_points_topic,
            "--imu-topic",
            self.settings.raw_imu_topic,
        ]

    @contextmanager
    def sensor_probe(self, backend: str):
        if self.settings.transport != "ssh" or backend == "sim":
            yield
            return
        log_path = self.root / "logs" / "sensor_probe.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("w") as log:
            process = subprocess.Popen(
                self.command(*self.sensor_command()),
                env=self.environment(backend),
                stdout=log,
                stderr=log,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
            try:
                yield
                if process.poll() is not None:
                    raise RuntimeError(log_path.read_text())
            finally:
                if process.poll() is None:
                    # Let the ROS node close SSH stdin, so the G1 reader exits cleanly.
                    os.kill(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.terminate()
                        process.wait(timeout=5)

    def backend_available(self, backend: str) -> bool:
        return backend in {"raw", "sim"} and all(
            (self.root / "install" / "lib" / package / executable).is_file()
            for package, executable in (
                ("fast_lio", "fastlio_mapping"),
                ("g1_mapping_octomap", "occupancy_mapper"),
            )
        )

    def probe_raw(
        self, timeout_seconds: int, *, backend: str = "raw"
    ) -> subprocess.CompletedProcess[str]:
        with self.sensor_probe(backend):
            return self._probe_raw(timeout_seconds, backend=backend)

    def _probe_raw(
        self, timeout_seconds: int, *, backend: str
    ) -> subprocess.CompletedProcess[str]:
        return self.run(
            "ros2",
            "run",
            "g1_mapping_tools",
            "sensor_doctor",
            "--points-topic",
            self.settings.raw_points_topic,
            "--imu-topic",
            self.settings.raw_imu_topic,
            "--duration",
            str(timeout_seconds),
            *(["--require-camera"] if backend == "sim" else []),
            backend=backend,
            timeout=timeout_seconds + 15,
            check=False,
        )

    def probe_camera(
        self, timeout_seconds: int, *, backend: str
    ) -> subprocess.CompletedProcess[str]:
        return self.run(
            "ros2",
            "run",
            "g1_mapping_tools",
            "zmq_camera_probe",
            "--host",
            self.settings.camera_host,
            "--port",
            str(self.settings.camera_zmq_port),
            "--camera-name",
            self.settings.camera_name,
            "--duration",
            str(timeout_seconds),
            backend=backend,
            timeout=timeout_seconds + 15,
            check=False,
        )

    @staticmethod
    def _process_file(session: MappingSession) -> Path:
        return session.directory / "logs" / "native_processes.json"

    def start(self, session: MappingSession) -> None:
        if session.backend not in {"raw", "sim"}:
            raise ValueError("native環境ではrawまたはsimを指定してください")
        roles = {
            "recorder": [
                "bash",
                str(self.settings.project_dir / "docker" / "record-topics.sh"),
            ],
            "trajectory": [
                "bash",
                str(self.settings.project_dir / "docker" / "trajectory-writer.sh"),
            ],
        }
        if self.settings.transport == "ssh" and session.backend != "sim":
            roles["sensors"] = self.sensor_command()
        if self.settings.camera_enabled and session.backend != "sim":
            roles["camera"] = [
                "ros2",
                "run",
                "g1_mapping_tools",
                "zmq_camera_bridge",
                "--ros-args",
                *[
                    argument
                    for name, value in {
                        "host": self.settings.camera_host,
                        "port": self.settings.camera_zmq_port,
                        "camera_name": self.settings.camera_name,
                        "frame_id": self.settings.camera_frame_id,
                        "width": self.settings.camera_width,
                        "height": self.settings.camera_height,
                        "fx": self.settings.camera_fx,
                        "fy": self.settings.camera_fy,
                        "cx": self.settings.camera_cx,
                        "cy": self.settings.camera_cy,
                    }.items()
                    for argument in ("-p", f"{name}:={value}")
                ],
            ]
        roles["lio"] = ["ros2", "launch", "g1_mapping_bringup", "raw_mapping.launch.py"]
        processes = {}
        try:
            for role, arguments in roles.items():
                with (session.directory / "logs" / f"{role}.log").open("a") as log:
                    process = subprocess.Popen(
                        self.command(*arguments),
                        env=self.environment(session.backend, session),
                        stdout=log,
                        stderr=log,
                        stdin=subprocess.DEVNULL,
                        start_new_session=True,
                    )
                processes[role] = {
                    "pid": process.pid,
                    "identity": _identity(process.pid),
                }
                self._process_file(session).write_text(
                    json.dumps(processes, indent=2) + "\n"
                )
            time.sleep(1.0)
            failed = [
                role for role, process in processes.items() if not self._alive(process)
            ]
            if failed:
                raise RuntimeError(
                    f"起動に失敗しました: {', '.join(failed)}。logs/を確認してください"
                )
        except BaseException:
            self._stop_processes(session)
            raise

    @staticmethod
    def _alive(process: dict) -> bool:
        return (
            process["identity"] is not None
            and _identity(process["pid"]) == process["identity"]
        )

    def _stop_processes(self, session: MappingSession) -> None:
        path = self._process_file(session)
        if not path.exists():
            return
        processes = json.loads(path.read_text())
        errors = []
        # Close the recorder last so its bag includes the final canonical scan/pose.
        for role in ("sensors", "lio", "camera", "trajectory", "recorder"):
            process = processes.get(role)
            if process is None or not self._alive(process):
                continue
            for sig, timeout in ((signal.SIGINT, 30), (signal.SIGTERM, 5)):
                try:
                    # The sensor node closes its own SSH connection on shutdown.
                    if role in {"sensors", "lio"}:
                        os.kill(process["pid"], sig)
                    else:
                        os.killpg(process["pid"], sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + timeout
                while self._alive(process) and time.monotonic() < deadline:
                    time.sleep(0.1)
                if not self._alive(process):
                    break
            if self._alive(process):
                errors.append(role)
        if errors:
            raise RuntimeError(
                f"正常終了できませんでした: {', '.join(errors)}。logs/を確認してください"
            )

    def save_and_stop(self, session: MappingSession) -> None:
        try:
            self.run(
                "ros2",
                "run",
                "g1_mapping_tools",
                "save_occupancy_map",
                "--timeout",
                "45",
                backend=session.backend,
                session=session,
                timeout=60,
            )
        finally:
            self._stop_processes(session)

    def collect_logs(self, session: MappingSession) -> None:
        # Each role writes directly to the session's logs directory.
        pass

    def ps(self, session: MappingSession) -> str:
        path = self._process_file(session)
        processes = json.loads(path.read_text()) if path.exists() else {}
        return "\n".join(
            f"{role}: {'running' if self._alive(process) else 'exited'} (pid={process['pid']})"
            for role, process in processes.items()
        )

    def rebuild_occupancy(
        self,
        session: MappingSession,
        output_path: Path,
        *,
        voxel_size: float | None = None,
        bag_dir: Path | None = None,
        cloud_topic: str = "/g1_mapping/cloud_registered",
    ) -> None:
        source = (bag_dir or session.directory / "raw" / "rosbag2").resolve()
        if not source.is_dir():
            raise RuntimeError(f"rosbagディレクトリがありません: {source}")
        manifest = json.loads(session.manifest_path.read_text())
        parameters = (
            manifest.get("occupancy_map") or self.settings.occupancy_parameters()
        )
        replay_config = source.parent / "occupancy.json"
        if bag_dir is not None and replay_config.is_file():
            parameters = json.loads(replay_config.read_text())["parameters"]
        if voxel_size is not None:
            parameters["voxel_size"] = voxel_size
        options = [
            argument
            for name, value in parameters.items()
            for argument in (f"--{name.replace('_', '-')}", str(value).lower())
        ]
        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        result = self.run(
            "ros2",
            "run",
            "g1_mapping_octomap",
            "rebuild_occupancy",
            "--bag-dir",
            str(source),
            "--output",
            str(output_path),
            "--cloud-topic",
            cloud_topic,
            *options,
            backend="sim",
            session=session,
        )
        print(result.stdout, end="")

    def view(
        self,
        *,
        mode: str,
        pcd_path: str,
        fixed_frame: str,
        map_topic: str,
        rviz: bool,
        backend: str | None = None,
    ) -> None:
        arguments = [
            "ros2",
            "launch",
            "g1_mapping_visualization",
            "view.launch.py",
            f"mode:={mode}",
            f"fixed_frame:={fixed_frame}",
            f"map_topic:={map_topic}",
            f"rviz:={str(rviz).lower()}",
        ]
        if pcd_path:
            arguments.append(f"pcd_path:={pcd_path}")
        result = subprocess.run(
            self.command(*arguments),
            env=self.environment(backend if mode == "live" else "sim"),
            check=False,
        )
        if result.returncode:
            raise RuntimeError(f"可視化が終了コード{result.returncode}を返しました")
