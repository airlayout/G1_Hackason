"""Docker Composeを介したbackend実行。"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess

from .config import Settings
from .session import MappingSession


class ComposeError(RuntimeError):
    pass


class ComposeRuntime:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings

    def _command(self, *arguments: str) -> list[str]:
        env_file = (
            self.settings.env_file
            if self.settings.env_file.exists()
            else self.settings.project_dir / ".env.example"
        )
        return [
            "docker",
            "compose",
            "--env-file",
            str(env_file),
            "-f",
            str(self.settings.compose_file),
            *arguments,
        ]

    def run(
        self,
        *arguments: str,
        session_id: str | None = None,
        timeout: float | None = None,
        check: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        result = subprocess.run(
            self._command(*arguments),
            cwd=self.settings.project_dir,
            env=self.settings.compose_environment(session_id),
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False,
        )
        if check and result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise ComposeError(detail or f"Composeが終了コード{result.returncode}を返しました")
        return result

    def image_exists(self, image: str) -> bool:
        result = subprocess.run(
            ["docker", "image", "inspect", image],
            text=True,
            capture_output=True,
            check=False,
        )
        return result.returncode == 0

    def build(self) -> None:
        result = self.run(
            "--profile",
            "onboard",
            "--profile",
            "raw",
            "--profile",
            "viz",
            "build",
        )
        if result.stdout:
            print(result.stdout, end="")

    def view(
        self,
        *,
        mode: str,
        pcd_path: str,
        fixed_frame: str,
        map_topic: str,
        rviz: bool,
    ) -> None:
        """可視化コンテナをforegroundで実行する。"""

        service = "rviz" if rviz else "pcd_server"
        arguments = ["--profile", "viz", "run", "--rm", "-T"]
        if mode == "saved":
            # 保存PCDの閲覧はG1用NICがない開発PCでも動作させる。
            loopback_uri = (
                "<CycloneDDS><Domain><General><Interfaces>"
                '<NetworkInterface name="lo" />'
                "</Interfaces></General></Domain></CycloneDDS>"
            )
            arguments.extend(["-e", f"CYCLONEDDS_URI={loopback_uri}"])
        launch_arguments = [f"mode:={mode}"]
        # ライブ表示ではPCDを読まないため pcd_path が空になる。ros2 launch は
        # `pcd_path:=` のような空値を malformed として拒否するので、空のときは
        # 引数自体を渡さない（view.launch.py が default_value="" を宣言済み）。
        if pcd_path:
            launch_arguments.append(f"pcd_path:={pcd_path}")
        launch_arguments.extend(
            [
                f"fixed_frame:={fixed_frame}",
                f"map_topic:={map_topic}",
                f"rviz:={'true' if rviz else 'false'}",
            ]
        )
        arguments.extend(
            [
                service,
                "ros2",
                "launch",
                "g1_mapping_visualization",
                "view.launch.py",
                *launch_arguments,
            ]
        )
        environment = self.settings.compose_environment()
        if rviz and not environment.get("XAUTHORITY"):
            xauthority_candidates = (
                Path.home() / ".Xauthority",
                Path(f"/run/user/{os.getuid()}/gdm/Xauthority"),
            )
            environment["XAUTHORITY"] = str(
                next(
                    (path for path in xauthority_candidates if path.is_file()),
                    Path("/dev/null"),
                )
            )
        result = subprocess.run(
            self._command(*arguments),
            cwd=self.settings.project_dir,
            env=environment,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            raise ComposeError(
                f"可視化コンテナが終了コード{result.returncode}を返しました"
            )

    def probe_onboard(self, timeout_seconds: int) -> subprocess.CompletedProcess[str]:
        return self.run(
            "--profile",
            "onboard",
            "run",
            "--rm",
            "-T",
            "onboard_client",
            "probe",
            "--timeout",
            str(timeout_seconds),
            timeout=timeout_seconds + 10,
            check=False,
        )

    def probe_raw(self, timeout_seconds: int) -> subprocess.CompletedProcess[str]:
        return self.run(
            "--profile",
            "raw",
            "run",
            "--rm",
            "-T",
            "raw_tools",
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
            timeout=timeout_seconds + 15,
            check=False,
        )

    def start(self, session: MappingSession) -> None:
        if session.backend == "onboard":
            self.run(
                "--profile",
                "onboard",
                "up",
                "-d",
                "onboard_recorder",
                "onboard_trajectory",
                session_id=session.session_id,
            )
            self.run(
                "--profile",
                "onboard",
                "run",
                "--rm",
                "-T",
                "onboard_client",
                "start",
                "--map",
                session.remote_map_path,
                session_id=session.session_id,
                timeout=20,
            )
            return

        if session.backend == "raw":
            self.run(
                "--profile",
                "raw",
                "up",
                "-d",
                "raw_recorder",
                "raw_trajectory",
                "raw_lio",
                session_id=session.session_id,
            )
            return

        raise ValueError(f"未知のbackendです: {session.backend}")

    def save_and_stop(self, session: MappingSession) -> None:
        if session.backend == "onboard":
            self.run(
                "--profile",
                "onboard",
                "run",
                "--rm",
                "-T",
                "onboard_client",
                "stop",
                "--map",
                session.remote_map_path,
                session_id=session.session_id,
                timeout=30,
            )
            self.run(
                "--profile",
                "onboard",
                "stop",
                "-t",
                "20",
                "onboard_trajectory",
                "onboard_recorder",
                session_id=session.session_id,
                check=False,
            )
            return

        if session.backend == "raw":
            self.run(
                "--profile",
                "raw",
                "exec",
                "-T",
                "raw_lio",
                "bash",
                "-lc",
                (
                    "source /opt/ros/humble/setup.bash && "
                    "source /opt/g1_ws/install/setup.bash && "
                    "ros2 service call /map_save std_srvs/srv/Trigger '{}'"
                ),
                session_id=session.session_id,
                timeout=60,
            )
            self.run(
                "--profile",
                "raw",
                "stop",
                "-t",
                "30",
                "raw_lio",
                "raw_trajectory",
                "raw_recorder",
                session_id=session.session_id,
                check=False,
            )
            return

        raise ValueError(f"未知のbackendです: {session.backend}")

    def collect_logs(self, session: MappingSession) -> None:
        profile = session.backend
        result = self.run(
            "--profile",
            profile,
            "logs",
            "--no-color",
            session_id=session.session_id,
            check=False,
        )
        log_path = session.directory / "logs" / "compose.log"
        log_path.write_text(result.stdout + result.stderr, encoding="utf-8")

    def ps(self, session: MappingSession) -> str:
        result = self.run(
            "--profile",
            session.backend,
            "ps",
            session_id=session.session_id,
            check=False,
        )
        return (result.stdout + result.stderr).strip()
