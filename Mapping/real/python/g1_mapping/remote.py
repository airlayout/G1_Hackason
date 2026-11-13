"""G1 capture control and checksum-verified, resumable collection over SSH."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from .config import Settings
from .session import MappingSession


def remote_enabled(settings: Settings, backend: str) -> bool:
    return settings.runtime == "native" and settings.transport == "ssh" and backend == "raw"


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


class RemoteRecording:
    def __init__(self, settings: Settings, session: MappingSession):
        self.settings, self.session = settings, session
        manifest = json.loads(session.manifest_path.read_text())
        self.info = manifest["remote_recording"]
        self.directory = self.info["directory"]
        self.target = f"{settings.ssh_user}@{settings.ssh_host or settings.g1_host}"
        self.script = self.info["script"]

    def command(self, action: str, *options: str) -> list[str]:
        return ["ssh", "-T", "-o", "IPQoS=none", "-o", "BatchMode=yes",
                "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=3",
                "-o", "ServerAliveCountMax=2", self.target,
                shlex.join(["python3", "-u", self.script, action,
                            "--directory", self.directory, *options])]

    def request(self, action: str, *options: str) -> dict:
        try:
            result = subprocess.run(self.command(action, *options), capture_output=True,
                                    text=True, timeout=40, check=False)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError("G1との通信がタイムアウトしました。再接続後にmapctl recoverを実行してください") from error
        if result.returncode:
            raise RuntimeError(f"G1 {action}: {result.stderr.strip() or 'SSH connection failed'}")
        try:
            return json.loads(result.stdout)
        except ValueError as error:
            raise RuntimeError(f"G1 {action}: invalid response: {result.stdout[-500:]}") from error

    def start(self) -> None:
        self.request("start", "--config", json.dumps(self.info["config"]))

    def stop(self) -> dict:
        return self.request("stop")

    def collect(self) -> Path:
        inventory = self.request("inventory")
        destination = self.session.directory / "raw" / "g1"
        destination.mkdir(parents=True, exist_ok=True)
        for entry in inventory["files"]:
            name = entry["name"]
            if name not in {"rosbag2/capture.db3", "rosbag2/metadata.yaml", "state.json", "config.json", "capture.log"}:
                raise RuntimeError("unexpected remote recording file")
            output = destination / name
            output.parent.mkdir(parents=True, exist_ok=True)
            if output.exists() and output.stat().st_size == entry["size"] and digest(output) == entry["sha256"]:
                continue
            partial = output.with_suffix(output.suffix + ".part")
            offset = partial.stat().st_size if partial.exists() else 0
            if offset > entry["size"]:
                partial.unlink()
                offset = 0
            if shutil.disk_usage(destination).free < entry["size"] - offset + 100 * 1024**2:
                raise RuntimeError("PCの空き容量が不足しています。G1の原本は保持されています")
            print(f"[COLLECT] {name}: {offset}/{entry['size']} bytes", flush=True)
            with partial.open("ab") as stream:
                result = subprocess.run(self.command("fetch", "--name", name, "--offset", str(offset)),
                                        stdout=stream, stderr=subprocess.PIPE, check=False)
            if result.returncode:
                raise RuntimeError("回収中に切断されました。mapctl recoverで続きから再開できます: " +
                                   result.stderr.decode(errors="replace")[-500:])
            if partial.stat().st_size != entry["size"] or digest(partial) != entry["sha256"]:
                partial.unlink()
                raise RuntimeError(f"回収ファイルの照合に失敗しました: {name}。mapctl recoverで再取得してください")
            partial.replace(output)
        report = destination / "inventory.json"
        report.write_text(json.dumps(inventory, indent=2) + "\n")
        state = inventory["state"]
        if state.get("status") != "completed" or state.get("dropped", 0):
            raise RuntimeError(f"G1記録に異常があります（回収済み）: {state.get('error', state)}")
        counts = state.get("counts", {})
        if not counts.get("1") or not counts.get("2"):
            raise RuntimeError("G1記録のLiDAR/IMUが空です")
        if self.info["config"].get("camera_required") and not counts.get("3"):
            raise RuntimeError("G1記録のRGBが空です")
        return destination / "rosbag2"
