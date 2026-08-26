"""ホスト環境と2つのMapping backendの非破壊診断。"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import ipaddress
import json
import shutil
import subprocess

from .compose import ComposeRuntime
from .config import Settings


@dataclass(frozen=True)
class Check:
    name: str
    status: str
    message: str


@dataclass(frozen=True)
class DiagnosticReport:
    requested_backend: str
    selected_backend: str | None
    checks: list[Check]

    @property
    def ready(self) -> bool:
        return self.selected_backend is not None and not any(
            check.status == "FAIL" and check.name.startswith("host.")
            for check in self.checks
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "requested_backend": self.requested_backend,
            "selected_backend": self.selected_backend,
            "ready": self.ready,
            "checks": [asdict(check) for check in self.checks],
        }


def _run(command: list[str], timeout: float = 10) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )


def _interface_addresses(interface: str) -> list[ipaddress.IPv4Interface]:
    result = _run(["ip", "-j", "address", "show", "dev", interface])
    if result.returncode != 0:
        return []
    payload = json.loads(result.stdout)
    addresses: list[ipaddress.IPv4Interface] = []
    for device in payload:
        for info in device.get("addr_info", []):
            if info.get("family") != "inet":
                continue
            addresses.append(
                ipaddress.ip_interface(f"{info['local']}/{info['prefixlen']}")
            )
    return addresses


def _host_checks(settings: Settings, *, mock: bool) -> list[Check]:
    checks: list[Check] = []
    if mock:
        checks.append(Check("host.network", "PASS", "mockではG1用NICを要求しません"))
    else:
        interface_path = Path("/sys/class/net") / settings.g1_iface
        if not interface_path.exists():
            checks.append(
                Check(
                    "host.network",
                    "FAIL",
                    f"NIC {settings.g1_iface} がありません。.envのG1_IFACEを確認してください",
                )
            )
        else:
            subnet = ipaddress.ip_network("192.168.123.0/24")
            addresses = _interface_addresses(settings.g1_iface)
            matching = [item for item in addresses if item.ip in subnet]
            if matching:
                checks.append(
                    Check("host.network", "PASS", f"{settings.g1_iface}: {matching[0]}")
                )
            else:
                checks.append(
                    Check(
                        "host.network",
                        "FAIL",
                        f"{settings.g1_iface}に192.168.123.0/24のIPv4がありません",
                    )
                )

    if shutil.which("docker") is None:
        checks.append(Check("host.docker", "FAIL", "dockerが見つかりません"))
    else:
        result = _run(["docker", "info"], timeout=15)
        checks.append(
            Check(
                "host.docker",
                "PASS" if result.returncode == 0 else "FAIL",
                "Docker daemonへ接続できます"
                if result.returncode == 0
                else (result.stderr.strip() or "Docker daemonへ接続できません"),
            )
        )

    result = _run(["docker", "compose", "version"])
    checks.append(
        Check(
            "host.compose",
            "PASS" if result.returncode == 0 else "FAIL",
            result.stdout.strip() or result.stderr.strip(),
        )
    )

    settings.runs_dir.mkdir(parents=True, exist_ok=True)
    free_gib = shutil.disk_usage(settings.runs_dir).free / (1024**3)
    checks.append(
        Check(
            "host.disk",
            "PASS" if free_gib >= settings.min_free_gib else "FAIL",
            f"空き容量 {free_gib:.1f} GiB（必要 {settings.min_free_gib:.1f} GiB以上）",
        )
    )

    if shutil.which("scp") is None:
        checks.append(
            Check(
                "host.scp",
                "WARN",
                "scpがありません。内蔵LIOのPCDをG1から自動回収できません",
            )
        )
    else:
        checks.append(Check("host.scp", "PASS", "scpを利用できます"))

    return checks


def diagnose(
    settings: Settings,
    *,
    requested_backend: str,
    mock: bool = False,
    timeout_seconds: int = 5,
) -> DiagnosticReport:
    checks = _host_checks(settings, mock=mock)
    if mock:
        checks.extend(
            [
                Check("backend.onboard", "PASS", "mock onboard serviceは応答可能です"),
                Check(
                    "sensor.raw",
                    "PASS",
                    "mock PointCloud2/IMUはfields・周波数・時刻とも正常です",
                ),
            ]
        )
        selected = "onboard" if requested_backend == "auto" else requested_backend
        return DiagnosticReport(requested_backend, selected, checks)

    runtime = ComposeRuntime(settings)
    candidates = (
        ["onboard", "raw"] if requested_backend == "auto" else [requested_backend]
    )
    selected: str | None = None

    for backend in candidates:
        image = f"g1-mapping-{backend}:local"
        if not runtime.image_exists(image):
            checks.append(
                Check(
                    f"backend.{backend}",
                    "FAIL",
                    f"image {image} がありません。先に ./mapctl build を実行してください",
                )
            )
            continue

        result = (
            runtime.probe_onboard(timeout_seconds)
            if backend == "onboard"
            else runtime.probe_raw(timeout_seconds)
        )
        detail = (result.stdout + result.stderr).strip()
        if result.returncode == 0:
            checks.append(Check(f"backend.{backend}", "PASS", detail or "応答あり"))
            selected = backend
            break
        checks.append(
            Check(
                f"backend.{backend}",
                "FAIL",
                detail or "backendから応答がありません",
            )
        )

    return DiagnosticReport(requested_backend, selected, checks)


def print_report(report: DiagnosticReport) -> None:
    for check in report.checks:
        print(f"[{check.status}] {check.name}: {check.message}")
    if report.selected_backend:
        print(f"[READY] 選択backend: {report.selected_backend}")
    else:
        print("[NOT_READY] 利用可能なMapping backendがありません")
