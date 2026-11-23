"""pc2_probe の、G1 が無くても確かめられる部分のテスト（DDS の受信は PC2 でしか確かめられない）。"""
import ast
import importlib.util
import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "pc2_probe.py"


@pytest.fixture(scope="module")
def probe():
    spec = importlib.util.spec_from_file_location("pc2_probe", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_syntax_is_python38_compatible():
    ast.parse(SCRIPT.read_text(encoding="utf-8"), feature_version=(3, 8))


def test_percentile_and_empty(probe):
    assert probe.percentile([], 0.95) is None
    assert probe.percentile([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], 0.95) == 10


def test_load_option_requires_dds(probe):
    with pytest.raises(SystemExit):
        probe.parse_args(["--load", "cpu:1"])
    with pytest.raises(SystemExit):
        probe.parse_args(["--seconds", "61", "--dds"])


def test_load_spec_rejects_bad_input(probe):
    for spec in ("cpu", "gpu:1", "yolo:"):
        with pytest.raises(ValueError):
            probe.Load(spec, 1)


def test_cpu_load_starts_capped_and_stops(probe):
    load = probe.Load("cpu:999", 1)
    info = load.start()
    try:
        assert 1 <= info["procs"] <= info["cap"]
        assert all(p.poll() is None for p in load.procs)
    finally:
        load.stop()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and any(p.poll() is None for p in load.procs):
        time.sleep(0.05)
    assert all(p.poll() is not None for p in load.procs)


def test_pepper_unreachable_is_reported_not_raised(probe):
    result = probe.reach_pepper("127.0.0.1", 9, tries=2, timeout=0.5)
    assert result["ok"] is False and len(result["connect_ms"]) == 2


def test_cli_without_dds_prints_json_report():
    out = subprocess.run([sys.executable, "-I", str(SCRIPT), "--pepper-ip", "127.0.0.1", "--pepper-port", "9"],
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    report = json.loads(out.stdout)
    assert "python" in report["sysinfo"] and report["pepper"]["ok"] is False


def test_stdin_run_works_like_ssh(probe):
    """SSH の標準入力で流し込む使い方（python3 -）でも動くこと。"""
    out = subprocess.run([sys.executable, "-I", "-"], input=SCRIPT.read_text(encoding="utf-8"),
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert "sysinfo" in json.loads(out.stdout)
