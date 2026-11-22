from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "run_named_reaction", ROOT / "scripts" / "run_named_reaction.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_frustration_profile_is_real_validated_and_excludes_legs() -> None:
    profile = MODULE.load_profile("frustration")
    assert profile["real_g1_validated"] is True
    assert profile["arms_amplitude"] == 0.5
    assert profile["waist_amplitude"] == 0.25
    assert profile["legs"] == "excluded"
    assert profile["requires_stop"] is True


def test_unknown_profile_is_rejected() -> None:
    with pytest.raises(ValueError, match="Unknown named reaction"):
        MODULE.load_profile("unknown")


def test_runner_command_uses_existing_validated_runtime() -> None:
    args = argparse.Namespace(
        network_interface="eth0", expected_arm_pid=2899, discovery_peer=[],
        engine_authorized=True,
    )
    command = MODULE.runner_command(
        args, MODULE.load_profile("frustration"), Path("/tmp/new-run")
    )
    joined = " ".join(map(str, command))
    assert "run_real_reaction.py" in joined
    assert "--motion-layout upper-body" in joined
    assert "--waist-scale 0.25" in joined
    assert "--prevalidated-runtime" in command
    assert "--final-gate-token" not in command


def test_standalone_runner_retains_final_manual_gate() -> None:
    args = argparse.Namespace(
        network_interface="eth0", expected_arm_pid=2899, discovery_peer=[],
        engine_authorized=False,
    )
    command = MODULE.runner_command(
        args, MODULE.load_profile("frustration"), Path("/tmp/new-run")
    )
    assert command[-2:] == ["--final-gate-token", MODULE.MANUAL_GATE]


def test_remote_engine_mode_is_machine_readable_but_attended_mode_is_interactive() -> None:
    base = dict(
        reaction="frustration", network_interface="eth0", expected_arm_pid=2899,
        discovery_peer=[], remote_pythonpath="/tmp/deps",
        remote_library_path="/tmp/lib", remote_root="/tmp/repo",
        ssh_control=None, ssh_target="unitree@example", timeout=180.0,
    )
    attended = MODULE.remote_command(argparse.Namespace(**base, engine_authorized=False))
    engine = MODULE.remote_command(argparse.Namespace(**base, engine_authorized=True))
    assert "--json" not in attended[-1]
    assert "--engine-authorized" not in attended[-1]
    assert "--json" in engine[-1]
    assert "--engine-authorized" in engine[-1]
