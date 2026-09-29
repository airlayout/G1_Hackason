"""実機送信前に必要な経路検証の回帰試験。"""

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "real"))

from push_button import ArmSdk, check_speed
from trajectory import ARM_JOINTS, DEFAULT_DURATIONS, PHASES, load_plan


def example_plan() -> dict:
    return {
        "schema": "g1-push-button-plan-v1",
        "arm_joint_names": list(ARM_JOINTS),
        "poses": {phase: [0.0] * 7 for phase in PHASES},
        "durations": DEFAULT_DURATIONS,
        "sim_result": {"success": True, "max_stroke_m": 0.0073},
    }


def test_rejects_unverified_or_invalid_joint_plan(tmp_path: Path) -> None:
    path = tmp_path / "plan.json"
    plan = example_plan()
    plan["sim_result"]["success"] = False
    path.write_text(json.dumps(plan))
    with pytest.raises(ValueError, match="押下が確認されていない"):
        load_plan(path)
    plan["sim_result"]["success"] = True
    plan["poses"]["press"][0] = math.nan
    path.write_text(json.dumps(plan))
    with pytest.raises(ValueError, match="関節範囲外"):
        load_plan(path)


def test_rejects_fast_motion() -> None:
    with pytest.raises(ValueError, match="上限"):
        check_speed((0.0,) * 7, (1.0,) * 7, 0.5)


def test_arm_packet_does_not_command_legs_or_waist() -> None:
    class FakePublisher:
        def __init__(self) -> None:
            self.writes = 0

        def Write(self, _command) -> None:
            self.writes += 1

    arm = ArmSdk.__new__(ArmSdk)
    arm._cmd = SimpleNamespace(motor_cmd=[SimpleNamespace(q=0.0) for _ in range(35)], crc=0)
    arm._crc = SimpleNamespace(Crc=lambda _command: 123)
    arm._publisher = FakePublisher()
    arm._hold = [0.1] * 14
    arm._last_right = None
    arm._last_weight = 0.0
    arm.publish((0.2,) * 7, 1.0, require_fresh_state=False)
    assert all(arm._cmd.motor_cmd[index].q == 0.0 for index in range(15))
    assert all(arm._cmd.motor_cmd[index].q == 0.1 for index in range(15, 22))
    assert all(arm._cmd.motor_cmd[index].q == 0.2 for index in range(22, 29))
    assert arm._cmd.motor_cmd[29].q == 1.0
    assert arm._publisher.writes == 1
