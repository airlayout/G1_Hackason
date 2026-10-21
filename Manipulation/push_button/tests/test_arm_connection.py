"""腕の接続確認: 実測による成否、異常時の解除、SDKのみの入口を検査。"""

import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "real")]
from check_arm_connection import ArmConnectionCheck, check_state, main, make_plan, validate_settings
import check_arm_connection
from push_button import ArmSdk


class Clock:
    def __init__(self):
        self.t = 0.

    def now(self):
        return self.t

    def wait(self, duration):
        self.t += duration


class FakeArm:
    def __init__(self, clock, *, follow=True, stale_after=None, interrupt=False):
        self.clock, self.follow, self.stale_after, self.interrupt = clock, follow, stale_after, interrupt
        self.q = [0.]*29
        self.q[22] = .3
        self.initial = self.q.copy()
        self.commands = []
        self.stops = 0
        self.releases = 0
        self.info = {"mode_machine": 5, "motor_count": 35, "fsm_id": 501}

    def connection_info(self):
        return self.info

    def stop_walking(self):
        self.stops += 1

    def current_configuration(self):
        stamp = self.clock.now()
        if self.stale_after is not None and stamp >= self.stale_after:
            stamp -= 1
        return tuple(self.q), (0., 0., 0.), stamp, 0., 0.

    def set_hold(self, arm):
        self.hold = arm

    def publish(self, right, weight, require_fresh_state=True):
        if self.interrupt and self.clock.now() > 3 and require_fresh_state:
            self.interrupt = False
            raise KeyboardInterrupt
        self.commands.append((self.clock.now(), tuple(right), weight, require_fresh_state))
        if self.follow:
            self.q[22:29] = right

    def release(self):
        self.releases += 1

    def close(self):
        self.closed = True


def test_small_raise_and_return_are_confirmed_from_observed_joint_angles():
    clock = Clock()
    arm = FakeArm(clock)
    result = ArmConnectionCheck(arm, now=clock.now, wait=clock.wait).execute()
    assert result["completed"] and result["movement_confirmed"] and result["return_confirmed"]
    assert result["observed_raise_deg"] == pytest.approx(5.)
    assert result["return_max_error_deg"] == pytest.approx(0.)
    assert arm.q == arm.initial
    assert arm.commands[0][2] == arm.commands[-1][2] == 0.
    assert result["release_sent"]
    assert arm.stops == 1
    assert {p["phase"] for p in result["phases"]} == {"enable", "raise", "hold", "return", "settle", "release"}
    for _, right, _, _ in arm.commands:
        assert right[1:] == tuple(arm.initial[23:29])
        assert arm.initial[22]-math.radians(5)-1e-10 <= right[0] <= arm.initial[22]
    # 最大速度は小動作用の低い上限内。重複時刻の端点は速度差分から除く。
    for a, b in zip(arm.commands, arm.commands[1:]):
        if b[0]-a[0] > 1e-6:
            assert abs(b[1][0]-a[1][0])/(b[0]-a[0]) <= .15


def test_sending_commands_without_actual_motion_is_not_success():
    clock = Clock()
    arm = FakeArm(clock, follow=False)
    result = ArmConnectionCheck(arm, now=clock.now, wait=clock.wait).execute()
    assert not result["completed"] and not result["movement_confirmed"]
    assert result["observed_raise_deg"] == 0.
    assert result["return_confirmed"] and result["release_sent"]
    assert len([p for p in result["phases"] if p["phase"] == "raise"]) == 1


@pytest.mark.parametrize("failure", ["stale", "interrupt"])
def test_state_loss_or_interrupt_attempts_weight_release_without_blind_return(failure):
    clock = Clock()
    arm = FakeArm(clock, stale_after=3. if failure == "stale" else None, interrupt=failure == "interrupt")
    result = ArmConnectionCheck(arm, now=clock.now, wait=clock.wait).execute()
    assert not result["completed"]
    assert result["release_sent"] and arm.commands[-1][2] == 0.
    assert "return" not in [p["phase"] for p in result["phases"]]
    assert all(command[1] == arm.commands[-1][1] for command in arm.commands if command[3] is False)
    if failure == "interrupt":
        assert result["interrupted"]


@pytest.mark.parametrize("key,value", [("fsm_id", 1), ("mode_machine", 3), ("motor_count", 23)])
def test_wrong_robot_or_standing_mode_is_rejected_before_commands(key, value):
    clock = Clock()
    arm = FakeArm(clock)
    arm.info[key] = value
    result = ArmConnectionCheck(arm, now=clock.now, wait=clock.wait).execute()
    assert not result["completed"]
    assert not arm.commands and arm.stops == 0


@pytest.mark.parametrize("angle,duration", [(0, 2), (12, 2), (5, 1), (5, 6), (math.nan, 2), (5, math.inf)])
def test_excessive_or_invalid_motion_settings_are_rejected(angle, duration):
    with pytest.raises(ValueError):
        validate_settings(angle, duration)


def test_stationarity_guard_reports_body_motion_and_bad_telemetry():
    clock = Clock()
    arm = FakeArm(clock)
    baseline = arm.current_configuration()
    bad = list(baseline)
    bad[3] = .2
    with pytest.raises(RuntimeError, match="静止"):
        check_state(bad, clock.now())
    bad = list(baseline)
    bad[0] = list(bad[0])
    bad[0][12] += .03
    with pytest.raises(RuntimeError, match="腰"):
        check_state(bad, clock.now(), baseline)
    bad[0][12] = 0.
    bad[0][23] = math.nan
    with pytest.raises(ValueError, match="状態データ"):
        check_state(bad, clock.now())
    arm.q[22] = -1.
    with pytest.raises(ValueError, match="開始範囲"):
        make_plan(arm.current_configuration(), 5., 2., clock.now())


def test_failed_dds_write_does_not_mark_command_as_sent():
    arm = ArmSdk.__new__(ArmSdk)
    arm._hold = [0.]*14
    arm._last_right = (0.,)*7
    arm._last_weight = .25
    arm._cmd = SimpleNamespace(motor_cmd=[SimpleNamespace(q=0.) for _ in range(35)], crc=0)
    arm._crc = SimpleNamespace(Crc=lambda _: 123)
    arm._publisher = SimpleNamespace(Write=lambda _: False)
    with pytest.raises(RuntimeError, match="送信に失敗"):
        arm.publish((.1,)*7, .5, require_fresh_state=False)
    assert arm._last_weight == .25
    assert arm._last_right == (0.,)*7


def test_common_entry_runs_offline_with_no_third_party_packages(tmp_path):
    clock = Clock()
    q = FakeArm(clock).q
    path, output = tmp_path / "state.json", tmp_path / "check.json"
    path.write_text(json.dumps({"motor_q_rad": q, "imu_rpy_rad": [0, 0, 0]}))
    completed = subprocess.run([sys.executable, "-S", str(ROOT / "real/run_push.py"), "--task", "check-arm",
                                "--state-json", str(path), "--output", str(output)],
                               text=True, capture_output=True, timeout=10)
    assert completed.returncode == 0, completed.stderr
    report = json.loads(output.read_text())
    assert not report["connection_verified"] and not report["completed"]
    assert report["plan"]["angle_deg"] == 5.


def test_saved_state_cannot_be_used_for_execution(tmp_path):
    output = tmp_path / "check.json"
    assert main(["--execute", "--state-json", "unused.json", "--output", str(output)]) == 2
    assert "オフライン計画専用" in json.loads(output.read_text())["error"]


@pytest.mark.parametrize("execute", [False, True])
def test_live_preview_and_cancel_do_not_send_motion_commands(monkeypatch, tmp_path, execute):
    clock = Clock()
    arm = FakeArm(clock)
    monkeypatch.setattr(check_arm_connection.time, "monotonic", clock.now)
    monkeypatch.setattr(check_arm_connection, "ArmSdk", lambda _: arm)
    monkeypatch.setattr("builtins.input", lambda _: "NO")
    output = tmp_path / "check.json"
    args = ["--network-interface", "test-only", "--output", str(output)]
    if execute:
        args.append("--execute")
    assert main(args) == 0
    report = json.loads(output.read_text())
    assert report["connection_verified"]
    assert not arm.commands and arm.stops == 0 and arm.closed
    assert not report["completed"]
    if execute:
        assert report["cancelled"]
