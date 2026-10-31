"""実機入口の一連の制御を、SDK送信なし・仮想時計で検査する。"""

import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "real")]
import align_button
from alignment import Observation
from button_feedback import LampFrame


@pytest.fixture
def live(monkeypatch, tmp_path):
    clock = SimpleNamespace(t=10.0)
    state = SimpleNamespace(q=np.zeros(29), attempts=[], captures=[], outcomes=[True],
                            return_fail=False, release_fail=False, lose_tracking=False)
    state.q[22:25] = [0.1, -0.3, 1.0]
    home = state.q[22:].copy()
    args = SimpleNamespace(clearance=0.05, stroke=0.0015, output=tmp_path / "live.json",
                           alignment_mode="single", normal_motion="smooth", lamp_min_rise=30.0,
                           correction_interval=2.0, correction_duration=0.6,
                           settle_time=0.6, phase_timeout=60.0)
    calibration = {"tip_radius_m": 0.015, "reference_waist_q_rad": [0, 0, 0],
                   "reference_imu_rpy_rad": [0, 0, 0]}
    from vision import ButtonTarget

    target = ButtonTarget((0.35, -0.25, 1.0), (1, 0, 0), 0.5, 1.0, 1.0)
    kin = SimpleNamespace(data=SimpleNamespace(qpos=state.q.copy()), qadr=np.arange(22, 29))
    kin.update = lambda q: setattr(kin.data, "qpos", np.array(q).copy())
    kin.update_fixed_base = lambda q, rpy: kin.update(q)
    kin.tip_position = lambda: kin.data.qpos[22:25].copy()
    kin.solve = lambda xyz, normal, **kwargs: SimpleNamespace(joint_angles=np.r_[xyz, np.zeros(4)])
    kin.validate_path = lambda *args: None

    class Arm:
        def current_arm(self):
            return tuple(state.q[15:29])

        def set_hold(self, current):
            pass

        def current_configuration(self):
            return tuple(state.q), (0, 0, 0), clock.t, 0, 0

        def release(self):
            if state.release_fail:
                raise RuntimeError("release lost")

    class Motion(align_button.ArmMotion):
        def enable(self):
            state.motion = self

        def wait(self, seconds):
            self.check()
            if not state.lose_tracking:
                state.q[22:] = self.current_target()
            clock.t += seconds

        def replay(self, targets, validate):
            state.return_targets = np.array(targets)
            if state.return_fail is True:
                raise ValueError("帰路で衝突")
            super().replay(targets, validate)
            if state.return_fail == "tracking":
                state.lose_tracking = True
                state.q[22] += 0.1

    class Camera:
        frame_id = 0

        def sample(self, level):
            self.frame_id += 1
            return LampFrame(np.full((40, 40, 3), level, dtype=np.uint8),
                             (10, 10, 30, 30), clock.t, self.frame_id)

        def observe(self):
            state.captures.append(clock.t)
            self.last_lamp_frame = self.sample(100)
            return Observation(target, clock.t, len(state.captures))

        def lamp_frames(self, after, timeout_s):
            attempt = len(state.attempts)
            state.attempts.append({"stroke": args.stroke, "after": after,
                                   "q_cmd": state.motion.current_target().copy()})
            outcome = state.outcomes[attempt]
            if outcome == "lost":
                raise RuntimeError("対象消失")
            levels = [180]*3 if outcome is True else [100]*3 if outcome is False else [100, 180, 180]
            for level in levels:
                clock.t += 0.02
                yield self.sample(level)

    monkeypatch.setattr(align_button, "ArmMotion", Motion)
    monkeypatch.setattr(align_button, "time", SimpleNamespace(monotonic=lambda: clock.t))

    def run():
        return align_button.execute_alignment(Arm(), Camera(), kin, calibration, args)

    state.run, state.args, state.home, state.kin = run, args, home, kin
    return state


@pytest.mark.parametrize("outcomes, expected, retries", [([True], True, 0),
    ([False, True], True, 1), ([False, False], False, 1), ([None], None, 0), (["lost"], None, 0)])
def test_lamp_result_retry_limit_and_reverse_return(live, outcomes, expected, retries):
    live.outcomes = outcomes
    report = live.run()
    assert report["lamp_confirmed"] is expected
    assert report["completed"] and report["arm_returned"]
    assert report["retry_count"] == retries
    assert len(live.attempts) == retries+1
    assert all(attempt["stroke"] == 0.0015 for attempt in live.attempts)
    np.testing.assert_allclose(live.q[22:], live.home)
    np.testing.assert_allclose(live.return_targets[-1], live.home)
    assert report["physical_press_success"] is None
    assert report["measurement_count"] == 2  # 確認画像で押下目標を更新しない。
    assert np.max(np.abs(np.diff(live.return_targets, axis=0)))*50 <= 0.8+1e-12
    assert all(sample["image_path"] for check in report["lamp_checks"] for sample in check["frames"])
    # 再試行でも基準画像・往路の記録は最初のものを使う。
    assert report["lamp_baseline"]["level"] == 100
    np.testing.assert_allclose(live.motion.approach.samples[-1][1], live.return_targets[0])


def test_return_failure_does_not_move_back_towards_button(live):
    live.return_fail = True
    with pytest.raises(ValueError, match="帰路で衝突"):
        live.run()
    report = json.loads(live.args.output.read_text())
    assert not report["completed"] and not report["arm_returned"]
    assert "retreat_error" not in report
    # 接続した姿勢のまま止め、ボタン方向へのIK再計算・退避を行わない。
    np.testing.assert_allclose(live.motion.current_target(), live.return_targets[0])


def test_release_failure_cannot_be_reported_as_completed(live):
    live.release_fail = True
    report = live.run()
    assert report["lamp_confirmed"] is True and report["arm_returned"]
    assert not report["completed"] and report["release_error"] == "release lost"


def test_home_command_without_measured_return_is_not_success(live):
    live.return_fail = "tracking"
    with pytest.raises(RuntimeError, match="開始姿勢"):
        live.run()
    report = json.loads(live.args.output.read_text())
    assert not report["completed"] and not report["arm_returned"]
    assert report["return_error_rad"] > np.deg2rad(2)


def test_replay_stops_on_late_update_without_sending_next_target(live, monkeypatch):
    live.run()
    initial = live.motion.current_target().copy()
    ticks = iter([0.0, 0.101])
    monkeypatch.setattr(align_button, "time", SimpleNamespace(monotonic=lambda: next(ticks)))
    with pytest.raises(RuntimeError, match="100ms"):
        live.motion.replay([np.full(7, 0.2)], lambda _: None)
    np.testing.assert_array_equal(live.motion.current_target(), initial)


def test_snapshot_failure_does_not_block_lamp_or_return(live, monkeypatch):
    cv2 = pytest.importorskip("cv2")
    monkeypatch.setattr(cv2, "imwrite", lambda *args: False)
    report = live.run()
    assert report["completed"] and report["arm_returned"] and report["lamp_confirmed"]
    assert len(report["snapshot_errors"]) == 4


def test_invalid_threshold_rejected_before_model_or_dds():
    assert align_button.main(["--model", "unused", "--weights", "unused", "--base-transform", "unused",
                              "--stroke", ".0015", "--output", "/tmp/unused.json",
                              "--lamp-min-rise", "nan"]) == 2


@pytest.mark.parametrize("confirmed, returned, completed, code", [(True, True, True, 0),
    (False, True, True, 2), (None, True, True, 2), (True, False, True, 2), (True, True, False, 2)])
def test_cli_success_requires_lamp_return_and_release(monkeypatch, confirmed, returned, completed, code):
    calibration = {"tip_offset_m": [0.11, 0, 0], "tip_radius_m": 0.015,
                   "pelvis_height_m": 0.79, "reference_waist_q_rad": [0, 0, 0],
                   "reference_imu_rpy_rad": [0, 0, 0]}
    monkeypatch.setattr(align_button, "read_calibration", lambda _: (calibration, None))
    monkeypatch.setattr(align_button, "load_robot", lambda *args: (object(), np.zeros(29)))
    kin = SimpleNamespace(update_fixed_base=lambda *args: None)
    monkeypatch.setattr(align_button, "ArmKinematics", lambda _: kin)
    monkeypatch.setattr(align_button, "J1CameraRuntime", lambda _: object())
    monkeypatch.setattr(align_button, "CalibratedHeadCamera", lambda *args: object())
    monkeypatch.setattr(align_button, "ArmSdk", lambda _: SimpleNamespace(
        stop_walking=lambda: None, current_configuration=lambda: None))
    monkeypatch.setattr(align_button, "validate_stationary_pose", lambda *args: None)
    monkeypatch.setattr(align_button, "RgbdLocalizer", lambda *args, **kw: SimpleNamespace(close=lambda: None))
    monkeypatch.setattr(align_button, "time", SimpleNamespace(sleep=lambda _: None,
                        time=time.time, monotonic=time.monotonic))
    monkeypatch.setattr("builtins.input", lambda _: "YES")
    report = {"lamp_confirmed": confirmed, "arm_returned": returned, "completed": completed,
              "release_completed": completed,
              "result_reason": "test result", "release_error": None if completed else "test release"}
    monkeypatch.setattr(align_button, "execute_alignment", lambda *args, **kwargs: report)
    assert align_button.main(["--model", "unused", "--weights", "unused", "--base-transform", "unused",
                              "--stroke", ".0015", "--output", "/tmp/unused.json",
                              "--execute", "--calibrated"]) == code
