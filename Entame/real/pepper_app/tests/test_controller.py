import time

import pytest
from conftest import frame_of

from pepperapp.controller import DEFAULT_RULES, AppController, Settings, build_robot
from pepperapp.robot import DryRunRobot
from pepperapp.rules import Rule


class Closable:
    name = "closable"

    def __init__(self):
        self.closed = False

    def read(self):
        time.sleep(0.05)
        import numpy as np
        return np.zeros((48, 64, 3), dtype=np.uint8)

    def close(self):
        self.closed = True


def _controller(tmp_path, **factories):
    return AppController(rules_path=tmp_path / "rules.yaml", default_rules_path=DEFAULT_RULES,
                         detector_factory=lambda config: (lambda frame: frame_of(width=64, height=48)),
                         **factories)


def test_start_snapshot_stop(tmp_path):
    source = Closable()
    controller = _controller(tmp_path, source_factory=lambda s: source,
                             robot_factory=lambda s: DryRunRobot(0.0))
    controller.start(Settings())
    deadline = time.monotonic() + 3
    while not controller.snapshot().running and time.monotonic() < deadline:
        time.sleep(0.05)
    assert controller.snapshot().running and controller.snapshot().reacting  # dry-run reacts
    controller.stop()
    assert source.closed and not controller.running


def test_two_tabs_starting_at_once_leave_one_loop(tmp_path):
    import threading
    sources = []

    def make_source(settings):
        time.sleep(0.2)  # building takes a while, like loading the model
        source = Closable()
        sources.append(source)
        return source

    controller = _controller(tmp_path, source_factory=make_source,
                             robot_factory=lambda s: DryRunRobot(0.0))
    threads = [threading.Thread(target=controller.start, args=(Settings(),)) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert len(sources) == 2 and [s.closed for s in sources].count(False) == 1
    controller.stop()
    assert all(s.closed for s in sources)


def test_failed_start_closes_what_was_opened(tmp_path):
    source = Closable()

    def broken_robot(settings):
        raise ConnectionError("Pepper unreachable")

    controller = _controller(tmp_path, source_factory=lambda s: source, robot_factory=broken_robot)
    with pytest.raises(ConnectionError):
        controller.start(Settings(robot_mode="pepper", pepper_ip="10.0.0.9"))
    assert source.closed and not controller.running


def test_pepper_mode_starts_without_reacting(tmp_path):
    controller = _controller(tmp_path, source_factory=lambda s: Closable(),
                             robot_factory=lambda s: DryRunRobot(0.0))
    controller.start(Settings(robot_mode="pepper", pepper_ip="10.0.0.9"))
    try:
        assert not controller.snapshot().reacting
    finally:
        controller.stop()


def test_rules_are_saved_and_reset(tmp_path):
    controller = _controller(tmp_path)
    default = controller.rules()
    controller.save_rules((Rule("wave", "wave"),))
    assert controller.rules() == (Rule("wave", "wave"),)
    assert controller.reset_rules() == default


def test_settings_validation_and_dry_run_robot():
    with pytest.raises(ValueError):
        Settings(source_kind="ftp")
    with pytest.raises(ValueError):
        Settings(robot_mode="walk")
    assert isinstance(build_robot(Settings()), DryRunRobot)
