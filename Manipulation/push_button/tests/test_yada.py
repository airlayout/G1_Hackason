"""採点接続の取り違え・古い画像・車いす用ボタンの誤選択を検証する。"""

import importlib.util
import sys
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim"))

from yada import YadaButtonController, color_boxes, select_target
from yada_paths import find_yada_root
from evaluate_yada import build_command
from vision import ButtonTarget


def target(z, y=0, x=0.32):
    return ButtonTarget((x, y, z), (1, 0, 0), 0.3, 1, 1)


def test_instruction_selects_general_pair_and_never_wheelchair_pair():
    buttons = [target(-0.03), target(0.20), target(-0.11), target(0.28)]
    assert select_target(buttons, "up").face_xyz[2] == 0.28
    assert select_target(buttons, "down").face_xyz[2] == 0.20
    with pytest.raises(ValueError, match="上下ボタン"):
        select_target([target(-0.03), target(-0.11)], "up")


def test_ambiguous_or_missing_button_is_not_guessed():
    with pytest.raises(ValueError, match="一意"):
        select_target([target(0.28)], "down")
    with pytest.raises(ValueError, match="一意"):
        select_target([target(0.28), target(0.20), target(0.28, y=0.03)], "up")
    with pytest.raises(ValueError, match="追跡"):
        select_target([target(0.20)], "up", previous=np.array([0.32, 0, 0.28]))


def test_old_or_repeated_frames_cannot_complete_remeasurement():
    controller = YadaButtonController.__new__(YadaButtonController)
    controller.last_image_t = 1.0
    controller.capture_after = 1.0
    controller.samples = deque(maxlen=3)
    for _ in range(3):
        assert controller._measure(SimpleNamespace(t=1.1, image_t=1.0)) is None
    assert not controller.samples
    with pytest.raises(ValueError, match="古すぎる"):
        controller._measure(SimpleNamespace(t=2.0, image_t=1.0))
    with pytest.raises(ValueError, match="時刻"):
        controller._measure(SimpleNamespace(t=1.0, image_t=1.1))


def test_color_detection_does_not_select_blue_wheelchair_symbol():
    rgb = np.full((240, 320, 3), 20, dtype=np.uint8)
    yy, xx = np.mgrid[:240, :320]
    rgb[(xx-160)**2 + (yy-60)**2 < 15**2] = 170
    rgb[(xx-160)**2 + (yy-130)**2 < 15**2] = 170
    rgb[170:200, 145:175] = [20, 50, 180]
    boxes = color_boxes(rgb)
    assert len(boxes) == 2
    assert all(b[3] < 170 for b in boxes)


def test_dds_does_not_silently_run_basic_when_realistic_requested():
    with pytest.raises(ValueError, match="basic"):
        build_command(Path("/tmp/yada"), "dds", ["--set=realistic"])


def test_explicit_missing_environment_does_not_fall_back(tmp_path):
    with pytest.raises(FileNotFoundError):
        find_yada_root(str(tmp_path))


def test_continuous_press_stops_when_state_updates_are_late():
    controller = YadaButtonController.__new__(YadaButtonController)
    controller.task = SimpleNamespace(control_dt=0.02)
    controller.phase = "press"
    controller.last_t = 1.0
    with pytest.raises(ValueError, match="100ms"):
        controller.act(SimpleNamespace(q=np.zeros(29), t=1.12))


def test_scoring_profile_is_rejected_for_real_robot(monkeypatch):
    # APIの型だけを用意し、機体やSDKを起動せずに実機接続拒否を確認する。
    monkeypatch.setitem(sys.modules, "contest.interface", SimpleNamespace(Action=object, Agent=object))
    spec = importlib.util.spec_from_file_location("test_scoring_agent", ROOT / "sim/yada_agent/agent.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    agent = module.PushButtonAgent.__new__(module.PushButtonAgent)
    agent.controller = SimpleNamespace(reset=lambda task: pytest.fail("採点設定を実機へ適用した"))
    with pytest.raises(ValueError, match="採点環境専用"):
        agent.reset(SimpleNamespace(sim="real"))
