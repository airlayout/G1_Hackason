"""正解枠で制御・接触を切り分ける試験。YOLOの精度評価は実行記録を参照。"""

import json
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))


@pytest.mark.parametrize("x,y", [(.33, -.25), (.32, -.25), (.34, -.25), (.33, -.26), (.33, -.24)])
def test_free_bottle_moves_five_cm_and_arm_returns_without_table_contact(x, y, tmp_path):
    model = os.environ.get("G1_MUJOCO_MODEL")
    if not model:
        pytest.skip("G1_MUJOCO_MODELに外部のMenagerieモデルを指定してください")
    pytest.importorskip("mujoco")
    from run_bottle_mujoco import main

    path = tmp_path / "report.json"
    code = main(["--model", model, "--detector", "oracle", "--bottle-x", str(x),
                 "--bottle-y", str(y), "--report-out", str(path)])
    report = json.loads(path.read_text())
    assert code == 0, report.get("error", report.get("verification_error", report))
    assert report["home_completed"] and report["visual_success"]
    assert report["measurement_count"] == 3
    evaluation = report["evaluation"]
    assert evaluation["ground_truth_used_for_control"] is False
    assert evaluation["bottle_displacement_m"][0] == pytest.approx(.05, abs=.01)
    assert abs(evaluation["bottle_displacement_m"][1]) < .01
    assert evaluation["bottle_tilt_deg"] < 15
    assert evaluation["max_unwanted_table_force_n"] < .1


def test_unreachable_push_is_rejected_before_arm_commands(tmp_path):
    model = os.environ.get("G1_MUJOCO_MODEL")
    if not model:
        pytest.skip("G1_MUJOCO_MODELを指定してください")
    from run_bottle_mujoco import main

    path = tmp_path / "unreachable.json"
    assert main(["--model", model, "--detector", "oracle", "--bottle-x", ".45",
                 "--report-out", str(path)]) == 1
    report = json.loads(path.read_text())
    assert not report["motion_completed"]
    assert [p["phase"] for p in report["phases"]] == ["measure"]
    assert "IK" in report["error"]


def test_visual_confirmation_failure_still_returns_arm_home(monkeypatch, tmp_path):
    model = os.environ.get("G1_MUJOCO_MODEL")
    if not model:
        pytest.skip("G1_MUJOCO_MODELを指定してください")
    from run_bottle_mujoco import main
    from bottle import BottleLocalizer

    observe = BottleLocalizer.observe

    def lose_after_push(self, *args, **kwargs):
        if self.after_push:
            raise RuntimeError("対象消失")
        return observe(self, *args, **kwargs)

    monkeypatch.setattr(BottleLocalizer, "observe", lose_after_push)
    path = tmp_path / "lost.json"
    assert main(["--model", model, "--detector", "oracle", "--report-out", str(path)]) == 1
    report = json.loads(path.read_text())
    assert report["home_completed"] and report["completed"]
    assert report["visual_success"] is None
    assert report["physical_push_success"] is False
    assert report["verification_error"] == "対象消失"
