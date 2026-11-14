"""経路座標、支持面、29軸への方策接続と、押下前の停止条件を検証する。"""

import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim"))

from walking import DEFAULT_ASSETS, WalkingPolicy, Waypoint, support_margin, velocity_to_waypoint, wrap_angle
from run_walk_and_press_mujoco import WalkAndPressExperiment, load_route


def test_waypoint_command_uses_robot_heading_and_shortest_yaw_turn():
    # 世界座標で北へ向かう目標は、北向きの機体には前進になる。
    command = velocity_to_waypoint([0, 0], np.pi/2, Waypoint(0, 2, 90))
    assert np.allclose(command, [0.3, 0, 0])
    command = velocity_to_waypoint([0, 0], np.deg2rad(179), Waypoint(0, 0, -179))
    assert command[2] == pytest.approx(1.5*np.deg2rad(2))
    assert wrap_angle(3*np.pi) == pytest.approx(np.pi)
    command = velocity_to_waypoint([0, 0], 0, Waypoint(-10, 10, 90))
    assert np.allclose(command, [-0.15, 0.15, 0.4])


@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_waypoint_rejects_nonfinite_coordinates(value):
    with pytest.raises(ValueError, match="有限値"):
        Waypoint(value, 0)


def test_support_uses_convex_polygon_instead_of_its_bounding_box():
    triangle = [(0, 0), (1, 0), (0, 1), (0, 0)]
    assert support_margin(triangle, (0.2, 0.2)) == pytest.approx(0.2)
    assert support_margin(triangle, (0.8, 0.8)) < 0
    assert support_margin(triangle, (0.5, 0.5)) == pytest.approx(0)
    assert support_margin([(0, 0), (1, 0)], (0.2, 0)) is None


@pytest.mark.parametrize("route", [[], {}, [{"x_m": 0.2, "y_m": 0}], [{"x_m": 0, "y_m": 0, "yaw_deg": 10}]])
def test_route_must_end_in_button_approach_area(tmp_path, route):
    path = tmp_path / "route.json"
    path.write_text(json.dumps(route))
    with pytest.raises(ValueError):
        load_route(path)


@pytest.fixture
def model_path():
    path = os.environ.get("G1_MUJOCO_MODEL")
    if not path:
        pytest.skip("G1_MUJOCO_MODEL にMenagerieモデルを指定してください")
    pytest.importorskip("torch")
    pytest.importorskip("yaml")
    if not (DEFAULT_ASSETS / "motion.pt").exists():
        pytest.skip("Navigation/sim/fetch_assets.sh で固定版の歩行資産を取得してください")
    return path


def sim_args(path, **overrides):
    values = dict(model=path, walk_assets=str(DEFAULT_ASSETS), route_json=None,
                  start_pose=[-1.2, 0, 0], navigation_timeout=40, button_direction="up",
                  stroke=0.0015, offset=[0, 0, 0], yaw_deg=0,
                  drift_after_approach=[0, 0, 0], clearance=0.05, weights=None,
                  video_out=None, gif_out=None, gif_fps=12, fixed_base=False,
                  elevator_front=True, alignment_mode="single", normal_motion="smooth",
                  correction_interval=2.0, correction_duration=0.6, settle_time=0.6, phase_timeout=60)
    values.update(overrides)
    return SimpleNamespace(**values)


def test_leg_adapter_ignores_arms_and_restores_without_resetting_body(model_path):
    import mujoco
    from run_mujoco import DEFAULT_TIP_OFFSET, build_model

    model, stand = build_model(Path(model_path), 0.36, -0.25, 1, 0.0015, DEFAULT_TIP_OFFSET, False)
    data = mujoco.MjData(model)
    data.qpos[:7], data.qpos[7:36], data.ctrl[:] = [0, 0, 0.793, 1, 0, 0, 0], stand, stand
    names = ("actuator_gainprm", "actuator_biasprm", "actuator_biastype", "actuator_ctrllimited",
             "dof_frictionloss", "dof_damping", "jnt_actfrcrange")
    original = {name: getattr(model, name).copy() for name in names}
    walker = WalkingPolicy(model, data)
    data.qpos[walker.qadr] = walker.defaults
    first = walker.observation()
    data.qpos[model.joint("right_elbow_joint").qposadr[0]] += 0.1
    assert np.array_equal(first, walker.observation())
    upper = np.setdiff1d(np.arange(model.nu), walker.actuator)
    assert np.array_equal(model.actuator_gainprm[upper], original["actuator_gainprm"][upper])
    assert np.array_equal(model.jnt_actfrcrange, original["jnt_actfrcrange"])
    mujoco.mj_forward(model, data)
    for _ in range(20):
        walker.step()
    assert walker.action.shape == (12,) and np.all(np.isfinite(walker.action))
    before_q, before_v, upper_ctrl = data.qpos.copy(), data.qvel.copy(), data.ctrl[upper].copy()
    walker.restore_position_control()
    assert np.array_equal(data.qpos, before_q) and np.array_equal(data.qvel, before_v)
    assert np.array_equal(data.ctrl[walker.actuator], data.qpos[walker.qadr])
    assert np.array_equal(data.ctrl[upper], upper_ctrl)
    assert not np.any(data.xfrc_applied) and not np.any(data.qfrc_applied)
    for name in names:
        assert np.array_equal(getattr(model, name), original[name])


def test_navigation_timeout_aborts_before_camera_measurement_or_press(model_path):
    experiment = WalkAndPressExperiment(sim_args(model_path, navigation_timeout=1))
    result = experiment.run()
    assert not result["success"] and "制限時間" in result["error"]
    assert not result["navigation_completed"]
    assert result["measurement_count"] == 0 and result["max_stroke_m"] == 0


def test_free_walk_stops_and_presses_in_one_simulation(model_path):
    import mujoco

    experiment = WalkAndPressExperiment(sim_args(model_path))
    assert experiment.model.cam_bodyid[experiment.model.camera("button_rgbd").id] == experiment.model.body("torso_link").id
    assert experiment.model.cam_mode[experiment.model.camera("button_rgbd").id] == mujoco.mjtCamLight.mjCAMLIGHT_FIXED
    result = experiment.run()
    # 経路途中を含めて物理時間・位置が連続し、押下開始前に両足で静止する。
    trace = result["navigation_trace"]
    intervals = np.diff([t["time_s"] for t in trace])
    assert np.all(intervals > 0)
    # カメラのズーム中はサンプル間隔が広がるため、実時間で割った速度を比較する。
    assert np.max(np.linalg.norm(np.diff([t["pelvis_xyz_m"] for t in trace], axis=0), axis=1)/intervals) < 0.5
    assert result["success"], (result["error"], result["max_stroke_m"])
    assert result["navigation"]["travel_distance_m"] > 1.1
    assert result["navigation"]["stationary_window_s"] >= 0.5
    assert result["handoff_support"]["double_support"]
    assert result["navigation"]["support_before_press"]["double_support"]
    assert result["remeasurement_count"] == 1 and 0.00135 <= result["max_stroke_m"] <= 0.00165
    assert result["max_other_stroke_m"] < 0.00015
    assert not np.any(experiment.data.xfrc_applied) and not np.any(experiment.data.qfrc_applied)
    # レポートがPython/NumPyの型差によらず保存できることも確認。
    json.dumps(result, allow_nan=False)
