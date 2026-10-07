"""公式モデルで、制御器が生成する接近軌道の壁への食い込みを検出する。"""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim"))

from yada import CLEARANCE, TIP_RADIUS, YadaButtonController
from yada_paths import find_yada_root


@pytest.mark.parametrize("seed", range(20))
def test_generated_approach_does_not_sweep_through_wall(seed, monkeypatch):
    # 採点用の依存・モデルを準備した環境で実行する。画像推定を固定し、軌道だけを検査する。
    pytest.importorskip("pinocchio")
    mujoco = pytest.importorskip("mujoco")
    try:
        root = find_yada_root()
    except FileNotFoundError:
        pytest.skip("sim/prepare_yada.py で採点環境を準備してください")
    monkeypatch.syspath_prepend(str(root))
    from common.config import load_config
    from contest.interface import TaskInfo
    from contest.robots.mujoco_robot import MujocoRobot
    from contest.task import make_trial

    cfg = load_config("contest.yaml")
    trial = make_trial(seed, cfg)
    robot = MujocoRobot(trial.scene_cfg, cfg)
    controller = YadaButtonController()
    try:
        robot.advance(cfg["settle_s"])
        kp, kd = robot.upper_gains()
        controller.reset(TaskInfo(trial.instruction, trial.target, "mujoco", cfg["time_limit_s"],
                                  1 / cfg["control_hz"], False, kp, kd))
        face = robot.scene.button(trial.target).face_center.copy()
        normal = np.array([1.0, 0.0, 0.0])
        monkeypatch.setattr(controller, "_measure", lambda obs: (face, normal))
        q = robot.data.qpos[robot._qadr].copy()
        approached = False
        for step in range(int(15 * cfg["control_hz"])):
            obs = SimpleNamespace(q=q, t=step / cfg["control_hz"],
                                  imu_quat=robot.data.xquat[robot._pelvis].copy())
            controller.act(obs)
            q = controller.q_cmd.copy()
            robot.data.qpos[robot._qadr] = q
            mujoco.mj_forward(robot.model, robot.data)
            for contact in robot.data.contact:
                a, b = int(contact.geom1), int(contact.geom2)
                touches_wall = ((a in robot.hall._hall_geoms and b in robot.hall._robot_geoms)
                                or (b in robot.hall._hall_geoms and a in robot.hall._robot_geoms))
                assert not touches_wall or contact.dist >= 0, f"seed={seed}, t={obs.t}, {a}/{b}"
            if controller.phase == "settle":
                approached = True
                break
        assert approached, "ボタン前への移動が終わらない"
        assert np.linalg.norm(controller.kin.fk_pos(q) - (face - (TIP_RADIUS + CLEARANCE) * normal)) < 0.001
    finally:
        robot.close()
