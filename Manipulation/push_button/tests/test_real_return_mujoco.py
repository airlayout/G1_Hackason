"""実機と同じMenagerie運動学で、接続と逆再生の衝突を検査する。"""

import math
import os
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kinematics import ArmKinematics, load_robot
from return_path import ApproachPath, validate_return_path
from trajectory import interpolate


@pytest.mark.parametrize("x,y,yaw,z,reachable", [
    (0.33, -0.25, -3, 1.0, True), (0.33, -0.25, -3, 0.92, True),
    (0.36, -0.25, 0, 1.0, True), (0.36, -0.25, 0, 0.92, True),
    (0.39, -0.22, 3, 1.0, True), (0.39, -0.22, 3, 0.92, False)])
def test_reverse_route_with_remeasurement_correction_clears_wall_and_body(x, y, yaw, z, reachable):
    model_path = os.environ.get("G1_MUJOCO_MODEL")
    if not model_path:
        pytest.skip("G1_MUJOCO_MODELにMenagerieのg1.xmlを指定してください")
    mujoco = pytest.importorskip("mujoco")
    face = np.array([x, y, z])
    angle = math.radians(yaw)
    normal = np.array([math.cos(angle), math.sin(angle), 0])

    def wall(root):
        position = face+0.05*normal
        ET.SubElement(root.find("worldbody"), "geom", name="return_wall", type="box",
                      pos=" ".join(map(str, position)), size="0.05 0.5 0.5",
                      quat=f"{math.cos(angle/2)} 0 0 {math.sin(angle/2)}")

    model, rest = load_robot(Path(model_path), (0.110, -0.003, 0), 0.79,
                             tip_radius=0.015, scene_builder=wall)
    kin = ArmKinematics(model)
    kin.update(rest)
    home = rest[kin.qadr].copy()
    path = ApproachPath(home)
    radius = 0.015
    original_face = face-np.array([0, 0.003, 0])

    def move_to(point, record):
        goal = kin.solve(point, normal).joint_angles
        kin.validate_path(goal, face, normal)
        start = kin.data.qpos[kin.qadr].copy()
        for sample in interpolate(tuple(start), tuple(goal), 2.0):
            qpos = kin.data.qpos.copy()
            qpos[kin.qadr] = sample
            kin.update(qpos)
            if record:
                path.append(sample, 0.02)

    if not reachable:
        # この配置は既存IKが初回の中間点を解けない。経路の記録・実行前に拒否する。
        with pytest.raises(ValueError, match="IKが収束"):
            move_to(original_face-(radius+0.09)*normal, True)
        assert len(path.samples) == 1
        return
    move_to(original_face-(radius+0.09)*normal, True)
    move_to(original_face-(radius+0.05)*normal, True)
    move_to(face-(radius+0.05)*normal, True)  # 1回再計測後の補正も逆再生する。
    path.freeze()
    # 押下時の別姿勢から同じ5cm手前を解き直すと、記録終点と僅かに異なる。
    # この静的検査の壁は可動ボタンを持たないため、接触3mm手前を使う。
    pressed = kin.solve(face-(radius+0.003)*normal, normal).joint_angles
    qpos = kin.data.qpos.copy()
    qpos[kin.qadr] = pressed
    kin.update(qpos)
    move_to(face-(radius+0.05)*normal, False)
    targets = path.reversed_targets()
    validate_return_path(kin, targets, face, normal)
    wall_id = model.geom("return_wall").id
    # 接続もsmoothstepでサンプリングし、独立したMuJoCo接触形状で検査する。
    connection = list(interpolate(tuple(kin.data.qpos[kin.qadr]), targets[0], 1.0))
    for goal in [*connection, *targets]:
        qpos = kin.data.qpos.copy()
        qpos[kin.qadr] = goal
        kin.update(qpos)
        for contact in kin.data.contact:
            pair = {int(contact.geom1), int(contact.geom2)}
            if wall_id in pair or pair.intersection(kin.arm_geoms):
                assert contact.dist >= -0.0005, (x, y, z, yaw, pair, contact.dist)
    np.testing.assert_allclose(kin.data.qpos[kin.qadr], home, atol=1e-12)
