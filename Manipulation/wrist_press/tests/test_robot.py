import numpy as np
import pytest

from Manipulation.wrist_press.runtime import source_root
from Manipulation.wrist_press.settings import load_settings
from Manipulation.wrist_press.types import Target


@pytest.fixture(scope="module")
def robot_checker():
    pytest.importorskip("pinocchio")
    pytest.importorskip("coal")
    try:
        source_root()
    except FileNotFoundError:
        pytest.skip("run wrist_press.runtime to prepare pinned G1 assets")
    from Manipulation.wrist_press.model import RobotModel
    from Manipulation.wrist_press.collision import CollisionChecker
    cfg=load_settings()
    robot=RobotModel(cfg)
    return robot,CollisionChecker(robot,cfg),cfg


def test_plastic_hand_and_camera_are_checked(robot_checker):
    robot,checker,cfg=robot_checker
    assert any("right_rubber_hand" in shape.name for shape in checker.shapes)
    assert any(shape.name == "wrist_camera_mount" for shape in checker.shapes)
    assert checker.valid(np.zeros(29))
    wall=Target("button","up",[.25,0,.1],[1,0,0],.02,0)
    assert not checker.valid(np.zeros(29),wall)
    assert "wall" in checker.last_reason or "tip" in checker.last_reason


def test_approach_and_return_paths_are_checked(robot_checker):
    from Manipulation.wrist_press.planning import Planner
    robot,checker,cfg=robot_checker
    target=Target("button","up",[.326,-.029,.288],[1,0,0],.021,0,
                  wall_point=[.328,-.029,.288])
    seed=np.zeros(29)
    seed[22:]=[-1,-.5,.3,.3,0,0,0]
    start=np.zeros(29)
    start[[15,16,18,22,23,25]]=np.radians([11.5,20,86,11.5,-20,86])
    axis=robot.axis(target.normal,[35,20])
    goal=robot.solve(target.point-(cfg["tip_radius_m"]+.05)*target.normal,seed,axis)
    planner=Planner(robot,checker,cfg)
    path=planner.transit(start,goal,target)
    assert all(checker.edge(a,b,target) for a,b in zip(path,path[1:]))
    returning=planner.transit(goal,start,target)
    assert all(checker.edge(a,b,target) for a,b in zip(returning,returning[1:]))


def test_contact_permission_does_not_exempt_the_hand(robot_checker):
    robot,checker,cfg=robot_checker
    target=Target("button","up",[.2,-.12,.1],[1,0,0],.02,0)
    assert not checker.valid(np.zeros(29),target,contact=True)
