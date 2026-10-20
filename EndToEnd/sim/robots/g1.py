"""29関節だけを持つG1のURDFとアクチュエーター設定。"""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

from g1_e2e.robot import RobotSpec


def make_robot_config(root: Path, robot: RobotSpec, *, fixed_base: bool = False) -> ArticulationCfg:
    """学習済みモデルを参照せずロボットを構成する。"""
    urdf = root / robot.urdf
    if not urdf.is_file():
        raise FileNotFoundError("ロボットアセットがありません。bash EndToEnd/setup.shを実行してください")
    return ArticulationCfg(
        prim_path="/World/G1",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=str(urdf),
            usd_dir=str(root / "assets/cache/usd" / ("fixed" if fixed_base else "floating")),
            fix_base=fixed_base,
            merge_fixed_joints=True,
            self_collision=True,
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, robot.spawn_height),
            rot=(0.0, 0.0, 0.0, 1.0),
            joint_pos=dict(zip(robot.joint_names, robot.default_position.tolist())),
            joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=1.0,
        actuators={
            "body": ImplicitActuatorCfg(
                joint_names_expr=list(robot.joint_names),
                stiffness=dict(zip(robot.joint_names, robot.stiffness.tolist())),
                damping=dict(zip(robot.joint_names, robot.damping.tolist())),
                effort_limit_sim=dict(zip(robot.joint_names, robot.effort_limit.tolist())),
                velocity_limit_sim=dict(zip(robot.joint_names, robot.velocity_limit.tolist())),
                armature=0.01,
            )
        },
    )
