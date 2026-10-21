"""PhysXの複製環境と1つのArticulation viewで全G1を進める。"""

from copy import copy

import numpy as np
import torch

from g1_e2e.contracts import Observation


def tensor(value):
    return value.torch if hasattr(value, "torch") else value


class VectorPatrolEnv:
    spacing_m = 500.0

    def __init__(self, root, spec, config, office, num_envs, device):
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg
        from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
        from sim.robots.g1 import make_robot_config

        self.root, self.spec, self.config, self.office = root, spec, config, office
        self.num_envs = num_envs
        self.sim = sim_utils.SimulationContext(
            sim_utils.SimulationCfg(
                dt=config.physics_dt,
                device=device,
                render_interval=round(config.control_dt / config.physics_dt),
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    static_friction=config.friction, dynamic_friction=config.friction, restitution=0.0
                ),
            )
        )
        self.device = self.sim.device
        cfg = InteractiveSceneCfg(num_envs=num_envs, env_spacing=self.spacing_m, replicate_physics=True)
        cfg.office = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Office", spawn=sim_utils.UsdFileCfg(usd_path=str(office.usd_path(root)))
        )
        cfg.robot = make_robot_config(root, spec).replace(prim_path="{ENV_REGEX_NS}/G1")
        self.scene = InteractiveScene(cfg)
        light = sim_utils.DomeLightCfg(intensity=2000.0)
        light.func("/World/Light", light)
        import omni.usd
        from pxr import Gf, Usd, UsdGeom, UsdLux

        stage = omni.usd.get_context().get_stage()
        self.light_counts = {"local": 0, "shared_global": 0, "disabled_global_duplicates": 0}
        for i in range(num_envs):
            office_root = stage.GetPrimAtPath(f"/World/envs/env_{i}/Office")
            lights = [prim for prim in Usd.PrimRange(office_root) if prim.HasAPI(UsdLux.LightAPI)]
            for prim in lights:
                if prim.IsA(UsdLux.DomeLight) or prim.IsA(UsdLux.DistantLight):
                    if i:
                        prim.SetActive(False)
                        self.light_counts["disabled_global_duplicates"] += 1
                    else:
                        self.light_counts["shared_global"] += 1
                else:
                    self.light_counts["local"] += 1

        sun = UsdLux.DistantLight.Define(omni.usd.get_context().get_stage(), "/World/CollectionSun")
        sun.CreateIntensityAttr(1500.0)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45, -25, -30))
        self.robot = self.scene["robot"]
        self.sim.reset()
        self.origins = tensor(self.scene.env_origins)
        self.origins_cpu = self.origins.cpu().numpy()
        self.sdk_indices = spec.simulator_indices(self.robot.joint_names)
        self.joint_ids = torch.tensor(self.sdk_indices, dtype=torch.int32, device=self.device)
        limits = tensor(self.robot.data.joint_pos_limits)[0, self.sdk_indices].cpu().numpy()
        self.lower, self.upper = limits[:, 0], limits[:, 1]
        self.episode = np.full(num_envs, -1, np.int64)
        self.steps = np.zeros(num_envs, np.int64)
        self.ticks = 0

    def reset(self, spawns):
        pose = tensor(self.robot.data.default_root_pose).clone()
        local = np.asarray(spawns)
        pose[:, :3] = self.origins + pose.new_tensor(
            np.column_stack(
                (local[:, :2], np.full(self.num_envs, self.spec.spawn_height + self.office.floor_height_m))
            )
        )
        pose[:, 3:] = pose.new_tensor(
            np.column_stack((np.zeros((self.num_envs, 2)), np.sin(local[:, 2] / 2), np.cos(local[:, 2] / 2)))
        )
        self.robot.write_root_pose_to_sim_index(root_pose=pose)
        self.robot.write_root_velocity_to_sim_index(
            root_velocity=tensor(self.robot.data.default_root_vel).clone()
        )
        self.robot.write_joint_position_to_sim_index(
            position=tensor(self.robot.data.default_joint_pos).clone()
        )
        self.robot.write_joint_velocity_to_sim_index(
            velocity=tensor(self.robot.data.default_joint_vel).clone()
        )
        self.robot.reset()
        self.episode += 1
        self.steps[:] = 0
        self.ticks = 0
        self.set_positions(np.broadcast_to(self.spec.default_position, (self.num_envs, 29)))
        self.robot.update(0.0)
        return self.observe()

    def observe(self):
        """全環境の制御観測を1回のGPU→CPUコピーで取り出す。"""
        data = self.robot.data
        values = (
            torch.cat(
                [
                    tensor(getattr(data, key))
                    for key in (
                        "joint_pos",
                        "joint_vel",
                        "root_quat_w",
                        "root_ang_vel_b",
                        "projected_gravity_b",
                        "root_pos_w",
                        "root_lin_vel_b",
                    )
                ],
                dim=1,
            )
            .cpu()
            .numpy()
        )
        parts = np.split(values, [29, 58, 62, 65, 68, 71], axis=1)
        q, dq = parts[0][:, self.sdk_indices], parts[1][:, self.sdk_indices]
        # 既存ImplicitActuatorと同じPD推定値。PhysXのPDは600Hzで動き、
        # 目標の再送は50Hzだけでよい。推定トルクは観測時のq/dqで計算する。
        torque = np.clip(
            self.spec.stiffness * (self.positions - q) - self.spec.damping * dq,
            -self.spec.effort_limit,
            self.spec.effort_limit,
        ).astype(np.float32)
        return [
            Observation(
                int(self.episode[i]),
                int(self.steps[i]),
                float(self.steps[i] * self.config.control_dt),
                parts[0][i, self.sdk_indices].copy(),
                parts[1][i, self.sdk_indices].copy(),
                torque[i].copy(),
                parts[2][i].copy(),
                parts[3][i].copy(),
                parts[4][i].copy(),
                {
                    "base_position_world_m": (parts[5][i] - self.origins_cpu[i]).copy(),
                    "base_linear_velocity_body_m_s": parts[6][i].copy(),
                },
            )
            for i in range(self.num_envs)
        ]

    def set_positions(self, positions):
        positions = np.clip(positions, self.lower, self.upper).astype(np.float32)
        self.robot.set_joint_position_target_index(
            target=torch.as_tensor(positions, device=self.device), joint_ids=self.joint_ids
        )
        self.positions = positions.copy()
        self.robot.write_data_to_sim()
        return positions

    def physics_step(self):
        self.sim.step(render=False)
        self.robot.update(self.config.physics_dt)
        self.ticks += 1

    def close(self):
        self.sim.stop()
        self.sim.clear_instance()


def shared_controllers(template, count):
    """ONNX sessionと重みは共有し、各G1の履歴と指令だけを独立させる。"""
    result = [copy(template) for _ in range(count)]
    for controller in result:
        controller.reset()
    return result
