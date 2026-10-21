"""モデルもタスク報酬も所有しない、平地G1のreset/step環境。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation

from g1_e2e.contracts import JointCommand, Observation, StepResult
from g1_e2e.robot import RobotSpec
from sim.robots.g1 import make_robot_config

from .config import FlatConfig


class G1FlatEnv:
    """起動済みIsaacアプリ上の1体環境。全29関節は常にSDK順で公開する。"""

    def __init__(
        self,
        root: Path,
        robot_spec: RobotSpec,
        config: FlatConfig,
        *,
        device: str = "cuda:0",
        fixed_base: bool = False,
        office=None,
    ) -> None:
        self.root, self.robot_spec, self.config = root, robot_spec, config
        self.fixed_base = fixed_base
        self.sim = sim_utils.SimulationContext(
            sim_utils.SimulationCfg(
                dt=config.physics_dt,
                device=device,
                render_interval=round(config.control_dt / config.physics_dt),
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    static_friction=config.friction,
                    dynamic_friction=config.friction,
                    restitution=0.0,
                ),
            )
        )
        self.office, self.scene_metadata = office, {"kind": "flat"}
        if office is None:
            ground = sim_utils.CuboidCfg(
                size=(200.0, 200.0, 0.1),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.38, 0.42)),
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    static_friction=config.friction,
                    dynamic_friction=config.friction,
                    restitution=0.0,
                ),
            )
            ground.func("/World/Ground", ground, translation=(0.0, 0.0, -0.05))
        else:
            from .office import spawn_office

            self.scene_metadata = {"kind": "office", **spawn_office(root, office)}
            self.static_roots = [self.scene_metadata["root_prim"]]
        light = sim_utils.DomeLightCfg(intensity=2000.0)
        light.func("/World/Light", light)
        self.robot = Articulation(make_robot_config(root, robot_spec, fixed_base=fixed_base))
        self.sim.reset()
        # Kit Visualizerはreset時に初期化されるため、その後にカメラを設定する。
        self.sim.set_camera_view(eye=(3.0, 3.0, 2.0), target=(0.0, 0.0, 0.8))
        self.sdk_indices = robot_spec.simulator_indices(self.robot.joint_names)
        self.device = self.sim.device
        # Isaac Lab 3.0のWarp kernelはjoint_idsにint32を要求する。
        self._indices = torch.tensor(self.sdk_indices, device=self.device, dtype=torch.int32)
        self._episode = -1
        self._step = 0
        self._done = True
        limits = self._numpy(self.robot.data.joint_pos_limits)[self.sdk_indices]
        self._lower, self._upper = limits[:, 0], limits[:, 1]
        print(f"[READY] 身体29関節を確認しました: {self.robot.joint_names}", flush=True)

    @staticmethod
    def _tensor(value: object) -> torch.Tensor:
        """Isaac Lab 3.0のProxyArrayをtorchとして取り出す。"""
        return value.torch if hasattr(value, "torch") else value

    def _numpy(self, value: object) -> np.ndarray:
        """外部からシミュレーターの状態を変更できないコピーを返す。"""
        return self._tensor(value)[0].detach().cpu().numpy().astype(np.float32, copy=True)

    def reset(self, *, seed: int = 0, spawn_xy_yaw=None) -> Observation:
        """root・全関節・内部バッファ・episode時計を初期化する。"""
        self._rng = np.random.default_rng(seed)
        data = self.robot.data
        root_pose = self._tensor(data.default_root_pose).clone()
        if spawn_xy_yaw is not None:
            x, y, yaw = spawn_xy_yaw
            if not np.isfinite([x, y, yaw]).all():
                raise ValueError("spawnのXY・yawは有限値です")
            root_pose[0, :3] = root_pose.new_tensor(
                [x, y, self.robot_spec.spawn_height + (self.office.floor_height_m if self.office else 0)]
            )
            root_pose[0, 3:] = root_pose.new_tensor([0, 0, np.sin(yaw / 2), np.cos(yaw / 2)])
        self.robot.write_root_pose_to_sim_index(root_pose=root_pose)
        self.robot.write_root_velocity_to_sim_index(root_velocity=self._tensor(data.default_root_vel).clone())
        self.robot.write_joint_position_to_sim_index(position=self._tensor(data.default_joint_pos).clone())
        self.robot.write_joint_velocity_to_sim_index(velocity=self._tensor(data.default_joint_vel).clone())
        self.robot.reset()
        self._episode += 1
        self._step = 0
        self._done = False
        self._set_command(JointCommand(self.robot_spec.default_position.copy()))
        self.robot.write_data_to_sim()
        self.robot.update(0.0)
        return self.observe()

    def observe(self) -> Observation:
        """正規化・履歴連結・希望速度の追加を行わず、現在の物理量を返す。"""
        data = self.robot.data
        return Observation(
            episode_id=self._episode,
            step=self._step,
            simulation_time_s=self._step * self.config.control_dt,
            joint_position_rad=self._numpy(data.joint_pos)[self.sdk_indices],
            joint_velocity_rad_s=self._numpy(data.joint_vel)[self.sdk_indices],
            joint_torque_nm=self._numpy(data.applied_torque)[self.sdk_indices],
            orientation_xyzw=self._numpy(data.root_quat_w),
            angular_velocity_body_rad_s=self._numpy(data.root_ang_vel_b),
            projected_gravity_body=self._numpy(data.projected_gravity_b),
            privileged={
                "base_position_world_m": self._numpy(data.root_pos_w),
                "base_linear_velocity_body_m_s": self._numpy(data.root_lin_vel_b),
            },
        )

    def _set_command(self, command: JointCommand) -> None:
        for setter, value in [
            (self.robot.set_joint_position_target_index, command.position_rad),
            (self.robot.set_joint_velocity_target_index, command.velocity_rad_s),
            (self.robot.set_joint_effort_target_index, command.feedforward_torque_nm),
        ]:
            setter(target=torch.tensor(value[None], device=self.device), joint_ids=self._indices)

    def step(self, command: JointCommand, *, render: bool = True, after_physics_step=None) -> StepResult:
        """1制御周期だけ物理を進める。終了後は呼び出し元にresetを要求する。"""
        if self._done:
            raise RuntimeError("stepの前にresetが必要です")
        requested = command.validated()
        applied = JointCommand(
            np.clip(requested.position_rad, self._lower, self._upper),
            np.clip(
                requested.velocity_rad_s, -self.robot_spec.velocity_limit, self.robot_spec.velocity_limit
            ),
            np.clip(
                requested.feedforward_torque_nm, -self.robot_spec.effort_limit, self.robot_spec.effort_limit
            ),
        )
        self._set_command(applied)
        decimation = round(self.config.control_dt / self.config.physics_dt)
        for substep in range(decimation):
            self.robot.write_data_to_sim()
            self.sim.step(render=render and substep == decimation - 1)
            self.robot.update(self.config.physics_dt)
            if after_physics_step is not None:
                after_physics_step(
                    self._episode,
                    self._step * self.config.control_dt + (substep + 1) * self.config.physics_dt,
                )
        self._step += 1
        obs = self.observe()
        if not all(
            np.isfinite(value).all()
            for value in [
                obs.joint_position_rad,
                obs.joint_velocity_rad_s,
                obs.joint_torque_nm,
                obs.orientation_xyzw,
                obs.angular_velocity_body_rad_s,
                obs.projected_gravity_body,
                *obs.privileged.values(),
            ]
        ):
            raise FloatingPointError("物理状態にNaN/Infが発生しました")
        height = float(obs.privileged["base_position_world_m"][2])
        tilt = float(np.arccos(np.clip(-obs.projected_gravity_body[2], -1, 1)))
        fallen = not self.fixed_base and (
            height < self.config.minimum_base_height or tilt > self.config.maximum_tilt_radians
        )
        timeout = self._step * self.config.control_dt >= self.config.episode_seconds
        self._done = fallen or timeout
        return StepResult(
            obs,
            requested,
            applied,
            fallen,
            timeout and not fallen,
            {
                "base_height_m": height,
                "tilt_rad": tilt,
                "termination_reason": "fall" if fallen else "time_limit" if timeout else None,
            },
        )

    def close(self) -> None:
        """環境のリソースを解放する。Isaacアプリの終了は呼び出し元が担当する。"""
        self._done = True
        self.sim.stop()
        self.sim.clear_instance()
