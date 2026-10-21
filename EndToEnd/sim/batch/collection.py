"""制御・物理・取得・非同期保存を、共通tickに沿って進める。"""

import time

import numpy as np

from g1_e2e.contracts import JointCommand, StepResult
from .recording import BatchStep


class ParallelCollector:
    def __init__(self, env, rig, recorder, teachers, controllers, schedule):
        self.env, self.rig, self.recorder = env, rig, recorder
        self.teachers, self.controllers, self.schedule = teachers, controllers, schedule
        self.fallen = np.zeros(env.num_envs, bool)
        self.durations = dict.fromkeys(
            ("physics_and_imu", "camera", "lidar_and_transfer", "teacher_and_control", "writer_backpressure"),
            0.0,
        )

    def _capture_camera(self, tick):
        started = time.perf_counter()
        images, extrinsics = self.rig.capture_camera()
        self.recorder.cameras(images, extrinsics, tick * self.schedule.physics_dt_s)
        self.durations["camera"] += time.perf_counter() - started

    def run(self, observations, steps):
        self.schedule.validate_steps(steps)
        env, rig, recorder = self.env, self.rig, self.recorder
        controllers, teachers = self.controllers, self.teachers
        durations, fallen = self.durations, self.fallen
        decimation = self.schedule.control_ticks
        rows = []
        collection_start = time.perf_counter()
        for step in range(steps):
            tick = step * decimation
            if tick % rig.camera_stride == 0:
                self._capture_camera(tick)
            phase = time.perf_counter()
            for controller, teacher, observation in zip(controllers, teachers, observations):
                controller.set_velocity(teacher.command(observation))
            commands = [
                controller.act(observation) for controller, observation in zip(controllers, observations)
            ]
            velocities = [controller.velocity_command for controller in controllers]
            positions = env.set_positions(np.stack([command.position_rad for command in commands]))
            applied = [JointCommand(position.copy()) for position in positions]
            durations["teacher_and_control"] += time.perf_counter() - phase
            for substep in range(decimation):
                phase = time.perf_counter()
                env.physics_step()
                local_tick = (env.ticks - 1) % rig.scan_ticks + 1
                rig.capture_inertial(local_tick)
                durations["physics_and_imu"] += time.perf_counter() - phase
                if (
                    env.ticks < steps * decimation
                    and env.ticks % rig.camera_stride == 0
                    and substep != decimation - 1
                ):
                    self._capture_camera(env.ticks)
            env.steps += 1
            next_observations = env.observe()
            terminated = np.array(
                [
                    o.privileged["base_position_world_m"][2] < env.config.minimum_base_height
                    or np.arccos(np.clip(-o.projected_gravity_body[2], -1, 1))
                    > env.config.maximum_tilt_radians
                    for o in next_observations
                ]
            )
            fallen |= terminated
            truncated = (
                np.full(env.num_envs, (step + 1) * env.config.control_dt >= env.config.episode_seconds)
                & ~terminated
            )
            results = [
                StepResult(next_obs, command, applied_command, bool(done), bool(limit), {})
                for next_obs, command, applied_command, done, limit in zip(
                    next_observations, commands, applied, terminated, truncated
                )
            ]
            rows.append(BatchStep(observations, results, velocities))
            observations = next_observations
            if env.ticks % rig.scan_ticks == 0:
                phase = time.perf_counter()
                scan = rig.capture_scan(
                    round((env.ticks - rig.scan_ticks) * env.config.physics_dt * rig.pattern.rate)
                )
                imu, poses = rig.imu.numpy(), rig.poses.numpy()
                for i, teacher in enumerate(teachers):
                    update_scan = getattr(teacher, "update_scan", None)
                    if update_scan is not None:
                        update_scan(
                            scan[i, :, 2:5], scan[i, :, 1].astype(bool), env.ticks * env.config.physics_dt
                        )
                directions = rig.pattern.sample(
                    np.arange(
                        round((env.ticks - rig.scan_ticks) * env.config.physics_dt * rig.pattern.rate),
                        round(env.ticks * env.config.physics_dt * rig.pattern.rate),
                    )
                )
                durations["lidar_and_transfer"] += time.perf_counter() - phase
                phase = time.perf_counter()
                recorder.block(imu, poses, scan, directions, env.ticks - rig.scan_ticks, env.ticks, rows)
                durations["writer_backpressure"] += time.perf_counter() - phase
                rows = []
                rig.roll_scan_buffer()
            if (step + 1) % 50 == 0:
                elapsed = time.perf_counter() - collection_start
                print(
                    f"[COLLECT] {step + 1}/{steps}; aggregate={(step + 1) * env.config.control_dt * env.num_envs / elapsed:.3f}x",
                    flush=True,
                )
        return observations
