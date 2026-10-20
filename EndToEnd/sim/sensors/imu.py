"""URDFのIMU位置で、PhysXの剛体状態から慣性量を取得する。描画へ依存しない。"""

from dataclasses import asdict

from g1_e2e.data.geometry import mount_matrix, pose_matrix
from g1_e2e.data.imu import measure_imu


class UrdfImuRig:
    def __init__(self, env, config):
        self.env, self.config = env, config
        self.body_ids = {
            mount.body: env.robot.body_names.index(mount.body) for mount in config.mounts.values()
        }
        self.transforms = {
            name: mount_matrix(mount.position_m, mount.rpy_deg) for name, mount in config.mounts.items()
        }

    @property
    def metadata(self):
        return {
            "primary": self.config.primary,
            "mounts": {name: asdict(mount) for name, mount in self.config.mounts.items()},
            "mapping_source": self.config.mapping_source,
            "mapping_status": "manufacturer simulation convention; individual hardware alignment unverified",
            "measurement": "specific force at URDF mount including angular acceleration and centripetal terms",
            "sampling_period_s": self.env.config.physics_dt,
            "orientation": "sensor-to-world xyzw; SDK converts to wxyz",
            "gyro": "sensor rad/s",
            "specific_force": "sensor m/s^2; acceleration minus gravity",
            "noise_model": "none; public mounting geometry is not an electronic noise calibration",
        }

    def capture(self, episode_id, time_s):
        env, data = self.env, self.env.robot.data
        values = {
            key: env._numpy(getattr(data, key))
            for key in [
                "body_link_pos_w",
                "body_link_quat_w",
                "body_com_pos_w",
                "body_com_lin_acc_w",
                "body_com_ang_vel_w",
                "body_com_ang_acc_w",
            ]
        }
        base = pose_matrix(env._numpy(data.root_pos_w), env._numpy(data.root_quat_w))
        frames = {}
        for name, mount in self.config.mounts.items():
            i = self.body_ids[mount.body]
            frames[name] = measure_imu(
                episode_id=episode_id,
                time_s=time_s,
                world_from_body=pose_matrix(values["body_link_pos_w"][i], values["body_link_quat_w"][i]),
                body_from_sensor=self.transforms[name],
                com_position_world_m=values["body_com_pos_w"][i],
                com_acceleration_world_m_s2=values["body_com_lin_acc_w"][i],
                angular_velocity_world_rad_s=values["body_com_ang_vel_w"][i],
                angular_acceleration_world_rad_s2=values["body_com_ang_acc_w"][i],
                gravity_world_m_s2=env.sim.cfg.gravity,
                world_from_base=base,
                acceleration_valid=time_s > 0,
            )
        return frames
