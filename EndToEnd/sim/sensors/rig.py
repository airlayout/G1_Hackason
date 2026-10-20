"""URDF取り付けに追従するRGB-D・LiDAR・身体IMUを取得する。"""

from __future__ import annotations

import numpy as np

from g1_e2e.data.config import SensorConfig
from g1_e2e.data.geometry import MOUNT_FROM_OPTICAL, base_pose, mount_matrix, pose_matrix

from .imu import UrdfImuRig
from .camera import CalibratedRgbdCamera
from .scene import add_landmarks, static_mesh
from .lidar import StaticRaycaster, make_lidar


class SensorRig:
    def __init__(self, env, config: SensorConfig):
        import carb
        import omni.kit.app
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaaclab.utils.warp import convert_to_warp_mesh
        from pxr import Gf, UsdGeom, UsdLux

        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("isaacsim.sensors.experimental.rtx", True)
        self.env, self.config = env, config
        if not config.body_imus.mounts:
            raise ValueError("収集する身体IMUのURDF取り付け定義が必要です")
        self.imus = UrdfImuRig(env, config.body_imus)
        self.gravity_world = np.asarray(env.sim.cfg.gravity, dtype=np.float32)
        self.camera_stride, _ = config.periods_in_steps(env.config.physics_dt)
        self.body_ids = {
            name: env.robot.body_names.index(name)
            for name in {config.camera.mount.body, config.lidar.mount.body, "pelvis"}
        }
        stage = omni.usd.get_context().get_stage()
        self.static_roots = add_landmarks() if config.landmarks else ["/World/Ground"]
        points, faces, self.mesh_paths = static_mesh(stage, self.static_roots)
        self.mesh = convert_to_warp_mesh(points, faces.reshape(-1), device=str(env.device))
        self.camera_mount = mount_matrix(config.camera.mount.position_m, config.camera.mount.rpy_deg)
        self.lidar_mount = mount_matrix(config.lidar.mount.position_m, config.lidar.mount.rpy_deg)
        self.lidar = make_lidar(
            env,
            config.lidar,
            StaticRaycaster(self.mesh, env.device, config.lidar, self.mesh_paths),
            lambda: self._body_world(config.lidar.mount.body) @ self.lidar_mount,
            lambda: self._body_world("pelvis"),
        )
        self.rgbd = CalibratedRgbdCamera(config.camera)
        # Explicit illumination also works in the headless experience.
        sun = UsdLux.DistantLight.Define(stage, "/World/CollectionSun")
        sun.CreateIntensityAttr(1500.0)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45, -25, -30))
        rep.orchestrator.set_capture_on_play(True)
        carb.settings.get_settings().set("/rtx/rendermode", "PathTracing")
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_auto_update(False)
        timeline.commit()

    def _body_world(self, name):
        data, i = self.env.robot.data, self.body_ids[name]
        return pose_matrix(
            self.env._numpy(data.body_link_pos_w)[i], self.env._numpy(data.body_link_quat_w)[i]
        )

    @property
    def calibration(self):
        common = {
            "camera/intrinsics": self.config.camera.intrinsics,
            **{
                f"imu/{name}/parent_from_sensor": transform
                for name, transform in self.imus.transforms.items()
            },
        }
        native = self.config.camera.depth
        if native is not None:
            common.update(
                {
                    "depth/intrinsics": native.pinhole.resized(native.pinhole.width, native.pinhole.height),
                    "depth/color_from_depth": native.color_from_depth,
                    "depth/scale_m": np.asarray(native.scale_m),
                }
            )
        return {**common, **{f"lidar/{key}": value for key, value in self.lidar.calibration.items()}}

    @property
    def metadata(self):
        result = {
            "sensor_config": self.config.to_dict(),
            "calibration_status": "URDF nominal mounts; RGB K from physical-device IO specification; base-to-sensor extrinsics not calibrated to the individual robot",
            "camera": {
                "model": "separate calibrated RGB and native-depth pinholes"
                if self.config.camera.depth
                else "ideal aligned RGB-D pinhole",
                "color_order": "RGB",
                "depth": "native optical Z in metres on RGB grid (RealSense align convention); missing pixels have valid=false"
                if self.config.camera.depth
                else "optical Z in metres; missing pixels have valid=false",
                "optical_axes": "X right, Y down, Z forward",
                "pixel_coordinates": "RealSense integer pixel centers; USD half-pixel convention corrected",
                "renderer": "PathTracing",
                "fov_deg_horizontal_vertical": self.config.camera.fov_deg,
                "intrinsics_source": self.config.camera.pinhole.source
                if self.config.camera.pinhole
                else "synthetic horizontal FOV",
                "depth_alignment": "native pixel corners projected using measured depth-to-color extrinsics; rectangle transfer with minimum Z16"
                if self.config.camera.depth
                else "ideal Z on RGB optical grid",
                "depth_alignment_source": "https://github.com/realsenseai/librealsense/blob/c84fe6438bd14a4cc69f46deae92c993268232c6/src/proc/align.cpp",
                "depth_physics": "ideal geometric depth; active stereo matching and IR failures unmodeled",
            },
            "lidar": self.lidar.metadata,
            "imu": self.imus.metadata,
            "primary_imu": self.config.body_imus.primary,
            "gravity_world_m_s2": self.gravity_world.tolist(),
            "joint_torque": "Isaac applied_torque; implicit PD estimate, not measured force",
            "clock": "simulation time in seconds, relative to episode reset",
            "transforms": "world_from_sensor maps column vectors; base X forward/Y left/Z up",
        }
        return result

    def reset(self, observation):
        self.latest_imus = self.imus.capture(observation.episode_id, observation.simulation_time_s)
        self.lidar.reset(observation.episode_id)

    def after_physics_step(self, episode_id, time_s):
        self.latest_imus = self.imus.capture(episode_id, time_s)
        self.lidar.advance(episode_id, time_s)

    def camera_due(self, time_s):
        tick = round(time_s / self.env.config.physics_dt)
        return tick % self.camera_stride == 0

    def capture_camera(self, observation):
        world_from_mount = self._body_world(self.config.camera.mount.body) @ self.camera_mount
        self.env.sim.forward()
        return self.rgbd.capture(
            observation.episode_id,
            observation.simulation_time_s,
            world_from_mount @ MOUNT_FROM_OPTICAL,
            base_pose(observation),
        )

    def capture_lidar(self, episode_id, time_s):
        return self.lidar.pop(episode_id, time_s)

    def specific_force(self, observation):
        frame = self.latest_imus[self.config.body_imus.primary]
        if (
            frame.episode_id != observation.episode_id
            or abs(frame.time_s - observation.simulation_time_s) > 1e-9
        ):
            raise RuntimeError("IMUと身体状態の時刻が一致しません")
        return (frame.base_from_sensor[:3, :3] @ frame.specific_force_sensor_m_s2).astype(np.float32)

    def close(self):
        self.rgbd.close()
