"""機体追従のRGB-D、静的形状への理想LiDAR、base COMの慣性量を取得する。"""

from __future__ import annotations

import numpy as np

from g1_e2e.data.config import SensorConfig
from g1_e2e.data.dataset import CameraFrame, LidarFrame, base_pose
from g1_e2e.data.geometry import MOUNT_FROM_OPTICAL, lidar_pattern, mount_matrix, pose_matrix, rotation_xyzw

from .scene import add_landmarks, static_mesh


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
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

        self.env, self.config = env, config
        self.gravity_world = np.asarray(env.sim.cfg.gravity, dtype=np.float32)
        self.camera_stride, self.lidar_stride = config.periods_in_steps(env.config.control_dt)
        self.body_ids = {
            name: env.robot.body_names.index(name)
            for name in {config.camera.mount.body, config.lidar.mount.body, "pelvis"}
        }
        stage = omni.usd.get_context().get_stage()
        self.static_roots = add_landmarks() if config.landmarks else ["/World/Ground"]
        points, faces, self.mesh_paths = static_mesh(stage, self.static_roots)
        self.mesh = convert_to_warp_mesh(points, faces.reshape(-1), device=str(env.device))
        self.directions, self.rings = lidar_pattern(
            config.lidar.channels, config.lidar.columns, config.lidar.vertical_fov_deg
        )
        self.camera_mount = mount_matrix(config.camera.mount.position_m, config.camera.mount.rpy_deg)
        self.lidar_mount = mount_matrix(config.lidar.mount.position_m, config.lidar.mount.rpy_deg)
        self.camera = UsdGeom.Camera.Define(stage, "/World/G1Sensors/Camera")
        cc = config.camera
        focal_mm = 24.0
        horizontal_aperture = 2 * focal_mm * np.tan(np.deg2rad(cc.horizontal_fov_deg) / 2)
        self.camera.CreateFocalLengthAttr(focal_mm)
        self.camera.CreateHorizontalApertureAttr(float(horizontal_aperture))
        self.camera.CreateVerticalApertureAttr(float(horizontal_aperture * cc.height / cc.width))
        self.camera.CreateClippingRangeAttr(Gf.Vec2f(cc.near_m, cc.far_m))
        rtx_camera = RtxCamera(str(self.camera.GetPath()), tick_rate=0.0)
        self.camera_sensor = CameraSensor(
            rtx_camera, resolution=(cc.height, cc.width), annotators=["rgb", "distance_to_image_plane"]
        )
        prim = self.camera.GetPrim()
        prim.GetAttribute("omni:rtx:autoExposure:enabled").Set(False)
        prim.GetAttribute("exposure:iso").Set(cc.exposure_iso)
        prim.GetAttribute("exposure:time").Set(cc.exposure_time_s)
        prim.GetAttribute("exposure:fStop").Set(cc.exposure_f_stop)
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
        return {
            "camera/intrinsics": self.config.camera.intrinsics,
            "lidar/ray_directions_sensor": self.directions,
            "lidar/ring": self.rings.astype(np.int32),
            "lidar/point_time_offset_s": np.zeros(len(self.rings), dtype=np.float32),
        }

    @property
    def metadata(self):
        return {
            "sensor_config": self.config.to_dict(),
            "calibration_status": "virtual mounts; not calibrated to physical G1/D435i/MID-360",
            "camera": {
                "model": "ideal aligned RGB-D pinhole",
                "color_order": "RGB",
                "depth": "optical Z in metres; missing pixels have valid=false",
                "optical_axes": "X right, Y down, Z forward",
                "renderer": "PathTracing",
            },
            "lidar": {
                "model": "instantaneous ideal 360-degree raycast; static geometry only",
                "intensity": "normalized cosine of incidence; no material reflectivity model",
                "ring": "vertical beam index",
                "point_time_offset_s": "0: simultaneous rays",
                "self_returns": False,
                "mesh_paths": self.mesh_paths,
                "no_hit": "valid=false; never used as an occupied or free-space target",
            },
            "imu": {
                "orientation": "ideal base link xyzw",
                "gyro": "body rad/s",
                "projected_gravity": "unit vector in base frame",
                "specific_force": "root COM acceleration minus gravity, expressed in base; m/s^2",
                "noise_model": "none",
                "sampling_period_s": self.env.config.control_dt,
            },
            "gravity_world_m_s2": self.gravity_world.tolist(),
            "joint_torque": "Isaac applied_torque; implicit PD estimate, not measured force",
            "clock": "simulation time in seconds, relative to episode reset",
            "transforms": "world_from_sensor maps column vectors; base X forward/Y left/Z up",
        }

    def capture_camera(self, observation):
        from isaacsim.core.rendering_manager import RenderingManager
        from pxr import Gf, UsdGeom

        world_from_mount = self._body_world(self.config.camera.mount.body) @ self.camera_mount
        eye = world_from_mount[:3, 3]
        target = eye + world_from_mount[:3, 0]
        up = world_from_mount[:3, 2]
        UsdGeom.Xformable(self.camera).MakeMatrixXform().Set(
            Gf.Matrix4d()
            .SetLookAt(Gf.Vec3d(*map(float, eye)), Gf.Vec3d(*map(float, target)), Gf.Vec3d(*map(float, up)))
            .GetInverse()
        )
        self.env.sim.forward()
        rgb, depth = None, None
        for update in range(max(30, self.config.camera.render_updates)):
            # This helper disables automatic physics while pumping the renderer.
            RenderingManager.render()
            if update + 1 < self.config.camera.render_updates:
                continue
            rgb, _ = self.camera_sensor.get_data("rgb")
            depth, _ = self.camera_sensor.get_data("distance_to_image_plane")
            if rgb is not None and depth is not None:
                break
        if rgb is None or depth is None:
            raise RuntimeError("RGB-Dの描画が完了しませんでした")
        rgb = rgb.numpy().copy()
        depth = depth.numpy()[..., 0].astype(np.float32, copy=True)
        cc = self.config.camera
        if rgb.shape != (cc.height, cc.width, 3) or depth.shape != (cc.height, cc.width):
            raise RuntimeError("RGB-Dのshapeが収集設定と一致しません")
        valid = np.isfinite(depth) & (depth >= cc.near_m) & (depth <= cc.far_m)
        depth[~valid] = 0.0
        world_from_optical = world_from_mount @ MOUNT_FROM_OPTICAL
        return CameraFrame(
            observation.episode_id,
            observation.simulation_time_s,
            rgb,
            depth,
            valid,
            world_from_optical,
            np.linalg.inv(base_pose(observation)) @ world_from_optical,
        )

    def capture_lidar(self, observation):
        import torch
        from isaaclab.utils.warp import raycast_mesh

        world_from_sensor = self._body_world(self.config.lidar.mount.body) @ self.lidar_mount
        direction_world = self.directions @ world_from_sensor[:3, :3].T
        origin = np.broadcast_to(world_from_sensor[:3, 3], direction_world.shape).copy()
        starts = torch.as_tensor(origin[None], device=self.env.device)
        directions = torch.as_tensor(direction_world[None], device=self.env.device)
        hits, ranges, normals, _ = raycast_mesh(
            starts,
            directions,
            self.mesh,
            max_dist=self.config.lidar.far_m,
            return_distance=True,
            return_normal=True,
        )
        ranges, normals = ranges[0].cpu().numpy(), normals[0].cpu().numpy()
        valid = np.isfinite(ranges) & (ranges >= self.config.lidar.near_m)
        ranges = np.where(valid, ranges, 0).astype(np.float32)
        points = self.directions * ranges[:, None]
        intensity = np.zeros(len(valid), dtype=np.float32)
        intensity[valid] = np.clip(-np.sum(direction_world[valid] * normals[valid], axis=-1), 0, 1)
        return LidarFrame(
            observation.episode_id,
            observation.simulation_time_s,
            points,
            ranges,
            valid,
            intensity,
            world_from_sensor,
            np.linalg.inv(base_pose(observation)) @ world_from_sensor,
        )

    def specific_force(self, observation):
        data = self.env.robot.data
        acceleration_world = self.env._numpy(data.body_com_lin_acc_w)[self.body_ids["pelvis"]]
        return (
            rotation_xyzw(observation.orientation_xyzw).T @ (acceleration_world - self.gravity_world)
        ).astype(np.float32)

    def close(self):
        if self.camera_sensor is not None:
            self.camera_sensor.detach_annotators(["rgb", "distance_to_image_plane"])
            # Release the render product and its prim wrappers before the environment closes its stage.
            self.camera_sensor = None
