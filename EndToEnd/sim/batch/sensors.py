"""Tiled RTXカメラとGPU上のIMU/LiDAR。物理時計・URDF校正は単体収集と共通。"""

import numpy as np
import torch
import warp as wp
from scipy.spatial.transform import Rotation

from g1_e2e.data.geometry import MOUNT_FROM_OPTICAL, mount_matrix, pose_matrix
from g1_e2e.data.livox import load_livox_pattern
from sim.sensors.camera import PinholeRenderer
from sim.sensors.scene import static_mesh

from .environment import tensor
from .kernels import capture_inertial, scan_livox


class VectorSensors:
    def __init__(self, env, config, schedule, *, render_updates=3):
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaaclab.utils.warp import convert_to_warp_mesh
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        self.env, self.config, self.render_updates = env, config, render_updates
        PinholeRenderer.prepare_runtime()
        from isaacsim.sensors.experimental.rtx import RtxCamera, TiledCameraSensor

        wp.set_device(env.device)
        if config.lidar.model != "livox_csv":
            raise ValueError("並列収集はLivox走査表を用います")
        self.camera_stride, self.scan_ticks = schedule.camera_ticks, schedule.scan_ticks
        self.names = list(config.body_imus.mounts)
        self.parents_cpu = np.array(
            [env.robot.body_names.index(m.body) for m in config.body_imus.mounts.values()], np.int32
        )
        transforms = [mount_matrix(m.position_m, m.rpy_deg) for m in config.body_imus.mounts.values()]
        self.transforms = dict(zip(self.names, transforms))
        self.parents = wp.array(self.parents_cpu, dtype=wp.int32, device=env.device)
        self.mount_positions = wp.array(
            np.array([m[:3, 3] for m in transforms]), dtype=wp.vec3f, device=env.device
        )
        self.mount_quats = wp.array(
            np.array([Rotation.from_matrix(m[:3, :3]).as_quat() for m in transforms]),
            dtype=wp.quatf,
            device=env.device,
        )
        self.pelvis = env.robot.body_names.index("pelvis")
        self.lidar_parent = env.robot.body_names.index(config.lidar.mount.body)
        self.lidar_mount = mount_matrix(config.lidar.mount.position_m, config.lidar.mount.rpy_deg)
        self.lidar_quat = wp.quatf(Rotation.from_matrix(self.lidar_mount[:3, :3]).as_quat())
        self.imu = wp.empty(
            (self.scan_ticks + 1, env.num_envs, len(self.names), 45), dtype=wp.float32, device=env.device
        )
        self.poses = wp.empty((self.scan_ticks + 1, env.num_envs, 2), dtype=wp.transformf, device=env.device)
        self.pattern = load_livox_pattern(env.root, config.lidar.pattern_path, config.lidar.repeat_pattern)
        self.pattern_gpu = wp.array(self.pattern.directions, dtype=wp.vec3f, device=env.device)
        points = round(config.lidar.period_s * self.pattern.rate)
        self.scan = wp.empty((env.num_envs, points, 12), dtype=wp.float32, device=env.device)
        stage = omni.usd.get_context().get_stage()
        vertices, faces, self.mesh_paths = static_mesh(stage, ["/World/envs/env_0/Office"])
        vertices -= env.origins_cpu[0]
        self.vertices, self.faces = vertices, faces
        self.mesh = convert_to_warp_mesh(vertices, faces.reshape(-1), device=str(env.device))
        self.color_mount = (
            mount_matrix(config.camera.mount.position_m, config.camera.mount.rpy_deg) @ MOUNT_FROM_OPTICAL
        )
        color_paths, cameras = [], []
        for i in range(env.num_envs):
            robot_prim = stage.GetPrimAtPath(f"/World/envs/env_{i}/G1")
            parents = [
                str(prim.GetPath())
                for prim in Usd.PrimRange(robot_prim)
                if prim.GetName() == config.camera.mount.body and prim.HasAPI(UsdPhysics.RigidBodyAPI)
            ]
            if len(parents) != 1:
                raise ValueError(f"カメラ取り付け親の剛体を一意に解決できません: {parents}")
            parent = parents[0]
            path = f"{parent}/E2eColor"
            camera = PinholeRenderer.define_camera(
                path, config.camera.width, config.camera.height, config.camera.intrinsics, config.camera
            )
            # USD cameraはOpenGL軸。保存するextrinsicは光学軸。
            opengl = self.color_mount @ np.diag([1, -1, -1, 1])
            UsdGeom.Xformable(camera).MakeMatrixXform().Set(Gf.Matrix4d(opengl.T.tolist()))
            cameras.append(camera)
            color_paths.append(path)
        self.colors = TiledCameraSensor(
            RtxCamera(color_paths, tick_rate=0.0),
            resolution=(config.camera.height, config.camera.width),
            annotators=["rgb"],
        )
        for camera in cameras:
            camera.GetVerticalApertureAttr().Set(
                float(
                    config.camera.height * camera.GetFocalLengthAttr().Get() / config.camera.intrinsics[1, 1]
                )
            )
            PinholeRenderer.configure_exposure(camera, config.camera)
        self.colors.render_product.CreatePixelAspectRatioAttr(
            float(config.camera.intrinsics[1, 1] / config.camera.intrinsics[0, 0])
        )
        timeline = omni.timeline.get_timeline_interface()
        rep.orchestrator.set_capture_on_play(True)
        timeline.set_auto_update(False)
        timeline.commit()

    @property
    def calibration(self):
        return {
            "camera/intrinsics": self.config.camera.intrinsics,
            **{f"imu/{name}/parent_from_sensor": value for name, value in self.transforms.items()},
            "lidar/ray_directions_sensor": np.empty((0, 3), np.float32),
            "lidar/ring": np.empty(0, np.int32),
            "lidar/point_time_offset_s": np.empty(0, np.float64),
        }

    def capture_inertial(self, tick):
        data = self.env.robot.data
        arrays = [
            wp.from_torch(tensor(getattr(data, key)), dtype=dtype)
            for key, dtype in (
                ("body_link_pos_w", wp.vec3f),
                ("body_link_quat_w", wp.quatf),
                ("body_com_pos_w", wp.vec3f),
                ("body_com_lin_acc_w", wp.vec3f),
                ("body_com_ang_vel_w", wp.vec3f),
                ("body_com_ang_acc_w", wp.vec3f),
            )
        ]
        wp.launch(
            capture_inertial,
            dim=(self.env.num_envs, len(self.names)),
            inputs=[
                *arrays,
                wp.from_torch(self.env.origins, dtype=wp.vec3f),
                self.parents,
                self.mount_positions,
                self.mount_quats,
                self.pelvis,
                self.lidar_parent,
                wp.vec3f(self.lidar_mount[:3, 3]),
                self.lidar_quat,
                wp.vec3f(self.env.sim.cfg.gravity),
                tick,
                self.imu,
                self.poses,
            ],
            device=self.env.device,
        )

    def capture_scan(self, start_index):
        cfg = self.config.lidar
        wp.launch(
            scan_livox,
            dim=self.scan.shape[:2],
            inputs=[
                self.mesh.id,
                self.poses,
                self.pattern_gpu,
                start_index,
                float(self.pattern.rate),
                self.env.config.physics_dt,
                cfg.near_m,
                cfg.far_m,
                self.scan,
            ],
            device=self.env.device,
        )
        return self.scan.numpy()

    def _capture_pixels(self):
        from isaacsim.core.rendering_manager import RenderingManager

        self.env.sim.forward()
        for update in range(max(30, self.render_updates)):
            RenderingManager.render()
            if update + 1 < self.render_updates:
                continue
            rgb = self.colors.get_data("rgb")[0]
            if rgb is not None:
                return rgb.numpy()
        raise RuntimeError("タイルRGBカメラの準備が完了しません")

    def warmup(self):
        # 材質・描画bufferを、物理時刻を進めず5更新してから収集へ渡す。
        for _ in range(5):
            self._capture_pixels()

    def capture_camera(self):
        rgb = self._capture_pixels()
        maxima = np.max(rgb, axis=(1, 2, 3))
        if np.any(maxima == 0):
            raise RuntimeError(f"RGBタイルが全黒です: per-environment maxima={maxima.tolist()}")
        data = self.env.robot.data
        parent = self.env.robot.body_names.index(self.config.camera.mount.body)
        packed = (
            torch.cat(
                [
                    tensor(data.body_link_pos_w)[:, parent],
                    tensor(data.body_link_quat_w)[:, parent],
                    tensor(data.root_pos_w),
                    tensor(data.root_quat_w),
                ],
                dim=1,
            )
            .cpu()
            .numpy()
        )
        extrinsics = []
        for i, row in enumerate(packed):
            origin = self.env.origins_cpu[i]
            body = pose_matrix(row[:3] - origin, row[3:7])
            base = pose_matrix(row[7:10] - origin, row[10:14])
            color = body @ self.color_mount
            extrinsics.append({"world_from_optical": color, "base_from_optical": np.linalg.inv(base) @ color})
        return {"rgb": rgb}, extrinsics

    def roll_scan_buffer(self):
        wp.copy(self.imu[0], self.imu[self.scan_ticks])
        wp.copy(self.poses[0], self.poses[self.scan_ticks])

    def close(self):
        self.colors.detach_annotators(["rgb"])
        self.colors = None
