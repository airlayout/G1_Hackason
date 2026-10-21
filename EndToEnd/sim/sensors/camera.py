"""RGB光学系を、公開された内部校正値で描画する。"""

import numpy as np

from g1_e2e.data.frames import CameraFrame


class PinholeRenderer:
    @staticmethod
    def prepare_runtime():
        import carb
        import omni.kit.app

        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("isaacsim.sensors.experimental.rtx", True)
        carb.settings.get_settings().set("/rtx/rendermode", "PathTracing")

    @staticmethod
    def define_camera(path, width, height, intrinsics, config):
        import omni.usd
        from pxr import Gf, UsdGeom

        camera = UsdGeom.Camera.Define(omni.usd.get_context().get_stage(), path)
        focal = 24.0
        fx, fy, cx, cy = intrinsics[0, 0], intrinsics[1, 1], intrinsics[0, 2], intrinsics[1, 2]
        # RealSenseの(0,0)は左上画素の中心、USDは画像端から0.5の位置。
        # 校正KはRealSense規約のまま保存し、描画器へ渡す主点だけ変換する。
        cx, cy = cx + 0.5, cy + 0.5
        camera.CreateFocalLengthAttr(focal)
        camera.CreateHorizontalApertureAttr(float(width * focal / fx))
        camera.CreateVerticalApertureAttr(float(height * focal / fy))
        camera.CreateHorizontalApertureOffsetAttr(float((width / 2 - cx) * focal / fx))
        camera.CreateVerticalApertureOffsetAttr(float((cy - height / 2) * focal / fy))
        camera.CreateClippingRangeAttr(Gf.Vec2f(config.near_m, config.far_m))
        return camera

    @staticmethod
    def configure_exposure(camera, config):
        from pxr import Sdf

        prim = camera.GetPrim()
        for name, value in {
            "omni:rtx:autoExposure:enabled": False,
            "exposure:iso": config.exposure_iso,
            "exposure:time": config.exposure_time_s,
            "exposure:fStop": config.exposure_f_stop,
        }.items():
            attribute = prim.GetAttribute(name)
            if not attribute or not attribute.GetTypeName():
                attribute = prim.CreateAttribute(
                    name, Sdf.ValueTypeNames.Bool if isinstance(value, bool) else Sdf.ValueTypeNames.Float
                )
            attribute.Set(value)

    def __init__(self, path, width, height, intrinsics, config, annotators):
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

        self.annotators, self.config = annotators, config
        self.camera = self.define_camera(path, width, height, intrinsics, config)
        self.sensor = CameraSensor(
            RtxCamera(path, tick_rate=0.0), resolution=(height, width), annotators=annotators
        )
        focal = self.camera.GetFocalLengthAttr().Get()
        fx, fy = intrinsics[0, 0], intrinsics[1, 1]
        self.camera.GetVerticalApertureAttr().Set(float(height * focal / fy))
        self.sensor.render_product.CreatePixelAspectRatioAttr(float(fy / fx))
        self.configure_exposure(self.camera, config)

    def set_pose(self, world_from_optical):
        from pxr import Gf, UsdGeom

        eye = world_from_optical[:3, 3]
        target, up = eye + world_from_optical[:3, 2], -world_from_optical[:3, 1]
        UsdGeom.Xformable(self.camera).MakeMatrixXform().Set(
            Gf.Matrix4d()
            .SetLookAt(Gf.Vec3d(*map(float, eye)), Gf.Vec3d(*map(float, target)), Gf.Vec3d(*map(float, up)))
            .GetInverse()
        )

    @staticmethod
    def capture_batch(cameras):
        """全RGBカメラの姿勢を先に設定し、同じ描画更新で取得する。"""
        from isaacsim.core.rendering_manager import RenderingManager

        updates = max(camera.config.render_updates for camera in cameras)
        for update in range(max(30, updates)):
            RenderingManager.render()
            if update + 1 < updates:
                continue
            data = [
                {name: camera.sensor.get_data(name)[0] for name in camera.annotators} for camera in cameras
            ]
            if all(value is not None for frame in data for value in frame.values()):
                return [{name: value.numpy().copy() for name, value in frame.items()} for frame in data]
        raise RuntimeError("カメラ描画が完了しませんでした")

    def close(self):
        self.sensor.detach_annotators(self.annotators)
        self.sensor = None


class CalibratedRgbCamera:
    def __init__(self, config):
        self.color = PinholeRenderer(
            "/World/G1Sensors/Camera",
            config.width,
            config.height,
            config.intrinsics,
            config,
            ["rgb"],
        )

    def capture(self, episode_id, time_s, world_from_optical, world_from_base):
        self.color.set_pose(world_from_optical)
        color = PinholeRenderer.capture_batch([self.color])[0]
        return CameraFrame(
            episode_id,
            time_s,
            color["rgb"],
            world_from_optical,
            np.linalg.inv(world_from_base) @ world_from_optical,
        )

    def close(self):
        self.color.close()
