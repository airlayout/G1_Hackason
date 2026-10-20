"""RGBとnative depthの別光学系を、公開された内部・外部校正値で描画する。"""

import numpy as np

from g1_e2e.data.frames import CameraFrame
from g1_e2e.data.depth import align_depth, quantize_depth


class PinholeRenderer:
    def __init__(self, path, width, height, intrinsics, config, annotators):
        import omni.usd
        from pxr import Gf, UsdGeom
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

        self.annotators, self.config = annotators, config
        self.camera = UsdGeom.Camera.Define(omni.usd.get_context().get_stage(), path)
        focal = 24.0
        fx, fy, cx, cy = intrinsics[0, 0], intrinsics[1, 1], intrinsics[0, 2], intrinsics[1, 2]
        # RealSenseの(0,0)は左上画素の中心、USDは画像端から0.5の位置。
        # 校正KはRealSense規約のまま保存し、描画器へ渡す主点だけ変換する。
        cx, cy = cx + 0.5, cy + 0.5
        self.camera.CreateFocalLengthAttr(focal)
        self.camera.CreateHorizontalApertureAttr(float(width * focal / fx))
        self.camera.CreateVerticalApertureAttr(float(height * focal / fy))
        self.camera.CreateHorizontalApertureOffsetAttr(float((width / 2 - cx) * focal / fx))
        self.camera.CreateVerticalApertureOffsetAttr(float((cy - height / 2) * focal / fy))
        self.camera.CreateClippingRangeAttr(Gf.Vec2f(config.near_m, config.far_m))
        self.sensor = CameraSensor(
            RtxCamera(path, tick_rate=0.0), resolution=(height, width), annotators=annotators
        )
        self.camera.GetVerticalApertureAttr().Set(float(height * focal / fy))
        self.sensor.render_product.CreatePixelAspectRatioAttr(float(fy / fx))
        prim = self.camera.GetPrim()
        for name, value in {
            "omni:rtx:autoExposure:enabled": False,
            "exposure:iso": config.exposure_iso,
            "exposure:time": config.exposure_time_s,
            "exposure:fStop": config.exposure_f_stop,
        }.items():
            prim.GetAttribute(name).Set(value)

    def capture(self, world_from_optical):
        from isaacsim.core.rendering_manager import RenderingManager
        from pxr import Gf, UsdGeom

        eye = world_from_optical[:3, 3]
        target, up = eye + world_from_optical[:3, 2], -world_from_optical[:3, 1]
        UsdGeom.Xformable(self.camera).MakeMatrixXform().Set(
            Gf.Matrix4d()
            .SetLookAt(Gf.Vec3d(*map(float, eye)), Gf.Vec3d(*map(float, target)), Gf.Vec3d(*map(float, up)))
            .GetInverse()
        )
        for update in range(max(30, self.config.render_updates)):
            RenderingManager.render()
            if update + 1 < self.config.render_updates:
                continue
            data = {name: self.sensor.get_data(name)[0] for name in self.annotators}
            if all(value is not None for value in data.values()):
                return {name: value.numpy().copy() for name, value in data.items()}
        raise RuntimeError("カメラ描画が完了しませんでした")

    def close(self):
        self.sensor.detach_annotators(self.annotators)
        self.sensor = None


class CalibratedRgbdCamera:
    def __init__(self, config):
        self.config = config
        native = config.depth
        self.color = PinholeRenderer(
            "/World/G1Sensors/Camera",
            config.width,
            config.height,
            config.intrinsics,
            config,
            ["rgb"] if native else ["rgb", "distance_to_image_plane"],
        )
        self.depth = (
            None
            if native is None
            else PinholeRenderer(
                "/World/G1Sensors/Depth",
                native.pinhole.width,
                native.pinhole.height,
                native.pinhole.resized(native.pinhole.width, native.pinhole.height),
                config,
                ["distance_to_image_plane"],
            )
        )

    def capture(self, episode_id, time_s, world_from_optical, world_from_base):
        config, native = self.config, self.config.depth
        color = self.color.capture(world_from_optical)
        extra = {}
        if native is not None:
            world_from_depth = world_from_optical @ np.linalg.inv(native.color_from_depth)
            raw = self.depth.capture(world_from_depth)["distance_to_image_plane"][..., 0]
            valid = np.isfinite(raw) & (raw >= config.near_m) & (raw <= config.far_m)
            z16, valid = quantize_depth(raw, valid, native.scale_m)
            aligned, aligned_valid = align_depth(
                z16,
                valid,
                depth_intrinsics=native.pinhole.resized(native.pinhole.width, native.pinhole.height),
                color_intrinsics=config.intrinsics,
                color_from_depth=native.color_from_depth,
                scale_m=native.scale_m,
                color_shape=(config.height, config.width),
            )
            extra = dict(
                native_depth_z16=z16,
                native_depth_valid=valid,
                world_from_depth_optical=world_from_depth.astype(np.float32),
            )
        else:
            raw = color["distance_to_image_plane"][..., 0]
            aligned_valid = np.isfinite(raw) & (raw >= config.near_m) & (raw <= config.far_m)
            aligned = np.where(aligned_valid, raw, 0).astype(np.float32)
        return CameraFrame(
            episode_id,
            time_s,
            color["rgb"],
            aligned,
            aligned_valid,
            world_from_optical,
            np.linalg.inv(world_from_base) @ world_from_optical,
            **extra,
        )

    def close(self):
        if self.depth is not None:
            self.depth.close()
        self.color.close()
