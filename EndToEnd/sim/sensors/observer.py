"""学習入力とは別の、同じ物理時刻を撮影する追従カメラ。"""

import numpy as np


class ObserverCamera:
    resolution = (384, 512)

    def __init__(self, env):
        import omni.usd
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
        from pxr import Gf, UsdGeom

        self.env = env
        stage = omni.usd.get_context().get_stage()
        self.camera = UsdGeom.Camera.Define(stage, "/World/G1Sensors/Observer")
        self.camera.CreateFocalLengthAttr(32.0)
        self.camera.CreateHorizontalApertureAttr(36.0)
        self.camera.CreateVerticalApertureAttr(27.0)
        self.camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
        self.sensor = CameraSensor(
            RtxCamera(str(self.camera.GetPath()), tick_rate=0.0),
            resolution=self.resolution,
            annotators=["rgb"],
        )
        prim = self.camera.GetPrim()
        for name, value in {
            "omni:rtx:autoExposure:enabled": False,
            "exposure:iso": 100.0,
            "exposure:time": 0.02,
            "exposure:fStop": 5.0,
        }.items():
            prim.GetAttribute(name).Set(value)
        # 視覚上の床模様のみ。接触形状とLiDARの平面は既存のGroundが所有する。
        vertices, indices, colors = [], [], []
        for x in range(-4, 25):
            for y in range(-7, 8):
                i = len(vertices)
                vertices.extend([(x, y, 0.001), (x + 1, y, 0.001), (x + 1, y + 1, 0.001), (x, y + 1, 0.001)])
                indices.extend([i, i + 1, i + 2, i + 3])
                shade = 0.32 if (x + y) % 2 else 0.42
                colors.append((shade, shade, shade))
        floor = UsdGeom.Mesh.Define(stage, "/World/ObserverFloor")
        floor.CreatePointsAttr(vertices)
        floor.CreateFaceVertexCountsAttr([4] * len(colors))
        floor.CreateFaceVertexIndicesAttr(indices)
        floor.CreateSubdivisionSchemeAttr("none")
        floor.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform).Set(colors)

    def capture(self, observation):
        from isaacsim.core.rendering_manager import RenderingManager
        from pxr import Gf, UsdGeom

        x, y, _ = observation.privileged["base_position_world_m"]
        eye, target = Gf.Vec3d(float(x + 2.1), float(y - 3.6), 1.9), Gf.Vec3d(float(x), float(y), 0.7)
        UsdGeom.Xformable(self.camera).MakeMatrixXform().Set(
            Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
        )
        self.env.sim.forward()
        for update in range(30):
            RenderingManager.render()
            if update < 2:
                continue
            rgb, _ = self.sensor.get_data("rgb")
            if rgb is not None:
                return np.ascontiguousarray(rgb.numpy(), dtype=np.uint8)
        raise RuntimeError("外部カメラの描画が完了しませんでした")

    def close(self):
        self.sensor.detach_annotators(["rgb"])
        self.sensor = None
