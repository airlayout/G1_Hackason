"""Optional Isaac adapter. Import only after AppLauncher starts the application."""
from __future__ import annotations

from types import SimpleNamespace
import numpy as np

from .types import CameraFrame


def make_isaac_robot(scene_cfg,contest_cfg,cfg):
    from contest.robots.isaac_robot import IsaacRobot, _torch
    import isaac_world as world
    from scipy.spatial.transform import Rotation
    from isaaclab.sensors import CameraCfg
    import isaaclab.sim as sim_utils
    from pxr import Usd, UsdPhysics
    original=world.Camera
    extras={}

    def camera_factory(camera_cfg):
        camera=original(camera_cfg)
        if camera_cfg.prim_path.endswith("/head_camera"):
            stage=sim_utils.get_current_stage()
            w=cfg["wrist"]
            link=world.find_link_path(stage,world.ROBOT_PATH,w["parent"])
            if cfg["use_wrist"]:
                h,width=w["height"],w["width"]
                fy=h/2/np.tan(np.radians(w["fovy_deg"])/2)
                rot=tuple(Rotation.from_euler("xyz",w["rpy_deg"],degrees=True).as_quat().tolist())
                wrist_cfg=CameraCfg(prim_path=f"{link}/press_wrist_camera",
                    offset=CameraCfg.OffsetCfg(pos=tuple(w["xyz"]),rot=rot,convention="ros"),
                    width=width,height=h,data_types=["rgb","distance_to_image_plane"],
                    spawn=sim_utils.PinholeCameraCfg(focal_length=fy*20.955/width,horizontal_aperture=20.955,
                                                     clipping_range=(.02,20.)))
                extras["wrist"]=original(wrist_cfg)
            mount=sim_utils.CuboidCfg(size=tuple(w["mount_size_m"]),
                                      collision_props=sim_utils.CollisionPropertiesCfg(),
                                      visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.1,.15,.2)))
            mount.func(f"{link}/press_camera_mount",mount,translation=tuple(w["xyz"]))
            for side in ("left","right"):
                path=world.find_link_path(stage,world.ROBOT_PATH,f"{side}_rubber_hand")
                for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
                    if prim.GetTypeName()=="Mesh":
                        UsdPhysics.CollisionAPI.Apply(prim)
                        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            if cfg["mode"]=="bottle":
                # Same fixed fixture coordinates as MuJoCo, relative to pelvis.
                z=float(scene_cfg["pelvis_height_m"] if "pelvis_height_m" in scene_cfg else world.build_hall_scene(scene_cfg).pelvis_height)-.035
                for name,radius,height,offset,color in (
                    ("body",.035,.21,0,(.65,.8,.9)),("label",.0355,.08,-.005,(.05,.2,.65)),
                    ("neck",.014,.05,.12,(.65,.8,.9)),("cap",.017,.016,.15,(.05,.2,.65))):
                    shape=sim_utils.CylinderCfg(radius=radius,height=height,
                        collision_props=sim_utils.CollisionPropertiesCfg(),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color))
                    shape.func(f"/World/press_bottle_{name}",shape,translation=(.4,-.18,z+offset))
        return camera

    world.Camera=camera_factory
    try:
        base=IsaacRobot(scene_cfg,contest_cfg)
    finally:
        world.Camera=original
    base.w.cameras.update(extras)

    class ExtendedIsaac:
        name,base_enabled="isaac",False
        def __getattr__(self,name):
            return getattr(base,name)
        def observe(self,t):
            obs=base.observe(t)
            data=dict(vars(obs))
            data["image_t"]=t
            if "wrist" in extras:
                camera=extras["wrist"]
                out=camera.data.output
                rgb=_torch(out["rgb"])[0,...,:3].cpu().numpy().astype(np.uint8)
                depth=_torch(out["distance_to_image_plane"])[0].squeeze(-1).cpu().numpy().astype(np.float32)
                depth[~np.isfinite(depth)]=0
                K=_torch(camera.data.intrinsic_matrices)[0].cpu().numpy().astype(float)
                # Nominal calibrated mounting + measured joints, not target ground truth.
                if not hasattr(self,"kinematics"):
                    from .model import RobotModel
                    self.kinematics=RobotModel(cfg)
                T=self.kinematics.wrist_transform(obs.q)
                data["wrist"]=CameraFrame(rgb,depth,K,t,obs.q.copy(),T,"wrist",int(round(t*1e6)))
            return SimpleNamespace(**data)
    return ExtendedIsaac()
