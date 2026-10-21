"""選択URDFの公称取り付けと、実行中の剛体・保存座標変換を照合する。"""

from dataclasses import asdict
import hashlib

import numpy as np

from g1_e2e.data.config import MountConfig
from g1_e2e.data.geometry import MOUNT_FROM_OPTICAL, base_pose, mount_matrix


def mounting_audit(env, rig, urdf):
    from pxr import Usd, UsdGeom, UsdPhysics

    current = env.observe()
    frames = rig.imus.capture(current.episode_id, current.simulation_time_s)
    result = {
        "urdf": str(urdf),
        "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
        "individual_hardware_calibration_verified": False,
        "mounts": {},
    }
    definitions = {
        "camera": rig.config.camera.mount,
        "lidar": rig.config.lidar.mount,
        **{f"imu_{name}": mount for name, mount in rig.config.body_imus.mounts.items()},
    }
    for name, mount in definitions.items():
        if mount.urdf_joint is None:
            raise ValueError(f"{name}: URDFに基づく取り付けがありません")
        expected = MountConfig.from_urdf(urdf, mount.urdf_joint)
        reference = mount_matrix(expected.position_m, expected.rpy_deg)
        actual = mount_matrix(mount.position_m, mount.rpy_deg)
        if (
            mount.body != expected.body
            or mount.body not in env.robot.body_names
            or not np.allclose(actual, reference, atol=1e-7)
        ):
            raise ValueError(f"{name}: 実行時の取り付けが選択URDFと一致しません")
        if name.startswith("imu_"):
            frame = frames[name[4:]]
            if not np.allclose(
                base_pose(current) @ frame.base_from_sensor, frame.world_from_sensor, atol=1e-5
            ):
                raise ValueError(f"{name}: 保存するbase/world変換が不整合です")
        result["mounts"][name] = {
            **asdict(mount),
            "parent_from_sensor": actual.tolist(),
            "matches_urdf": True,
        }
    camera_world = rig._body_world(rig.config.camera.mount.body) @ rig.camera_mount @ MOUNT_FROM_OPTICAL
    result["camera_world_from_optical_at_reset"] = camera_world.tolist()
    # USD cameraのOpenGL軸(+X右,+Y上,-Z前)を保存するoptical軸へ戻して独立に照合する。
    optical_from_gl = np.diag([1, -1, -1, 1])
    color_usd = np.asarray(
        UsdGeom.Xformable(rig.camera.color.camera).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    ).T
    if not np.allclose(color_usd @ optical_from_gl, camera_world, atol=1e-5):
        raise ValueError("描画中のRGBカメラの位置・光軸がURDF取り付けと一致しません")
    result["rendered_rgb_pose_matches_mount"] = True

    result["lidar_imu_note"] = (
        "MID-360 IMU is at the nominal LiDAR URDF frame; internal factory IMU-to-optical calibration is unavailable"
    )
    stage = env.sim.stage
    root_paths = rig.static_roots
    result["static_colliders"] = sum(
        int(
            prim.HasAPI(UsdPhysics.CollisionAPI)
            and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False
        )
        for path in root_paths
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path), Usd.TraverseInstanceProxies())
    )
    if not result["static_colliders"]:
        raise ValueError("シーンに有効な物理collisionがありません")
    return result
