"""Calibrated, exposure-time geometry; no ROS, SDK or detector imports."""
from __future__ import annotations

import math
import numpy as np


class Unmeasured(ValueError):
    """The available evidence does not support this measurement."""


def transform(value):
    T = np.asarray(value, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1])
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-6)):
        raise Unmeasured("invalid rigid transform")
    return T


def quaternion(wxyz):
    q = np.asarray(wxyz, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < .5:
        raise Unmeasured("invalid IMU quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rotation(axis, angle):
    axis = np.asarray(axis, float)
    if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-9:
        raise Unmeasured("invalid calibrated joint axis")
    x, y, z = axis / np.linalg.norm(axis)
    A = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*A + (1-math.cos(angle))*(A @ A)


def interpolate_state(states, t, max_gap):
    """Interpolate bounded encoder angles and quaternion; never extrapolate."""
    ordered = sorted(states, key=lambda s: s["t"])
    times = np.array([s["t"] for s in ordered], float)
    if not len(times) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise Unmeasured("missing/duplicate body timestamps")
    if t < times[0] or t > times[-1]:
        raise Unmeasured("exposure outside body history")
    right = int(np.searchsorted(times, t))
    if times[right] == t:
        a = b = ordered[right]
        ratio = 0.
    else:
        a, b = ordered[right-1], ordered[right]
        if b["t"]-a["t"] > max_gap:
            raise Unmeasured("body history gap at exposure")
        ratio = (t-a["t"])/(b["t"]-a["t"])
    qa, qb = np.asarray(a["q"], float), np.asarray(b["q"], float)
    if qa.shape != (29,) or qb.shape != (29,) or not np.isfinite([qa, qb]).all():
        raise Unmeasured("invalid 29-joint body observation")
    ua, ub = np.asarray(a["imu_wxyz"], float), np.asarray(b["imu_wxyz"], float)
    quaternion(ua)
    quaternion(ub)
    ua, ub = ua/np.linalg.norm(ua), ub/np.linalg.norm(ub)
    if ua @ ub < 0:
        ub = -ub
    angle = math.acos(float(np.clip(ua @ ub, -1, 1)))
    u = ((1-ratio)*ua + ratio*ub if angle < 1e-6 else
         (math.sin((1-ratio)*angle)*ua + math.sin(ratio*angle)*ub)/math.sin(angle))
    return {"q": (qa+(qb-qa)*ratio), "imu_wxyz": u,
            "stamp_kinds": sorted({a.get("stamp_kind", "unknown"), b.get("stamp_kind", "unknown")})}


def kinematic_chain(joints, tail, q):
    T = np.eye(4)
    for joint in joints:
        if not isinstance(joint["q_index"], int) or not 0 <= joint["q_index"] < len(q):
            raise Unmeasured("calibrated joint index outside body state")
        T = T @ transform(joint["T_parent_joint"])
        R = np.eye(4)
        R[:3, :3] = rotation(joint["axis"], float(q[joint["q_index"]])+joint.get("offset_rad", 0))
        T = T @ R
    return T @ transform(tail)


def camera_transform(profile, q):
    if not profile.get("measured"):
        raise Unmeasured("camera extrinsics are not measured")
    return kinematic_chain(profile.get("joints", []), profile["T_last_optical"], q)


def polygon_mask(shape, polygon):
    p = np.asarray(polygon, float)
    if (p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all()
            or np.any(p < 0) or np.any(p[:, 0] >= shape[1]) or np.any(p[:, 1] >= shape[0])):
        raise Unmeasured("panel polygon is outside the image")
    y, x = np.indices(shape)
    inside = np.zeros(shape, bool)
    for a, b in zip(p, np.roll(p, -1, axis=0)):
        if a[1] != b[1]:
            inside ^= ((a[1] > y) != (b[1] > y)) & (x < (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0])
    return inside


def panel_pose(depth, K, annotation, T_pelvis_optical, up_pelvis, min_points=50):
    """Panel origin is the button ray's projection onto the measured panel plane.

    This deliberately does NOT estimate button protrusion, press stroke or reachability.
    Panel +x points towards the robot; +z follows gravity projected onto the panel.
    """
    depth, K, T = np.asarray(depth), np.asarray(K, float), transform(T_pelvis_optical)
    if (depth.ndim != 2 or K.shape != (3, 3) or not np.isfinite(K).all()
            or K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1])):
        raise Unmeasured("missing/invalid aligned depth or intrinsics")
    mask = polygon_mask(depth.shape, annotation["panel_polygon"])
    uv = np.asarray(annotation["button_uv"], float)
    if uv.shape != (2,) or not np.isfinite(uv).all() or not (0 <= uv[0] < depth.shape[1] and 0 <= uv[1] < depth.shape[0]):
        raise Unmeasured("button annotation outside image")
    if not mask[int(uv[1]), int(uv[0])]:
        raise Unmeasured("button annotation outside measured panel polygon")
    # Exclude the button itself when fitting the panel surface.
    y, x = np.indices(depth.shape)
    mask &= (x-uv[0])**2+(y-uv[1])**2 > annotation.get("button_exclusion_px", 4)**2
    mask &= np.isfinite(depth) & (depth > .05) & (depth < 5.)
    v, u = np.where(mask)
    if len(u) < min_points:
        raise Unmeasured("insufficient panel depth")
    stride = max(1, len(u)//4000)
    u, v = u[::stride], v[::stride]
    P = (np.linalg.solve(K, np.vstack([u, v, np.ones(len(u))])) * depth[v, u]).T
    for _ in range(3):
        center = P.mean(axis=0)
        _, singular, V = np.linalg.svd(P-center, full_matrices=False)
        if singular[1] < .005:
            raise Unmeasured("degenerate panel depth geometry")
        n = V[-1]
        residual = (P-center) @ n
        keep = np.abs(residual) <= max(.002, 3*1.4826*np.median(np.abs(residual)))
        if keep.sum() < min_points:
            raise Unmeasured("insufficient panel plane inliers")
        P = P[keep]
    center = P.mean(axis=0)
    _, _, V = np.linalg.svd(P-center, full_matrices=False)
    n = V[-1]
    rms = float(np.sqrt(np.mean(((P-center) @ n)**2)))
    if rms > .01:
        raise Unmeasured("panel plane residual exceeds 1 cm")
    ray = np.linalg.solve(K, [*uv, 1.])
    denominator = float(n @ ray)
    if abs(denominator) < .1:
        raise Unmeasured("button ray grazes panel")
    distance = float(n @ center)/denominator
    if not .05 < distance < 5:
        raise Unmeasured("button projection is behind/outside camera range")
    origin = T[:3, :3] @ (distance*ray)+T[:3, 3]
    normal = T[:3, :3] @ n
    if normal @ (-origin) < 0:
        normal = -normal
    up = np.asarray(up_pelvis, float)
    z = up-normal*(up @ normal)
    if not np.isfinite(z).all() or np.linalg.norm(z) < .5:
        raise Unmeasured("panel normal incompatible with measured gravity")
    z /= np.linalg.norm(z)
    basis = np.column_stack([normal, np.cross(z, normal), z])
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = basis.T, -basis.T @ origin
    fwd = basis.T @ np.array([1., 0, 0])
    return {"T_panel_pelvis": out.tolist(), "stand_off_m": float(out[0, 3]),
            "lateral_m": float(out[1, 3]), "height_m": float(out[2, 3]),
            "heading_rad": math.atan2(-fwd[1], -fwd[0]),
            "plane_rms_m": rms, "plane_points": len(P)}


def pose_matrix(xyz, xyzw):
    T = np.eye(4)
    T[:3, 3] = np.asarray(xyz, float)
    x, y, z, w = xyzw
    T[:3, :3] = quaternion([w, x, y, z])
    return transform(T)


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi
