"""Mesh-level self checks and conservative sensed wall/target constraints."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import numpy as np

from .model import ARM
from .types import unit


@dataclass
class Shape:
    name: str
    geometry: object
    parent: int
    placement: object
    vertices: np.ndarray
    radius: float = 0.0


class CollisionChecker:
    def __init__(self, robot, cfg):
        import coal
        from scipy.spatial import ConvexHull
        self.coal, self.robot, self.cfg = coal, robot, cfg
        pin, model = robot.pin, robot.model
        urdf = robot.robot_cfg["model"]["urdf_path"]
        dirs = [str(Path(urdf).parent)]
        collision = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.COLLISION, package_dirs=dirs)
        visual = pin.buildGeomFromUrdf(model, urdf, pin.GeometryType.VISUAL, package_dirs=dirs)
        objects = list(collision.geometryObjects)
        # Official URDF omits the plastic hand from collision geometry.
        objects += [o for o in visual.geometryObjects if "rubber_hand" in o.meshPath]
        self.shapes = []
        for o in objects:
            g = o.geometry
            g.computeLocalAABB()
            if hasattr(g, "vertices"):
                v = np.asarray(g.vertices()).copy()
                if len(v) > 8:
                    v = v[ConvexHull(v).vertices]
            else:
                # Box of a primitive is conservative for wall clearance.
                lo, hi = np.array(g.aabb_local.min_), np.array(g.aabb_local.max_)
                v = np.array([[x, y, z] for x in [lo[0], hi[0]] for y in [lo[1], hi[1]] for z in [lo[2], hi[2]]])
            self.shapes.append(Shape(o.name, g, o.parentJoint, o.placement, v))
        w = cfg["wrist"]
        frame = model.frames[model.getFrameId(w["parent"])]
        placement = frame.placement * pin.SE3(np.eye(3), np.asarray(w["xyz"]))
        size = np.asarray(w["mount_size_m"])
        vertices = np.array([[x, y, z] for x in [-size[0]/2, size[0]/2]
                             for y in [-size[1]/2, size[1]/2] for z in [-size[2]/2, size[2]/2]])
        self.shapes.append(Shape("wrist_camera_mount", coal.Box(*size), frame.parentJoint, placement, vertices))
        self.moving = [i for i, s in enumerate(self.shapes) if s.parent >= 23]
        self.pairs = []
        for i in self.moving:
            a = self.shapes[i]
            for j, b in enumerate(self.shapes):
                if j in self.moving and j <= i:
                    continue
                # Adjacent articulations have intentional overlaps in the URDF.
                pa, pb = a.parent, b.parent
                if pa == pb or model.parents[pa] == pb or model.parents[pb] == pa:
                    continue
                # The roll/yaw housings of the compact wrist overlap at permitted
                # pitch angles; this is an articulated assembly, not an arm/body pair.
                if {a.name,b.name} == {"right_wrist_roll_link_0","right_wrist_yaw_link_0"}:
                    continue
                if "rubber_hand" in a.name and b.parent in (27, 28, 29):
                    continue
                if "rubber_hand" in b.name and a.parent in (27, 28, 29):
                    continue
                self.pairs.append((i, j))
        self.obstacles = []
        for ob in cfg["obstacles"]:
            center, size = np.asarray(ob["center"], float), np.asarray(ob["size"], float)
            if center.shape != (3,) or size.shape != (3,) or not np.isfinite([center, size]).all() or np.any(size <= 0):
                raise ValueError("invalid measured obstacle box")
            self.obstacles.append((coal.Box(*size), coal.Transform3s(np.eye(3), center)))
        self.last_reason = ""
        self.bounds = []
        for s in self.shapes:
            s.geometry.computeLocalAABB()
            lo, hi = np.array(s.geometry.aabb_local.min_), np.array(s.geometry.aabb_local.max_)
            self.bounds.append(np.array([[x,y,z] for x in (lo[0],hi[0])
                                         for y in (lo[1],hi[1]) for z in (lo[2],hi[2])]))

    def _distance(self, a, Ta, b, Tb):
        c = self.coal
        return c.distance(a, c.Transform3s(Ta.rotation, Ta.translation), b, Tb,
                          c.DistanceRequest(), c.DistanceResult())

    def valid(self, q, target=None, contact=False, padding_m=0.0):
        r, c = self.robot, self.coal
        if (np.asarray(q).shape != (29,) or not np.isfinite(q).all()
                or np.any(q[ARM] < r.lower) or np.any(q[ARM] > r.upper)):
            if np.asarray(q).shape==(29,) and np.isfinite(q).all():
                self.last_reason = "joint limit: "+str({int(22+i):float(q[22+i]) for i in range(7)
                    if q[22+i]<r.lower[i] or q[22+i]>r.upper[i]})
            else:
                self.last_reason="invalid joint state"
            return False
        r.pin.forwardKinematics(r.model, r.data, q)
        transforms = [r.data.oMi[s.parent] * s.placement for s in self.shapes]
        margin = self.cfg["clearance_m"]
        world_margin=margin+padding_m
        bounds = [v @ T.rotation.T + T.translation for v, T in zip(self.bounds, transforms)]
        lows, highs = np.array([v.min(axis=0) for v in bounds]), np.array([v.max(axis=0) for v in bounds])
        for i, j in self.pairs:
            a, b = self.shapes[i], self.shapes[j]
            Tb = transforms[j]
            # Shoulder interfaces are close even at a valid neutral pose.
            gap = 0.0 if a.parent in (23, 24, 25) else margin
            separation = np.maximum(0, np.maximum(lows[i]-highs[j], lows[j]-highs[i]))
            if np.linalg.norm(separation) > gap + 1e-5:
                continue
            if self._distance(a.geometry, transforms[i], b.geometry, c.Transform3s(Tb.rotation, Tb.translation)) < gap - 1e-5:
                self.last_reason = f"self collision: {a.name} / {b.name}"
                return False
        for i in self.moving:
            a, T = self.shapes[i], transforms[i]
            for ob, Tob in self.obstacles:
                if self._distance(a.geometry, T, ob, Tob) < world_margin:
                    self.last_reason = f"obstacle: {a.name}"
                    return False
            if target is None:
                continue
            vertices = a.vertices @ T.rotation.T + T.translation
            if target.kind == "button":
                face = target.wall_point if target.wall_point is not None else target.point
                # During pressing only the fingertip ball is exempt; the hand mesh is not.
                gap = 0.001 if contact else world_margin
                if np.max((vertices - face) @ target.normal) > -gap:
                    self.last_reason = f"wall clearance: {a.name}"
                    return False
            else:
                # Fixed bottle is a finite cylinder, not a panel plane.
                center = target.point + self.cfg["bottle"]["radius_m"] * target.normal
                cylinder = c.Cylinder(self.cfg["bottle"]["radius_m"], 0.24)
                if self._distance(a.geometry, T, cylinder, c.Transform3s(np.eye(3), center)) < (0.0005 if contact else world_margin):
                    self.last_reason = f"bottle clearance: {a.name}"
                    return False
        if target is not None:
            tip = r.tip(q)
            delta = tip - target.point
            signed = float(delta @ target.normal)
            lateral = np.linalg.norm(delta - signed * target.normal)
            penetration = signed + self.cfg["tip_radius_m"]
            if contact:
                limit = self.cfg[target.kind]["max_command_penetration_m"]
                if (penetration > -margin and lateral > target.radius - self.cfg["tip_radius_m"]) or penetration > limit:
                    self.last_reason = "tip outside permitted target contact"
                    return False
            elif penetration > -world_margin:
                self.last_reason = "tip too close to surface"
                return False
        self.last_reason = ""
        return True

    def edge(self, start, end, target=None, contact=False, padding_m=0.0):
        # Both command interpolation and measured-to-command segment are checked.
        step = self.cfg["edge_resolution_rad"]
        count = max(2, int(np.ceil(np.max(np.abs(end[ARM] - start[ARM])) / step)) + 1)
        return all(self.valid(start + (end-start)*s, target, contact,padding_m) for s in np.linspace(0, 1, count))
