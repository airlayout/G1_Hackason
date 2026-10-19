"""URDF kinematics and whole-arm geometry. No simulator imports."""
from __future__ import annotations

import numpy as np

from .runtime import j1_module, robot_config
from .types import unit, vector

ARM = np.arange(22, 29)


class RobotModel:
    def __init__(self, cfg):
        import pinocchio as pin
        self.pin = pin
        self.cfg = cfg
        self.robot_cfg = robot_config()
        self.robot_cfg["end_effector"]["right"]["offset"] = cfg["tip_offset_m"]
        ik_cfg = j1_module("config").load_config("press.yaml")["ik"]
        ik_cfg.update(tol_pos_m=0.0004, tol_axis_deg=6, max_iter=250)
        self.kin = j1_module("kinematics").ArmKinematics(self.robot_cfg, "right", ik_cfg)
        self.model, self.data = self.kin.model, self.kin.model.createData()
        self.lower = self.model.lowerPositionLimit[ARM] + 0.025
        self.upper = self.model.upperPositionLimit[ARM] - 0.025
        self.camera_tf = j1_module("camera_geometry").HeadCameraTransform(self.robot_cfg)

    def tip(self, q):
        return self.kin.fk_pos(vector(q, 29))

    def transform(self, q, frame):
        self.pin.framesForwardKinematics(self.model, self.data, vector(q, 29))
        if not self.model.existFrame(frame):
            raise ValueError(f"frame missing from URDF: {frame}")
        return self.data.oMf[self.model.getFrameId(frame)].homogeneous.copy()

    def head_transform(self, q):
        r, p = self.camera_tf.pelvis_from_optical(q[12:15])
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = r, p
        return T

    def wrist_transform(self, q):
        from scipy.spatial.transform import Rotation
        w = self.cfg["wrist"]
        mount = np.eye(4)
        mount[:3, :3] = Rotation.from_euler("xyz", w["rpy_deg"], degrees=True).as_matrix()
        mount[:3, 3] = w["xyz"]
        return self.transform(q, w["parent"]) @ mount

    def axis(self, normal, tilt):
        n = unit(normal)
        left = unit(np.cross([0, 0, 1], n))
        up = np.cross(n, left)
        a, b = np.radians(tilt)
        return unit(np.cos(b) * (np.cos(a) * n + np.sin(a) * left) + np.sin(b) * up)

    def solve(self, point, q, axis):
        if np.asarray(axis).shape == (3,3):
            return self._solve_pose(vector(point),vector(q,29),np.asarray(axis,float))
        result = self.kin.ik(vector(point), vector(q, 29), unit(axis))
        if not result.success:
            raise ValueError(f"IK failed: {result.reason}")
        return result.q

    def orientation(self,normal,tilt,roll):
        from scipy.spatial.transform import Rotation
        x=self.axis(normal,tilt)
        y=unit(np.cross([0,0,1],x))
        z=np.cross(x,y)
        return np.column_stack([x,y,z]) @ Rotation.from_rotvec([roll,0,0]).as_matrix()

    def _solve_pose(self,point,q,rotation):
        pin=self.pin
        for _ in range(300):
            pin.computeJointJacobians(self.model,self.data,q)
            pin.framesForwardKinematics(self.model,self.data,q)
            T=self.data.oMf[self.kin.ee_frame]
            ep=point-T.translation
            er=pin.log3(rotation @ T.rotation.T)
            if np.linalg.norm(ep)<.0004 and np.linalg.norm(er)<.035:
                return q
            J=pin.getFrameJacobian(self.model,self.data,self.kin.ee_frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:,ARM].copy()
            J[3:]*=.15
            error=np.r_[ep,.15*er]
            step=J.T @ np.linalg.solve(J @ J.T+np.eye(6)*1e-6,error)
            step*=min(1,.1/max(np.max(np.abs(step)),1e-12))
            q[ARM]=np.clip(q[ARM]+step,self.lower,self.upper)
        raise ValueError("full hand orientation IK did not converge")

    def gravity_command(self, q, kp):
        kp = np.asarray(kp, dtype=float)
        out = q.copy()
        g = self.pin.computeGeneralizedGravity(self.model, self.data, q)
        if np.any(kp[ARM] <= 0):
            raise ValueError("arm PD gains are required")
        out[ARM] += self.cfg["gravity_scale"] * g[ARM] / kp[ARM]
        # Feed-forward must not consume the joint safety margin. Saturation is
        # handled by measured tracking/alignment checks, not by exceeding limits.
        out[ARM]=np.clip(out[ARM],self.lower+.003,self.upper-.003)
        return out

    def gravity_torque(self,q):
        tau=self.pin.computeGeneralizedGravity(self.model,self.data,q)
        out=np.zeros(29)
        out[ARM]=np.clip(self.cfg["gravity_scale"]*tau[ARM],
                         -self.model.effortLimit[ARM],self.model.effortLimit[ARM])
        return out

    def pressing_torque(self,q,normal,force):
        pin=self.pin
        pin.computeJointJacobians(self.model,self.data,q)
        pin.framesForwardKinematics(self.model,self.data,q)
        J=pin.getFrameJacobian(self.model,self.data,self.kin.ee_frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        out=np.zeros(29)
        out[ARM]=J[:3,ARM].T @ (unit(normal)*force)
        return out
