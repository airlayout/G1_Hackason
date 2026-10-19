"""Additive MuJoCo fixtures around the pinned Yada environment.

Only this test adapter sees simulator state. No ground truth reaches the agent.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from types import SimpleNamespace
import numpy as np

from .types import CameraFrame


def add_robot_geometry(spec, cfg):
    import mujoco
    from scipy.spatial.transform import Rotation
    w = cfg["wrist"]
    body = spec.body(w["parent"])
    camera = body.add_camera(name="press_wrist_camera", pos=w["xyz"])
    R = Rotation.from_euler("xyz", w["rpy_deg"], degrees=True).as_matrix()
    camera.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    camera.alt.xyaxes = [*R[:, 0], *(-R[:, 1])]
    camera.fovy = w["fovy_deg"]
    body.add_geom(name="press_camera_mount", type=mujoco.mjtGeom.mjGEOM_BOX,
                  pos=w["xyz"], size=np.asarray(w["mount_size_m"])/2,
                  mass=0.03, rgba=[0.1, 0.15, 0.2, 1])
    # Include full plastic hands in PHYSICAL tests, not only the invisible tip balls.
    for geom in list(spec.geoms):
        if geom.meshname and "rubber_hand" in geom.meshname:
            geom.contype = geom.conaffinity = 1


def add_bottle(spec):
    import mujoco
    pelvis_z = float(spec.body("pelvis").pos[2])
    # Fixed upright fixture. Its label is opaque so RGB-D can see the pressed side.
    body = spec.worldbody.add_body(name="press_fixed_bottle", pos=[0.40, -0.18, pelvis_z-0.035])
    body.add_geom(name="press_bottle_body", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                  size=[.035, .105, 0], rgba=[.65,.8,.9,1], mass=.2)
    body.add_geom(name="press_bottle_label", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                  pos=[0,0,-.005], size=[.0355,.04,0], rgba=[.05,.2,.65,1], mass=.01)
    body.add_geom(name="press_bottle_shoulder", type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                  pos=[0,0,.095], size=[.034,.034,.035], rgba=[.65,.8,.9,1], mass=.01)
    body.add_geom(name="press_bottle_neck", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                  pos=[0,0,.127], size=[.014,.02,0], rgba=[.65,.8,.9,1], mass=.01)
    body.add_geom(name="press_bottle_cap", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                  pos=[0,0,.15], size=[.017,.008,0], rgba=[.05,.2,.65,1], mass=.01)
    spec.worldbody.add_geom(name="press_bottle_fixture", type=mujoco.mjtGeom.mjGEOM_BOX,
                            pos=[.40,-.18,pelvis_z-.153], size=[.10,.12,.013], rgba=[.4,.3,.2,1])


@contextmanager
def extended_builder(cfg):
    """Temporary construction hook; never modify the cached upstream files."""
    from common.j1gen_bridge import j1gen
    module = j1gen("robot_model")
    original = module.build_spec
    def build(*args, **kwargs):
        spec = original(*args, **kwargs)
        add_robot_geometry(spec, cfg)
        if cfg["mode"] == "bottle":
            add_bottle(spec)
        return spec
    module.build_spec = build
    try:
        yield
    finally:
        module.build_spec = original


def make_mujoco_robot(scene_cfg, contest_cfg, cfg, view=False, realism=None):
    import mujoco
    from contest.robots.mujoco_robot import MujocoRobot
    with extended_builder(cfg):
        base = MujocoRobot(scene_cfg, contest_cfg, viewer=view, realism=realism)
    w = cfg["wrist"]
    renderer = mujoco.Renderer(base.model, height=w["height"], width=w["width"])
    depth_renderer = mujoco.Renderer(base.model, height=w["height"], width=w["width"])
    depth_renderer.enable_depth_rendering()
    fy = w["height"]/2/np.tan(np.radians(w["fovy_deg"])/2)
    K = np.array([[fy,0,w["width"]/2],[0,fy,w["height"]/2],[0,0,1.]])
    cid = base.model.camera("press_wrist_camera").id

    class ExtendedRobot:
        name, base_enabled = "mujoco", False
        supports_torque_feedforward=True
        last_head = None
        last_wrist = None
        last_capture = -1.0
        def __getattr__(self, name):
            return getattr(base, name)
        def command_with_feedforward(self,q_upper,base_cmd,tau_ff):
            base.command(q_upper,base_cmd)
            # PD offset is exactly equivalent to adding tau_ff to the actuator.
            # It stays in the plant adapter; commanded nominal geometry is unchanged.
            if base.realism is None or base.realism.command_latency_s<=0:
                base.q_des[22:]+=tau_ff[22:]/base._kp_eff[22:]
            else:
                stamp,q=base._cmd_queue[-1]
                q=q.copy()
                q[22:]+=tau_ff[22:]/base._kp_eff[22:]
                base._cmd_queue[-1]=(stamp,q)
        def observe(self, t):
            due = self.last_head is None or t-self.last_capture >= .099
            if due:
                obs = base.observe(t)
                self.last_head = obs
                self.last_capture = t
            else:
                obs = self.last_head
            data = dict(vars(obs))
            data["q"] = base.data.qpos[base._qadr].copy()
            data["dq"] = base.data.qvel[base._vadr].copy()
            data["t"] = t
            if cfg["use_wrist"] and due:
                renderer.update_scene(base.data, camera="press_wrist_camera")
                depth_renderer.update_scene(base.data, camera="press_wrist_camera")
                # Exposure-state transform derived from simulated camera calibration, not target truth.
                R = base.data.cam_xmat[cid].reshape(3,3) @ np.diag([1,-1,-1])
                T = np.eye(4)
                pelvis = base.model.body("pelvis").id
                Rp = base.data.xmat[pelvis].reshape(3,3)
                T[:3,:3], T[:3,3] = Rp.T @ R, Rp.T @ (base.data.cam_xpos[cid]-base.data.xpos[pelvis])
                self.last_wrist = CameraFrame(renderer.render().copy(), depth_renderer.render().copy(),
                                             K.copy(), t, data["q"].copy(), T, "wrist", int(round(t*1e6)))
            data["wrist"] = self.last_wrist
            return SimpleNamespace(**data)
        def advance(self, dt):
            self.bottle_contact = False
            self.unintended_force = 0.0
            self.bottle_force = 0.0
            info = base.advance(dt)
            self.unintended_force=max(self.unintended_force,info.max_contact_force or 0.0)
            return info
        def record_contacts(self):
            for index, contact in enumerate(base.data.contact):
                names = [base.model.geom(int(g)).name for g in (contact.geom1, contact.geom2)]
                if any(n.startswith("press_bottle_") and n != "press_bottle_fixture" for n in names):
                    force = np.zeros(6)
                    mujoco.mj_contactForce(base.model, base.data, index, force)
                    if "right_fingertip_collision" in names:
                        self.bottle_contact = True
                        self.bottle_force = max(self.bottle_force, float(abs(force[0])))
                    elif any("right_" in n for n in names):
                        self.unintended_force = max(self.unintended_force, float(abs(force[0])))
                elif any(n.startswith("button_") and n.endswith("_cap") for n in names):
                    if "right_fingertip_collision" not in names and any("right_" in n or "left_" in n for n in names):
                        force=np.zeros(6)
                        mujoco.mj_contactForce(base.model,base.data,index,force)
                        self.unintended_force=max(self.unintended_force,float(abs(force[0])))
        def close(self):
            renderer.close()
            depth_renderer.close()
            base.close()
    robot=ExtendedRobot()
    update=base.hall.update
    def update_with_contact_recording():
        update()
        robot.record_contacts()
    base.hall.update=update_with_contact_recording
    return robot
