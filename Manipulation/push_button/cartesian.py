"""7軸の速度・加速度・関節範囲を同時に満たす手先速度制御。"""

import numpy as np
import mujoco
from scipy.optimize import lsq_linear

from trajectory import ARM_JOINTS,validate_arm_q


class CartesianJointController:
    def __init__(self, initial, settings):
        self.position=np.array(validate_arm_q(np.asarray(initial).tolist()))
        self.rest=self.position.copy()
        self.velocity=np.zeros(7)
        self.settings=settings
        self.diagnostics={}

    def next_goal(self, kin, target, normal, dt):
        if not np.isfinite(dt) or not 0 < dt <= .1:
            raise ValueError('手先速度制御の時間刻みが不正です')
        original=kin.data.qpos.copy()
        try:
            q=original.copy();q[kin.qadr]=self.position;kin.update(q)
            position_error=np.asarray(target)-kin.tip_position()
            axis=kin.data.site_xmat[kin.site_id].reshape(3,3)[:,0]
            jpos=np.zeros((3,kin.model.nv));jrot=np.zeros_like(jpos)
            mujoco.mj_jacSite(kin.model,kin.data,jpos,jrot,kin.site_id)
            jaxis=np.cross(jrot[:,kin.dadr].T,axis).T
            cartesian_velocity=12.*position_error
            speed=np.linalg.norm(cartesian_velocity)
            if speed > .06:
                cartesian_velocity *= .06/speed
            matrix=np.vstack([jpos[:,kin.dadr],.03*jaxis,.01*np.eye(7)])
            rhs=np.r_[cartesian_velocity,.03*8.*(np.asarray(normal)-axis),.01*.5*(self.rest-self.position)]
            limits=kin.model.jnt_range[[kin.model.joint(n).id for n in ARM_JOINTS]]
            acceleration=self.settings.joint_acceleration_rad_s2*dt
            distance_lower=np.maximum(0.,self.position-(limits[:,0]+.025))
            distance_upper=np.maximum(0.,limits[:,1]-.025-self.position)
            brake_lower=np.sqrt(acceleration**2+2*self.settings.joint_acceleration_rad_s2*distance_lower)-acceleration
            brake_upper=np.sqrt(acceleration**2+2*self.settings.joint_acceleration_rad_s2*distance_upper)-acceleration
            lower=np.maximum.reduce([np.full(7,-self.settings.joint_speed_rad_s),
                self.velocity-acceleration,-brake_lower])
            upper=np.minimum.reduce([np.full(7,self.settings.joint_speed_rad_s),
                self.velocity+acceleration,brake_upper])
            if np.any(lower >= upper):
                raise ValueError('手先速度制御で関節制約を満たせません')
            solution=lsq_linear(matrix,rhs,bounds=(lower,upper),tol=1e-8,max_iter=50)
            if not solution.success:
                raise ValueError('制約付き手先速度制御が収束しません')
            self.velocity=solution.x
            self.position=np.array(validate_arm_q((self.position+self.velocity*dt).tolist()))
            q[kin.qadr] = self.position
            kin.update(q)
            axis = kin.data.site_xmat[kin.site_id].reshape(3,3)[:,0]
            axis_error = float(np.degrees(np.arccos(np.clip(axis@normal, -1., 1.))))
            if axis_error > 10.:
                raise ValueError('手先速度制御の方向誤差が10度を超えました')
            self.diagnostics={'command_position_error_m':float(np.linalg.norm(position_error)),
                               'axis_error_deg': axis_error,
                               'joint_command_velocity_rad_s':self.velocity.tolist(),
                               'velocity_solver_iterations':solution.nit}
            return self.position.copy()
        finally:
            kin.update(original)
