"""既知の台面と直交する二辺を使い、RGB-Dから床基準の機体位置を求める。

対象物の正解座標・シミュレーター状態は使わない。校正済みカメラと実測した
台の高さ・平面範囲が必要。台面や二辺を確認できない計測は棄却する。
"""

import math

import cv2
import numpy as np


def _edge(points, axis):
    if len(points) < 30:
        raise ValueError('台の二辺を確認できません')
    # 回転は撮影時のIMUとカメラ校正から与える。腕の遮蔽が作る内側の輪郭で
    # 台の向きを推定し直すと、位置まで数cmずれるため、辺の法線は固定する。
    normal = np.eye(2)[axis]
    edge_location = float(np.quantile(points[:,axis],.25))
    points = points[np.abs(points[:,axis]-edge_location)<.003]
    if len(points)<30:
        raise ValueError('台の辺の有効点が不足しています')
    center=np.median(points,axis=0)
    tangent = np.array([-normal[1],normal[0]])
    span = float(np.ptp(points@tangent))
    if span < .08 or float(np.sqrt(np.mean(((points-center)@normal)**2))) > .004:
        raise ValueError('台の辺の長さまたは直線精度が不足しています')
    return points, normal, span


def estimate_table_transform(depth_m, intrinsics, nominal_transform, settings):
    depth = np.asarray(depth_m)
    vv, uu = np.mgrid[0:depth.shape[0],0:depth.shape[1]]
    z = depth.ravel()
    optical = np.column_stack(((uu.ravel()-intrinsics.cx)*z/intrinsics.fx,
                               (vv.ravel()-intrinsics.cy)*z/intrinsics.fy,z))
    points = optical@nominal_transform[:3,:3].T+nominal_transform[:3,3]
    x0,x1,y0,y1 = settings.table_bounds
    mask = ((z > .1)&np.all(np.isfinite(points),axis=1)
        &(np.abs(points[:,2]-settings.table_height)<.025)
        &(points[:,0]>x0-.035)&(points[:,0]<x1+.035)
        &(points[:,1]>y0-.035)&(points[:,1]<y1+.035))
    plane = points[mask][::4]
    if len(plane) < 500:
        raise ValueError('台面の深度が不足しています')
    for _ in range(3):
        center = np.median(plane,axis=0)
        _,_,vh = np.linalg.svd(plane-center,full_matrices=False)
        normal = vh[-1]
        if normal[2] < 0:
            normal = -normal
        residual = np.abs((plane-center)@normal)
        plane = plane[residual < max(.002,float(np.quantile(residual,.9)))]
    if normal[2] < math.cos(math.radians(5)):
        raise ValueError('台面の傾きが校正範囲外です')
    in_plane = mask&(np.abs((points-center)@normal)<.004)
    image_mask = in_plane.reshape(depth.shape).astype(np.uint8)
    closed = cv2.morphologyEx(image_mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    boundary = closed-cv2.erode(closed,np.ones((3,3),np.uint8))
    boundary[:3,:]=boundary[-3:,:]=boundary[:,:3]=boundary[:,-3:]=0
    edge_mask = boundary.ravel().astype(bool)&in_plane
    front = points[edge_mask&(np.abs(points[:,0]-x0)<.035),:2]
    near = points[edge_mask&(np.abs(points[:,1]-y0)<.035),:2]
    front,nx,sx = _edge(front,0)
    near,ny,sy = _edge(near,1)
    front_x = front[:,0]
    near_y = near[:,1]
    transform = np.eye(4)
    transform[:3,3] = [x0-float(np.median(front_x)),y0-float(np.median(near_y)),
                       settings.table_height-float(np.median(plane[:,2]))]
    if np.linalg.norm(transform[:3,3]) > .08:
        raise ValueError('台基準の補正量が校正範囲外です')
    return transform


class TableReference:
    def __init__(self, settings, pelvis_height):
        self.settings,self.pelvis_height = settings,pelvis_height
        self.stamp = None
        self.offset = self.velocity = None
        self.measurements = []

    def update(self, depth, intrinsics, nominal_transform, nominal_base, captured_at):
        transform = estimate_table_transform(depth,intrinsics,nominal_transform,self.settings)
        offset = (transform@nominal_base)[:3,3]-[0.,0.,self.pelvis_height]
        if self.stamp is not None:
            dt = captured_at-self.stamp
            if dt <= 0:
                raise ValueError('台計測の撮影時刻が重複しています')
            if np.linalg.norm(offset-self.offset) > .025:
                raise ValueError('台基準の機体位置が不連続です')
            measured_velocity = (offset-self.offset)/dt
            self.velocity += np.clip(measured_velocity-self.velocity,-.3*dt,.3*dt)
            self.velocity = np.clip(self.velocity,-.05,.05)
        else:
            self.velocity = np.zeros(3)
        self.offset,self.stamp = offset,captured_at
        self.measurements.append({'captured_at_s': captured_at, 'base_offset_m': offset.tolist(),
                                   'base_velocity_m_s': self.velocity.tolist()})
        return transform

    def predict(self, now, *, strict=True):
        if self.stamp is None or now-self.stamp > .5:
            if strict:
                raise RuntimeError('新しい台基準の機体位置を取得できません')
            return np.zeros(3) if self.offset is None else self.offset.copy()
        dt = min(.15,max(0.,now-self.stamp))
        return self.offset+np.clip(self.velocity*dt,[-.003,-.003,-.0005],[.003,.003,.0005])
