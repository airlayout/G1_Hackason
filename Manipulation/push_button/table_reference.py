"""既知の台面と直交する二辺を使い、RGB-Dから床基準の機体位置を求める。

対象物の正解座標・シミュレーター状態は使わない。校正済みカメラと実測した
台の高さ・平面範囲が必要。台面や二辺を確認できない計測は棄却する。
"""

import math
from time import perf_counter

import cv2
import numpy as np


def _edge(points, axis, side='min'):
    if len(points) < 30:
        raise ValueError('台の二辺を確認できません')
    # 回転は撮影時のIMUとカメラ校正から与える。腕の遮蔽が作る内側の輪郭で
    # 台の向きを推定し直すと、位置まで数cmずれるため、辺の法線は固定する。
    normal = np.eye(2)[axis]
    edge_location = float(np.quantile(points[:,axis], .25 if side == 'min' else .75))
    points = points[np.abs(points[:,axis]-edge_location)<.003]
    if len(points)<30:
        raise ValueError('台の辺の有効点が不足しています')
    center=np.median(points,axis=0)
    tangent = np.array([-normal[1],normal[0]])
    span = float(np.ptp(points@tangent))
    if span < .08 or float(np.sqrt(np.mean(((points-center)@normal)**2))) > .004:
        raise ValueError('台の辺の長さまたは直線精度が不足しています')
    return points, normal, span


def estimate_table_transform(depth_m, intrinsics, nominal_transform, settings, *, diagnostics=None):
    details = diagnostics if diagnostics is not None else {}
    details.update(edges={}, selected_edges=[])
    depth = np.asarray(depth_m)
    # The plane/edges are centimetre-scale features. Sample at 2px rather than
    # building and transforming 307,200 points at every update on a CPU.
    stride = 2 if min(depth.shape) >= 400 else 1
    depth = depth[::stride, ::stride]
    vv, uu = np.mgrid[0:depth.shape[0],0:depth.shape[1]]
    z = depth.ravel()
    optical = np.column_stack(((uu.ravel()*stride-intrinsics.cx)*z/intrinsics.fx,
                               (vv.ravel()*stride-intrinsics.cy)*z/intrinsics.fy,z))
    points = optical@nominal_transform[:3,:3].T+nominal_transform[:3,3]
    x0,x1,y0,y1 = settings.table_bounds
    mask = ((z > .1)&np.all(np.isfinite(points),axis=1)
        &(np.abs(points[:,2]-settings.table_height)<.025)
        &(points[:,0]>x0-.035)&(points[:,0]<x1+.035)
        &(points[:,1]>y0-.035)&(points[:,1]<y1+.035))
    plane = points[mask][::4]
    details['plane_point_count'] = len(plane)
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
    # Interior holes include arm occlusions and depth dropouts, not table edges.
    boundary = np.zeros_like(closed)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(boundary, contours, -1, 1, 1)
    boundary[:3,:]=boundary[-3:,:]=boundary[:,:3]=boundary[:,-3:]=0
    edge_mask = boundary.ravel().astype(bool)&in_plane
    valid_edges = [[], []]
    for name, axis, bound, side in (('x_min', 0, x0, 'min'), ('x_max', 0, x1, 'max'),
                                     ('y_min', 1, y0, 'min'), ('y_max', 1, y1, 'max')):
        indices = np.flatnonzero(edge_mask & (np.abs(points[:, axis]-bound) < .035))
        row = details['edges'][name] = {'point_count': 0, 'candidate_count': len(indices),
                                       'status': 'rejected', 'pixels_uv': []}
        # Require valid depth below the tabletop just outside the candidate edge.
        # This rejects clipped ROI boundaries and foreground occluder contours.
        outside = points[indices].copy()
        outside[:, axis] += -.015 if side == 'min' else .015
        camera = (outside-nominal_transform[:3, 3]) @ nominal_transform[:3, :3]
        projected = np.zeros((len(indices), 2))
        positive = camera[:, 2] > .1
        projected[positive] = (camera[positive, :2]/camera[positive, 2, None]
            * [intrinsics.fx, intrinsics.fy]+[intrinsics.cx, intrinsics.cy])/stride
        uv = np.rint(projected).astype(int)
        inside = positive & (uv[:, 0] >= 3) & (uv[:, 0] < depth.shape[1]-3) \
            & (uv[:, 1] >= 3) & (uv[:, 1] < depth.shape[0]-3)
        indices, uv, projected = indices[inside], uv[inside], projected[inside]
        neighbours = uv[:, 1]*depth.shape[1]+uv[:, 0]
        supported = (z[neighbours] > .1) & np.isfinite(points[neighbours, 2]) \
            & (points[neighbours, 2] < settings.table_height-.025)
        indices, projected = indices[supported], projected[supported]
        sample = indices[np.linspace(0, len(indices)-1, min(80, len(indices)), dtype=int)]
        row['pixels_uv'] = np.column_stack((uu.ravel()[sample]*stride,
                                            vv.ravel()[sample]*stride)).tolist()
        # A silhouette lies between the last tabletop pixel and the next pixel.
        # Use half a sample outward and intersect the fitted plane, avoiding the
        # opposite signed pixel-centre bias on min/max edges at 2px sampling.
        pixels = np.column_stack((uu.ravel()[indices], vv.ravel()[indices]))
        direction = projected-pixels
        direction /= np.maximum(np.max(np.abs(direction), axis=1, keepdims=True), 1e-9)
        pixels = (pixels+.5*direction)*stride
        rays = np.column_stack(((pixels[:, 0]-intrinsics.cx)/intrinsics.fx,
                                (pixels[:, 1]-intrinsics.cy)/intrinsics.fy, np.ones(len(indices))))
        rays = rays@nominal_transform[:3, :3].T
        distance = ((center-nominal_transform[:3, 3])@normal)/(rays@normal)
        edge_points = nominal_transform[:3, 3]+distance[:, None]*rays
        try:
            fitted, normal_xy, span = _edge(edge_points[:, :2], axis, side)
        except ValueError as exc:
            row['reason'] = str(exc)
            continue
        residual = float(np.sqrt(np.mean(((fitted-np.median(fitted, axis=0))@normal_xy)**2)))
        offset = bound-float(np.median(fitted[:, axis]))
        row.update(status='valid', point_count=len(fitted), span_m=span,
                   residual_m=residual, offset_m=offset)
        valid_edges[axis].append(name)
    offsets = []
    for names in valid_edges:
        if not names:
            raise ValueError('台の二辺を確認できません')
        rows = details['edges']
        if np.ptp([rows[name]['offset_m'] for name in names]) > .004:
            raise ValueError('台の対辺から求めた補正量が一致しません')
        name = min(names, key=lambda n: (rows[n]['residual_m'], -rows[n]['span_m'],
                                         -rows[n]['point_count'], n))
        details['selected_edges'].append(name)
        offsets.append(rows[name]['offset_m'])
    transform = np.eye(4)
    transform[:3,3] = [*offsets, settings.table_height-float(np.median(plane[:,2]))]
    if np.linalg.norm(transform[:3,3]) > .08:
        raise ValueError('台基準の補正量が校正範囲外です')
    return transform


class TableReference:
    def __init__(self, settings, pelvis_height):
        self.settings,self.pelvis_height = settings,pelvis_height
        self.stamp = None
        self.offset = self.velocity = None
        self.measurements = []
        self.diagnostics = []

    def update(self, depth, intrinsics, nominal_transform, nominal_base, captured_at):
        started = perf_counter()
        details = {'captured_at_s': captured_at, 'status': 'rejected'}
        self.diagnostics.append(details)
        try:
            return self._update(depth, intrinsics, nominal_transform, nominal_base, captured_at, details)
        except ValueError as exc:
            details['reason'] = str(exc)
            raise
        finally:
            details['estimate_wall_s'] = perf_counter()-started
            if details['status'] == 'accepted':
                self.measurements[-1]['estimate_wall_s'] = details['estimate_wall_s']

    def _update(self, depth, intrinsics, nominal_transform, nominal_base, captured_at, details):
        transform = estimate_table_transform(depth,intrinsics,nominal_transform,self.settings,
                                             diagnostics=details)
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
        details.update(status='accepted', base_offset_m=offset.tolist())
        self.measurements.append({'captured_at_s': captured_at, 'base_offset_m': offset.tolist(),
                                   'base_velocity_m_s': self.velocity.tolist(),
                                   'selected_edges': details['selected_edges']})
        return transform

    def predict(self, now, *, strict=True):
        if self.stamp is None or now-self.stamp > .5:
            if strict:
                raise RuntimeError('新しい台基準の機体位置を取得できません')
            return np.zeros(3) if self.offset is None else self.offset.copy()
        dt = min(.15,max(0.,now-self.stamp))
        return self.offset+np.clip(self.velocity*dt,[-.003,-.003,-.0005],[.003,.003,.0005])
