"""台のRGB-D幾何だけで機体位置を復元し、欠損や不整合を棄却する。"""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bottle import BottleSettings
from table_reference import TableReference,estimate_table_transform
from vision import CameraIntrinsics


def table_depth(offset=(.004,-.003,.001), *, missing=False, reflect_x=False, reflect_y=False):
    settings = BottleSettings(.75)
    eye,target = np.array([-.12,-.62,1.19]),np.array([.33,-.25,.85])
    if reflect_x:
        eye[0], target[0] = 1.05-eye[0], 1.05-target[0]
    if reflect_y:
        eye[1], target[1] = -.58-eye[1], -.58-target[1]
    forward = target-eye;forward/=np.linalg.norm(forward)
    right = np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right)
    down = np.cross(forward,right)
    nominal = np.eye(4);nominal[:3,:3]=np.column_stack([right,down,forward]);nominal[:3,3]=eye
    intr = CameraIntrinsics(539.,539.,319.5,239.5)
    vv,uu = np.mgrid[:480,:640]
    rays = np.stack([(uu-intr.cx)/intr.fx,(vv-intr.cy)/intr.fy,np.ones_like(uu)],axis=-1)
    rays = rays@nominal[:3,:3].T
    origin = eye+offset
    z = (settings.table_height-origin[2])/rays[:,:,2]
    points = origin+rays*z[:,:,None]
    x0,x1,y0,y1=settings.table_bounds
    mask=(z>.1)&(points[:,:,0]>=x0)&(points[:,:,0]<=x1)&(points[:,:,1]>=y0)&(points[:,:,1]<=y1)
    depth=np.where(mask,z,2.).astype(np.float32)
    rng=np.random.default_rng(917)
    depth[mask]+=rng.normal(0,.001,mask.sum())
    depth=np.round(depth*1000)/1000
    depth[rng.random(depth.shape)<.01]=0.
    if missing:depth[:]=0.
    return depth,intr,nominal,settings


def test_table_recovers_translation_from_noisy_quantized_depth():
    depth,intr,nominal,settings=table_depth()
    transform=estimate_table_transform(depth,intr,nominal,settings)
    assert transform[:3,3] == pytest.approx((.004,-.003,.001),abs=.002)
    assert transform[:3,:3] == pytest.approx(np.eye(3),abs=.005)


def test_missing_table_is_rejected_and_stale_reference_cannot_drive_push():
    depth,intr,nominal,settings=table_depth(missing=True)
    with pytest.raises(ValueError,match='深度が不足'):
        estimate_table_transform(depth,intr,nominal,settings)
    reference=TableReference(settings,.79)
    with pytest.raises(RuntimeError,match='新しい台基準'):
        reference.predict(0.)
    depth,intr,nominal,settings=table_depth()
    base=np.eye(4);base[2,3]=.79
    reference.update(depth,intr,nominal,base,1.)
    assert np.all(np.isfinite(reference.predict(1.1)))
    with pytest.raises(RuntimeError,match='新しい台基準'):
        reference.predict(1.6)
    # Recovery may inspect the last known pose without treating it as a fresh observation.
    assert reference.predict(1.6,strict=False) == pytest.approx(reference.offset)


def test_table_timestamps_must_increase():
    depth,intr,nominal,settings=table_depth()
    reference=TableReference(settings,.79)
    base=np.eye(4);base[2,3]=.79
    reference.update(depth,intr,nominal,base,1.)
    with pytest.raises(ValueError,match='撮影時刻が重複'):
        reference.update(depth,intr,nominal,base,1.)
    assert reference.diagnostics[-1]['status'] == 'rejected'
    assert '重複' in reference.diagnostics[-1]['reason']


@pytest.mark.parametrize('x_side,y_side', [('min', 'min'), ('min', 'max'), ('max', 'min'), ('max', 'max')])
def test_all_orthogonal_edge_pairs_recover_signed_translation(monkeypatch, x_side, y_side):
    import table_reference
    original = table_reference._edge

    def visible_pair(points, axis, side='min'):
        if side != (x_side, y_side)[axis]:
            raise ValueError('test: opposite edge outside view')
        return original(points, axis, side)

    monkeypatch.setattr(table_reference, '_edge', visible_pair)
    details = {}
    transform = estimate_table_transform(*table_depth(), diagnostics=details)
    assert transform[:3, 3] == pytest.approx((.004, -.003, .001), abs=.002)
    assert details['selected_edges'] == [f'x_{x_side}', f'y_{y_side}']
    for name in details['selected_edges']:
        assert details['edges'][name]['point_count'] >= 30
        assert details['edges'][name]['span_m'] >= .08
        assert details['edges'][name]['pixels_uv']


@pytest.mark.parametrize('reflect_x,reflect_y', [(False, True), (True, False), (True, True)])
def test_table_seen_from_opposite_directions(reflect_x, reflect_y):
    transform = estimate_table_transform(*table_depth(reflect_x=reflect_x, reflect_y=reflect_y))
    assert transform[:3, 3] == pytest.approx((.004, -.003, .001), abs=.002)


def test_one_axis_only_never_becomes_a_valid_table_pose(monkeypatch):
    import table_reference
    original = table_reference._edge
    def one_axis(points, axis, side='min'):
        if axis == 1:
            raise ValueError('occluded')
        return original(points, axis, side)
    monkeypatch.setattr(table_reference, '_edge', one_axis)
    reference = TableReference(BottleSettings(.75), .79)
    depth, intr, nominal, _ = table_depth()
    with pytest.raises(ValueError, match='二辺'):
        reference.update(depth, intr, nominal, np.eye(4), 1.)
    assert reference.stamp is None and not reference.measurements
    assert reference.diagnostics[-1]['edges']['y_max']['reason'] == 'occluded'


def test_inconsistent_opposite_edges_are_rejected(monkeypatch):
    import table_reference
    original = table_reference._edge
    def disagree(points, axis, side='min'):
        fitted, normal, span = original(points, axis, side)
        if axis == 0 and side == 'max':
            fitted = fitted.copy()
            fitted[:, 0] += .01
        return fitted, normal, span
    monkeypatch.setattr(table_reference, '_edge', disagree)
    with pytest.raises(ValueError, match='対辺'):
        estimate_table_transform(*table_depth())


def test_interior_occlusion_is_not_used_as_a_table_edge():
    depth, intr, nominal, settings = table_depth()
    depth[190:265, 280:340] = .2
    transform = estimate_table_transform(depth, intr, nominal, settings)
    assert transform[:3, 3] == pytest.approx((.004, -.003, .001), abs=.002)


def test_table_bounds_mask_does_not_invent_edges_on_an_unbounded_plane():
    depth = np.full((480, 640), .75)
    nominal = np.diag([1., -1., -1., 1.])
    nominal[:3, 3] = [.525, -.29, 1.5]
    with pytest.raises(ValueError, match='二辺'):
        estimate_table_transform(depth, CameraIntrinsics(539., 539., 319.5, 239.5),
                                 nominal, BottleSettings(.75))
