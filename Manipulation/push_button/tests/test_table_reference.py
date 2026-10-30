"""台のRGB-D幾何だけで機体位置を復元し、欠損や不整合を棄却する。"""

from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bottle import BottleSettings
from table_reference import TableReference,estimate_table_transform
from vision import CameraIntrinsics


def table_depth(offset=(.004,-.003,.001), *, missing=False):
    settings = BottleSettings(.75)
    eye,target = np.array([-.12,-.62,1.19]),np.array([.33,-.25,.85])
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
