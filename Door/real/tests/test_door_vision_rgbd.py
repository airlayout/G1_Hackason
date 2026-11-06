"""11-06（カラー＋深度）: 実機の深度をシミュレータと同じ画角・形に取り出す（door_vision_encoder.depth_from_intrinsics）の試験。

シミュレータの深度はカラーと同じカメラ（焦点 1.88・開口 2.6035×1.4621＝69°×42°）で描き、54×96 に間引いたもの。
実機の D435i の深度（87°×58°・848×480 なら fx ≈ 425）から、その画角の中央を最近傍で取る。
"""
from __future__ import annotations

import math

import numpy as np
import pytest

import door_vision_encoder as dve

# D435 の深度 848×480 のおよその内部の値（横の画角 87°）
W, H = 848, 480
FX = (W / 2) / math.tan(math.radians(87 / 2))
FY = (H / 2) / math.tan(math.radians(58 / 2))
CX, CY = W / 2, H / 2


def test_constant_plane_stays_constant_and_shape():
    d = np.full((H, W), 1.234, dtype=np.float32)
    out = dve.depth_from_intrinsics(d, FX, FY, CX, CY)
    assert out.shape == (dve.DEPTH_H, dve.DEPTH_W) == (54, 96)
    np.testing.assert_allclose(out, 1.234, rtol=1e-6)


def test_crop_takes_the_sim_field_of_view():
    """シミュレータの画角の端（tan 0.692）は、深度の画面では中心から fx × 0.692 画素。その外の印は写らない"""
    d = np.ones((H, W), dtype=np.float32)
    edge = FX * dve.SIM_TAN_HALF[0]
    d[:, : int(CX - edge) - 3] = 2.5  # シミュレータの画角の外（左）
    d[:, int(CX + edge) + 3 :] = 2.5  # 外（右）
    out = dve.depth_from_intrinsics(d, FX, FY, CX, CY)
    np.testing.assert_allclose(out, 1.0)
    # 画角の 87° は 69° より広いので、取り出す所はすべて画面の中
    assert CX - edge > 0 and CX + edge < W


def test_left_right_and_up_down_kept():
    """深度の画面の左半分が近ければ、出力も左半分が近い（左右・上下を入れ替えない）"""
    d = np.full((H, W), 2.0, dtype=np.float32)
    d[:, : W // 2] = 0.5
    d[: H // 2, :] = np.where(np.arange(W) < W // 2, 0.5, 1.0)
    out = dve.depth_from_intrinsics(d, FX, FY, CX, CY)
    assert (out[:, : dve.DEPTH_W // 2] == 0.5).all()
    assert (out[: dve.DEPTH_H // 2, dve.DEPTH_W // 2 :] == 1.0).all()
    assert (out[dve.DEPTH_H // 2 :, dve.DEPTH_W // 2 :] == 2.0).all()


def test_invalid_far_and_outside():
    d = np.full((H, W), 1.0, dtype=np.float32)
    d[CY.__int__(), :] = 0.0
    d[:10, :] = np.nan
    d[-5:, :] = 9.0
    out = dve.depth_from_intrinsics(d, FX, FY, CX, CY)
    assert np.isfinite(out).all()
    assert out.max() <= dve.DEPTH_MAX
    # 画角の狭い深度（fx が大きい）だと、出力の端は画面の外＝0
    narrow = dve.depth_from_intrinsics(np.ones((H, W), dtype=np.float32), FX * 2.0, FY * 2.0, CX, CY)
    assert (narrow[:, 0] == 0).all() and (narrow[:, -1] == 0).all() and (narrow[0] == 0).all()
    assert narrow[dve.DEPTH_H // 2, dve.DEPTH_W // 2] == 1.0


def test_depth_input_matches_sim_layout():
    d = np.zeros((dve.DEPTH_H, dve.DEPTH_W), dtype=np.float32)
    d[0, 0] = 1.5
    x = dve.depth_input(d)
    assert x.shape == (2, 54, 96) and x.dtype == np.float32
    assert x[0, 0, 0] == pytest.approx(0.5) and x[1, 0, 0] == 1.0 and x[1, 0, 1] == 0.0
