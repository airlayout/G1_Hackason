"""パッチ 0102（11-10・頭のカメラの深度を RGB と同じ歩に 16 ビットの PNG で書く）の試験。
Isaac Sim は起こさない（state_recorder.py から _write_rgb を取り出し、偽の環境で動かす）。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- depth_png=True なら depth/ に mm の uint16 で書く。無限・非数は 0。形 [N, H, W, 1] でも [N, H, W] でもよい
- depth_png=False（既定）なら今までどおり RGB だけ
"""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from gr00t.rl.utils.state_recorder import StateRecorder


def _env(rgb, depth):
    out = {"rgb": rgb, "depth": depth}
    cam = SimpleNamespace(data=SimpleNamespace(output=out))
    sim = SimpleNamespace(ego_camera=cam, scene=SimpleNamespace(sensors={"ego_camera": cam}))
    return SimpleNamespace(simulator=sim, common_step_counter=42)


def _rec(tmp_path: Path, depth_png: bool):
    rec = StateRecorder.__new__(StateRecorder)  # __init__ は扉の環境の値を読むので通さない
    rec.dir, rec.num, rec.depth_png = str(tmp_path), 2, depth_png
    return rec


@pytest.mark.parametrize("shape4", [True, False])
def test_depth_png_in_mm_with_invalid_zero(tmp_path, shape4):
    cv2 = pytest.importorskip("cv2")
    rgb = torch.zeros(3, 4, 5, 4, dtype=torch.uint8)
    d = torch.full((3, 4, 5), 1.2345)
    d[0, 0, 0] = float("inf")
    d[1, 1, 1] = float("nan")
    _rec(tmp_path, True)._write_rgb(_env(rgb, d[..., None] if shape4 else d))
    a = cv2.imread(str(tmp_path / "depth" / "env00_step000042.png"), cv2.IMREAD_UNCHANGED)
    b = cv2.imread(str(tmp_path / "depth" / "env01_step000042.png"), cv2.IMREAD_UNCHANGED)
    assert a.dtype == np.uint16 and a.shape == (4, 5)
    assert a[0, 0] == 0 and b[1, 1] == 0  # 無限・非数は 0
    assert a[2, 3] == 1234 or a[2, 3] == 1235  # mm に丸める
    assert not (tmp_path / "depth" / "env02_step000042.png").exists()  # num 台だけ
    assert (tmp_path / "rgb" / "env00_step000042.png").exists()


def test_no_depth_by_default(tmp_path):
    pytest.importorskip("cv2")
    _rec(tmp_path, False)._write_rgb(_env(torch.zeros(2, 4, 5, 4, dtype=torch.uint8), torch.ones(2, 4, 5)))
    assert (tmp_path / "rgb" / "env00_step000042.png").exists()
    assert not (tmp_path / "depth").exists()
