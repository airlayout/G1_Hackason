"""パッチ 0094（11-06・カラー＋深度の網）の試験。Isaac Sim は起こさない。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_vis && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 深度はカラーと同じカメラで描き、縦横 1/2 に間引いて使う（108×192 → 54×96）。無限・非数は 0（測れない）、3 m で詰める
- 網に渡す深度には D435 の型の揺れと欠けを足す（depth_noise）。測れない所は測れないまま
- DoorRGBDEncoder: カラーの枝（ResNet18）と深度の枝（小さな CNN）をつなぐ。color_off の台はカラーの枝を 0 にする
- load_encoder は重みの encoder_kind で網の種類を選ぶ（前の rgb の重みはそのまま読める）
"""
from __future__ import annotations

import subprocess
import sys

import numpy as np
import torch

from gr00t.rl.envs.door.door_vision_feat import (
    DEPTH_MAX,
    DOOR_TRUTH_DIM,
    AUX_IN_DIM,
    DoorHead,
    DoorRGBDEncoder,
    DoorVisionEncoder,
    depth_clean,
    depth_input,
    depth_noise,
    load_encoder,
)


def _rgbd(n=3, seed=0):
    g = torch.Generator().manual_seed(seed)
    rgb = torch.randint(0, 256, (n, 108, 192, 3), dtype=torch.uint8, generator=g)
    # 滑らかな面（台ごとの距離＋横の傾き）。画素ごとの乱数だと全部が「縁」になる
    base = 0.4 + torch.rand(n, 1, 1, generator=g) * 1.5
    depth = base + torch.linspace(0.0, 0.3, 96)[None, None, :] + torch.zeros(n, 54, 1)
    return rgb, depth


def test_depth_clean_strides_and_marks_invalid():
    d = torch.full((2, 108, 192, ), 1.0)
    d[0, 0, 0] = float("inf")
    d[0, 0, 2] = float("nan")
    d[1, 2, 2] = 10.0
    out = depth_clean(d)
    assert out.shape == (2, 54, 96)
    assert out[0, 0, 0] == 0.0 and out[0, 0, 1] == 0.0
    assert out[1, 1, 1] == DEPTH_MAX


def test_depth_noise_keeps_invalid_and_range():
    _, d = _rgbd(8)
    d[:, :5, :] = 0.0
    d[:, 10, 10] = 0.1  # 最短距離より近い
    g = torch.Generator().manual_seed(1)
    out = depth_noise(d, g)
    assert out.shape == d.shape
    assert (out[:, :5, :] == 0).all()
    assert (out[:, 10, 10] == 0).all()
    assert out.min() >= 0.0 and out.max() <= DEPTH_MAX
    valid = (d > 0.3) & (out > 0)
    rel = ((out - d).abs()[valid] / d[valid]).median()
    assert rel < 0.03  # 揺れは数 % 以内
    assert (out == 0).float().mean() < 0.5  # 欠けすぎない
    out2 = depth_noise(d, torch.Generator().manual_seed(1))
    assert torch.equal(out, out2)  # 種が同じなら同じ


def test_depth_noise_drops_jump_edges_more():
    d = torch.full((64, 54, 96), 1.0)
    d[:, :, 48:] = 2.0  # 縦の縁で 1 m 跳ぶ
    out = depth_noise(d, torch.Generator().manual_seed(2))
    edge = (out[:, :, 48] == 0).float().mean()
    flat = (out[:, :, 20] == 0).float().mean()
    assert edge > flat + 0.4


def test_depth_input_channels():
    d = torch.tensor([[[0.0, 1.5]]])
    x = depth_input(d)
    assert x.shape == (1, 2, 1, 2)
    np.testing.assert_allclose(x[0, :, 0].numpy(), [[0.0, 1.5 / DEPTH_MAX], [0.0, 1.0]])


def test_rgbd_encoder_shapes_and_color_off():
    torch.manual_seed(0)
    enc = DoorRGBDEncoder(128, pretrained=False).eval()
    rgb, depth = _rgbd(3)
    with torch.no_grad():
        f = enc(rgb, depth)
        assert f.shape == (3, 128)
        off = torch.tensor([True, False, True])
        f_off = enc(rgb, depth, off)
        rgb2 = torch.randint(0, 256, rgb.shape, dtype=torch.uint8)
        f_off2 = enc(rgb2, depth, off)
    # カラーを落とした台は、カラーを替えても特徴が変わらない
    torch.testing.assert_close(f_off[off], f_off2[off])
    assert not torch.allclose(f_off[~off], f_off2[~off])
    # 落とさない台は color_off を渡さないのと同じ
    torch.testing.assert_close(f_off[~off], f[~off])


def test_head_works_on_rgbd_features():
    enc = DoorRGBDEncoder(128, pretrained=False).eval()
    head = DoorHead(128)
    rgb, depth = _rgbd(2)
    with torch.no_grad():
        y = head(enc(rgb, depth), torch.zeros(2, AUX_IN_DIM))
    assert y.shape == (2, DOOR_TRUTH_DIM)


def test_load_encoder_picks_kind(tmp_path):
    enc = DoorRGBDEncoder(128, pretrained=False)
    p = tmp_path / "rgbd.pt"
    torch.save({"encoder": enc.state_dict(), "head": DoorHead(128).state_dict(), "encoder_kind": "rgbd"}, p)
    got = load_encoder(str(p), 128, "cpu")
    assert isinstance(got, DoorRGBDEncoder) and got.uses_depth
    assert not any(q.requires_grad for q in got.parameters())
    old = DoorVisionEncoder(128, pretrained=False)
    p2 = tmp_path / "rgb.pt"
    torch.save({"encoder": old.state_dict(), "head": DoorHead(128).state_dict()}, p2)  # 11-05 までの重み（encoder_kind なし）
    got2 = load_encoder(str(p2), 128, "cpu")
    assert isinstance(got2, DoorVisionEncoder) and not getattr(got2, "uses_depth", False)


def test_pretrain_runs_on_tiny_rgbd_data(tmp_path):
    """網の先の学習が、深度つきの小さな記録で 1 エポック回り、encoder_kind=rgbd で保存する（CPU・ImageNet の重みは使わない）"""
    frames = tmp_path / "frames"
    frames.mkdir()
    rng = np.random.default_rng(0)
    n = 96
    np.savez(
        frames / "shard_00000.npz",
        rgb=rng.integers(0, 256, (n, 108, 192, 3), dtype=np.uint8),
        depth=(0.3 + rng.random((n, 54, 96)) * 2).astype(np.float16),
        aux=rng.normal(size=(n, AUX_IN_DIM)).astype(np.float32),
        truth=rng.normal(size=(n, DOOR_TRUTH_DIM)).astype(np.float32),
        stage=rng.integers(0, 6, n).astype(np.float32),
        pitch_deg=np.full(n, 50.0, dtype=np.float32),
        ep_step=np.zeros(n, dtype=np.float32),
        env_id=(np.arange(n) % 16).astype(np.float32),
    )
    out = tmp_path / "enc.pt"
    # --init に乱数の重みを渡して ImageNet の重みを取りに行かない
    init = tmp_path / "init.pt"
    torch.save({"encoder": DoorRGBDEncoder(128, pretrained=False).state_dict(), "head": DoorHead(128).state_dict(),
                "encoder_kind": "rgbd"}, init)
    r = subprocess.run(
        [sys.executable, "gr00t/rl/scripts/door_vision_pretrain.py", "--data", str(frames), "--out", str(out),
         "--epochs", "1", "--batch", "16", "--device", "cpu", "--init", str(init), "--eval_batch", "32"],
        capture_output=True, text=True, timeout=600,
    )
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    assert "深度だけ" in r.stdout
    ck = torch.load(out, map_location="cpu", weights_only=False)
    assert ck["encoder_kind"] == "rgbd"
